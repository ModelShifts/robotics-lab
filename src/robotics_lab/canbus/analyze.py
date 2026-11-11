"""Decode a CAN trace and report rate, jitter, latency and bus load."""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from pathlib import Path

from .. import RoboticsLabError
from ..schedule import BITRATE, ID_COMMAND, ID_FEEDBACK, ID_STATE
from .bits import bus_load, frame_bits
from .dbc import load_dbc
from .trace import Frame, read_trace

# Edges paired by the Sequence signal: source id, target id, label.
EDGES = (
    (ID_STATE, ID_COMMAND, "JointState", "JointCommand"),
    (ID_COMMAND, ID_FEEDBACK, "JointCommand", "JointFeedback"),
    (ID_STATE, ID_FEEDBACK, "JointState", "JointFeedback"),
)


@dataclass
class MessageReport:
    name: str
    arbitration_id: int
    count: int
    rate_hz: float
    jitter_ms: float


@dataclass
class CanEdge:
    source: str
    target: str
    count: int
    latency_ms_mean: float
    latency_ms_std: float
    latency_ms_p50: float
    latency_ms_p95: float
    times_s: list[float] = field(default_factory=list)
    latency_ms: list[float] = field(default_factory=list)


@dataclass
class DecodedRow:
    time_ns: int
    arbitration_id: int
    message: str
    signal: str
    value: float
    unit: str


@dataclass
class CanReport:
    bitrate: int
    frame_count: int
    duration_s: float
    bus_load: float
    messages: list[MessageReport]
    edges: list[CanEdge]
    decoded: list[DecodedRow]
    unknown_ids: int

    def summary(self) -> dict:
        return {
            "bitrate": self.bitrate,
            "frame_count": self.frame_count,
            "duration_s": self.duration_s,
            "bus_load": self.bus_load,
            "unknown_ids": self.unknown_ids,
            "messages": [
                {
                    "name": m.name,
                    "arbitration_id": m.arbitration_id,
                    "count": m.count,
                    "rate_hz": m.rate_hz,
                    "jitter_ms": m.jitter_ms,
                }
                for m in self.messages
            ],
            "edges": [
                {
                    "source": e.source,
                    "target": e.target,
                    "count": e.count,
                    "latency_ms_mean": e.latency_ms_mean,
                    "latency_ms_std": e.latency_ms_std,
                    "latency_ms_p50": e.latency_ms_p50,
                    "latency_ms_p95": e.latency_ms_p95,
                }
                for e in self.edges
            ],
        }


def _percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    pos = (len(ordered) - 1) * q
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - pos) + ordered[hi] * (pos - lo)


def _as_float(value) -> float:
    if hasattr(value, "value"):  # cantools NamedSignalValue
        value = value.value
    return float(value)


def decode_frames(frames: list[Frame], dbc_path: Path | str | None = None) -> tuple[list[DecodedRow], int]:
    db = load_dbc(dbc_path)
    by_id = {message.frame_id: message for message in db.messages}
    rows: list[DecodedRow] = []
    unknown = 0
    for frame in frames:
        message = by_id.get(frame.arbitration_id)
        if message is None:
            unknown += 1
            continue
        decoded = message.decode(frame.data)
        for signal in message.signals:
            rows.append(
                DecodedRow(
                    time_ns=frame.time_ns,
                    arbitration_id=frame.arbitration_id,
                    message=message.name,
                    signal=signal.name,
                    value=_as_float(decoded[signal.name]),
                    unit=signal.unit or "",
                )
            )
    return rows, unknown


def analyze_frames(
    frames: list[Frame],
    bitrate: int = BITRATE,
    dbc_path: Path | str | None = None,
) -> CanReport:
    if not frames:
        raise RoboticsLabError("CAN trace is empty")
    decoded, unknown = decode_frames(frames, dbc_path)
    bits = [frame_bits(frame.dlc, extended=False) for frame in frames]
    times = [frame.time_ns for frame in frames]
    load, duration_s = bus_load(bits, times, bitrate)

    by_name: dict[str, list[int]] = {}
    for row in decoded:
        if row.signal == "Sequence":
            by_name.setdefault(row.message, []).append(row.time_ns)
    messages: list[MessageReport] = []
    id_of = {ID_STATE: "JointState", ID_COMMAND: "JointCommand", ID_FEEDBACK: "JointFeedback"}
    name_to_id = {name: arb for arb, name in id_of.items()}
    for name in ("JointState", "JointCommand", "JointFeedback"):
        stamps = by_name.get(name, [])
        periods_ms = [(b - a) / 1e6 for a, b in zip(stamps, stamps[1:])]
        messages.append(
            MessageReport(
                name=name,
                arbitration_id=name_to_id[name],
                count=len(stamps),
                rate_hz=(len(stamps) - 1) / ((stamps[-1] - stamps[0]) / 1e9) if len(stamps) > 1 else 0.0,
                jitter_ms=statistics.stdev(periods_ms) if len(periods_ms) > 1 else 0.0,
            )
        )

    # sequence -> time_ns, per message name
    seq_time: dict[str, dict[int, int]] = {}
    for row in decoded:
        if row.signal != "Sequence":
            continue
        seq_time.setdefault(row.message, {})[int(round(row.value))] = row.time_ns
    t0 = frames[0].time_ns
    edges: list[CanEdge] = []
    for _src_id, _dst_id, source, target in EDGES:
        left = seq_time.get(source, {})
        right = seq_time.get(target, {})
        paired = [(left[seq], (right[seq] - left[seq]) / 1e6) for seq in sorted(left) if seq in right]
        values = [v for _t, v in paired]
        if not values:
            raise RoboticsLabError(f"no Sequence matches for {source} -> {target}")
        edges.append(
            CanEdge(
                source=source,
                target=target,
                count=len(values),
                latency_ms_mean=statistics.fmean(values),
                latency_ms_std=statistics.stdev(values) if len(values) > 1 else 0.0,
                latency_ms_p50=_percentile(values, 0.50),
                latency_ms_p95=_percentile(values, 0.95),
                times_s=[(t - t0) / 1e9 for t, _v in paired],
                latency_ms=values,
            )
        )

    return CanReport(
        bitrate=bitrate,
        frame_count=len(frames),
        duration_s=duration_s,
        bus_load=load,
        messages=messages,
        edges=edges,
        decoded=decoded,
        unknown_ids=unknown,
    )


def analyze_trace(path: Path | str, dbc_path: Path | str | None = None, bitrate: int | None = None) -> CanReport:
    frames, commented = read_trace(path)
    rate = bitrate if bitrate is not None else (commented if commented is not None else BITRATE)
    return analyze_frames(frames, bitrate=rate, dbc_path=dbc_path)
