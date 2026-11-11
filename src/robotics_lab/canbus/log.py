"""Record CAN frames with python-can and export decoded CSV and Parquet.

On the virtual interface this also runs the synthetic joint-loop sender, because
a virtual channel otherwise has no traffic. On SocketCAN it only listens.
"""

from __future__ import annotations

import csv
import json
import threading
import time
from pathlib import Path

from .. import RoboticsLabError
from ..schedule import BITRATE
from .analyze import CanReport, DecodedRow, analyze_frames
from .trace import Frame, generate_frames, write_trace


def _write_decoded_csv(path: Path, rows: list[DecodedRow]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_ns", "arbitration_id", "message", "signal", "value", "unit"])
        for row in rows:
            writer.writerow([row.time_ns, row.arbitration_id, row.message, row.signal, row.value, row.unit])


def _write_decoded_parquet(path: Path, rows: list[DecodedRow]) -> None:
    import pyarrow as pa
    import pyarrow.parquet as pq

    table = pa.table(
        {
            "time_ns": [row.time_ns for row in rows],
            "arbitration_id": [row.arbitration_id for row in rows],
            "message": [row.message for row in rows],
            "signal": [row.signal for row in rows],
            "value": [row.value for row in rows],
            "unit": [row.unit for row in rows],
        }
    )
    pq.write_table(table, path)


def _send_virtual(channel: str, frames: list[Frame]) -> None:
    import can

    tx = can.Bus(interface="virtual", channel=channel)
    try:
        origin = time.perf_counter()
        t0 = frames[0].time_ns
        for frame in frames:
            wait = (frame.time_ns - t0) / 1e9 - (time.perf_counter() - origin)
            if wait > 0:
                time.sleep(wait)
            tx.send(
                can.Message(arbitration_id=frame.arbitration_id, data=frame.data, is_extended_id=False)
            )
    finally:
        tx.shutdown()


def record_bus(
    interface: str,
    channel: str,
    out: Path | str,
    seconds: float,
    dbc_path: Path | str | None = None,
    bitrate: int = BITRATE,
) -> CanReport:
    """Record ``seconds`` of traffic into ``out`` (trace, decoded CSV, Parquet, JSON)."""
    import can

    if seconds <= 0:
        raise RoboticsLabError("--seconds must be positive")
    if interface not in {"virtual", "socketcan"}:
        raise RoboticsLabError("interface must be 'virtual' or 'socketcan'")

    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    received: list = []
    stop = threading.Event()

    try:
        rx = can.Bus(interface=interface, channel=channel)
    except Exception as exc:
        raise RoboticsLabError(
            f"could not open {interface} channel {channel!r} ({exc}). "
            "The virtual interface needs no hardware; SocketCAN needs an existing channel."
        ) from exc

    def listen() -> None:
        while not stop.is_set():
            msg = rx.recv(timeout=0.05)
            if msg is not None and not msg.is_error_frame:
                received.append(msg)

    thread = threading.Thread(target=listen, name="can-listen", daemon=True)
    thread.start()
    time.sleep(0.02)
    try:
        if interface == "virtual":
            try:
                frames = generate_frames(seconds, dbc_path)
            except Exception as exc:
                raise RoboticsLabError(
                    "the virtual sender needs JointState, JointCommand and JointFeedback in the DBC "
                    f"({exc})"
                ) from exc
            _send_virtual(channel, frames)
            time.sleep(0.05)
        else:
            time.sleep(seconds)
    finally:
        stop.set()
        thread.join(timeout=2)
        rx.shutdown()

    frames = [
        Frame(time_ns=int(round(msg.timestamp * 1e9)), arbitration_id=int(msg.arbitration_id), data=bytes(msg.data))
        for msg in received
        if not msg.is_extended_id
    ]
    frames.sort(key=lambda frame: (frame.time_ns, frame.arbitration_id))
    if not frames:
        raise RoboticsLabError(f"no CAN frames received on {interface}:{channel}")

    write_trace(out / "trace.csv", frames, bitrate=bitrate)
    report = analyze_frames(frames, bitrate=bitrate, dbc_path=dbc_path)
    _write_decoded_csv(out / "decoded.csv", report.decoded)
    _write_decoded_parquet(out / "decoded.parquet", report.decoded)
    (out / "report.json").write_text(json.dumps(report.summary(), indent=2) + "\n")
    return report
