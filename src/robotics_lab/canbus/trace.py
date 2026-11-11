"""Deterministic CAN trace of the synthetic joint loop, encoded with cantools."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

from ..schedule import (
    BITRATE,
    ID_COMMAND,
    ID_FEEDBACK,
    ID_STATE,
    command_log_ns,
    feedback_log_ns,
    joint_motion,
    sample_count,
    sensor_log_ns,
)
from .dbc import load_dbc

TRACE_HEADER = ("time_ns", "arbitration_id", "data_hex")


@dataclass(frozen=True)
class Frame:
    time_ns: int
    arbitration_id: int
    data: bytes

    @property
    def dlc(self) -> int:
        return len(self.data)


def generate_frames(duration_s: float | None = None, dbc_path: Path | str | None = None) -> list[Frame]:
    """Encode one JointState, JointCommand and JointFeedback per sample."""
    db = load_dbc(dbc_path)
    state = db.get_message_by_name("JointState")
    command = db.get_message_by_name("JointCommand")
    feedback = db.get_message_by_name("JointFeedback")
    frames: list[Frame] = []
    for i in range(sample_count(2.0 if duration_s is None else duration_s)):
        pos, vel = joint_motion(i)
        seq = i % 65536
        frames.append(
            Frame(
                sensor_log_ns(i),
                ID_STATE,
                bytes(state.encode({"Position": pos, "Velocity": vel, "Sequence": seq})),
            )
        )
        frames.append(
            Frame(
                command_log_ns(i),
                ID_COMMAND,
                bytes(command.encode({"Position": pos, "Sequence": seq})),
            )
        )
        frames.append(
            Frame(
                feedback_log_ns(i),
                ID_FEEDBACK,
                bytes(feedback.encode({"Position": pos, "Sequence": seq})),
            )
        )
    frames.sort(key=lambda frame: (frame.time_ns, frame.arbitration_id))
    return frames


def write_trace(path: Path | str, frames: list[Frame], bitrate: int = BITRATE) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        handle.write(f"# bitrate={bitrate}\n")
        writer = csv.writer(handle)
        writer.writerow(TRACE_HEADER)
        for frame in frames:
            writer.writerow([frame.time_ns, frame.arbitration_id, frame.data.hex()])
    return path


def read_trace(path: Path | str) -> tuple[list[Frame], int | None]:
    """Read a trace CSV. Returns frames and the bitrate comment, if present."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    bitrate: int | None = None
    frames: list[Frame] = []
    with path.open(newline="") as handle:
        lines = []
        for line in handle:
            if line.startswith("#"):
                text = line[1:].strip()
                if text.startswith("bitrate="):
                    bitrate = int(text.split("=", 1)[1])
                continue
            lines.append(line)
    reader = csv.DictReader(lines)
    if reader.fieldnames is None or tuple(reader.fieldnames) != TRACE_HEADER:
        raise ValueError(f"{path} must have columns {','.join(TRACE_HEADER)}")
    for row in reader:
        data = bytes.fromhex(row["data_hex"])
        frames.append(Frame(int(row["time_ns"]), int(row["arbitration_id"]), data))
    return frames, bitrate
