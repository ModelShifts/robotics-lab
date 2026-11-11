import json
from pathlib import Path

import pyarrow.parquet as pq

from robotics_lab import RoboticsLabError
from robotics_lab.canbus.analyze import analyze_trace
from robotics_lab.canbus.bits import frame_bits
from robotics_lab.canbus.log import record_bus
from robotics_lab.schedule import joint_motion

ROOT = Path(__file__).resolve().parents[1]


def test_worst_case_frame_length_matches_the_published_count():
    # 47 + 8*8 + floor((34 + 64 - 1) / 4) = 135. Extended: 80 + 10*8 = 160.
    assert frame_bits(8) == 47 + 64 + (34 + 64 - 1) // 4 == 135
    assert frame_bits(0) == 55
    assert frame_bits(8, extended=True) == 160


def test_sample_trace_latency_and_decode():
    truth = json.loads((ROOT / "samples" / "can" / "trace.truth.json").read_text())
    report = analyze_trace(ROOT / "samples" / "can" / "trace.csv", ROOT / "samples" / "can" / "robot.dbc")
    assert report.frame_count == truth["frames"]
    assert report.unknown_ids == 0
    assert report.bitrate == truth["bitrate"]
    assert 0.07 < report.bus_load < 0.09
    edges = {(e.source, e.target): e for e in report.edges}
    assert abs(edges[("JointState", "JointCommand")].latency_ms_mean - 2.0) < 1e-6
    assert abs(edges[("JointCommand", "JointFeedback")].latency_ms_mean - 3.0) < 1e-6
    assert abs(edges[("JointState", "JointFeedback")].latency_ms_mean - 5.0) < 1e-6
    positions = [row for row in report.decoded if row.message == "JointState" and row.signal == "Position"]
    assert len(positions) == truth["samples"]
    assert max(abs(row.value - joint_motion(i)[0]) for i, row in enumerate(positions)) < 0.001


def test_virtual_bus_log_writes_parquet(tmp_path):
    report = record_bus("virtual", "robotlab-pytest", tmp_path, seconds=0.3, dbc_path=ROOT / "samples" / "can" / "robot.dbc")
    assert report.frame_count >= 30
    assert (tmp_path / "trace.csv").is_file()
    assert (tmp_path / "decoded.csv").is_file()
    table = pq.read_table(tmp_path / "decoded.parquet")
    assert table.num_rows > 0
    assert "Position" in table.column("signal").to_pylist()


def test_socketcan_missing_channel_is_an_error(tmp_path):
    try:
        record_bus("socketcan", "vcan-does-not-exist", tmp_path, seconds=0.2)
    except RoboticsLabError as exc:
        assert "SocketCAN" in str(exc)
    else:
        raise AssertionError("expected a clear error when the SocketCAN channel cannot be opened")
