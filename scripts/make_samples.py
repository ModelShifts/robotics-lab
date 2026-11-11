"""Regenerate the committed synthetic samples.

    python scripts/make_samples.py

The rosbag2 bytes are deterministic for a fixed rosbags version (pinned in
pyproject.toml). The CAN trace is integer nanoseconds, not a live capture.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robotics_lab.canbus.dbc import write_dbc  # noqa: E402
from robotics_lab.canbus.trace import generate_frames, write_trace  # noqa: E402
from robotics_lab.ros.simulate import simulate_bag  # noqa: E402
from robotics_lab.schedule import (  # noqa: E402
    BITRATE,
    CONTROLLER_DELAY_NS,
    DURATION_S,
    PERIOD_NS,
    PLANT_DELAY_NS,
    sample_count,
)


def main() -> int:
    bag = ROOT / "samples" / "ros" / "node_graph"
    simulate_bag(bag, duration_s=DURATION_S)
    truth = {
        "description": (
            "Generator settings for the synthetic joint loop. "
            "Offset cycles have mean zero, so mean latencies equal these delays "
            "over a whole number of cycles (200 samples is 25 cycles of 8)."
        ),
        "samples": sample_count(DURATION_S),
        "period_ns": PERIOD_NS,
        "rate_hz": 1e9 / PERIOD_NS,
        "controller_delay_ms": CONTROLLER_DELAY_NS / 1e6,
        "plant_delay_ms": PLANT_DELAY_NS / 1e6,
        "end_to_end_ms": (CONTROLLER_DELAY_NS + PLANT_DELAY_NS) / 1e6,
    }
    (ROOT / "samples" / "ros" / "node_graph.truth.json").write_text(json.dumps(truth, indent=2) + "\n")

    dbc = write_dbc(ROOT / "samples" / "can" / "robot.dbc")
    trace = write_trace(ROOT / "samples" / "can" / "trace.csv", generate_frames(DURATION_S), bitrate=BITRATE)
    can_truth = {
        "description": "Same joint-loop schedule as the rosbag2 sample, encoded with samples/can/robot.dbc.",
        "bitrate": BITRATE,
        "samples": sample_count(DURATION_S),
        "frames": sample_count(DURATION_S) * 3,
        "frame_bits_worst_case_8byte": 135,
        "controller_delay_ms": CONTROLLER_DELAY_NS / 1e6,
        "plant_delay_ms": PLANT_DELAY_NS / 1e6,
        "end_to_end_ms": (CONTROLLER_DELAY_NS + PLANT_DELAY_NS) / 1e6,
    }
    (ROOT / "samples" / "can" / "trace.truth.json").write_text(json.dumps(can_truth, indent=2) + "\n")
    print(f"wrote {bag}")
    print(f"wrote {dbc}")
    print(f"wrote {trace}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
