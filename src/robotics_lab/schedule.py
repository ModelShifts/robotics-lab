"""Synthetic joint loop shared by the ROS 2 bag and the CAN trace.

Three stages, one sample period:

- ``joint_sensor`` publishes a joint sample.
- ``controller`` publishes a command one processing delay later.
- ``plant`` publishes feedback one actuation delay after the command.

Offsets are a fixed cycle with mean zero, so measured mean latencies match the
delays exactly over a whole number of cycles. Nothing here is random, so the
committed samples do not depend on a library RNG.
"""

from __future__ import annotations

import math

PERIOD_NS = 10_000_000
CONTROLLER_DELAY_NS = 2_000_000
PLANT_DELAY_NS = 3_000_000
DURATION_S = 2.0
T0_NS = 1_000_000_000

# Log-time offsets added to the ideal sample time. Each tuple sums to zero.
SENSOR_OFFSET_NS = (0, 200_000, -100_000, 150_000, -250_000, 50_000, 100_000, -150_000)
CONTROLLER_OFFSET_NS = (0, 40_000, -20_000, 60_000, -40_000, 20_000, -60_000, 0)
PLANT_OFFSET_NS = (0, -30_000, 50_000, -10_000, 40_000, -50_000, 20_000, -20_000)

TOPIC_STATE = "/joint_states"
TOPIC_COMMAND = "/joint_commands"
TOPIC_FEEDBACK = "/joint_feedback"

ID_STATE = 0x100
ID_COMMAND = 0x200
ID_FEEDBACK = 0x300

BITRATE = 500_000

MOTION_HZ = 0.5
MOTION_AMPLITUDE_RAD = 0.4


def sample_count(duration_s: float = DURATION_S) -> int:
    """Number of sensor samples in ``duration_s`` at ``PERIOD_NS``."""
    period_s = PERIOD_NS / 1e9
    n = int(round(duration_s / period_s))
    if n < 2:
        raise ValueError("duration must cover at least two samples")
    return n


def ideal_stamp_ns(index: int, t0_ns: int = T0_NS) -> int:
    return t0_ns + index * PERIOD_NS


def sensor_log_ns(index: int, t0_ns: int = T0_NS) -> int:
    return ideal_stamp_ns(index, t0_ns) + SENSOR_OFFSET_NS[index % len(SENSOR_OFFSET_NS)]


def command_log_ns(index: int, t0_ns: int = T0_NS) -> int:
    off = CONTROLLER_OFFSET_NS[index % len(CONTROLLER_OFFSET_NS)]
    return sensor_log_ns(index, t0_ns) + CONTROLLER_DELAY_NS + off


def feedback_log_ns(index: int, t0_ns: int = T0_NS) -> int:
    off = PLANT_OFFSET_NS[index % len(PLANT_OFFSET_NS)]
    return command_log_ns(index, t0_ns) + PLANT_DELAY_NS + off


def joint_motion(index: int) -> tuple[float, float]:
    """Joint position (rad) and velocity (rad/s) for sample ``index``."""
    t = index * PERIOD_NS / 1e9
    w = 2.0 * math.pi * MOTION_HZ
    pos = MOTION_AMPLITUDE_RAD * math.sin(w * t)
    vel = MOTION_AMPLITUDE_RAD * w * math.cos(w * t)
    return pos, vel


def graph_document() -> dict:
    """Node graph written next to the bag and read by the analyzer."""
    return {
        "name": "joint loop",
        "description": (
            "Synthetic graph. joint_sensor publishes /joint_states. "
            "controller republishes the sample on /joint_commands after a processing delay. "
            "plant republishes it on /joint_feedback after an actuation delay. "
            "Every message keeps the sensor sample's header stamp."
        ),
        "period_ns": PERIOD_NS,
        "nodes": [
            {"name": "joint_sensor", "publishes": [TOPIC_STATE]},
            {
                "name": "controller",
                "subscribes": [TOPIC_STATE],
                "publishes": [TOPIC_COMMAND],
                "delay_ns": CONTROLLER_DELAY_NS,
            },
            {
                "name": "plant",
                "subscribes": [TOPIC_COMMAND],
                "publishes": [TOPIC_FEEDBACK],
                "delay_ns": PLANT_DELAY_NS,
            },
        ],
        "edges": [
            {"source": TOPIC_STATE, "target": TOPIC_COMMAND},
            {"source": TOPIC_COMMAND, "target": TOPIC_FEEDBACK},
            {"source": TOPIC_STATE, "target": TOPIC_FEEDBACK},
        ],
    }
