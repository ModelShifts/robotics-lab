"""Write a rosbag2 mcap recording of the synthetic joint loop.

Messages are ``sensor_msgs/msg/JointState``, CDR-serialized by ``rosbags``
(Apache-2.0) into an mcap rosbag2. The header stamp is the sensor sample time.
Downstream messages copy that stamp, which is how the analyzer pairs them.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import numpy as np

from ..schedule import (
    TOPIC_COMMAND,
    TOPIC_FEEDBACK,
    TOPIC_STATE,
    command_log_ns,
    feedback_log_ns,
    graph_document,
    ideal_stamp_ns,
    joint_motion,
    sample_count,
    sensor_log_ns,
)


def _typestore():
    from rosbags.typesys import Stores, get_typestore

    return get_typestore(Stores.ROS2_JAZZY)


def _stamp(types, time_ns: int):
    time_ns = int(time_ns)
    return types["builtin_interfaces/msg/Time"](sec=time_ns // 1_000_000_000, nanosec=time_ns % 1_000_000_000)


def _joint_state(types, stamp_ns: int, position: float, velocity: float):
    header = types["std_msgs/msg/Header"](stamp=_stamp(types, stamp_ns), frame_id="arm")
    return types["sensor_msgs/msg/JointState"](
        header=header,
        name=["joint_1"],
        position=np.asarray([position], dtype=np.float64),
        velocity=np.asarray([velocity], dtype=np.float64),
        effort=np.asarray([0.0], dtype=np.float64),
    )


def simulate_bag(path: Path | str, duration_s: float | None = None) -> Path:
    """Create a rosbag2 directory at ``path`` and a sibling ``*.graph.json``.

    An existing directory at ``path`` is replaced. Returns ``path``.
    """
    from rosbags.rosbag2 import StoragePlugin, Writer

    path = Path(path)
    if path.exists():
        shutil.rmtree(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    n = sample_count(2.0 if duration_s is None else duration_s)
    store = _typestore()
    types = store.types
    msgtype = types["sensor_msgs/msg/JointState"].__msgtype__

    events: list[tuple[int, int, str, object]] = []
    for i in range(n):
        stamp = ideal_stamp_ns(i)
        pos, vel = joint_motion(i)
        events.append((sensor_log_ns(i), 0, TOPIC_STATE, _joint_state(types, stamp, pos, vel)))
        events.append((command_log_ns(i), 1, TOPIC_COMMAND, _joint_state(types, stamp, pos, vel)))
        events.append((feedback_log_ns(i), 2, TOPIC_FEEDBACK, _joint_state(types, stamp, pos, vel)))
    events.sort()

    with Writer(path, version=9, storage_plugin=StoragePlugin.MCAP) as writer:
        conns = {
            topic: writer.add_connection(topic, msgtype, typestore=store)
            for topic in (TOPIC_STATE, TOPIC_COMMAND, TOPIC_FEEDBACK)
        }
        for log_ns, _tie, topic, msg in events:
            writer.write(conns[topic], log_ns, store.serialize_cdr(msg, msgtype))

    graph_path = path.with_suffix(".graph.json")
    graph_path.write_text(json.dumps(graph_document(), indent=2) + "\n")
    return path
