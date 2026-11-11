"""Topic rate, jitter and end-to-end latency for a rosbag2 recording.

Log times and channel counts are read with the ``mcap`` library (MIT).
Message fields are decoded with ``rosbags`` (Apache-2.0). The two readers
must agree on the per-topic message count.

End-to-end latency along an edge is the difference of log times of the two
messages that carry the same ``header.stamp``. Stamp latency on one topic is
``log_time - header.stamp``, the usual ROS convention when the stamp is the
source time.
"""

from __future__ import annotations

import json
import math
import statistics
from dataclasses import dataclass, field
from pathlib import Path

from .. import RoboticsLabError


@dataclass
class TopicReport:
    topic: str
    count: int
    rate_hz: float
    period_ms_mean: float
    jitter_ms: float
    stamp_latency_ms_mean: float | None
    stamp_latency_ms_std: float | None


@dataclass
class EdgeReport:
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
class BagReport:
    path: str
    topics: list[TopicReport]
    edges: list[EdgeReport]
    mcap_message_count: int

    def summary(self) -> dict:
        return {
            "path": self.path,
            "mcap_message_count": self.mcap_message_count,
            "topics": [
                {
                    "topic": t.topic,
                    "count": t.count,
                    "rate_hz": t.rate_hz,
                    "period_ms_mean": t.period_ms_mean,
                    "jitter_ms": t.jitter_ms,
                    "stamp_latency_ms_mean": t.stamp_latency_ms_mean,
                    "stamp_latency_ms_std": t.stamp_latency_ms_std,
                }
                for t in self.topics
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
    if not values:
        return math.nan
    ordered = sorted(values)
    pos = (len(ordered) - 1) * q
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - pos) + ordered[hi] * (pos - lo)


def _mcap_counts(bag: Path) -> tuple[dict[str, int], int]:
    from mcap.reader import make_reader

    files = sorted(bag.glob("*.mcap"))
    if len(files) != 1:
        raise RoboticsLabError(f"{bag} should contain exactly one .mcap file (found {len(files)})")
    with files[0].open("rb") as handle:
        reader = make_reader(handle)
        summary = reader.get_summary()
        if summary is None or summary.statistics is None:
            counts: dict[str, int] = {}
            total = 0
            for _schema, channel, _message in reader.iter_messages():
                counts[channel.topic] = counts.get(channel.topic, 0) + 1
                total += 1
            return counts, total
        id_to_topic = {cid: channel.topic for cid, channel in summary.channels.items()}
        counts = {id_to_topic[cid]: n for cid, n in summary.statistics.channel_message_counts.items()}
        return counts, summary.statistics.message_count


def _load_graph(bag: Path, graph: Path | None) -> dict | None:
    if graph is not None:
        path = graph
    else:
        path = bag.with_suffix(".graph.json")
    if not path.is_file():
        return None
    return json.loads(path.read_text())


def _stamp_ns(msg) -> int | None:
    header = getattr(msg, "header", None)
    if header is None or not hasattr(header, "stamp"):
        return None
    stamp = header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def analyze_bag(path: Path | str, graph: Path | str | None = None) -> BagReport:
    """Measure rates, jitter and edge latency in a rosbag2 directory."""
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore

    bag = Path(path)
    if not bag.is_dir():
        raise RoboticsLabError(f"rosbag2 directory not found: {bag}")
    if not (bag / "metadata.yaml").is_file():
        raise RoboticsLabError(f"{bag} has no metadata.yaml; expected a rosbag2 directory")

    mcap_counts, mcap_total = _mcap_counts(bag)
    store = get_typestore(Stores.ROS2_JAZZY)

    # topic -> list of (log_ns, stamp_ns or None)
    series: dict[str, list[tuple[int, int | None]]] = {}
    with Reader(bag) as reader:
        for connection, log_ns, raw in reader.messages():
            msg = store.deserialize_cdr(raw, connection.msgtype)
            series.setdefault(connection.topic, []).append((int(log_ns), _stamp_ns(msg)))

    ros_counts = {topic: len(rows) for topic, rows in series.items()}
    if ros_counts != mcap_counts:
        raise RoboticsLabError(
            f"mcap channel counts {mcap_counts} do not match decoded rosbag2 counts {ros_counts}"
        )

    topics: list[TopicReport] = []
    for topic in sorted(series):
        rows = series[topic]
        logs = [log for log, _stamp in rows]
        periods_ms = [(b - a) / 1e6 for a, b in zip(logs, logs[1:])]
        stamps = [log - stamp for log, stamp in rows if stamp is not None]
        stamp_ms = [v / 1e6 for v in stamps]
        topics.append(
            TopicReport(
                topic=topic,
                count=len(rows),
                rate_hz=(len(logs) - 1) / ((logs[-1] - logs[0]) / 1e9) if len(logs) > 1 else 0.0,
                period_ms_mean=statistics.fmean(periods_ms) if periods_ms else 0.0,
                jitter_ms=statistics.stdev(periods_ms) if len(periods_ms) > 1 else 0.0,
                stamp_latency_ms_mean=statistics.fmean(stamp_ms) if stamp_ms else None,
                stamp_latency_ms_std=statistics.stdev(stamp_ms) if len(stamp_ms) > 1 else None,
            )
        )

    edges: list[EdgeReport] = []
    document = _load_graph(bag, Path(graph) if graph else None)
    if document is not None:
        by_stamp: dict[str, dict[int, int]] = {}
        for topic, rows in series.items():
            paired: dict[int, int] = {}
            for log_ns, stamp in rows:
                if stamp is not None:
                    paired[stamp] = log_ns
            by_stamp[topic] = paired
        t0 = min(log for rows in series.values() for log, _stamp in rows)
        for edge in document.get("edges", []):
            source, target = edge["source"], edge["target"]
            if source not in by_stamp or target not in by_stamp:
                raise RoboticsLabError(f"graph edge {source} -> {target} is not in the bag")
            latencies: list[tuple[int, float]] = []
            for stamp, log_s in by_stamp[source].items():
                log_t = by_stamp[target].get(stamp)
                if log_t is None:
                    continue
                latencies.append((log_s, (log_t - log_s) / 1e6))
            latencies.sort()
            values = [v for _t, v in latencies]
            if not values:
                raise RoboticsLabError(f"no messages matched on {source} -> {target}")
            edges.append(
                EdgeReport(
                    source=source,
                    target=target,
                    count=len(values),
                    latency_ms_mean=statistics.fmean(values),
                    latency_ms_std=statistics.stdev(values) if len(values) > 1 else 0.0,
                    latency_ms_p50=_percentile(values, 0.50),
                    latency_ms_p95=_percentile(values, 0.95),
                    times_s=[(t - t0) / 1e9 for t, _v in latencies],
                    latency_ms=values,
                )
            )

    return BagReport(path=str(bag), topics=topics, edges=edges, mcap_message_count=mcap_total)
