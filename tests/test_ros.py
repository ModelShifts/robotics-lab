import json
from pathlib import Path

from robotics_lab.ros.analyze import analyze_bag
from robotics_lab.ros.simulate import simulate_bag
from robotics_lab.schedule import TOPIC_COMMAND, TOPIC_FEEDBACK, TOPIC_STATE

ROOT = Path(__file__).resolve().parents[1]
SAMPLE = ROOT / "samples" / "ros" / "node_graph"


def test_sample_matches_schedule():
    truth = json.loads((ROOT / "samples" / "ros" / "node_graph.truth.json").read_text())
    report = analyze_bag(SAMPLE)
    assert report.mcap_message_count == truth["samples"] * 3
    by = {topic.topic: topic for topic in report.topics}
    for topic in (TOPIC_STATE, TOPIC_COMMAND, TOPIC_FEEDBACK):
        assert abs(by[topic].rate_hz - truth["rate_hz"]) < 0.02
        assert 0.15 < by[topic].jitter_ms < 0.45
    assert abs(by[TOPIC_STATE].stamp_latency_ms_mean) < 1e-6
    assert abs(by[TOPIC_COMMAND].stamp_latency_ms_mean - 2.0) < 1e-6
    assert abs(by[TOPIC_FEEDBACK].stamp_latency_ms_mean - 5.0) < 1e-6
    edges = {(e.source, e.target): e for e in report.edges}
    assert abs(edges[(TOPIC_STATE, TOPIC_COMMAND)].latency_ms_mean - 2.0) < 1e-6
    assert abs(edges[(TOPIC_COMMAND, TOPIC_FEEDBACK)].latency_ms_mean - 3.0) < 1e-6
    assert abs(edges[(TOPIC_STATE, TOPIC_FEEDBACK)].latency_ms_mean - 5.0) < 1e-6
    assert edges[(TOPIC_STATE, TOPIC_FEEDBACK)].count == 200


def test_simulate_is_deterministic(tmp_path):
    first = tmp_path / "a" / "node_graph"
    second = tmp_path / "b" / "node_graph"
    simulate_bag(first)
    simulate_bag(second)
    assert (first / "node_graph.mcap").read_bytes() == (second / "node_graph.mcap").read_bytes()
    assert (first / "metadata.yaml").read_text() == (second / "metadata.yaml").read_text()
    again = analyze_bag(first)
    e2e = [e for e in again.edges if e.source == TOPIC_STATE and e.target == TOPIC_FEEDBACK][0]
    assert abs(e2e.latency_ms_mean - 5.0) < 1e-6
