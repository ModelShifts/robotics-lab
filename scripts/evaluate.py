"""Compare the tools with the synthetic generator settings.

    python scripts/evaluate.py
    python scripts/evaluate.py --out eval.json

Exits 0 when every check passes. The thresholds are the generator settings
(or a bound around them), not a claim about any other data.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robotics_lab.canbus.analyze import analyze_trace  # noqa: E402
from robotics_lab.papers.dmp import run_dmp  # noqa: E402
from robotics_lab.papers.ilqr import run_double_integrator, run_pendulum  # noqa: E402
from robotics_lab.ros.analyze import analyze_bag  # noqa: E402
from robotics_lab.schedule import TOPIC_COMMAND, TOPIC_FEEDBACK, TOPIC_STATE  # noqa: E402


def check(name: str, ok: bool, expected: str, measured: str) -> dict:
    return {"check": name, "pass": bool(ok), "expected": expected, "measured": measured}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out")
    args = parser.parse_args()

    ros_truth = json.loads((ROOT / "samples" / "ros" / "node_graph.truth.json").read_text())
    can_truth = json.loads((ROOT / "samples" / "can" / "trace.truth.json").read_text())
    bag = analyze_bag(ROOT / "samples" / "ros" / "node_graph")
    can = analyze_trace(ROOT / "samples" / "can" / "trace.csv", ROOT / "samples" / "can" / "robot.dbc")
    dmp = run_dmp()
    pendulum = run_pendulum()
    linear, lqr, gap = run_double_integrator()

    rows: list[dict] = []
    rate = ros_truth["rate_hz"]
    for topic in bag.topics:
        rows.append(check(
            f"ROS {topic.topic} rate",
            abs(topic.rate_hz - rate) < 0.02,
            f"{rate} Hz ± 0.02",
            f"{topic.rate_hz:.6f} Hz",
        ))
        rows.append(check(
            f"ROS {topic.topic} jitter band",
            0.15 < topic.jitter_ms < 0.45,
            "0.15 ms < std < 0.45 ms",
            f"{topic.jitter_ms:.4f} ms",
        ))
    by_topic = {topic.topic: topic for topic in bag.topics}
    rows.append(check(
        "ROS sensor stamp latency",
        abs(by_topic[TOPIC_STATE].stamp_latency_ms_mean) < 1e-6,
        "0 ms (stamp is the sample time)",
        f"{by_topic[TOPIC_STATE].stamp_latency_ms_mean:.3g} ms",
    ))
    rows.append(check(
        "ROS command stamp latency",
        abs(by_topic[TOPIC_COMMAND].stamp_latency_ms_mean - ros_truth["controller_delay_ms"]) < 1e-6,
        f"{ros_truth['controller_delay_ms']} ms",
        f"{by_topic[TOPIC_COMMAND].stamp_latency_ms_mean:.6f} ms",
    ))
    rows.append(check(
        "ROS feedback stamp latency",
        abs(by_topic[TOPIC_FEEDBACK].stamp_latency_ms_mean - ros_truth["end_to_end_ms"]) < 1e-6,
        f"{ros_truth['end_to_end_ms']} ms",
        f"{by_topic[TOPIC_FEEDBACK].stamp_latency_ms_mean:.6f} ms",
    ))
    expected_edges = {
        (TOPIC_STATE, TOPIC_COMMAND): ros_truth["controller_delay_ms"],
        (TOPIC_COMMAND, TOPIC_FEEDBACK): ros_truth["plant_delay_ms"],
        (TOPIC_STATE, TOPIC_FEEDBACK): ros_truth["end_to_end_ms"],
    }
    for edge in bag.edges:
        want = expected_edges[(edge.source, edge.target)]
        rows.append(check(
            f"ROS edge {edge.source} → {edge.target}",
            abs(edge.latency_ms_mean - want) < 1e-6 and edge.count == ros_truth["samples"],
            f"{want} ms, n={ros_truth['samples']}",
            f"{edge.latency_ms_mean:.6f} ms, n={edge.count}",
        ))

    rows.append(check(
        "CAN frame count",
        can.frame_count == can_truth["frames"] and can.unknown_ids == 0,
        str(can_truth["frames"]),
        f"{can.frame_count}, unknown {can.unknown_ids}",
    ))
    # 600 frames * 135 bits / (500000 bit/s * ~1.995 s) is about 0.081.
    rows.append(check(
        "CAN bus load",
        0.07 < can.bus_load < 0.09,
        "about 0.081 (135 bits × 300 frames/s / 500000 bit/s)",
        f"{can.bus_load:.6f}",
    ))
    for edge in can.edges:
        want = {
            ("JointState", "JointCommand"): can_truth["controller_delay_ms"],
            ("JointCommand", "JointFeedback"): can_truth["plant_delay_ms"],
            ("JointState", "JointFeedback"): can_truth["end_to_end_ms"],
        }[(edge.source, edge.target)]
        rows.append(check(
            f"CAN edge {edge.source} → {edge.target}",
            abs(edge.latency_ms_mean - want) < 1e-6,
            f"{want} ms",
            f"{edge.latency_ms_mean:.6f} ms",
        ))

    rows.append(check("DMP reproduction RMSE", dmp.rmse < 0.02, "< 0.02", f"{dmp.rmse:.5f}"))
    rows.append(check("DMP goal error", dmp.goal_error < 1e-3, "< 0.001", f"{dmp.goal_error:.6g}"))
    rows.append(check("DMP shape RMSE at new goal", dmp.shape_rmse < 0.02, "< 0.02", f"{dmp.shape_rmse:.5f}"))
    rows.append(check(
        "DMP time-scale ratio",
        1.95 < dmp.time_scale_ratio < 2.05,
        "2 ± 0.05",
        f"{dmp.time_scale_ratio:.4f}",
    ))
    monotone = all(b <= a + 1e-8 for a, b in zip(pendulum.costs, pendulum.costs[1:]))
    rows.append(check(
        "iLQR cost decreases",
        monotone and pendulum.costs[-1] < pendulum.costs[0] / 5,
        "monotone and final < initial/5",
        f"{pendulum.costs[0]:.4g} → {pendulum.costs[-1]:.4g} in {len(pendulum.costs) - 1} steps",
    ))
    rows.append(check("iLQR pendulum goal error", pendulum.goal_error < 1e-4, "< 1e-4", f"{pendulum.goal_error:.3g}"))
    rows.append(check(
        "iLQR matches Riccati on the double integrator",
        gap < 1e-6 and abs(float(linear.xs[-1, 0])) < 0.02,
        "relative cost gap < 1e-6, |position| < 0.02",
        f"gap {gap:.3g}, position {float(linear.xs[-1, 0]):.5f}, LQR cost {lqr.cost:.6f}",
    ))

    failed = [row for row in rows if not row["pass"]]
    print(f"\n| Check | Result | Expected | Measured |")
    print("|---|---|---|---|")
    for row in rows:
        mark = "pass" if row["pass"] else "FAIL"
        print(f"| {row['check']} | {mark} | {row['expected']} | {row['measured']} |")
    print(f"\n{len(rows) - len(failed)}/{len(rows)} passed")
    if args.out:
        Path(args.out).write_text(json.dumps({"passed": len(rows) - len(failed), "failed": len(failed), "checks": rows}, indent=2) + "\n")
        print(f"wrote {args.out}", file=sys.stderr)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
