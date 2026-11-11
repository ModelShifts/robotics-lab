"""Command-line interface.

    robotics-lab ros analyze samples/ros/node_graph
    robotics-lab can log --interface virtual --channel robotlab --dbc samples/can/robot.dbc --seconds 1 --out out/can
    robotics-lab can analyze samples/can/trace.csv --dbc samples/can/robot.dbc
    robotics-lab papers dmp --out out/dmp
    robotics-lab papers ilqr --out out/ilqr
    robotics-lab serve
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from . import RoboticsLabError, __version__
from .canbus.analyze import CanReport, analyze_trace
from .canbus.log import record_bus
from .report import text_table, write_csv, write_html
from .ros.analyze import BagReport, analyze_bag
from .ros.simulate import simulate_bag
from .schedule import BITRATE


def _load_dotenv() -> None:
    try:
        from dotenv import load_dotenv
    except ImportError:
        return
    load_dotenv()


def _topic_rows(report: BagReport) -> tuple[list[str], list[list[object]]]:
    headers = ["topic", "count", "rate_hz", "period_ms", "jitter_ms", "stamp_latency_ms"]
    rows = [
        [t.topic, t.count, t.rate_hz, t.period_ms_mean, t.jitter_ms, t.stamp_latency_ms_mean if t.stamp_latency_ms_mean is not None else ""]
        for t in report.topics
    ]
    return headers, rows


def _edge_rows(report: BagReport) -> tuple[list[str], list[list[object]]]:
    headers = ["source", "target", "count", "latency_ms", "std_ms", "p50_ms", "p95_ms"]
    rows = [
        [e.source, e.target, e.count, e.latency_ms_mean, e.latency_ms_std, e.latency_ms_p50, e.latency_ms_p95]
        for e in report.edges
    ]
    return headers, rows


def _print_bag(report: BagReport, as_json: bool) -> None:
    if as_json:
        print(json.dumps(report.summary(), indent=2))
        return
    print(f"rosbag2 {report.path}  ·  {report.mcap_message_count} messages")
    headers, rows = _topic_rows(report)
    print("\nTopics")
    print(text_table(headers, rows))
    if report.edges:
        headers, rows = _edge_rows(report)
        print("\nEdges (matched on header.stamp)")
        print(text_table(headers, rows))
    else:
        print("\nNo graph file beside the bag, so edge latency was not computed.")


def _write_bag_files(report: BagReport, csv_path: str | None, html_path: str | None) -> None:
    topic_headers, topic_rows = _topic_rows(report)
    edge_headers, edge_rows = _edge_rows(report)
    if csv_path:
        edge_path = Path(csv_path).with_name(Path(csv_path).stem + "_edges.csv")
        write_csv(csv_path, topic_headers, topic_rows)
        write_csv(edge_path, edge_headers, edge_rows)
        print(f"wrote {csv_path} and {edge_path}", file=sys.stderr)
    if html_path:
        write_html(
            html_path,
            "ROS 2 bag report",
            [("Topics", topic_headers, topic_rows), ("Edges", edge_headers, edge_rows)],
        )
        print(f"wrote {html_path}", file=sys.stderr)


def cmd_ros_analyze(args: argparse.Namespace) -> int:
    report = analyze_bag(args.bag, args.graph)
    _print_bag(report, args.json)
    _write_bag_files(report, args.csv, args.html)
    return 0


def cmd_ros_simulate(args: argparse.Namespace) -> int:
    path = simulate_bag(args.out, duration_s=args.seconds)
    print(f"wrote {path} and {path.with_suffix('.graph.json')}")
    return 0


def _can_rows(report: CanReport) -> tuple[tuple[list[str], list[list[object]]], tuple[list[str], list[list[object]]]]:
    message_headers = ["message", "id", "count", "rate_hz", "jitter_ms"]
    message_rows = [[m.name, m.arbitration_id, m.count, m.rate_hz, m.jitter_ms] for m in report.messages]
    edge_headers = ["source", "target", "count", "latency_ms", "std_ms", "p50_ms", "p95_ms"]
    edge_rows = [
        [e.source, e.target, e.count, e.latency_ms_mean, e.latency_ms_std, e.latency_ms_p50, e.latency_ms_p95]
        for e in report.edges
    ]
    return (message_headers, message_rows), (edge_headers, edge_rows)


def _print_can(report: CanReport, as_json: bool) -> None:
    if as_json:
        print(json.dumps(report.summary(), indent=2))
        return
    print(
        f"{report.frame_count} frames  ·  {report.duration_s:.6g} s  ·  "
        f"bus load {100 * report.bus_load:.2f}% at {report.bitrate} bit/s"
    )
    print("Bus load uses a worst-case classical-CAN bit-stuffing estimate, including the 3-bit intermission.")
    (mh, mr), (eh, er) = _can_rows(report)
    print("\nMessages")
    print(text_table(mh, mr))
    print("\nEdges (matched on Sequence)")
    print(text_table(eh, er))
    if report.unknown_ids:
        print(f"\n{report.unknown_ids} frames were not in the DBC.")


def cmd_can_analyze(args: argparse.Namespace) -> int:
    report = analyze_trace(args.trace, args.dbc, bitrate=args.bitrate)
    _print_can(report, args.json)
    if args.csv:
        (mh, mr), (eh, er) = _can_rows(report)
        write_csv(args.csv, mh, mr)
        edge_path = Path(args.csv).with_name(Path(args.csv).stem + "_edges.csv")
        write_csv(edge_path, eh, er)
        print(f"wrote {args.csv} and {edge_path}", file=sys.stderr)
    if args.html:
        (mh, mr), (eh, er) = _can_rows(report)
        write_html(args.html, "CAN trace report", [("Messages", mh, mr), ("Edges", eh, er)])
        print(f"wrote {args.html}", file=sys.stderr)
    return 0


def cmd_can_log(args: argparse.Namespace) -> int:
    report = record_bus(args.interface, args.channel, args.out, args.seconds, args.dbc, bitrate=args.bitrate)
    print(f"wrote {args.out}/trace.csv, decoded.csv, decoded.parquet, report.json", file=sys.stderr)
    _print_can(report, args.json)
    return 0


def cmd_papers_list(_args: argparse.Namespace) -> int:
    from .papers.dmp import CITATION as dmp_citation
    from .papers.ilqr import CITATION as ilqr_citation
    from .papers.ilqr import LQR_CITATION, REGULARIZATION_CITATION

    print("dmp   discrete dynamic movement primitives")
    print(f"      {dmp_citation}")
    print("ilqr  iterative LQR on a pendulum, and finite-horizon LQR on a double integrator")
    print(f"      {ilqr_citation}")
    print(f"      {LQR_CITATION}")
    print(f"      {REGULARIZATION_CITATION}")
    return 0


def _need_figures():
    try:
        from . import figures
    except ImportError as exc:
        raise RoboticsLabError("plotting needs the demo extra: pip install -e '.[demo]'") from exc
    return figures


def cmd_papers_dmp(args: argparse.Namespace) -> int:
    from .papers.dmp import run_dmp

    result = run_dmp()
    summary = {
        "citation": result.citation,
        "rmse": result.rmse,
        "goal_error": result.goal_error,
        "scaled_goal": result.scaled_goal,
        "scaled_goal_error": result.scaled_goal_error,
        "shape_rmse": result.shape_rmse,
        "time_scale": result.time_scale,
        "time_scale_ratio": result.time_scale_ratio,
    }
    if args.json:
        print(json.dumps(summary, indent=2))
    else:
        print("Discrete DMP (Ijspeert et al. 2013), synthetic demonstration")
        print(text_table(
            ["rmse", "goal_error", "shape_rmse", "time_scale_ratio"],
            [[result.rmse, result.goal_error, result.shape_rmse, result.time_scale_ratio]],
        ))
    if args.out:
        figures = _need_figures()
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        figures.dmp_figure(result, out / "dmp_fit.png")
        figures.dmp_scaling_figure(result, out / "dmp_scaling.png")
        (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(f"wrote {out}/dmp_fit.png and {out}/dmp_scaling.png", file=sys.stderr)
    return 0


def cmd_papers_ilqr(args: argparse.Namespace) -> int:
    from .papers.ilqr import run_double_integrator, run_pendulum

    pendulum = run_pendulum()
    linear, lqr, gap = run_double_integrator()
    summary = {
        "citation": pendulum.citation,
        "pendulum_iterations": len(pendulum.costs) - 1,
        "pendulum_cost_initial": pendulum.costs[0],
        "pendulum_cost_final": pendulum.costs[-1],
        "pendulum_goal_error": pendulum.goal_error,
        "pendulum_costs": pendulum.costs,
        "double_integrator_ilqr_cost": linear.costs[-1],
        "double_integrator_lqr_cost": lqr.cost,
        "double_integrator_relative_gap": gap,
        "double_integrator_final_position": float(linear.xs[-1, 0]),
    }
    if args.json:
        print(json.dumps(summary, indent=2))
    else:
        print("iLQR pendulum (Li and Todorov 2004) and finite-horizon LQR on a double integrator")
        print(text_table(
            ["pendulum_goal_error", "pendulum_cost_final", "lqr_relative_gap"],
            [[pendulum.goal_error, pendulum.costs[-1], gap]],
        ))
    if args.out:
        figures = _need_figures()
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        figures.pendulum_figure(pendulum, out / "ilqr_pendulum.png")
        figures.lqr_figure(linear, lqr, out / "lqr_double_integrator.png")
        (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(f"wrote {out}/ilqr_pendulum.png and {out}/lqr_double_integrator.png", file=sys.stderr)
    return 0


def cmd_serve(args: argparse.Namespace) -> int:
    from .webapp import launch

    launch(host=args.host, port=args.port)
    return 0


def build_parser() -> argparse.ArgumentParser:
    _load_dotenv()
    parser = argparse.ArgumentParser(
        prog="robotics-lab",
        description="ROS 2 bag and CAN bus tools, plus two robotics papers in simulation.",
    )
    parser.add_argument("--version", action="version", version=f"robotics-lab {__version__}")
    sub = parser.add_subparsers(dest="command", required=True)

    ros = sub.add_parser("ros", help="rosbag2 rate, latency and jitter")
    ros_sub = ros.add_subparsers(dest="ros_command", required=True)
    analyze = ros_sub.add_parser("analyze", help="measure a rosbag2 directory")
    analyze.add_argument("bag", nargs="?", default=os.environ.get("ROS_BAG", "samples/ros/node_graph"))
    analyze.add_argument("--graph", help="graph JSON (default: <bag>.graph.json)")
    analyze.add_argument("--csv", help="write the topic table here; edges go to a sibling *_edges.csv")
    analyze.add_argument("--html", help="write an HTML report")
    analyze.add_argument("--json", action="store_true", help="print JSON instead of a table")
    analyze.set_defaults(func=cmd_ros_analyze)
    simulate = ros_sub.add_parser("simulate", help="write a synthetic rosbag2 mcap of the joint loop")
    simulate.add_argument("--out", default="out/node_graph")
    simulate.add_argument("--seconds", type=float, default=2.0)
    simulate.set_defaults(func=cmd_ros_simulate)

    can = sub.add_parser("can", help="CAN log, DBC decode, bus load")
    can_sub = can.add_subparsers(dest="can_command", required=True)
    log = can_sub.add_parser("log", help="record a virtual bus or SocketCAN")
    log.add_argument("--interface", default=os.environ.get("CAN_INTERFACE", "virtual"), choices=["virtual", "socketcan"])
    log.add_argument("--channel", default=os.environ.get("CAN_CHANNEL", "robotlab"))
    log.add_argument("--dbc", default=os.environ.get("CAN_DBC_FILE", "samples/can/robot.dbc"))
    log.add_argument("--bitrate", type=int, default=int(os.environ.get("CAN_BITRATE", BITRATE)))
    log.add_argument("--seconds", type=float, default=1.0)
    log.add_argument("--out", default="out/can")
    log.add_argument("--json", action="store_true")
    log.set_defaults(func=cmd_can_log)
    can_an = can_sub.add_parser("analyze", help="decode a trace CSV and report load, rate and latency")
    can_an.add_argument("trace", nargs="?", default="samples/can/trace.csv")
    can_an.add_argument("--dbc", default=os.environ.get("CAN_DBC_FILE", "samples/can/robot.dbc"))
    can_an.add_argument("--bitrate", type=int, default=None, help="overrides the bitrate comment in the trace")
    can_an.add_argument("--csv")
    can_an.add_argument("--html")
    can_an.add_argument("--json", action="store_true")
    can_an.set_defaults(func=cmd_can_analyze)

    papers = sub.add_parser("papers", help="simulation re-implementations")
    papers_sub = papers.add_subparsers(dest="papers_command", required=True)
    listed = papers_sub.add_parser("list", help="print the citations")
    listed.set_defaults(func=cmd_papers_list)
    dmp = papers_sub.add_parser("dmp", help="discrete dynamic movement primitives")
    dmp.add_argument("--out", help="directory for the figures")
    dmp.add_argument("--json", action="store_true")
    dmp.set_defaults(func=cmd_papers_dmp)
    ilqr = papers_sub.add_parser("ilqr", help="iLQR pendulum and double-integrator LQR")
    ilqr.add_argument("--out", help="directory for the figures")
    ilqr.add_argument("--json", action="store_true")
    ilqr.set_defaults(func=cmd_papers_ilqr)

    serve = sub.add_parser("serve", help="local web demo")
    serve.add_argument("--host", default="127.0.0.1")
    serve.add_argument("--port", type=int, default=7860)
    serve.set_defaults(func=cmd_serve)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except (RoboticsLabError, FileNotFoundError, ValueError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
