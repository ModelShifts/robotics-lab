"""Local web demo. Header and colours follow the document-extractor demo."""

from __future__ import annotations

from pathlib import Path

from . import __version__
from .canbus.analyze import analyze_trace
from .canbus.log import record_bus
from .figures import can_latency_figure, dmp_figure, lqr_figure, pendulum_figure, ros_latency_figure
from .papers.dmp import run_dmp
from .papers.ilqr import run_double_integrator, run_pendulum
from .ros.analyze import analyze_bag

CSS = """
:root, .gradio-container { --ms-ink:#1c1a17; --ms-rust:#8a3a24; --ms-paper:#f3efe6; --ms-cream:#faf7f0; }
.gradio-container { background: var(--ms-cream) !important; font-family: Georgia, 'Times New Roman', serif !important; max-width: 1240px !important; }
.ms-header { border-bottom: 1px solid var(--ms-ink); padding: 10px 4px 12px; margin-bottom: 8px; }
.ms-brand { letter-spacing: .22em; font-size: 12px; color: var(--ms-rust); font-weight: 700; }
.ms-header h1 { font-family: Georgia, serif; font-weight: 400; font-size: 30px; margin: 2px 0 2px; color: var(--ms-ink); }
.ms-header p { margin: 0; color: #4a463f; font-style: italic; }
.ms-meta { margin-top: 8px; font-family: ui-monospace, Menlo, monospace; font-size: 12px; color: #6b665e; }
.ms-muted { color: #6b665e; font-size: 12px; }
.ms-card { background: #fff !important; border: 1px solid #ddd5c6 !important; border-radius: 10px !important; }
.ms-section-title { font-size: 12px; letter-spacing: .14em; text-transform: uppercase; color: var(--ms-rust); margin: 2px 0; }
.ms-table * { font-family: ui-sans-serif, system-ui, -apple-system, 'Segoe UI', sans-serif !important; font-size: 12.5px !important; }
.ms-status { font-size: 14px; }
button.primary, .ms-primary button { background: var(--ms-rust) !important; border-color: var(--ms-rust) !important; color: #fff !important; }
footer { display: none !important; }
"""


def _root() -> Path:
    for candidate in (Path.cwd(), Path(__file__).resolve().parents[2]):
        if (candidate / "samples" / "ros" / "node_graph").is_dir():
            return candidate
    return Path.cwd()


def ros_view():
    bag = _root() / "samples" / "ros" / "node_graph"
    report = analyze_bag(bag)
    topics = [
        [t.topic, t.count, round(t.rate_hz, 3), round(t.jitter_ms, 4), None if t.stamp_latency_ms_mean is None else round(t.stamp_latency_ms_mean, 4)]
        for t in report.topics
    ]
    edges = [
        [e.source, e.target, e.count, round(e.latency_ms_mean, 4), round(e.latency_ms_std, 4), round(e.latency_ms_p95, 4)]
        for e in report.edges
    ]
    note = (
        f"Computed just now from `{bag.name}` ({report.mcap_message_count} messages). "
        "Log times come from the mcap reader; fields are decoded with rosbags. No ROS 2 install is used."
    )
    return topics, edges, ros_latency_figure(report), note


def can_view(report=None, note: str | None = None):
    if report is None:
        trace = _root() / "samples" / "can" / "trace.csv"
        dbc = _root() / "samples" / "can" / "robot.dbc"
        report = analyze_trace(trace, dbc)
        note = (
            f"Computed just now from `{trace.name}` ({report.frame_count} frames). "
            f"Bus load {100 * report.bus_load:.2f}% is a worst-case bit-stuffing estimate at {report.bitrate} bit/s."
        )
    messages = [[m.name, m.arbitration_id, m.count, round(m.rate_hz, 3), round(m.jitter_ms, 4)] for m in report.messages]
    edges = [
        [e.source, e.target, e.count, round(e.latency_ms_mean, 4), round(e.latency_ms_std, 4), round(e.latency_ms_p95, 4)]
        for e in report.edges
    ]
    return messages, edges, can_latency_figure(report), note


def can_sample_view():
    return can_view()


def can_record():
    root = _root()
    out = root / "out" / "web-can"
    report = record_bus(
        "virtual",
        "robotlab-web",
        out,
        seconds=0.5,
        dbc_path=root / "samples" / "can" / "robot.dbc",
    )
    note = (
        f"Recorded just now on the virtual bus `robotlab-web` ({report.frame_count} frames). "
        f"Files are in `{out}`. Timestamps are the host clock, so the delays are not the exact schedule."
    )
    return can_view(report, note)


def papers_view():
    dmp = run_dmp()
    pendulum = run_pendulum()
    linear, lqr, gap = run_double_integrator()
    rows = [
        ["DMP reproduction RMSE", f"{dmp.rmse:.4f}"],
        ["DMP goal error", f"{dmp.goal_error:.4g}"],
        ["DMP shape RMSE at a new goal", f"{dmp.shape_rmse:.4f}"],
        ["DMP duration ratio when tau doubles", f"{dmp.time_scale_ratio:.3f}"],
        ["Pendulum goal error", f"{pendulum.goal_error:.4g}"],
        ["Pendulum cost, first to last", f"{pendulum.costs[0]:.4g} → {pendulum.costs[-1]:.4g}"],
        ["Double-integrator cost gap, iLQR vs Riccati", f"{gap:.3g}"],
    ]
    note = "Computed just now in this process. Both papers are simulation only; the demonstrations are synthetic."
    return rows, dmp_figure(dmp), pendulum_figure(pendulum), lqr_figure(linear, lqr), note


def theme():
    import gradio as gr

    return gr.themes.Base(
        primary_hue=gr.themes.colors.orange,
        neutral_hue=gr.themes.colors.stone,
        font=[gr.themes.Font("Georgia"), gr.themes.Font("Times New Roman"), gr.themes.Font("serif")],
        font_mono=[gr.themes.Font("ui-monospace"), gr.themes.Font("Menlo"), gr.themes.Font("monospace")],
        radius_size=gr.themes.sizes.radius_md,
    )


def build_app():
    import gradio as gr

    topics, edges, ros_image, ros_note = ros_view()
    messages, can_edges, can_image, can_note = can_sample_view()
    paper_rows, dmp_image, pend_image, lqr_image, paper_note = papers_view()

    with gr.Blocks(title="robotics-lab · Modelshifts") as demo:
        gr.HTML(
            "<div class='ms-header'><div class='ms-brand'>MODELSHIFTS</div><h1>robotics-lab</h1>"
            "<p>ROS 2 and CAN bus tools, plus re-implementations of robotics papers.</p>"
            f"<div class='ms-meta'>v{__version__} · synthetic data · no language model</div></div>"
        )
        with gr.Tabs():
            with gr.Tab("ROS 2 bag"):
                gr.HTML("<div class='ms-section-title'>Topic rate, jitter and end-to-end latency</div>")
                with gr.Row(equal_height=False):
                    with gr.Column(scale=5):
                        ros_topics = gr.Dataframe(
                            headers=["Topic", "Count", "Rate (Hz)", "Jitter (ms)", "Stamp latency (ms)"],
                            value=topics,
                            interactive=False,
                            elem_classes="ms-table",
                        )
                        ros_edges = gr.Dataframe(
                            headers=["Source", "Target", "Count", "Latency ms", "Std ms", "p95"],
                            value=edges,
                            interactive=False,
                            elem_classes="ms-table",
                        )
                        ros_status = gr.Markdown(ros_note, elem_classes="ms-status")
                        ros_btn = gr.Button("Analyze sample bag again", variant="primary", elem_classes="ms-primary")
                    with gr.Column(scale=6):
                        ros_plot = gr.Image(value=ros_image, type="pil", show_label=False, height=480, elem_classes="ms-card")
                ros_btn.click(ros_view, outputs=[ros_topics, ros_edges, ros_plot, ros_status])
            with gr.Tab("CAN bus"):
                gr.HTML("<div class='ms-section-title'>DBC decode, latency and bus load</div>")
                with gr.Row(equal_height=False):
                    with gr.Column(scale=5):
                        can_messages = gr.Dataframe(
                            headers=["Message", "ID", "Count", "Rate (Hz)", "Jitter (ms)"],
                            value=messages,
                            interactive=False,
                            elem_classes="ms-table",
                        )
                        can_edge_table = gr.Dataframe(
                            headers=["Source", "Target", "Count", "Latency ms", "Std ms", "p95"],
                            value=can_edges,
                            interactive=False,
                            elem_classes="ms-table",
                        )
                        can_status = gr.Markdown(can_note, elem_classes="ms-status")
                        with gr.Row():
                            can_btn = gr.Button("Analyze sample trace", variant="primary", elem_classes="ms-primary")
                            can_rec = gr.Button("Record 0.5 s on the virtual bus")
                    with gr.Column(scale=6):
                        can_plot = gr.Image(value=can_image, type="pil", show_label=False, height=480, elem_classes="ms-card")
                can_btn.click(can_sample_view, outputs=[can_messages, can_edge_table, can_plot, can_status])
                can_rec.click(can_record, outputs=[can_messages, can_edge_table, can_plot, can_status])
            with gr.Tab("Papers"):
                gr.HTML("<div class='ms-section-title'>DMP and iLQR, computed in this process</div>")
                with gr.Row():
                    dmp_plot = gr.Image(value=dmp_image, type="pil", show_label=False, height=460, elem_classes="ms-card")
                    pend_plot = gr.Image(value=pend_image, type="pil", show_label=False, height=460, elem_classes="ms-card")
                lqr_plot = gr.Image(value=lqr_image, type="pil", show_label=False, height=300, elem_classes="ms-card")
                paper_table = gr.Dataframe(
                    headers=["Check", "Value"],
                    value=paper_rows,
                    interactive=False,
                    elem_classes="ms-table",
                )
                paper_status = gr.Markdown(paper_note, elem_classes="ms-status")
                paper_btn = gr.Button("Run both papers again", variant="primary", elem_classes="ms-primary")
                paper_btn.click(papers_view, outputs=[paper_table, dmp_plot, pend_plot, lqr_plot, paper_status])
        gr.HTML(
            "<p class='ms-muted'>Samples are synthetic. Live ROS 2 nodes are not included. "
            "SocketCAN is available from the CLI when the channel exists; this page records the virtual bus. "
            "python-can is LGPL-3.0. The other direct dependencies used here are MIT, BSD or Apache-2.0.</p>"
        )
    return demo


def launch(host: str = "127.0.0.1", port: int = 7860) -> None:
    demo = build_app()
    demo.queue().launch(
        server_name=host,
        server_port=port,
        share=False,
        theme=theme(),
        css=CSS,
        footer_links=[],
        ssr_mode=False,
    )
