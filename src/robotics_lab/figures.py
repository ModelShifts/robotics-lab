"""Matplotlib figures for the analyzers and the two papers.

Colours match the README banner: ink #1c1a17, rust #8a3a24, cream #faf7f0.
"""

from __future__ import annotations

import io
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from .canbus.analyze import CanReport
from .papers.dmp import DMPResult, integrate
from .papers.dmp import DT as DMP_DT
from .papers.ilqr import ILQRResult, Trajectory
from .ros.analyze import BagReport

INK = "#1c1a17"
RUST = "#8a3a24"
CREAM = "#faf7f0"
MUTED = "#6b665e"
GRID = "#e4dccb"
BLUE = "#3d4c5c"


def _style() -> None:
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.size": 11,
            "axes.edgecolor": INK,
            "axes.labelcolor": INK,
            "axes.titlecolor": INK,
            "xtick.color": INK,
            "ytick.color": INK,
            "text.color": INK,
            "figure.facecolor": CREAM,
            "axes.facecolor": CREAM,
            "axes.grid": True,
            "grid.color": GRID,
            "grid.linewidth": 0.6,
            "axes.axisbelow": True,
            "legend.frameon": False,
        }
    )


def to_image(fig: plt.Figure) -> Image.Image:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=120)
    plt.close(fig)
    buf.seek(0)
    return Image.open(buf).convert("RGB")


def _save(fig: plt.Figure, path: Path | None) -> Image.Image:
    image = to_image(fig)
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        image.save(path)
    return image


def ros_latency_figure(report: BagReport, path: Path | None = None) -> Image.Image:
    _style()
    fig, ax = plt.subplots(figsize=(8.2, 4.4))
    colours = [INK, RUST, BLUE]
    for edge, colour in zip(report.edges, colours):
        ax.plot(edge.times_s, edge.latency_ms, color=colour, lw=1.4, label=f"{edge.source} → {edge.target}")
    ax.set_xlabel("Time from first message (s)")
    ax.set_ylabel("Log-time latency (ms)")
    ax.set_title("End-to-end latency along the synthetic node graph")
    fig.legend(loc="lower center", ncol=1, fontsize=8, bbox_to_anchor=(0.5, 0.0))
    fig.tight_layout(rect=(0, 0.16, 1, 1))
    return _save(fig, path)


def can_latency_figure(report: CanReport, path: Path | None = None) -> Image.Image:
    _style()
    fig, ax = plt.subplots(figsize=(8.2, 4.4))
    colours = [INK, RUST, BLUE]
    for edge, colour in zip(report.edges, colours):
        ax.plot(edge.times_s, edge.latency_ms, color=colour, lw=1.4, label=f"{edge.source} → {edge.target}")
    ax.set_xlabel("Time from first frame (s)")
    ax.set_ylabel("Latency (ms)")
    ax.set_title(f"CAN latency  ·  bus load {100 * report.bus_load:.2f}% at {report.bitrate} bit/s")
    fig.legend(loc="lower center", ncol=1, fontsize=8, bbox_to_anchor=(0.5, 0.0))
    fig.tight_layout(rect=(0, 0.16, 1, 1))
    return _save(fig, path)


def dmp_figure(result: DMPResult, path: Path | None = None) -> Image.Image:
    """Panels in the arrangement of Figure 1 of Ijspeert et al. (2013), plus scaling."""
    _style()
    fig, axes = plt.subplots(3, 2, figsize=(9.4, 8.2), sharex=False)
    demo_style = dict(color=MUTED, ls="--", lw=1.3, label="demonstration")
    fit_style = dict(color=INK, lw=1.5, label="DMP")

    axes[0, 0].plot(result.t_demo, result.y_demo, **demo_style)
    axes[0, 0].plot(result.t, result.y, **fit_style)
    axes[0, 0].set_ylabel("y")
    axes[0, 0].set_title("Position")
    axes[0, 0].legend(loc="lower right")

    axes[0, 1].plot(result.t_demo, result.yd_demo, **demo_style)
    axes[0, 1].plot(result.t, result.yd, **fit_style)
    axes[0, 1].set_ylabel("dy/dt")
    axes[0, 1].set_title("Velocity")

    axes[1, 0].plot(result.t_demo, result.ydd_demo, **demo_style)
    axes[1, 0].plot(result.t, result.ydd, **fit_style)
    axes[1, 0].set_ylabel("d²y/dt²")
    axes[1, 0].set_title("Acceleration")

    for i in range(result.psi.shape[1]):
        axes[1, 1].plot(result.t, result.psi[:, i], color=RUST, lw=0.9, alpha=0.85)
    axes[1, 1].set_ylabel("ψ(x)")
    axes[1, 1].set_title("Basis functions (20)")

    axes[2, 0].plot(result.t, result.x, color=INK, lw=1.5)
    axes[2, 0].set_ylabel("x")
    axes[2, 0].set_xlabel("Time (s)")
    axes[2, 0].set_title("Phase")

    for goal, colour, label in (
        (result.goal, INK, "trained goal"),
        (result.scaled_goal, RUST, "new goal"),
        (0.4, BLUE, "new goal 0.4"),
    ):
        t, y, *_rest = integrate(result.tau, goal, result.y0, result.weights, result.centers, result.widths, DMP_DT, 1.5)
        axes[2, 1].plot(t, y, color=colour, lw=1.4, label=label)
    axes[2, 1].set_ylabel("y")
    axes[2, 1].set_xlabel("Time (s)")
    axes[2, 1].set_title("Same weights, new goals")
    axes[2, 1].legend(loc="best")

    fig.suptitle("Discrete DMP  ·  Ijspeert et al. 2013, Figure 1 form", fontsize=13)
    fig.tight_layout(rect=(0, 0.02, 1, 0.96))
    fig.text(
        0.01,
        0.005,
        "Synthetic minimum-jerk path plus a negative Gaussian bump. Not a digitization of the paper's figure.",
        color=MUTED,
        fontsize=8,
    )
    return _save(fig, path)


def dmp_scaling_figure(result: DMPResult, path: Path | None = None) -> Image.Image:
    """Temporal scaling, section 2.1.4: doubling tau stretches the movement."""
    _style()
    fig, ax = plt.subplots(figsize=(8.2, 4.2))
    t_slow, y_slow, *_rest = integrate(
        result.tau * result.time_scale, result.goal, result.y0, result.weights, result.centers, result.widths, DMP_DT, 3.0
    )
    ax.plot(result.t, result.y, color=INK, lw=1.6, label=f"tau = {result.tau:g}")
    ax.plot(t_slow, y_slow, color=RUST, lw=1.6, label=f"tau = {result.tau * result.time_scale:g}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("y")
    ax.set_title("Temporal scaling with the same weights")
    ax.legend(loc="lower right")
    fig.tight_layout()
    return _save(fig, path)


def pendulum_figure(result: ILQRResult, path: Path | None = None) -> Image.Image:
    _style()
    t = np.arange(result.xs.shape[0]) * result.dt
    fig, axes = plt.subplots(2, 2, figsize=(9.2, 6.2))
    axes[0, 0].plot(t, result.xs[:, 0], color=INK, lw=1.6)
    axes[0, 0].axhline(0, color=GRID, lw=0.8)
    axes[0, 0].set_ylabel("theta (rad)")
    axes[0, 0].set_title("Angle from upright")
    axes[0, 1].plot(t, result.xs[:, 1], color=RUST, lw=1.6)
    axes[0, 1].set_ylabel("omega (rad/s)")
    axes[0, 1].set_title("Angular rate")
    tu = t[:-1]
    axes[1, 0].plot(tu, result.us[:, 0], color=BLUE, lw=1.4)
    axes[1, 0].set_ylabel("torque")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_title("Control")
    axes[1, 1].plot(range(len(result.costs)), result.costs, color=RUST, marker="o", lw=1.4)
    axes[1, 1].set_xlabel("Iteration")
    axes[1, 1].set_ylabel("Cost")
    axes[1, 1].set_title("Cost after each accepted step")
    fig.suptitle("Pendulum iLQR  ·  Li and Todorov 2004", fontsize=13)
    fig.tight_layout(rect=(0, 0.03, 1, 0.95))
    fig.text(
        0.01,
        0.005,
        "Single pendulum, m = l = 1, theta = 0 upright, started at theta = pi. Not the arm model in the paper.",
        color=MUTED,
        fontsize=8,
    )
    return _save(fig, path)


def lqr_figure(ilqr_result: ILQRResult, lqr: Trajectory, path: Path | None = None) -> Image.Image:
    _style()
    t = np.arange(ilqr_result.xs.shape[0]) * ilqr_result.dt
    fig, axes = plt.subplots(1, 2, figsize=(9.2, 3.8))
    axes[0].plot(t, lqr.xs[:, 0], color=MUTED, ls="--", lw=2.2, label="Riccati LQR")
    axes[0].plot(t, ilqr_result.xs[:, 0], color=INK, lw=1.3, label="iLQR")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("position")
    axes[0].set_title("Double integrator position")
    axes[0].legend(loc="best")
    axes[1].plot(t, lqr.xs[:, 1], color=MUTED, ls="--", lw=2.2, label="Riccati LQR")
    axes[1].plot(t, ilqr_result.xs[:, 1], color=RUST, lw=1.3, label="iLQR")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("velocity")
    axes[1].set_title("Double integrator velocity")
    axes[1].legend(loc="best")
    fig.suptitle("Linear-quadratic case: one iLQR iteration matches finite-horizon LQR", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    return _save(fig, path)
