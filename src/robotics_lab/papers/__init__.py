"""Simulation re-implementations of two robotics papers."""

from .dmp import CITATION as DMP_CITATION
from .dmp import run_dmp
from .ilqr import CITATION as ILQR_CITATION
from .ilqr import run_double_integrator, run_pendulum

__all__ = [
    "DMP_CITATION",
    "ILQR_CITATION",
    "run_dmp",
    "run_double_integrator",
    "run_pendulum",
]
