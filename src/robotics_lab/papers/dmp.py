"""Discrete dynamic movement primitives, from the published equations.

Ijspeert, Nakanishi, Hoffmann, Pastor and Schaal (2013), Neural Computation
25(2), 328-373. Equation numbers below refer to that paper. Table 1 supplies
the constants used throughout the letter: alpha_z = 25, beta_z = alpha_z / 4,
alpha_x = alpha_z / 3. Integration is forward Euler with a 0.001 s step, as
in the letter.

The demonstration is synthetic: a fifth-order minimum-jerk polynomial from 0
to 1, plus a negative Gaussian bump so the velocity reverses, which is the
shape described for their Figure 1. It is not a digitization of that figure.
The rhythmic (limit-cycle) system in the same paper is not implemented.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

ALPHA_Z = 25.0
BETA_Z = ALPHA_Z / 4.0
ALPHA_X = ALPHA_Z / 3.0
N_BASIS = 20
DT = 0.001
DEMO_DURATION = 1.0

CITATION = (
    "Ijspeert, A. J., Nakanishi, J., Hoffmann, H., Pastor, P., and Schaal, S. (2013). "
    "Dynamical movement primitives: learning attractor models for motor behaviors. "
    "Neural Computation, 25(2), 328-373. https://doi.org/10.1162/NECO_a_00393"
)


@dataclass
class DMPResult:
    citation: str
    t_demo: np.ndarray
    y_demo: np.ndarray
    yd_demo: np.ndarray
    ydd_demo: np.ndarray
    t: np.ndarray
    y: np.ndarray
    yd: np.ndarray
    ydd: np.ndarray
    x: np.ndarray
    weights: np.ndarray
    centers: np.ndarray
    widths: np.ndarray
    psi: np.ndarray
    tau: float
    y0: float
    goal: float
    rmse: float
    goal_error: float
    scaled_goal: float
    scaled_goal_error: float
    shape_rmse: float
    time_scale: float
    time_scale_ratio: float


def demonstration(dt: float = DT, duration: float = DEMO_DURATION) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Minimum-jerk path from 0 to 1 plus a negative Gaussian bump. Analytic derivatives."""
    t = np.arange(0.0, duration + dt * 0.5, dt)
    s = t / duration
    y = 10 * s**3 - 15 * s**4 + 6 * s**5
    yd = (30 * s**2 - 60 * s**3 + 30 * s**4) / duration
    ydd = (60 * s - 180 * s**2 + 120 * s**3) / duration**2
    center, width, amp = 0.35, 0.07, 0.30
    d = t - center
    gauss = np.exp(-0.5 * (d / width) ** 2)
    y = y - amp * gauss
    yd = yd + amp * (d / width**2) * gauss
    ydd = ydd + amp * gauss / width**2 * (1.0 - (d / width) ** 2)
    return t, y, yd, ydd


def basis_layout(n: int = N_BASIS, tau: float = DEMO_DURATION) -> tuple[np.ndarray, np.ndarray]:
    """Centers equally spaced in time, which is exponential spacing in x (Table 1)."""
    t_i = tau * np.arange(n) / (n - 1)
    centers = np.exp(-ALPHA_X * t_i / tau)
    # Neighboring centers are one time step of the layout apart. The factor
    # makes a kernel fall to exp(-4) at the next center, local enough to fit
    # the bump in the demonstration. Table 1 only says the widths follow that
    # spacing; it does not give this constant.
    widths = np.empty(n)
    for i in range(n - 1):
        delta = centers[i] - centers[i + 1]
        widths[i] = 4.0 / (delta * delta)
    widths[-1] = widths[-2]
    return centers, widths


def _forcing(x: float, goal: float, y0: float, weights: np.ndarray, centers: np.ndarray, widths: np.ndarray) -> float:
    """Equation 2.3. Returns 0 when every basis function is inactive."""
    psi = np.exp(-widths * (x - centers) ** 2)
    total = float(psi.sum())
    if total < 1e-12:
        return 0.0
    return float(psi @ weights) / total * x * (goal - y0)


def _basis_matrix(x: np.ndarray, centers: np.ndarray, widths: np.ndarray) -> np.ndarray:
    return np.exp(-widths[None, :] * (x[:, None] - centers[None, :]) ** 2)


def learn_weights(
    t: np.ndarray,
    y: np.ndarray,
    yd: np.ndarray,
    ydd: np.ndarray,
    tau: float,
    goal: float,
    y0: float,
    centers: np.ndarray,
    widths: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Locally weighted regression, equations 2.12 and 2.14.

    The phase x is integrated with the same Euler step used at runtime.
    """
    dt = float(t[1] - t[0])
    x = np.empty_like(t)
    x[0] = 1.0
    for i in range(len(t) - 1):
        x[i + 1] = x[i] + dt * (-ALPHA_X * x[i] / tau)
    # Equation 2.12. z = tau * ydot, so tau * zdot = tau^2 * ydd.
    f_target = tau**2 * ydd - ALPHA_Z * (BETA_Z * (goal - y) - tau * yd)
    xi = x * (goal - y0)
    psi = _basis_matrix(x, centers, widths)
    weights = np.zeros(centers.shape[0])
    for i in range(weights.shape[0]):
        numer = float(psi[:, i] @ (xi * f_target))
        denom = float(psi[:, i] @ (xi * xi))
        weights[i] = numer / denom if denom > 1e-12 else 0.0
    return weights, x


def integrate(
    tau: float,
    goal: float,
    y0: float,
    weights: np.ndarray,
    centers: np.ndarray,
    widths: np.ndarray,
    dt: float,
    t_final: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Euler integration of equations 2.1 and 2.2."""
    steps = int(round(t_final / dt))
    t = np.arange(steps + 1) * dt
    y = np.empty(steps + 1)
    z = np.empty(steps + 1)
    x = np.empty(steps + 1)
    yd = np.empty(steps + 1)
    ydd = np.empty(steps + 1)
    y[0] = y0
    z[0] = 0.0
    x[0] = 1.0
    for i in range(steps + 1):
        f = _forcing(float(x[i]), goal, y0, weights, centers, widths)
        z_dot = (ALPHA_Z * (BETA_Z * (goal - y[i]) - z[i]) + f) / tau
        y_dot = z[i] / tau
        yd[i] = y_dot
        ydd[i] = z_dot / tau
        if i == steps:
            break
        x_dot = (-ALPHA_X * x[i]) / tau
        y[i + 1] = y[i] + dt * y_dot
        z[i + 1] = z[i] + dt * z_dot
        x[i + 1] = x[i] + dt * x_dot
    return t, y, yd, ydd, x, z


def _time_to_fraction(t: np.ndarray, y: np.ndarray, y0: float, goal: float, fraction: float) -> float:
    target = y0 + fraction * (goal - y0)
    for time, value in zip(t, y):
        if value >= target:
            return float(time)
    return float(t[-1])


def run_dmp() -> DMPResult:
    """Fit the synthetic demonstration and check spatial and temporal scaling."""
    t_demo, y_demo, yd_demo, ydd_demo = demonstration()
    tau = DEMO_DURATION
    y0 = float(y_demo[0])
    goal = float(y_demo[-1])
    centers, widths = basis_layout(N_BASIS, tau)
    weights, _x_learn = learn_weights(t_demo, y_demo, yd_demo, ydd_demo, tau, goal, y0, centers, widths)
    t, y, yd, ydd, x, _z = integrate(tau, goal, y0, weights, centers, widths, DT, t_final=1.5)
    # Compare on the demonstration window only.
    n = len(t_demo)
    rmse = float(np.sqrt(np.mean((y[:n] - y_demo) ** 2)))
    goal_error = abs(float(y[-1]) - goal)

    scaled_goal = 1.5
    _ts, ys, _yds, _ydds, _xs, _zs = integrate(tau, scaled_goal, y0, weights, centers, widths, DT, t_final=1.5)
    scale = scaled_goal - y0
    base = goal - y0
    shape_rmse = float(np.sqrt(np.mean(((ys[:n] - y0) / scale - (y_demo - y0) / base) ** 2)))
    scaled_goal_error = abs(float(ys[-1]) - scaled_goal)

    time_scale = 2.0
    t_slow, y_slow, *_rest = integrate(tau * time_scale, goal, y0, weights, centers, widths, DT, t_final=3.0)
    t_fast = _time_to_fraction(t, y, y0, goal, 0.9)
    t_stretched = _time_to_fraction(t_slow, y_slow, y0, goal, 0.9)
    ratio = t_stretched / t_fast if t_fast > 0 else float("inf")

    return DMPResult(
        citation=CITATION,
        t_demo=t_demo,
        y_demo=y_demo,
        yd_demo=yd_demo,
        ydd_demo=ydd_demo,
        t=t,
        y=y,
        yd=yd,
        ydd=ydd,
        x=x,
        weights=weights,
        centers=centers,
        widths=widths,
        psi=_basis_matrix(x, centers, widths),
        tau=tau,
        y0=y0,
        goal=goal,
        rmse=rmse,
        goal_error=goal_error,
        scaled_goal=scaled_goal,
        scaled_goal_error=scaled_goal_error,
        shape_rmse=shape_rmse,
        time_scale=time_scale,
        time_scale_ratio=ratio,
    )
