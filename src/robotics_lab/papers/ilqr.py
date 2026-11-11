"""Iterative LQR, and finite-horizon LQR as its linear special case.

Li and Todorov (2004), Iterative linear quadratic regulator design for
nonlinear biological movement systems, ICINCO. The backward pass below is
that iteration: linearize the dynamics, take a quadratic model of the cost,
solve for an affine control update, and roll out with a line search.

On a linear plant with quadratic cost the quadratic model is exact, so one
iteration matches a finite-horizon Riccati LQR (Anderson and Moore, Optimal
Control: Linear Quadratic Methods, 1990). That check uses a double integrator.

The nonlinear example is a single pendulum, angle measured from upright.
It is not the biomechanical arm in Li and Todorov. If Q_uu is not positive
definite, a multiple of the identity is added until it is; that safeguard is
the one used by Tassa, Mansard and Todorov (ICRA 2014). Control limits are
not projected.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

CITATION = (
    "Li, W. and Todorov, E. (2004). Iterative linear quadratic regulator design for "
    "nonlinear biological movement systems. Proceedings of the 1st International "
    "Conference on Informatics in Control, Automation and Robotics (ICINCO)."
)
LQR_CITATION = (
    "Anderson, B. D. O. and Moore, J. B. (1990). Optimal Control: Linear Quadratic Methods. "
    "Prentice Hall."
)
REGULARIZATION_CITATION = (
    "Tassa, Y., Mansard, N., and Todorov, E. (2014). Control-limited differential dynamic "
    "programming. IEEE International Conference on Robotics and Automation."
)


@dataclass
class Trajectory:
    xs: np.ndarray  # (N+1, n)
    us: np.ndarray  # (N, m)
    cost: float


@dataclass
class ILQRResult:
    citation: str
    costs: list[float]
    xs: np.ndarray
    us: np.ndarray
    goal: np.ndarray
    goal_error: float
    dt: float


def stage_cost(x: np.ndarray, u: np.ndarray, goal: np.ndarray, q: np.ndarray, r: np.ndarray) -> float:
    e = x - goal
    return float(0.5 * e @ q @ e + 0.5 * u @ r @ u)


def terminal_cost(x: np.ndarray, goal: np.ndarray, qf: np.ndarray) -> float:
    e = x - goal
    return float(0.5 * e @ qf @ e)


def trajectory_cost(xs: np.ndarray, us: np.ndarray, goal: np.ndarray, q: np.ndarray, r: np.ndarray, qf: np.ndarray) -> float:
    total = terminal_cost(xs[-1], goal, qf)
    for x, u in zip(xs[:-1], us):
        total += stage_cost(x, u, goal, q, r)
    return total


def finite_horizon_lqr(a: np.ndarray, b: np.ndarray, q: np.ndarray, r: np.ndarray, qf: np.ndarray, horizon: int) -> list[np.ndarray]:
    """Gains K_k for u = -K x. Cost is 1/2 x'Q x + 1/2 u'R u, goal at the origin."""
    p = qf.copy()
    gains: list[np.ndarray] = []
    for _ in range(horizon):
        bt_p = b.T @ p
        k = np.linalg.solve(r + bt_p @ b, bt_p @ a)
        p = q + a.T @ p @ a - a.T @ p @ b @ k
        p = 0.5 * (p + p.T)
        gains.append(k)
    gains.reverse()
    return gains


def rollout_linear(a: np.ndarray, b: np.ndarray, x0: np.ndarray, gains: list[np.ndarray], goal, q, r, qf) -> Trajectory:
    n = x0.size
    xs = np.zeros((len(gains) + 1, n))
    us = np.zeros((len(gains), b.shape[1]))
    xs[0] = x0
    for k, gain in enumerate(gains):
        us[k] = -gain @ xs[k]
        xs[k + 1] = a @ xs[k] + b @ us[k]
    return Trajectory(xs, us, trajectory_cost(xs, us, goal, q, r, qf))


def _gains(q_uu: np.ndarray, q_u: np.ndarray, q_ux: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    eye = np.eye(q_uu.shape[0])
    mu = 0.0
    for _ in range(12):
        try:
            factor = q_uu + mu * eye
            np.linalg.cholesky(factor)
            k = -np.linalg.solve(factor, q_u)
            gain = -np.linalg.solve(factor, q_ux)
            return k, gain
        except np.linalg.LinAlgError:
            mu = 1e-4 if mu == 0.0 else mu * 10.0
    raise np.linalg.LinAlgError("Q_uu stayed indefinite")


def ilqr(
    step,
    x0: np.ndarray,
    u_guess: np.ndarray,
    goal: np.ndarray,
    q: np.ndarray,
    r: np.ndarray,
    qf: np.ndarray,
    max_iter: int = 25,
    jacobians=None,
) -> ILQRResult:
    """Run iLQR. ``step(x, u) -> x_next``. ``u_guess`` has shape (N, m)."""
    u_bar = np.array(u_guess, dtype=float, copy=True)
    horizon, m = u_bar.shape

    def rollout(controls: np.ndarray) -> np.ndarray:
        xs = np.zeros((horizon + 1, x0.size))
        xs[0] = x0
        for k in range(horizon):
            xs[k + 1] = step(xs[k], controls[k])
        return xs

    def jacobians_fd(x: np.ndarray, u: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if jacobians is not None:
            return jacobians(x, u)
        eps = 1e-6
        a = np.zeros((x.size, x.size))
        b = np.zeros((x.size, u.size))
        for i in range(x.size):
            d = np.zeros(x.size)
            d[i] = eps
            a[:, i] = (step(x + d, u) - step(x - d, u)) / (2 * eps)
        for i in range(u.size):
            d = np.zeros(u.size)
            d[i] = eps
            b[:, i] = (step(x, u + d) - step(x, u - d)) / (2 * eps)
        return a, b

    x_bar = rollout(u_bar)
    costs = [trajectory_cost(x_bar, u_bar, goal, q, r, qf)]

    for _ in range(max_iter):
        k_ff = [np.zeros(m) for _ in range(horizon)]
        k_fb = [np.zeros((m, x0.size)) for _ in range(horizon)]
        v_x = qf @ (x_bar[-1] - goal)
        v_xx = qf.copy()
        for k in range(horizon - 1, -1, -1):
            a, b = jacobians_fd(x_bar[k], u_bar[k])
            e = x_bar[k] - goal
            l_x = q @ e
            l_u = r @ u_bar[k]
            q_x = l_x + a.T @ v_x
            q_u = l_u + b.T @ v_x
            q_xx = q + a.T @ v_xx @ a
            q_ux = b.T @ v_xx @ a
            q_uu = r + b.T @ v_xx @ b
            k_ff[k], k_fb[k] = _gains(q_uu, q_u, q_ux)
            v_x = q_x + k_fb[k].T @ q_uu @ k_ff[k] + k_fb[k].T @ q_u + q_ux.T @ k_ff[k]
            v_xx = q_xx + k_fb[k].T @ q_uu @ k_fb[k] + k_fb[k].T @ q_ux + q_ux.T @ k_fb[k]
            v_xx = 0.5 * (v_xx + v_xx.T)

        accepted = None
        base = costs[-1]
        for alpha in (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125, 0.01):
            xs = np.zeros_like(x_bar)
            us = np.zeros_like(u_bar)
            xs[0] = x0
            ok = True
            for k in range(horizon):
                us[k] = u_bar[k] + alpha * k_ff[k] + k_fb[k] @ (xs[k] - x_bar[k])
                nxt = step(xs[k], us[k])
                if not np.all(np.isfinite(nxt)):
                    ok = False
                    break
                xs[k + 1] = nxt
            if not ok:
                continue
            cost = trajectory_cost(xs, us, goal, q, r, qf)
            if cost < base - 1e-9:
                accepted = (xs, us, cost)
                break
        if accepted is None:
            break
        x_bar, u_bar, cost = accepted
        costs.append(cost)
        if abs(costs[-2] - cost) < 1e-8 * (1.0 + abs(cost)):
            break

    return ILQRResult(
        citation=CITATION,
        costs=costs,
        xs=x_bar,
        us=u_bar,
        goal=goal,
        goal_error=float(np.linalg.norm(x_bar[-1] - goal)),
        dt=0.0,
    )


def double_integrator(dt: float = 0.05):
    """Discrete double integrator, zero-order hold on acceleration."""
    a = np.array([[1.0, dt], [0.0, 1.0]])
    b = np.array([[0.5 * dt * dt], [dt]])
    return a, b


def run_double_integrator() -> tuple[ILQRResult, Trajectory, float]:
    """iLQR and Riccati LQR on the same linear problem. Returns both and the relative cost gap."""
    dt = 0.05
    horizon = 60
    a, b = double_integrator(dt)
    q = np.diag([1.0, 0.1])
    r = np.array([[0.05]])
    qf = np.diag([20.0, 2.0])
    goal = np.zeros(2)
    x0 = np.array([1.0, 0.0])
    gains = finite_horizon_lqr(a, b, q, r, qf, horizon)
    lqr = rollout_linear(a, b, x0, gains, goal, q, r, qf)

    def step(x, u):
        return a @ x + b @ u

    result = ilqr(step, x0, np.zeros((horizon, 1)), goal, q, r, qf, max_iter=5, jacobians=lambda x, u: (a, b))
    result.dt = dt
    gap = abs(result.costs[-1] - lqr.cost) / max(lqr.cost, 1e-12)
    return result, lqr, gap


def _pendulum_deriv(x: np.ndarray, u: np.ndarray) -> np.ndarray:
    # theta = 0 is upright. m = l = 1.
    g, b_visc = 9.81, 0.05
    theta, omega = float(x[0]), float(x[1])
    torque = float(u[0])
    alpha = g * np.sin(theta) - b_visc * omega + torque
    return np.array([omega, alpha])


def pendulum_step(x: np.ndarray, u: np.ndarray, dt: float = 0.05) -> np.ndarray:
    """One RK4 step with torque held constant."""

    def f(state: np.ndarray) -> np.ndarray:
        return _pendulum_deriv(state, u)

    k1 = f(x)
    k2 = f(x + 0.5 * dt * k1)
    k3 = f(x + 0.5 * dt * k2)
    k4 = f(x + dt * k3)
    return x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def run_pendulum() -> ILQRResult:
    """Bring a pendulum from hanging to upright. Angle is measured from upright."""
    dt = 0.05
    horizon = 80
    q = np.diag([30.0, 1.0])
    r = np.array([[0.05]])
    qf = np.diag([400.0, 20.0])
    goal = np.zeros(2)
    x0 = np.array([np.pi, 0.0])
    result = ilqr(
        lambda x, u: pendulum_step(x, u, dt),
        x0,
        np.zeros((horizon, 1)),
        goal,
        q,
        r,
        qf,
        max_iter=30,
    )
    result.dt = dt
    return result
