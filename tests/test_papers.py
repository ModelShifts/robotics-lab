from robotics_lab.papers.dmp import run_dmp
from robotics_lab.papers.ilqr import run_double_integrator, run_pendulum


def test_dmp_tracks_the_demonstration_and_scales():
    result = run_dmp()
    assert result.rmse < 0.02
    assert result.goal_error < 1e-3
    assert result.shape_rmse < 0.02
    assert result.scaled_goal_error < 1e-3
    assert 1.95 < result.time_scale_ratio < 2.05
    assert result.weights.shape == (20,)


def test_ilqr_pendulum_reaches_upright_and_cost_falls():
    result = run_pendulum()
    assert all(b <= a + 1e-8 for a, b in zip(result.costs, result.costs[1:]))
    assert result.costs[-1] < result.costs[0] / 5
    assert result.goal_error < 1e-4
    assert abs(result.xs[-1, 0]) < 1e-3
    assert abs(result.xs[-1, 1]) < 1e-3


def test_ilqr_matches_riccati_on_a_linear_system():
    result, lqr, gap = run_double_integrator()
    assert gap < 1e-6
    assert abs(result.costs[-1] - lqr.cost) < 1e-8
    assert abs(float(result.xs[-1, 0])) < 0.02
    assert abs(float(result.xs[-1, 0] - lqr.xs[-1, 0])) < 1e-8
