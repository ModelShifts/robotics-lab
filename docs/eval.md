# Evaluation

Figures in the README and on the case page come from this table. They were measured on 12 November 2026 (Asia/Singapore) by `python scripts/evaluate.py` against the synthetic generator settings. There is no language-model backend.

The samples are a synthetic rosbag2 (`samples/ros/node_graph`) and a synthetic CAN trace (`samples/can/trace.csv` plus `robot.dbc`), produced by `scripts/make_samples.py`. DMP and iLQR run in simulation from the published equations. The tools have not been run on hardware or on live ROS 2 nodes.

**Command:** `python scripts/evaluate.py`

Raw JSON: [`eval.json`](eval.json).

| Check | Result | Expected | Measured |
|---|---|---|---|
| ROS /joint_commands rate | pass | 100.0 Hz ± 0.02 | 100.007538 Hz |
| ROS /joint_commands jitter band | pass | 0.15 ms < std < 0.45 ms | 0.3045 ms |
| ROS /joint_feedback rate | pass | 100.0 Hz ± 0.02 | 100.008543 Hz |
| ROS /joint_feedback jitter band | pass | 0.15 ms < std < 0.45 ms | 0.2642 ms |
| ROS /joint_states rate | pass | 100.0 Hz ± 0.02 | 100.007538 Hz |
| ROS /joint_states jitter band | pass | 0.15 ms < std < 0.45 ms | 0.2585 ms |
| ROS sensor stamp latency | pass | 0 ms (stamp is the sample time) | 1.73e-18 ms |
| ROS command stamp latency | pass | 2.0 ms | 2.000000 ms |
| ROS feedback stamp latency | pass | 5.0 ms | 5.000000 ms |
| ROS edge /joint_states → /joint_commands | pass | 2.0 ms, n=200 | 2.000000 ms, n=200 |
| ROS edge /joint_commands → /joint_feedback | pass | 3.0 ms, n=200 | 3.000000 ms, n=200 |
| ROS edge /joint_states → /joint_feedback | pass | 5.0 ms, n=200 | 5.000000 ms, n=200 |
| CAN frame count | pass | 600 | 600, unknown 0 |
| CAN bus load | pass | about 0.081 (135 bits × 300 frames/s / 500000 bit/s) | 0.081199 |
| CAN edge JointState → JointCommand | pass | 2.0 ms | 2.000000 ms |
| CAN edge JointCommand → JointFeedback | pass | 3.0 ms | 3.000000 ms |
| CAN edge JointState → JointFeedback | pass | 5.0 ms | 5.000000 ms |
| DMP reproduction RMSE | pass | < 0.02 | 0.01072 |
| DMP goal error | pass | < 0.001 | 0.000316484 |
| DMP shape RMSE at new goal | pass | < 0.02 | 0.01072 |
| DMP time-scale ratio | pass | 2 ± 0.05 | 2.0027 |
| iLQR cost decreases | pass | monotone and final < initial/5 | 1.382e+04 → 1072 in 6 steps |
| iLQR pendulum goal error | pass | < 1e-4 | 1.35e-07 |
| iLQR matches Riccati on the double integrator | pass | relative cost gap < 1e-6, |position| < 0.02 | gap 1.16e-16, position -0.00770, LQR cost 7.654202 |
| **All** | **24/24 passed** | | |

Grouped: 12 ROS checks, 5 CAN checks, 4 DMP checks, 3 iLQR checks — 24/24.

Headline measured values used in the README:

- ROS topic rates: 100.007538 Hz, 100.008543 Hz, 100.007538 Hz (expected 100.0 Hz ± 0.02) on `/joint_commands`, `/joint_feedback`, `/joint_states`.
- ROS stamp latencies: 0 ms (sensor), 2.000000 ms (command), 5.000000 ms (feedback).
- ROS edges: 2.000000 ms, 3.000000 ms, 5.000000 ms, n=200.
- CAN: 600 frames, unknown 0; bus load 0.081199 (expected about 0.081); edges 2.000000, 3.000000, 5.000000 ms.
- DMP: reproduction RMSE 0.01072, goal error 0.000316484, shape RMSE at new goal 0.01072, time-scale ratio 2.0027.
- iLQR: cost 1.382e+04 → 1072 in 6 steps (monotone, final < initial/5); pendulum goal error 1.35e-07; Riccati relative cost gap 1.16e-16, position −0.00770, LQR cost 7.654202.
