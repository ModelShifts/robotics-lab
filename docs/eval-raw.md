
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

24/24 passed
