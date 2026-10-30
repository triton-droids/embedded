# Safety checks

For launch modes, monitoring, ROS network setup and self-collision, see the
[package guide](../../README.md).

## Safety checks layout

`safety_node` is the only publisher of `/safety/estop`. It turns each ROS
message into a plain observation and hands it to `SafetySupervisor`, which
passes it to every check and latches the estop if any check trips. The checks
live in `humanoid_safety/checks/`, one file per sensor area:

| File | Checks | Input |
|---|---|---|
| `base.py` | `SafetyCheck` base class, observation types, `FreshnessCheck` | |
| `supervisor.py` | `SafetySupervisor` latch and reset | |
| `imu.py` | fall, IMU freshness, IMU stuck | `/imu/data_raw` |
| `motors.py` | temperature, fault bits, torque, stale feedback | `/motor_telemetry`, `/joint_states` |
| `joints.py` | position and velocity limits | `/joint_states` |
| `comms.py` | CAN transport fault or silent motor node | `/can_timing` |
| `power.py` | battery voltage | `/battery_state` |

Only the IMU checks are on by default; turn the others on in
`config/safety.yaml` once tested. The node subscribes only to topics an
enabled check uses.
