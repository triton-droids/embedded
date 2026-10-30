# Physical robot archive

This repository's active target is MuJoCo simulation. This directory preserves
physical-device sources and reference material; `COLCON_IGNORE` excludes the
entire tree from default discovery, dependency installation and workspace builds.
Nothing here is installed by the simulation control package. Existing launch
source and defaults are preserved; physical options in the shared vision/policy
launches still require separately restored hardware packages. MuJoCo does not
enable those options.

| Path | Archived content |
|---|---|
| `motor_control_hybrid/` | RobStride CAN node/transport, ENCOS C++ backend and fetch recipe, physical bench configs, control launches and associated tests. |
| `imu_process/` | Body/head MCU firmware, serial/CAN/UDP acquisition, fusion helpers and fall detector. |
| `attitude_sensing_pkg/` | Original serial ROS IMU package, formerly under `imu_process/Attitude_Sensing(Depreciated)/src/`. |
| `utils/` | Direct-device RobStride gain tuner and compatibility launcher. |
| `bench/` | Physical BNO085 streaming benchmark and PlatformIO configuration. |
| `humanoid_arm_description/` | Independent arm model and meshes, not used by the leg simulator. |
| `launch/` | Physical system and camera/micro-ROS head-tracking launch recipes. |
| `scripts/` | Direct CAN wire check. |
| `devcontainer/` | Original hardware-oriented Dockerfile and ENCOS zstd shim. |
| `docs/` | Original interface/deployment reference and CAN/motor protocol documentation. |

## Boundaries

The simulator still uses `motor_control_interfaces`, the C++ scheduler, safety,
Web monitoring, the ONNX policy, motor calibration helpers and the leg model.
Those remain outside this archive. In particular, `motors.yaml`, `MotorSettings`
and the ankle mapping are retained because simulated backend feedback depends
on them; retaining those helpers does not open any CAN or serial device.

The original hardware code used installed `motor_control_hybrid.*` modules and
hardware launch files. Archiving does not make this tree a standalone deployable
hardware workspace. Tests/recipes here document the original layout and require
restoring that packaging and its dependencies before use. Use the physical-robot
repository for commissioning; do not remove the ignore marker expecting that to
restore a complete deployment.

[Original interfaces and deployment reference](docs/ros_interfaces.md) describes
the pre-archive composition. Simulation users should start at the
[workspace overview](../README.md) or [simulation guide](../simulation/humanoid_sim/README.md).
