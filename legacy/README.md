# Legacy packages and SDK

Retained sources outside the active body-IMU, ONNX/CAN, and head-tracking
composition. Directory relocation preserves ROS package names and interfaces.

| Directory | Package/import | Purpose |
|---|---|---|
| [humanoid_leg_moveit_config](humanoid_leg_moveit_config) | `humanoid_leg_moveit_config` | Leg MoveIt and ros2_control demo. |
| [humanoid_arm_moveit_config](humanoid_arm_moveit_config) | `humanoid_arm_moveit_config` | Arm MoveIt and ros2_control demo. |
| [sdk_prototype](sdk_prototype) | `legacy.sdk_prototype` | Application SDK, gRPC client, browser tools, and examples. |

The retained serial IMU package is now in
[`imu_process/Attitude_Sensing`](<../imu_process/Attitude_Sensing(Depreciated)>), together with
the other IMU sources. Its ROS package name remains `attitude_sensing_pkg`.

The two MoveIt packages contain `COLCON_IGNORE` and are excluded from ordinary
workspace builds. Their sources remain available here. To build them explicitly,
remove the corresponding marker first and use `--cmake-clean-cache` because old
CMake caches may still reference their former source locations. Restore the
markers afterward to exclude them from subsequent builds.

SDK examples now run from `legacy/sdk_prototype/`, and imports use
`from legacy.sdk_prototype.python.robot_sdk import ...`. The former root sample
is retained as `legacy/sdk_prototype/example_use_of_sdk.py`. Relocating these
sources does not validate the old examples against current hardware.

Generated build/install/log directories retained inside moved packages are
historical artifacts, not portable runtime setups. Use the workspace-level
build and install instead.
