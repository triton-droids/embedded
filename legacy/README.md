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

Colcon discovers the MoveIt packages recursively; there is no blanket
`COLCON_IGNORE` here. Build and launch from the repository root. Existing
MoveIt CMake caches may contain former source locations; rebuild with:

```bash
colcon build --symlink-install \
  --packages-select humanoid_leg_moveit_config humanoid_arm_moveit_config \
  --cmake-clean-cache
source install/setup.bash
```

SDK examples now run from `legacy/sdk_prototype/`, and imports use
`from legacy.sdk_prototype.python.robot_sdk import ...`. The former root sample
is retained as `legacy/sdk_prototype/example_use_of_sdk.py`. Relocating these
sources does not validate the old examples against current hardware.

Generated build/install/log directories retained inside moved packages are
historical artifacts, not portable runtime setups. Use the workspace-level
build and install instead.
