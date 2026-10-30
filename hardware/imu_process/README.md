# IMU acquisition and processing

This directory groups body/head firmware, shared processing, acquisition and
acceptance tools. The older serial ROS package is retained under `legacy/`;
ROS topic names and firmware protocols are unchanged.

| Path | Purpose |
|---|---|
| [policy_imu_microros](policy_imu/policy_imu_microros/README.md) | Body BNO085 micro-ROS firmware, math helpers/tests, and read-only ROS acceptance probe. Publishes `/imu/data_raw` and `/imu/status`. |
| [head_mcu](head_mcu/README.md) | Combined head IMU and PCA9685 servo firmware, moved intact from the vision package. Publishes `/head/imu` and `/head/mcu_status`; consumes `/head/servo_target_rad`. |
| [Serial ROS reader](../attitude_sensing_pkg) | Retained JSON/BNO085 CSV reader and optional Madgwick launch. ROS package: `attitude_sensing_pkg`. |
| [fall_detector.py](fall_detector.py) | Reusable IMU fall detector and compatibility loading of serial helpers, used by the RobStride gain tuner. It does not open hardware on import. |
| [policy_imu](policy_imu/README.md) | Serial policy-IMU firmware. |
| [imu_read_Wifi](imu_read_Wifi) | Older IMU firmware and UDP receiver. |
| [main_serial.py](main_serial.py) | Standalone serial fusion entry point. |
| [fusion.py](fusion.py), [quat.py](quat.py), [math3d.py](math3d.py) | Standalone fusion, quaternion, and vector helpers. |
| [can_read_imu.py](can_read_imu.py) | Standalone CAN IMU reader. |

Use `from imu_process.fall_detector import IMUFallDetector` for the shared
fall detector. The gain tuner imports this implementation directly; no duplicate
IMU implementation is kept in `hardware/utils/`.

The body micro-ROS producer and serial ROS reader are alternatives for an IMU
topic. The head MCU combines sensing and actuation in one sketch, so its whole
sketch and guide live here. Host visual control remains in `humanoid_vision`.
Safety/policy consumers and their package integration tests stay in their ROS
packages; their subscriptions are not additional IMU acquisition implementations.

## Serial ROS package

From the repository root, the policy setup script includes the relocated source:

```bash
ROS_DISTRO=jazzy bash scripts/setup_tracking_policy.sh
```

For an existing workspace, rebuild the moved Python package to refresh installed
source links:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select attitude_sensing_pkg
source install/setup.bash
```

## Standalone serial tools

The existing standalone fusion scripts use sibling-module imports. Run them by
file path from the repository root, for example:

```bash
python3 hardware/imu_process/main_serial.py --help
```

See the firmware-specific guides for wiring, calibration, dependencies, and
build/flash commands. Directory relocation is not hardware validation.
