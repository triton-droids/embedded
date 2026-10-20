# Body policy IMU over micro-ROS

This separate ESP32-S3 sketch provides raw six-axis IMU data for the body policy.
It has **no servo or motor commands**. It does not replace the head module firmware.
The body sensor must be rigidly attached to the torso; configure and verify the
host's `imu_to_body_rotation` for its actual mount. GPIO4/5 and supported MPU
WHO_AM_I values follow the existing serial policy sketch; confirm the actual
board/sensor before flashing. This revision has not been flashed or hardware-tested.

- I2C SDA4/SCL5, 100 kHz, address 0x68/0x69; serial micro-XRCE-DDS at 460800 baud.
- `/imu/data_raw`, best-effort, requested 200 Hz, frame `imu_link`.
- SI acceleration m/s² and angular velocity rad/s; orientation unavailable.
- Agent time sync before publication; retry every 5 s with a bounded 10 ms wait.
  Failed sync immediately stops IMU publication; sync older than 15 s is invalid.
  Reconnection destroys/recreates entities and obtains a fresh clock offset.
- Stamp the synchronized epoch **at register-read start**, before I2C/serial/DDS.
  This is a host-comparable read timestamp, not a measured sensor data-ready edge.
  IMU internal filtering, clock error and sample-edge alignment remain unmeasured.
- Ping and sync can skip sensor periods. No catch-up or fake fresh timestamps.
  Run Agent and policy on the same host; otherwise synchronize host clocks first.
  This path uses wall/ROS time, not paused simulation time.

With an already installed compatible Arduino ESP32-S3 core and micro_ros_arduino
library (use the library/Agent versions validated for your ROS distribution):

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3:FlashSize=8M,CDCOnBoot=default \
  --libraries /path/to/arduino/libraries imu_process/policy_imu_microros
```

No dependencies are installed by this change. After separately identifying the
MCU and deploying the firmware, start the existing Agent using its exact device:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/ACTUAL_DEVICE -b 460800
ros2 launch motor_control_hybrid tracking_policy.launch.py
```

The launch defaults to consuming the externally published `/imu/data_raw`.
For legacy JSON firmware explicitly pass `start_imu_reader:=true`. Never let the
Agent and JSON reader open the same device. Neither command above enables motors.

Before hardware control, measure source age, IMU/joint timestamp skew, dropped
samples, Agent disconnect/reconnect, and end-to-end latency under full load. The
host's default 20 ms skew limit is a software acceptance threshold, not measured
synchronization accuracy. CAN feedback still uses host reception timestamps.

Clock API reference: [official micro-ROS time synchronization declarations](https://github.com/micro-ROS/micro_ros_arduino/blob/kilted/src/rmw_microros/time_sync.h).
