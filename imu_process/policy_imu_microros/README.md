# ESP32-S3 + BNO085 body IMU over micro-ROS

This sketch reuses the existing serial micro-ROS transport, Agent discovery,
clock synchronization, and reconnect lifecycle. It publishes body IMU telemetry
only: there are no motor/servo outputs or command subscriptions. The independent
head IMU firmware is outside this change.

## Wiring and report contract

| BNO085 breakout | ESP32-S3 |
| --- | --- |
| SDA | GPIO5 |
| SCL | GPIO4 |
| INT (active low) | GPIO6 |
| RST (active low) | GPIO7 |
| GND | GND |
| Power input | 3.3 V, subject to the breakout's specified input pin |

Select I2C mode (PS0=low, PS1=low); breakout defaults/pull resistors vary. The
sketch tries 0x4A and 0x4B, uses 400 kHz I2C with a 512-byte receive buffer and a
10 ms transaction timeout. USB UART micro-XRCE-DDS runs at 460800 baud. Do not
open a serial monitor or print text on the binary micro-ROS transport.

`/imu/data_raw` is `sensor_msgs/msg/Imu`, Reliable/Volatile, requested 100 Hz.
All three BNO085 reports request 100 Hz; output is limited to 100 Hz and may be
lower due to report phase, synchronization, I2C/serial load or reliable ACK waits.
The reliable publisher session wait is bounded to 10 ms. Measure actual output.

| ROS field | BNO085 SH-2 report | Meaning |
| --- | --- | --- |
| orientation | GAME_ROTATION_VECTOR | Fused sensor-to-world quaternion; relative yaw |
| angular_velocity | GYROSCOPE_CALIBRATED | Measured gyro XYZ in rad/s |
| linear_acceleration | ACCELEROMETER | Measured specific force in m/s², including gravity |

These are SI sensor reports, not raw ADC counts. No host attitude differentiation
is used for angular velocity. Do not replace ACCELEROMETER with LINEAR_ACCELERATION:
Tracking still requires approximately 1g acceleration at rest, even in message
orientation mode. Game RV supplies gravity/tilt without magnetometer heading
corrections. Yaw can drift; this is not a compass or absolute heading source.
Orientation covariance is a configured placeholder (roll/pitch 0.001 rad²,
yaw 1e6 rad²), not a measured uncertainty. Gyro/acceleration covariance is unknown.

`bno085_sh2.h` uses CEVA's SH-2 callbacks bundled with the SparkFun library,
so every report in a mixed SHTP packet is decoded. It provides its own bounded,
non-logging Wire HAL. GPIO6 falling edges timestamp host receipt; SH-2 applies
report timing metadata. No application or library debug text shares Serial.

### Mounting

All published fields use `frame_id=imu_link`, interpreted as a body-aligned frame.
Set `BODY_FROM_SENSOR` in `bno085_sh2.h` to the measured mounting quaternion,
ordered w,x,y,z. It rotates sensor vectors into the body frame. Orientation is
transformed consistently using q_WB=q_WS*inverse(q_BS). The default identity is
only a bench assumption; it is not physical mounting calibration. Keep Tracking's
`imu_to_body_rotation` at identity when the publisher compensates mounting.

At rest with the body upright, expect acceleration approximately [0,0,+9.80665],
angular velocity near zero, roll/pitch near zero, and projected down [0,0,-1].
Check positive body X/Y/Z rotations physically before attaching to a robot.
The firmware does not tare tilt at startup, so a tilted boot is still tilted.

### Freshness and recovery

A message requires a new report from each of quaternion, acceleration and gyro.
Every component must be <=30 ms old and their timestamps must differ by <=20 ms.
The message stamp uses the oldest component's SH-2 reconstructed host sample time,
converted into synchronized Agent epoch time. Future, stale, duplicate or invalid
components stop publication; old values are not re-stamped as fresh measurements.
The GPIO interrupt/report reconstruction does not measure all internal fusion
latency or prove synchronization with physical CAN sample edges.

Clock synchronization is required before IMU publication, retried every 5 s.
A failed sync immediately suspends publication, as does sync age above 15 s.
The Agent and policy should run on the same host with real wall/ROS time.
Sensor resets discard cached fields and re-enable reports. A one-second event
outage or report setup failure retries sensor initialization after two seconds.
Sensor recovery may exceed control watchdog limits: Tracking faults stay latched
and require restart; Safety estop requires a healthy input and explicit reset.

`/imu/status` (`std_msgs/msg/String`, JSON, 1 Hz) reports I2C address, report setup,
clock state, publication count/failures and each component's age. Age -1 means
no valid component yet. It remains available when IMU output is withheld.

## Build and flash

Validated tool selection for this implementation: Arduino ESP32 core 2.0.17,
micro_ros_arduino v2.0.8-jazzy, SparkFun BNO08x Cortex Based IMU 1.0.6.
Use an Agent matching the micro-ROS distribution. The precompiled archive has
32-bit time_t/timespec; Arduino ESP32 core 3.3.6 has a 64-bit time ABI. That
combination compiled but live testing yielded 1 Hz output and future stamps.
The sketch now rejects that ABI at compile time; use the pinned 2.0.17 core or
rebuild micro-ROS for a different toolchain before changing the ABI guard. The connected board was
identified as ESP32-S3 revision 0.2, 16 MB Flash, 8 MB embedded PSRAM; the sketch
uses no PSRAM. CDCOnBoot=default keeps Serial on UART for its USB serial bridge.
Other ESP32-S3 boards with native USB may need a different CDC configuration.

Use an isolated Arduino CLI data directory containing core 2.0.17. Do not let
CLI select a newer globally installed core. This session's tested configuration
is `/tmp/body-imu-tools/arduino-cli2.yaml`; CLI is
`/tmp/body-imu-tools/arduino-cli`, libraries are `/tmp/body-imu-libraries`, and the
compatible build directory is `/tmp/body-imu-build2`. Temporary files may be
removed by the OS. Final firmware and validation logs are also saved under the
repository's ignored `build/body_imu_firmware/` directory.

Keep downloaded libraries outside tracked source. The Jazzy source archive omits
src/esp32s3/libmicroros.a even though its generation script defines ESP32-S3 using
the same esp32_toolchain.cmake and colcon.meta as ESP32. For this exact Jazzy
archive, the tested build placed its esp32 archive in src/esp32s3. This is specific
to that release/build setup; do not assume it applies to other distributions or
cores. Rebuild with the upstream library generator when those inputs differ.

```bash
# Libraries directory contains micro_ros_arduino and SparkFun_BNO08x libraries.
arduino-cli --config-file /path/to/isolated-core2-cli.yaml compile \
  --fqbn esp32:esp32:esp32s3:FlashSize=16M,CDCOnBoot=default \
  --libraries /path/to/arduino/libraries \
  --build-path /tmp/body-imu-build imu_process/policy_imu_microros

# Confirm port identity and close any Agent before uploading.
arduino-cli --config-file /path/to/isolated-core2-cli.yaml upload \
  --fqbn esp32:esp32:esp32s3:FlashSize=16M,CDCOnBoot=default \
  --port /dev/serial/by-id/ACTUAL_DEVICE --input-dir /tmp/body-imu-build
```

## Test without motor output

Run from the repository root. Never let the JSON reader and Agent open the same
port. Use the actual serial path (the identified bridge has by-id suffix
USB_Single_Serial_5AE8013110-if00). Close an existing test Agent before restarting.

Terminal 1, start the existing Jazzy Agent image:

```bash
docker run --rm --name body-imu-agent --network host \
  --device=/dev/ttyACM0:/dev/ttyACM0 \
  microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0 -b 460800
```

Terminal 2, inspect endpoints and collect 15 seconds of data. Arbitrary sensor
placement is allowed; upright pose and mounting calibration are not required:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic info /imu/data_raw --verbose
ros2 topic echo /imu/status --once
python3 imu_process/policy_imu_microros/check_body_imu.py
```

The probe deliberately subscribes Reliable, matching Safety. It checks finite
values, frame, normalized available orientation, increasing stamps, source age
in [-10,100] ms, receipt gaps <=100 ms and output rate >=50 Hz. Upright testing
also checks 1g force, low gyro, roll/pitch, and quaternion/acceleration agreement.
The optional --stationary-upright mode is for a later, physically aligned mount;
it is not part of this arbitrary-placement test. A pass is a bench input contract
check, not proof of full-load robot timing or
physical mounting calibration. Without --stationary-upright, gently tilt and
rotate while watching the printed roll/pitch and gyro axes. Tilt boot/reboot
must not silently zero roll/pitch.

Terminal 3, build the existing package and test Tracking in quaternion mode:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths humanoid_control --packages-up-to motor_control_hybrid
source install/setup.bash
# Use your existing policy Python environment. This session's temporary deps:
export PYTHONPATH="/tmp/body-imu-policy-python${PYTHONPATH:+:$PYTHONPATH}"
ros2 launch motor_control_hybrid tracking_policy.launch.py \
  start_imu_reader:=false orientation_source:=message \
  joint_feedback_mode:=zero calibration_seconds:=2.0
```

Keep it still during the two-second gyro calibration. Check `/policy/status`:
there should be no fault and receipt age should remain below 100 ms. This launch
only publishes `/policy/*`; C++ bench control, CAN and motor enable are not started.
The launch's default orientation_source remains complementary for old callers;
explicitly pass message for BNO085 fused orientation.

For placement-independent Safety timeout testing, keep all motor/CAN nodes
stopped and disable only fall detection on isolated test output topics:

```bash
ros2 run humanoid_safety safety_node --ros-args \
  -p fall_detection_enabled:=false \
  -p estop_topic:=/imu_test/estop -p status_topic:=/imu_test/safety_status \
  -r /safety/reset:=/imu_test/safety_reset
ros2 topic echo /imu_test/estop
```

With fresh samples the timeout test should remain false, independent of sensor
pose. For a later fall-detection test, enable fall detection explicitly: holding
roll or pitch beyond 40 degrees for three received samples latches true; invalid
orientation also trips. This threshold is robot policy, not an IMU data validity
check. Reset a timeout-only test latch after fresh data resumes:

```bash
ros2 service call /imu_test/safety_reset std_srvs/srv/Trigger '{}'
```

To test outages, stop only the Agent (`docker stop body-imu-agent`). Tracking
should latch an IMU timeout (~100 ms); Safety should latch after ~250 ms. Restart
the Agent: MCU should reconnect and resume fresh telemetry, but Tracking still
requires restart and Safety still requires reset. These tests must never be run
with enabled actuators. Do not run unrelated hybrid/CAN launch files for IMU tests.

Host-only math check:

```bash
g++ -std=c++17 -Wall -Wextra -Werror \
  imu_process/policy_imu_microros/test/body_imu_math_test.cpp -o /tmp/body-imu-math-test
/tmp/body-imu-math-test
```

References: [SH-2 HAL contract](https://github.com/ceva-dsp/sh2/blob/master/sh2_hal.h),
[SparkFun driver](https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library),
[micro-ROS Jazzy library generation](https://github.com/micro-ROS/micro_ros_arduino/blob/v2.0.8-jazzy/extras/library_generation/library_generation.sh).

## Bench validation from this change

The final compatible build compiled and flashed successfully; esptool verified
flash hashes. A 15-second Reliable ROS probe received 817 messages at 55.04 Hz,
with zero invalid or non-increasing stamps, mean source age 20.17 ms, maximum
32.70 ms, and maximum receipt gap 31.62 ms. These are measured bench figures,
not a claim of 100 Hz or full-load real-time guarantees. Sensor placement was
arbitrary; no upright check, pose zeroing or mounting compensation was applied.
At rest, quaternion-predicted up and measured acceleration differed by 0.20°
on average, checking mutual consistency without assuming a particular pose.

The existing Tracking node ran with orientation_source=message and zero joint
feedback: 617 target-angle messages at 50.00 Hz, no fault, no rejected IMU
stamps, no 20 ms overruns. Outputs were monitor topics only; reference motion
ends by holding its last frame, so this is an input/inference test, not walking.

A separate Safety test disabled fall detection and remapped estop/status/reset
to /imu_test/*: fresh input was healthy; stopping the Agent latched an IMU timeout
at 0.253 s against the 0.250 s limit. Restarting the Agent resumed samples while
the Safety latch stayed set. Test nodes were stopped; the IMU-only Agent was
left running as body-imu-agent. No CAN or motor nodes were started.

The source/firmware, compile/upload logs, raw probe results and binary hashes
are recorded in ignored build/body_imu_firmware/. Existing control-node source
was not changed; the Tracking launch only exposes its existing orientation_source
parameter. Hardware mounting and actuator operation remain outside this test.
