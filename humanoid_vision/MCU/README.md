# Head MCU micro-ROS firmware

`humanoid_vision_microros/humanoid_vision_microros.ino` targets the tested
ESP32-S3 head controller. It keeps the IMU and PCA9685 on the shared I2C bus:

- SDA: GPIO4
- SCL: GPIO5
- IMU: `0x68` or `0x69`
- PCA9685: `0x40`, 50 Hz
- servo: ninth physical PCA9685 connector, zero-based channel 8

The servo starts full-off. Invalid angles, loss of the micro-ROS Agent, or 250
ms without a new command also set channel 8 full-off. The DS3225 needs an
independent supply on PCA9685 `V+`; all grounds must be common.

## ROS 2 interface

| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `/head/imu` | `sensor_msgs/msg/Imu` | MCU -> ROS | 100 Hz acceleration and angular velocity in SI units |
| `/head/servo_target_rad` | `std_msgs/msg/Float32` | ROS -> MCU | target joint angle in radians for PCA9685 channel 8 |
| `/head/mcu_status` | `std_msgs/msg/Int32` | MCU -> ROS | bit 0 IMU, bit 1 PCA9685, bit 2 servo enabled, bit 3 clock synchronized |

The IMU does not currently estimate orientation, so
`orientation_covariance[0]` is `-1`. The visual outer loop stays on the Jetson.
The MCU applies angle and pulse limits, converts the requested joint angle to
PWM, and runs the local safety watchdog.

The initial calibration in the sketch is `0 rad = 1500 us`, with `-1..+1 rad`
mapped to `1000..2000 us`. Before installing the linkage, measure the actual
joint and adjust `SERVO_MIN_RAD`, `SERVO_MAX_RAD`, `SERVO_US_PER_RAD`, and
`SERVO_DIRECTION`. This calibration is what makes the commanded ROS angle a
physical joint angle rather than merely a renamed PWM value.

## Build and flash

Use the official `micro_ros_arduino` Jazzy library. The `v2.0.8-jazzy` source
contains the ESP32-S3 build target, but its generated source archive currently
omits `src/esp32s3/libmicroros.a`. Its official generation script builds the
ESP32 and ESP32-S3 entries with the same toolchain/configuration, so prepare
the extracted library once as follows (replace the path with your extraction):

```bash
cd /path/to/micro_ros_arduino-2.0.8-jazzy
mkdir -p src/esp32s3
cp src/esp32/libmicroros.a src/esp32s3/libmicroros.a
```

Then compile with the directory containing that library:

```bash
arduino-cli compile \
  --libraries /path/to/arduino/libraries \
  --fqbn esp32:esp32:esp32s3:FlashSize=8M,CDCOnBoot=default \
  humanoid_vision/MCU/humanoid_vision_microros

arduino-cli upload -p /dev/ttyACM0 \
  --fqbn esp32:esp32:esp32s3:FlashSize=8M,CDCOnBoot=default \
  humanoid_vision/MCU/humanoid_vision_microros
```

The firmware overrides the Arduino library's default serial transport to
460800 baud. Stop any serial monitor, then start a Jazzy Agent at the same
baud rate:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 -b 460800 -v4
```

If this reports `Package 'micro_ros_agent' not found`, the host does not have
the Agent installed. Run the official Jazzy container instead (no local
`micro_ros_agent` ROS package is needed):

```bash
docker run --rm --name head-micro-ros-agent --network host --ipc host \
  --device /dev/ttyACM0:/dev/ttyACM0 \
  --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  microros/micro-ros-agent:jazzy serial \
  --dev /dev/ttyACM0 -b 460800 -v4
```

Use `sudo docker` if your login does not have Docker access. Keep this terminal
running. The ESP32 is the QinHeng `1a86:55d3` USB serial device; the ST-LINK
serial device belongs to the STM32. Check `/dev/serial/by-id/` after reconnecting
USB because `/dev/ttyACM0` numbering can change.

In a second ROS 2 terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /head/mcu_status
ros2 topic hz /head/imu
```

With the head mechanically unloaded, command the zero-radian position at a
refresh rate faster than the 250 ms watchdog:

```bash
ros2 topic pub -r 20 /head/servo_target_rad std_msgs/msg/Float32 '{data: 0.0}'
```

Stop the publisher with Ctrl-C; the MCU releases the output after 250 ms.

Do not open a serial monitor while the Agent owns `/dev/ttyACM0`; micro-ROS
uses a binary XRCE-DDS stream rather than printable JSON.
