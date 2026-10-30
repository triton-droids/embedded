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

Use the official `micro_ros_arduino` library for **ROS 2 Humble**, with an
ESP32-S3 build matching the installed Arduino ESP32 core. Install the prepared
library in your Arduino libraries directory before compiling.

Confirm that the library contains `src/esp32s3/libmicroros.a`. If it does not,
generate the ESP32-S3 library using the upstream Humble library builder and its
supported toolchain, or select a Humble release that includes the target.
Do not copy `src/esp32/libmicroros.a` into the ESP32-S3 directory as a workaround.
The Humble firmware library build and flash have not yet been verified on this
board; the successful manual test used the previously flashed firmware.

Then compile with the directory containing that library:

```bash
arduino-cli compile \
  --libraries /path/to/arduino/libraries \
  --fqbn esp32:esp32:esp32s3:FlashSize=8M,CDCOnBoot=default \
  hardware/imu_process/head_mcu/humanoid_vision_microros

arduino-cli upload -p /dev/ttyACM0 \
  --fqbn esp32:esp32:esp32s3:FlashSize=8M,CDCOnBoot=default \
  hardware/imu_process/head_mcu/humanoid_vision_microros
```

## Build the native Humble Agent once

The host uses ROS 2 Humble and a locally compiled Agent by default. The firmware
uses serial transport at **460800 baud**. IMU reception and manual servo commands have been verified with the Humble
Agent and the previously flashed firmware on this Jetson. Rebuilding that
firmware with the Humble library and full visual tracking still need end-to-end
hardware verification.

Open a terminal with system Python (leave Conda or virtual environments first):

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y git build-essential cmake python3-rosdep \
  python3-vcstool python3-colcon-common-extensions

mkdir -p ~/microros_ws/src
cd ~/microros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
# Continue only after the Agent source download succeeds.
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

If rosdep has not been initialized, run `sudo rosdep init` once and retry.
For `vcs: command not found`, install `python3-vcstool`, then rerun
`create_agent_ws.sh` and `build_agent.sh`.

## Identify the serial device and grant access

```bash
lsusb
ls -l /dev/serial/by-id/
sudo usermod -aG dialout "$USER"
```

On this Jetson, the ESP32 serial adapter is QinHeng `1a86:55d3`:
`usb-1a86_USB_Single_Serial_5A4E134473-if00 -> ../../ttyACM0`.
The RealSense D435 is Intel `8086:0b07`, a separate device. Check the port again
after reconnecting USB; its number may change.

After adding the group, log out and back in (reconnect SSH if applicable).
Confirm that `id -nG` includes `dialout`. A password requested by `newgrp` may
be a group password rather than your sudo password; a fresh login avoids this.

## Load the environment in each new terminal

```bash
source /opt/ros/humble/setup.bash
source ~/microros_ws/install/local_setup.bash
source ~/Documents/Github/embedded/install/local_setup.bash
export ROS_DOMAIN_ID=0
```

Source once per terminal. These lines can be added to `~/.bashrc` without
duplicating existing entries. A terminal that only inspects standard messages
needs the Humble setup and the same ROS_DOMAIN_ID.

## Test the MCU and servo independently

In terminal A, start one Agent and leave it running:

```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 -b 460800 -v4
```

Close serial monitors. Press ESP32 RESET/EN if a new session is needed.
In terminal B, run these checks separately; stop each continuous command with
Ctrl-C before starting the next:

```bash
ros2 topic echo /head/mcu_status std_msgs/msg/Int32 --qos-reliability best_effort
ros2 topic hz /head/imu
ros2 topic info /head/servo_target_rad -v
```

Status bits are 1=IMU initialized, 2=PCA9685 initialized, 4=servo output enabled,
and 8=clock synchronized. Status `11` means IMU/PCA9685 initialized and clock
synchronized with PWM off; enabled output normally gives `15`. Initialization
bits do not establish ongoing hardware health. The command topic must have a
`head_mcu` subscriber; a topic name or publisher alone does not prove delivery.

Power the servo independently, connect common ground, and remove the linkage
for the first test. Publish the center command continuously:

```bash
ros2 topic pub -r 20 /head/servo_target_rad std_msgs/msg/Float32 '{data: 0.0}'
```

Stop with Ctrl-C, then test a small angle of about 5.7 degrees:

```bash
ros2 topic pub -r 20 /head/servo_target_rad std_msgs/msg/Float32 '{data: 0.1}'
```

After publication stops, the firmware turns PWM off after approximately 250 ms;
blocking operations can delay watchdog checks. Commands are absolute angles
within -1..1 rad. There is no external encoder feedback.

## Start the complete visual tracking system

Stop the manual publisher and standalone Agent with Ctrl-C before launching.
Build after pulling source changes; skip this if the installed build is current:

```bash
cd ~/Documents/Github/embedded
colcon build --packages-select humanoid_vision --symlink-install
source install/local_setup.bash
```

With the terminal environment loaded, start the camera, detector, single-axis
tracker, and locally compiled Humble Agent:

```bash
ros2 launch humanoid_vision head_tracking.launch.py \
  agent_mode:=native \
  serial_port:=/dev/ttyACM0 \
  color_profile:=640,480,15 \
  target_timeout_s:=0.3 \
  show_image:=false
```

The launch defaults to the native Agent; `agent_mode:=native` makes the choice
explicit. If a standalone Agent is already running, use `start_agent:=false`
instead of starting another. Ctrl-C stops the managed processes.

The current model detects faces. Tracking defaults to one yaw axis. Hold a face
on the left or right of the image for several seconds; the center has an 8 px
deadband. Direction defaults to -1. If the error grows as the servo moves, stop
and restart with `direction:=1.0`. Default command limits are +/-0.6 rad and
0.3 rad/s. Diagnose missing commands before increasing gains.

The source timeout default is 0.15 s. The command above uses 0.3 s to accommodate
15 FPS input and processing delay; this also delays stopping after target loss.
Choose the timeout using measured image age and detection stability.

### Optional launch arguments

| Argument | Purpose |
|---|---|
| `show_image:=true` | Show detection boxes on the Jetson graphical desktop |
| `use_cuda:=true` | Require TensorRT CUDA support and a compatible engine |
| `use_cuda:=false` | Select OpenCV CPU inference |
| `enable_depth:=true` | Enable aligned depth for 3D output |
| `color_profile:=640,480,30` | Request 30 FPS; verify USB bandwidth and actual input rate |
| `axis:=pitch` | Select the pitch axis; verify the physical axis and direction |

CUDA defaults to `auto`, which uses the detector build default. Confirm
`backend: TensorRT CUDA FP16` in startup logs, or inspect the parameter:

```bash
ros2 param get /ball_detection_node use_cuda
```

With 15 FPS input, approximately 14-15 processed FPS is expected even with CUDA.
A detection time near 30 ms/frame is only the per-frame processing time.
Use `lsusb -t` to inspect the D435 connection: 480M indicates USB 2 and 5000M
indicates USB 3. Enabling depth increases bandwidth requirements.

### Vision topics and runtime checks

| Topic | Content |
|---|---|
| `/vision/target_center` | Target center in pixels; tracking input, independent of depth |
| `/vision/ball_3d_position` | 3D target point when aligned depth is valid; currently a face |
| `/head/servo_target_rad` | Absolute angle command sent to the MCU |

3D points use meters in the camera optical frame: X right, Y down, Z forward.
The detector creates the 3D topic, but depth is disabled by default, so no 3D
points are produced. The current implementation uses cached depth without
mandatory RGB/depth timestamp synchronization or a transform to the body frame.

Run checks separately:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /vision/target_center
ros2 topic echo /head/servo_target_rad
ros2 topic info /head/servo_target_rad -v
```

Target publication rate also depends on missed detections. The command topic
should have a `track_node` publisher and a `head_mcu` subscriber, both RELIABLE.

## Troubleshooting

- `errno: 13`: serial permissions are missing. Confirm `dialout` appears in
  `id -nG` after logging in again.
- Serial port busy or repeated reconnections: stop the launch, then inspect
  `sudo fuser -v /dev/ttyACM0`. Confirm any old Agent PID before terminating it
  with `kill -TERM <PID>`. Start only one Agent after the port is released.
- Topics exist without messages: check the Agent session, IMU, and status.
  Subscribe to status with BEST_EFFORT. Temporarily use Agent verbosity `-v6`
  to inspect serial reception and DDS writes; avoid prolonged verbose logging.
- Target commands exist but the servo does not move: inspect command topic
  endpoints. `Subscription count: 0` means no command receiver is visible in
  the ROS graph. Reestablish the MCU/Agent session. Status `11` means output is
  disabled; invalid commands, failed PCA writes, or watchdog expiry can also
  explain it. Check command values and logs before changing visual gains.
- Repeated `Target lost/stale`: inspect detection stability, image timestamps,
  and latency. Target loss intentionally stops commands. A Tracking log only
  establishes tracker activity, not delivery to the MCU.

The serial stream carries binary XRCE-DDS. Do not open a serial monitor or print
application debug text on the same serial port while the Agent is running.
