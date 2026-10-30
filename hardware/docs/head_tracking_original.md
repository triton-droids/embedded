# Archived physical head tracking guide

This guide describes the original physical camera/MCU composition, not the
active simulation launch.

# Single-axis head tracking

`ball_detection_node` selects the highest-confidence box and publishes its
pixel center as `geometry_msgs/msg/PointStamped` on `/vision/target_center`.
`point.x=u`, `point.y=v`, `point.z=0`; the header is copied from the color image.
No message is published when there is no detection. This path does not need
depth. The current detector still uses the face model, so it tracks a face
until a ball model is configured.

`track_node` consumes that center and the color `CameraInfo`. For the selected
axis it computes the angular offset from the image center using the focal
length, applies a P gain to generate angular velocity, limits acceleration
and speed, and integrates the velocity into a target angle. It publishes
`std_msgs/msg/Float32` radians on `/head/servo_target_rad` at 50 Hz. The current
MCU supports only channel 8: use either yaw OR pitch, not two controllers on
the same command topic.

## Build

From the embedded workspace, build the full vision package where OpenCV 4.5
and the detector dependencies are available:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select humanoid_vision --cmake-args -DTRACK_ONLY=OFF
source install/setup.bash
```

For a tracker-only host with no detector SDK, use a separate build/install:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/head_track_logs build --base-paths humanoid_vision \
  --build-base /tmp/head_track_build --install-base /tmp/head_track_install \
  --cmake-args -DTRACK_ONLY=ON
source /tmp/head_track_install/setup.bash
```

## Run

### Unified launch

After building the full package and sourcing its install:

```bash
ros2 launch humanoid_vision head_tracking.launch.py show_image:=true
```

This starts the RealSense color driver, detector, tracker and Docker Agent.
Depth is off by default because visual centering only needs RGB and CameraInfo;
use `enable_depth:=true` to also enable aligned depth/3D output. Ctrl-C stops
the processes started by this launch. An exit of any managed process also
shuts the launch down. Fresh target messages govern tracking readiness, so no
arbitrary startup sleep is needed.

The host needs `realsense2_camera`, the full detector build, and either Docker
access or an installed native Agent. The launch does not install dependencies.
Docker must be accessible without an interactive sudo prompt.

If the existing `head-micro-ros-agent` container is still running, reuse it:

```bash
ros2 launch humanoid_vision head_tracking.launch.py start_agent:=false
```

Alternatively stop it with `docker stop head-micro-ros-agent` before starting
the all-in-one launch. Do not run two Agents on the same USB port. To use an
installed native Agent, set `agent_mode:=native`.

When the camera is already publishing (or runs on another host), also use
`start_camera:=false`. All processes must use the same `ROS_DOMAIN_ID`.
Custom input topics can be passed via `color_topic`, `camera_info_topic` and
`depth_topic`. Tracking controls are exposed directly as launch arguments:

```bash
ros2 launch humanoid_vision head_tracking.launch.py \
  start_camera:=false start_agent:=false show_image:=true \
  axis:=yaw direction:=1.0 max_speed_rad_s:=0.2
```

`start_detector:=false` supports external center publishers and tracker-only
builds; `start_tracker:=false` supports viewing detections without movement.
Check available arguments with `ros2 launch ... --show-args`.

### Separate processes

Start the camera driver, micro-ROS Agent (see [head MCU guide](../imu_process/head_mcu/README.md)), and detector.
The color image dimensions and optical frame must match `CameraInfo`.

```bash
ros2 run humanoid_vision ball_detection_node --ros-args -p show_image:=true
```

In a terminal sourced with the tracker install:

```bash
ros2 run humanoid_vision track_node --ros-args -p axis:=yaw -p direction:=-1.0
```

Start unloaded and verify that the target approaches the image center. If it
moves away, stop with Ctrl-C and reverse `direction` to `1.0`. For an up/down
servo use `axis:=pitch` instead. No second axis is commanded automatically.
The detector display marks the target center and image center with a line.

| Parameter | Default | Meaning |
|---|---:|---|
| `kp` | 1.0 | Angular error to angular velocity gain (1/s) |
| `deadband_px` | 8.0 | No requested velocity inside center deadband |
| `max_speed_rad_s` | 0.3 | Commanded speed limit |
| `max_accel_rad_s2` | 0.8 | Commanded acceleration limit while tracking |
| `min_angle_rad`, `max_angle_rad` | -0.6, 0.6 | Conservative command angle bounds |
| `initial_angle_rad` | 0.0 | Initial COMMAND estimate; not measured joint position |
| `target_timeout_s` | 0.15 | Maximum detection stamp/receipt age |
| `control_rate_hz` | 50.0 | Output refresh rate |
| `target_topic` | `/vision/target_center` | Pixel center input |
| `camera_info_topic` | `/camera/camera/color/camera_info` | Color intrinsics |
| `command_topic` | `/head/servo_target_rad` | MCU angle output |

Clock stamps must share the ROS time base. No valid target/intrinsics means no
servo command. Stale, invalid, or missing targets stop output; the MCU releases
the servo after its additional 250 ms watchdog. This is a release policy, not
a brake/hold policy. Acquiring a target after startup or a manual movement can
move to the stored command angle: without an encoder the tracker cannot infer
the actual starting joint angle. Calibrate the MCU pulse-to-angle mapping and
set `initial_angle_rad` to match the physical starting pose.

Only one publisher should command the MCU. Stop manual `ros2 topic pub` tests
before starting the tracker. Depth and IMU orientation are not used by this
first visual P controller. Target selection follows highest confidence and
can switch between objects; identity association is not implemented yet.

## Integration check without hardware

After sourcing the tracker install:

```bash
/usr/bin/python3 humanoid_vision/algorithm/test/test_track_node.py
# Test the same controller through the unified launch (hardware components off):
/usr/bin/python3 humanoid_vision/algorithm/test/test_track_node.py --launch
```

The test launches a tracker on ROS domain 71 and publishes synthetic camera
intrinsics/centers on `/track_test/*`; no MCU command topic is used.
