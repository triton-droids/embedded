# Vision, Head Control, and Ball-Chase Plan

## Objective

Build a RoboCup perception and control pipeline that keeps the camera pointed
at the ball, transforms the measured ball position into the robot body frame,
commands the existing velocity-conditioned walking policy, approaches a safe
kick pose, and stops safely whenever perception becomes stale.

The first implementation should use conventional feedback control for gaze and
chase commands, the existing RL policy for balanced walking, and a validated
fixed kick motion. Head MPC and more advanced kick control should be added only
after the basic interfaces are measured and stable.

## Target Architecture

```text
RealSense color + aligned depth
              │
              ▼
       YOLO ball detector
              │ PointStamped in camera optical frame
              ▼
        TF2 to base_link
              │
       ┌──────┴────────┐
       ▼               ▼
head gaze control   ball chase control
 yaw/pitch command     /cmd_vel
       │               │
       ▼               ▼
 head motors       RL walking policy
                       │
                       ▼
             /desired_motor_subset
```

The detector must remain independent of locomotion. It publishes a target
measurement; a separate controller owns robot motion and stale-data handling.

## Planned ROS Interfaces

| Topic | Type | Producer | Consumer |
| --- | --- | --- | --- |
| `/vision/ball_position_camera` | `geometry_msgs/PointStamped` | detector | TF/fusion |
| `/vision/ball_position` | `geometry_msgs/PointStamped` | TF/fusion | chase controller |
| `/cmd_vel` | `geometry_msgs/Twist` | chase controller | policy bridge |
| `/joint_states` | `sensor_msgs/JointState` | motor feedback | TF and controllers |
| `/imu/body` | `sensor_msgs/Imu` | torso IMU | estimator/head controller |
| `/imu/head` | `sensor_msgs/Imu` | camera assembly IMU | head controller |
| `/head_controller/commands` | controller-specific | gaze controller | yaw/pitch motors |
| `/desired_motor_subset` | `MotorCommand` | walking policy | motor scheduler |

Topic names may be changed during integration, but frames and ownership must
remain explicit.

## Required TF Model

Create a URDF/Xacro model with:

```text
base_link
├── body_imu_link
└── neck_yaw_link
    └── neck_pitch_link
        └── camera_link
            ├── head_imu_link
            └── camera_color_optical_frame
```

Tasks:

- Measure the yaw-axis origin relative to `base_link`.
- Measure the pitch-axis origin relative to the yaw link.
- Measure the camera pose relative to the pitch link.
- Calibrate yaw and pitch encoder zeros and signs.
- Publish actual measured yaw/pitch positions in `/joint_states`.
- Reuse the official RealSense optical-frame model where possible.
- Transform each ball measurement using its own timestamp, not the latest TF.
- Add a TF test in which known optical-frame points map to expected body-frame
  directions.

After conversion to `base_link`, use the standard axes:

```text
X forward, Y left, Z up
heading = atan2(Y, X)
```

Do not manually add the head yaw after TF conversion; that would compensate the
same motion twice.

## Head Hardware and Sensing

Use at least two active head joints:

```text
neck_yaw:   horizontal ball tracking
neck_pitch: vertical tracking and near-foot viewing
```

Use two IMUs with different roles:

- The body IMU measures gait-induced torso motion for disturbance feed-forward.
- The head IMU is rigidly mounted with the camera and measures residual camera
  angular motion for feedback.
- Joint encoders provide head angle, velocity, and mechanical-limit state.
- Vision provides the slower line-of-sight correction to the ball.

The camera and head IMU must be rigidly connected. If vibration isolation is
used, isolate the complete camera-and-IMU assembly rather than the IMU alone.

Initial rate targets:

```text
IMUs:             100-200 Hz or faster
head joint state: 100 Hz or faster
head control:      50-100 Hz
vision:            13 Hz minimum, 30 Hz target
body chase:        20-50 Hz
walking policy:    existing 50 Hz loop
```

## Head-Control Development

### Stage 1: Position and Limit Validation

- Command small yaw and pitch motions independently.
- Confirm encoder sign, zero, range, maximum speed, and emergency stop.
- Confirm that TF moves in the same direction as the physical camera.
- Validate cable clearance through the full permitted range.

### Stage 2: IMU Feed-Forward and PID

Implement a high-rate controller before MPC:

```text
head command = gaze tracking correction
             + body angular-velocity feed-forward
             + head angular-velocity feedback
```

Use gyro measurements for fast compensation. Use visual error to correct slow
drift. Do not depend on magnetometer yaw near motors.

### Stage 3: Head MPC

After actuator latency and limits are measured, formulate MPC with a state such
as:

```text
[yaw, pitch, yaw_rate, pitch_rate, image_error_u, image_error_v]
```

Inputs are yaw and pitch motor commands. Treat torso angular velocity as a
measured disturbance. Penalize:

- image error;
- residual camera angular velocity;
- head velocity and acceleration;
- control effort and command-rate changes.

Constrain joint position, velocity, acceleration, torque, and cable-safe range.
A 100-300 ms prediction horizon is a reasonable starting point to test, not a
final requirement.

The head should track quickly while the body follows more slowly. When head yaw
passes a configurable threshold, initially around 20 degrees, command the body
to turn in the same direction so the head can return toward its center.

MPC can compensate rotational disturbance but cannot remove image motion caused
by vertical translation, structural vibration, rolling shutter, or long
exposure. Camera exposure, lighting, mechanical stiffness, and backlash remain
part of the stabilization problem.

## Ball Chase Control

Add a dedicated `ball_chase_controller_node` that subscribes to the ball point
in `base_link` and publishes `/cmd_vel`.

Initial control law:

```text
distance = hypot(ball_x, ball_y)
heading  = atan2(ball_y, ball_x)
yaw_rate = clamp(K_heading * heading)
forward  = clamp(K_distance * (distance - stop_distance))
```

Do not move forward until heading error is within a configured gate. Reduce
speed as the robot approaches the ball. If a measurement is older than
0.2-0.3 seconds, publish a zero command immediately.

Update `policy_bridge_node` to subscribe to `/cmd_vel` and copy:

```text
[linear.x, linear.y, angular.z]
```

into the existing velocity-conditioned policy observation. Keep the safety
supervisor independent of both vision and policy output.

## Behaviour State Machine

Implement these states explicitly:

```text
SEARCH
  Head scans for a ball; body remains stopped initially.

TRACK
  Head keeps the ball near the image center.

APPROACH
  Body turns and walks while the head continues tracking.

ALIGN_KICK
  Robot moves the ball to a measured pose relative to the selected kick foot.

KICK
  Walking stops and a validated kick motion executes.

RECOVER
  Robot returns to a stable standing/walking state.
```

Foot position does not need to be visible in the image. Obtain foot poses from
joint feedback and forward kinematics, then compare the ball and foot in a
common frame.

Begin with a fixed, verified kick trajectory. MPC may later manage centre of
mass, contact, and single-support constraints. OMPL is optional for geometric
collision-free planning or obstacle routing; it is not a replacement for
dynamic walking or balance control.

## Camera Strategy

Start with one moving head camera if its pitch range and field of view keep the
ball visible at the kick pose.

Add a second downward-looking camera only if physical testing shows a near-foot
blind region or self-occlusion. If added, publish separate measurements:

```text
/vision/head/ball_position
/vision/down/ball_position
```

Transform both to `base_link`, reject stale data, and select measurements using
freshness, confidence, range, and hysteresis. Do not average points from
different timestamps blindly.

## Perception Work

- Replace the temporary face ONNX model with a licensed ball model.
- Build the C++ runtime against an OpenCV version that loads the selected ONNX
  model.
- Enable CUDA only after the linked OpenCV reports CUDA and cuDNN support.
- Move inference out of the subscription callback so new frames replace stale
  queued frames rather than accumulating latency.
- Synchronize color, aligned depth, and camera information.
- Make the output topic and target label configurable.
- Publish detection confidence and validity information in a dedicated message
  if `PointStamped` becomes insufficient.
- Add target filtering and tracking after raw timestamped output is verified.

## Milestones and Acceptance Checks

### M1: Perception on a Static Head

- Ball model loads in the C++ node.
- Color and aligned depth are near 30 Hz, or their lower measured rate is
  documented.
- Stationary-ball 3D output is stable and in metres.
- Invalid depth produces no false zero-distance target.

### M2: Dynamic TF

- Real head encoder feedback drives yaw/pitch TF.
- A stationary world target remains approximately stationary in `base_link`
  while the head moves.
- Message timestamps and TF lookup do not produce systematic direction errors.

### M3: Head Tracking

- Head tracks a slowly moving ball without reaching limits.
- Loss of detection transitions to hold or scan safely.
- Body-follow logic recentres head yaw.

### M4: Gaze Stabilization

- Body-IMU feed-forward reduces image angular motion during a controlled gait.
- Head-IMU feedback reduces the remaining disturbance.
- PID baseline is recorded before MPC is evaluated.

### M5: Chase Without Kicking

- `/cmd_vel` reaches the RL policy at runtime.
- The robot turns before walking when the heading error is large.
- The robot stops at a conservative distance and stops on stale vision.
- Tests begin with supported/tethered hardware and low velocity limits.

### M6: Kick Alignment and Motion

- Ball and both feet are expressed in a common frame.
- The robot reaches a repeatable pre-kick pose.
- A separately validated kick executes only from a stable state.
- Recovery and failure transitions are tested before autonomous play.

## External Designs to Reuse Selectively

- [`adityakamath/pantilt_ros2`](https://github.com/adityakamath/pantilt_ros2):
  ROS 2 pan/tilt URDF, `ros2_control`, hardware, and MuJoCo structure.
- [`realsenseai/realsense-ros`](https://github.com/realsenseai/realsense-ros):
  official RealSense description and optical frames.
- [`BoosterRobotics/robocup_demo`](https://github.com/BoosterRobotics/robocup_demo):
  ball search, head tracking, chase, and kick behaviour examples.
- [`kingston23/pan_tilt`](https://github.com/kingston23/pan_tilt): simple ROS
  2/Gazebo RealSense pan/tilt example.
- Roncone et al., [*Gaze Stabilization for Humanoid Robots*](https://arxiv.org/abs/1411.3525):
  anticipatory self-motion compensation plus gyro feedback.

These projects are references rather than drop-in solutions. None currently
provides this repository's exact RealSense, dual-IMU, head-MPC, RL-walking, and
motor-interface combination.
