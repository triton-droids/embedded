# humanoid_localization

Base pose without simulator ground truth: body IMU attitude (BNO085 game
rotation vector) plus stereo visual odometry from cuVSLAM (Isaac ROS 3.2, Jetson
GPU). Navigation consumers (A*) read `/localization/odom`; the walking
controller's WBC still uses ground truth.

```
mujoco_sim (enable_stereo) ─ shared state ─ sim_stereo_publisher ── /stereo/{left,right}/{image_raw,camera_info},
        │                                                          TF floating_base→stereo_rig
        │                                   │
        │                        cuVSLAM (container) ── /visual_slam/tracking/odometry
        │                                   │
        └── /imu/data_raw ──────── vio_localizer ── /localization/odom (world → floating_base)
```

## Sensors and the competition rules

The simulated pair stands in for the D435 infrared imagers with the projector
off: 50 mm baseline, 58° vertical field of view, 640×480 grayscale, 15 Hz,
ideal rectified pinholes. With `stereo_split` (default) the pair is rendered by
`sim_stereo_publisher` from the kinematic state mujoco_sim shares after every
physics step, so rendering never holds the simulator's GIL. RoboCup's 2026 sensor list allows stereo cameras and
IMUs without a compass; the sponsored-robot guidelines require active camera
components (such as a RealSense projector) to be covered. The BNO085 is used in
game-rotation-vector mode (no magnetometer), which is what `/imu/data_raw`
simulates: yaw is relative to power-on. `imu_yaw_error_deg` /
`imu_yaw_error_period_s` add a sinusoidal heading error to the simulated
attitude (off by default).

## Method

- Orientation: IMU quaternion, with a one-time yaw offset so the start heading
  equals `initial_yaw`.
- Position: each VO step is converted to a base displacement with the base→rig
  transform at both stamps (the camera moves with the head), rotated into
  `world` with the IMU attitude, and added. Steps implying more than
  `max_speed_m_s` are rejected; a VO gap longer than 0.5 s restarts the
  increment chain without moving the estimate.
- cuVSLAM runs as pure stereo VO (`enable_imu_fusion: false`): its IMU must be
  rigid with the cameras, and the body IMU is not (head pan/tilt). It publishes
  no TF; `vio_localizer` owns the pose.
- The start pose (`initial_x/y/z/yaw`) is a known placement, like a kickoff
  position. In the ball-chase scene it is the spawn pose (0, 0, yaw 0).

## Run (Jetson)

```bash
# once: Isaac ROS base image, then the derived image
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git ~/isaac_ros_ws/src/isaac_ros_common
TERM=xterm ~/isaac_ros_ws/src/isaac_ros_common/scripts/build_image_layers.sh \
  --image_key aarch64.ros2_humble --image_name isaac_ros_dev-aarch64
scripts/run_cuvslam.sh build

# terminal 1: simulation with stereo and the localizer (textured scene)
scripts/ball_chase_walk.sh scene:=hard enable_vio:=true enable_astar:=true
# terminal 2: cuVSLAM
scripts/run_cuvslam.sh
# terminal 3: error against ground truth (evaluation only)
ros2 run humanoid_localization localization_eval --ros-args -p duration_s:=120.0
```

For a long route with turns, `ros2 run humanoid_localization route_driver`
(simulation only) keeps the ball out of view until `/localization/status`
reports `initialized`, then places it 1.3 m ahead and 35° to the left of the
robot each time the robot reaches it (16 legs by default).

Terminals 2 and 3 need the same `ROS_DOMAIN_ID` as the simulation
(`ball_chase_walk.sh` uses 0 by default) and terminal 3 the same DDS settings
(see `scripts/ball_chase_walk.sh`).

## Limits

- The base→rig TF comes from the simulated camera pose; hardware needs it from
  head joint forward kinematics.
- No VO tracking status is consumed (the Isaac status message type is not
  installed on the host); bad tracking is caught only by the speed gate.
- Yaw is the IMU's; VO yaw is not fused, so game-rotation-vector yaw drift
  becomes heading drift. Global correction (field lines, goals) is separate work.
- Measured on the Jetson (hard scene, 16-leg route, ~14 m, IMU heading error
  ±3°/60 s): final position error 1.6–2.6 % of the distance, mostly from VO jumps
  while turning towards the untextured corner of the field.
- The simulated scene is far less textured than a real venue; VO results here
  do not predict results on real images.
