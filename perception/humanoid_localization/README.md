# humanoid_localization

Base pose without simulator ground truth: body IMU attitude (BNO085 game
rotation vector) plus stereo-inertial visual odometry from cuVSLAM (Isaac ROS
4.6, JetPack 7.2.x, Jetson GPU) with the camera-carrier IMU (revL head, MPU9250). Navigation consumers (A*) read `/localization/odom`; the walking
controller's WBC still uses ground truth.

```
mujoco_sim (enable_stereo) ─ shared state ─ sim_stereo_publisher ── /stereo/{left,right}/{image_raw,camera_info},
        │                                                          TF floating_base→stereo_rig, stereo_rig→head_imu_link
        ├── IMU snapshot ─ sim_imu_publisher ── /head/imu (200 Hz) ─┐
        │                                   │                       │
        │                        cuVSLAM (container) ───────────────┘── /visual_slam/tracking/odometry
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

The camera-carrier IMU (`/head/imu`, frame `head_imu_link`) is an MPU9250 on
the revL head, on the same rigid body as the stereo pair, with its
magnetometer unused. In simulation it is sampled at every 5 ms physics
step (200 Hz) and published from `sim_imu_publisher`, a separate process, so
simulator GIL stalls cannot leave gaps in it. That process wakes at 100 Hz and
publishes every sample since its last wake-up from a ring in the shared IMU
segment. Waking at twice the sample rate instead cost about 45 % of an Orin
Nano core. Its white noise uses the datasheet
densities. It also has a constant gyro bias but no bias random walk
(`simulation/humanoid_ball_chase/config/scene*.yaml`, `head_imu_*`
parameters). The axes are those of the `head_imu` site (the camera carrier's
CAD axes). They are not the physical MPU9250 axes marked in the CAD
(`imu_mount.neutral_sensor_axes`), so on hardware the URDF's
`stereo_rig → head_imu_link` must carry the real mounting rotation.

## Method

- Orientation: IMU quaternion, with a one-time yaw offset so the start heading
  equals `initial_yaw`.
- Position: each VO step is converted to a base displacement with the base→rig
  transform at both stamps (the camera moves with the head), rotated into
  `world` with the IMU attitude, and added. Steps implying more than
  `max_speed_m_s` are rejected; a VO gap longer than 0.5 s restarts the
  increment chain without moving the estimate.
- cuVSLAM fuses the camera-carrier IMU (`tracking_mode: 1`, `imu_frame:
  head_imu_link`, MPU9250 noise densities, `calibration_frequency: 200`). The
  body IMU cannot be used: it is not rigid with the cameras (head pan/tilt).
  The extrinsic is the static TF `stereo_rig → head_imu_link`, computed from
  the model by the stereo publisher. `imu_fusion:=false` (on
  `run_cuvslam.sh` / `run_isaac_perception.sh`) gives the earlier pure stereo VO.
  cuVSLAM publishes no TF; `vio_localizer` owns the pose and still takes its
  attitude from the body IMU.
- The start pose (`initial_x/y/z/yaw`) is a known placement, like a kickoff
  position. In the ball-chase scene it is the spawn pose (0, 0, yaw 0).

## Deployment target and host migration

Target: JetPack 7.2.x (L4T R39.2.x), Ubuntu 24.04, ROS 2 Jazzy and
Isaac ROS 4.6. The previous JetPack 6 / Humble / Isaac ROS 3.2 image is not
compatible with this deployment. This change prepares deployment; it does not
flash the device or claim GPU validation.

Host inspected on 2026-11-14: Orin Nano Super reference developer kit,
L4T R36.4.7, Ubuntu 22.04.5, ROS Humble. Root is on `/dev/mmcblk0p1`,
with approximately 213 GiB used; no separate NVMe target was visible.
The old host is intentionally rejected by `scripts/run_cuvslam.sh check`.

Before flashing, back up the workspace **including uncommitted and untracked
files**, models and calibration data to a separate physical device. A copy on
the current root partition is not a recovery backup. Select the installation
target explicitly: flashing the current card can erase the workspace and OS.
Use NVIDIA's JetPack 7.2.x installation instructions for the Orin Nano; do not
perform a generic Ubuntu distribution upgrade or replace R36 apt sources with
R39. JetPack 7.2.1 uses a unified ISO rather than an Orin Nano SD-card image.
Firmware and rollback procedures must follow the official board instructions.

After installing the new BSP, install ROS 2 Jazzy and its CycloneDDS RMW, then
recreate `rosenv` with the new system Python and rebuild the workspace. Do not
reuse Humble's `build/`, `install/`, `log/` or Python environment. Follow the
workspace setup instructions and first validate simulation without VIO.

## Build and run (after host migration)

Follow the **release-4.6** Isaac ROS setup guide (links below) to initialize the
Docker development environment. Select its JetPack 7.2 / Jazzy base, with the
`release-4.6 noble-jetpack` signed apt repository configured. The old
`release-3.2` `build_image_layers.sh` recipe no longer applies. Obtain the actual
base image reference from the configured CLI / local Docker image list; use an
immutable digest where available, rather than guessing a registry tag or using
`latest`. Set `ISAAC_ROS_BASE_IMAGE` to that reference. The Dockerfile checks
Ubuntu, Jazzy and the repository before installing cuVSLAM.

```bash
scripts/run_cuvslam.sh check
# Replace the value with the verified Isaac ROS 4.6 JetPack 7.2 base reference.
export ISAAC_ROS_BASE_IMAGE='<verified-image-reference>'
scripts/run_cuvslam.sh build

# terminal 1: simulation with stereo and the localizer (textured scene)
scripts/ball_chase_walk.sh scene:=hard enable_vio:=true enable_astar:=true
# terminal 2: cuVSLAM
scripts/run_cuvslam.sh
# terminal 3: error against ground truth (evaluation only)
ros2 run humanoid_localization localization_eval --ros-args -p duration_s:=120.0
```

All terminals must use the same `ROS_DOMAIN_ID` (default 0),
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_LOCALHOST_ONLY=1` and
`cyclonedds_sim.xml` settings. Rebuild the simulator to install the updated XML.
For terminal 3, set `CYCLONEDDS_URI` to the installed simulator config as in
`scripts/ball_chase_walk.sh`. The container uses the repository copy of that
same config. It no longer mounts the host's old `tegra` or `vpi3` directories.

The 4.6 API replaces `enable_imu_fusion` with `tracking_mode`. Both cuVSLAM
launch paths map `imu_fusion:=true` (default) to mode 1 using the rigidly mounted
`/head/imu`, and `imu_fusion:=false` to mode 0 (stereo VO). The body IMU remains
downstream only. Existing container-name, launch-path and CPU-affinity overrides
are preserved for the combined YOLO/cuVSLAM graph. YOLO TensorRT engines must be
rebuilt for the target device and new runtime before using that graph.

Before accepting deployment, verify both stereo inputs at 15 Hz, camera-info
and TF availability, continuous VO and localization output, and the 120-second
localization evaluation. Exercise head pan/tilt, walking, low texture and VO
loss/recovery; compare drift and output gaps with the JetPack 6 baseline.
Container build and live GPU execution have not been verified on JetPack 7.

Official references:

- [JetPack downloads and board installation links](https://developer.nvidia.com/embedded/jetpack/downloads)
- [Isaac ROS 4.6 setup](https://nvidia-isaac-ros.github.io/v/release-4.6/getting_started/index.html)
- [Isaac ROS 4.6 development environment](https://nvidia-isaac-ros.github.io/v/release-4.6/concepts/dev_env/index.html)
- [4.6 cuVSLAM parameter and topic implementation](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/release-4.6/isaac_ros_visual_slam/src/visual_slam_node.cpp)

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
- IMU fusion (2026-11-14) is checked only off the Jetson: unit tests of the
  extrinsic (position, and gyro axes against the rig's angular velocity at
  several head poses) and of `/head/imu` from `sim_imu_publisher` (199 Hz, no
  repeated stamps, noise and bias as configured). It has not run in cuVSLAM
  yet, and the measurements above are pure stereo VO.
- On hardware the head IMU and the D435 need a common clock (MCU time sync);
  cuVSLAM rejects IMU jitter over `imu_jitter_threshold_ms` (10 ms).
