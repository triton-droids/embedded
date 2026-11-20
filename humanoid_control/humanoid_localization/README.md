# humanoid_localization

Base pose from stereo visual odometry on the D435 infrared pair (cuVSLAM,
Isaac ROS 4.6 on the Jetson GPU), optionally combined with the body IMU
attitude. Output: `/localization/odom` (frame `world`, child `base_frame`).

```
realsense2_camera (host) ── /camera/camera/infra{1,2}/{image_rect_raw,camera_info}, TF camera_link → optical frames
        │
        └── cuVSLAM (container, scripts/run_cuvslam.sh) ── /visual_slam/tracking/odometry (pose of camera_link)
                                                         │
static TF base_link → camera_link (measured mount) ──────┤
/imu/data_raw (attitude_source:=imu only) ───────────────┴── vio_localizer ── /localization/odom, /localization/status
```

Ported from `sim-experimental-cyclonedds`, where it was developed against a
simulated passive stereo pair. The simulator-only tools (`localization_eval`
against ground truth, `route_driver`) are not included.

## Method

- **Position**: each VO step is converted to a base displacement with the
  `base_frame → camera_link` transform at both stamps, rotated into `world` and
  added. Steps implying more than `max_speed_m_s` (2 m/s) are rejected; a VO gap
  longer than 0.5 s restarts the increment chain without moving the estimate.
- **Attitude** (`attitude_source`):
  - `vo` (launch default): from the VO rig pose. The world frame is fixed at the
    first VO sample, assuming the base is level there at `initial_yaw`. Use it
    with an IMU that publishes no orientation: the control launch runs the
    serial MPU reader with orientation output off. VO yaw drift is not corrected.
  - `imu`: from the orientation quaternion on `imu_topic`, with a one-time yaw
    offset so the start heading equals `initial_yaw`. This is the intended mode
    with the BNO085 body IMU firmware (game rotation vector, no magnetometer);
    it has not run on hardware yet.
- cuVSLAM runs as pure stereo VO (`imu_fusion:=false`). Fusing the head IMU
  (`/head/imu`) needs the measured `camera_link → head_imu_link` transform and a
  common camera/IMU clock; cuVSLAM rejects IMU jitter over 10 ms.
- cuVSLAM publishes no TF; `vio_localizer` owns the base pose.

## Camera

- Infrared pair only (`enable_color:=false` in `vio_d435.launch.py`), 640×480 at
  30 Hz, projector off (`depth_module.emitter_enabled: 0`). The projected dot
  pattern moves with the camera and would be tracked as static scenery.
- Passive stereo needs ambient near-infrared light and texture. Dim LED or
  fluorescent lighting gives dark images and long exposures (motion blur);
  uniform surfaces (plain walls, carpet) give few features.
- One process per camera: when head tracking also needs the color stream, start
  a single `realsense2_camera_node` with both and pass `start_camera:=false` here.

## Run

The `humanoid_cuvslam:4.6-jp7` image must exist on the Jetson (JetPack 7.2.x):

```bash
scripts/run_cuvslam.sh check
export ISAAC_ROS_BASE_IMAGE='<verified Isaac ROS 4.6 JetPack 7.2 Jazzy base reference>'
scripts/run_cuvslam.sh build    # once; about 32 GB
```

Measure the camera mount on the robot (metres, radians, `base_link` frame), then:

```bash
# terminal 1 (host): camera, mount TF, vio_localizer
ros2 launch humanoid_localization vio_d435.launch.py \
  camera_x:=<m> camera_y:=<m> camera_z:=<m> camera_roll:=<rad> camera_pitch:=<rad> camera_yaw:=<rad>
# terminal 2: cuVSLAM in the container
scripts/run_cuvslam.sh
# check
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic echo --field data /localization/status
```

Both sides must use the same `ROS_DOMAIN_ID` and DDS profile; the container
reads `config/dds/cyclonedds_$DDS_PROFILE.xml` (default `local`), like
`scripts/dds_env.sh` on the host.

## Verified (2026-11-19, Jetson Orin Nano Super, JetPack 7.2.1, Jazzy)

- `test/test_fusion.py`: 7 passed (both attitude sources; moving head,
  arbitrary VO frame, start heading, step rejection).
- Real D435, camera stationary, mount TF zero, `attitude_source:=vo`: infrared
  pair 30.0 Hz each with the projector off, cuVSLAM initialized and published
  odometry at 30.0 Hz, `/localization/odom` at 50 Hz. Over more than 60 s,
  3835 VO steps were accepted, none rejected, and the position stayed at
  (0.000, 0.000, 0.000) m. cuVSLAM reported 4 dropped frames (66.7 ms gaps).

Not verified: tracking while moving, low light or low texture, `attitude_source:=imu`
with the BNO085, head IMU fusion, combined load with the detector and the
control stack.

## Limits

- The camera mount is a static TF, valid only while the head servo holds one
  pose. The head has no encoder; a moving head needs forward kinematics from a
  measured angle.
- No VO tracking status is consumed; bad tracking is caught only by the speed gate.
- Neither attitude source has an absolute heading; global correction (field
  lines, goals) is separate work.
