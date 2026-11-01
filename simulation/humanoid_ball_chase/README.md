# humanoid_ball_chase

Independent MuJoCo D435 camera/mount and football pursuit package. It reuses
`humanoid_sim` peripherals, `humanoid_vision/track_node`, the existing C++ motor
scheduler, Web monitor, safety supervisor, and policy node.

**Current boundary:** visual detection, moving camera TF, depth projection, head
yaw/pitch tracking and bounded pursuit velocity commands are implemented. The bundled
56-input tracking ONNX follows embedded reference joint positions/velocities;
it does not accept a base velocity command. Consequently this launch does not
make the robot walk toward the ball. `/ball_chase/cmd_vel` needs a trained
velocity-conditioned locomotion policy with its exact observation/export
contract. No base teleport, applied chase forces, motor bypass or reinterpretation
of tracking observations is used. `auto_start:=true` releases the existing
reference-motion demo, and does not close the pursuit locomotion loop.

## Run

### Optional 2D A* global planner

Start the normal launch with `enable_astar:=true`, or run the planner alone:

```bash
ros2 run humanoid_ball_chase ball_chase_astar
```

The planner uses eight-connected A* with a Euclidean heuristic. It blocks unknown
cells and occupancy >= 50, inflates obstacles for a configurable circular robot
footprint (`robot_radius_m`, default 0.25 m), and forbids diagonal corner cutting.
The radius is an initial simulation setting, not a verified walking envelope.

| Topic | Type | Purpose |
|---|---|---|
| `/ball_chase/map` | `nav_msgs/OccupancyGrid` | External obstacle map; reliable/transient-local QoS |
| `/ball_chase/goal` | `geometry_msgs/PoseStamped` | Static XY goal; final heading is currently ignored |
| `/sim/ground_truth/odom` | `nav_msgs/Odometry` | Simulation-only start position |
| `/ball_chase/global_path` | `nav_msgs/Path` | Cell-centre route, replanned at 2 Hz and on map/goal updates |
| `/ball_chase/planner_status` | `std_msgs/String` | Planned, waiting, or rejection reason |

All three inputs must share a nonempty frame (`world` for this simulation).
Map origin rotation must be identity; rotated maps are explicitly rejected.
Odometry source and receipt must both be <= `pose_timeout_s` (default 1 s).
Invalid/stale inputs, blocked endpoints, no route, or the 100,000-expansion
budget produce an empty path, clearing the previous route. Maps are limited to
250,000 cells. Unknown space is never assumed traversable.

With a map already published, an example goal is:

```bash
ros2 topic pub --once /ball_chase/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 0.0, y: 2.0}, orientation: {w: 1.0}}}'
```

This is a global-path interface. It does not yet turn camera depth into a map,
transform the detected football into a navigation goal, follow the path, or
connect velocity commands to locomotion. Existing direct visual pursuit commands
remain independent. No empty free-space map is synthesized when a map is missing.

### Camera and football simulation

```bash
source /opt/ros/jazzy/setup.bash
source scripts/activate_rosenv.sh
python -m pip install -r simulation/humanoid_ball_chase/requirements.txt
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 /usr/bin/python3 -m colcon build \
  --executor sequential --symlink-install --packages-up-to humanoid_ball_chase \
  --cmake-args -DTRACK_ONLY=ON -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch humanoid_ball_chase ball_chase.launch.py
```

The default domain is 61, localhost only, monitor port 8081. Motors remain
disabled on the gantry by default. Add `use_viewer:=false` for headless EGL
(`export MUJOCO_GL=egl`). Avoid another ROS publisher using the same camera TF.
Camera and head tracking are simulation replacements; no physical camera or
serial Agent is launched. No robot CAN process is launched.

## Ball controls

The football is radius 0.10 m, mass 0.20 kg, with a free joint, floor
collision and friction. Its initial world position is `(-0.25, 1.8, 0.11)` m.
This scene defines forward along the actual foot toe direction, world `+Y` at spawn. Use the
viewer's mouse perturbations or these simulator-only controls on domain 61:

```bash
export ROS_DOMAIN_ID=61 ROS_LOCALHOST_ONLY=1
ros2 service call /ball_chase/reset_ball std_srvs/srv/Trigger '{}'
ros2 topic pub --once /ball_chase/throw_position geometry_msgs/msg/PointStamped \
  '{header: {frame_id: world}, point: {x: -0.4, y: 2.0, z: 0.3}}'
ros2 topic pub --once /ball_chase/throw_velocity geometry_msgs/msg/Twist \
  '{linear: {x: -0.15, y: 0.1, z: 0.6}}'
```

`throw_velocity` uses world linear velocity and free-joint local angular
velocity. Placement clears velocity. Whole-world `/sim/reset` also restores the
ball. Ball ground truth is published separately for diagnostics, and never
feeds detection or pursuit.

## Interfaces

RGB `rgb8`, aligned depth `16UC1` mm, and CameraInfo use the existing
`/camera/camera/` namespace and matching stamps. The scene uses 424x240 at 6 Hz
with a 58-degree ideal vertical field of view. These are chosen simulation
intrinsics, not D435 factory calibration or sensor-noise emulation.

Default detection uses the newer checkout's `td-perception.onnx` YOLO11n
seven-class model through ONNX Runtime CPU. Only class 0 (`ball`) supplies a
target, with confidence >= 0.30 and NMS IoU 0.45. Input is letterboxed RGB
640x640. `assets/MODEL_SOURCE.json` records its source, SHA256 and the model
metadata's AGPL-3.0 license declaration. The orange HSV detector remains an
explicit `orange_test` unit-test mode; it is not an automatic fallback.

The scene uses a green pitch and a procedural black/white football (12 black
pentagons, 20 white hexagons), with an invisible sphere collision shape. On the
initial rendered green-pitch frame, the actual model detects the football at
about 0.90 confidence. This is a scene-specific observation, not a general
accuracy claim. Valid aligned depth estimates optical XYZ. Outputs remain
compatible with `humanoid_vision`: `/vision/target_center` (PointStamped pixels)
and `/vision/ball_3d_position` (PointStamped optical metres). The old face model
is disabled. Two instances of `track_node` own yaw `/head/servo_target_rad` and
pitch `/head/tilt_target_rad`. The simulator publishes measured joint positions
on `/head/joint_states`; no commanded-angle proxy feeds camera TF.

`base_link -> camera_color_optical_frame` TF comes from actual simulated base
and camera poses at the image sample. ROS base axes are X forward, Y left, Z up;
optical axes are X right, Y down, Z forward. The scene sets
`base_forward_axis=positive_y`: ROS base X follows robot +Y, and ROS base Y
follows robot -X. The CAD lens face, optical axis and toe direction now agree. Dynamic yaw is included. Pursuit
uses image-time TF, a stopping distance, heading gate and bounded speed. Loss,
stale input, invalid depth, mismatched frames or missing TF produce zero
TwistStamped commands on `/ball_chase/cmd_vel`. `/ball_chase/status` exposes
perception state; it does not prove a locomotion consumer exists.

## Asset provenance and assumptions

Assets are imported from the user-provided checkout
`/home/rcli/Github/td_perception_camera`, archive
`ds3225_d435_pantilt_rev_k_slotted_horn.zip`. Its Git remote is
`git@github.com:triton-droids/td_perception.git`, branch `p-33-ball-tracker`,
commit `d75a50b8adb356a025fea7d9511fa55630d16477`.
`assets/SOURCE.json` records the archive SHA256 and assembly mesh SHA256.
All 12 generated STL files are byte-identical to the earlier local revision K
assets, so this source confirmation does not change simulation geometry.

Reimport assets from that checkout with:

```bash
python3 simulation/humanoid_ball_chase/scripts/import_camera_assets.py \
  /home/rcli/Github/td_perception_camera
```

Meshes are converted from the archive's assembly tessellation, with millimetres
scaled to metres. They include the three printed supports, servo envelopes,
horn envelopes, D435 envelope and schematic lens windows; keep-out volumes and
screws are omitted.

The newer checkout also contains a package named `humanoid_vision`. It adds
six-/seven-class YOLO11 ball models (`nubot-yolo`, `td-perception`), class-0
ball target selection, a second pitch tracker, and yaw search after target loss.
It preserves the image/depth/CameraInfo inputs and PointStamped target outputs
used here. The packages share a ROS package name, so select one workspace's
version rather than placing both sources in this colcon build. The simulation
reuses this workspace's axis-configurable tracker twice and runs the newer
football ONNX via Python, avoiding the newer C++ detector's OpenCV/ROS build
requirements. Hardware serial processes and the newer automatic yaw-search
behavior are not enabled. RGB-D synchronization and moving camera TF remain
provided by the chase package.

Yaw and pitch use physical simulated joints. Pitch starts 30 degrees down;
its command is an offset in [-0.6, +0.6] rad, giving an optical declination of
approximately -4.4 to +64.4 degrees. Positive simulated pitch (about body -X) looks down, so
its tracker direction is +1 (the hardware linkage's -1 sign is not copied).
The actuator has a 2.5 Nm limit, a target slew limit of 0.8 rad/s, and the host
tracker uses 0.3 rad/s / 0.6 rad/s². The independent 250 ms command watchdog
releases pitch drive after commands have started; before its first command,
it holds neutral for camera acquisition. Reset restores neutral. A simulated
0.03 Nm gearbox friction holds the camera when drive is released. These
masses, friction, optical centre, adapter and intrinsics are assumptions, not
physical calibration; the CAD horn fit remains unverified. Dynamic camera TF
and the head IMU now follow both actual joints. Added mass affects the policy.

## Verification

Tests cover football ONNX inference on real renders, RGB-D validity, pursuit
limits, CAD compilation, sphere floor contact, pitch drive/release/reset and
optical TF under yaw/pitch. `test/runtime_pitch_smoke.py` verifies actual
measured pitch while repeatedly positioning the football at low/high/low
heights, plus loss stop and reacquisition. Position resets hold test height
against gravity; detections and control still consume rendered images only.
Runtime verification and exact results are recorded in `VALIDATION.md`.

## CPU inference budget

The current host measured 0.46–0.48 s per YOLO frame under GUI/ROS load,
versus about 0.12 s in the earlier lighter-load run. The camera acquires at
6 Hz and inference keeps only the newest pending frame. This launch sets
`vision_target_timeout_s:=1.0` consistently for perception, yaw and pitch.
The detector executable itself retains a 0.35 s default outside this launch.
The launch argument bounds accepted image and receipt age; it can be reduced
on a faster host. Slow detections are rejected rather than restamped.

This change increases how long the host can reuse an old visual observation:
up to 1 s from its original image timestamp. On explicit detection failure,
pursuit goes to zero immediately; without new images it expires at that
source/receipt limit. The independent 250 ms servo-command watchdog and all
leg motor deadlines/safety thresholds remain unchanged. They do not reduce
the host's visual-data age budget. This is a simulation CPU compromise.
