# humanoid_ball_chase

Independent MuJoCo D435 camera/mount and football pursuit package. It reuses
`humanoid_sim` peripherals, `humanoid_vision/track_node`, the existing C++ motor
scheduler, Web monitor, safety supervisor, and policy node.

**Current boundary:** visual detection, moving camera TF, depth projection, head
yaw/pitch tracking and bounded pursuit velocity commands are implemented. The bundled
56-input tracking ONNX follows embedded reference joint positions/velocities;
it does not accept a base velocity command. Consequently the default launch
(`controller:=policy`) does not make the robot walk toward the ball.
`controller:=model_walk` runs the model-based walking controller instead and
feeds it `/ball_chase/cmd_vel` (see `humanoid_control/walking_controller/README.md`):
the robot then walks to the ball and stops in front of it. The ball world
model (`world_model:=true`, default; see "Ball world model") keeps chasing a
ball that is briefly unseen and aims slightly ahead of a moving ball. There is
still no search behaviour: once the track is lost (about 2 s unseen) the robot
stops. `/ball_chase/cmd_vel` needs a trained
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
| `odom_topic` (default `/sim/ground_truth/odom`) | `nav_msgs/Odometry` | Start position; the launch sets `/localization/odom` with `enable_vio:=true` |
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

The chase launch enables `dds_tuning:=on` and `imu_split:=true` by default.
The former selects the simulator's bounded-blocking Fast DDS profile; the latter
publishes body IMU in a separate process while preserving acquisition timestamps.
Use `dds_tuning:=off` or `imu_split:=false` for controlled comparisons. These
options do not relax the 250 ms safety IMU timeout or establish walking stability.
WBC is not enabled: the current shared launch rejects WBC together with vision.

### Harder perception scene (`scene:=hard`)

```bash
scripts/ball_chase_walk.sh scene:=hard
```

`scene:=hard` (`scene.py:build_scene_hard`, `config/scene_hard.yaml`) keeps
the robot, ball, physics and start pose of the default scene and adds what the
real HSLVision images show detectors missing:

- a row of three red ~0.9 m robots 0.35-0.4 m behind the ball, two blue
  robots further back, a person in jeans with white shoes beside the ball and
  two more people further away;
- white field lines through and next to the ball (goal line, box lines,
  side line, penalty mark, a centre-circle arc), goal posts and crossbar,
  coloured advertising boards, striped and speckled turf;
- a low, shadow-casting sun;
- camera effects (see `humanoid_sim/image_degradation.py`): motion blur
  from the camera's actual angular velocity over a 30 ms exposure, slight
  defocus, gain 1.2 with ±15 % per-frame exposure flicker, sensor noise
  (σ = 5) and JPEG quality 45. Depth stays ideal.

All added geometry is visual only (no collision, no mass), so locomotion and
contacts are identical to the default scene; the clutter also appears in the
depth image. The effects cost about 19 ms per 424x240 frame on the Jetson
in GIL-releasing OpenCV calls.

Chase runs on the Jetson (`controller:=model_walk`, 55 s each, 2026-11-09),
per-frame vision status from `/ball_chase/status`:

| model | released runs | reached ball (0.47-0.52 m) | frames with `detection_failed` |
|---|---|---|---|
| td-perception | 6/6 | 6/6 | 7-15 % |
| nubot-yolo | 5/6 | 4/5 (one estop at 4 s: `command expired`, before the GIL fix) | 0 % |

The ranking is the reverse of the real-image HSLVision result (see above):
on six offline-rendered frames td-perception missed the two most blurred
ones, and nubot-yolo was partly trained on rendered (NUpbr) images. More
recall in this scene does not mean more recall on a real camera. Use this scene to exercise the
chase loop under intermittent detections and clutter, not to choose a
detector; rendered frames do not predict real-camera accuracy.

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
seven-class model. By default (`vision_device:=gpu`) it runs as a TensorRT FP16
engine; `vision_device:=cpu` uses ONNX Runtime on one CPU thread (see "Detector
on GPU" below). Only class 0 (`ball`) supplies a
target, with confidence >= 0.30 and NMS IoU 0.45. Input is letterboxed RGB
640x640. `assets/MODEL_SOURCE.json` records its source, SHA256 and the model
metadata's AGPL-3.0 license declaration. The orange HSV detector remains an
explicit `orange_test` unit-test mode; it is not an automatic fallback.

The six-class `nubot-yolo.onnx` (same YOLO11n format, class 0 = ball; source in
`assets/NUBOT_MODEL_SOURCE.json`) is bundled as a non-default alternative:
`vision_model_path:=$(ros2 pkg prefix humanoid_ball_chase)/share/humanoid_ball_chase/assets/nubot-yolo.onnx`.
On the HSLVision test split (911 images, same decoder for both) its ball AP is
0.50 / AP50 0.74 against 0.76 / 0.93 for td-perception, with 0.14 false balls
per empty image at confidence 0.30 against 0. td-perception was trained on that
dataset's train split, which favours it. In the simulated chase
(`controller:=model_walk`, Jetson, 2026-11-09) both reach the ball and stop
0.50-0.52 m in front of it (nubot: 3/3 released runs, no falls). Both have
the same latency (CPU about 380 ms, GPU about 25 ms per frame).

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

With `world_model:=false` that is the whole pursuit: each frame gives a
command, and any loss stops at once. The default world model is below.

### Ball world model

Both scenes enable the simulated odometry TF (`world → odom → base_footprint →
base_link`, see `humanoid_sim/README.md`, "Odometry TF"). With
`world_model:=true` (default) every detection is put into the world-fixed
`odom` frame at its image stamp and fed to a Kalman tracker (constant velocity
with exponential rolling friction, vendored from `td_ball_tracker` at cae8fb1
into `humanoid_ball_chase/tracking/`). Robot motion is therefore not mistaken
for ball motion, and the estimate survives frames without a detection.

* **Ball center.** Depth at the box center is the ball's front surface; the
  point is moved one `ball_radius` (0.10 m) further along the ray, in both
  modes. `/vision/ball_3d_position` is now the center, so the robot stops
  about 0.07 m closer than before for the same `stop_distance`.
* **States** (td_ball_tracker): tentative (2 hits within 0.6 s needed, retuned
  for 6 Hz) → tracking → coasting (no detection) → lost after 2.0 s or a
  position σ above 1.5 m.
* **Chase** (20 Hz timer, current `odom → base_link`): tentative or lost →
  zero command. Seen within the last 0.5 s → chase as before. Coasting →
  chase the prediction at half speed, and only turn once σ exceeds 0.5 m. A
  ball moving faster than 0.15 m/s with a confident velocity → chase where
  the motion model puts it `lead_time_s` (1 s) later, at most 1 m ahead. An
  earlier version chased the rest point `p + v / friction`; on this floor that
  is metres away, and with a ball circling the robot it turned the robot away
  until the ball left the view.
* **Head.** Both `track_node`s follow `/ball_chase/head_target`: the detection
  pixel when seen, the projected prediction while coasting (also off-image or
  behind the camera, so the head turns towards it). Lost → no target; the
  trackers stop as before.
* **Friction.** The MuJoCo ball's speed decays at about 0.06/s (measured
  2026-11-10 by pushing it), far less than turf; the launch sets
  `tracker.friction: 0.06`. On a real pitch it must be measured again.
* `/ball_chase/status` adds `world_model` (tracker status, odom position,
  velocity, σ, time since seen, chase mode/point); `state` is then
  `tracking`, `coasting`, or the tracker status.
* **Odometry.** `odometry:=ground_truth` (default) or `odometry:=leg`: the
  walking controller package's `leg_odometry` (body IMU attitude + stance-foot
  kinematics, what the robot has) publishes `odom → base_footprint →
  base_link` instead; the chase results are the same (see
  `humanoid_control/walking_controller/README.md`, "Leg odometry").
* **Kick** (`kick:=true`, needs `controller:=model_walk`; `kick.py`). Within
  0.9 m of a still ball the chase hands over to a slow approach that turns
  until the ball sits ahead of the foot on its side, walks up until the ball is
  about 0.19 m ahead of `base_link` (the kick window is 0.18-0.29 m true; the
  estimate reads 4-7 cm short at this range), waits for the walking
  controller to stand, sends `/walk/kick`, then holds 2 s and chases again.
  The ball position is frozen in odom once it is within 0.45 m: next to the
  feet the robot's own legs hide it, and partial views made the tracker think
  it rolled. `/ball_chase/status` `world_model.kick` has the state, foot,
  attempts and kicks. Jetson, still ball (2026-11-11): 4 of 4 runs kicked it
  on the first attempt (2.6-9.8 m on this low-friction floor), no falls.
* Test hook: `ros2 topic pub --once /ball_chase/debug/blind std_msgs/msg/Float32 '{data: 1.5}'`
  ignores detections for 1.5 s, like an occlusion.

Jetson, `scripts/ball_chase_walk.sh`, GPU detector, one run per cell
(2026-11-10), with the walking controller's original caps (0.06 m, 0.08 rad
per step) and the first rest-point version of the chase. Arrive = ball center
within 0.70 m; no falls in any run.

| Scenario | `world_model:=true` | `world_model:=false` |
|---|---|---|
| Still ball | arrives 14.0 s, stops 0.55 m | arrives 13.0 s, stops 0.49 m |
| 1.5 s blind while walking | keeps walking (mean 0.13 m/s), arrives 8.5 s after | stops (mean 0.02 m/s), arrives 9.5 s after |
| Push (0.35, 0.45) m/s while walking; ball rolls 5.1 m | arrives 55.7 s after | arrives 54.4 s after |
| Push 0.6 m/s sideways when stopped; ball rolls 5.4 m | never lost, arrives 53.7 s after | never lost, arrives 57.0 s after |
| Push 1.5 m/s sideways; ball rolls 13.7 m | smooth chase with coasting, lost at about 10 m (24 s), never found; ends 12.6 m away | stop-go: lost/seen every 0.2-0.5 s (lost 84 % of the time); ends 13.1 m away |

Tracker position error against ground truth: 0.3-1.1 cm mean, p95 under
2.2 cm. A ball that leaves detection range is not searched for.

**Walking caps.** At 0.08 rad per step the robot turned only about
0.13 rad/s, so a ball circling it (`/ball_chase/orbit`, below) drifted out of
view. The launch now passes `walk_max_step_m:=0.08` and
`walk_max_turn_rad:=0.15` (node limits 0.10 / 0.15). The earlier 0.08 rad cap
avoided a heading limit cycle with the 0.5-1 s CPU detector; with the GPU
detector none was seen. With `vision_device:=cpu` pass 0.06 / 0.08. Jetson,
world model on (lead 1 s), 2026-11-10, no falls in 17 runs:

| Scenario | 0.06 m / 0.08 rad | 0.08 m / 0.15 rad |
|---|---|---|
| Still ball | arrives 13.5 s | arrives 9.9-10.8 s (4 runs), stops 0.50-0.52 m |
| 1.5 s blind while walking | arrives 9.0 s after | arrives 7.0 s after |
| Push while walking; ball rolls 5.1 m | arrives 54.3 s after | arrives 43.8 s after |
| Ball circles at r 1.5 m, 0.25 m/s | lost 15 %, heading error 65 deg | never lost, 2.6 deg (also `scene:=hard`: 3.2 deg) |
| Ball circles at 0.4 m/s | - | never lost, 5.2 deg |
| Ball circles at 0.6 m/s | lost 50 % | lost 41 % (needs about 0.4 rad/s) |

Ball circling (simulator only): `ros2 topic pub --once /ball_chase/orbit
std_msgs/msg/Float32MultiArray '{data: [1.5, 0.25]}'` (radius m, speed m/s,
positive counter-clockwise, centred on the robot as it moves); `'{data: []}'`
stops it.

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

## Detector on GPU (TensorRT)

Build the engines once on the target Jetson (about 10 minutes per model on an
Orin Nano; they are device- and TensorRT-specific and ignored by Git):

```bash
scripts/build_ball_chase_engines.sh              # every assets/*.onnx
scripts/build_ball_chase_engines.sh nubot-yolo   # one model
```

Each `assets/<model>-fp16.engine` gets a `.json` record with its ONNX SHA-256;
the detector refuses a missing or mismatched engine and names this script. It
loads JetPack's system TensorRT Python binding (rosenv has none) and calls
`libcudart` through ctypes; no extra Python packages are installed. Pre/post
processing, thresholds and NMS are the same as on CPU.
`test/test_football_detector_gpu.py` checks GPU against CPU (class scores
within 0.02, confident boxes within 1 px of the 640 input) and is skipped where
no engine exists.

Jetson Orin Nano Super (JetPack 6.2.1, TensorRT 10.3, MAXN_SUPER), 424x240
frames, per `detect()` call:

| device | network | detect() incl. pre/post-processing |
|---|---|---|
| CPU, ONNX Runtime, 1 thread | 377-381 ms | 382-386 ms |
| GPU, TensorRT FP16 | 14-16 ms | 25 ms |

Eight instrumented chase runs per device (`controller:=model_walk`, clean and
`scene:=hard`, both models, 60 s, 2026-11-09; ground truth and
`/ball_chase/status` recorded):

| | CPU | GPU |
|---|---|---|
| processed detections | 2.4 /s | 5.7-6.0 /s (every 6 Hz frame) |
| image age when used, p50 / p95 | 0.74-0.78 / 0.94-0.96 s | 0.14-0.19 / 0.22-0.27 s |
| control ticks rejected as `stale_image_stamp` | 0.4-5 % | 0 % |
| release to within 0.62 m of the ball | 23-30 s | 15.5-20 s |
| final distance, base to ball centre | 0.59-0.62 m | 0.40-0.60 m |
| vision process CPU | 110 % of a core | 20 % |
| whole board busy (6 cores) | 64-68 % | 53-55 % |
| simulation real-time factor p50 | 0.83-0.87 | 0.90-0.93 |
| GPU load (tegrastats GR3D) | about 5 % | 9-20 % |

With fresh detections the robot first ended 0.1-0.2 m closer: stepping in
place after the command reached zero crept towards the ball (closest 0.375 m
centre distance). Since 2026-11-10 the walking controller stands when the
command rests and the vision node adds stop/yaw hysteresis (`resume_margin`
0.08 m, `yaw_deadband_rad` 0.05, `yaw_deadband_hold_rad` 0.12): final
0.53-0.58 m, creep under 1 mm/s (design doc §21). In `scene:=hard`, td-perception drops the ball in
4-46 % of control ticks only after it has stopped close to the ball (ball at
the image bottom among legs and shoes); while approaching it fails in about
0-1 %. One estop (`command expired`) per eight runs occurs on both devices
and is not detector related.

`vision_target_timeout_s:=1.0` (below) was chosen for CPU inference; with the
GPU the p95 image age is under 0.3 s, so it can be lowered, which has not been
tested yet.

## Isaac ROS detector (`vision_detector:=isaac`)

The same YOLO11n engines also run in the Isaac ROS pipeline (Jetson only):
encoder (aspect-preserving resize, centered padding) -> TensorRT ->
YOLOv8 decoder in one NITROS container. Their output `[1, 4 + classes, 8400]` has
the YOLOv8 layout, so the models are used unchanged. The pipeline publishes all
classes on `/isaac_yolo/detections` (`vision_msgs/Detection2DArray`, boxes in
640x640 network pixels); `ball_chase_vision` then chases one class id.

```bash
scripts/run_cuvslam.sh build          # once: the Isaac ROS image (also used for cuVSLAM)
scripts/run_isaac_yolo.sh             # terminal 1, td-perception (7 classes)
scripts/ball_chase_walk.sh scene:=hard vision_detector:=isaac vision_target_class:=0   # terminal 2
```

`vision_target_class` (td-perception): 0 ball, 1 goalpost, 2 robot,
3 L-, 4 T-, 5 X-intersection, 6 penalty mark. For nubot-yolo pass
`num_classes:=6 engine_file_path:=...` to `run_isaac_yolo.sh`. Each RGB-D frame
waits up to `detection_wait_s` (0.3 s) for the detections with its stamp, maps
the box back to image pixels and reads depth like the built-in detector
(`point_from_box`). A class with several instances (robots, intersections) locks
on the instance nearest the previous target (camera-frame 3D point) and re-picks
the one nearest the camera after `target_reacquire_s` (1 s) without it.

Measured on the Orin Nano (hard scene, 60 s, same engine): the ball agrees with
the built-in detector in 342 of 342 shared frames, 0.64 px median center
difference, similar latency; no CPU saving in simulation, where images reach the
container over DDS. Log messages and `ball_radius` still assume a ball.
The original `evaluation/vio_isaac_trials/` evidence directory is not present in
this checkout. Current goal-shot trials and video locations are indexed in the
[2026-11-13 report](../../docs/reports/2026-11-13-goal-shot.md); that report does
not independently revalidate the earlier detector comparison above.

## Localization without simulator odometry (`enable_vio:=true`)

Renders a passive stereo pair (the D435 IR imagers with the projector off),
starts `vio_localizer` (body IMU + cuVSLAM visual odometry) and points the A*
planner at `/localization/odom`. The walking controller keeps ground truth.
cuVSLAM runs in the Isaac ROS container; start it before the robot moves, since
the localizer takes the known start pose at its first VO frame:

```bash
scripts/run_cuvslam.sh                                    # terminal 1 (Jetson)
scripts/ball_chase_walk.sh scene:=hard enable_vio:=true   # terminal 2
```

`imu_yaw_error_deg` / `imu_yaw_error_period_s` add a sinusoidal heading error to
the simulated BNO085. Details and accuracy: [`humanoid_localization`](../../humanoid_control/humanoid_localization/README.md).
On the Orin Nano, cuVSLAM and the Isaac YOLO container together leave the walking
controller short of CPU (falls while turning were observed).

## CPU inference budget (`vision_device:=cpu`)

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


## Experimental goal-directed shot (simulation)

**2026-11-13 publication note:** this section describes the tested Jetson
working tree. Recent implementation/test files remain uncommitted; this
publication contains Markdown only. A checkout of this commit alone does not
provide the complete new shot path. See the
[test report](../../docs/reports/2026-11-13-goal-shot.md) and
[handoff](../../docs/handoff-goal-shot-2026-11-13.md).


`controller:=model_walk world_model:=true kick:=true shoot:=true` enables a one-shot
supervisor. Set `shoot_goal_frame` and `shoot_goal_x_m` / `shoot_goal_y_m` to the known
goal center; defaults are the simulation world frame and (0.35, 1.8) m. These
coordinates must describe the goal in the selected scene. `scene:=shoot` adds a
clean shooting fixture with that default goal and a ball at world (-0.35, 0.95).
The goal posts are visual; scoring uses the ball crossing the goal opening. The
feature does not detect the goal center from the camera.

The supervisor uses the visual world-model ball estimate and TF to stage 0.65 m
behind the ball with a kicking-foot offset (0.12 m coarse staging tolerance), avoids crossing the ball on its way,
aligns with the ball-to-goal ray and waits for actual standing before final approach.
Shooting uses the gravity-aligned `shoot_control_frame` (default `base_footprint`)
for both ball and goal coordinates; camera extrinsics still use the full body pose.
This keeps torso lean from changing the planar kick-distance measurement. Missing
TF holds zero command. A pose controller jointly follows the position behind the
ball and the goal heading;
the generic foot-bearing steering no longer competes with goal alignment. A gross
staging displacement (>0.18 m) returns to staging. Within 0.40 m the command is capped to 0.03 m/s and 0.12 rad/s and prioritizes
goal heading. Entering the kick window uses 0.025 m forward / 0.035 m lateral
tolerance and <=0.08 rad goal-heading error; the existing 1.6x exit hysteresis
absorbs the final support shift. The robot stands for 0.3 s before admission through the existing KickApproach and /walk/kick
interface. Final approach uses the existing robot model and measured `/joint_states`
to compute the current kicking-foot center. The ball target is 0.20 m in front of
that foot, expressed in the same gravity-aligned frame. Joint source age and local
receipt age must each be <=0.15 s; missing, invalid or stale geometry holds final
approach and forbids a kick. After standing, the original tight kick window is
checked again. `shoot_urdf_path` defaults to the installed
`humanoid_leg_description/urdf/human_offset_corrected.urdf`, and
`shoot_base_forward_axis` defaults to `positive_y`, matching this simulation.
The model, joint names and axis must match any other deployment. Inspect
`world_model.last_foot_admission` and `world_model.kick.foot_reference_xy`.
This is read-only geometry; it does not write motor commands. A stationary visual anchor with <=0.12 m
uncertainty may bridge up to 20 s of lost view before close approach. Within
0.70 m a fixed anchor is used for navigation/alignment. At approximately 0.40 m
during final approach, close visual confirmation is required: the head continues
tracking, and every consistent accepted image updates the target in odometry
coordinates. Confirmation requires at least three image observations spanning
0.20 s, <=0.06 m spread, confidence >=0.5 and measurement sigma <=0.12 m.
When occluded, the last confirmed image position is carried by current TF for
up to 12 s from its original image stamp, assuming the ball stays still.
Observations with confidence <0.5 do not update or invalidate the last reliable
position, and do not refresh its original timestamp. A gap in good detections
resets candidate confirmation without revoking a consistent, unexpired last
position. A contradictory high-quality new position still stops use of the old
anchor; a single new image cannot revive an expired target. Other inconsistent
observations or missing/expired close confirmation hold locomotion
and forbid a new kick request; they do not cancel an already executing kick.
Inspect `world_model.final_ball` for close confirmation and image age. Normal TF
and vision validity checks still apply. No simulator ball truth is used for control.

With `shoot:=true`, YOLO detections use a known-radius fit to the visible depth
surface to estimate the ball center, instead of adding one radius to the median
box-center depth. The fit uses at most 600 points and 1024 deterministic hypotheses;
insufficient surface support is rejected. The center is not shifted by a radius a
second time. The configured `ball_radius` must match the ball; this is a simulation
assumption, not validation on a physical depth camera. Confidence <0.5 still cannot
refresh the saved kicking target. Other chase modes keep their original depth path.

Alignment targets 0.055 rad before approach. A kick request is refused if the
remaining heading error exceeds 0.18 rad (~10 degrees); it holds and reports that
repositioning is required, rather than turning beside the ball. The attempt ends
after one kick or after 120 s. Inspect `world_model.kick` on /ball_chase/status for
phase, heading error and refusal reason. Restart the vision node for another
attempt. `shoot:=false` remains the default.

The current tests use known goal coordinates and simulation ground-truth robot
odometry through the existing TF source. They are not visual goal localization or
physical robot qualification. Existing motor expiry, tilt and stop protections are
unchanged. Test fixtures, recordings and development failures are documented in the
[dated test report](../../docs/reports/2026-11-13-goal-shot.md). Raw evidence remains
on the Jetson under `/home/darin/vio_runs/shot_refine_20261113/`.

Example (activate the normal ROS environment first):

```bash
ros2 launch humanoid_ball_chase ball_chase.launch.py controller:=model_walk \
  auto_start:=true scene:=shoot shoot:=true kick:=true world_model:=true \
  use_viewer:=false walk_max_step_m:=0.06 walk_max_turn_rad:=0.08
```

The default detector is the existing football detector. For Isaac, use
`vision_detector:=isaac` and start the normal Isaac detector on the same ROS domain.


### Latest validation and learned-motion plan

The final working-tree regression suite passed 117 non-GPU tests; actual Isaac
GPU inference was exercised in complete simulation. The selected recording
`video03` completed autonomous approach, standing admission, kick and `done`:
0.8724m/s peak ball speed, 1.488cm goal-center crossing error, no pre-kick ball
contact or control fault during the observation window. Other attempts still
failed to score or stopped on command expiry. A previous successful but inaccurate
trial moved the ball 2.31cm before kicking, breaking the saved-target stationary
assumption. No claim of general stability or hardware readiness follows.

The next intended controller is ML for walking, turning and kicking. Keep visual
ball tracking, confidence/time validity, known goal and task supervision reusable.
The replacement needs compatible command and action-status semantics; simply
changing an MPC file to an inference call is insufficient when the policy replaces
whole action execution. See [design §22](../../docs/model_based_walking_design.md#22-goal-shot-validation-and-ml-transition-2026-11-13).
