# humanoid_sim — MuJoCo humanoid simulation

This is the active runtime of this simulation-only repository. Physical device
implementations are retained in [hardware/](../../hardware/README.md), outside the
default build and install. The names in the table below identify the original
interfaces being simulated.

Runs the robot software stack without hardware. One `mujoco_sim` process owns
the physics and hosts one ROS node per replaced peripheral; every other node is
the deployed implementation, started unmodified.

| Hardware process | Replaced by | Interface kept |
|---|---|---|
| `python_can_node` (RobStride CAN) | `sim_motor_backend` | `joint_states`, `motor_status`, `motor_telemetry`, `can_timing`, `motor_commands`, `/safety/estop`, `motor_backend/get_feedback`, `motor_backend/execute_operation` |
| Body IMU MCU + micro-ROS Agent | `sim_body_imu` | `/imu/data_raw` (frame `imu_link`, 100 Hz), `/imu/status` |
| Head MCU + micro-ROS Agent | `sim_head_mcu` | `/head/imu`, `/head/mcu_status`, `/head/servo_target_rad` (250 ms watchdog) |
| `realsense2_camera_node` | `sim_camera` | `/camera/camera/color/{image_raw,camera_info}`, optional aligned depth |

The default `minimal:=false` loop retains `cpp_control_node`, the Web monitor,
head MCU, policy and safety supervisor. Policy motion commands always pass through
C++ before reaching the simulated backend. RViz and the joint-state relay are
both disabled by default (`use_rviz:=false start_joint_state_monitor:=false`).
Vision remains opt-in through `enable_vision:=true`.

Minimal mode is available only for disconnected diagnostics (`connect_policy:=false`);
it cannot bypass the C++ command path for a connected policy.

## Quick start

In the dev container, after [Setup](#setup):

```bash
source install/setup.bash
ros2 launch humanoid_sim sim.launch.py auto_start:=true
```

To try balancing around reference frame 0 without advancing the walk:

```bash
ros2 launch humanoid_sim sim.launch.py auto_start:=true hold_first_frame:=true
```

This keeps the model's original frame-0 joint position/velocity reference and
continues closed-loop policy corrections at 50 Hz. It does not lock the joints
or turn the walking model into a validated standing policy. Omit the argument
to play the walking reference normally.

On a host outside the dev container, use the shared activation script in each
launch terminal after completing Setup:

```bash
source scripts/activate_rosenv.sh
ros2 launch humanoid_sim sim.launch.py auto_start:=true
```

The MuJoCo viewer opens with the robot on a virtual gantry. After the
policy's 2 s calibration the motors enable, the gantry releases and the
policy plays its ~6 s reference walk. It often falls within a few steps; see
[Current status](#current-status). Stop with Ctrl+C. Without `auto_start`
nothing moves until you enable the motors and release the gantry yourself
([Typical policy run](#typical-policy-run)); all other ways to watch and
control the running simulation are in
[Interacting with a running simulation](#interacting-with-a-running-simulation).

## What is simulated, and how faithfully

- **Robot**: `chrobot_16kg_candidate.xml`, the MJCF saved with the deployed
  tracking model, loaded unchanged with meshes from `humanoid_leg_description`.
  That file is itself a *candidate* (CAD masses normalized to 16 kg, frames not
  validated), so the simulation inherits its uncertainty.
- **Motors**: the MIT law `tau = kp(q* - q) + kd(dq* - dq) + tau_ff`, evaluated
  every physics step and clamped to the registry torque (or the RobStride peak
  for the model when `max_torque_nm` is null). Command admission, timestamp
  checks, deadline expiry ("explicit enable required"), estop and joint-limit
  clipping follow `python_can_node`. Not modelled: enable acknowledgement
  delay, CAN latency or loss, friction, backlash, motor current limits, the
  ankle four-bar linkage (the sim drives the ankle joint directly), and real
  temperatures (a placeholder first-order heating model drives telemetry).
- **IMU**: MuJoCo gyro, accelerometer (specific force, ~+9.81 on Z upright) and
  frame quaternion at the `imu` site, which is identity-aligned with the base as
  in training. Noise and gyro bias are optional and zero by default.
- **Head**: the robot MJCF has no head, so the simulator adds a 50 g yaw head
  on the `top` site with a rate-limited servo and a forward camera. Positive
  yaw turns toward robot left. Geometry, mass and servo constants are
  placeholders.
- **Camera**: ideal pinhole (no distortion), `fovy` 42.5°, 640×480, 15 Hz.
- **Time**: physics runs at real-time pace and every node uses wall-clock ROS
  time (no `/clock`), because the deployed nodes check input age and deadlines
  against host time. If the host cannot keep up, simulated time slows down and
  `/sim/status` reports `physics_overruns` and the real-time factor.

The simulation exercises software wiring, timing rules and safety latches. It
does not validate hardware calibration, polarity, or sim-to-real transfer.

## Setup

Inside the dev container (or any ROS 2 Humble/Jazzy host), install the shared
Python dependencies into the workspace virtualenv, then build:

```bash
source scripts/setup_rosenv.sh  # Set ROS_DISTRO=humble on Humble hosts.
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --symlink-install \
  --executor sequential --packages-up-to humanoid_sim
source install/setup.bash
```

The installer includes MuJoCo, ONNX Runtime, control dependencies, pytest,
Pinocchio and a locally built TSID with the WBC patches. Its first run downloads
native dependencies and compiles TSID using one compiler process by default.
It also configures the virtualenv package path for ROS console scripts using
system Python. No manual `PYTHONPATH` export is required.

## Run

```bash
ros2 launch humanoid_sim sim.launch.py
```

This opens the MuJoCo viewer and holds the robot on a virtual gantry, with
2 cm initial foot clearance (`foot_clearance_m: 0.02`). The default loop keeps Web, C++ forwarding, the head MCU and the moving target;
RViz and the joint-state relay stay off, and camera/vision remain opt-in. After each
(re)spawn the physics is pre-rolled for 2 s (`settle_s`).
All motors start **disabled**; nothing in the launch enables them unless
`auto_start:=true` is set.

Autostart waits for both the gantry service and the motor command subscriber
before enabling on a fresh policy command. Discovery has a 2 s timeout after
the first fresh command (`sim_autostart.startup_timeout_s` node parameter).
After ENABLE, autostart keeps watching the fresh policy stream. The simulator
allows release only when all motors have accepted motion commands with positive
position gains and at least 40 ms remaining lifetime. A not-yet-ready backend
can be checked again within the startup budget, without repeating ENABLE.
An aborted start sends DISABLE and keeps the gantry engaged; it never re-arms.

The robot stack starts only once the simulator emits its private `Simulation ready`
stdout handshake after the first valid body IMU and joint feedback have actually
been published (captured in the launch log), so
the safety supervisor's 3 s IMU startup grace is not spent while MuJoCo
loads. If `mujoco_sim` exits, the whole launch shuts down.

ROS nodes default to `WARN`, including the simulated motor backend and included
safety/vision nodes. Routine per-motor enable/disable INFO logs are suppressed.
For temporary diagnostics, pass `log_level:=info`. ROS launch may still print
its own process start/stop notices.

Launch arguments (`ros2 launch humanoid_sim sim.launch.py --show-args`
prints the same list):

| Argument | Default | Effect |
|---|---|---|
| `minimal` | `false` | Full stack by default; `true` requires disconnected policy |
| `start_joint_state_monitor` | `false` | Start the optional joint-state visualization relay |
| `log_level` | `warn` | Only ROS warnings, errors and fatal diagnostics by default |
| `use_viewer` | `true` | Open the MuJoCo viewer; set `false` for headless runs |
| `use_rviz` | `false` | With `minimal:=false`, start robot_state_publisher and RViz |
| `enable_policy` | `true` | Run `tracking_policy_node` on simulated IMU and joint feedback |
| `orientation_source` | `message` | Use the simulated quaternion for gravity; `complementary` selects the legacy estimator explicitly |
| `hold_first_frame` | `false` | Freeze the reference at frame 0 while continuing policy inference and feedback corrections |
| `connect_policy` | `true` | Connect policy through C++; requires `minimal:=false` |
| `auto_start` | `false` | Enable the motors and release the gantry when the policy starts its reference motion |
| `release_delay_s` | `0.1` | With `auto_start`: delay from enable to gantry release |
| `start_base_held` | `true` | Start on the gantry |
| `calibration_seconds` | `2.0` | Policy gyro calibration; keep the base held meanwhile |
| `enable_vision` | `false` | Requires `minimal:=false`; render camera and start detector + tracker |
| `start_detector` | `true` | Set `false` for `TRACK_ONLY` builds |
| `use_cuda` | `false` | Detector backend |
| `enable_depth` | `false` | Also render aligned depth (needed for the detector's 3D output) |
| `target_image` | empty | PNG image for the target board (MuJoCo only decodes PNG) |
| `pd_profile` | `tuned` | PD path (no WBC): `tuned` = 200 Hz C++ and `/joint_states`, policy clock starts at gantry release, looped reference, no head sim unless vision, skip isolated IMU acceleration spikes; `original` = previous defaults. Explicit arguments still override |
| `control_rate_hz` | `auto` | `cpp_control_node` scheduler rate; `auto`: 200 (tuned) or 50 (original) |
| `policy_imu_accel_outlier` | `auto` | `latch` a fault on an out-of-range IMU acceleration or `skip` isolated outliers; `auto`: skip with tuned PD |
| `enable_wbc` | `false` | Deprecated, see `docs/reports/2026-11-08-wbc-deprecation.md` |
| `cmd_timeout_s` | `0.5` | `cpp_control_node` timeout for its input commands |
| `can_command_timeout_s` | `0.5` | Simulated backend command lifetime (`python_can_node`'s `command_timeout_s`) |
| `fall_detection_enabled` | empty | Override `safety.yaml` fall detection (`true`/`false`) |
| `trip_on_imu_timeout` | empty | Override `safety.yaml` IMU-timeout trip |
| `localhost_only` | `1` | Confine DDS discovery to this host |
| `ros_domain_id` | `$ROS_DOMAIN_ID` or `0` | ROS domain for every node in the launch |
| `web_port` | `8080` | Web monitor port with `minimal:=false` |
| `motor_config_file` | installed `motors.yaml` | Motor registry for the backend, controller and policy |
| `safety_config_file` | installed `safety.yaml` | Safety supervisor configuration |
| `sim_config_file` | installed `config/sim.yaml` | Parameters of the nodes inside `mujoco_sim` |
| `model_path` | installed `20260914_170827.onnx` | Policy model |

### Keep simulated traffic off the robot

`sim_motor_backend` publishes `/joint_states` exactly like the CAN backend. On
the robot's domain and network that would feed a real controller simulated
feedback. The launch therefore sets `ROS_LOCALHOST_ONLY=1` by default (the dev
container also uses domain 42). Shells you use to interact with the simulation
need the same settings:

```bash
export ROS_LOCALHOST_ONLY=1
```

## Interacting with a running simulation

Every command below runs in another shell with the workspace sourced
(`source install/setup.bash`), `ROS_LOCALHOST_ONLY=1`, and the same
`ROS_DOMAIN_ID` as the launch.

### Windows

- **MuJoCo viewer** (`use_viewer`): a *passive* viewer. It only displays the
  physics through an independent process and model/data mirror. Only bounded,
  nonblocking latest-state queues connect the GUI to the physics process.
  GUI synchronization and its Python execution lock cannot block sensor or
  motor callbacks in `mujoco_sim`. `mujoco_sim` steps physics in real time, so its Run/Pause and
  reset controls are inactive and the simulation cannot be paused or slowed
  from it. The usual mouse controls move the camera, and MuJoCo's perturbation
  works: double-click a body, then Ctrl + right-drag to push it. Closing the viewer does not stop the
  simulation; stop the launch with Ctrl+C.
- **RViz** (`use_rviz`): the robot model posed from the measured
  `/joint_states`, with the base fixed; it does not show the robot moving
  across the floor.
- **Web monitor** at `http://127.0.0.1:8080` (`web_port`): motor status,
  telemetry and safety state. Its enable/disable/home buttons are rejected
  in the simulation, because the launch leaves `cpp_control_node`'s
  abstract actions disabled; use the commands below.

### Watching state

| Topic | Shows |
|---|---|
| `/sim/status` | Simulated time, real-time factor, gantry, estop, `physics_paused` and `control_fault` |
| `/sim/ground_truth/odom` | True base pose and velocity (world frame) |
| `/policy/status` | Policy state (`calibrating`, `running`, `fault`), reference frame, fault reason |
| `/joint_states`, `/motor_status`, `/motor_telemetry` | Simulated motor feedback, as from `python_can_node` |
| `/imu/data_raw`, `/head/imu`, `/head/mcu_status` | Simulated IMUs and head board status |
| `/safety/estop`, `/safety/status` | Safety supervisor latch and diagnostics |

For example `ros2 topic echo /policy/status`.

With `pause_on_control_fault: true` (default), loss of enabled, valid motor
control after release freezes physics at the failure pose and disables every
motor. This is a simulation diagnostic stop, not successful balance or physical
safety validation. IMU/joint publishers stop rather than re-stamping the frozen
sample as fresh. Inspect `/sim/status` and restart the launch to recover; traffic
alone never clears the pause. Set this simulator parameter to `false` only when
intentionally observing uncontrolled physics.

Safety still evaluates at 100 Hz. Its IMU subscription retains only the latest
sample (Reliable/Volatile, depth 1); ordinary diagnostics publish at 10 Hz.
The Reliable/Transient Local estop state publishes on initialization and changes,
so new subscribers still receive the current latch without a 100 Hz duplicate
stream. A new trip is sent before logging or diagnostic work.

In disconnected minimal mode, `/motor_commands` carries lifecycle commands, so its Hz is
not the policy-control rate. Measure `/policy/desired_motor_subset` instead.
Temperature/status JSON is disabled in the minimal loop; full mode publishes it
at 5 Hz separately from the 50 Hz joint feedback.

### Commands

| Action | Command |
|---|---|
| Release / re-engage the gantry | `ros2 service call /sim/hold_base std_srvs/srv/SetBool "{data: false}"` (`true` moves the base back to spawn first) |
| Respawn: spawn pose, gantry on, motors disabled | `ros2 service call /sim/reset std_srvs/srv/Trigger "{}"` |
| Clear the latched safety estop | `ros2 service call /safety/reset std_srvs/srv/Trigger "{}"` (refused while the robot is still tilted beyond the reset limits) |
| Enable or disable motors | `MotorCommand` with `mode: [3]` (ENABLE) or `[4]` (DISABLE) on `/motor_commands`; see the policy run below |
| Point the head | `ros2 topic pub -r 20 /head/servo_target_rad std_msgs/msg/Float32 "{data: 0.5}"` (rad, + is robot left; the servo releases 0.25 s after the stream stops) |

After a fall: reset the safety latch and the simulation, then restart the
launch. `tracking_policy_node` latches its own estop fault and only clears it
on restart.

### Driving joints by hand

Launch without the policy, so `cpp_control_node` listens on
`/desired_motor_subset`:

```bash
ros2 launch humanoid_sim sim.launch.py enable_policy:=false
```

Start a stream of commands first, then enable the joint. A bare ENABLE
expires after 0.5 s without motion commands, and `ros2 topic pub` takes
about a second to start publishing, so enabling first usually times out:

```bash
ros2 topic pub -r 20 /desired_motor_subset motor_control_interfaces/msg/MotorCommand \
  "{header: {stamp: now}, joint_name: [left_knee_joint], mode: [2], position: [-0.8], kp: [80.0], kd: [1.0]}"
```

```bash
ros2 topic pub --once /motor_commands motor_control_interfaces/msg/MotorCommand \
  "{header: {stamp: now}, joint_name: [left_knee_joint], mode: [3]}"
```

Mode 2 is MIT motion control: position in rad with the given stiffness and
damping. When the stream stops, the joint times out, goes limp and needs a
new ENABLE.

### Typical policy run

The tracking policy plays a 299-frame (~6 s) reference walk, starting the
moment its gyro calibration finishes, whether or not the motors are enabled;
afterwards it holds the last frame. To start the walk with the robot on the
floor, let the launch do it:

```bash
ros2 launch humanoid_sim sim.launch.py auto_start:=true
```

`sim_autostart` waits for the policy's first command (reference frame 0),
publishes ENABLE for the policy joints on `/motor_commands`, and calls
`/sim/hold_base false` `release_delay_s` later. It acts once; after a reset,
restart the launch (or follow the manual steps below).

Manually:

1. Wait for the policy to finish its 2 s gyro calibration (`/policy/status`).
   The gantry keeps the base still meanwhile.
2. Enable the motors through the same path as on hardware. For example, an
   enable for every joint directly to the backend:

   ```bash
   ros2 topic pub --once /motor_commands motor_control_interfaces/msg/MotorCommand \
     "{header: {stamp: now}, joint_name: [left_hip1_joint, left_hip2_joint, left_thigh_joint, left_knee_joint, left_ankle_joint, right_hip1_joint, right_hip2_joint, right_thigh_joint, right_knee_joint, right_ankle_joint], mode: [3]}"
   ```

   `--once` with `stamp: now` matters: commands older than 0.5 s are rejected.
3. Release the gantry when ready:

   ```bash
   ros2 service call /sim/hold_base std_srvs/srv/SetBool "{data: false}"
   ```

The reference motion starts when the policy finishes calibrating, not when the
motors are enabled, so enable promptly or restart the policy node. A bare
ENABLE also expires after 0.5 s without motion commands, as on hardware. A
fall trips the latched estop exactly as on hardware; reset with
`/safety/reset` and `/sim/reset`, and restart the policy node (its estop fault
is latched too).

### Current status

With `auto_start:=true` the policy → C++ → simulated motor chain runs.
Walking outcomes vary between launches; repeatable sustained locomotion is
not established. Feedback age, command scheduling and contact timing affect the
trained policy's closed loop. Validate stability under the deployed timing.

For the optional RL → WBC → C++ → MuJoCo path, current usage and limitations
are maintained in [wbc_control_node](../../humanoid_control/wbc_control_node/README.md).

### Simulator-only interfaces

| Name | Type | Purpose |
|---|---|---|
| `/sim/reset` | `std_srvs/Trigger` | Spawn pose, gantry engaged, all motors disabled |
| `/sim/hold_base` | `std_srvs/SetBool` | Engage (moves the base back to spawn) or release the gantry |
| `/sim/ground_truth/odom` | `nav_msgs/Odometry` | Base pose; twist is in the **world** frame |
| `/sim/status` | `std_msgs/String` | JSON: sim time, real-time factor, overruns, gantry, estop |

The `mujoco_sim` process can also be started alone, to drive the real nodes
from another launch:

```bash
ros2 run humanoid_sim mujoco_sim --ros-args \
  --params-file install/humanoid_sim/share/humanoid_sim/config/sim.yaml \
  -p use_viewer:=true
```

## Configuration

`config/sim.yaml` has one section per node in the process (`mujoco_sim`,
`sim_motor_backend`, `sim_body_imu`, `sim_head_mcu`, `sim_camera`). Joint
names, limits, gains and torque limits come from the motor registry
(`motor_control_hybrid/config/motors.yaml` by default; override with
`motor_config_file:=`). The registry's `hardware_verified` flag is not checked
because no hardware is opened.

Rendering uses `MUJOCO_GL` to pick an OpenGL backend: the default `glfw` needs
a display; use `MUJOCO_GL=egl` headless (`osmesa` needs libOSMesa). If no
backend works the camera node logs an error and the rest of the simulation
keeps running. With software rendering (no GPU, as in the dev container) the
640×480 camera reaches about 6 Hz instead of the configured 15 Hz.

### Vision

`enable_vision:=true` renders the head camera and starts the repository's
detector and `track_node` through `head_tracking.launch.py` (camera and Agent
disabled). For the detector to see a face, pass a PNG face image as
`target_image`; the board sweeps ±0.4 m sideways 1.5 m in front of the robot.
The tracker → `sim_head_mcu` → servo path is verified. In the dev container the
detector itself currently aborts while loading its ONNX model with the system
OpenCV 4.5.4 (the build expects the SDK from `scripts/setup_opencv45.sh`); that
is independent of the simulation. Use `start_detector:=false` to run the rest.

## Tests

```bash
cd simulation/humanoid_sim
python -m pytest test
```

`test_motor_model.py` checks the command rules without ROS or MuJoCo;
`test_world.py` checks spawn clearance, that the settled gantry passes the
policy's calibration thresholds, target tracking, limp motors, gantry release
and the head servo on the real robot model.

## Overnight fall-prediction data collection

Activate the workspace and run from `embedded/`:

```bash
export ROS_DISTRO=humble  # this host; use your installed distro elsewhere
source scripts/activate_rosenv.sh
python scripts/collect_fall_data.py --hours 6.5 \
  --output data/fall_collection/my_run
```

The output directory must be new. Default domain 87 is locked against another
collector and uses localhost-only DDS. Camera, depth, vision, viewer and RViz
are off. The existing tracking policy, C++ forwarding, motor model, body/head
IMUs and Safety supervisor remain active. Each bounded episode restarts the
whole launch, which resets policy calibration, motor enable and safety latches;
no hardware backend is launched. Standing-reference and walking-reference
trials alternate. Holding frame 0 is not a validated standing controller.

One third of trials receive no push. Others receive a world-horizontal force
on the floating base: 5–100 N, 50–250 ms, starting 0.25–2 s after release, with
seeded direction/magnitude/timing. Forces are applied at physics steps and
cleared after the pulse. A fall before the planned push is retained and its
actual applied step count recorded. The initial foot clearance is zero, so
release does not deliberately drop the robot.

**Collection-only change:** physics continues after Safety disables motors;
the usual diagnostic freeze is suppressed only when `collection_output_dir`
is set. Safety still trips, disables motors and rejects new motion. Outcomes
therefore describe the existing controller **with Safety**, not an unprotected
policy or the counterfactual effect of a new protective action. Normal launches
retain their original freeze behavior. The robot geometry/mass and peripheral
approximations documented above still apply; camera-off does not improve their
physical fidelity.

Episodes end at sustained non-foot ground contact (>1 N for 20 ms), followed by
350 ms of observation, or after 15 simulated seconds following release. Physics
is capped at real-time pace, not accelerated. Samples use the exact 100 Hz noisy
IMU values published by `sim_body_imu`, acquired with the same-step truth under
the world lock. Roll/pitch are radians; gyro rad/s; acceleration specific force
in m/s² (gravity included as measured by the existing sensor). Joint positions,
velocities, height, timestamps, estop and hold/fault state are also recorded.
Contact is checked every physics step, independently of the sensor timer.

Files:

- `status.json`: current PID, progress, valid label counts and final state.
- `episodes.jsonl`: completion/failure, mode, command and metadata per trial.
- `episode_*/samples.jsonl`: raw observations; `features` has the eight inputs.
- `episode_*/metadata.json`: seed, push, fall time and label counts.
- `episode_*.log`, `ros_logs/`: process diagnostics.
- `run.json`, `source/`: arguments, Git state and source/config snapshots.

For a row with label 0 or 1, use that row and its preceding 19 rows as the input.
Label 1 means a confirmed fall occurs in the next 300 simulated milliseconds;
label 0 means the full horizon was observed without a fall. Label `null` means
unknown/excluded: gantry/warmup, invalid or discontinuous history, already fallen,
or an incomplete future horizon. Do not convert null to zero. Sample spacing is
checked against [5,20] ms; timestamps are preserved for stricter resampling.
Only episodes with `valid: true` in the manifest should enter the initial
training dataset. Split whole episodes before constructing train/test windows.

The runner uses a 6.5-hour monotonic deadline including startup/reset overhead,
with shutdown grace up to ~16 s. It stops early on five consecutive unusable
or crashed episodes, or less than 2 GiB free disk. Each episode is buffered in
memory and written atomically at shutdown; a hard crash can lose the current
episode, while completed episodes remain. Stop gracefully using `kill -TERM`
on the PID in `status.json`; do not kill individual ROS nodes. Readiness checks
and data quality still determine the actual usable dataset size.

Short smoke test (fresh output path):

```bash
python scripts/collect_fall_data.py --hours 0.1 --episode-seconds 8 \
  --max-episodes 3 --output /tmp/fall_collection_smoke
```

## CycloneDDS repeated-start test

The existing default ROS environment is left unchanged. `scripts/run_cyclonedds_test.sh`
activates a local Cyclone overlay in `.deps/cyclonedds/root/opt/ros/humble` and sets
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` for its process tree. Official arm64 ROS
Debian packages are downloaded and extracted locally when system installation
is unavailable; versions/hashes for this run are in `.deps/cyclonedds/packages.json`.
Required packages are rmw-cyclonedds-cpp, cyclonedds and the three iceoryx packages.
No development build or system-wide middleware switch is required.

```bash
bash scripts/run_cyclonedds_test.sh --hours 2 --max-episodes 100 \
  --max-consecutive-failures 100 --episode-seconds 20 \
  --profile recovery --seed 48048 --domain 89 \
  --output data/fall_collection/cyclonedds_100
```

The local XML disables multicast and lists 127.0.0.1 for peer discovery, with an
automatic participant index. Humble's ROS_LOCALHOST_ONLY=1 already selects the
loopback interface, so the XML does not add a duplicate interface entry.
Configuration reference: [CycloneDDS 0.10.5](https://cyclonedds.io/docs/cyclonedds/0.10.5/config/cyclonedds_specifics.html).

Every simulator episode records `get_rmw_implementation_identifier()` in metadata;
the collector excludes episodes if this differs from the requested RMW. The
policy, C++ forwarding, motor model, IMUs and Safety remain active. The process
environment selects the same RMW for all ROS nodes. Camera/viewer are off.
The above seeds and 20-second duration match the first 100 recovery-profile
scenarios from the earlier run, but scheduling and run conditions differ, so
physical outcome differences cannot be attributed solely to DDS.

After the run, `dds_summary.json` and `DDS_REPORT.md` summarize attempts, valid
trajectories, actual RMW checks, node crashes, observed IMU intervals and physics
overruns. This tests complete-stack startup/communication, not a new trained-model
accuracy result. The two-hour deadline may finish early at 100 attempts. A run
that exhausts its time budget must be checked for actual completed attempt count.
`status.json` and `episodes.jsonl` preserve all failures; no failure is silently
replaced by an extra successful trial. Two preliminary smoke trials are separate
from the formal 100 attempts.
