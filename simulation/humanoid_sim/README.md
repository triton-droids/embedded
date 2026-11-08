# humanoid_sim — MuJoCo humanoid simulation

Application publishers and subscribers in the simulation/chase/control chain
use KeepLast=2, including camera, policy, WBC, C++, Web and Safety. Reliable and
best-effort choices are preserved; estop remains reliable/transient-local, now
with depth 2. ROS-internal discovery, parameter events and service queues are
not topic history settings changed by this tuning. Source timestamps, measured joint
velocity, IMU angular velocity and controller gains are unchanged. Damping uses
`kd * (dq_ref - dq)` from measured velocity, not differences between DDS history
entries. Safety freshness limits are unchanged.

Body IMU publication uses a dedicated single-thread executor by default
(`dedicated_imu_executor:=true`), with diagnostics on the other executor.
`/imu/status` exposes `publish_thread`, `publish_native_tid` and `qos_depth` for
runtime readback. This isolates callbacks but still shares the Python GIL and
physics lock; it is not a process-isolation or realtime guarantee.

### Timing isolation and transport tuning

Three opt-out switches exist because the measured failure was a publish call, not
compute: one `rclpy` publish waited 331.9 ms on an Imu message while a physics
step costs 0.05 ms and the sample lock microseconds.

- `imu_split:=true` (default) publishes `/imu/data_raw` from `sim_imu_publisher`,
  a separate process. The physics thread writes one fixed-size record per step to
  a memory-mapped segment (`humanoid_imu_snapshot.bin`, seqlock-protected) and
  only that process publishes, so a slow `/joint_states` write cannot delay the
  IMU. Acquisition still happens in the simulator, so a stall there still freezes
  the sample stamp. `imu_split:=false` restores the in-process publisher.
  `/imu/status` reports `separate_process` so the active mode is visible at
  runtime.
- `dds_tuning:=on` sets `FASTRTPS_DEFAULT_PROFILES_FILE` to
  `config/fastdds_profile.xml`: larger history resource limits, a
  `PREALLOCATED_WITH_REALLOC` memory policy and a capped `max_blocking_time`, so
  one slow write degrades to a dropped state sample instead of a ~300 ms stall.
  Reliability and history depth are untouched, so the QoS contract is unchanged.
  Off by default; compare `max_publish_call_s` in `/imu/status` between runs.
- `gc_freeze` (default `true`, a `mujoco_sim` parameter) runs `gc.freeze()` once
  after startup so the long-lived model, registry and nodes are not rescanned by
  every cyclic collection. New objects are still collected normally. Set
  `-p gc_freeze:=false` to compare `max_gc_pause_s` with `gc_diagnostics:=true`.


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
| Body IMU MCU + micro-ROS Agent | `sim_body_imu` (or `sim_imu_publisher` with `imu_split:=true`) | `/imu/data_raw` (frame `imu_link`, 100 Hz), `/imu/status` |
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
the first fresh command (`sim_autostart.startup_timeout_s` node parameter); the
WBC launch uses 10 s to accommodate cold DDS discovery and native imports.
After ENABLE, autostart keeps watching the fresh policy stream. The simulator
allows release only when all motors have accepted motion commands with positive
position gains and at least 40 ms remaining lifetime. A not-yet-ready backend
can be checked again within the startup budget, without repeating ENABLE.
An aborted start sends DISABLE and keeps the gantry engaged; it never re-arms.
After success or failure, autostart drains its reliable command publisher and
exits, releasing its high-rate command subscriptions.

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
| `gil_switch_interval_s` | `0.0` | `mujoco_sim` Python GIL switch interval; `0` keeps the 5 ms default |
| `ros_domain_id` | `$ROS_DOMAIN_ID` or `0` | ROS domain for every node in the launch |
| `web_port` | `8080` | Web monitor port with `minimal:=false` |
| `motor_config_file` | installed `motors.yaml` | Motor registry for the backend, controller and policy |
| `safety_config_file` | installed `safety.yaml` | Safety supervisor configuration |
| `sim_config_file` | installed `config/sim.yaml` | Parameters of the nodes inside `mujoco_sim` |
| `model_path` | installed `20260914_170827.onnx` | Policy model |

### Experimental launch

`sim_experimental.launch.py` is `sim.launch.py` with timing fixes that are
still being evaluated. It accepts every argument above and passes it through:

```bash
scripts/sim_experimental.sh                          # sources rosenv, checks, launches
scripts/sim_experimental.sh enable_vision:=true start_detector:=false
ros2 launch humanoid_sim sim_experimental.launch.py  # same, without the checks
```

| Argument | Default | What it changes |
|---|---|---|
| `rmw_implementation` | `rmw_cyclonedds_cpp` | DDS of every node of the launch |
| `cyclonedds_config` | installed `config/cyclonedds_sim.xml` (`cyclonedds_sim_humble.xml` when `ROS_DISTRO=humble`) | Loopback interface only, up to 16 MB socket receive buffer; skipped if `CYCLONEDDS_URI` is already set |
| `gil_switch_interval_s` | `0.0005` | Python GIL switch interval of `mujoco_sim` |

#### What the `sim-experimental-cyclonedds` branch changes

The branch starts from `909d0f9` (`fix(sim): remove catch-up bursts`) plus the
local revert commit; it does **not** contain PR #62 or #64, which were merged
into `origin/humanoid_ros2_simulation` afterwards.

| File | Change | Effect on `sim.launch.py` |
|---|---|---|
| `launch/sim_experimental.launch.py` | New: sets the three arguments above, then includes `sim.launch.py` | none |
| `config/cyclonedds_sim.xml` | New: CycloneDDS on `lo` only, receive buffer up to 16 MB | none |
| `config/cyclonedds_sim_humble.xml` | New: same receive buffer without the `lo` pinning, chosen on Humble | none |
| `scripts/sim_experimental.sh` | New: one-click start with environment checks | none |
| `tools/timing/` | New: measurement tools, see [tools/timing/README.md](tools/timing/README.md) | none |
| `humanoid_sim/main.py` | GIL switch interval is a `mujoco_sim` parameter (`gil_switch_interval_s`) | default `0` keeps the old behaviour |
| `launch/sim.launch.py` | New `gil_switch_interval_s` argument forwarded to `mujoco_sim` | default `0.0`, unchanged |
| `package.xml`, `setup.py` | `rmw_cyclonedds_cpp` dependency; install `config/*.xml` | none |

`sim.launch.py` still runs on Fast DDS with the default switch interval, so the
two launches can be compared on the same build.

#### Why

Measured on native Ubuntu 24.04 with an i7-1165G7 (robot held on the gantry):

- **Fast DDS stalls.** About 2 s after another node joins the graph (most often
  one with service clients of the simulator: `cpp_control_node`,
  `sim_autostart`, `ros2 service call`), a Fast DDS 2.14 `publish()` can block
  for ~330 ms. rclpy holds the GIL there, so the whole process stops, the
  simulator or any other Python node, and the safety IMU freshness check
  (250 ms) or the policy input timeout (100 ms) latches. Startup estops: about
  30 % with Fast DDS, 0 of 25 with CycloneDDS. Shared memory off, asynchronous
  publishing and staggered node start did not remove it. The root cause inside
  Fast DDS is not known.
- **Loopback only.** With `ROS_LOCALHOST_ONLY=1` CycloneDDS still advertises an
  arbitrarily chosen external interface. On a Wi-Fi address in 100.64.0.0/10
  with Tailscale running, that address is routed into `tailscale0` and no node
  discovered any other. The config pins CycloneDDS to `lo`. On Humble,
  `rmw_cyclonedds_cpp` itself adds `127.0.0.1` for `ROS_LOCALHOST_ONLY=1` and
  selects only `lo`; pinning `lo` again fails every node with "the same
  interface may not be selected twice" / "failed to create domain". The launch
  and scripts therefore use `cyclonedds_sim_humble.xml` (receive buffer only)
  when `ROS_DISTRO=humble`.
- **Camera frames.** CycloneDDS fragments the 0.6–0.9 MB images over UDP. With
  the default 208 KB `net.core.rmem_max`, 7–14 % of color and 2–9 % of depth
  frames were lost in transport; with 16 MB, none. The request is capped by
  `net.core.rmem_max`, so raise that on the host (the script warns when it is low).
- **GIL switch interval.** With CPython's 5 ms default the 5 ms physics step can
  wait a full period for the executor or camera thread. 0.5 ms cut physics
  overruns from 82–339 to 8–13 per 30 s.

#### Testing it step by step

1. Get the branch:

   ```bash
   git fetch origin sim-experimental-cyclonedds
   git switch -c sim-experimental-cyclonedds origin/sim-experimental-cyclonedds
   ```

2. Install CycloneDDS and raise the socket receive buffer limit (once per host;
   in the dev container rosdep installs the package):

   ```bash
   sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
   echo 'net.core.rmem_max=33554432' | sudo tee /etc/sysctl.d/60-cyclonedds.conf
   sudo sysctl --system
   ```

3. Build. On hosts without the OpenCV 4.5 SDK add `-DTRACK_ONLY=ON` (tracker
   only); if the rosenv lacks `setuptools`, build the interface packages with
   the system Python:

   ```bash
   source scripts/activate_rosenv.sh
   colcon build --symlink-install --packages-up-to humanoid_sim humanoid_vision \
     --cmake-args -DTRACK_ONLY=ON -DPython3_EXECUTABLE=/usr/bin/python3
   ```

4. Start it once by hand and check the first lines: `Experimental simulation:
   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and no `SAFETY ESTOP LATCHED`:

   ```bash
   scripts/sim_experimental.sh enable_vision:=true start_detector:=false enable_depth:=true
   ```

5. Measure both launches with the tools. Each writes `summary.md` and
   `environment.txt` under `local_test/sim_timing/<tool>_<launch>_<host>_<time>/`:

   ```bash
   T=simulation/humanoid_sim/tools/timing
   LAUNCH=sim $T/startup_trials.sh 10      # original: startup estops
   $T/startup_trials.sh 10                 # experimental
   LAUNCH=sim $T/dds_join_repro.sh 6       # minimal Fast DDS stall reproduction
   $T/dds_join_repro.sh 6
   $T/image_loss.sh 3                      # camera frame loss (experimental)
   $T/camera_matrix.sh                     # feedback timing with/without camera
   ```

   About 25 minutes in total. Close CPU-heavy applications first; they change
   the CPU and overrun numbers, not the stall counts.

6. Report the `summary.md` and `environment.txt` files, the CPU model, and
   whether the host is native Linux or WSL.

Shells that interact with this simulation should use the same DDS and config:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(ros2 pkg prefix humanoid_sim)/share/humanoid_sim/config/cyclonedds_sim.xml
# On Humble use cyclonedds_sim_humble.xml instead (see "Loopback only" above).
```

Known issues, not addressed by this launch:

- With `auto_start:=true` the robot falls 2–9 s after the gantry release in
  about half the runs, with either DDS and no IMU timeout.
- `scripts/setup_rosenv.sh` fails on a fresh host: `requirements-wbc.txt` pins
  `cmeel-eigen==3.4.0`, which is no longer on PyPI.
- Only measured on one native Ubuntu host so far (not WSL, not the i9-12900HX).

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
at 5 Hz separately from the 50 Hz joint feedback (200 Hz in WBC mode).
The Web monitor consumes `/sim/monitor/joint_states` at 10 Hz, controlled by
`sim_motor_backend.monitor_rate_hz`; it retains the original feedback stamp and
stops when physics is fault-paused. Controllers continue using `/joint_states`.
The simulation launch enables asynchronous Fast DDS publication for the entire
stack, including Safety, Web and policy, to reduce blocking in diagnostic writes.
Simulation-only `/safety/status` uses best-effort depth 1 (all built-in consumers
request best-effort); `/safety/estop` remains reliable and transient-local. The
standalone hardware safety launch retains reliable diagnostics by default.
This does not guarantee nonblocking writes under overload or relax any deadline.

In WBC mode, `dedicated_command_executor:=true` creates a lightweight
`sim_motor_command_rx` node inside the same simulator process. Its single-thread
executor receives reliable motor commands and latched estop independently of the
sensor timer executor. Both use the existing world lock; command queue depth and
ENABLE/DISABLE semantics are preserved. Set the argument to `false` for an A/B
timing comparison. This uses no general-purpose Python worker pool.

IMU, joint feedback, odometry and foot feedback use the wall-clock acquisition
time recorded by the physics step, rather than each ROS callback's execution
time. Republishing an unchanged physical sample does not refresh its source age.
Odometry and foot contacts are copied under one lock from the same step.

`/imu/status` includes `last_publish_age_s`, `max_publish_gap_s`,
`max_sample_wait_s`, and `control_fault`. These distinguish a publishing gap,
sample acquisition delay, and an intentional stop after a control fault. Viewer
state transfer and synchronization run at 5 Hz; the passive native window can
still redraw between state updates.

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
the world lock (collection therefore forces `imu_split:=false`). Roll/pitch are radians; gyro rad/s; acceleration specific force
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

## Render on another computer

The optional TCP viewer moves visualization to a separate computer. The simulation
computer runs physics, policy, WBC, C++ control and Safety without a local GUI.
The viewer receives the compiled model (including mesh/texture assets) once per
connection, then latest-only integration states at 15 Hz by default. It does not
run physics integration, requires no ROS installation, and sends no motor,
mouse-force, reset or release commands back to the simulation. Local camera
rotation/zoom remains available. One viewer connection is supported at a time;
restart the client to reconnect after a disconnect or simulator restart.

On the simulation computer, activate the workspace as usual, then run:

```bash
source scripts/activate_rosenv.sh
ros2 launch wbc_control_node rl_wbc_sim.launch.py \
  auto_start:=true use_viewer:=false remote_viewer:=true \
  remote_viewer_host:=0.0.0.0 remote_viewer_port:=8765 \
  remote_viewer_rate_hz:=15.0
```

This explicitly exposes TCP port 8765 to the network. Use a trusted LAN only:
this read-only protocol has no authentication or encryption. It does not require
changing `ROS_LOCALHOST_ONLY` or exposing ROS discovery. Use `remote_viewer:=true`
with `humanoid_sim sim.launch.py` in the same way for the PD path. The stream is
independent of the existing motor Web monitor.

On the display computer (Windows, Linux or macOS), copy just
`simulation/humanoid_sim/humanoid_sim/remote_viewer.py` from this checkout, then:

```bash
python -m pip install "mujoco==3.3.7" numpy
python remote_viewer.py --host SIMULATION_COMPUTER_IP --port 8765
```

The installed MuJoCo version must exactly match the server. Check the server with
`python -c "import mujoco; print(mujoco.__version__)"` in the activated workspace
and replace the pinned version above if necessary. Compiled MJB compatibility
also depends on MuJoCo's build/platform; cross-machine compatibility has only
been tested on the local platform so far. A mismatched version is rejected before
loading the binary model. On Windows, `py` can be used instead of `python`. On
macOS, launch the GUI with `mjpython remote_viewer.py --host ... --port 8765`.
If the display computer already has this package on its Python path, use
`python -m humanoid_sim.remote_viewer` instead of copying the file.

To keep the server bound to loopback, omit `remote_viewer_host:=0.0.0.0` above.
On the display computer open a separate terminal and keep this tunnel running:

```bash
ssh -N -L 8765:127.0.0.1:8765 USER@SIMULATION_COMPUTER_IP
```

Then connect the viewer with `--host 127.0.0.1 --port 8765`. The SSH server must
already be available on the simulation computer. Allow TCP 8765 through the
simulation computer's firewall only when using the direct LAN option.

To verify reception without a display:

```bash
python remote_viewer.py --host SIMULATION_COMPUTER_IP --headless --frames 30
```

Sampling skips a busy physics lock, and network IO runs in a separate process
with a one-element snapshot queue and bounded socket timeouts. A missing, slow,
or disconnected viewer does not hold up the control path. There is still a small
state-copy and sampler scheduling cost on the simulation host; no hard real-time
or IMU-timeout improvement is guaranteed. The state stream rate may be set in
`(0, 60]` Hz. Keep `use_viewer:=false` to avoid also rendering locally. A busy port
or sender startup failure is logged while the simulation continues headless.
