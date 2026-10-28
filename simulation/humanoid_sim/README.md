# humanoid_sim — MuJoCo stand-ins for the robot's hardware

Runs the robot software stack without hardware. One `mujoco_sim` process owns
the physics and hosts one ROS node per replaced peripheral; every other node is
the deployed implementation, started unmodified.

| Hardware process | Replaced by | Interface kept |
|---|---|---|
| `python_can_node` (RobStride CAN) | `sim_motor_backend` | `joint_states`, `motor_status`, `motor_telemetry`, `can_timing`, `motor_commands`, `/safety/estop`, `motor_backend/get_feedback`, `motor_backend/execute_operation` |
| Body IMU MCU + micro-ROS Agent | `sim_body_imu` | `/imu/data_raw` (frame `imu_link`, 100 Hz), `/imu/status` |
| Head MCU + micro-ROS Agent | `sim_head_mcu` | `/head/imu`, `/head/mcu_status`, `/head/servo_target_rad` (250 ms watchdog) |
| `realsense2_camera_node` | `sim_camera` | `/camera/camera/color/{image_raw,camera_info}`, optional aligned depth |

Started unmodified by `sim.launch.py`: `cpp_control_node`, `safety_node`,
`motor_web_node`, `joint_state_monitor_node`, `robot_state_publisher` + RViz,
`tracking_policy_node`, and with `enable_vision:=true` the detector and
`track_node`. `encos_can_node` is not replaced; the existing launches do not
select it either.

## Quick start

In the dev container, after [Setup](#setup):

```bash
source install/setup.bash
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

Inside the dev container (or any ROS 2 Humble/Jazzy host), install MuJoCo into
the workspace virtualenv, then build:

```bash
source rosenv/bin/activate
pip install -r simulation/humanoid_sim/requirements.txt
colcon build --symlink-install --packages-up-to humanoid_sim
source install/setup.bash
```

The policy additionally needs `requirements-policy-onnx.txt`.

## Run

```bash
ros2 launch humanoid_sim sim.launch.py
```

This opens the MuJoCo viewer and RViz, starts the Web monitor on
`127.0.0.1:8080`, and holds the robot on a virtual gantry: a weld with the
soles 5 cm above the floor (`foot_clearance_m`). Closer to the floor the limp
legs fold against it and keep shaking the base, which fails the policy's
stationary gyro calibration. After each (re)spawn the physics is pre-rolled
for 2 s (`settle_s`) so the legs already hang still when the nodes start.
All motors start **disabled**; nothing in the launch enables them unless
`auto_start:=true` is set.

The robot stack starts only once the simulator logs `Simulation ready`, so
the safety supervisor's 3 s IMU startup grace is not spent while MuJoCo
loads. If `mujoco_sim` exits, the whole launch shuts down.

Launch arguments (`ros2 launch humanoid_sim sim.launch.py --show-args`
prints the same list):

| Argument | Default | Effect |
|---|---|---|
| `use_viewer` / `use_rviz` | `true` | GUI windows; set `false` for headless runs |
| `enable_policy` | `true` | Run `tracking_policy_node` on simulated IMU and joint feedback |
| `connect_policy` | `true` | Remap `cpp_control_node` input to `/policy/desired_motor_subset`; `false` leaves it on `/desired_motor_subset` for manual commands |
| `auto_start` | `false` | Enable the motors and release the gantry when the policy starts its reference motion |
| `release_delay_s` | `0.1` | With `auto_start`: delay from enable to gantry release |
| `start_base_held` | `true` | Start on the gantry |
| `calibration_seconds` | `2.0` | Policy gyro calibration; keep the base held meanwhile |
| `enable_vision` | `false` | Render the head camera and start detector + head tracker |
| `start_detector` | `true` | Set `false` for `TRACK_ONLY` builds |
| `use_cuda` | `false` | Detector backend |
| `enable_depth` | `false` | Also render aligned depth (needed for the detector's 3D output) |
| `target_image` | empty | PNG image for the target board (MuJoCo only decodes PNG) |
| `control_rate_hz` | `50.0` | `cpp_control_node` scheduler rate |
| `cmd_timeout_s` | `0.5` | `cpp_control_node` timeout for its input commands |
| `can_command_timeout_s` | `0.5` | Simulated backend command lifetime (`python_can_node`'s `command_timeout_s`) |
| `fall_detection_enabled` | empty | Override `safety.yaml` fall detection (`true`/`false`) |
| `trip_on_imu_timeout` | empty | Override `safety.yaml` IMU-timeout trip |
| `localhost_only` | `1` | Confine DDS discovery to this host |
| `ros_domain_id` | `$ROS_DOMAIN_ID` or `0` | ROS domain for every node in the launch |
| `web_port` | `8080` | Web monitor port |
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
  physics, which `mujoco_sim` steps in real time, so its Run/Pause and
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
| `/sim/status` | Simulated time, real-time factor, physics overruns, gantry, estop |
| `/sim/ground_truth/odom` | True base pose and velocity (world frame) |
| `/policy/status` | Policy state (`calibrating`, `running`, `fault`), reference frame, fault reason |
| `/joint_states`, `/motor_status`, `/motor_telemetry` | Simulated motor feedback, as from `python_can_node` |
| `/imu/data_raw`, `/head/imu`, `/head/mcu_status` | Simulated IMUs and head board status |
| `/safety/estop`, `/safety/status` | Safety supervisor latch and diagnostics |

For example `ros2 topic echo /policy/status`.

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

With `auto_start:=true` the full chain runs (policy at 50 Hz →
`cpp_control_node` → simulated motors). Outcomes vary between launches:
sometimes the robot walks part of the reference, often it falls within a
step or two of release. Running the same policy and world in a direct loop
(scripts not in the repository) separates the causes:

- With the policy acting on the current state every 20 ms, it walks the
  reference (about 2.3 m along +Y in 6 s), including after the 5 cm drop from
  the gantry. The world model, IMU frame and motor law are therefore
  consistent with training.
- Adding the deployed chain's delays breaks it. Joint feedback up to 20 ms
  old (`/joint_states` at 50 Hz) and up to 20 ms of command delay
  (`cpp_control_node` forwards on its own 50 Hz tick) each degrade tracking;
  with 10 ms or more of both, most trials fall within 1.5 s, and outcomes
  vary run to run. Hardware adds CAN latency on top, so this is likely a
  property of the policy (trained without latency) rather than of the
  simulation.
- The three 50 Hz loops (policy, `cpp_control_node`, `/joint_states`) are
  not synchronized, so these delays are set by process start times and differ
  between launches, which fits the launch-to-launch variation. Touchdown
  phase and contact sensitivity add to it.
- Rigid foot impacts produce accelerometer samples above 100 m/s². The
  policy node rejects those as invalid and latches a fault; the real IMU's
  range and filtering would not report them this way.

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
