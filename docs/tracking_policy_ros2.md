# Leg tracking policy on ROS 2 (no SDK)

This integration runs the `humanoid_locomotion_v3` tracking ONNX export at
**50 Hz (20 ms per control period)**. It reads the existing attitude-sensing
package's ROS messages and publishes ten target joint angles. The default launch
uses the physical ESP32-S3 IMU at `/dev/ttyACM0`, 460800 baud, and zero joint-state
placeholders because no motors or encoders are connected.

## Data path

```mermaid
flowchart LR
  ESP[ESP32-S3 / six-axis IMU] -->|SI JSON / USB serial| Reader[imu_reader_node]
  Reader -->|/imu/data_raw| Policy[tracking_policy_node / ONNX / 50 Hz]
  Feedback[JointState feedback / optional fake node] --> Policy
  Policy --> Angles[/policy/target_angles / radians]
  Policy --> Desired[/policy/desired_motor_subset]
  Desired --> CPP[optional cpp_control_node]
  CPP --> Monitor[/policy/motor_commands / isolated bench]
```

No SDK client or gateway, CAN node, websocket, or motor-enable operation is part
of this launch. The existing Torch bridge remains available for its own model
format; its velocity-command observations and action clipping do not match this
tracking export.

## Install and build

On Ubuntu 22.04 / ROS 2 Humble, with the sibling `simulation` repository and its
LFS model already downloaded:

```bash
cd ~/Github/embedded
ROS_DISTRO=humble bash scripts/setup_tracking_policy.sh
source /opt/ros/humble/setup.bash
source rosenv/bin/activate
source install/setup.bash
ros2 launch motor_control_hybrid tracking_policy.launch.py
```

Keep the IMU stationary for the initial two-second gyro-bias calibration.
The setup installs only `requirements-policy-onnx.txt` and builds the four ROS
packages needed for this path. It does not install Torch or the SDK requirements.
It uses the ROS distribution's `/usr/bin/python3` and NumPy 1.x for Humble's
native module compatibility. `rosenv/COLCON_IGNORE` prevents colcon from trying
to build Python dependency test examples inside the virtual environment.

In each additional terminal, source ROS, `rosenv/bin/activate`, and
`install/setup.bash`, then inspect:

```bash
ros2 topic echo /policy/target_angles
ros2 topic echo /policy/status
ros2 topic hz /policy/target_angles --window 5000
```

Target positions are in **radians**; multiply by `180/pi` to display degrees.
`/policy/actions` contains the ten raw policy actions.
`/policy/desired_motor_subset` uses the existing `MotorCommand` message, with
ten names, motion mode, target positions, zero desired velocity/torque, and the
model's simulation stiffness/damping metadata. These gains are telemetry values,
not validated physical-actuator gains. `/policy/status` includes inference time,
callback time, tick interval, frame index, feedback mode, IMU receipt age, faults,
and model SHA256. Statistics cover the latest 3000 successful ticks.

## Test the existing C++ command path

```bash
ros2 launch motor_control_hybrid tracking_policy.launch.py \
  use_fake_joint_states:=true joint_feedback_mode:=joint_states \
  run_cpp_control:=true
```

This uses all ten policy joint names, routes fake feedback through
`/policy/joint_states`, and remaps the C++ output to `/policy/motor_commands`.
Fake motors start disabled and receive no enable command; this verifies message
transport and scheduler behavior, not actuator dynamics or walking stability.
On an upstream timeout the existing C++ watchdog publishes disable messages on
the isolated bench topic.

For a separate IMU publisher, set `start_imu_reader:=false imu_topic:=/your/imu`.
For real joint feedback, set `joint_feedback_mode:=joint_states
joint_states_topic:=/joint_states`. Every message must contain all ten policy
joint names with finite positions **and velocities**, in radians and radians/s;
ordering is mapped by name. Missing velocities are not silently replaced with
zeros. The no-motor default explicitly labels its feedback as `zero`.

## Exact model contract

Default model:
`~/Github/simulation/logs/legs_tracking/20260914_170827/20260914_170827.onnx`

The inputs are `obs: float32[1,56]` and `time_step: float32[1,1]`. Observation
normalization is already embedded in the ONNX graph.

| Observation slice | Meaning | Units |
| --- | --- | --- |
| 0:10 | Embedded reference joint position | rad |
| 10:20 | Embedded reference joint velocity | rad/s |
| 20:23 | Bias-corrected body angular velocity | rad/s |
| 23:33 | Measured joint position minus default position | rad |
| 33:43 | Measured joint velocity | rad/s |
| 43:53 | Previous **raw** policy action | action units |
| 53:56 | Projected world down in body frame | unit vector |

Joint order is left hip1, hip2, thigh, knee, ankle, then the same five right joints.
Names come from model metadata. Target position is
`default_joint_pos + action_scale * action`: this export has zero offsets and
`action_scale=0.2` rad per action unit. Actions are not restricted to `[-1,1]`;
clamping them here would change the learned policy.

Reference motion advances at 50 frames/s according to elapsed monotonic time,
so delayed callbacks do not replay an outdated frame. The embedded reference has
299 frames; after frame 298, the last frame is held, rather than wrapping across
an untrained discontinuity. Long-run measurements after this point assess
runtime timing, not continued walking-motion tracking.

## IMU, frames, and input faults

The ESP32-S3 JSON format is:

```json
{"t_us":5000,"seq":1,"accel_mps2":[0,0,9.80665],"gyro_rad_s":[0,0,0]}
```

The existing reader now recognizes these SI keys, rejects missing/non-finite
samples instead of publishing zeros, and handles the 32-bit microsecond counter
wrap. Older `acc`/`gyro` JSON and BNO085 CSV parsing are preserved.

The default policy estimator propagates the down vector with gyro measurements
and corrects it using acceleration when its magnitude is near 1 g. Upright
gravity is `[0,0,-1]`. This is a six-axis bench estimator; translational
acceleration can distort its attitude correction. Identity IMU-to-body mounting
is the bench assumption, not a verified robot installation.

Direct-node parameters support a row-major `imu_to_body_rotation` (orthonormal,
determinant +1), an expected `imu_frame`, and `orientation_source:=message` for
an external attitude estimator. A message quaternion must be normalized and
represent world-from-sensor orientation; unavailable orientation (`covariance[0]
< 0`) is rejected. Gyro and gravity are rotated into the policy's body frame.
Use the training model's body-axis conventions when calibrating the mounting.

The reader's ROS timestamp is **host publication time**, not synchronized IMU
capture time. Receipt age and ROS timestamp age are both checked, but neither
measures the complete sensor-to-control latency. Device timestamps are used for
serial integration intervals; hardware clock synchronization remains future work.

Stale IMU/feedback (>100 ms), invalid frames/values, failed calibration, or a
latched `/safety/estop` stop policy output. Faults latch until node restart. Timing
uses a steady clock; message headers use ROS time. This is a normal Linux/ROS 2
executor, not a hard real-time scheduler. Callback overruns and actual interval
jitter must both be measured.

## Physical deployment boundary

Zero/fake feedback and an unmounted IMU can produce angles outside physical
joint limits. `/policy/*` outputs are diagnostic targets, not proof of stable
physical control. This launch does not publish `/motor_commands` or
`/desired_motor_subset`. The synchronized `motors.yaml` defines the ten policy
leg joints with unverified hardware fields. Before routing commands to hardware,
complete the ten-joint motor IDs, encoder mapping, signs, offsets, limits,
appropriate gains, mounting calibration, command limits and validated stop path.
No guessed mappings or SDK implementation are added by this integration.

## Synchronized motor registry

`motor_control_hybrid/config/motors.yaml` now stores the ten joints in the active
tracking export's order, its Kp/Kd, zero default positions, action scale 0.2,
50 Hz contract, and simulation joint limits. The launch reads this order for
fake feedback. On startup the policy checks the registry against the model's
SHA256 and metadata, including the motor gain entries; configuration drift
requires resynchronization.

```bash
source rosenv/bin/activate
python scripts/sync_tracking_motor_config.py
```

The sync reads the sibling simulation export and its saved training XML. It
preserves existing hardware mapping fields for matching joint names and marks
the result `hardware_verified: false` after every sync. CAN startup refuses this
registry until the hardware fields are completed and explicitly verified. CAN
IDs, interfaces, models, master IDs, directions, encoder offsets and hardware
torque limits are unset because simulation does not establish them.

The original test_joint/test_joint2 configuration is preserved in
`config/bench_motors.yaml`; `control_config.yaml` remains the separate arm
configuration. `policy_bridge_config.json` remains the separate legacy Torch
model contract and is not consumed by the tracking launch.

The CAN driver's encoder convention is `q_joint = direction *
(q_motor - encoder_offset_rad)`. Commands use the inverse transform, apply
configured position/velocity limits and optional hardware torque limits, and
default missing command gains to per-motor Kp/Kd. Explicit command gains still
take precedence. Cached feedback older than `feedback_timeout_s` (default 0.1 s)
is excluded, and JointState names follow registry order. All mappings are checked
before any bus connection. These changes have software tests; physical hardware
calibration and validation remain outstanding.

Run the configuration/CAN transformation tests without physical CAN I/O:

```bash
source /opt/ros/humble/setup.bash
source rosenv/bin/activate
source install/setup.bash
python -m unittest discover -s humanoid_control/motor_control_hybrid/test \
  -p test_motor_configuration.py -v
```

## Native ROS 2 frequency test and logs

The environment has to be sourced in **every** terminal:

```bash
cd ~/Github/embedded
source /opt/ros/humble/setup.bash
source rosenv/bin/activate
source install/setup.bash
```

Before launching, check `ls -l /dev/ttyACM0` and `fuser /dev/ttyACM0`.
Only the reader should use this serial port: close serial monitors or standalone
policy benches before starting the launch. If the port has another name, pass
`port:=/dev/ttyACM1` (or the actual name) to the launch command.
Keep the IMU still during calibration and wait for the status to become running.

With the launch running in terminal A, check status and one target message in
terminal B before starting a frequency capture:

```bash
ros2 node list
ros2 topic echo /policy/status std_msgs/msg/String --once --field data --full-length
ros2 topic echo /policy/target_angles sensor_msgs/msg/JointState --once --full-length
```

Ten named `position` values are target angles in radians, not measured motor
angles. `/policy/actions` contains raw model outputs. The expected status has
`state=running`, `fault=null`, `requested_hz=50`, `hardware_output=false`, and
`joint_feedback=zero` for the default launch. The C++/fake launch instead reports
`joint_feedback=joint_states`.

Use native CLI tools for 60-second captures; run each command in its own sourced
terminal if simultaneous measurements are desired:

```bash
mkdir -p ~/policy-test-logs
timeout --signal=INT 60s ros2 topic hz /policy/target_angles --window 5000 \
  > ~/policy-test-logs/target_angles_hz.txt 2>&1
timeout --signal=INT 60s ros2 topic hz /imu/data_raw --window 5000 \
  > ~/policy-test-logs/imu_hz.txt 2>&1
# Only when the optional C++ path is launched:
timeout --signal=INT 60s ros2 topic hz /policy/motor_commands --window 5000 \
  > ~/policy-test-logs/motor_commands_hz.txt 2>&1
ros2 topic echo /policy/status std_msgs/msg/String --once --field data --full-length \
  > ~/policy-test-logs/status_after.txt
tail -n 4 ~/policy-test-logs/target_angles_hz.txt
```

The commands shown together above run sequentially in one terminal; use separate
terminals to compare the topics over the same time interval. `timeout` normally
returns exit code 124 after its time limit. Use new filenames for each run.
For an interactive live display, omit `timeout` and the output redirection and
stop `ros2 topic hz` with Ctrl+C. Avoid concurrent builds/tests and continuous
full-message printing during a timing baseline. The default launch does not
start a custom observer.

Expect approximately 50 Hz for policy/C++ topics and 200 Hz for this IMU firmware.
`hz` reports receiving rate; `max` is the largest receiving interval in its
window. `max=0.028s` means 28 ms, even when the average is 50 Hz. Its extrema are
rounded to milliseconds. A window of 5000 samples covers about 100 seconds at
50 Hz and 25 seconds at 200 Hz; a 60-second policy test contains roughly 3000
samples after CLI discovery.

Also inspect `inference_ms`, `control_callback_ms`, and `tick_interval_ms` in
status. `callback_overruns_20ms=0` only means each successful callback's measured
work took at most 20 ms; it does **not** mean every callback started on time.
Check status again after the capture: C++ can keep publishing disable messages
after policy failure, so its frequency alone cannot validate the policy.

On `IMU timeout`, check IMU frequency, port ownership, cable and reader logs.
On `JointState timeout`, check `/policy/joint_states` and the selected feedback
mode. Calibration failure requires a stationary IMU during restart. Faults latch;
after resolving the cause, stop the launch with Ctrl+C and relaunch. There is no
automatic reset. Stop all test terminals with Ctrl+C when finished and check
`fuser /dev/ttyACM0` before another program uses the port.

The current export is restricted to 50 Hz by the node because its reference and
previous-action observations use that timing. After about 5.96 seconds it holds
the final reference frame; longer tests evaluate runtime timing, not a repeating
walking motion. See [measured results](tracking_policy_performance.md).

## Regression tests

```bash
cd ~/Github/embedded
source /opt/ros/humble/setup.bash
source rosenv/bin/activate
source install/setup.bash
python -m pip install 'pytest>=7,<9'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q \
  humanoid_control/motor_control_hybrid/test/test_tracking_onnx.py \
  humanoid_control/motor_control_hybrid/test/test_tracking_node.py \
  Attitude_Sensing/src/attitude_sensing_pkg/test/test_imu_read.py
```

These check the actual model observation/action contract, name reordering,
partial/non-finite feedback rejection, down-vector sign, timestamp freshness,
ESP32 SI parsing, and microsecond rollover. Model-dependent tests skip when the
external LFS export is absent; parsing and math checks still run.
