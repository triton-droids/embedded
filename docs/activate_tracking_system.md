# Activate the tracking policy and RViz monitor

This guide starts the ESP32/MPU6050 reader, 50 Hz ONNX policy, ten fake motors,
JointState network relay, and RViz display. It also describes the current
real-motor feedback workflow and the remaining requirements for physical motion.

Commands use the current Jetson checkout at
`/home/darin/Documents/Github/embedded` and ROS 2 Humble. On another machine,
replace checkout paths with its local paths. Run each numbered terminal in a
separate shell. Stop previous launches before starting replacements.

## 1. Install and build once on the Jetson

Prerequisites: ROS 2 Humble, colcon, Python venv support, and the sibling
`simulation` checkout with the actual ONNX export downloaded (not an LFS pointer).

```bash
cd /home/darin/Documents/Github/embedded
bash scripts/setup_tracking_policy.sh

source /opt/ros/humble/setup.bash
source rosenv/bin/activate

colcon build --symlink-install \
  --base-paths humanoid_control humanoid_leg_description \
  --packages-select motor_control_hybrid humanoid_leg_description \
  --cmake-args \
  -DPython3_EXECUTABLE=/home/darin/Documents/Github/embedded/rosenv/bin/python

source install/setup.bash
```

The checked-in `config/motors.yaml` is synchronized with export
`20260914_170827`. The tracking node checks its SHA256, joint order, offsets,
action scale and PD gains against the loaded model. Its hardware mapping fields
are unset, and `hardware_verified: false` allows fake operation while rejecting
CAN activation.

## 2. Source every Jetson terminal

Paste this block into **each** Jetson terminal before its ROS commands:

```bash
cd /home/darin/Documents/Github/embedded
source /opt/ros/humble/setup.bash
source rosenv/bin/activate
source install/setup.bash

export PYTHONPATH="$PWD/rosenv/lib/python3.10/site-packages:$PYTHONPATH"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_LOG_DIR=/tmp/embedded-policy-ros-logs
```

The Python path makes the serial dependency available to the IMU executable,
whose interpreter is system Python. Use matching ROS domain and RMW settings on
the Jetson and RViz computer. Domain 0 is used throughout this guide; change it
on both computers if your robot network uses another domain. Synchronize their
system clocks.

## 3. Terminal A — start the real IMU and fake-motor policy

Check the serial port first:

```bash
ls -l /dev/ttyACM0
fuser /dev/ttyACM0
```

Only one process may read ACM0. Close serial monitors and stop any old policy
reader before proceeding. The current device was observed transmitting CSV at
115200 baud; despite its name, the existing `bno085_csv` parser accepts this
MPU6050 telemetry layout. These flags are specific to the currently installed
firmware.

Keep the IMU stationary during the initial two-second calibration:

```bash
ros2 launch motor_control_hybrid tracking_policy.launch.py \
  model_path:=/home/darin/Documents/Github/simulation/logs/legs_tracking/20260914_170827/20260914_170827.onnx \
  port:=/dev/ttyACM0 \
  baud:=115200 \
  input_format:=bno085_csv \
  use_fake_joint_states:=true \
  joint_feedback_mode:=joint_states \
  run_cpp_control:=true
```

This starts `imu_reader_node`, `tracking_policy_node`, `/policy/fake_motor_node`
and `cpp_control_node`. Commands are isolated on `/policy/motor_commands`; this
launch does not start CAN or enable physical motors.

If you install the repository's `imu_process/policy_imu` SI JSON firmware, use
`baud:=460800 input_format:=json` instead. Do not upload firmware just to start
the current working CSV setup.

For the simpler stationary-feedback bench, use `run_cpp_control:=false` and skip
the fake-enable step below. You can still visualize policy targets.

## 4. Terminal B — check status and optionally enable fake movement

After sourcing the environment block:

```bash
ros2 topic echo /policy/status std_msgs/msg/String \
  --once --field data --full-length
```

Wait for `state: running`, `fault: null`, `requested_hz: 50`,
`joint_feedback: joint_states`, and `hardware_output: false`.

Fake motors start disabled and publish zero positions. To make them follow the
policy through the C++ relay, send this **once**, after the policy is running:

```bash
ros2 topic pub --once /policy/motor_commands \
  motor_control_interfaces/msg/MotorCommand \
  "{joint_name: [left_hip1_joint, left_hip2_joint, left_thigh_joint, left_knee_joint, left_ankle_joint, right_hip1_joint, right_hip2_joint, right_thigh_joint, right_knee_joint, right_ankle_joint], mode: [3]}"
```

Mode 3 is enable; mode 4 is disable. This command targets the fake-only topic.
Do not change it to a physical motor command topic. Fake motion is a simple
rate-limited target follower, not a dynamics or hardware-gain validation.

The embedded reference lasts approximately six seconds and then holds its final
frame. Continued publication does not imply a repeating walking cycle. Restart
the policy to run the reference from its beginning.

## 5. Terminal C — start the JointState network relay

Choose one source. To display the fake motors' simulated positions:

```bash
ros2 launch motor_control_hybrid joint_state_monitor.launch.py \
  source_topic:=/policy/joint_states \
  output_topic:=/joint_states
```

To display the commanded policy pose instead, stop that relay and run:

```bash
ros2 launch motor_control_hybrid joint_state_monitor.launch.py \
  source_topic:=/policy/target_angles \
  output_topic:=/joint_states
```

Run only one relay publishing `/joint_states`. Target angles are commands, not
measured motor positions. The relay preserves original timestamps, rejects
malformed/non-finite samples, and publishes only when new source messages arrive.

## 6. RViz computer — display incoming joint states

Check out this repository on the RViz computer so its URDF and meshes are
available locally. Build the description package once:

```bash
cd /path/to/embedded
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths humanoid_leg_description \
  --packages-select humanoid_leg_description
```

In the display terminal, source the environment and confirm network reception:

```bash
cd /path/to/embedded
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 topic echo /joint_states sensor_msgs/msg/JointState --once

ros2 launch humanoid_leg_description display.launch.py \
  use_joint_state_gui:=false \
  joint_states_topic:=/joint_states
```

This starts `robot_state_publisher` and RViz. The supplied RViz configuration
uses fixed frame `world` and the robot mesh model. Only one robot state publisher
should serve this robot in the ROS domain. The manual joint GUI is disabled so
it does not compete with live input. Joint visualization keeps the base fixed;
it does not apply IMU attitude or estimate whole-body translation.

If running RViz on the Jetson itself, use the Jetson environment block and the
same display launch command; a separate description build is already covered
by step 1. For a headless TF check, append `use_rviz:=false`.

## 7. Verify the complete data path

In another sourced Jetson terminal:

```bash
ros2 node list
ros2 topic echo /policy/target_angles sensor_msgs/msg/JointState --once
ros2 topic echo /policy/joint_states sensor_msgs/msg/JointState --once
ros2 topic echo /joint_states sensor_msgs/msg/JointState --once
ros2 topic echo /policy/status std_msgs/msg/String \
  --once --field data --full-length
ros2 topic hz /policy/target_angles --window 500
```

Stop the `hz` command with Ctrl+C. Check the other rates separately:

```bash
ros2 topic hz /joint_states --window 500
# With run_cpp_control:=true:
ros2 topic hz /policy/motor_commands --window 500
```

Expect approximately 50 Hz for policy, fake feedback and relay output. Check
policy status after measuring; motor command frequency alone can include
watchdog disable messages after a fault. Inspect both callback duration and tick
interval jitter. The runtime uses ordinary Linux/ROS scheduling, not hard real time.

## 8. Real motors — configuration and feedback commissioning

Physical policy motion is not ready to activate from the synchronized simulation
registry. Simulation supplies joint names, gains and limits, but not physical
CAN IDs, encoder calibration or validated hardware gains.

Before starting CAN, complete and verify the ten-joint entries in
`humanoid_control/motor_control_hybrid/config/motors.yaml`: CAN interface and ID,
motor model, master ID, direction, encoder offset and applicable hardware limits.
Review the simulation gains and limits against the actual robot. Set
`hardware_verified: true` only after that review. The driver validates mappings
before opening any bus; null or conflicting mappings are rejected. Configure
the actual CAN interface and bitrate according to the installed hardware.

Encoder conversion is `q_joint = direction * (q_motor - encoder_offset_rad)`.
The driver applies the inverse conversion to commands. It uses per-motor gain
defaults, lets explicit command gains take precedence, and excludes feedback
older than 100 ms by default. The tracking node requires finite position **and
velocity** for all ten names.

Stop the fake policy and its relay before this alternate workflow. These commands
commission real feedback while keeping policy commands isolated.

Terminal A, with the Jetson environment sourced:

```bash
python -m pip install robstride-dynamics
ros2 run motor_control_hybrid python_can_node --ros-args \
  -p motor_config_file:="$PWD/humanoid_control/motor_control_hybrid/config/motors.yaml" \
  -p publish_rate_hz:=50.0 \
  -p feedback_poll_hz:=50.0 \
  -r joint_states:=/hardware/joint_states \
  -r motor_commands:=/hardware/unconnected_motor_commands
```

Terminal B, with the Jetson environment sourced:

```bash
ros2 launch motor_control_hybrid tracking_policy.launch.py \
  model_path:=/home/darin/Documents/Github/simulation/logs/legs_tracking/20260914_170827/20260914_170827.onnx \
  port:=/dev/ttyACM0 baud:=115200 input_format:=bno085_csv \
  use_fake_joint_states:=false \
  joint_feedback_mode:=joint_states \
  joint_states_topic:=/hardware/joint_states \
  run_cpp_control:=false
```

Terminal C, with the Jetson environment sourced:

```bash
ros2 launch motor_control_hybrid joint_state_monitor.launch.py \
  source_topic:=/hardware/joint_states \
  output_topic:=/joint_states
```

Use the same remote RViz command from step 6. These commands send no policy
commands to hardware and perform no enable operation. Physical actuation still
requires the commissioned command route, appropriate hardware gains, mounting
calibration and validated watchdog/stop behavior. The tracking launch verifies
its simulation registry against ONNX metadata; independently tuned hardware
gains need an explicit hardware-side command adaptation rather than changing
the model contract or forwarding simulation gains unchanged.

## 9. Stop and restart

For moving fake motors, first send disable from a sourced terminal:

```bash
ros2 topic pub --once /policy/motor_commands \
  motor_control_interfaces/msg/MotorCommand \
  "{joint_name: [left_hip1_joint, left_hip2_joint, left_thigh_joint, left_knee_joint, left_ankle_joint, right_hip1_joint, right_hip2_joint, right_thigh_joint, right_knee_joint, right_ankle_joint], mode: [4]}"
```

Then stop the policy launch, monitor and RViz terminals with Ctrl+C. Stop any
feedback-only CAN terminal as well. Verify ACM0 is free before restarting:

```bash
fuser /dev/ttyACM0
ros2 node list
```

If a launch leaves child processes behind, inspect the process tree and stop
the specific remaining processes before relaunching. Avoid starting duplicate
readers, fake feedback publishers or policy nodes. Policy faults latch until
restart; resolving the underlying problem alone does not resume output.

## Troubleshooting and resynchronization

| Symptom | Check |
| --- | --- |
| `waiting_for_imu` | ACM0 ownership, firmware baud and format; current firmware needs 115200 CSV |
| Missing `serial` dependency | Source rosenv and export its site-packages in PYTHONPATH |
| Calibration failure | Keep the IMU still, then restart the launch |
| `JointState timeout` | Fake node/real feedback is publishing all ten names with velocities; check duplicate or stale processes |
| Registry/model mismatch | Use the matching export or resynchronize the registry below |
| CAN rejects simulation registry | Hardware mapping is incomplete/unverified; fake mode remains available |
| RViz receives no joint states | Matching domain/RMW, ROS_LOCALHOST_ONLY=0, multicast discovery and DDS UDP traffic |
| RViz model is missing | Source the description package, keep meshes locally, fixed frame world, and check robot_state_publisher |
| RViz stays at zero | Fake feedback is disabled; enable fake movement or display policy targets |
| Motion stops changing after six seconds | Expected hold of the final reference frame |

After changing ROS networking variables, run `ros2 daemon stop` and
`ros2 daemon start` in that environment. Routed VPNs may need explicit DDS peer
configuration; a reachable IP alone does not guarantee discovery.

To refresh the registry after choosing a different matching tracking export:

```bash
cd /home/darin/Documents/Github/embedded
source rosenv/bin/activate
python scripts/sync_tracking_motor_config.py \
  --model /absolute/path/to/tracking_export.onnx
```

The export directory must also contain its saved `chrobot_16kg_actuated.xml` and
`chrobot_16kg_candidate.xml`. The sync preserves existing hardware mapping fields
for matching names but resets `hardware_verified: false`; review it again before
CAN use. Rebuild `motor_control_hybrid` if using a copied install. Supply the same
new model path to the tracking launch. No resync is needed for ordinary starts.

More detail: [tracking policy contract](tracking_policy_ros2.md),
[remote RViz](remote_rviz.md), and [configuration audit](motor_configuration_audit.md).
