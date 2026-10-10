# Live joint visualization over ROS 2

`joint_state_monitor_node` forwards a selected `sensor_msgs/msg/JointState`
stream to `/joint_states`. It preserves names, positions, timestamps and optional
velocity/effort arrays, drops malformed/non-finite samples, and stops publishing
when its source stops. It never sends motor commands. Do not run another publisher
on its output topic, or use that visualization stream as motor feedback.

The default source is `/policy/joint_states` (motor feedback). For the current
disabled fake-motor bench this stays at zero. Select `/policy/target_angles` to
see the policy's **commanded** pose instead. These targets are not measured robot
positions. The current tracking export holds its last frame after six seconds.
The URDF base stays fixed in `world`; joint visualization does not estimate the
robot's base motion or apply IMU attitude.

## Robot / Jetson

Build once from the repository root, after building the policy dependencies:

```bash
source /opt/ros/humble/setup.bash
source rosenv/bin/activate
colcon build --symlink-install --base-paths humanoid_control humanoid_leg_description \
  --packages-select motor_control_hybrid humanoid_leg_description
source install/setup.bash
```

Both computers must use the same ROS domain, and network discovery must be
enabled. The commands below use domain 0 (the ROS default). Set these variables
before starting the policy, monitor or RViz. Use the same ROS distribution and
RMW implementation on both machines; synchronize their system clocks.

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch motor_control_hybrid joint_state_monitor.launch.py \
  source_topic:=/policy/target_angles
```

For feedback, use `source_topic:=/policy/joint_states`. For a hardware feedback
publisher, pass its actual topic. If it already publishes `/joint_states`, RViz
can consume it directly without this relay.

## RViz computer

Copy/checkout this repository so the meshes are available locally. Build the
description package, then start its display with the manual joint GUI disabled:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
colcon build --symlink-install --base-paths humanoid_leg_description \
  --packages-select humanoid_leg_description
source install/setup.bash
ros2 topic echo /joint_states sensor_msgs/msg/JointState --once
ros2 launch humanoid_leg_description display.launch.py \
  use_joint_state_gui:=false joint_states_topic:=/joint_states
```

This starts `robot_state_publisher` locally to turn incoming joint positions into
TF and opens RViz with the mesh model, fixed frame `world`, and a transient-local
robot-description subscription. Only run one robot state publisher in this
domain for this robot. For a headless TF check, append `use_rviz:=false`.

ROS 2 DDS carries the joint messages across the network; no websocket is needed.
Both computers need a reachable LAN with multicast discovery and DDS UDP traffic
permitted. If the topic is absent, compare `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`
and `RMW_IMPLEMENTATION`, restart `ros2 daemon` after changing them, and check
network/firewall settings. Discovery across routed VPNs may require explicit DDS
peer configuration.

References: [ROS 2 environment settings](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.rst),
[robot_state_publisher](https://github.com/ros/robot_state_publisher/tree/humble).

## Fake motor verification

If `ros2 run motor_control_hybrid joint_state_monitor_node` reports
`No executable found`, rebuild and source the workspace using the build commands
above. An older install may not contain this newly added executable.

In a sourced terminal, start a dedicated fake feedback stream:

```bash
ros2 launch motor_control_hybrid joint_state_monitor.launch.py \
  use_fake_motor:=true source_topic:=/pose_test/feedback
```

This uses the ten robot joint names from `config/motors.yaml`. The standalone
fake motor defaults to `test_joint` and `test_joint2`, which are absent from the
leg URDF. Start the RViz display as above with its manual joint GUI disabled.
In another sourced terminal, enable and move one fake joint:

```bash
ros2 topic pub --once /pose_test/motor_commands motor_control_interfaces/msg/MotorCommand \
  '{joint_name: [left_hip1_joint], mode: [3]}'
ros2 topic pub --once /pose_test/motor_commands motor_control_interfaces/msg/MotorCommand \
  '{joint_name: [left_hip1_joint], mode: [1], position: [0.4], velocity: [1.0]}'
```

The left hip should move to 0.4 radians. Fake motors start disabled, so a position
command alone will not move them. Do not run a second feedback publisher on
`/pose_test/feedback` or a second relay on `/joint_states`.

For an automated headless check of moving feedback, unchanged relay samples and
the resulting TF rotation, run from the sourced repository root:

```bash
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 RUN_ROS_INTEGRATION=1 \
  /usr/bin/python3 -m pytest -q \
  humanoid_control/motor_control_hybrid/test/test_pose_monitor_integration.py
```

Use an unused domain ID. This test requires permission to create local DDS
sockets; restricted execution sandboxes may deny them.
