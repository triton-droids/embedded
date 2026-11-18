# Workspace launch entry points

Run from the repository root. Packages in this workspace share the generated
`install/setup.bash` after a successful colcon build; a launch directory alone
is not a colcon package.

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch launch/head_tracking.launch.py
```

`head_tracking.launch.py` includes the installed `humanoid_vision` launch without
copying its configuration. All component switches and arguments remain available:

```bash
ros2 launch launch/head_tracking.launch.py --show-args
```

The package entry point also remains available:

```bash
ros2 launch humanoid_vision head_tracking.launch.py
```

See `humanoid_vision/TRACKING.md` for camera, micro-ROS Agent, detector and tracker
requirements. The full vision build requires OpenCV 4.5.x and the ROS dependencies
listed in its package manifest. Importing the package does not install these SDKs.

For tracker-only development without the detector SDKs:

```bash
colcon build --symlink-install --packages-select humanoid_vision --cmake-args -DTRACK_ONLY=ON
source install/setup.bash
ros2 launch launch/head_tracking.launch.py start_detector:=false
```

The tracker-only launch still starts the camera and Agent by default. Disable
`start_camera` and/or `start_agent` when those components run elsewhere.

## Leg display

```bash
colcon build --symlink-install --packages-select humanoid_leg_description humanoid_arm_description
source install/setup.bash
ros2 launch launch/display.launch.py
```

Display defaults to `humanoid_leg_description/urdf/human_offset_corrected.urdf`.
The existing `ros2 launch humanoid_arm_description display.launch.py` entry also
forwards to this leg display. MoveIt demo continues to use the arm model.
For live joint states, pass `use_joint_state_gui:=false`.

## Unified system (without MoveIt)

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch launch/system.launch.py
```

Starts real CAN, C++ control, safety supervisor, camera, detector, head tracker,
micro-ROS Agent, leg robot state publisher and RViz. SDK/HTTP gateways, pendulum WebSocket UI, policy
bridge, legacy IMU and MoveIt remain disabled. Joint GUI defaults off to avoid
publishing manual states alongside hardware states. Leg rendering requires
matching leg joint names on `/joint_states`; the launch does not translate arm
or fake test joints into leg joints.

The unified launch requests SCHED_RR 20 for CAN TX, SCHED_RR 21 for each CAN RX, and SCHED_RR 10 for
the C++ executor (timer and subscriptions together). Other nodes keep normal
scheduling. These are initial experiment values, not measured deadlines.
Set `enable_rt:=false` to disable both requests. The launching user's
`ulimit -r` must be at least 21; permission failures warn and continue. Scheduling
requests do not guarantee startup ordering or hardware readiness.

Use `enable_vision:=false` or `enable_display:=false` to omit subsystems. Use `--show-args` for options.
For tracker-only builds, pass `start_detector:=false`; camera and Agent still
require their own installed packages. No hardware process is started by building.

### Independent CAN transport

The control package, interfaces, control core and safety supervisor are aligned
with the `Humanoid_ros2` transport implementation. RX continuously waits for
frames on one thread per interface. TX sends without waiting for feedback;
new commands wake it immediately, while hold-last repetition uses a deadline
wait. `control_rate_hz` remains 50 Hz by default for control, publication and TX
repetition; it does not throttle incoming feedback. The old `feedback_poll_hz`
argument in the child launch is compatibility-only and is not a system option.

Use `can_tx_rt_priority:=20 can_rx_rt_priority:=21 control_rt_priority:=10`
to tune the independent scheduling requests. Rebuild all affected packages and
source the new setup after this update; the motor command interface has changed.
