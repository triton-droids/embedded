# Safety and ROS network monitors

The safety package now owns the live motor status Web UI and the RViz joint
relay. Web styling comes from the existing `sdk_prototype/webui` status table;
the new backend uses ROS subscriptions only. Legacy SDK/CAN tuner tools remain
separate. Monitoring modes do not start CAN, model inference, a fake motor, or a
motor command publisher. They do not start a second safety supervisor implicitly.

| `mode` | Processes |
|---|---|
| `supervisor` (unchanged default) | Safety supervisor |
| `web` | Read-only HTTP status node |
| `rviz` | JointState relay, robot_state_publisher, RViz2 |
| `monitor` | Web and RViz views |
| `all` | Supervisor and both views; only use where this robot's supervisor belongs |

Build from the workspace root using existing ROS dependencies:

```bash
source /opt/ros/jazzy/setup.bash  # Use the distribution installed on each host.
colcon build --packages-select humanoid_safety humanoid_leg_description motor_control_hybrid
source install/setup.bash
```

## ROS network / domain

Both the robot's state publisher and the monitor must run in the same ROS domain
with network discovery permitted. The modes inherit the current RMW/DDS/peer
configuration; they do not change interfaces, firewall rules, or motor settings.
No custom `ros_net` command is needed. For a LAN setup, before starting processes
on **both** computers:

```bash
export ROS_DOMAIN_ID=20                  # Example: use your robot's actual domain.
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

Use synchronized host clocks; telemetry age is computed from source timestamps.
Domain selection is not authentication or encryption. Routed/VPN deployments
may need explicit peer configuration supplied by the existing ROS network setup.
Use matching ROS and RMW versions. The launch also accepts `ros_domain_id:=20`;
it applies to all launched child processes, including RViz.

## Web on a PC receiving robot DDS feedback

```bash
ros2 launch humanoid_safety safety.launch.py mode:=web source_topic:=/joint_states
```

Open <http://127.0.0.1:8080>. The browser uses HTTP to the local monitor; the
monitor obtains robot data over ROS DDS. To serve browsers on the LAN, explicitly
pass `host:=0.0.0.0`, then visit that computer's IP and port 8080.

The UI shows rad, rad/s, effort Nm, source age, online/stale state, and the last
reported safety estop. Missing values are unknown, not zeros. Stale values stay
visible but are marked offline. Offline is not proof of physical stopping.
The node has a bounded per-joint cache and a single HTTP owner with bounded socket
waits; browser polling allows only one outstanding request.

If installed, `motor_control_hybrid/config/motors.yaml` supplies expected names,
IDs/models and the legacy `/motor_status` enumeration-index mapping. Override
using `motor_config_file:=/absolute/path/motors.yaml` when the robot uses another
registry. Without a registry, only observed joints appear, ID/model/temperature
remain unknown. Temperature has receipt age only because that JSON topic has no
source timestamp; it never makes joint feedback fresh. Missing data is not polled
from a motor SDK. POST/control requests are rejected.

For the isolated policy bench, pass `source_topic:=/policy/joint_states` and, if
applicable, `motor_status_topic:=/policy/motor_status`. `/policy/target_angles`
shows **commanded**, not measured, pose; do not label it actual motor feedback.

## RViz on the display PC

```bash
ros2 launch humanoid_safety safety.launch.py mode:=rviz source_topic:=/joint_states
# Both views:
ros2 launch humanoid_safety safety.launch.py mode:=monitor source_topic:=/joint_states
```

The safety-owned `joint_state_monitor_node` validates finite samples and source
age, preserves the source header, and relays to
`/safety/visualization/joint_states`. A local robot_state_publisher supplies TF to
RViz, which loads the local leg URDF/meshes. The manual joint GUI is disabled.
Do not feed this visualization topic back into control. RViz holds the last pose
when data stops; consult web freshness or ROS diagnostics for liveness. The base
is fixed in world; IMU attitude/base translation are not estimated here.

Only run one robot_state_publisher for this robot in the domain. If it already
runs elsewhere, use `start_display:=false` for relay-only mode and open your
existing RViz display. For a headless TF check, use `use_rviz:=false`.
The old `motor_control_hybrid` relay executable remains a compatibility wrapper.

Monitoring software cannot make ROS discovery or clocks correct automatically.
Check received topics on the display PC before diagnosing empty views. Physical
cross-host discovery, hardware telemetry, and graphical RViz still require live
verification on the target network.

## Software verification

From a sourced workspace with system pytest available:

```bash
python3 -m pytest -q humanoid_control/humanoid_safety/test/test_motor_monitor.py \
  humanoid_control/humanoid_safety/test/test_checks.py
ROS_DOMAIN_ID=184 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/safety-monitor-test-logs \
  RUN_ROS_INTEGRATION=1 python3 -m pytest -q \
  humanoid_control/humanoid_safety/test/test_motor_monitor_integration.py
```

Use an unused domain for integration. It publishes only synthetic JointState
feedback, runs `mode:=monitor use_rviz:=false`, checks HTTP values/expiry, the
source-preserving relay and expected hip TF rotation, and asserts that no motor
command publisher exists. It also verifies that HTTP command POST is rejected.
This tests headless TF, not the graphical RViz renderer or cross-host discovery.
