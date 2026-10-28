# Safety and ROS network monitors

The safety package now owns the live motor status Web UI and the RViz joint
relay. Web styling comes from the existing `legacy/sdk_prototype/webui` status table;
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

The advisory self-collision monitor has its own launch file,
`self_collision.launch.py`; see [docs/self_collision_monitor.md](../../docs/self_collision_monitor.md).
It publishes warnings only and is never part of the latched supervisor.

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

Select a joint in the table or selector to inspect its feedback and review
**Home** or **Set Zero**. Home means moving to the calibrated zero; Set Zero
means making the current physical position the motor's mechanical zero.
The reviews describe the intended control_core workflow, including pausing and
decelerating body motion, verifying the zero write, rebinding coordinates and
resuming previously authorized control. ROS joint position is not raw encoder
position and cannot verify a mechanical zero write.

These action panels are currently **preview only**. Execute remains disabled:
the HTTP node still serves read-only feedback and rejects POST requests. No
action is submitted, and no motor enable or calibration state is inferred from
telemetry. The [command design](../../docs/robstride_abstract_command_design.md)
describes the backend integration that remains to be implemented.

If installed, `motor_control_hybrid/config/motors.yaml` supplies expected names
and CAN identities. Override using `motor_config_file:=/absolute/path/motors.yaml`.
Every configured motor appears even before feedback arrives. The configuration
tables show all per-motor YAML fields and global/model parameters. The server
checks the selected file once per second and reloads changes without a UI or node
restart. The page reports the exact file path, last successful load time and
reload errors. Invalid or temporarily missing files retain the last valid table;
changed motor identities or coordinate mappings invalidate cached feedback.
This reload changes monitoring only, not the running motor controller's settings.
To watch the source checkout rather than an installed copy, use:

```bash
ros2 launch humanoid_safety safety.launch.py mode:=web motor_config_file:="$PWD/humanoid_control/motor_control_hybrid/config/motors.yaml"
```

The Web monitor consumes `/motor_telemetry` (String JSON schema version 1), not
legacy enumeration-index `/motor_status`. Each record contains `joint_name`,
`can_interface`, `motor_id`, `model`, `source_stamp` (ROS seconds) and
`temperature_c`. Registry order is irrelevant: identity fields must match the
local registry for temperature to be joined. Without a registry, identity comes
from the named telemetry. Missing/mismatched telemetry is unknown. The legacy
SDK topic is unchanged; older CAN publishers must be upgraded to supply named
temperature telemetry.

The CAN publisher captures the source stamp when feedback is accepted, correcting
for its known host-receipt age. Re-publishing a cached sample preserves that
stamp. This is a host-derived feedback timestamp, not proof of synchronized motor
capture. Temperature needs both fresh source time and monotonic receipt time;
replayed stamps cannot refresh it or make JointState feedback online.

Supervisor liveness is tracked separately through stamped `/safety/status`
diagnostics (configurable with `supervisor_status_topic`). Only a fresh
`humanoid_safety/supervisor` summary with `hardware_id=humanoid` renews liveness.
The UI/API retain `last_reported_estop` as history; `estop` becomes null when the
supervisor heartbeat or Bool receipt expires. `supervisor_online` means fresh
heartbeat, not healthy checks: level/message are displayed separately. Paused
ROS clocks and replayed diagnostics cannot keep a dead supervisor online.
Freshness uses `state_timeout_s` (default 0.5 seconds).

For the isolated policy bench, pass `source_topic:=/policy/joint_states` and, if
applicable, `motor_status_topic:=/policy/motor_telemetry`. `/policy/target_angles`
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

## Safety checks layout

`safety_node` is the only publisher of `/safety/estop`. It turns each ROS
message into a plain observation and hands it to `SafetySupervisor`, which
passes it to every check and latches the estop if any check trips. The checks
live in `humanoid_safety/checks/`, one file per sensor area:

| File | Checks | Input |
|---|---|---|
| `base.py` | `SafetyCheck` base class, observation types, `FreshnessCheck` | |
| `supervisor.py` | `SafetySupervisor` latch and reset | |
| `imu.py` | fall, IMU freshness, IMU stuck | `/imu/data_raw` |
| `motors.py` | temperature, fault bits, torque, stale feedback | `/motor_telemetry`, `/joint_states` |
| `joints.py` | position and velocity limits | `/joint_states` |
| `comms.py` | CAN transport fault or silent motor node | `/can_timing` |
| `power.py` | battery voltage | `/battery_state` |

Only the IMU checks are on by default; turn the others on in
`config/safety.yaml` once tested. The node subscribes only to topics an
enabled check uses.

## Software verification

From a sourced workspace with system pytest available:

```bash
python3 -m pytest -q humanoid_control/humanoid_safety/test/test_motor_monitor.py \
  humanoid_control/humanoid_safety/test/test_checks.py \
  humanoid_control/humanoid_safety/test/test_hardware_checks.py \
  humanoid_control/humanoid_safety/test/self_collision/test_checker.py \
  humanoid_control/humanoid_safety/test/self_collision/test_pinocchio_backend.py
ROS_DOMAIN_ID=184 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/safety-monitor-test-logs \
  RUN_ROS_INTEGRATION=1 python3 -m pytest -q \
  humanoid_control/humanoid_safety/test/test_motor_monitor_integration.py
```

Use an unused domain for integration. It publishes only synthetic JointState
feedback, runs `mode:=monitor use_rviz:=false`, checks HTTP values/expiry, the
source-preserving relay and expected hip TF rotation, and asserts that no motor
command publisher exists. It also verifies that HTTP command POST is rejected.
This tests headless TF, not the graphical RViz renderer or cross-host discovery.


### Supervisor IMU time contract

The supervisor requires a synchronized, nonzero `Imu.header.stamp`. At receipt,
source age must not exceed `imu_timeout_s` (default 0.25 s); future skew up to
`imu_future_tolerance_s` (default 0.01 s) is tolerated. Duplicate or decreasing
source stamps are discarded before either fall detection or freshness updates.
Invalid/unavailable quaternions retain the existing fall-detection fault behavior
and cannot refresh freshness. Source validation remains enabled when the receipt
watchdog is disabled; if `imu_timeout_s<=0`, source age uses the 0.25 s default.

Freshness expires when source age at receipt plus subsequent host monotonic
elapsed time exceeds the timeout. Startup grace, reset checks and the watchdog
timer also use steady time, so paused or jumping ROS time cannot suspend expiry.
ROS time is still used for source-age comparison and diagnostic header stamps.
After a backward epoch jump, samples remain rejected until their stamps again
exceed the last accepted stamp and satisfy source age. A recovered stream does
not clear the estop latch: explicit reset is still required.

Run callback/clock regression tests after sourcing ROS Jazzy:

```bash
PYTHONPATH=humanoid_control/humanoid_safety:$PYTHONPATH python3 -m pytest -q \
  humanoid_control/humanoid_safety/test/test_imu_time_contract.py
# Optional isolated DDS test; starts no motor/CAN nodes:
ROS_DOMAIN_ID=84 RUN_ROS_INTEGRATION=1 \
  PYTHONPATH=humanoid_control/humanoid_safety:$PYTHONPATH python3 -m pytest -q \
  humanoid_control/humanoid_safety/test/test_imu_time_contract.py
```

## Unified control, safety and model display

The existing `joint_state_monitor_node` is the pose display relay; no second
angle-conversion node is needed. The CAN node already converts raw motor angles
through polarity, encoder offset and ankle linkage mapping into named ROS joint
angles in `/joint_states`. The relay forwards those positions and source stamps
to `/safety/visualization/joint_states`; `robot_state_publisher` applies URDF
forward kinematics to publish TF for RViz. This displays measured articulation,
with a fixed base; it does not estimate world position, odometry or base tilt.

Start the full body stack from a sourced workspace:

```bash
ros2 launch motor_control_hybrid control_test.launch.py
```

This starts exactly one safety supervisor, the read-only motor Web UI, the
Python CAN backend, the C++ control node, this package's joint-state relay,
`robot_state_publisher` and RViz. It starts no policy, legacy command gateway or
manual joint-state GUI, and does not automatically enable, home or zero motors.
The usual hardware-verification gates remain in force. Provide real body IMU
feedback on `/imu/data_raw`; without it, safety latches after its startup grace.

For a hardware-free smoke test:

```bash
ros2 launch motor_control_hybrid control_test.launch.py enable_fake_motor:=true use_rviz:=false
```

Fake mode replaces the CAN node. The Web UI defaults to `127.0.0.1:8080`;
`host`, `port`, `ros_domain_id`, `motor_config_file`, `safety_config_file`,
`control_rate_hz` and command timeouts are launch arguments. A fake robot without
IMU input still trips the same safety checks. `use_rviz:=false` keeps the model
TF publisher running for headless testing.

For a mechanically supported bench without an IMU, explicitly override both
IMU-dependent checks through launch arguments:

```bash
ros2 launch motor_control_hybrid control_test.launch.py fall_detection_enabled:=false trip_on_imu_timeout:=false
```

These overrides also work with `safety.launch.py`. Omitted arguments preserve
the selected YAML values (both true in the shipped configuration). Feedback and
command watchdogs and the estop interface remain active. This disables fall and
missing-IMU protection; use it only for supported bench work.

RViz uses measured `/joint_states` by default. Explicitly select
`display_source_topic:=/policy/target_angles` to inspect model target angles;
that view shows commanded posture rather than measured posture. This changes
only the model-display input; Web and control still use measured feedback.

### Web action admission

The selected-joint dialog submits explicit JSON requests to `/api/action`.
The server validates the selected joint, fresh feedback and a live released
estop, then calls `/control_core/submit_abstract_command`. Poll `/api/status`
for admission and subsequent `/control_core/abstract_command_status` results.
Queued/accepted is not completion; uncertain responses block automatic retries.
The Web process owns no CAN socket and sends no direct motor command.

Home moves the selected joint toward its current coordinate zero (0 rad),
without writing encoder zero or automatically enabling motors. The core requires
finite registry limits containing both current position and zero, settled and
fresh complete feedback, and a live healthy supervisor. It owns the command
stream during the move, disables other joints, limits the target ramp to
0.1 rad/s, uses kp=2/kd=0.2, and disables outputs on completion, tracking error,
timeout, stale feedback or estop. Completion requires measured position within
0.01 rad and speed within 0.02 rad/s for 0.3 s. Missing registry limits reject
Home. Validate gains and physical behavior before hardware use.

Set Zero uses the existing abstract operation and retains its commissioning,
firmware, exclusive-lease and readback admission checks. The current backend
still rejects unimplemented zero transactions; the UI shows the reason rather
than treating submission as successful zeroing. `/api/command` remains disabled.

### All-motor actions and fake zero

`All motors · Go Home` submits one core-owned HOME request for every configured
joint. All targets ramp toward the current coordinate zero, and every selected
joint must settle before completion. No motor is automatically enabled.
`All motors · Set Zero` submits one joint at a time, waiting for verified completion
before proceeding. Rejection/failure stops the remaining batch; completed counts
and per-joint results remain visible. Offline members reject admission.

With `enable_fake_motor:=true`, the core explicitly selects only
`/simulation/motor_backend/*` services. The fake backend supports synthetic
mechanical-zero changes; completion requires subsequent stamped zero feedback
and a simulated coordinate rebind. This does not calibrate hardware. Real CAN
mode keeps its commissioning and unimplemented firmware/readback guards, and
never selects the simulation services automatically.

### Enable and Disable

The UI offers selected-joint and all-joint Enable/Disable. Enable waits for fresh
raw enabled feedback and maintains only zero-gain/zero-feedforward idle commands;
it does not hold the robot upright. Disable is allowed without live feedback or
a released estop and may interrupt active actions. Completion requires fresh
state confirmation, otherwise the UI reports unverified physical state. Home,
zero, safety stops and explicit Disable end idle enable ownership.
See [CAN audit](../../docs/can_protocol_audit.md) for protocol profiles and limits.

