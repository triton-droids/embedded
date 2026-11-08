# Reference excursion experiment

## Combined camera chase experiment

An additional launch connects the actual RGB-D chase request to the reference
warp at runtime, using the original ONNX and motor registry:

This is a forward-reference experiment, not a complete chasing controller.
Turning and a balanced stop are not implemented. A turn request above 0.25 rad/s
latches a stop; subsequent WBC expiry and paused physics can produce a secondary
IMU timeout. See [CHASE_STOP_DIAGNOSIS.md](CHASE_STOP_DIAGNOSIS.md).

```bash
./experimental/reference_warp/run_chase.sh start_chase:=false
```

Defaults are ROS domain 182, Web port 8182, GUI enabled, and an 8 s maximum walk
budget. The command above keeps the gantry held and motors disabled.
`start_chase:=true` explicitly releases this unfinished motion experiment.
Use `start_chase`, not the inner simulator's `auto_start` argument. The experiment
uses the existing ENABLE action through C++ and waits for verified feedback and
fresh forwarded motion before asking the simulator to release the gantry.

Headless with camera rendering and network DDS discovery (localhost restriction
is disabled by default in this experimental launch):

```bash
./experimental/reference_warp/run_chase.sh \
  start_chase:=false use_viewer:=false localhost_only:=0
```

Other ROS machines should use `ROS_DOMAIN_ID=182` and `ROS_LOCALHOST_ONLY=0`.
`use_viewer:=false` disables the GUI; RGB/depth rendering and chase remain active.

The wrapper activates the private Cyclone DDS packages for this launch only.
It does not overwrite system ROS or change QoS, the 250 ms safety limit, command
expiry or automatic recovery. If the private packages are missing, run
`bash experimental/reference_warp/prepare_cyclone_trial.sh` first.
Direct `ros2 launch` still uses the middleware selected in that shell.
Fast DDS synchronous and asynchronous writes both reproduced long native lock
waits; see [DDS_LOCK_DIAGNOSIS.md](DDS_LOCK_DIAGNOSIS.md) for the measured workaround
and its limits. The complete walking controller is not validated by held tests.

The experiment enables a separate body-IMU executor and timing diagnostics.
Application topic histories throughout this chain are now KeepLast=2. The IMU
thread and its native TID are reported in `/imu/status`. Dedicated Python threads
still share a GIL. Zero-copy capability was measured separately; the existing
IMU/image/joint/command types cannot loan samples in the tested middleware builds.
See [KEEP_LAST_2_AND_ZERO_COPY.md](KEEP_LAST_2_AND_ZERO_COPY.md).
See [IMU_TIMING.md](IMU_TIMING.md)
for measured failures, mitigations and the distinction between a primary IMU
timeout and a timeout following a deliberate chase stop. Safety remains 250 ms.
For measured GUI/camera CPU costs, physics pacing and the full-stack CPU/IMU
comparison, see [GUI_CPU.md](GUI_CPU.md).
The experimental launch now defaults to `priority_mode:=auto`, applying
IMU > WBC > other scheduling and affinity. On this unprivileged workstation it
uses nice 0/5/10; FIFO 70/60 requires OS permission. See [PRIORITY.md](PRIORITY.md)
for actual thread readback, diagnostics isolation and retained failure results.

```
RGB + aligned depth -> football detection -> /ball_chase/cmd_vel
    -> experimental chase_policy.py -> /policy/desired_motor_subset
    -> WBC -> /desired_motor_subset
    -> existing cpp_control_node -> /motor_commands -> simulated backend
```

The original motor Web monitor and safety supervisor remain active. The combined
launch now inserts WBC in `policy_impedance` mode: the trained fast motor PD loop
is preserved with a bounded WBC torque correction (±0.02 Nm). The policy has no
direct publisher to C++'s input: it publishes `/policy/desired_motor_subset`;
WBC alone publishes `/desired_motor_subset`, which C++ forwards to the backend.
See [WBC_CHASE_RESULTS.md](WBC_CHASE_RESULTS.md) for this revision. Earlier PD
combined tests remain historical records, not validation of the WBC combination.

The production launch's WBC/vision restriction is unchanged. This experimental
composition generates a separate URDF with the actual neutral D435/mount payload
(0.297 kg), publishes foot contacts and uses existing WBC rates and deadlines.
Neutral-head mass matrix, CoM and gravity are checked against MuJoCo. Pan/tilt
still move, but changing inertia and servo reactions are approximated by this
rigid neutral payload. A yaw/pitch angle grid measured a maximum generalized
gravity-entry difference of 0.02716. Dynamic head reaction compensation is not
established by this approximation.

The experimental scene reuses the complete camera/mount/football scene, placing
the ball straight ahead at world `(0, 1.8, 0.11)` m, including reset. Floor
clearance is set to zero. The vision stop distance is 1 m. Valid forward requests
in `(0, 0.2]` map linearly to reference excursion scales `[0.95, 1.05]`, with a
0.1/s scale slew bound and consistent velocity references. This is a heuristic
mapping; requested m/s is **not** calibrated actual walking speed. The ONNX
weights and source hashes remain original; only in-memory reference observations
are changed by the experiment. No generated ONNX is required for this launch.

`/experimental/chase_status` reports requested/applied scale, command, arming and
fault reason. No commands are produced before a valid chase request and a fresh
simulator status. The source and receipt age limit for chase commands is 0.25 s;
the vision node separately enforces its 1 s source-image age budget. Parent
policy IMU/joint deadlines and the trained 50 Hz inference contract are unchanged.

**Restricted to straight approach:** angular requests above 0.25 rad/s are
rejected, rather than converted into an unvalidated turn. Smaller angular
requests only define the permitted heading corridor; they are not executed as
turning commands. Zero requests, invalid/stale input, excessive turn demand,
original policy faults or the walk budget stop command refresh and latch the
experiment. The existing command-expiry path then stops/pauses simulation. This
is **not a balanced standing stop** and requires restarting to resume. No base
teleportation, gantry recapture or applied chase forces are used. A* paths are
not consumed by this adapter.

Tests and a recorder:

```bash
ROS_DOMAIN_ID=183 python -m pytest experimental/reference_warp/test_prepare.py \
  experimental/reference_warp/test_chase_policy.py \
  experimental/reference_warp/test_wbc_payload.py -q
ROS_DOMAIN_ID=182 python experimental/reference_warp/monitor_chase.py \
  --duration 15 --output experimental/reference_warp/results/my-chase.json
```

The recorder can place the simulated ball behind the robot with `--occlude-at
9` to exercise loss behavior. Do that test in the held preview first. Captured
results and limitations are in [CHASE_RESULTS.md](CHASE_RESULTS.md).

## Static reference variants

Simulation-only attempt to reuse the existing tracking policy with slightly
different sagittal reference excursions. Everything added for this experiment
lives in this directory. Production code, source ONNX and motor registry are
unchanged. This does not establish a velocity-conditioned walking controller.

`prepare.py` makes a separate ONNX and motor registry under ignored `generated/`.
Only `joint_pos.1` and `joint_vel.1` initializers change; all neural weights and
other initializers are checked byte-for-byte. The copied registry records the
new model SHA256 while preserving gains, joint limits and `hardware_verified:
false`. The identity variant is an exact copy of the source ONNX.

The six modified hinges are left/right `hip1`, `knee`, and `ankle`, whose saved
training XML axes are X. `thigh` is a Z hinge despite its name and is excluded,
as are both `hip2` joints. This experiment does not command yaw or turn steps.

For each selected reference angle, with `q0` its first-frame value:

```
q_new  = q0 + g(t) * (q_original - q0)
dq_new = g(t) * dq_original + g_dot(t) * (q_original - q0)
g(t)   = 1 + (scale - 1) * smoothstep5(clamp(t / 1 second, 0, 1))
```

The quintic ramp starts at identity and has zero first/second derivative at its
ends. After 1 s the scale is constant, including the existing WBC loop starting
at frame 50. Only scales in [0.9, 1.1] are accepted. Modified references outside
the original joint limits are rejected. This scales joint excursions, **not a
measured footstep length**. The exported angular velocities are transformed
consistently; their original relation to position is inherited from the dataset.

## Prepare and run

From the workspace root:

```bash
source scripts/activate_rosenv.sh
source install/setup.bash
python experimental/reference_warp/prepare.py --scale 0.95

ros2 launch wbc_control_node rl_wbc_sim.launch.py \
  auto_start:=true use_viewer:=true ros_domain_id:=181 web_port:=8181 \
  model_path:="$PWD/experimental/reference_warp/generated/scale_0.950/20260914_170827.onnx" \
  motor_config_file:="$PWD/experimental/reference_warp/generated/scale_0.950/motors.yaml"
```

This preserves policy -> WBC -> C++ -> simulated backend and the safety/Web
stack, including all original deadlines. WBC remains in its conservative
`policy_impedance` mode. Do not use the generated configurations on hardware.

## Repeatable comparison

DDS timing experiments and the private strict-realtime build comparison are in
[DDS_TUNING_RESULTS.md](DDS_TUNING_RESULTS.md). The six held trials found improved
IMU continuity with targeted 5 ms waits, but also unhandled publish errors that
terminate cpp_control_node. The private build is experimental and is not selected
by the default launch.
Publication failure handling and the subsequent held-simulation fault-injection
checks are documented in [PUBLISH_FAILURE_HANDLING.md](PUBLISH_FAILURE_HANDLING.md).

```bash
python -m pytest experimental/reference_warp/test_prepare.py -q
python experimental/reference_warp/run_trials.py --repetitions 2
```

The runner serializes GUI trials of 1.00, 0.95, and 1.05. Each attempt has its
own launch log, source/model hashes, full received odometry/status history, and
summary in `results/<timestamp>/`. Runs stop 10 wall seconds after the first
observed released status, or after 25 seconds without release. Window detection
requires a new MuJoCo window, ignoring already-open simulations.

Categories distinguish GUI-invalid, not-released, confirmed fall, censored stop,
and observed-no-fall. A logged safety fall or base height below 0.35 m is a
confirmed fall criterion. First-release status timing is limited by status
publication rate. Recorded displacement spans received odometry after that
status; it can include motion after a safety stop and is not a valid commanded
walking-speed measurement for censored/fallen trials. Repetitions are exploratory,
not a statistical stability comparison.

Initial attempt `results/20261031-184209/` had an instrumentation failure while
saving Jazzy byte-valued diagnostic levels. Its launch log is preserved, and it
must not be used as a complete trial. Subsequent runs use the corrected integer
conversion and a new result directory.
