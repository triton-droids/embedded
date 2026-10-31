# wbc_control_node

Simulation-only TSID bridge:

```
tracking_policy_node → /policy/desired_motor_subset
                    → wbc_control_node → /wbc/desired_motor_subset
                    → cpp_control_node → /motor_commands → MuJoCo
```

The policy and its angle targets are unchanged. The WBC reorders joint feedback,
converts `/sim/ground_truth/odom` to the aligned Pinocchio base frame, uses
`/sim/ground_truth/feet` (sole heights in `position`, ground normal forces in
`effort`), and solves bounded inverse dynamics.

The default `wbc_tracking_mode=policy_impedance` preserves the policy's trained
fast joint PD loop. At each WBC sample, define
`tau_pd = kp*(q_ref-q) + kd*(dq_ref-dq)` and
`tau_nominal = clip(tau_pd + policy_feedforward, -limits, limits)`.
A soft TSID actuation task tracks this nominal torque; the QP has hard actuator
bounds and a +/-0.02 Nm nominal-torque correction bound. Contact motion, CoM,
body and posture tasks are soft, with small weights to avoid overriding the
trained policy. Dynamics and contact-force friction/normal bounds remain hard.
This is a conservative policy torque correction, not an independent gait planner.

The command sent through C++ contains the original targets/gains and
`tau_feedforward = policy_feedforward + tau_wbc - tau_nominal`. Thus PD is
counted once. When the nominal actuator command is unsaturated, the sum at the
feedback sample equals the WBC result. During saturation, the fast actuator
clamp remains authoritative, and the discrepancy at that sample is at most the
small correction bound. This avoids cancelling large raw PD errors with a
large feedforward term, which would change the trained saturation behavior.
MuJoCo evaluates PD every physics step, including between ROS updates. QP
predictions do not constitute a future-state torque guarantee. Numerical torque
bound errors up to 0.001 Nm are projected back inside the exact QP bounds;
larger violations stop control. No actuator limit is enlarged.

`wbc_tracking_mode=policy_torque` sends the entire solved torque with zero
backend PD, for experiments. `wbc_tracking_mode=posture` retains the earlier
acceleration/posture WBC. Both changed the trained closed loop and failed to
establish stable walking in the earlier tests. WBC solve failure, expiry or
estop disables outputs in every mode; there is no bypass or fallback.
Motor enable and C++ forwarding remain in the path. While the gantry is held,
the bridge publishes fresh zero-gain/zero-torque motion commands for verified
arming, and the policy supplies a zero prior-action observation. This avoids
pre-driving the joints and accumulating an artificial moving-reference action
history before release. Normal policy/WBC output starts after release status;
this behavior is simulation-only.

WBC autostart enables through `/control_core/submit_abstract_command`, waits for
the matching verified-enabled COMPLETED status, and requires C++ motor output
matching a fresh WBC source before asking the backend to release the gantry. A fresh zero-torque arming
command is sufficient; torque-only readiness is enabled for this mode.
Before ENABLE, autostart also requires a fresh source-matched WBC command to
have traversed C++ continuously for 0.3 seconds (no receive gap over 50 ms),
so arming waits for a healthy discovered input path.
After admission it clears that evidence and requires new forwarding again.
WBC simulation staggers supervisor/Web startup by 3 seconds while policy/TSID
load on the gantry. During this stage WBC gates on unreleased safety and C++
rejects enable without a healthy supervisor; motors stay disabled. No safety
threshold or automatic reset is introduced.

Rejected admission can be retried within the startup budget; an accepted or
ambiguous ENABLE is never automatically repeated. Failure leaves the gantry
held and sends DISABLE. During zero-torque arming, stale inputs wait for fresh
samples. Release immediately activates the latched timeout/fault checks.
Explicit estop never resets automatically.

Build after installing Pinocchio/TSID in rosenv, including the TSID rotor-setter
binding, soft-contact removal and actuation-equality patches documented in
`experimental/README.md`:

```bash
source scripts/activate_rosenv.sh
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 /usr/bin/python3 -m colcon build --executor sequential --symlink-install --packages-up-to wbc_control_node --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch wbc_control_node rl_wbc_sim.launch.py auto_start:=true
```

`auto_start` defaults false. With false, motors remain disabled/gantry held until
enabled and released through existing simulation controls. For headless runs use
`use_viewer:=false`. Existing `ros2 launch humanoid_sim sim.launch.py` still uses
the original policy/PD path; `enable_wbc:=true` opts in to this bridge.
The Python prefix defaults to `python3` from PATH; activate rosenv or set
`wbc_python_executable` to its absolute interpreter path. Ordinary ROS console
scripts may otherwise use system Python without TSID.

The viewer runs in a separate spawned process. GUI synchronization, rendering
and its Python execution lock cannot block the simulator's ROS callbacks.
Only bounded latest-state/perturbation queues connect it to physics; the physics
process never waits for the viewer to consume a frame. WBC mode uses asynchronous FastDDS publication in the simulator, WBC and C++
to decouple network writes from callbacks and command scheduling. Expiry and
safety thresholds remain unchanged; this is not a real-time guarantee.

In WBC simulation mode, joint feedback and C++ forwarding run at 200 Hz,
WBC runs at 100 Hz, and policy inference remains at its trained 50 Hz. The
ordinary PD launch retains its original rates. The viewer follows the robot.

All arrays follow `controller.joint_names`. Original policy source stamps are
preserved and deadlines may only shorten. Stale, misframed, invalid, expired,
failed or late results are not forwarded; active faults send DISABLE and latch
until node restart. There is no stale-result PD fallback or automatic rearm.
`/wbc/status` reports progress, contacts, solver timing and faults.

The WBC simulation disables the unmodelled head/target and initializes the soles
on the floor. Contact selection is a debounced heuristic: planned foot lift is
estimated by FK of the policy's joint targets; measured sole heights and normal
forces determine touchdown/liftoff. It is not a terrain-aware gait planner.
CoM references default to FK of policy targets at the measured base pose; body
orientation defaults to its initial orientation. `contact_mode=double_support`
and `com_reference_mode=hold` are available for standing diagnostics.

Adding this controller changes the closed-loop system the policy was trained
with. Successful command forwarding is not evidence of stable walking; evaluate
the policy under this controller and retrain/tune if required. Ground truth is
simulation-only and must be replaced by state/contact estimation for hardware.

## Reference playback and validation

WBC walking defaults to `wbc_reference_end_behavior=loop`. The bundled ONNX has
299 reference frames and a nonzero final joint velocity. Holding that last
moving reference indefinitely does not provide a valid stationary target.
The reference clock stays at frame zero until `/sim/status` confirms gantry
release, while policy inference and safety continue running. The simulator
publishes that status immediately on release. The initial run then plays
frames 0..250 and repeats [50, 251), a segment chosen
for nearby joint position/velocity at its boundary (a small joint-position
difference near frames 251 and 50). Previous-action history is
preserved. This is a model-specific phase loop, not arbitrary trajectory
retargeting. `wbc_reference_loop_start_frame`/`wbc_reference_loop_end_frame`
are configurable and checked against the export. Other policies need their own
validated loop. The ONNX file and registry are unchanged.

`wbc_reference_end_behavior=hold_last_frame` restores finite-clip playback.
`hold_first_frame=true` overrides both and keeps frame zero. The ordinary
`humanoid_sim` PD launch keeps its original hold-last behavior. `/policy/status`
reports frame, loop cycles and reference behavior. Rebuild `motor_control_hybrid`
after editing its policy node: its installed ROS executable is a copied Python
program, even when the importable module is symlinked.

Manual walking regression (isolated local ROS domain 173, no physical devices):

```bash
source scripts/activate_rosenv.sh
WBC_PROBE_SECONDS=25 rosenv/bin/python humanoid_control/wbc_control_node/test/ros_chain_smoke.py --walking
WBC_PROBE_SECONDS=30 rosenv/bin/python humanoid_control/wbc_control_node/test/ros_chain_smoke.py --walking --viewer
```

The regression checks actual sole clearance >1 cm, normal force <2 N for at
least 40 ms followed by landing force >5 N, with the other foot supporting
>20 N. It requires at least two such air/landing cycles per foot, >20 cm net
horizontal displacement, a completed reference cycle, C++ source forwarding,
no estop/fault, successful gantry release and clean shutdown. GUI runs also
reject headless fallback. Outputs are `/tmp/wbc-live-walking.json` and `.log`.
`--motion` checks finite motion; no flag checks a held first-frame target.

## Current limitations

The current controller produces alternating steps. Finite GUI runs of 30 and
40 seconds succeeded, but an independent restart fell after about 21 seconds.
Repeatable sustained walking has not been established. Regression tests cover
software contracts, model consistency and fault handling; they do not prove
balance or hardware readiness.

The default preserves policy impedance with a small WBC torque correction.
Policy/controller adaptation under deployed timing, full gait planning,
velocity commands, terrain handling and robust balance remain open work.
