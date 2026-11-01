# WBC plus camera chase — 2026-10-31

The experimental chase launch now runs:

```
RGB-D chase -> reference warp -> /policy/desired_motor_subset
  -> WBC (policy_impedance) -> /desired_motor_subset
  -> cpp_control_node -> /motor_commands -> MuJoCo
```

There is no policy-to-C++ bypass. WBC retains the trained fast PD loop and adds
a correction bounded to ±0.02 Nm, using the same rates, torque limits, source
timeouts, solve budget, enabled-feedback checks and release admission as the
existing WBC experiment. The Web monitor and safety supervisor remain present.
The generic production launch and its WBC/vision guard are unchanged; only this
experimental composition is extended.

## Camera payload model

`wbc_payload.py` generates `generated/camera_payload.urdf` and its dynamics
metadata from the existing aligned description and compiled camera scene.
The added 0.297 kg from `head`, `d435_mount_base` and `d435_pitch` is aggregated
with its actual neutral CoM and inertia, rather than omitted. Source URDF, ONNX
and motor registry are untouched. The generated model has ten driven leg joints.

With zero pan/tilt joint angles, three randomized base/leg configurations match
MuJoCo's projected mass matrix and generalized gravity to 1e-9, and robot CoM to
1e-10. The head servos remain active in simulation. Their moving inertia and
servo reactions are approximated by the neutral rigid payload, not represented
by two more WBC coordinates. On yaw {-1,0,1}, pitch {-0.6,0,0.6} radians, the
largest generalized gravity-entry difference was 0.0271581. This is a quantified
quasi-static approximation, not exact moving-head dynamics or hardware validation.

All **31 tests passed**, including the prior reference/chase tests plus the
payload equivalence and approximation checks.

## Actual GUI attempts (all retained)

| Record | Result |
|---|---|
| `chase-wbc-import-failure.log`, `chase-wbc-live.json` | Startup failed before simulation: system Python used by ros2 launch could not import Pinocchio. Fixed by generating the model in the activated rosenv subprocess; empty telemetry record retained. |
| `chase-wbc-v2.json` and `.log` | WBC performed eight solves on the gantry. IMU/chase freshness stopped startup; no release. This is not a walking/stability pass. |
| `chase-wbc-v3.json` and `.log` | C++ accepted ENABLE, verified all motors enabled, and released the gantry. WBC entered running state and performed 291 solves. The chase turn-demand gate then stopped the experiment. |

The released attempt recorded 145 policy messages, 292 WBC output messages
(including the stop), 145 matched policy/WBC source stamps, and 579 C++ outputs
with stamps matching captured WBC outputs. Running-state WBC computation samples
were approximately 0.84–1.05 ms. The raw records include `/wbc/status`, camera
depth metadata, vision, policy, simulator and safety status, and odometry.

Measured displacement from first held odometry to final paused odometry was
world **(+0.0754, +0.3244, -0.0116) m**; initial forward is +Y. This demonstrates
actual movement through WBC, not successful arrival or a calibrated step length.
The first observed chase fault requested angular velocity ~0.354 rad/s, exceeding
the experiment's 0.25 rad/s supported heading corridor. WBC then stopped on input
timeout, the backend paused, and the paused IMU subsequently tripped the unchanged
freshness supervisor. This is a censored stop, not a fall or balanced stand.

The GUI window was observed. Experiments were serialized in ROS domains 190–192
with separate Web ports. The running attempt was shut down after recording; no
simulation is intentionally left running. No safety deadline was relaxed and no
robot teleportation, applied chase force or hardware command was used.

## Run

```bash
source scripts/activate_rosenv.sh
ros2 launch "$PWD/experimental/reference_warp/chase.launch.py" start_chase:=true
```

The default is now WBC; there is no PD bypass switch in this experiment. Omitting
`start_chase:=true` keeps the base held and motors disabled. Commanded turning,
balanced stopping, calibrated walking speed and A* following still require work.
