# walking_controller

Model-based walking without RL: a linear inverted pendulum (LIPM) MPC plans
the centre of pressure, a TSID whole-body controller turns it into joint
torques. Simulation only (uses MuJoCo ground truth for base and feet).

**Status (2026-11-08): stands, steps in place and walks forward with a
variable step length in the offline loop and the full ROS chain.** Not yet
done: turning, larger steps (> 0.10 m commanded), push recovery while
walking, a state estimator for hardware.

```
/joint_states, /sim/ground_truth/{odom,feet}, /sim/status, /safety/estop, /walk/step_length
  → walking_controller (200 Hz) → /walk/desired_motor_subset → cpp_control_node → MuJoCo
```

## Run

```bash
source scripts/activate_rosenv.sh
scripts/sim_experimental.sh controller:=model_walk auto_start:=true          # steps in place
ros2 topic pub --once /walk/step_length std_msgs/msg/Float64 "{data: 0.06}"  # walk forward
```

`step_length_m` is clamped to 0–0.10 m. `/walk/status` (JSON, 2 Hz) has the
phase, step count, step length, cycle time and input age (`input_age_ms`,
`joint_age_ms`: sample stamp to cycle start, last 2 s).

The node runs one cycle per physics sample (node parameter `trigger: state`,
default): as soon as odom, feet and `/joint_states` with the same header stamp
have all arrived. `trigger: timer` is the earlier fixed 200 Hz timer. With
`controller:=model_walk` the simulator receives the commands on its own
executor (`dedicated_command_executor`, default true).

Offline (no ROS; one episode ≈ one CPU core, keep `--workers` small):

```bash
python3 humanoid_control/walking_controller/tools/walk_eval.py fixed --length 0.06
python3 humanoid_control/walking_controller/tools/walk_eval.py variable
python3 humanoid_control/walking_controller/tools/walk_eval.py sweep --workers 2
python3 -m pytest humanoid_control/walking_controller/test   # unit + offline smoke, ~15 s
WALK_SLOW_TESTS=1 python3 -m pytest humanoid_control/walking_controller/test   # + 49 s latency runs
```

Tests: MPC (exact LIPM prediction, CoP bounds, terminal DCM, closed-loop
capture, active-set solver = `lsq_linear`), WBC edge contacts (corners, per-foot
force cap, lateral CoM authority flat 0.006 vs both edges 0.97), node trigger,
offline stand / step in place / walk, and the latency limit (15 ms walks; 20 ms
is a strict xfail, so an improvement shows up as a failure to update).

## How it works

Measured first on the RL gait (it already walks on this robot): ~48 % double
support (median 220 ms), single support 100–250 ms, lateral CoM speed
≈ 0.36 m/s at touchdown, stance feet tilted sideways 3–12°, stance width
300–478 mm.

- **Lateral weight transfer needs both feet on an edge.** Without ankle roll,
  with one or both feet flat the lateral CoM is not actuated (null-space
  authority ≈ 5e-4). With both feet rolling on parallel edges (CoM right of
  the feet: left foot inner edge, right foot outer edge) it is 0.30–0.38.
  The WBC has per-foot modes `flat` (Contact6d), `inner`/`outer` (two
  ContactPoint on that edge) and `swing`.
- **MPC** (`lipm_mpc.py`): per horizontal axis, CoP as decision variable,
  exact LIPM discretisation, 25 × 40 ms horizon, box bounds from the planned
  support (double support: both feet; single support: stance sole), terminal
  DCM cost. Box-constrained QP, solved by a warm-started primal-dual active
  set (0.43 ms per axis on the Jetson; `scipy.optimize.lsq_linear` took 1.09
  ms); if it does not reach a KKT point it falls back to `lsq_linear`.
  Future footholds are previewed (stance foot + nominal offset + step length).
- **Gait** (`stepping.py`): DS 0.2 s → SS 0.3 s, both feet lean towards
  where the CoM is going (80 ms ahead) in DS, the next swing foot is unloaded
  over the last 60 ms, the stance foot is held flat in SS, the swing foot
  lands at the stance foot plus the nominal offset (no accumulated drift).
- **WBC** (`wbc.py`): CoM acceleration feedforward ω²(c − p₀), CoP task p₀,
  zero angular momentum, pelvis orientation, posture; soft contacts
  (weight 1e3, f_min 0). Joint loop reproduces the WBC torque (low kp/kd).
- **Stand** (`gait.py`): gantry hold, landing, 1 s squat to knee −0.9 rad
  with stiff PD to the planned posture, as in the earlier M1 experiment.

## Results

Offline loop (MuJoCo, 10 ms command delay):

| Test | Result |
|---|---|
| Step in place 25 s | 53 steps, no fall, width 0.30–0.32 m, CoM z mean 0.373 m |
| Fixed step 0.02–0.10 m, 20 s each | all 41–42 steps, 0.6–2.5 m forward |
| Step length changed during one run (0 → 0.05 → 0.10 → 0.02 → 0.08 → 0) | 28 s, no fall; progress follows the command |
| Delay 10/15/20 ms × friction 0.65/1.15 × step 0/0.06 | 12/12 for 20 s |
| Step 0.15 m / 0.20 m | falls after 22 / 2 steps |

Offline at 0.06 m, in place for 9 s then walking: 10 and 15 ms command delay
walk 60 s; 20 / 25 / 30 / 40 ms delay fall after 35 / 29 / 25 / 14 s
(`Torso tilt`: the yaw swings grow, then one knee straightens). The 20 s sweep
above is too short to show this.

Full ROS chain (CycloneDDS, headless, IMU safety trip on):

| Host | Result |
|---|---|
| Earlier run, other host | 81 steps in 34 s, 1.33 m forward at 0.06, no estop, cycle p99 < 4.2 ms |
| Jetson, before 2026-11-08 latency fix | falls after 25–37 s of walking at 0.06 (3/3); sample → applied torque ≈ 25 ms |
| Jetson, after | 3/3 no fall: 2 × 53 s and 1 × 141 s (359 steps, 14.8 m, 0.041 m/step); ≈ 15 ms. The 3 runs with only part of the fix (≈ 18 ms) also walked 53 s |

Latency on the Jetson, sample → controller publish / publish → torque applied
in MuJoCo: 14.7 / 10 ms before, 7.3 / 7 ms after (timer replaced by the
same-sample trigger, MPC solver above, simulator command executor). Cycle
2.8 ms mean, 4.3 ms p99; 187 Hz of commands. The margin to the 20 ms offline
limit is small; physics overruns grew from 9 to 106 in the 141 s run.

Known issues: walking needs a total loop latency below ~15–20 ms (see above);
commanded steps above ~0.05 m are only partly realised
(0.10 m → ~0.055 m per step; the swing foot probably touches down early); one
GUI run hit the 0.3 s IMU-freshness estop after 33 steps while the host was
heavily loaded, later GUI runs at 30 Hz and 5 Hz viewer rate did not.
