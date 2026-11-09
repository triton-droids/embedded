# walking_controller

Model-based walking without RL: a linear inverted pendulum (LIPM) MPC plans
the centre of pressure, a TSID whole-body controller turns it into joint
torques. Simulation only (uses MuJoCo ground truth for base and feet).

**Status (2026-11-09): stands, steps in place, walks forward with a
variable step length and turns (in place or on an arc) in the offline loop
and the full ROS chain.** Not yet done: larger steps (> 0.10 m commanded),
push recovery while walking, side steps, a state estimator for hardware.

```
/joint_states, /sim/ground_truth/{odom,feet}, /sim/status, /safety/estop, /walk/step_length, /walk/turn
  → walking_controller (200 Hz) → /walk/desired_motor_subset → cpp_control_node → MuJoCo
```

## Run

```bash
source scripts/activate_rosenv.sh
scripts/sim_experimental.sh controller:=model_walk auto_start:=true          # steps in place
ros2 topic pub --once /walk/step_length std_msgs/msg/Float64 "{data: 0.06}"  # walk forward
ros2 topic pub --once /walk/turn std_msgs/msg/Float64 "{data: 0.10}"         # turn left, rad/step
```

`step_length_m` is clamped to 0–0.10 m, `/walk/turn` to ±0.15 rad per step
(+ = left; both can be combined for an arc). With `walk_cmd_vel_topic:=<topic>`
(sim.launch.py) the node also follows a `geometry_msgs/TwistStamped` velocity:
step = linear.x × 0.5 s, turn = angular.z × 0.5 s, capped at 0.06 m and
0.08 rad per step (`cmd_vel_max_step_m`, `cmd_vel_max_turn_rad`); no message
for 0.5 s → step 0, turn 0. Turn first: the step shrinks while turning hard (full at |turn| ≤ 0.04 rad,
a third at the 0.08 cap; `cmd_vel_turn_free_rad`, `cmd_vel_turn_stop_rad`). With the
'state' trigger, 15 ms without a complete odom/feet/joints sample runs one cycle on the
latest of each (`trigger_fallback_s`). Loaded Jetson chase: 0/8 falls, 0/8 stalls (before:
2 falls, 1 stall in 13), approach ~27 s instead of ~18 s (design doc §20). Camera ball chase:

```bash
scripts/ball_chase_walk.sh                       # headless, domain 0, CycloneDDS
scripts/ball_chase_walk.sh remote_viewer:=true   # + read-only 3D view on another computer (not on the Jetson's desktop)
```

To watch from another computer, only the rendering runs there (MuJoCo
version must match, 3.3.7; see `simulation/humanoid_sim/README.md`):

```bash
ssh -N -L 8765:127.0.0.1:8765 USER@SIMULATION_HOST       # keep running
python remote_viewer.py --host 127.0.0.1 --port 8765     # simulation/humanoid_sim/humanoid_sim/remote_viewer.py
```

The camera image is `/camera/camera/color/image_raw` (e.g. `rqt_image_view` on
the simulation host, same `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY=1` and
`CYCLONEDDS_URI`); `/ball_chase/status` and `/walk/status` show the pursuit. `/walk/status` (JSON, 2 Hz) has the
phase, step count, step length, turn, frame heading, cycle time, prediction
horizon and input age (`input_age_ms`,
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
offline stand / step in place / walk, realised step length (≥ 0.85 of 0.05 m),
state prediction, turning (both ways, 90°+ in place, arc, walking after a
160° turn; slow: chase pattern and 49 s continuous turns), and the latency limit (15 and 20 ms walk 49 s; 30 ms is
a strict xfail, so an improvement shows up as a failure to update).

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
  Touchdown is a swing-foot force > 20 N after s > 0.6 of SS. The swing foot
  rolls up to 7° (no ankle roll) and peaks at ~1.5 of the planned 2.5 cm, so
  a sole corner reaches the floor right at s ≈ 0.6 on every step; its
  horizontal motion therefore finishes at s = 0.85 (`swing_fwd_end`), not 1.0.
- **Turning** (`turn_rad`): at lift-off the swing target (stance foot +
  nominal offset) is rotated by the turn about the stance foot. Once turning,
  the walking frame (MPC axes) follows the measured stance foot yaw + turn/2,
  the swing foot's yaw is interpolated from its start to the target by s = 0.5
  (`swing_yaw_end`; each swing turns its foot by ~2× the turn), the torso yaw
  reference follows the frame, and the sole-centre offsets are rotated with
  each foot's yaw. With turn 0 and no earlier turn the straight gait is
  unchanged bit for bit.
- **Latency compensation** (`gait.py`, `predict_s` = 10 ms): every cycle the
  measured state is predicted at constant velocity to when the command acts
  (input age + `predict_s`) and the controller clock is shifted by `predict_s`.
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
| Fixed step 0.02 / 0.05 / 0.08 / 0.10 m, 20 s each | all 41–42 steps; 0.78 / 1.83 / 2.59 / 3.09 m forward |
| Step length changed during one run (0 → 0.05 → 0.10 → 0.02 → 0.08 → 0) | 28 s, no fall; progress follows the command |
| Delay 10/15/20 ms × friction 0.65/1.15 × step 0/0.06 | 12/12 for 20 s |
| Step 0.15 m / 0.20 m (outside the node's 0.10 m clamp) | falls on the first step (before `swing_fwd_end`: after 22 / 2 steps, when the steps came out much shorter) |

Latency (offline, in place for 9 s then 0.06 m, 49 s; ROS chain ≈ 15 ms):

| Command delay | 10 ms | 20 ms | 25 ms | 30 ms | 40 ms |
|---|---|---|---|---|---|
| no compensation | walks | falls 30 s | falls 32 s | falls 18 s | falls 10 s |
| prediction = actual delay | walks | walks | walks | walks | falls 6 s |
| default (10 ms prediction) | walks | walks | — | falls 20 s | — |

Without compensation the falls start with growing yaw swings, then one knee
straightens. Extrapolating with the last WBC acceleration instead of constant
velocity fails at 30 ms; predicting only 5 ms falls at 20 ms. The 20 s sweep
is too short to show any of this.

Realised / commanded step length (offline, 20 s, after the first 4 steps):

| | 0.02 | 0.05 | 0.08 | 0.10 |
|---|---|---|---|---|
| swing ends at s = 1.0, no compensation (2026-11-08 start) | 0.76 | 0.73 | 0.65 | 0.61 |
| s = 0.9, no compensation | 0.98 | 0.90 | 0.79 | 0.75 |
| s = 0.9 + prediction | 0.77 | 0.74 | 0.62 | 0.57 |
| **s = 0.85 + prediction (default)** | 0.88 | 0.78 | 0.67 | 0.62 |
| s = 0.8 + prediction | 0.99 | 0.86 | 0.73 | falls at 0.10 |

Part of the earlier step length came from the uncompensated delay (steps
grow with delay); with prediction the steps are shorter, 0.85 recovers part
of it and passes the sweep, fixed 0.02–0.10, variable and 20 ms / 49 s.
`walk_eval.py sweep` at 0.06 (20 s): 1.72–2.16 m at the start, 1.89–2.46 m now.

Full ROS chain (CycloneDDS, headless, IMU safety trip on):

| Host / version | Result |
|---|---|
| Earlier run, other host | 81 steps in 34 s, 1.33 m forward at 0.06, no estop, cycle p99 < 4.2 ms |
| Jetson, timer trigger (start of 2026-11-08) | falls after 25–37 s of walking at 0.06 (3/3); sample → applied torque ≈ 25 ms |
| Jetson, same-sample trigger | 3/3 no fall (2 × 53 s, 141 s); ≈ 15 ms; 0.041 m/step |
| + `swing_fwd_end` 0.9 | 3/3 no fall (2 × 53 s, 141 s); 0.049–0.052 m/step |
| **+ prediction, 0.85 (current)** | 3/3 no fall: 2 × 53 s and 140 s (364 steps, 14.9 m, 0.041 m/step, lateral ≤ 0.09 m); prediction horizon 14.4 ms median (12–17) |

Latency on the Jetson, sample → controller publish / publish → torque applied
in MuJoCo: 14.7 / 10 ms with the timer, 7.3 / 7 ms with the same-sample
trigger, MPC solver above and the simulator command executor. Cycle 2.8 ms
mean, 4.4 ms p99; 182–193 Hz of commands. Physics overruns grew from 9 to
106 in a 141 s run.

Turning (offline, 20 s, delay 10/15/20 ms × friction 0.65/1.15): 18/18 OK;
per step 6.7–7.6° at 0.15 rad (8.6°) both ways, 5.0–5.6° on a 0.06 m arc at
0.10 rad (5.7°). 49 s at 15 ms: continuous 0.10 / 0.15 / −0.15 rad turns
(590–780°), an arc and a chase pattern (turn ~90°, walk 3 s, turn back,
× 4) all walk.

Camera ball chase (`controller:=model_walk`, Jetson, headless, YOLO on CPU,
6 Hz RGB-D): the robot walks from 1.7 m to the ball and stops 0.5–0.6 m in
front of it, then keeps facing it; 2 of the last 3 runs ran 45–80 s without a
fault. Velocity caps from measurements: 0.10 m steps fall at 15 ms latency
offline (0.06 m with ±0.12 rad turns walks at 15 and 20 ms); with 0.15
rad/step the chase heading loop limit-cycles (yaw ±20°, turn flipping
between the caps each step, offline with the same pursuit law ±19°), at
0.08 rad/step offline ±6°, chain −1…+21°. The camera mount (0.3 kg, not in
the controller model), a controller clock up to 15 % faster than physics
(the simulator's real-time factor drops to 0.77–0.95 with the camera) and
the pursuit's 2 Hz / 0.5–0.7 s vision lag each walk offline.

ROS chain (Jetson), via `/walk/turn`: 2/2 no fall over 62 s: +0.10 rad for
11.5 s → +155° / +166° (5.2–5.5° per step), walk 0.06 m 7 s (0.65–0.71 m),
−0.10 rad for 11 s → −131° (4.7° per step), walk 15.5 s (1.32–1.34 m).

Known issues: the chase is load-sensitive on the Jetson. Plain runs: 4/4 × 65 s without a fault. A MuJoCo window rendering on the Jetson's own desktop stopped 3/3 runs (`command expired`, GPU/shared memory exhausted), so watch from another computer. With a recorder or the viewer stream also running, 2 runs fell during the first approach (command gaps of 30–40 ms, input age p99 12–13 ms instead of 9–11); offline with a constant 15–25 ms delay this does not fall (design doc §16). With measured, randomised chain timing (`offline.run(..., timing=ChainTiming(...))`, `DEGRADED_TIMING`) the recorded commands fall in ~3 % of seeds. Every run bounces (single-support force ~40–250 N) because the stance contact is pulled flat at lift-off (`contact_kp`); lowering it (`walk_contact_kp:=50`) widens the margins (force 66–185 N) but did not reduce falls offline or on the chain (1/8 vs 1/7, all in the first approach; design doc §17). After a turn there is no fixed heading any more (the frame
follows the feet), so walking straight drifts in yaw (−7 to +14° over 7–15 s
in the chain); a caller such as ball chase closes the heading loop with
`/walk/turn`.  realised steps are 0.6–0.9 of the command; a full 0.10 m step
(s = 0.8 above) falls, so larger steps are a balance limit, not the touchdown.
A CPU overload (two offline episodes next to the chain and a recorder, more
than nproc − 1) stalled the commands for 80 ms while stepping in place: the
simulator paused physics (`command expired`), the IMU stopped and the 0.3 s
IMU-freshness estop latched. This is probably also the earlier GUI-run estop
under load: the IMU estop is the last link, not the cause.
