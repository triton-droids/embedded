# Model-based walking controller (no RL) — design

**Status (2026-11-08, later): revived and walking in simulation.** The first
attempt (M1 stand, §10; flat-foot rocking, §11) stalled; measuring the RL gait
showed that both feet roll on their edges in double support, which unlocks the
lateral CoM. A LIPM MPC + TSID WBC built on that steps in place and walks
forward with a variable step length. Current design and results:
[humanoid_control/walking_controller/README.md](../humanoid_control/walking_controller/README.md).
§10–§11 below are kept as history. **Date:** 2026-11-08.
**Scope:** MuJoCo simulation first; hardware is out of scope for this document.

## 1. Goal

Walk the robot to a commanded goal pose `(x, y, yaw)` on flat ground with a
model-based controller: footstep planning, centre-of-mass (CoM) planning and a
TSID whole-body controller (WBC) that outputs joint torques. No learned policy
is in the loop.

Non-goals for the first version: stairs or rough terrain, running, hardware
state estimation, obstacle avoidance beyond following a path that
`ball_chase_astar` already produces.

### Why the existing WBC cannot be reused as-is

`wbc_control_node` is a correction layer around the RL tracking policy. It
tracks the policy's PD torque and may change it by at most ±0.02 Nm
(`max_policy_torque_correction_nm`), so it cannot generate a gait or steer.
It is also deprecated: with no push, 17/48 WBC episodes fell against 1/48 for
tuned PD ([2026-11-08-wbc-deprecation.md](reports/2026-11-08-wbc-deprecation.md)).
That result concerns WBC *on top of a policy trained for direct PD*; it says
nothing about a WBC that owns the whole motion, which is what this proposal
builds. The Pinocchio/TSID model setup, contact handling and torque bounds in
`wbc_control_node/controller.py` are reused.

## 2. Robot constraints (measured from the simulation model)

Values read from `chrobot_16kg_candidate.xml` as compiled by
`humanoid_sim.model_builder.build_model`, and from `motors.yaml`.

| Quantity | Value | Consequence |
|---|---|---|
| Mass | 16.0 kg | |
| CoM height at spawn | 0.44 m | LIPM ω = √(g/z) ≈ 4.7 rad/s; plan with z_c ≈ 0.40 m to keep the knees bent |
| Forward axis | world ±y (knee and ankle axes are world x) | Frames in this document use x forward after conversion |
| Leg joints | 5 per leg: hip pitch (`hip1`), hip roll (`hip2`), hip yaw (`thigh`), knee pitch, ankle pitch | **No ankle roll** (see §4.4) |
| Hip yaw range | ±0.785 rad | Bounds the turn per step |
| Ankle range | ±0.6 rad | |
| Torque limits | hip pitch/knee 120 Nm, hip roll/yaw 60 Nm, **ankle 17 Nm** | Small sagittal ankle authority |
| Foot sole | box 0.08 m wide × 0.21 m long | Lateral support polygon is narrow |
| Stance width at spawn | 0.31 m between foot centres | |
| Physics step | 5 ms | Control above 200 Hz has no benefit |
| Floor friction | MuJoCo default 1.0 | Plan with μ = 0.6 |
| Motor interface | MIT law: `kp*(q_ref-q) + kd*(dq_ref-dq) + tau_ff`, evaluated every physics step | Allows torque feedforward plus a low-gain joint loop |

## 3. Architecture

```
/walk/goal (PoseStamped, world)        /ball_chase/global_path (optional)
              \                        /
               Footstep planner  ─────────────►  footstep queue
                       │
               CoM / DCM planner (LIPM, re-planned every step)
                       │
               Swing-foot trajectory generator
                       │
               Gait state machine (contact schedule, phase timing)
                       │
               TSID WBC  (200 Hz)  ◄── /sim/ground_truth/odom, /sim/ground_truth/feet,
                       │                /joint_states, /sim/status, /safety/estop
                       ▼
       /walk/desired_motor_subset (MotorCommand, MODE_MOTION)
                       ▼
       cpp_control_node → /motor_commands → MuJoCo
```

All boxes except `cpp_control_node` live in one new node, `walking_controller`,
so that planner and WBC share one clock and one state snapshot. Planning code is
pure Python with no ROS imports, so it can be unit-tested and run offline.

## 4. Components

### 4.1 Goal interface

| Topic | Type | Meaning |
|---|---|---|
| `/walk/goal` | `geometry_msgs/PoseStamped` | Target pose in `world`; a new goal replaces the old one at the next step boundary |
| `/walk/cmd_vel` | `geometry_msgs/Twist` | Optional velocity mode, mutually exclusive with a goal; bounded by the step limits below |
| `/walk/stop` | `std_srvs/Trigger` | Finish the current step, then stand in double support |
| `/walk/status` | `std_msgs/String` (JSON) | State, current/next footstep, distance to goal, WBC solve time, faults |

### 4.2 Footstep planner

- Unicycle model: walk toward the goal, turn in place when the heading error is
  above 0.5 rad, align with the goal yaw within 0.2 m of it.
- Initial step limits: forward 0.10 m, backward 0.05 m, lateral 0.05 m, turn
  0.15 rad per step (well inside the ±0.785 rad hip yaw range).
- Nominal stance width 0.30 m; the minimum lateral foot-centre distance is
  0.20 m, so the 0.08 m wide feet cannot collide.
- Plan a horizon of 4 steps and re-plan at every touchdown from the measured
  stance foot pose, never from the planned one.
- Goal reached: position error < 0.10 m and yaw error < 0.15 rad, then the last
  step closes the stance to nominal width.

### 4.3 CoM / DCM planner

- Linear inverted pendulum with constant height z_c = 0.40 m.
- Plan the divergent component of motion (DCM) backwards from the end of the
  horizon, with the ZMP reference at each stance foot centre (double support:
  linear transfer between feet).
- Initial timing: single support T_ss = 0.40 s, double support T_ds = 0.10 s.
- Every WBC cycle, the measured DCM error feeds back into the CoM acceleration
  command (standard DCM tracking law, gain k_ξ ≈ 2–3).

### 4.4 Lateral balance without ankle roll

The stance foot can only generate a roll moment through the hip roll joint and
a rigid shank, and in single support a 5-DOF leg cannot fix all six degrees of
freedom of the torso relative to the foot. Therefore:

- The ZMP can still move inside the sole in roll, because the foot stays flat
  as long as the contact wrench constraint holds, but the authority is small
  (±0.04 m). Lateral balance relies mainly on **footstep adjustment**: at each
  step the next lateral foot position is corrected by the measured DCM offset
  (capture-point stepping), bounded by the kinematic limits.
- Torso roll and pitch are soft tasks with low weight. Torso yaw is not
  constrained.
- If stepping in place (milestone M3) fails because of this, the fallbacks in
  order are: shorter T_ss (0.30 s), lower z_c, and a narrower stance width.

**Measured in M1: the lateral CoM is not actuated in double support.** With
both feet flat, the null space of the contact constraints has 5 DOF. Projected
onto it, the CoM Jacobian has singular values ≈ 0.21 (forward), 0.03–0.05
(height) and ≈ 5e-4 (along the line between the feet), for knee angles −0.3 to
−1.2 rad. The pelvis orientation stays controllable. Consequences:

- The WBC masks the lateral CoM axis in double support. Leaving it in demands
  huge joint accelerations after a sideways push.
- **Lateral weight shift in double support is not possible** (M2 as first
  written). The CoM can move sideways only while one foot is unloaded, so the
  frontal-plane motion must come from single support, with short double
  support phases that do not have to carry lateral CoM velocity.

### 4.5 Swing-foot trajectory

- Quintic position profile from lift-off to touchdown, apex height 0.04 m,
  with zero velocity and acceleration at both ends.
- Yaw interpolated linearly to the target yaw; roll and pitch held at zero.
- Early touchdown (normal force > 30 N on `/sim/ground_truth/feet`) ends the
  swing; a late touchdown extends it by up to 0.05 s, after which the
  controller lowers the foot at 0.1 m/s and records a fault if it still has not
  landed.

### 4.6 Whole-body controller

TSID `InverseDynamicsFormulationAccForce` with `SolverHQuadProgFast`, the same
model as `wbc_control_node` (`human_offset_corrected.urdf` plus its
`.dynamics.json` for armature and damping).

| Task / constraint | Type | Weight / priority | Gains |
|---|---|---|---|
| Rigid-body dynamics | equality | hard | |
| Torque limits from `motors.yaml` | bounds | hard | |
| Stance-foot contact (`left_foot`, `right_foot`) | 6D contact, friction cone μ = 0.6, CoP inside the sole, normal force 0–400 N | soft, weight 1e3 (M1: a hard contact makes the QP infeasible once a real foot rocks) | contact motion task kp 100 |
| CoM | equality, **lateral axis masked in double support** (§4.4) | 1.0 | kp 15 horizontal (CoP-bound), 40 vertical; M2+: from DCM law (§4.3) |
| Swing-foot position + yaw | SE3, mask xyz + yaw | 1.0 | kp 200, kd 2√kp |
| Swing-foot roll, pitch | SE3, mask roll/pitch | 0.1 | kp 100 |
| Pelvis roll, pitch, yaw | SE3, orientation only | 0.1 | kp 50 |
| Joint posture (knees bent) | joint posture | 0.01 | kp 20 |
| Contact-force regularisation | | 1e-5 | |

The swing foot has 6 targets but only 5 leg joints, so its roll and pitch get a
lower weight than its position and yaw.

At a contact switch, the force bound of the leg being unloaded ramps to zero
over T_ds before the contact is removed. This prevents force discontinuities.

### 4.7 Command output

Each cycle the QP returns joint torques `tau`. The node publishes a
`MotorCommand` in `MODE_MOTION` with `torque = tau` as feedforward plus a stiff
joint PD toward a **planned posture anchor** (changed during M1):

- `position = q_anchor`, `velocity = dq_anchor`; `q_anchor` is the planned
  posture (stand: min-jerk from the landing posture to the nominal one, same
  profile as the CoM reference) plus a correction that follows the measured
  posture with a 2 s time constant
- `kp` 100 (ankle 30), `kd` 2

Two alternatives were measured offline and rejected:

- PD counted once at the sample (`torque = tau - PD`, low gains, the original
  plan): no position stiffness. The pelvis is 3.5 of 16 kg; a 20 N push
  pitches it 30° in 70 ms, before the QP reacts. 11/48 recoveries at 15–35 N.
- Integrating the QP accelerations into a persistent `q_des`/`v_des`: with the
  feet pinned, residual accelerations accumulate in `v_des` and the QP becomes
  infeasible within ~5 s even without pushes.

An anchor that only trails the measurement keeps the robot straight-legged
(the PD resists the squat the WBC asks for).
- `valid_until = stamp + 0.08 s` (`sim_autostart` needs > 0.04 s of validity to release the gantry); a stale command disables output in C++

`cpp_control_node` gets the new topic through the same remapping that
`sim.launch.py` already uses for `/wbc/desired_motor_subset`.

### 4.8 Timing

- Control at 200 Hz, one cycle per physics step. The existing WBC measured a
  mean QP solve of 0.19 ms ([2026-10-30-wbc-pd-comparison.md](reports/2026-10-30-wbc-pd-comparison.md)).
  The budget for the whole Python cycle (state conversion, planners, TSID
  setup, publish) is 2 ms; `/walk/status` reports p99 and maximum cycle time.
- Footstep and DCM re-planning run only at touchdown, not every cycle.
- If the 2 ms budget is not met, first move the planners out of the cycle; only
  then consider C++.
- The cycle runs once per physics sample, triggered when odom, feet and
  `/joint_states` with the same stamp have arrived (node parameter `trigger`,
  default `state`), not on a free-running timer. The simulator receives the
  commands on its own executor. What limits walking is the total loop latency
  (sample → torque applied), not the cycle time: without compensation it must
  stay below ~15–20 ms (§12); with the state prediction of §14 the limit is
  ~30 ms when the prediction matches and 20 ms with 10 ms prediction error.

### 4.9 State input (simulation only)

- Base pose and twist from `/sim/ground_truth/odom`. Joint state comes from
  `/joint_states`, foot contact from `/sim/ground_truth/feet`: sole heights are
  in `position`, normal forces in `effort`.
- Contact is detected when the normal force is above 30 N; the planner's
  schedule decides which foot should be in contact, and the measurement only
  confirms early or late touchdown.
- The conversion from the MuJoCo base frame to the Pinocchio base frame is
  already implemented in `WBCController.state_from_sim` and is reused.

### 4.10 Gait state machine and safety

```
WAIT_RELEASE ─► STAND ─► (goal) ─► WALK ─► (goal reached / stop) ─► STAND
     │             │                  │
     └─────────────┴──────────────────┴─► FAULT (outputs disabled, no auto-recovery)
```

- `WAIT_RELEASE`: gantry held; publish zero-gain zero-torque arming commands,
  as the current WBC bridge does, so `sim_autostart` can verify the path.
- `STAND`: double support, CoM above the support centre, posture task active.
- FAULT triggers: QP failure, input older than 0.1 s, cycle time over 10 ms,
  torso tilt over 0.5 rad, CoM outside the support polygon by more than 0.05 m
  in double support, `/safety/estop`. In FAULT the node stops publishing; C++
  expiry and the safety chain then disable the motors. There is no bypass.
- `/sim/reset` returns the node to `WAIT_RELEASE`.

## 5. Packaging and launch

- New package `humanoid_control/walking_controller` (ament_python):
  - `planning/` (footsteps, DCM, swing) — no ROS, unit tested
  - `wbc.py` — TSID problem, importing the reusable parts of
    `wbc_control_node.controller` (or moving them to a shared module)
  - `node.py` — ROS wrapper, state machine and status
- `sim.launch.py`: new argument `controller` with values `policy` (default,
  unchanged behaviour) or `model_walk`. `model_walk` disables
  `tracking_policy_node` and WBC, starts `walking_controller` and remaps
  `cpp_control_node` to `/walk/desired_motor_subset`.
- Dependencies: the existing WBC environment from `scripts/setup_wbc.sh`;
  nothing new.

## 6. Milestones and acceptance criteria

Each milestone is run headless with seeded episodes, as in
`scripts/collect_fall_data.py`, before moving on. Push tests reuse `scripts/collect_fall_data.py --profile recovery`.

| # | Milestone | Acceptance |
|---|---|---|
| M1 | Stand after gantry release | 20/20 seeds stand 20 s; torso tilt < 0.1 rad; recovers from the `--profile recovery` pushes in ≥ 15/20 — **done except push (§10)** |
| M2 | ~~Lateral weight shift in double support~~ (impossible, §4.4) → lift one foot and balance on the other | Unload one foot and lift it 2 cm, hold 2 s on each foot, 20/20 seeds; needs a definition decision |
| M3 | Step in place | 30 s, 20/20 seeds; foot slip < 0.02 m per step; drift < 0.2 m |
| M4 | Walk straight | 0.10 m/s for 5 m, ≥ 18/20 seeds; lateral drift < 0.3 m |
| M5 | Turn and side-step | 90° turn in place and 0.5 m side-step, ≥ 18/20 seeds |
| M6 | Walk to goal | 10 random goals 1–3 m away, ≥ 9/10 reached within 0.10 m / 0.15 rad without a fall |
| M7 | Follow A* path | `/ball_chase/global_path` around one obstacle, ≥ 4/5 reach the goal |

M3 is the go/no-go point for this approach (§4.4).

## 7. Testing

- Unit tests (pytest, no ROS): footstep limits and collision-free widths, DCM
  plan continuity and terminal condition, swing boundary conditions, gait
  timing.
- WBC tests: on the gantry pose and a few random configurations, the QP is
  feasible, the torques are within limits and the contact forces are inside the
  cone; TSID dynamics match MuJoCo's mass matrix and gravity (existing approach
  in `experimental/reference_warp`).
- Simulation tests: one headless smoke test per milestone in CI-sized form
  (single seed, short duration); full seed sweeps run manually and are recorded
  under `docs/reports/`.

## 8. Risks

| Risk | Likelihood | Mitigation |
|---|---|---|
| No ankle roll makes single support unstable | High | Capture-point stepping, faster steps, lower CoM; M3 decides |
| 17 Nm ankle limits sagittal ZMP control | Medium | Rely on DCM stepping in x as well; shorter steps |
| Python cycle exceeds budget | Low–medium | Planners outside the cycle; C++ only if needed |
| Contact switching impacts | Medium | Force ramps over T_ds, touchdown velocity ≤ 0.1 m/s |
| Model mismatch between URDF and MuJoCo XML | Low | Mass matrix / gravity equivalence test before M1 |
| Ground-truth state hides estimation problems | Certain for hardware | Explicitly out of scope; hardware needs an IMU + kinematics estimator |

## 9. Open questions

1. Is a hardware deployment expected later? If so, the state estimator should be
   designed alongside M4 instead of after M7.
2. Should the planner accept the A* path directly (M7), or should a separate
   path follower convert it to `/walk/goal` waypoints?
3. Can `wbc_control_node`, which is deprecated, have its reusable parts moved
   into a shared module, or should they be copied into the new package?

## 10. M1 results (2026-11-08)

Offline loop (`walking_controller.offline`: same MuJoCo model, gantry and MIT
actuator law as `humanoid_sim`, 10 ms command delay), default parameters:

| Test | Result |
|---|---|
| No push, 20 seeds × 20 s | **20/20 stood**; max pelvis tilt 0.033 rad; final CoM height error +2 mm |
| Recovery pushes 15–35 N (seeds 48–95) | **38/48 recovered**; every fall had ≥ 3.6 N·s; largest recovery 4.2 N·s |
| Recovery pushes 35–65 N (seeds 96–143) | 8/48 recovered |
| Controller cycle (single process) | 0.27 ms mean, 0.64 ms p99, 1.3 ms max; QP 0.06 ms mean |

The push criterion (≥ 15/20) is met for 15–35 N but not for 35–65 N. Pushes
above ~3.6 N·s (CoM velocity ≈ 0.22 m/s, capture point ≈ 5 cm out) need a step
or a better use of the CoP; both come with M3.

Full ROS chain (`controller:=model_walk auto_start:=true`, CycloneDDS,
headless, IMU-timeout safety trip enabled): **3/3 runs** released the gantry,
landed, squatted to the nominal posture (knee −0.90 rad, CoM 0.386 m against
0.382 m) and stood about 27 s until shutdown, with no estop and control cycles
under 3.5 ms. Earlier runs on the same host hit the 0.3 s IMU-freshness estop
while a second simulation was running. The RL controller hit it too under that
load (2/2), so the estop came from host load, not from this controller. The GUI
launch (`use_viewer:=true`) also stands.

Findings that changed the design (§4.4, §4.6, §4.7):

1. Lateral CoM not actuated in double support → masked CoM axis; M2 redefined.
2. TSID: a task mask set after `addMotionTask` leaves a stale constraint row;
   the CoM task is re-added when the mask changes (regression test).
3. Horizontal CoM gain is CoP-bound: kp 60 asks for a CoP 11 cm ahead of the
   CoM, beyond the toe. kp 15 horizontal.
4. The CoM height target must come from the nominal posture's kinematics, or
   the posture task holds the CoM 1.7 cm high.
5. Hard contacts become infeasible when a real foot rocks → soft, weight 1e3,
   `f_min = 0`.
6. Joint stiffness toward a planned anchor (§4.7) is required for pushes on the
   light pelvis.

## 11. Rocking feasibility experiment (2026-11-08)

**Why rocking.** Static single support is impossible from the 0.31 m stance:
the CoM is 0.155 m from either foot and cannot move sideways in double support
(§4.4). The stance cannot be narrowed either: with no ankle roll, the
foot tilts sideways ~1.5° per cm of narrowing, and the IK cannot reach 0.19 m
even with a 7.4° tilt. The only way to unload a foot is lateral rocking in
single support, with the CoP at the stance foot (linear inverted pendulum,
ω ≈ 5.05 rad/s).

**Method.** `walking_controller/rock.py`, offline loop only. After the M1
stand, support alternates by a DCM rule: with ξ = x + v/ω, switch when ξ is
within b of the swing foot, b = 2d / (1 + e^(ωT)), so the step time is T
(b ≈ 0.055 m for T = 0.3 s). Lateral CoM follows the LIPM with the CoP at the
stance-foot centre; forward CoM and height are held; the swing foot lifts 1–4
cm and returns to its nominal pose. Double support is instantaneous. Required
lateral CoM speed at each switch: 0.44–0.60 m/s for T = 0.25–0.40 s.

A flat foot does not rule this out: the ground reaction force points from the
CoP to the CoM, which needs μ ≈ 0.41 (simulation floor: 0.65–1.15).

**Result: not achieved.** About 20 variants (step time 0.3/0.4 s, lift 1–4
cm, swing/pelvis/CoM task weights, joint loop torque-only or stiff per-step
integrated, diagnostic kick-start). None reached a steady rocking orbit:

| Best variants | Result |
|---|---|
| Torque-only joint loop (as M1 sample mode) | Swing foot never unloads, or the pelvis pitches 27° in 0.3 s; fault at 0.5 s |
| Stiff integrated joint loop (kp 100) | Swing foot lifts 11 mm, but both feet unload together (vertical drop), hard landings; fault at 0.55 s, robot down |
| Same + CoM task weight 10–30 | Up to 8 switches over 1.4 s, height held, but steps stay 0.07–0.14 s (design 0.3 s), feet slide up to 14 cm; pelvis tilt fault |

Observed failure mechanisms:

1. **Start from rest.** With the CoM centred, lifting a foot makes the robot
   fall towards that foot at once (DCM reaches the switch point after ~0.1 s).
   The leg must retract faster than the body drops onto it. The controller
   either keeps the swing foot loaded (no fall, no rocking) or lets the stance
   leg give way.
2. **Instant switches with a lateral lock.** At each touchdown the CoM moves
   sideways at ~0.5 m/s, but with both feet flat it cannot move sideways at
   all, so every touchdown is an impact; feet slide by centimetres.
3. **Pelvis pitch.** The light pelvis (3.5 of 16 kg) is again the first thing
   to go: it pitches 10–27° within a few switches.

**Assessment.** The LIPM rocking gait is not ruled out physically, but this
controller (flat 6D contacts, instantaneous support switches, no impact or
start-up handling) does not reach it. Getting further would need at least: a
start-up manoeuvre that creates lateral momentum, touchdown/impact handling
with a short force-ramped double support or edge (line) contacts that let a
foot roll, and slip-aware contact switching. That is new research work with an
uncertain outcome, not tuning. The RL policy already walks on this hardware.

## 12. ROS-chain latency on the Jetson (2026-11-08)

**Symptom.** On the Jetson host (aarch64, 6 cores, ROS 2 Humble) the full
chain fell after 25–37 s of walking at 0.06 m (3/3, `Torso tilt`); the
offline loop walked 60 s. Ruled out from recordings: collision with the
target board (behind the robot, no collision geometry), twist frame (world in
both), joint/odom misalignment, missing commands before the fall, foot-force
computation, thermal derating, a different MJCF.

**Mechanism.** Torque-level control is delay-sensitive. With more delay the
yaw swings grow (−7° vs ±1.5° offline), one knee then straightens to 0 rad
(offline it stays ≤ −0.92), the CoM rises and the robot falls. Steps also
became too long (0.069 m/step against 0.042 m offline at the same command).
Offline, 9 s in place then 0.06 m:

| Command delay | 10 ms | 15 ms | 20 ms | 25 ms | 30 ms | 40 ms |
|---|---|---|---|---|---|---|
| Result | 60 s | 60 s | fall 35 s | fall 29 s | fall 25 s | fall 14 s |

The `walk_eval.py sweep` (20 s episodes, up to 20 ms) ends before these falls.

**Measured latency** (recorded at full rate; joint sample matched to the
command's `q_ref`, applied torque by cross-correlating `tau_ff` with
`/joint_states` effort):

| Segment | Before | After |
|---|---|---|
| Joint sample → controller publish (median) | 14.7 ms | 7.3 ms |
| Controller publish → torque applied | 10 ms | 7 ms |
| Controller cycle mean / p99 | 4.6 / 6.1 ms | 2.8 / 4.3 ms |
| Command rate | 151 Hz | 187 Hz |

Only 4.3 ms of the 14.7 ms was computation; the rest was the 200 Hz timer's
phase and executor queueing, and the joints used were usually one physics
step older than odom (they arrive ~1.3 ms after odom of the same sample).

**Changes.** Same-sample trigger instead of the timer (sensor queues depth 1);
the MPC's `lsq_linear` (1.09 ms per axis) replaced by a warm-started
active-set box QP (0.43 ms, same optimum to 4e-12 on 7104 recorded problems,
`lsq_linear` fallback); the simulator's dedicated command executor enabled for
`model_walk`. Offline results are unchanged (identical step counts and
distances).

**Result.** 3/3 chain runs without a fall, 2 × 53 s and 141 s (359 steps,
14.8 m, 0.041 m/step). Remaining latency is mostly outside the controller:
odom published up to one physics step after sampling (input age 4.4 ms mean),
C++ forwarding on its own 200 Hz timer (2.8 ms median) and application on the
next physics step. The margin to ~20 ms is small: a regression should
include ≥ 20 ms delay over ≥ 40 s. Latency compensation followed (§14).

## 13. Step length shortfall (2026-11-08)

**Symptom.** Offline, realised / commanded step length was 0.76 / 0.73 /
0.65 / 0.61 for 0.02 / 0.05 / 0.08 / 0.10 m.

**Hypothesis (early touchdown) confirmed.** Logged per step: every touchdown
came from the force condition (> 20 N) at s = 0.58–0.63, i.e. at the earliest
s the gate allows (> 0.6); none from the s ≥ 1 height condition. The swing
foot had then covered min-jerk m(0.6) = 0.68 of its travel, which matches the
achieved travel (0.10 m: planned 0.163, achieved 0.112 m, 0.68 × 0.163 =
0.111). Why the floor is reached at s ≈ 0.6 with a 2.4 cm reference there:
the foot frame peaks at only 1.6 cm (s = 0.4) and falls behind the reference,
and the foot rolls up to 7.4° sideways (no ankle roll), which lowers one sole
corner by another ~1.5 cm (corner spread 3 cm).

**Fix.** The horizontal swing trajectory finishes at `swing_fwd_end` of SS
(0.9 here; 0.85 since the latency compensation, §14); height still uses the
whole phase. Evaluated on the delay ×
friction sweep, fixed 0.02–0.10 m and the variable schedule (17 episodes per
candidate) plus 15 ms delay over 49 s:

| `swing_fwd_end` | Realised / commanded 0.02 / 0.05 / 0.08 / 0.10 | 17 episodes | 15 ms, 49 s |
|---|---|---|---|
| 1.0 (before) | 0.76 / 0.73 / 0.65 / 0.61 | 17/17 | walks |
| 0.9 (default) | 0.98 / 0.90 / 0.79 / 0.75 | 17/17 | walks |
| 0.85 | 1.03 / 0.99 / 0.81 / 0.75 | 17/17 | falls at 46 s |
| 0.8 | 1.18 / 1.06 / 0.88 / 0.99 | 0.10 m falls at 12.7 s | — |

Also tried: lift 4–5 cm (0.68–0.80 at 0.10 m, 5 cm falls), swing weight 10
and kp 400 (0.90, but larger lateral drift), orientation weight 1 (0.63);
combinations of a higher lift with front-loading fell in several episodes.

**Result.** `walk_eval.py sweep`: 12/12 before and after; at 0.06 m 1.72–2.16
m before, 2.08–2.67 m after in 20 s. ROS chain on the Jetson (~15 ms) at 0.06:
0.041 → 0.049–0.052 m per step, 3/3 without a fall (2 × 53 s, 141 s / 361 steps
/ 17.8 m). Commands of 0.15 / 0.20 m (outside the node's 0.10 m clamp) now fall
on the first step instead of after 22 / 2: the old steps survived only because
they came out much shorter. Above ~0.08 m the step is still short:
s = 0.8 realises a full 0.10 m step and then falls, so what limits larger steps
now is balance (the open 0.15 / 0.20 m large-step work), not touchdown timing.

## 14. Latency compensation (2026-11-09)

**Method.** `GaitController.step` predicts the measured state to when the
command acts: horizon h = input age (sample stamp → cycle start, measured by
the node; 0 offline) + `predict_s` (fixed delay after the cycle starts:
compute, transport, actuation; 10 ms, which is both the Jetson chain's and
the offline loop's default). q⁺ = integrate(q, v·h), v⁺ = v. The controller
clock is shifted by `predict_s`, so references and phase timing refer to the
same instant (no measurable effect on its own). Not applied while the gantry
holds the robot.

**Offline**, in place 9 s then 0.06 m, 49 s:

| Command delay | none | prediction = delay, constant v | + last WBC acceleration | 10 ms prediction (default) |
|---|---|---|---|---|
| 10 ms | walks | walks | walks | walks |
| 20 ms | falls 30 s | walks | walks | walks |
| 25 ms | falls 32 s | walks | walks | — |
| 30 ms | falls 18 s | walks | swing never touches down, 16 s | falls 20 s |
| 40 ms | falls 10 s | swing never touches down, 6 s | falls 4 s | — |

The QP acceleration is too noisy to extrapolate. Half the prediction (5 ms)
falls at 20 ms after 42–46 s.

**Step length.** Steps grow with uncompensated delay, so with prediction they
are shorter (s = 0.9: 0.98 / 0.90 / 0.79 / 0.75 → 0.77 / 0.74 / 0.62 / 0.57
at 0.02 / 0.05 / 0.08 / 0.10 m). `swing_fwd_end` 0.85 gives 0.88 / 0.78 /
0.67 / 0.62 and passes all 17 evaluation episodes and 20 ms / 49 s; 0.8 falls
at 0.10 m. Default: prediction on, 0.85.

**ROS chain (Jetson).** 3/3 without a fall: 2 × 53 s and 140 s (364 steps,
14.9 m, 0.041 m/step at 0.06, lateral ≤ 0.09 m). The node's prediction horizon
was 14.4 ms median (12–17 ms), matching the ≈ 15 ms measured in §12.

**Overload (not a latency case).** With two offline episodes, a recorder and
the chain (more than nproc − 1 busy processes), the commands stalled for 80 ms
while stepping in place: the simulator paused physics (`command expired for
left_hip1_joint`), the IMU stopped publishing and the 0.3 s IMU-freshness
estop latched 0.3 s later. This is the likely mechanism of the earlier
IMU-freshness estop under load; the estop is the last link, not the cause.

## 15. Turning (2026-11-09)

**Interface.** `StepParams.turn_rad` / `/walk/turn` (Float64, rad per step,
+ = left, clamped to ±0.15 rad as in §4.2), combinable with the step length.
`/walk/cmd_vel` and `/walk/goal` (§4.1) can be built on it.

**Method.** At lift-off the swing target (level stance foot × nominal
offset) is rotated by the turn about the stance foot's vertical axis, then
the step length is added along the new forward axis. Once a turn has been
commanded:

- the walking frame (MPC lateral/forward axes, origin) is rotated at each
  lift-off to the measured stance-foot yaw + turn/2 (closed loop; rotating it
  open loop by the command let it lag the feet by 20° and fell);
- the swing foot's yaw goes from its start to the target by s = 0.5 of SS
  (`swing_yaw_end`). Each foot moves every other step, so a swing turns its
  foot by about twice the turn; interpolating only the commanded turn, or
  finishing at s = 0.85, realised ~50 % of the turn (touchdown at s ≈ 0.6). A
  higher swing-orientation weight did not help;
- the torso yaw reference follows the frame heading through the swing;
- **the sole-centre offset (foot frame → sole centre) is rotated with the
  foot's yaw.** It was a world vector from the start, so after ~150° the
  support regions were several cm off: the feet midpoint drifted 0.4 mm per
  degree from the frame, walking after a 150° turn fell within ~4 s and
  continuous turns ≥ 0.10 rad/step fell after 20–33 s (stance widening,
  hip roll drifting, more often to the left). A first diagnosis from
  0.5 s samples (one per step) read the normal lateral sway as a bias.

With turn 0 and no earlier turn everything is bit-identical to the straight
gait (`walk_eval.py` sweep and variable outputs unchanged).

**Results (offline).** Delay 10/15/20 ms × friction 0.65/1.15 × {0.15 rad
left, right, 0.10 rad on a 0.06 m arc}, 20 s: 18/18; per step 6.7–7.6° at
0.15 rad (78–88 %), 5.0–5.6° on the arc (87–98 %). In place at 0.10 rad:
94 % (left). 49 s at 15 ms: continuous 0.10 / 0.15 / −0.15 rad (590–780°),
arc, and a chase pattern (turn ~90°, walk 3 s, turn back, × 4) at 15 and
20 ms all walk. M5's 90° in-place turn: met both ways (not yet over 20
seeds; the offline loop has no random seed, only delay × friction).

**ROS chain (Jetson).** 2/2 without a fall: +0.10 rad/step for 11.5 s →
+155° / +166°, walk 0.06 m for 7 s, −0.10 rad for 11 s → −131°, walk 15.5 s.
After a turn the frame follows the feet, so there is no fixed heading: straight
walking then drifted −7 to +14° in yaw. Heading hold belongs to the caller
(ball chase, `/walk/goal`), which closes the loop with `/walk/turn`.

## 16. Camera ball chase with model_walk (2026-11-09)

`ball_chase.launch.py controller:=model_walk` (the restriction "model_walk
requires enable_vision:=false" in sim.launch.py was untested, not a known
failure, and is removed) feeds `/ball_chase/cmd_vel` (TwistStamped, ≤ 0.2 m/s,
≤ 0.5 rad/s) to the walking node: per step × 0.5 s, capped, stale 0.5 s → stop.

Findings, in the order they came up:

1. **First command is the maximum** (vision locks on 1.7 m ahead: 0.2 m/s,
   0.24 rad/s → 0.10 m and 0.12 rad from the first steps) and the robot fell
   after 4 s. Offline at 15 ms a 0.10 m step falls even straight (15 s);
   0.06 m with ±0.12 rad turns walks at 15 and 20 ms, 0.08 m with a turn
   falls at 20 ms → `cmd_vel_max_step_m` 0.06.
2. **Heading limit cycle.** The pursuit's yaw gain 1.5 rad/s per rad is 0.75
   rad of turn per step per rad of heading error; with 0.5–1 s vision lag
   (one detection every few hundred ms, sometimes a wrong one) the turn
   flipped between ±0.15 each step and the yaw swung −13…+28°. Offline with
   the same pursuit law on ground truth (2 Hz, 0.7 s lag): yaw std 19° at
   0.15, 6° at 0.08, 3° at 0.05 (but 34 sign flips); a low-pass made it worse
   → `cmd_vel_max_turn_rad` 0.08. Chain: yaw −1…+21°.
3. Ruled out offline: the camera mount (0.3 kg, not in the controller's
   model; the offline loop can now load the chase scene via `world=`), the
   simulator running slower than real time (real-time factor 0.77–0.95 with
   the camera; a controller clock 15 % faster than physics still walks) and
   the vision lag itself.

**Result.** The robot walks from 1.7 m to the ball and stops 0.5–0.6 m in front
of it, facing it; 2 of the last 3 runs had no fault for 45–80 s. A ball moved
90° to the side leaves the 58° view and the pursuit stops (no search).

**Open.** One run stopped at 44 s: a command arrived 77 ms old (validity 80
ms), the simulator paused physics, inputs went stale and the IMU-freshness
estop latched — the same chain as the overload case in §14. With the camera
the simulator renders and publishes images in the Python process that also
receives commands; whether that or the controller stalls is the next thing to
measure.

