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
