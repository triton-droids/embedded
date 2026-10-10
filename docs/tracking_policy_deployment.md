# Tracking Policy Deployment Guide

How to deploy the mjlab / rsl_rl motion-tracking ONNX policy on the Triton
humanoid legs, and every convention, quirk and measurement that had to be
established to make it work.

Companion scripts:

```
ctrl_scripts/run_tracking_policy.py           the 50 Hz runner
ctrl_scripts/run_tracking_config.json         its config
utils/validation_code/joint_direction_monitor.py   read-only direction checks
```

(The older 60 Hz rl-games runner, `run_policy.py`, used stacked, scaled,
interleaved observations. It was removed in the October 2026 cleanup and is in
the branch history.)

---

## 1. The policy

Trained with **mjlab** (MuJoCo) + **rsl_rl** PPO, a BeyondMimic-style motion
tracking task. Source run:

```
simulation/logs/legs_tracking/20260914_170827/
  20260914_170827.onnx      <- deploy this
  agent.json                 rsl_rl config
  tracking_task.py           task definition and observation overrides
  reference.npz              the walking reference, 299 frames @ 50 fps
  chrobot_16kg_candidate.xml robot model
```

### The ONNX is self-contained

```
inputs : obs [1,56]    time_step [1,1]   (float, not int)
outputs: actions [1,10]
         joint_pos [1,10]  joint_vel [1,10]
         body_pos_w  body_quat_w  body_lin_vel_w  body_ang_vel_w
```

The observation normalizer (`policy.obs_normalizer`) **and the entire 299-frame
reference motion** are baked into the graph. Feed it a `time_step` and it
returns the reference pose for that frame; feed that back into `obs[0:20]` on
the next step. No `.npz` to ship, no normalizer state to reconstruct. Verified
against `reference.npz` to a max absolute error of `2.89e-08`.

Network: `56 -> 128 -> 64 -> 10`, ELU. Only `onnxruntime` is needed — torch is
not required anywhere in the deploy path.

`time_step` **clamps, it does not loop**. Frames past 298 return frame 298
forever, and the start and end poses differ enough that wrapping to 0 would be a
discontinuity. The runner therefore stops at frame 298 — about 6 seconds.

### Observation layout (56)

| idx | term | dim | source |
|---|---|---|---|
| 0:10 | reference `joint_pos` | 10 | ONNX output at `time_step` |
| 10:20 | reference `joint_vel` | 10 | ONNX output at `time_step` |
| 20:23 | `base_ang_vel` | 3 | gyro, rad/s, **unscaled** |
| 23:33 | `joint_pos - default` | 10 | encoders |
| 33:43 | `joint_vel` | 10 | encoders |
| 43:53 | last action | 10 | previous output |
| 53:56 | `projected_gravity` | 3 | IMU, **= -up_body** |

Derived from `mjlab/tasks/tracking/tracking_env_cfg.py` plus the overrides in
`tracking_task.py`, which pop `motion_anchor_pos_b`, `motion_anchor_ori_b` and
`base_lin_vel`, then append `gravity`. Dict insertion order fixes the layout.

Things that are easy to get wrong here:

- **No scaling anywhere.** Raw SI units. The baked-in normalizer handles it.
- **`projected_gravity`, not an up vector**: the opposite sign from an IMU
  "up" vector.
- **No frame stacking.**
- **`gravity` was trained with no noise term.** Every other actor observation
  has a `Unoise`; this one does not. The policy has never seen a corrupted
  gravity vector, so it has no robustness margin there. Mount the IMU well.

### Action semantics

```
joint_target = default_joint_pos + 0.2 * action
```

`default_joint_pos` is **all zeros**. The model has no keyframe and
`EntityCfg.InitialStateCfg.joint_pos` defaults to `{".*": 0.0}`. So the robot's
physical zero — legs straight — *is* the pose the network expects, and
`obs[23:33]` is just the measured angle.

This overrides `robot_hardware.DEFAULT_JOINT_POS_REAL_RAD_BY_JOINT`, which is a
crouched pose meant for the velocity policy. Getting this wrong offsets every
action.

### Control rate

`sim timestep 0.005 x decimation 4` = **50 Hz**, matching `walking_reference_50hz.npz`.

---

## 2. Coordinate frames

### The base frame is NOT x-forward

```
+X = RIGHT      +Y = FORWARD      +Z = UP
```

Not the IsaacLab `x-forward, y-left, z-up` convention. Established from:

- Foot collision box `size="0.04 0.105 0.00763"` — 8 cm across X, 21 cm along Y.
  Feet are long fore-aft, so Y is the fore-aft axis.
- The reference motion translates **+3.35 m along +Y** over 5.98 s at 0.56 m/s.
- Right-handedness with +Z up then forces +X = right.
- Consistent with `hip1` (axis `1 0 0`) swinging +/-0.35 rad symmetric about zero
  (hip flexion), and `knee` being negative-only (human-like flexion).

The IMU site is `<site name="imu" />` directly in `floating_base` with no `pos`
and no `quat`, so `base_ang_vel` and `projected_gravity` are in the same frame.
One transform covers both.

### The left/right naming is mirrored

**Joints prefixed `left_` are physically on the robot's RIGHT side, and vice
versa.** The sim XML places the `left_` bodies at +X, which is the right side.
Hardware uses the same mirrored names.

```
motors 1-5   -> robot's PHYSICAL RIGHT leg
motors 6-10  -> robot's PHYSICAL LEFT leg
```

Because the mirror is consistent across sim and hardware, **no motor remapping
is needed**. But never use these names to decide which physical leg to touch,
and never write a test instruction in terms of "the left leg".

---

## 3. Joint direction conventions

### Verified directions

Each was confirmed by hand back-driving the joint with torque off and watching
the reported angle. Instructions are phrased relative to the robot's own body so
there is no observer-relative ambiguity.

| Motor | Joint | Push this way | Expect | Result |
|---|---|---|---|---|
| 1 | left_hip1 | swing leg forward | + | correct |
| 2 | left_hip2 | swing leg inward, toward the other leg | + | **INVERTED** |
| 3 | left_thigh | rotate foot toe-out | + | **INVERTED** |
| 4 | left_knee | bend knee, heel toward buttock | - | correct |
| 5 | left_ankle | lift toes up | + | correct |
| 6 | right_hip1 | swing leg forward | + | correct |
| 7 | right_hip2 | swing leg outward, away from the other leg | + | correct |
| 8 | right_thigh | rotate foot toe-out | + | **INVERTED** |
| 9 | right_knee | bend knee, heel toward buttock | - | correct |
| 10 | right_ankle | lift toes up | + | correct |

### Why exactly these three

The hardware uses a **body-symmetric** convention for the lateral joints
(outward is positive on both legs). The sim is **world-aligned** — both hip2
joints have literally the same axis `0 1 0`, so positive is the same world
direction for both. Those agree on one leg and disagree on the other, so exactly
one hip2 needs flipping. It is motor 2.

The thighs are body-symmetric in both sim and hardware but with opposite
polarity, so both flip. The sagittal joints (hip1, knee, ankle) are identical
under either convention, and all six matched.

Corrected in `run_tracking_config.json`, **not** in `robot_hardware.py`:

```json
"joint_sign_by_joint": {
  "left_hip2_joint":  -1.0,
  "left_thigh_joint": -1.0,
  "right_thigh_joint": -1.0
}
```

`INVERSION_ARRAY` is shared by `gain_tuner.py`, `motor_health_gui.py` and the
validation tools, which are calibrated against it. The disagreement is specifically between the sim and
hardware conventions, so the bridge belongs in the runner.

The sign is applied in **both** directions: hardware -> sim on the observed
`joint_pos`/`joint_vel`, and sim -> hardware on the commanded target.

### Limits follow the sign

`robot_hardware.JOINT_LIMITS_RAD_BY_JOINT` was transcribed from the model, so
those values are in the **sim** convention. For a sign-flipped joint the
hardware's reachable interval is the negated-and-swapped one. The runner maps
limits through `joint_sign` rather than trusting the stored pair:

```
left_hip2  sim (-1.5700, +0.4363)  ->  hardware (-0.4363, +1.5700)
thighs     symmetric +/-0.785      ->  unchanged
```

**This mapping is inferred, not measured on hardware.** It is bounded by an
explicit clamp until someone verifies it:

```json
"joint_clamp_rad_by_joint": {
  "left_hip2_joint":  [-0.5, 0.5],
  "right_hip2_joint": [-0.5, 0.5]
}
```

The reference only uses hip2 between -0.14 and +0.12 rad, so +/-0.5 is ample.

### Re-verifying directions

Read-only. Never enables torque, never writes a target, never sets zero. Motors
must be **powered but not enabled**.

```bash
# guided sweep with per-joint verdicts and a summary table
./.venv/bin/python utils/validation_code/joint_direction_monitor.py --sequence all --seconds 25

# one joint
./.venv/bin/python utils/validation_code/joint_direction_monitor.py --joint 4 --seconds 25

# free-run table (diagnostics only, not for verdicts)
./.venv/bin/python utils/validation_code/joint_direction_monitor.py --motors 2,7 --seconds 40
```

Move at least 10 degrees, slowly, and brace the parent link so only one joint
moves. Move slowly — the motors damp when back-driven fast even unpowered.

Do the cross-check joints (2, 4, 7, 9) first. They are known-answer tests: if one
of them disagrees with the prediction, the frame derivation or the test method is
wrong and none of the other results can be trusted.

---

## 4. IMU

### Hardware

ESP32 dev board (dual USB-C: one to a CH343 UART bridge, one native USB-Serial-JTAG)
with an MPU-6050 on I2C. Firmware: `utils/imu_read/imu_read.ino`.

```
MPU-6050    SDA=GPIO22  SCL=GPIO21   addr 0x68
CAN         TX=GPIO5    RX=GPIO4     (unused in the serial path)
accel       +/-2 g    -> 16384 LSB/g
gyro        +/-250 dps -> 131 LSB/dps
DLPF        reg 0x1A = 0x03 (~44 Hz)
sample      200 Hz firmware-side, ~143 Hz over serial
```

**Plug into the UART port**, which enumerates as `1a86:55d3` (QinHeng CH343).
The native USB port (`303a:1001`, "USB JTAG/serial debug unit") is silent,
because `Serial.printf` maps to UART0 unless the sketch is built with
`USB CDC On Boot: Enabled`. Both ports enumerate either way, so a silent port
looks connected.

Config uses the stable by-id path:

```
/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE8012534-if00
```

Serial mode needs no CAN transceiver. A VP230 (SN65HVD230) is wired for the CAN
path but is not used, and CAN mode would add polled request/reply traffic on the
same bus as the motors.

### Mounting and the axis permutation

The board is mounted **+X forward, +Y left, +Z up**. The policy's base frame is
**+X right, +Y forward, +Z up**. Exact 90-degree rotation, no interpolation error:

```
X_policy = -Y_sensor
Y_policy = +X_sensor
Z_policy = +Z_sensor
```

Expressed in the config as `(source_index, sign)` per policy axis:

```json
"axis_map": [[1, -1], [0, 1], [2, 1]]
```

Applied to **both** the gyro and the gravity vector. `projected_gravity` is
`-up_body` after permutation. Verified: upright, the runner reports
`g = (+0.00, -0.00, -1.00)`.

To re-derive the mounting after any change, tilt the pelvis and watch `up_body`:

- **nose-down** -> up shifts toward the sensor axis pointing **backward**
- **right-side-down** -> up shifts toward the sensor axis pointing **left**

### Gyro bias

Measured `(-1.33, +1.45, +2.91)` dps, stable to under 0.1 dps across two hours.
Subtracted as a constant in the **sensor** frame, before the permutation:

```json
"gyro_bias_dps": [-1.33, 1.45, 2.91]
```

This matters because `imu_read.py` never bias-corrects `gyro_dps` itself — only
the integrator's internal omega is corrected. Without the subtraction,
`obs[20:23]` carries a permanent 0.061 rad/s of phantom rotation.

### ZUPT threshold

`RK4DeadReckoner`'s stationary detector enters at
`zupt_gyro_dps * stationary_sensitivity_scale` = `2.0 * 1.5` = **3.0 dps**. The
measured bias magnitude is **3.51 dps**, so the detector never fires and the
estimator never learns the bias. The runner passes `zupt_gyro_dps=6.0`.

Effect is modest — with `kp_acc = 2.0`, the steady-state tilt error from the
roll/pitch bias is `0.0344 / 2.0` = about **1 degree**. Yaw drift is irrelevant
here: both observations this policy uses are yaw-invariant.

### Mount quality

Rigidly bolted, measured with the robot upright and still:

```
tilt from vertical  1.72 deg      (target < 2)
accel noise         0.002 g/axis
gyro noise          0.035-0.041 dps
|acc|               0.9813 g      (1.9% low, clears the 0.05 gate)
```

A mounting tilt is a **constant** error in `projected_gravity`, which the policy
reads as a permanently leaning torso and will try to correct. Keep it under 2
degrees. Re-measure with motors enabled and holding, not on a desk — motor
vibration through the frame is the real noise source.

---

## 5. CAN bus

### Bring-up

```bash
ip link show can0                       # want UP, LOWER_UP, state UP

sudo modprobe slcan
sudo slcand -o -c -s6 /dev/ttyACM1 can0
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

**Use `/dev/ttyACM1` explicitly.** `setup.sh` auto-detects, but errors out when
it finds more than one `/dev/ttyACM*` — and the IMU board is a second one.
Adapter is a CANable2 (`16d0:117e`) in slcan mode.

### Verify

```bash
./.venv/bin/python utils/validation_code/id_check.py \
    --channel can0 --bitrate 1000000 --start 1 --end 10
```

Want all ten IDs. If nothing replies, check `ip -s link show can0`: if **TX
increments and RX does not**, the laptop is transmitting and nothing is
answering — that is motor power or the CAN harness, not software.

### The batched-feedback fix

`RobotInterface.read_feedback` calls `receive_status_frame` once per motor, and
that method waits for one specific device ID while **discarding** frames from
every other motor. When replies arrive out of order the wanted frame is thrown
away by an earlier read, and that read burns its full 0.1 s timeout.

Measured at 50 Hz with ten motors: **3.8 Hz achieved**, with about a third of
steps taking ~0.75 s (7-8 timeouts each).

`RobotInterface.read_feedback_batched` drains all pending status frames in one
pass and dispatches each by device ID, with a single 15 ms budget for all ten.

```
before:  250 steps in 66.20s =  3.8 Hz, 83 overruns
after:   250 steps in  4.99s = 50.1 Hz,  0 overruns
         9.96 of 10 motors replied per cycle
```

It is a **new** method; `read_feedback` is untouched, so the other tools keep
their existing behaviour. Enabled by
`"use_batched_feedback": true`.

If the rate ever regresses, the next suspect is the adapter: slcan tunnels each
CAN frame as ASCII over USB serial. CANable2 also supports native gs_usb /
candleLight firmware, which is a much lower-latency transport.

---

## 6. Running it

### Modes

```bash
# no CAN at all. IMU and policy only. Nothing can move.
./.venv/bin/python ctrl_scripts/run_tracking_policy.py --offline --steps 120

# full CAN read/write at rate, command frozen at the startup pose.
# Validates timing and feedback without commanding motion. Motors go stiff.
./.venv/bin/python ctrl_scripts/run_tracking_policy.py --hold --steps 250

# live: 299 frames at 50 Hz, about 6 seconds, then it stops and disables.
./.venv/bin/python ctrl_scripts/run_tracking_policy.py
```

`--dry-run` also exists but **is close to useless for validation**: motors send
status frames only in reply to a command, and dry-run suppresses the command, so
every read times out, the loop runs at ~2 Hz, and the observations are built on
stale positions from `connect()`. Use `--hold` instead.

### Per-step log

Every run, in every mode, records each control step to
`logs/tracking/tracking_<mode>_<timestamp>.npz` (`log_dir` in the config,
`--log-dir` to override, `--no-log` to skip). The file is written from
`shutdown()`, so a Ctrl+C or safety trip still leaves the steps up to that point,
and `meta` records why the run ended.

Each step stores the whole chain from policy to motor:

```
ref_pos_sim, ref_vel_sim   reference from the ONNX, sim convention
obs, action                exactly what went into and came out of the network
target_raw                 sign * (default + action_scale * action)
target_clipped             after joint limits and clamps
commanded                  after the max_vel_rad_s rate limit
sent / final_cmd           what was written / after RobotInterface's own clamp
joint_pos, joint_vel       what the policy saw (joint_pos is clamped to limits)
joint_pos_unclamped        the real angle, so overshoot past a limit is visible
motor_pos/vel/torque/temp  raw motor space, before INVERSION_ARRAY
feedback_fresh             1 if that motor replied this step
ang_vel, proj_gravity      IMU, policy frame
t, loop_dt, time_step      timing
```

Summarize and plot it:

```bash
./.venv/bin/python utils/plot_tracking_log.py logs/tracking/tracking_live_<timestamp>.npz
```

The policy table shows action size, how often targets were clipped or
rate-limited, and the error against the reference. The motor table shows
tracking error (measured minus sent), torque, temperature and missed replies per
motor, and compares the two knees. Three PNGs are written next to the log:
position (reference, sent, measured), action with torque, and IMU with loop
timing. In `--offline` the joints never move, so the actions there are a
reaction to a frozen robot and not what a live run will produce.

### Torque guard

The runner keeps every motor inside the ratings in the RobStride manuals
(`TorqueGuard` in `run_tracking_policy.py`, settings under `torque_guard` in the
config). Everything is in motor space, so it works through the ankle linkage.

```
model   peak   rated at stall   command cap   stop if |tau| >=   stop if 2 s RMS >
RS-04   120    28.5 Nm          80 Nm         108 Nm x3 replies  28.5 Nm
RS-03    60    15   Nm          40 Nm          54 Nm x3 replies  15   Nm
RS-02    17     6   Nm          11 Nm          15 Nm x3 replies   6   Nm
```

- **Command cap.** Before each write, the target is pulled toward the current
  position so the spring term `kp * (target - pos)` stays under the cap. The
  damping term is left alone, so braking is never weakened.
- **Stops.** The run ends, holding and then disabling every motor, if a motor
  sits near peak torque, averages more than its stalled rating over 2 s, reaches
  85 C (warning at 70 C), reports an impossible temperature, or misses 5 replies
  in a row.
- The motors' own `TORQUE_LIMIT` is not touched: all three manuals say not to
  change it, so it stays as the last line of defence.

Replayed against the 2026-10-02 live run, the guard would have stopped it at
0.56 s on motor 5's missing replies, and would not have limited any hip, thigh
or knee command. The log summary prints each motor's peak as a % of peak torque,
its worst rolling average as a % of rated torque, and how often the guard
stepped in.

### Pre-flight

1. `can0` up, all ten motors answer `id_check.py`
2. Legs straight, mechanical zeros set (`./zero_out.sh`). **Zeros are lost on any
   power cycle**, and this policy's zero pose is the physical zero.
3. Monitor shows every joint near 0 degrees with the legs straight
4. `--offline` runs at ~50 Hz and reports `g = (0, 0, -1)` upright
5. `--hold` runs at ~50 Hz with ~10/10 motors replying
6. Robot **suspended**, legs hanging free, power kill in hand

### What live does

Walks from frame 0. There is no standing phase and no way to command a hold —
motion starts on the first step. `Ctrl+C` or a safety trip writes a hold at the
current position, waits 50 ms, then **disables all motors**, so the legs go
limp. Limp is not safe if the robot is bearing weight.

### Startup banner

A correct run prints:

```
[SIGN]     inverted vs sim, corrected here: ['left_hip2_joint', 'left_thigh_joint', 'right_thigh_joint']
[LIMITS]   left_hip2_joint flipped: hardware range (-0.4363, +1.5700)
[CLAMP]    left_hip2_joint  restricted to (-0.3862, +0.5000)
[CLAMP]    right_hip2_joint restricted to (-0.3862, +0.5000)
[SAFETY]   limit margin 0.050 rad (2.9 deg) applied to the safety checker only
[IMU]      streaming from /dev/serial/by-id/usb-1a86_...
```

If any of those are missing, the config is not being read.

### What to watch

- `|a|max` — 0.8 to 8.7 in hold testing. Sustained saturation is a problem.
- `g=(...,-1.00)` — third component near -1 while suspended
- `w` — near zero while still; nonzero means real rotation or bias drift
- `[SAFETY]` lines — the 90 degree jump check stays armed
- the closing `[TIMING]` line

---

## 7. Known issues and gotchas

**Temperature is garbage.** `Tmax=6534C`. The decode is
`float(temperature_u16) * 0.1` from a `>HHHH` unpack, and the raw field reads
`65340`. Cosmetic only — `_check_joint_state_safety` uses position limits and
jump size, never temperature. But you cannot use that number to watch motor heat.

**`MEASURED_POSITION` (0x3016) is dead** on these motors, returning exactly
`0.000000`. The live value is `MECHANICAL_POSITION` (0x7019), which is unwrapped
and can sit near 2*pi — wrap to +/-pi before use.

**`setup.sh` fails with two ACM devices.** The IMU board is a second
`/dev/ttyACM*`. Pass the adapter explicitly.

**The knee rests exactly on its limit.** Knee range is `(-2.0944, 0)` and the
policy's zero pose is legs-straight, so the joint sits at its upper bound and
sensor noise trips the safety check instantly. Handled by
`"safety_limit_margin_rad": 0.05`, applied only to the limits the safety checker
sees; the runner's own clipping stays tight (knee upper bound `-0.052 rad`, so
hyperextension is never commanded).

**`--dry-run` still energizes motors.** `RobotInterface.connect()` calls
`bus.enable()` regardless of `dry_run`; only target writes are suppressed.

**Mechanical zeros are lost on power cycle.** Straighten the legs and re-run
`./zero_out.sh`.

**`robot_hardware` gains, not the config's.** `apply_shared_hardware_config`
overwrites `kp_by_joint` with `POLICY_KP_BY_JOINT` — hips 200-300, knees 100,
ankles 120. The `"kp": 10` in the config file is dead. Sim trained against
`kp 100/100/100/80/20`, different units, not directly comparable.

**`max_vel_rad_s` is at 1.0**, down from 4.5, for early runs. It will lag the
reference. Raise it once direction and stability are confirmed.

---

## 8. Open items

- The `left_hip2` limit interval is derived from the sign flip, not measured.
  Bounded by the +/-0.5 clamp until someone verifies the real mechanical range.
- Accelerometer reads 0.9813 g, a stable 1.9% low. Clears the 0.05 gate. A
  six-position accel calibration would clean it up.
- `base_lin_vel` is absent from this policy's observation, which is why it works
  on hardware with no state estimator. Any policy that *does* use it will read a
  hard zero and should not be deployed here.
