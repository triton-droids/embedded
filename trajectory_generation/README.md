# Trajectory Generation (IK Stepping)

This folder contains tools to:
- Generate IK-based stepping joint trajectories from the URDF.
- Replay and evaluate those trajectories in MuJoCo.

## Files
- `generate_ik_stepping_trajectory.py`: builds the stepping trajectory CSV.
- `evaluate_ik_stepping_trajectory_mujoco.py`: replays trajectory in `scene.xml` and reports tracking metrics.
- `human_offset_corrected.urdf`: URDF used by the generator.
- `scene.xml`: MuJoCo model used by the evaluator.

## Requirements
- Python with `numpy` installed.
- MuJoCo Python package for evaluation:
  - `pip install mujoco`

## 1) Foot Tracking Point Convention

Use the bottom-center point of each foot collision box as the IK tracking point.
Even if a debug label says `dbg_toe_bottom`, these values are bottom-center points (not toe points), which is typically more stable for IK contact behavior.

Recommended offsets:
- Left: `(-0.01, 0.025, -0.04263)`
- Right: `(0.01, 0.025, -0.04263)`

## 2) Generate a contact-mode trajectory (recommended)

Run from this folder:

```bash
cd trajectory_generation

python generate_ik_stepping_trajectory.py \
  --urdf human_offset_corrected.urdf \
  --mode contact \
  --hz 400 \
  --duration 20 \
  --step_freq 1.0 \
  --forward_axis y \
  --ax 0.12 \
  --az 0.04 \
  --left_foot_offset -0.01 0.025 -0.04263 \
  --right_foot_offset 0.01 0.025 -0.04263 \
  --out_csv ik_stepping_y_big.csv \
  --out_meta ik_stepping_y_big.json
```

also this version gives a standing one:

```bash
cd /Users/darin/Desktop/_/club_stuff/tritondroids/embedded/trajectory_generation
python generate_ik_stepping_trajectory.py \         
    --urdf human_offset_corrected.urdf \
    --mode contact \                         
    --hz 400 \                                      
    --duration 20 \           
    --step_freq 1.2 \
    --forward_axis y \
    --ax 0.0 \
    --az 0.07 \
    --gamma 1.8 \
    --ramp_time 1.0 \
    --left_foot_offset -0.01 0.025 -0.04263 \
    --right_foot_offset 0.01 0.025 -0.04263 \
    --out_csv ik_standing_leg_swing_fast.csv \
    --out_meta ik_standing_leg_swing_fast_meta.json
```

```bash
cd /Users/darin/Desktop/_/club_stuff/tritondroids/embedded/trajectory_generation
python generate_ik_stepping_trajectory.py --urdf human_offset_corrected.urdf --mode contact --hz 400 --duration 20 \
  --step_freq 1.5 \
  --forward_axis y \
  --ax 0.0 \
  --az 0.05 \
  --gamma 1.8 \
  --ramp_time 1.0 \
  --left_foot_offset -0.01 0.025 -0.04263 \
  --right_foot_offset 0.01 0.025 -0.04263 \
  --out_csv ik_standing_leg_swing_fast_short.csv \
  --out_meta ik_standing_leg_swing_fast_short_meta.json
```

Expected terminal output:
- `[OK] Wrote trajectory CSV: ik_stepping_trajectory.csv`
- `[OK] Wrote metadata JSON: ik_stepping_meta.json`
- Neutral foot positions printout

Important shell note:
- Do not put a trailing space after a line-continuation slash (`\`), or the command will break.

## 3) Optional: Make These Offsets Script Defaults

So you do not need to pass offset flags every run, update `generate_ik_stepping_trajectory.py`:

```python
ap.add_argument("--left_foot_offset", type=float, nargs=3,
                default=[-0.01, 0.025, -0.04263])
ap.add_argument("--right_foot_offset", type=float, nargs=3,
                default=[0.01, 0.025, -0.04263])
```

## 4) Optional: Print Active Offsets at Startup

Right after `args = ap.parse_args()`:

```python
print("[INFO] Using foot offsets:")
print("  left :", args.left_foot_offset)
print("  right:", args.right_foot_offset)
```

## 5) Generate an air-mode trajectory (optional)

```bash
python generate_ik_stepping_trajectory.py \
  --urdf human_offset_corrected.urdf \
  --mode air \
  --air_clearance 0.03 \
  --hz 400 \
  --duration 30 \
  --step_freq 0.6 \
  --ax 0.02 \
  --az 0.015 \
  --out_csv traj_air.csv \
  --out_meta traj_air_meta.json
```

## 6) Key generator arguments
- `--urdf`: input robot URDF.
- `--mode`: `contact` or `air`.
- `--hz`: trajectory sample rate.
- `--duration`: trajectory length in seconds.
- `--step_freq`: stepping frequency in Hz.
- `--ax`: foot fore-aft amplitude (m).
- `--az`: foot lift amplitude (m).
- `--gamma`: lift shape exponent (default `1.8`).
- `--ramp_time`: smooth ramp-in/ramp-out duration (s).
- `--left_foot_offset`: left foot target-point offset in foot frame (x y z), default `[-0.01, 0.025, -0.04263]`.
- `--right_foot_offset`: right foot target-point offset in foot frame (x y z), default `[0.01, 0.025, -0.04263]`.
- `--out_csv`: output trajectory CSV path.
- `--out_meta`: output metadata JSON path.

## 7) Visualize trajectory in MuJoCo (no logging)

If you only want to watch the motion in MuJoCo and do not want any evaluation files:

```bash
mjpython evaluate_ik_stepping_trajectory_mujoco.py \
  --model scene.xml \
  --traj_csv ik_standing_leg_swing_fast_short.csv \
  --traj_meta ik_standing_leg_swing_fast_short_meta.json \
  --render --realtime --no_log
```

This opens the MuJoCo viewer, replays the trajectory, and exits without writing CSV/JSON outputs.

## 8) Evaluate trajectory in MuJoCo (with logs)

```bash
mjpython evaluate_ik_stepping_trajectory_mujoco.py \
  --model scene.xml \
  --traj_csv ik_standing_leg_swing.csv \
  --traj_meta ik_standing_leg_swing.json \
  --out_csv traj_contact_eval.csv \
  --out_json traj_contact_eval_summary.json
```

Evaluation outputs:
- `traj_contact_eval.csv`: per-sample target/actual/error values and contact flags.
- `traj_contact_eval_summary.json`: aggregate RMSE/max error, IMU height stats, contact ratios, and fall flag.

Optional viewer while still logging:

```bash
python evaluate_ik_stepping_trajectory_mujoco.py \
  --model scene.xml \
  --traj_csv traj_contact.csv \
  --traj_meta traj_contact_meta.json \
  --render --realtime
```
