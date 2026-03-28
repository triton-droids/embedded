# IK Runbook (Latest Runs, Full Motors)

This runbook captures the latest IK data-collection workflow from `latest_ik_runs.txt`.

## Scope
- Active motors: `1 2 3 4 5 6 7 8 9 10` (includes ankles `5` and `10`)
- Input trajectories from `../trajectory_generation/`
- Output logs saved under `./motor_dataset/...`

## 0) Run from the correct folder
```bash
cd /home/droids/Documents/embedded/data_collection
```

## 1) Build subset IK CSVs
```bash
python3 make_ik_csv_subset.py \
  --ids "1 2 3 4 5 6 7 8 9 10" \
  --input ../trajectory_generation/ik_standing_leg_swing_fast.csv \
  --out ./ik_per_motor/ik_subset_standing_fast_12345678910.csv \
  --repeat 8

python3 make_ik_csv_subset.py \
  --ids "1 2 3 4 5 6 7 8 9 10" \
  --input ../trajectory_generation/ik_standing_leg_swing_fast_short.csv \
  --out ./ik_per_motor/ik_subset_standing_fast_short_12345678910.csv \
  --repeat 12

python3 make_ik_csv_subset.py \
  --ids "1 2 3 4 5 6 7 8 9 10" \
  --input ../trajectory_generation/ik_stepping_y_big.csv \
  --out ./ik_per_motor/ik_subset_stepping_y_big_12345678910.csv \
  --repeat 8

python3 make_ik_csv_subset.py \
  --ids "1 2 3 4 5 6 7 8 9 10" \
  --input ../trajectory_generation/ik_standing_leg_swing_fast_tall.csv \
  --out ./ik_per_motor/ik_subset_standing_fast_tall_12345678910.csv \
  --repeat 18

python3 make_ik_csv_subset.py \
  --ids "1 2 3 4 5 6 7 8 9 10" \
  --input ../trajectory_generation/ik_standing_leg_swing_fast_tall_faster.csv \
  --out ./ik_per_motor/ik_subset_standing_fast_tall_faster_12345678910.csv \
  --repeat 129

python3 make_ik_csv_subset.py \
  --ids "1 2 3 4 5 6 7 8 9 10" \
  --input ../trajectory_generation/ik_standing_leg_swing_fast_tall_faster2.csv \
  --out ./ik_per_motor/ik_subset_standing_fast_tall_faster2_12345678910.csv \
  --repeat 8
```

## 2) Replay each IK subset and log to separate datasets
```bash
python3 collect_motor_dataset.py \
  --campaign ik \
  --motor-ids "1 2 3 4 5 6 7 8 9 10" \
  --multi-ids "1 2 3 4 5 6 7 8 9 10" \
  --ik-path ./ik_per_motor/ik_subset_standing_fast_12345678910.csv \
  --hz 120 \
  --campaign-dir ./motor_dataset/ik_standing_fast_12345678910 \
  --default-move-duration-s 4.0 \
  --default-move-hz 200 \
  --save-csv \
  --no-safety

python3 collect_motor_dataset.py \
  --campaign ik \
  --motor-ids "1 2 3 4 5 6 7 8 9 10" \
  --multi-ids "1 2 3 4 5 6 7 8 9 10" \
  --ik-path ./ik_per_motor/ik_subset_standing_fast_short_12345678910.csv \
  --hz 120 \
  --campaign-dir ./motor_dataset/ik_standing_fast_short_12345678910 \
  --default-move-duration-s 4.0 \
  --default-move-hz 200 \
  --save-csv \
  --no-safety

python3 collect_motor_dataset.py \
  --campaign ik \
  --motor-ids "1 2 3 4 5 6 7 8 9 10" \
  --multi-ids "1 2 3 4 5 6 7 8 9 10" \
  --ik-path ./ik_per_motor/ik_subset_stepping_y_big_12345678910.csv \
  --hz 120 \
  --campaign-dir ./motor_dataset/ik_stepping_y_big_12345678910 \
  --default-move-duration-s 4.0 \
  --default-move-hz 200 \
  --save-csv \
  --no-safety

python3 collect_motor_dataset.py \
  --campaign ik \
  --motor-ids "1 2 3 4 5 6 7 8 9 10" \
  --multi-ids "1 2 3 4 5 6 7 8 9 10" \
  --ik-path ./ik_per_motor/ik_subset_standing_fast_tall_12345678910.csv \
  --hz 120 \
  --campaign-dir ./motor_dataset/ik_standing_fast_tall_12345678910 \
  --default-move-duration-s 4.0 \
  --default-move-hz 200 \
  --save-csv \
  --no-safety

python3 collect_motor_dataset.py \
  --campaign ik \
  --motor-ids "1 2 3 4 5 6 7 8 9 10" \
  --multi-ids "1 2 3 4 5 6 7 8 9 10" \
  --ik-path ./ik_per_motor/ik_subset_standing_fast_tall_faster_12345678910.csv \
  --hz 120 \
  --campaign-dir ./motor_dataset/ik_standing_fast_tall_faster_12345678910 \
  --default-move-duration-s 4.0 \
  --default-move-hz 200 \
  --save-csv \
  --no-safety

python3 collect_motor_dataset.py \
  --campaign ik \
  --motor-ids "1 2 3 4 5 6 7 8 9 10" \
  --multi-ids "1 2 3 4 5 6 7 8 9 10" \
  --ik-path ./ik_per_motor/ik_subset_standing_fast_tall_faster2_12345678910.csv \
  --hz 120 \
  --campaign-dir ./motor_dataset/ik_standing_fast_tall_faster2_12345678910 \
  --default-move-duration-s 4.0 \
  --default-move-hz 200 \
  --save-csv \
  --no-safety
```

## 2b) Run commands for newly generated subset files
```bash
python3 collect_motor_dataset.py \
  --campaign ik \
  --motor-ids "1 2 3 4 5 6 7 8 9 10" \
  --multi-ids "1 2 3 4 5 6 7 8 9 10" \
  --ik-path ./ik_per_motor/ik_subset_standing_fast_tall_12345678910.csv \
  --hz 120 \
  --campaign-dir ./motor_dataset/ik_standing_fast_tall_12345678910 \
  --default-move-duration-s 4.0 \
  --default-move-hz 200 \
  --save-csv \
  --no-safety

python3 collect_motor_dataset.py \
  --campaign ik \
  --motor-ids "1 2 3 4 5 6 7 8 9 10" \
  --multi-ids "1 2 3 4 5 6 7 8 9 10" \
  --ik-path ./ik_per_motor/ik_subset_standing_fast_tall_faster_12345678910.csv \
  --hz 120 \
  --campaign-dir ./motor_dataset/ik_standing_fast_tall_faster_12345678910 \
  --default-move-duration-s 4.0 \
  --default-move-hz 200 \
  --save-csv \
  --no-safety

python3 collect_motor_dataset.py \
  --campaign ik \
  --motor-ids "1 2 3 4 5 6 7 8 9 10" \
  --multi-ids "1 2 3 4 5 6 7 8 9 10" \
  --ik-path ./ik_per_motor/ik_subset_standing_fast_tall_faster2_12345678910.csv \
  --hz 120 \
  --campaign-dir ./motor_dataset/ik_standing_fast_tall_faster2_12345678910 \
  --default-move-duration-s 4.0 \
  --default-move-hz 200 \
  --save-csv \
  --no-safety
```

## 3) Single-motor IK run example
```bash
python3 collect_motor_dataset.py \
  --campaign ik \
  --motor-ids "9" \
  --multi-ids "9" \
  --ik-path ./ik_per_motor/ik_stepping_motor9_left_knee_joint.csv \
  --hz 400 \
  --campaign-dir ./motor_dataset/ik_motor9_test \
  --default-move-duration-s 4.0 \
  --default-move-hz 200 \
  --save-csv \
  --no-safety
```

## Notes
- Keep `--ik-path` on one line with the filename. Splitting it across lines without `\` causes `command not found` errors.
- Use a unique `--campaign-dir` per run so logs stay separated.
