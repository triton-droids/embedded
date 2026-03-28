#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Build a subset IK CSV from a full trajectory CSV.

Default behavior:
- Input:  ../trajectory_generation/ik_stepping_trajectory.csv
- Output: ./ik_per_motor/ik_stepping_motors_1_2_3_4_6_7_8_9.csv
- IDs:    1 2 3 4 6 7 8 9 (i.e., excludes 5 and 10)
- Repeat: 1x

Output columns:
  t + joint columns in the same order as --ids
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


MOTOR_TO_JOINT = {
    1: "left_hip1_joint",
    2: "left_hip2_joint",
    3: "left_thigh_joint",
    4: "left_knee_joint",
    5: "left_ankle_joint",
    6: "right_hip1_joint",
    7: "right_hip2_joint",
    8: "right_thigh_joint",
    9: "right_knee_joint",
    10: "right_ankle_joint",
}


def parse_ids(text: str) -> list[int]:
    ids = [int(x) for x in text.split() if x.strip()]
    if not ids:
        raise ValueError("IDs list is empty.")
    if len(set(ids)) != len(ids):
        raise ValueError("IDs list contains duplicates.")
    bad = [i for i in ids if i not in MOTOR_TO_JOINT]
    if bad:
        raise ValueError(f"Unsupported motor IDs: {bad}. Valid range is 1..10.")
    return ids


def make_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Create IK CSV subset with selected motor joint columns.")
    p.add_argument(
        "--input",
        default="../trajectory_generation/ik_stepping_trajectory.csv",
        help="Path to full IK trajectory CSV.",
    )
    p.add_argument(
        "--out",
        default="./ik_per_motor/ik_stepping_motors_1_2_3_4_6_7_8_9.csv",
        help="Output CSV path.",
    )
    p.add_argument(
        "--ids",
        default="1 2 3 4 6 7 8 9",
        help='Space-separated motor IDs, e.g. "1 2 3 4 6 7 8 9".',
    )
    p.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="How many times to tile the trajectory in one output CSV (continuous time).",
    )
    return p


def main() -> None:
    args = make_argparser().parse_args()
    if args.repeat < 1:
        raise SystemExit("--repeat must be >= 1")
    ids = parse_ids(args.ids)
    joints = [MOTOR_TO_JOINT[mid] for mid in ids]

    in_path = Path(args.input).expanduser().resolve()
    out_path = Path(args.out).expanduser().resolve()

    if not in_path.exists():
        raise SystemExit(f"Input CSV not found: {in_path}")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_cols = ["t"] + joints

    with in_path.open("r", newline="") as f_in:
        reader = csv.DictReader(f_in)
        fieldnames = set(reader.fieldnames or [])
        missing = [c for c in out_cols if c not in fieldnames]
        if missing:
            raise SystemExit(f"Input CSV missing required columns: {missing}")
        rows = list(reader)

    if len(rows) < 2:
        raise SystemExit("Input CSV must contain at least 2 data rows.")

    try:
        t_vals = [float(r["t"]) for r in rows]
    except Exception as exc:
        raise SystemExit(f"Failed to parse 't' column as float: {exc}")

    dt = t_vals[1] - t_vals[0]
    if dt <= 0.0:
        raise SystemExit("Input CSV time column must be strictly increasing.")
    for i in range(2, len(t_vals)):
        if t_vals[i] <= t_vals[i - 1]:
            raise SystemExit("Input CSV time column must be strictly increasing.")

    duration = t_vals[-1] - t_vals[0]
    block_stride = duration + dt

    with out_path.open("w", newline="") as f_out:
        writer = csv.DictWriter(f_out, fieldnames=out_cols)
        writer.writeheader()
        for rep in range(args.repeat):
            t_offset = rep * block_stride
            for i, row in enumerate(rows):
                out_row = {c: row[c] for c in out_cols}
                out_row["t"] = f"{(t_vals[i] - t_vals[0] + t_offset):.9f}"
                writer.writerow(out_row)

    ids_text = " ".join(str(i) for i in ids)
    print(f"Wrote: {out_path}")
    print(f"IDs order: {ids_text}")
    print(f"Repeat factor: {args.repeat}x")
    print(
        "Run with:\n"
        "python3 collect_motor_dataset.py "
        f'--campaign ik --motor-ids "{ids_text}" --multi-ids "{ids_text}" '
        f'--ik-path "{out_path}" --hz 400 '
        "--campaign-dir ./motor_dataset/ik_subset_test --default-move-duration-s 4.0 "
        "--default-move-hz 200 --save-csv --no-safety"
    )


if __name__ == "__main__":
    main()
