#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Convert a full IK CSV (with many columns) into per-motor IK CSV files.

Output format per file:
  t,<joint_column_for_motor>

This matches collect_motor_dataset.py --campaign ik when using --multi-ids "<single_id>".
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


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Create per-motor IK CSV files from a full IK CSV.")
    p.add_argument(
        "--input",
        required=True,
        help="Path to full IK CSV (e.g. ../trajectory_generation/ik_stepping_trajectory.csv).",
    )
    p.add_argument(
        "--out-dir",
        default="./ik_per_motor",
        help="Directory for generated per-motor CSV files.",
    )
    p.add_argument(
        "--prefix",
        default="ik",
        help="Output filename prefix. Final name: <prefix>_motor<ID>_<joint>.csv",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    in_path = Path(args.input).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()

    if not in_path.exists():
        raise SystemExit(f"Input CSV not found: {in_path}")

    out_dir.mkdir(parents=True, exist_ok=True)

    with in_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = set(reader.fieldnames or [])
        if "t" not in fieldnames:
            raise SystemExit("Input CSV must contain 't' column.")

        for mid, joint_col in MOTOR_TO_JOINT.items():
            if joint_col not in fieldnames:
                raise SystemExit(f"Input CSV missing required joint column: {joint_col}")

        rows = list(reader)

    for mid, joint_col in MOTOR_TO_JOINT.items():
        out_path = out_dir / f"{args.prefix}_motor{mid}_{joint_col}.csv"
        with out_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["t", joint_col])
            writer.writeheader()
            for row in rows:
                writer.writerow({"t": row["t"], joint_col: row[joint_col]})
        print(f"Wrote {out_path}")

    print("\nRun example for motor 4:")
    print(
        "python3 collect_motor_dataset.py "
        "--campaign ik --motor-ids \"1 2 3 4 5 6 7 8 9 10\" --multi-ids \"4\" "
        f"--ik-path \"{out_dir / (args.prefix + '_motor4_left_knee_joint.csv')}\" "
        "--hz 400 --campaign-dir ./motor_dataset/ik_motor4_test --save-csv"
    )


if __name__ == "__main__":
    main()