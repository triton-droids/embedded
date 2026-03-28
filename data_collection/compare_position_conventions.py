#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Compare the same motor readings under the dataset-controller and gain-tuner conventions.

This uses the same RobstrideController bring-up path as data collection so the
motors are enabled and read the same way as the working scripts.
"""

from __future__ import annotations

import argparse
import math
from typing import Dict, List

from collect_motor_dataset import (
    DEFAULT_JOINT_POS_BY_ID,
    INVERSION_BY_ID as DATASET_INV_BY_ID,
    JOINT_LIMITS,
    RobstrideController,
    clamp,
    scan_motor_ids_or_parse,
)


# Keep this aligned with utils/gain_tuner.py.
GAIN_TUNER_INVERSION_ARRAY = [-1, -1, -1, 1, 1, 1, -1, 1, -1, -1]
GAIN_INV_BY_ID: Dict[int, int] = {
    i + 1: GAIN_TUNER_INVERSION_ARRAY[i] for i in range(len(GAIN_TUNER_INVERSION_ARRAY))
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Compare dataset and gain-tuner position conventions.")
    p.add_argument("--motor-ids", default="", help="Space-separated motor IDs. Empty means scan bus.")
    p.add_argument("--channel", default="can0", help="CAN interface name.")
    p.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bitrate.")
    p.add_argument("--hz", type=float, default=120.0, help="Controller/read rate.")
    p.add_argument("--kp", type=float, default=10.0, help="Fallback kp for unsupported IDs.")
    p.add_argument("--kd", type=float, default=0.2, help="Fallback kd for unsupported IDs.")
    p.add_argument(
        "--safety-read-hz",
        type=float,
        default=120.0,
        help="Joint safety monitor read rate.",
    )
    p.add_argument(
        "--safety-max-jump-deg",
        type=float,
        default=90.0,
        help="Joint safety monitor max jump threshold.",
    )
    p.add_argument(
        "--no-safety",
        action="store_true",
        help="Disable joint-position safety monitor.",
    )
    return p.parse_args()


def mean(vals: List[float]) -> float:
    return sum(vals) / max(1, len(vals))


def main() -> int:
    args = parse_args()
    motor_ids = scan_motor_ids_or_parse(args.motor_ids, args.channel)

    ctrl = RobstrideController(
        motor_ids=motor_ids,
        channel=args.channel,
        bitrate=args.bitrate,
        kp=args.kp,
        kd=args.kd,
        control_hz=args.hz,
        safety_read_hz=args.safety_read_hz,
        safety_max_jump_deg=args.safety_max_jump_deg,
        safety_enabled=(not args.no_safety),
    )

    try:
        ctrl.connect()
        print(f"Connected on {args.channel}. Comparing motors: {sorted(motor_ids)}")
        print("-" * 166)
        print(
            f"{'ID':<4} {'Raw(rad)':<10} {'Dataset(rad)':<12} {'Gain(rad)':<10} "
            f"{'Dflt(rad)':<10} {'ErrDs(rad)':<11} {'ErrGt(rad)':<11} "
            f"{'DsDir':<6} {'GtDir':<6} {'Vel':<9} {'Tq':<9} {'Temp':<8}"
        )
        print("-" * 166)

        for mid in sorted(motor_ids):
            pos_vals: List[float] = []
            vel_vals: List[float] = []
            tq_vals: List[float] = []
            temp_vals: List[float] = []

            for _ in range(3):
                try:
                    pos, vel, tq, temp = ctrl.read(mid)
                    pos_vals.append(float(pos))
                    vel_vals.append(float(vel))
                    tq_vals.append(float(tq))
                    temp_vals.append(float(temp))
                except Exception:
                    break

            st = ctrl.states[mid]
            if not pos_vals:
                pos_vals.append(float(st.pos))
                vel_vals.append(float(st.vel))
                tq_vals.append(float(st.tq))
                temp_vals.append(float(st.temp))

            raw_pos = mean(pos_vals)
            vel = mean(vel_vals)
            tq = mean(tq_vals)
            temp = mean(temp_vals)

            ds_dir = int(DATASET_INV_BY_ID.get(mid, 1))
            gt_dir = int(GAIN_INV_BY_ID.get(mid, 1))
            ds_pos = raw_pos / float(ds_dir)
            gt_pos = raw_pos / float(gt_dir)

            lim_lo, lim_hi = JOINT_LIMITS.get(mid, (-math.inf, math.inf))
            default_target = DEFAULT_JOINT_POS_BY_ID.get(mid, math.nan)
            if math.isfinite(default_target):
                default_target = clamp(default_target, lim_lo, lim_hi)
                ds_err = ds_pos - default_target
                gt_err = gt_pos - default_target
                default_str = f"{default_target:<10.4f}"
                ds_err_str = f"{ds_err:<11.4f}"
                gt_err_str = f"{gt_err:<11.4f}"
            else:
                default_str = f"{'n/a':<10}"
                ds_err_str = f"{'n/a':<11}"
                gt_err_str = f"{'n/a':<11}"

            print(
                f"{mid:<4} {raw_pos:<10.4f} {ds_pos:<12.4f} {gt_pos:<10.4f} "
                f"{default_str} {ds_err_str} {gt_err_str} "
                f"{ds_dir:<6d} {gt_dir:<6d} {vel:<9.4f} {tq:<9.4f} {temp:<8.1f}"
            )

        print("-" * 166)
        print("`Dataset(rad)` uses data_collection/collect_motor_dataset.py inversion.")
        print("`Gain(rad)` uses utils/gain_tuner.py inversion.")
        print("`ErrDs(rad)` and `ErrGt(rad)` are versus the dataset default target.")
        return 0
    finally:
        try:
            ctrl.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
