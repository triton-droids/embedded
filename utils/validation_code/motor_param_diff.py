#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Compare every readable parameter between two or more motors. READ ONLY: never
enables torque, never writes a parameter, never sets zero.

Built to answer "is this motor actually deteriorating, or is it just configured
differently?". Two motors of the same model driving mirrored joints should agree
on every CONFIG value; anything that differs is a configuration difference, not
mechanical wear.

STATE values (position, current, bus voltage) are expected to differ and are
reported separately for information.

Usage:
    ./.venv/bin/python utils/validation_code/motor_param_diff.py            # 4 vs 9
    ./.venv/bin/python utils/validation_code/motor_param_diff.py --motors 5,10
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import numpy as np

from robot_hardware import BITRATE, CAN_CHANNEL, JOINT_NAME_BY_ID, MOTOR_MODEL_BY_ID
from robstride_dynamics import Motor, ParameterType, RobstrideBus

# Values that should be IDENTICAL across two same-model motors. A difference
# here is a configuration problem and explains a difference in feel.
CONFIG = [
    "MODE", "TORQUE_LIMIT", "VELOCITY_LIMIT", "CURRENT_LIMIT",
    "CURRENT_KP", "CURRENT_KI", "CURRENT_FILTER_GAIN",
    "POSITION_KP", "VELOCITY_KP", "VELOCITY_KI", "VELOCITY_FILTER_GAIN",
    "VEL_ACCELERATION_TARGET", "PP_VELOCITY_MAX", "PP_ACCELERATION_TARGET",
    "EPSCAN_TIME", "CAN_TIMEOUT", "ZERO_STATE", "MECHANICAL_OFFSET",
]

# Live values. Expected to differ; shown for information.
STATE = [
    "MECHANICAL_POSITION", "MECHANICAL_VELOCITY", "MEASURED_VELOCITY",
    "MEASURED_TORQUE", "IQ_FILTERED", "IQ_TARGET", "VBUS",
    "POSITION_TARGET", "VELOCITY_TARGET",
]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--motors", default="4,9", help="Comma-separated motor IDs to compare.")
    p.add_argument("--channel", default=CAN_CHANNEL)
    p.add_argument("--bitrate", type=int, default=BITRATE)
    p.add_argument("--all", action="store_true",
                   help="Also print parameters that match, not just differences.")
    return p.parse_args()


def fmt(v) -> str:
    if v is None:
        return "--"
    if isinstance(v, str):
        return v
    try:
        f = float(v)
    except (TypeError, ValueError):
        return str(v)
    return f"{f:.6g}"


def main() -> None:
    args = parse_args()
    ids = [int(x) for x in args.motors.split(",") if x.strip()]
    if len(ids) < 2:
        raise SystemExit("need at least two motor IDs to compare")

    motors = {f"motor_{i}": Motor(id=i, model=MOTOR_MODEL_BY_ID.get(i, "rs-03")) for i in ids}
    calibration = {n: {"direction": 1, "homing_offset": 0.0} for n in motors}

    bus = RobstrideBus(args.channel, motors, calibration, bitrate=args.bitrate)
    bus.connect(handshake=True)          # opens socketcan only; no torque
    print("READ ONLY: torque is never enabled and no parameter is written.\n")

    models = {i: MOTOR_MODEL_BY_ID.get(i, "?") for i in ids}
    print("comparing: " + " vs ".join(
        f"motor {i} ({JOINT_NAME_BY_ID.get(i,'?')}, {models[i]})" for i in ids))
    distinct_models = set(models.values())
    if len(distinct_models) > 1:
        print(f"[!] different models {sorted(distinct_models)} — config differences may be legitimate")
    print()

    def read(mid: int, name: str):
        try:
            pt = getattr(ParameterType, name)
        except AttributeError:
            return None
        try:
            v = bus.read(f"motor_{mid}", pt)
            arr = np.asarray(v).reshape(-1)
            return arr[0] if arr.size else None
        except Exception:
            return None

    def section(title: str, names: list[str], flag_diffs: bool) -> list[str]:
        width = max(len(n) for n in names) + 2
        header = f"{'parameter':<{width}}" + "".join(f"{('motor ' + str(i)):>16}" for i in ids)
        print(title)
        print("-" * len(header))
        print(header)
        differing = []
        for name in names:
            vals = [read(i, name) for i in ids]
            if all(v is None for v in vals):
                continue
            nums = [None if v is None else float(v) for v in vals]
            same = all(
                (a is None and b is None) or
                (a is not None and b is not None and abs(a - b) <= 1e-6 * max(1.0, abs(a), abs(b)))
                for a, b in zip(nums, nums[1:])
            )
            if same and not args.all:
                continue
            mark = "  <== DIFFERS" if (not same and flag_diffs) else ""
            print(f"{name:<{width}}" + "".join(f"{fmt(v):>16}" for v in vals) + mark)
            if not same:
                differing.append(name)
        print()
        return differing

    try:
        diffs = section("CONFIG  (should be identical between same-model motors)", CONFIG, True)
        section("STATE   (expected to differ; informational)", STATE, False)
    finally:
        bus.disconnect(disable_torque=False)   # stay strictly read-only

    # Only some CONFIG parameters can affect how a joint feels when back-driven
    # by hand. Profile parameters shape commanded trajectories in velocity and
    # position-profile modes and do nothing while the motor is disabled, so a
    # difference there does NOT explain a difference in feel.
    PROFILE_ONLY = {
        "VEL_ACCELERATION_TARGET", "PP_VELOCITY_MAX", "PP_ACCELERATION_TARGET",
        "EPSCAN_TIME", "CAN_TIMEOUT",
    }
    print("=" * 66)
    if diffs:
        relevant = [d for d in diffs if d not in PROFILE_ONLY]
        print(f"  {len(diffs)} CONFIG parameter(s) differ: {', '.join(diffs)}")
        if relevant:
            print(f"  Of these, {', '.join(relevant)} can affect how the joint")
            print("  behaves. Investigate before suspecting mechanical wear.")
        else:
            print("  All of these are motion-profile parameters. They shape commanded")
            print("  trajectories in velocity / position-profile mode and do nothing")
            print("  while the motor is disabled, so they do NOT explain a difference")
            print("  in how the joint feels by hand. They do suggest the motors were")
            print("  configured or flashed separately.")
            print("  Everything that could affect back-drive resistance matches, so")
            print("  look mechanically: the joint, its bearings, or the load below it.")
    else:
        print("  No CONFIG differences. The motors are configured identically,")
        print("  so a difference in how they feel is mechanical or in the load")
        print("  downstream of the joint (linkage, bearings, pose, mass).")
    print("=" * 66)


if __name__ == "__main__":
    main()
