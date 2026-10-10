#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Passive joint-direction monitor. READ ONLY: never enables torque, never writes a
target, never sets zero. Safe to run with the robot unpowered-but-on-bus.

Use it to confirm that the hardware's positive joint direction matches the sim's.
Back-drive one joint BY HAND in the direction the sim calls positive and watch
whether the reported angle rises. If it falls, that joint is inverted relative to
the policy's convention.

Expected motion for +angle, derived from chrobot_16kg_candidate.xml in the
base frame (+X right, +Y forward, +Z up):

    hip1   +  leg swings FORWARD
    hip2   +  leg swings toward the robot's LEFT
    thigh  +  left: foot yaws to robot's RIGHT / right: foot yaws to robot's LEFT
    knee   -  shank swings BACKWARD   (range is -2.0944..0, so only negative)
    ankle  +  TOES UP

Usage:
    ./.venv/bin/python utils/validation_code/joint_direction_monitor.py --motors 4
    ./.venv/bin/python utils/validation_code/joint_direction_monitor.py --motors 1,3,5
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import numpy as np

from robot_hardware import (
    BITRATE,
    CAN_CHANNEL,
    INVERSION_BY_ID,
    JOINT_NAME_BY_ID,
    MOTOR_MODEL_BY_ID,
)
from robstride_dynamics import Motor, ParameterType, RobstrideBus

RAD2DEG = 180.0 / np.pi


# Expected sign of the delta when the joint is pushed as instructed, derived
# from chrobot_16kg_candidate.xml in the base frame (+X right, +Y fwd, +Z up).
# NOTE ON NAMING: joints prefixed "left_" are physically on the robot's RIGHT
# side, and vice versa. The sim XML puts the "left_" bodies at +X, and the
# reference motion establishes +X = right. Hardware uses the same mirrored
# names, so the mapping is consistent and no remap is needed -- but the labels
# are misleading, so every instruction below is phrased side-agnostically.
#
# Motors 1-5 drive the robot's PHYSICAL RIGHT leg.
# Motors 6-10 drive the robot's PHYSICAL LEFT leg.
EXPECTED = {
    1:  (+1, "swing THIS leg FORWARD, out in front of the robot"),
    2:  (+1, "swing THIS leg INWARD, toward the other leg"),
    3:  (+1, "rotate THIS foot TOE-OUT, away from the other foot"),
    4:  (-1, "bend THIS knee, heel toward the buttock"),
    5:  (+1, "lift THIS foot's TOES UP toward the shin"),
    6:  (+1, "swing THIS leg FORWARD, out in front of the robot"),
    7:  (+1, "swing THIS leg OUTWARD, away from the other leg"),
    8:  (+1, "rotate THIS foot TOE-OUT, away from the other foot"),
    9:  (-1, "bend THIS knee, heel toward the buttock"),
    10: (+1, "lift THIS foot's TOES UP toward the shin"),
}
CROSS_CHECK = {2, 4, 7, 9}   # already confirmed by the asymmetric joint limits


def run_single(bus, read_joint, wrap, mid: int, seconds: float, hz: float) -> str:
    """Guided single-joint test that prints its own verdict. Returns the verdict."""
    sign, instruction = EXPECTED[mid]
    name = JOINT_NAME_BY_ID.get(mid, f"motor_{mid}")
    tag = "  [CROSS-CHECK: answer already known]" if mid in CROSS_CHECK else ""
    print("=" * 66)
    print(f"  motor {mid}  {name}{tag}")
    print(f"  Hold still. Baseline in 3 s...")
    print("=" * 66)
    time.sleep(3.0)

    base = read_joint(mid)
    if base is None:
        print(f"[!] no reply from motor {mid}")
        return "NO REPLY"
    print(f"  baseline {base*RAD2DEG:+.2f} deg captured\n")
    print(f"  NOW: {instruction}")
    print(f"  Move at least 10 deg, slowly, then hold.\n")

    peak = 0.0
    period = 1.0 / max(0.5, hz)
    t_end = time.time() + seconds
    try:
        while time.time() < t_end:
            v = read_joint(mid)
            if v is not None:
                d = wrap(v - base) * RAD2DEG
                if abs(d) > abs(peak):
                    peak = d
                bar = "#" * min(40, int(abs(d)))
                print(f"\r  delta {d:+7.2f} deg   peak {peak:+7.2f}   {bar:<40}", end="", flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        pass

    print("\n")
    if abs(peak) < 8.0:
        print(f"  INCONCLUSIVE: peak was only {peak:+.2f} deg. Move it at least 10 deg and retry.")
        return f"INCONCLUSIVE ({peak:+.1f})"
    ok = (peak > 0) == (sign > 0)
    print(f"  peak deflection {peak:+.2f} deg, expected sign {'+' if sign > 0 else '-'}")
    if ok:
        print(f"  ==> MATCHES the sim convention. motor {mid} is CORRECT.")
    else:
        print(f"  ==> OPPOSITE of the sim convention. motor {mid} is INVERTED.")
        if mid in CROSS_CHECK:
            print("      This is a cross-check joint whose answer was already known from")
            print("      the joint limits. A failure here means the test method or the")
            print("      frame derivation is wrong -- STOP and re-examine before trusting")
            print("      any of the other results.")
    return (f"CORRECT ({peak:+.1f})" if ok else f"INVERTED ({peak:+.1f})")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--motors", default="1,2,3,4,5,6,7,8,9,10",
                   help="Comma-separated motor IDs to watch (free-run table mode).")
    p.add_argument("--joint", type=int, default=None,
                   help="Guided single-joint direction test; prints its own verdict.")
    p.add_argument("--sequence", default=None,
                   help="Guided sweep: comma-separated motor IDs, or 'all'. "
                        "Pauses between joints and prints a summary table.")
    p.add_argument("--channel", default=CAN_CHANNEL)
    p.add_argument("--bitrate", type=int, default=BITRATE)
    p.add_argument("--seconds", type=float, default=60.0)
    p.add_argument("--hz", type=float, default=4.0)
    return p.parse_args()


def main() -> None:
    args = parse_args()
    ids = [int(x) for x in args.motors.split(",") if x.strip()]

    motors = {f"motor_{i}": Motor(id=i, model=MOTOR_MODEL_BY_ID.get(i, "rs-03")) for i in ids}
    calibration = {name: {"direction": 1, "homing_offset": 0.0} for name in motors}

    bus = RobstrideBus(args.channel, motors, calibration, bitrate=args.bitrate)
    bus.connect(handshake=True)          # opens socketcan only; no torque
    print("READ ONLY: torque is never enabled and no targets are written.\n")

    def wrap_to_pi(x: float) -> float:
        return (x + np.pi) % (2.0 * np.pi) - np.pi

    def read_joint(mid: int) -> float | None:
        """Motor angle (rad), wrapped to (-pi, pi], with hardware inversion applied.

        MEASURED_POSITION (0x3016) reads a constant 0 on these motors; the live
        value is MECHANICAL_POSITION (0x7019), which is unwrapped and can sit
        near 2*pi, hence the wrap.
        """
        try:
            v = bus.read(f"motor_{mid}", ParameterType.MECHANICAL_POSITION)
            raw = float(np.asarray(v).reshape(-1)[0])
            return wrap_to_pi(raw) * float(INVERSION_BY_ID.get(mid, 1))
        except Exception:
            return None

    if args.sequence is not None:
        order = (list(range(1, 11)) if args.sequence.strip().lower() == "all"
                 else [int(x) for x in args.sequence.split(",") if x.strip()])
        results: dict[int, str] = {}
        try:
            for n, mid in enumerate(order, 1):
                if mid not in EXPECTED:
                    print(f"[!] skipping {mid}: not a known motor id")
                    continue
                print(f"\n[{n}/{len(order)}]")
                results[mid] = run_single(bus, read_joint, wrap_to_pi, mid, args.seconds, args.hz)
                if n < len(order):
                    try:
                        input("\n  Press Enter for the next joint (Ctrl+C to stop)... ")
                    except (EOFError, KeyboardInterrupt):
                        break
        finally:
            bus.disconnect(disable_torque=False)
        print("\n" + "=" * 66)
        print("  SUMMARY")
        print("=" * 66)
        for mid in order:
            if mid in results:
                flag = "  [cross-check]" if mid in CROSS_CHECK else ""
                print(f"  motor {mid:>2}  {JOINT_NAME_BY_ID.get(mid,'?'):<20} {results[mid]}{flag}")
        bad = [m for m, v in results.items() if v.startswith("INVERTED")]
        if bad:
            print(f"\n  INVERTED: {bad}  -- send this list to fold into the runner.")
        else:
            print("\n  No inversions found.")
        return

    if args.joint is not None:
        if args.joint not in EXPECTED:
            raise SystemExit(f"--joint must be 1..10, got {args.joint}")
        try:
            run_single(bus, read_joint, wrap_to_pi, args.joint, args.seconds, args.hz)
        finally:
            bus.disconnect(disable_torque=False)
        return

    baseline: dict[int, float] = {}
    for mid in ids:
        v = read_joint(mid)
        if v is not None:
            baseline[mid] = v
    missing = [m for m in ids if m not in baseline]
    if missing:
        print(f"[!] no reply from motor(s): {missing}")
    print(f"baseline captured for {sorted(baseline)}\n")
    print("Move a joint by hand. 'delta' is the change since baseline, in degrees.")
    print("A POSITIVE delta means you moved it in the hardware's positive direction.\n")

    hdr = f"{'id':>3} {'joint':<20}{'angle deg':>11}{'delta deg':>11}"
    period = 1.0 / max(0.5, args.hz)
    t_end = time.time() + args.seconds
    try:
        while time.time() < t_end:
            rows = []
            for mid in ids:
                v = read_joint(mid)
                if v is None or mid not in baseline:
                    rows.append(f"{mid:>3} {JOINT_NAME_BY_ID.get(mid,'?'):<20}{'--':>11}{'--':>11}")
                    continue
                d = wrap_to_pi(v - baseline[mid]) * RAD2DEG
                mark = "  <==" if abs(d) > 2.0 else ""
                rows.append(f"{mid:>3} {JOINT_NAME_BY_ID.get(mid,'?'):<20}"
                            f"{v*RAD2DEG:>11.2f}{d:>11.2f}{mark}")
            print("\n" + hdr)
            print("\n".join(rows), flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        bus.disconnect(disable_torque=False)   # stay strictly read-only


if __name__ == "__main__":
    main()
