#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Move motors to the same default joint positions used by collect_motor_dataset.py.

Behavior:
- Connects to the selected motors
- Smoothly moves them to the dataset collector's default joint positions
- Holds that pose while the script is running
- Disables all selected motors on normal exit or Ctrl+C
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from typing import Dict, Optional

from collect_motor_dataset import (
    DEFAULT_JOINT_POS_BY_ID,
    RobstrideController,
    clamp,
    scan_motor_ids_or_parse,
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Move motors to the dataset default pose, then go limp on exit.")
    p.add_argument("--motor-ids", default="", help="Space-separated motor IDs. Empty means scan bus.")
    p.add_argument("--channel", default="can0", help="CAN interface name.")
    p.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bitrate.")
    p.add_argument("--hz", type=float, default=120.0, help="Control/read rate for controller setup.")
    p.add_argument("--kp", type=float, default=10.0, help="Fallback kp if a motor is not in GAINS_BY_ID.")
    p.add_argument("--kd", type=float, default=0.2, help="Fallback kd if a motor is not in GAINS_BY_ID.")
    p.add_argument(
        "--move-duration-s",
        type=float,
        default=6.0,
        help="Seconds to ramp into the default pose.",
    )
    p.add_argument(
        "--move-hz",
        type=float,
        default=200.0,
        help="Interpolation rate used during the startup move.",
    )
    p.add_argument(
        "--settle-duration-s",
        type=float,
        default=2.0,
        help="Seconds to actively hold the final target before printing final readings.",
    )
    p.add_argument(
        "--hold-hz",
        type=float,
        default=60.0,
        help="Rate used to actively resend the final target while holding.",
    )
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


def active_hold_targets(ctrl: RobstrideController, targets: Dict[int, float], duration_s: Optional[float], hz: float, stop_fn) -> None:
    hz = max(1.0, float(hz))
    dt = 1.0 / hz
    next_t = time.perf_counter()
    deadline = None if duration_s is None else (next_t + max(0.0, float(duration_s)))
    fail_counts = {mid: 0 for mid in targets}

    while True:
        if stop_fn():
            return
        now = time.perf_counter()
        if deadline is not None and now >= deadline:
            break

        for mid, tgt in targets.items():
            try:
                ctrl.write_logical_and_read(mid, tgt, timeout=0.01)
                fail_counts[mid] = 0
            except Exception as exc:
                fail_counts[mid] += 1
                if fail_counts[mid] <= 3 or (fail_counts[mid] % 20 == 0):
                    print(f"WARNING: hold write/read failed on motor {mid}: {exc}")

        next_t += dt
        now2 = time.perf_counter()
        if now2 < next_t:
            time.sleep(next_t - now2)
        else:
            next_t = now2


def print_final_readings(ctrl: RobstrideController, targets: Dict[int, float]) -> None:
    ctrl.read_all_once()
    print("Final readings at hold transition:")
    print("-" * 106)
    print(
        f"{'ID':<4} {'Cmd(rad)':<10} {'Pos(rad)':<10} {'Err(rad)':<10} "
        f"{'Vel(rad/s)':<12} {'Tq(Nm)':<10} {'Temp(C)':<10}"
    )
    print("-" * 106)
    for mid in sorted(targets.keys()):
        st = ctrl.states[mid]
        measured_logical = float(st.pos) / float(st.direction)
        err = measured_logical - float(st.cmd_logical)
        temp_str = "nan" if not math.isfinite(float(st.temp)) else f"{float(st.temp):<10.1f}"
        print(
            f"{mid:<4} {float(st.cmd_logical):<10.4f} {measured_logical:<10.4f} {err:<10.4f} "
            f"{float(st.vel):<12.4f} {float(st.tq):<10.4f} {temp_str}"
        )
    print("-" * 106)


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

    stop_requested = False

    def _request_stop(_sig=None, _frm=None):
        nonlocal stop_requested
        if not stop_requested:
            print("\nStop requested. Disabling motors...")
        stop_requested = True

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    try:
        ctrl.connect()
        print(f"Connected on {args.channel}. Motors: {ctrl.motor_ids}")

        default_targets: Dict[int, float] = {}
        for mid in ctrl.motor_ids:
            if mid not in DEFAULT_JOINT_POS_BY_ID:
                continue
            st = ctrl.states[mid]
            default_targets[mid] = clamp(DEFAULT_JOINT_POS_BY_ID[mid], st.lim_lo, st.lim_hi)

        if not default_targets:
            print("No default targets found for the selected motor IDs.")
            return 1

        print(
            "Moving controlled joints to default joint positions "
            f"(duration={args.move_duration_s:.2f}s, rate={args.move_hz:.1f}Hz)..."
        )
        ctrl.move_to_targets(
            default_targets,
            duration_s=float(args.move_duration_s),
            hz=float(args.move_hz),
        )
        print(
            "Actively holding final target to settle "
            f"(duration={args.settle_duration_s:.2f}s, rate={args.hold_hz:.1f}Hz)..."
        )
        active_hold_targets(
            ctrl,
            default_targets,
            duration_s=float(args.settle_duration_s),
            hz=float(args.hold_hz),
            stop_fn=lambda: stop_requested,
        )
        if stop_requested:
            return 0
        print_final_readings(ctrl, default_targets)
        print("Default joint positioning complete.")
        print(f"Actively holding default pose at {args.hold_hz:.1f}Hz. Press Ctrl+C to disable motors and exit.")
        active_hold_targets(
            ctrl,
            default_targets,
            duration_s=None,
            hz=float(args.hold_hz),
            stop_fn=lambda: stop_requested,
        )

        return 0
    finally:
        try:
            ctrl.shutdown()
        except Exception as exc:
            print(f"WARNING: shutdown failed: {exc}")


if __name__ == "__main__":
    sys.exit(main())
