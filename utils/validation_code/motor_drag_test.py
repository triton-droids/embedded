#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Measure mechanical drag in a single motor by sweeping it slowly and logging the
torque required. Run it on two mirrored joints and compare.

MOVES THE JOINT. One motor at a time, low gains, small slow arc. Suspend the
robot and keep the limb clear before running.

How it separates friction from gravity
--------------------------------------
During a slow sweep, inertial torque is negligible, so

    moving one way:   tau = tau_gravity + tau_friction
    moving the other: tau = tau_gravity - tau_friction

so, splitting the samples by the sign of the velocity,

    friction ~= (mean_tau_positive - mean_tau_negative) / 2
    gravity  ~= (mean_tau_positive + mean_tau_negative) / 2

The friction term is what to compare between two motors. It is pose-independent
to first order, which the raw torque is not.

Everything is in MOTOR space, not joint space: no inversion is applied. Two
mirrored joints therefore sweep in physically opposite directions, so their
gravity terms will have opposite sign. Friction magnitude stays comparable.

Usage
-----
    ./.venv/bin/python utils/validation_code/motor_drag_test.py --motor 4
    ./.venv/bin/python utils/validation_code/motor_drag_test.py --motor 9

Compare the reported friction figures. Same model, same role, similar pose ->
a materially larger friction term is real drag.
"""

from __future__ import annotations

import argparse
import math
import statistics as st
import struct
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
    JOINT_LIMITS_BY_ID,
    JOINT_NAME_BY_ID,
    MOTOR_MODEL_BY_ID,
)
from robstride_dynamics import CommunicationType, Motor, ParameterType, RobstrideBus

RAD2DEG = 180.0 / math.pi


# NOTE: deliberately no wrap_to_pi / MECHANICAL_POSITION helper here. MIT
# position is a continuous multi-turn value over +/-4*pi; wrapping a pose to
# +/-pi and commanding it into that space makes the motor unwind a full
# revolution at whatever torque the gains allow.


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--motor", type=int, required=True, help="Motor ID to test.")
    p.add_argument("--amplitude-deg", type=float, default=12.0,
                   help="Half-amplitude of the sweep about the start pose.")
    p.add_argument("--period-s", type=float, default=8.0,
                   help="Seconds per cycle. Slow keeps inertia negligible.")
    p.add_argument("--cycles", type=float, default=2.0)
    p.add_argument("--kp", type=float, default=25.0, help="MIT position gain. Keep low.")
    p.add_argument("--kd", type=float, default=1.0, help="MIT damping gain.")
    p.add_argument("--rate-hz", type=float, default=50.0)
    p.add_argument("--channel", default=CAN_CHANNEL)
    p.add_argument("--bitrate", type=int, default=BITRATE)
    p.add_argument("--max-err-deg", type=float, default=15.0,
                   help="Abort if |measured - commanded| exceeds this. Catches a frame "
                        "mismatch in the first few milliseconds instead of saturating.")
    p.add_argument("--max-torque-nm", type=float, default=25.0,
                   help="Abort if |torque| exceeds this.")
    p.add_argument("--csv", default=None, help="Optional path to write the raw log.")
    p.add_argument("--yes", action="store_true", help="Skip the confirmation prompt.")
    return p.parse_args()


def main() -> None:
    a = parse_args()
    mid = a.motor
    name = f"motor_{mid}"
    joint = JOINT_NAME_BY_ID.get(mid, name)
    model = MOTOR_MODEL_BY_ID.get(mid, "rs-03")
    amp = math.radians(a.amplitude_deg)
    dt = 1.0 / a.rate_hz

    print("=" * 68)
    print(f"  DRAG TEST — motor {mid} ({joint}, {model})")
    print(f"  sweep +/-{a.amplitude_deg:.1f} deg, {a.period_s:.1f} s/cycle x {a.cycles:g}")
    print(f"  gains kp={a.kp:g} kd={a.kd:g}")
    print("  THIS MOVES THE JOINT. Robot suspended, limb clear.")
    print("=" * 68)
    if not a.yes:
        try:
            if input("  type 'go' to start: ").strip().lower() != "go":
                print("  aborted."); return
        except (EOFError, KeyboardInterrupt):
            print("\n  aborted."); return

    motors = {name: Motor(id=mid, model=model)}
    calibration = {name: {"direction": 1, "homing_offset": 0.0}}
    bus = RobstrideBus(a.channel, motors, calibration, bitrate=a.bitrate)
    bus.connect(handshake=True)

    def set_mit_mode() -> None:
        param_id, _, _ = ParameterType.MODE
        data = struct.pack("<HH", param_id, 0x00) + struct.pack("<bBH", 0, 0, 0)
        bus.transmit(CommunicationType.WRITE_PARAMETER, bus.host_id, mid, data)
        time.sleep(0.05)

    log: list[tuple[float, float, float, float, float]] = []

    # The start pose MUST come from the MIT status frame, in the same space the
    # command is sent in. MIT position spans +/-4*pi and is a continuous
    # multi-turn value; MECHANICAL_POSITION is a different representation, and
    # wrapping it to +/-pi discards the turn count. Commanding a wrapped value
    # into unwrapped MIT space makes the motor try to unwind a whole revolution
    # at full torque -- which is exactly what happened on 2026-09-27.
    try:
        bus.enable(name)
        time.sleep(0.1)
        set_mit_mode()
        start, _v0, _t0, _tmp = bus.read_operation_frame(name, timeout=0.2)
        start = float(start)
    except Exception as exc:
        print(f"  [!] could not read the MIT start position: {exc}")
        try:
            bus.disable(name)
        except Exception:
            pass
        bus.disconnect(disable_torque=False)
        return

    span = 4.0 * math.pi
    if abs(start) > span - amp:
        print(f"  [!] start {start:+.3f} rad is too close to the +/-4*pi MIT range edge.")
        bus.disable(name); bus.disconnect(disable_torque=False); return

    print(f"\n  start {start*RAD2DEG:+.2f} deg, sweeping +/-{amp*RAD2DEG:.1f} deg\n")
    try:
        # Ease into the start pose with a soft gain before sweeping, so a bad
        # start value shows up as a small nudge rather than a lunge.
        for i in range(25):
            bus.write_operation_frame(name, start, a.kp * 0.3, a.kd, 0.0, 0.0)
            time.sleep(0.02)
        pos0, _v, tq0, _t = bus.read_operation_frame(name, timeout=0.05)
        if abs(float(pos0) - start) > math.radians(a.max_err_deg):
            raise RuntimeError(
                f"start pose did not settle: commanded {start:+.3f} rad, "
                f"measured {float(pos0):+.3f} rad. Aborting before the sweep.")
        time.sleep(0.2)

        duration = a.period_s * a.cycles
        miss = 0
        t0 = time.perf_counter()
        next_tick = t0
        while True:
            t = time.perf_counter() - t0
            if t >= duration:
                break
            phase = 2.0 * math.pi * t / a.period_s
            cmd = start + amp * math.sin(phase)
            cmd_vel = amp * (2.0 * math.pi / a.period_s) * math.cos(phase)

            bus.write_operation_frame(name, cmd, a.kp, a.kd, 0.0, 0.0)
            try:
                pos, vel, tq, _temp = bus.read_operation_frame(name, timeout=0.02)
                pos, vel, tq = float(pos), float(vel), float(tq)
                log.append((t, cmd, pos, vel, tq))
                if abs(pos - cmd) > math.radians(a.max_err_deg):
                    raise RuntimeError(
                        f"tracking error {abs(pos-cmd)*RAD2DEG:.1f} deg exceeds "
                        f"{a.max_err_deg:.1f} deg -- the motor is not following. "
                        f"Aborting.")
                if abs(tq) > a.max_torque_nm:
                    raise RuntimeError(
                        f"torque {abs(tq):.1f} Nm exceeds {a.max_torque_nm:.1f} Nm. "
                        f"Aborting.")
                miss = 0
            except RuntimeError:
                raise
            except Exception:
                # A failed status read means the guards below have nothing to
                # check while the loop keeps commanding. Blind commanding is
                # exactly how the 2026-09-27 overload went unnoticed, so give
                # up rather than continue without feedback.
                miss += 1
                if miss >= 5:
                    raise RuntimeError(
                        f"{miss} consecutive status reads failed -- commanding "
                        f"blind. Aborting.")

            if t % 1.0 < dt:
                print(f"\r  t={t:5.1f}s  cmd={cmd*RAD2DEG:+7.2f}  n={len(log):4d}",
                      end="", flush=True)
            next_tick += dt
            sleep = next_tick - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)

        # ease back to the start pose before releasing
        for i in range(30):
            bus.write_operation_frame(name, start, a.kp, a.kd, 0.0, 0.0)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n  interrupted")
    except RuntimeError as exc:
        print(f"\n\n  [ABORT] {exc}")
    finally:
        try:
            here, _v, _t, _tm = bus.read_operation_frame(name, timeout=0.05)
            bus.write_operation_frame(name, float(here), 0.0, 0.5, 0.0, 0.0)
            time.sleep(0.05)
            bus.disable(name)
        except Exception:
            pass
        bus.disconnect(disable_torque=False)
        print("\n  motor disabled.\n")

    if len(log) < 40:
        print(f"  only {len(log)} samples; not enough to analyse.")
        return

    if a.csv:
        with open(a.csv, "w", encoding="utf-8") as f:
            f.write("t_s,cmd_rad,pos_rad,vel_rad_s,torque_nm\n")
            for row in log:
                f.write(",".join(f"{x:.6f}" for x in row) + "\n")
        print(f"  raw log -> {a.csv}")

    vel = [r[3] for r in log]
    tq = [r[4] for r in log]
    err = [abs(r[1] - r[2]) for r in log]
    vmax = max(abs(v) for v in vel) or 1.0
    gate = 0.25 * vmax          # ignore near-stationary samples at the turnarounds

    pos_t = [q for v, q in zip(vel, tq) if v > gate]
    neg_t = [q for v, q in zip(vel, tq) if v < -gate]

    print(f"  samples        {len(log)}  ({len(pos_t)} fwd / {len(neg_t)} rev above the velocity gate)")
    print(f"  peak speed     {vmax:.3f} rad/s")
    print(f"  tracking err   mean {st.mean(err)*RAD2DEG:.2f} deg,  max {max(err)*RAD2DEG:.2f} deg")
    print(f"  torque         mean |tau| {st.mean([abs(q) for q in tq]):.3f} Nm,  "
          f"peak {max(abs(q) for q in tq):.3f} Nm")

    if len(pos_t) >= 10 and len(neg_t) >= 10:
        mp, mn = st.mean(pos_t), st.mean(neg_t)
        friction = (mp - mn) / 2.0
        gravity = (mp + mn) / 2.0
        print()
        print(f"  mean tau, moving forward   {mp:+.3f} Nm")
        print(f"  mean tau, moving reverse   {mn:+.3f} Nm")
        print(f"  --> FRICTION term          {abs(friction):.3f} Nm   <-- compare this between motors")
        print(f"  --> gravity/bias term      {gravity:+.3f} Nm   (pose-dependent, not comparable)")
    else:
        print("\n  not enough samples on both directions to separate friction from gravity;"
              "\n  try a longer --period-s or more --cycles.")


if __name__ == "__main__":
    main()
