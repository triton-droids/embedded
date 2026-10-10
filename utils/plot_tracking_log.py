#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Summarize and plot a per-step log written by ctrl_scripts/run_tracking_policy.py.

Answers two questions per joint:

  Is the policy doing what it should?
      action size, how often the target hit a joint limit (clipped) or the
      max_vel rate limit, and how far the robot ended up from the reference.

  Are the motors doing what they were told?
      tracking error between what was sent and where the joint actually went
      (unclamped), torque, temperature, and how often a motor missed a reply.

Every joint-space number is in the HARDWARE convention. The reference comes out
of the policy in the sim convention and is converted with joint_sign first.

Usage:
    ./.venv/bin/python utils/plot_tracking_log.py logs/tracking/tracking_live_20261002_013000.npz
    ./.venv/bin/python utils/plot_tracking_log.py <log.npz> --show     # also open the windows
    ./.venv/bin/python utils/plot_tracking_log.py <log.npz> --no-plot  # table only
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

RAD2DEG = 180.0 / np.pi

# From the RobStride RS02/RS03/RS04 manuals: peak torque and rated torque at stall.
PEAK_NM = {"rs-04": 120.0, "rs-03": 60.0, "rs-02": 17.0}
RATED_STALL_NM = {"rs-04": 28.5, "rs-03": 15.0, "rs-02": 6.0}

# Joint names are mirrored relative to the robot: left_* (motors 1-5) is the
# physical RIGHT leg. Label plots by motor and physical side, never by name.
PHYSICAL_SIDE = {"left": "phys R", "right": "phys L"}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("log", help="Path to a tracking_*.npz log.")
    p.add_argument("--show", action="store_true", help="Open the plot windows as well as saving PNGs.")
    p.add_argument("--no-plot", action="store_true", help="Print the summary only.")
    return p.parse_args()


def label(meta: dict, i: int) -> str:
    name = meta["joint_names"][i]
    side = PHYSICAL_SIDE.get(name.split("_")[0], "?")
    short = name.replace("_joint", "").split("_", 1)[1]
    mid = meta.get("motor_ids", [None] * len(meta["joint_names"]))[i]
    return f"m{mid} {short} ({side})" if mid is not None else f"{short} ({side})"


def rms(x: np.ndarray, axis: int = 0) -> np.ndarray:
    return np.sqrt(np.nanmean(np.square(x), axis=axis))


def summarize(d: dict, meta: dict) -> None:
    n = len(d["t"])
    dt = 1.0 / float(meta["control_hz"])
    loop_dt = d["loop_dt"][1:]
    sign = np.asarray(meta["joint_sign"])
    has_motors = not np.all(np.isnan(d["motor_pos"]))

    print("=" * 96)
    print(f"  {meta['mode']} run, {meta['started']}   steps {n}   frames {int(d['time_step'][0])}"
          f"..{int(d['time_step'][-1])}   stop: {meta.get('stop_reason', '?')}")
    if loop_dt.size:
        hz = 1.0 / np.median(loop_dt)
        over = int(np.sum(loop_dt > 1.5 * dt))
        print(f"  rate {hz:.1f} Hz (target {meta['control_hz']:.0f})   steps over 1.5x budget: {over}"
              f"   worst step {loop_dt.max() * 1000:.1f} ms")
    print(f"  policy {meta['policy_path']}")
    print(f"  action_scale {meta['action_scale']}   gains kp {meta.get('kp', '-')}")
    print("=" * 96)

    ref_hw = sign * d["ref_pos_sim"]
    action = d["action"]
    clipped = np.abs(d["target_raw"] - d["target_clipped"]) > 1e-6
    rate_lim = np.abs(d["commanded"] - d["target_clipped"]) > 1e-4

    print("\nPOLICY  (is it asking for sensible things?)")
    print(f"  {'joint':<24}{'|a| max':>9}{'|a| p95':>9}{'clipped':>9}{'rate-lim':>10}"
          f"{'ref err rms':>13}{'ref err max':>13}")
    meas = d["joint_pos_unclamped"] if has_motors else d["joint_pos"]
    ref_err = (meas - ref_hw) * RAD2DEG
    for i in range(action.shape[1]):
        ref_cols = (f"{rms(ref_err[:, i]):11.1f} deg{np.nanmax(np.abs(ref_err[:, i])):9.1f} deg"
                    if has_motors else f"{'-':>13}{'-':>13}")
        print(f"  {label(meta, i):<24}{np.abs(action[:, i]).max():9.2f}"
              f"{np.percentile(np.abs(action[:, i]), 95):9.2f}"
              f"{100 * clipped[:, i].mean():8.0f}%{100 * rate_lim[:, i].mean():9.0f}%{ref_cols}")
    print("  clipped  = policy target beyond the joint limit/clamp, so it was cut")
    print("  rate-lim = target moved faster than max_vel_rad_s allows, so the command lagged")
    print("  ref err  = measured joint vs the reference motion (policy + motors together)")

    if not has_motors:
        print("\n(offline run: no motor data. The joints never move, so the policy is reacting to a")
        print(" robot frozen at zero; actions and clipping here are not what a live run will show.)")
        return

    trk = (d["joint_pos_unclamped"] - d["sent"]) * RAD2DEG
    fresh = d["feedback_fresh"]
    print("\nMOTORS  (did they follow what was sent?)")
    print(f"  {'joint':<24}{'trk rms':>9}{'trk max':>9}{'|tau| pk':>10}{'|tau| rms':>10}"
          f"{'T max':>8}{'missed':>8}")
    for i in range(trk.shape[1]):
        tq = np.abs(d["motor_torque"][:, i])
        print(f"  {label(meta, i):<24}{rms(trk[:, i]):7.2f} °{np.nanmax(np.abs(trk[:, i])):7.2f} °"
              f"{np.nanmax(tq):8.1f}Nm{rms(tq):8.1f}Nm{np.nanmax(d['motor_temp'][:, i]):6.0f}°C"
              f"{100 * (1 - fresh[:, i].mean()):7.0f}%")
    print("  trk    = measured (unclamped) minus sent target; lag of a few degrees is normal at speed")
    print("  missed = control steps where that motor sent no status frame")

    models = meta.get("motor_models")
    if not models and meta.get("motor_ids"):
        # Logs recorded before motor_models was added: look the models up.
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from robot_hardware import MOTOR_MODEL_BY_ID
        models = [MOTOR_MODEL_BY_ID[mid] for mid in meta["motor_ids"]]
    if models:
        guard = meta.get("torque_guard", {})
        win = max(1, int(round(float(guard.get("sustained_window_s", 2.0)) * float(meta["control_hz"]))))
        clipped = d.get("guard_clipped")
        print("\nTORQUE MARGINS  (fresh replies only; ratings from the RobStride manuals)")
        print(f"  {'joint':<24}{'model':>6}{'|tau| pk':>10}{'% peak':>8}{'worst ' + str(win) + '-step RMS':>20}"
              f"{'% rated':>9}{'guard clip':>12}")
        for i, model in enumerate(models):
            tq = np.where(fresh[:, i] > 0, np.abs(d["motor_torque"][:, i]), np.nan)
            pk = np.nanmax(tq) if np.any(~np.isnan(tq)) else np.nan
            sq = np.nan_to_num(tq ** 2)
            n = np.convolve(fresh[:, i], np.ones(win), "valid")
            s2 = np.convolve(sq, np.ones(win), "valid")
            worst = np.sqrt(np.max(np.where(n > 0, s2 / np.maximum(n, 1), 0.0))) if len(n) else np.nan
            clip = f"{100 * clipped[:, i].mean():.0f}%" if clipped is not None else "-"
            flag = "  <==" if (pk > 0.8 * PEAK_NM[model] or worst > 0.8 * RATED_STALL_NM[model]) else ""
            print(f"  {label(meta, i):<24}{model:>6}{pk:8.1f}Nm{100 * pk / PEAK_NM[model]:7.0f}%"
                  f"{worst:18.1f}Nm{100 * worst / RATED_STALL_NM[model]:8.0f}%{clip:>12}{flag}")
        print("  % peak  = highest single reading vs the motor's peak torque")
        print("  % rated = worst rolling average vs rated torque at stall (above 100% the motor heats up)")
        print("  guard clip = steps where the torque guard pulled the target in")

    worst = int(np.nanargmax(rms(trk)))
    print(f"\n  worst tracking: {label(meta, worst)} at {rms(trk[:, worst]):.2f} deg rms")
    knees = [i for i, j in enumerate(meta["joint_names"]) if "knee" in j]
    if len(knees) == 2:
        a, b = knees
        print(f"  knee pair: {label(meta, a)} {rms(trk[:, a]):.2f} deg vs "
              f"{label(meta, b)} {rms(trk[:, b]):.2f} deg rms")


def plot(d: dict, meta: dict, out_stem: Path, show: bool) -> None:
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = d["t"]
    sign = np.asarray(meta["joint_sign"])
    ref_hw = sign * d["ref_pos_sim"] * RAD2DEG
    has_motors = not np.all(np.isnan(d["motor_pos"]))
    n_j = len(meta["joint_names"])

    # 1. Position: reference, sent target, measured.
    fig, axes = plt.subplots(5, 2, figsize=(14, 13), sharex=True)
    for i in range(n_j):
        ax = axes[i % 5, i // 5]
        ax.plot(t, ref_hw[:, i], color="0.6", lw=1.2, label="reference")
        ax.plot(t, d["sent"][:, i] * RAD2DEG, color="C0", lw=1.2, label="sent")
        if has_motors:
            ax.plot(t, d["joint_pos_unclamped"][:, i] * RAD2DEG, color="C3", lw=1.0, label="measured")
        lo, hi = meta["clip_lo"][i] * RAD2DEG, meta["clip_hi"][i] * RAD2DEG
        ax.axhline(lo, color="C1", lw=0.6, ls="--")
        ax.axhline(hi, color="C1", lw=0.6, ls="--")
        ax.set_title(label(meta, i), fontsize=9)
        ax.set_ylabel("deg", fontsize=8)
        ax.grid(alpha=0.3)
    axes[0, 0].legend(fontsize=8, loc="best")
    axes[-1, 0].set_xlabel("t [s]")
    axes[-1, 1].set_xlabel("t [s]")
    fig.suptitle(f"Joint position, hardware convention (dashed = clip limits)   {out_stem.name}", fontsize=10)
    fig.tight_layout()
    fig.savefig(f"{out_stem}_position.png", dpi=110)

    # 2. Policy action and motor torque.
    fig, axes = plt.subplots(5, 2, figsize=(14, 13), sharex=True)
    for i in range(n_j):
        ax = axes[i % 5, i // 5]
        ax.plot(t, d["action"][:, i], color="C2", lw=1.0, label="action")
        ax.set_ylabel("action", fontsize=8, color="C2")
        ax.grid(alpha=0.3)
        ax.set_title(label(meta, i), fontsize=9)
        if has_motors:
            ax2 = ax.twinx()
            ax2.plot(t, d["motor_torque"][:, i], color="C4", lw=0.9, label="torque")
            ax2.set_ylabel("Nm (motor space)", fontsize=8, color="C4")
    axes[-1, 0].set_xlabel("t [s]")
    axes[-1, 1].set_xlabel("t [s]")
    fig.suptitle(f"Policy action (green) and motor torque (purple)   {out_stem.name}", fontsize=10)
    fig.tight_layout()
    fig.savefig(f"{out_stem}_action_torque.png", dpi=110)

    # 3. IMU and timing.
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for k, c in enumerate("xyz"):
        axes[0].plot(t, d["proj_gravity"][:, k], label=f"g {c}")
        axes[1].plot(t, d["ang_vel"][:, k], label=f"w {c}")
    axes[0].set_ylabel("projected gravity")
    axes[1].set_ylabel("ang vel [rad/s]")
    axes[2].plot(t[1:], d["loop_dt"][1:] * 1000, color="k", lw=0.8)
    axes[2].axhline(1000.0 / meta["control_hz"], color="C3", ls="--", lw=0.8)
    axes[2].set_ylabel("step [ms]")
    axes[2].set_xlabel("t [s]")
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    axes[1].legend(fontsize=8)
    fig.suptitle(f"IMU (policy frame) and loop timing   {out_stem.name}", fontsize=10)
    fig.tight_layout()
    fig.savefig(f"{out_stem}_imu_timing.png", dpi=110)

    print(f"\nplots -> {out_stem}_position.png, _action_torque.png, _imu_timing.png")
    if show:
        plt.show()


def main() -> None:
    args = parse_args()
    path = Path(args.log).expanduser()
    with np.load(path, allow_pickle=False) as z:
        meta = json.loads(str(z["meta"]))
        d = {k: z[k] for k in z.files if k != "meta"}
    summarize(d, meta)
    if not args.no_plot:
        plot(d, meta, path.with_suffix(""), args.show)


if __name__ == "__main__":
    main()
