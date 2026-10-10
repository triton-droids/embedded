#!/usr/bin/env python3
"""
Summarise a system-ID session written by ctrl_scripts/sysid_logger.py.

    ./.venv/bin/python utils/sysid_report.py logs/system_id/<session>
    ./.venv/bin/python utils/sysid_report.py logs/system_id/<session>/m9_step6_*.npz
    ./.venv/bin/python utils/sysid_report.py logs/tracking/tracking_live_*.npz   # old tracker logs

Per run it prints:
- timing: loop period, achieved rate
- comms per motor: reply rate, reply latency (kernel rx minus tx), state age
- silences: steps where motors stopped replying, with the fault bits, mode,
  torque and VBUS from just before
- status flags, raw frame types, CAN error frames
- power: VBUS per motor
- trial response for the moving joint: tracking error, delay, step metrics

Old tracker logs only have feedback_fresh and motor state, so only the silence
and torque sections apply to them.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

FAULT_NAMES = ["undervoltage", "overcurrent", "overtemp", "encoder", "stall", "uncalibrated"]


def pct(x, q):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    return float(np.percentile(x, q)) if x.size else float("nan")


def fmt(v, spec=".1f", none="-"):
    return none if v is None or not np.isfinite(v) else format(v, spec)


def load(path: Path) -> tuple[dict, dict]:
    z = np.load(path, allow_pickle=False)
    meta = json.loads(str(z["meta"])) if "meta" in z.files else {}
    return meta, {k: z[k] for k in z.files if k != "meta"}


def runs_in(target: Path) -> list[Path]:
    if target.is_dir():
        return sorted(target.glob("*.npz"))
    return [target]


def streaks(mask: np.ndarray) -> list[tuple[int, int]]:
    """(start, length) of runs of True."""
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        elif not m and start is not None:
            out.append((start, i - start))
            start = None
    if start is not None:
        out.append((start, len(mask) - start))
    return out


def estimate_delay(cmd: np.ndarray, pos: np.ndarray, dt: float, max_lag: int = 10) -> float:
    """Lag (s) maximising the correlation of the command and position rates,
    refined with a parabola through the peak."""
    a, b = np.diff(cmd), np.diff(pos)
    ok = np.isfinite(a) & np.isfinite(b)
    a, b = a[ok] - a[ok].mean(), b[ok] - b[ok].mean()
    if a.size < 3 * max_lag or np.std(a) < 1e-9:
        return float("nan")
    lags = np.arange(0, max_lag + 1)
    c = np.array([np.dot(a[: len(a) - k], b[k:]) / (len(a) - k) for k in lags])
    k = int(np.argmax(c))
    if 0 < k < max_lag:
        d = c[k - 1] - 2 * c[k] + c[k + 1]
        frac = 0.5 * (c[k - 1] - c[k + 1]) / d if d != 0 else 0.0
        return (k + frac) * dt
    return k * dt


def step_metrics(t, target, pos):
    """For each change in the (pre-slew) target: 90% rise time, overshoot, final error."""
    out = []
    changes = np.flatnonzero(np.abs(np.diff(target)) > 1e-4) + 1
    bounds = list(changes) + [len(target)]
    for s, e in zip(bounds[:-1], bounds[1:]):
        p0, goal = pos[s - 1], target[s]
        size = goal - p0
        if abs(size) < math.radians(0.5) or e - s < 10:
            continue
        seg = (pos[s:e] - p0) / size
        reach = np.flatnonzero(seg >= 0.9)
        rise = (t[s + reach[0]] - t[s]) if reach.size else float("nan")
        out.append({
            "t": float(t[s]), "size_deg": math.degrees(size),
            "rise90_ms": rise * 1000, "overshoot_pct": max(0.0, (np.nanmax(seg) - 1.0) * 100),
            "final_err_deg": math.degrees(pos[e - 1] - goal),
        })
    return out


def report_run(path: Path) -> dict:
    meta, d = load(path)
    print("=" * 100)
    print(f"{path.name}   run={meta.get('run', meta.get('mode', '?'))}   "
          f"stop: {meta.get('stop_reason', '?')}")
    n = len(d.get("t", []))
    if n == 0:
        print("  no steps recorded")
        return {}
    ids = meta.get("motor_ids") or list(range(1, 11))
    names = meta.get("joint_names") or [f"m{i}" for i in ids]
    t = d["t"]
    dt_nom = 1.0 / float(meta.get("control_hz", 50.0))

    # ---- timing
    loop = d["loop_dt"][1:] * 1000 if "loop_dt" in d else np.array([])
    dur = t[-1] - t[0] if n > 1 else 0.0
    print(f"  TIMING  {n} steps, {dur:.2f} s, {(n - 1) / dur if dur > 0 else float('nan'):.1f} Hz | "
          f"loop dt p50 {fmt(pct(loop, 50))} p99 {fmt(pct(loop, 99))} max {fmt(pct(loop, 100))} ms"
          + (f" | step time incl. reply wait p99 {pct(d['compute_s'] * 1000, 99):.1f} ms"
             if "compute_s" in d else ""))

    fresh = d["feedback_fresh"] > 0.5 if "feedback_fresh" in d else np.ones((n, len(ids)), bool)
    fresh_eval = fresh[1:]   # the first cycle routinely misses replies

    # ---- comms per motor
    has_lat = "reply_latency" in d
    print(f"\n  {'COMMS':<20} {'reply%':>7} {'missed':>6} {'streak':>6} "
          + (f"{'lat p50':>8} {'p95':>6} {'p99':>6} {'max':>6} {'age p50':>8} {'max':>6} {'dup':>4}"
             if has_lat else ""))
    for j, (mid, nm) in enumerate(zip(ids, names)):
        miss = ~fresh_eval[:, j]
        longest = max((L for _, L in streaks(miss)), default=0)
        row = f"  m{mid:<2} {nm:<16} {100 * fresh_eval[:, j].mean():>7.1f} {int(miss.sum()):>6} {longest:>6} "
        if has_lat:
            # Step 0 drains the replies left over from connect(), which are old.
            lat = d["reply_latency"][1:, j] * 1000
            age = d["state_age"][1:, j] * 1000
            row += (f"{fmt(pct(lat, 50)):>8} {fmt(pct(lat, 95)):>6} {fmt(pct(lat, 99)):>6} "
                    f"{fmt(pct(lat, 100)):>6} {fmt(pct(age, 50)):>8} {fmt(pct(age, 100)):>6} "
                    f"{int(d['status_dup'][:, j].sum()):>4}")
        print(row)
    if has_lat:
        print("  (lat = kernel receive time minus the previous command's send time; "
              "age = how old the newest reply is when the step uses it, both ms)")

    # ---- silences
    any_miss = (~fresh).any(axis=1)
    any_miss[0] = False
    silences = streaks(any_miss)
    print(f"\n  SILENCES  {len(silences)} spans with at least one motor missing")
    torque = d.get("motor_torque")
    vbus_last = d.get("vbus_last")
    for s, L in silences[:20]:
        who = np.flatnonzero((~fresh[s:s + L]).any(axis=0))
        bus_wide = bool((~fresh[s:s + L]).all(axis=1).any())
        line = (f"    step {s} t={t[s]:.2f}s len {L} ({L * dt_nom * 1000:.0f} ms) "
                f"{'BUS-WIDE ' if bus_wide else ''}motors {[ids[k] for k in who]}")
        # Each silent motor's own last reply before it went quiet.
        last = []
        for k in who:
            first_miss = s + int(np.argmax(~fresh[s:s + L, k]))
            rows = np.flatnonzero(fresh[:first_miss, k])
            last.append(int(rows[-1]) if rows.size else -1)
        have = [(k, r) for k, r in zip(who, last) if r >= 0]
        if have:
            if torque is not None:
                line += f" | last tau {[round(float(torque[r, k]), 2) for k, r in have]}"
            if "status_mode" in d:
                line += f" mode {[int(d['status_mode'][r, k]) for k, r in have]}"
                fb = [int(d["status_fault_bits"][r, k]) for k, r in have]
                if any(v > 0 for v in fb):
                    line += f" faults {[hex(v) for v in fb]}"
            prev = max(r for _, r in have)
            if vbus_last is not None and np.isfinite(vbus_last[prev]).any():
                line += f" VBUS min {np.nanmin(vbus_last[prev]):.2f} V"
        print(line)
    if len(silences) > 20:
        print(f"    ... {len(silences) - 20} more")

    # ---- status flags and frames
    if "status_fault_bits" in d:
        fb = d["status_fault_bits"].astype(int)
        md = d["status_mode"].astype(int)
        for j, mid in enumerate(ids):
            bits = np.bitwise_or.reduce(np.where(fb[:, j] > 0, fb[:, j], 0))
            modes = sorted(set(md[:, j][md[:, j] >= 0].tolist()))
            if bits or modes not in ([2], []):
                flags = [FAULT_NAMES[b] for b in range(6) if (bits >> b) & 1]
                print(f"  STATUS  m{mid}: modes seen {modes} (2 = run), fault bits {flags or 'none'}")
    if "rx_arb_id" in d and len(d["rx_arb_id"]):
        arb = d["rx_arb_id"].astype(np.int64)
        ext = d["rx_extended"].astype(bool)
        types = (arb >> 24) & 0x1F
        counts = {int(k): int(v) for k, v in zip(*np.unique(types[ext], return_counts=True))}
        print(f"\n  FRAMES  rx {len(arb)} (by type {counts}, non-extended {int((~ext).sum())}), "
              f"tx {len(d['tx_step'])}, tx failures {int((~d['tx_ok'].astype(bool)).sum())}")
    events = meta.get("events") or []
    if events:
        kinds: dict[str, int] = {}
        for ev in events:
            kinds[ev["kind"]] = kinds.get(ev["kind"], 0) + 1
        print(f"  EVENTS  {kinds}")
        for ev in [e for e in events if e["kind"] not in ("missed reply",)][:15]:
            who = f" m{ev['motor_id']}" if ev.get("motor_id") is not None else ""
            print(f"    step {ev['step']} t={ev['t']:.2f}s {ev['kind']}{who}: {ev['detail']}")

    # ---- power
    if "vbus" in d and np.isfinite(d["vbus"]).any():
        v = d["vbus"]
        print(f"\n  POWER  VBUS per motor (V), polled one motor per step")
        print("    " + "  ".join(f"m{mid}:{fmt(np.nanmin(v[:, j]), '.2f')}-{fmt(np.nanmax(v[:, j]), '.2f')}"
                                 if np.isfinite(v[:, j]).any() else f"m{mid}:-"
                                 for j, mid in enumerate(ids)))
        k = int(np.nanargmin(v) // v.shape[1])
        print(f"    lowest {np.nanmin(v):.2f} V at t={t[k]:.2f}s, total |tau| then "
              f"{np.nansum(np.abs(torque[k])) if torque is not None else float('nan'):.1f} Nm")

    # ---- torque / temperature
    if torque is not None:
        temp = d.get("motor_temp")
        print(f"\n  LOAD   max|tau| (Nm) / temp start->end (C)")
        print("    " + "  ".join(
            f"m{mid}:{np.nanmax(np.abs(torque[:, j])):.1f}"
            + (f"/{temp[0, j]:.0f}->{temp[-1, j]:.0f}" if temp is not None else "")
            for j, mid in enumerate(ids)))

    # ---- trial response
    info = meta.get("trial_info") or {}
    result = {"file": path.name, "silences": len(silences)}
    if info and "sent" in d:
        j = int(info["active_index"])
        trial = meta.get("trial") or {}
        ok = np.isfinite(d["sent"][:, j])
        tt, sent = t[ok], d["sent"][ok, j]
        pos = d["joint_pos_unclamped"][ok, j]
        target = d["target_clipped"][ok, j]
        err = pos - sent
        delay = estimate_delay(sent, pos, dt_nom)
        slewed = np.abs(d["commanded"][ok, j] - target) > 1e-6
        print(f"\n  RESPONSE  m{ids[j]} {names[j]}  {trial.get('kind')} ({trial.get('tag')})  "
              f"base {math.degrees(info['base_rad']):+.2f} deg")
        print(f"    position - sent: RMS {math.degrees(np.sqrt(np.mean(err ** 2))):.2f} deg, "
              f"max {math.degrees(np.max(np.abs(err))):.2f} deg | delay {fmt(delay * 1000)} ms | "
              f"slew-limited {100 * slewed.mean():.0f}% of steps | "
              f"guard-clipped {int(d['guard_clipped'][ok, j].sum())} steps")
        if trial.get("kind") == "steps":
            for m in step_metrics(tt, target, pos):
                print(f"    step {m['size_deg']:+6.2f} deg at {m['t']:.2f}s: rise90 {fmt(m['rise90_ms'], '.0f')} ms, "
                      f"overshoot {m['overshoot_pct']:.0f}%, final error {m['final_err_deg']:+.2f} deg")
        result.update(delay_ms=delay * 1000, rms_err_deg=math.degrees(np.sqrt(np.mean(err ** 2))))
    return result


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("target", help="session directory or .npz")
    args = p.parse_args()
    target = Path(args.target).expanduser()
    manifest = target / "manifest.json" if target.is_dir() else None
    if manifest is not None and manifest.is_file():
        m = json.loads(manifest.read_text())
        print(f"SESSION {m.get('session')}  created {m.get('created')}  {len(m['runs'])} runs")
        for r in m["runs"]:
            print(f"  {r.get('started', ''):<16} {r['run']:<18} {r.get('stop_reason', '')}")
    paths = runs_in(target)
    if not paths:
        sys.exit(f"no .npz under {target}")
    for path in paths:
        if path.stat().st_size == 0:
            print("=" * 100)
            print(f"{path.name}   EMPTY FILE (0 bytes), skipped")
            continue
        report_run(path)


if __name__ == "__main__":
    main()
