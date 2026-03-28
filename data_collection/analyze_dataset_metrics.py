#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path

import numpy as np

MOTOR_GROUPS = {
    "group_1_6": (1, 6),
    "group_2_7": (2, 7),
    "group_3_8": (3, 8),
    "group_4_9": (4, 9),
    "group_5_10": (5, 10),
}


def moving_average(x: np.ndarray, win: int) -> np.ndarray:
    if x.size == 0 or win <= 1:
        return x.copy()
    kernel = np.ones(win, dtype=np.float64) / float(win)
    pad_left = win // 2
    pad_right = win - 1 - pad_left
    xp = np.pad(x, (pad_left, pad_right), mode="edge")
    return np.convolve(xp, kernel, mode="valid")


def summarize(x: np.ndarray) -> dict[str, float]:
    if x.size == 0:
        return {}
    return {
        "min": float(np.min(x)),
        "max": float(np.max(x)),
        "p1": float(np.percentile(x, 1)),
        "p5": float(np.percentile(x, 5)),
        "p50": float(np.percentile(x, 50)),
        "p95": float(np.percentile(x, 95)),
        "p99": float(np.percentile(x, 99)),
        "std": float(np.std(x)),
        "var": float(np.var(x)),
        "n": int(x.size),
    }


def p95_abs(x: np.ndarray) -> float:
    if x.size == 0:
        return float("nan")
    return float(np.percentile(np.abs(x), 95))


def append_summary(lines: list[str], title: str, stats: dict[str, float], unit: str) -> None:
    lines.append(title)
    if not stats:
        lines.append("  empty")
        return
    for key in ("min", "max", "p1", "p5", "p50", "p95", "p99", "std", "var", "n"):
        value = stats[key]
        suffix = f" {unit}" if key != "n" and unit else ""
        lines.append(f"  {key}: {value}{suffix}")


def concat_arrays(chunks: list[np.ndarray]) -> np.ndarray:
    return np.concatenate(chunks) if chunks else np.array([], dtype=np.float64)


def format_float(x: float) -> str:
    return f"{x:.6f}" if np.isfinite(x) else "nan"


def group_name_for_motor(motor_id: int) -> str | None:
    for group_name, motor_ids in MOTOR_GROUPS.items():
        if motor_id in motor_ids:
            return group_name
    return None


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Analyze collected motor dataset logs for quiet-state noise and "
            "timestamp-based latency metrics."
        )
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path(__file__).resolve().parent / "motor_dataset",
        help="Root directory containing collected run folders.",
    )
    parser.add_argument(
        "--quiet-command-rate-rads",
        type=float,
        default=0.01,
        help="Treat a sample as quiet when commanded position rate is below this threshold.",
    )
    parser.add_argument(
        "--quiet-velocity-rads",
        type=float,
        default=0.05,
        help="Treat a sample as quiet when measured angular velocity is below this threshold.",
    )
    parser.add_argument(
        "--trend-window-s",
        type=float,
        default=0.25,
        help="Moving-average window used to remove slow trends before measuring residual noise.",
    )
    parser.add_argument(
        "--lag-corr-threshold",
        type=float,
        default=0.8,
        help="Minimum cross-correlation required before reporting estimated actuation lag.",
    )
    parser.add_argument(
        "--report-out",
        type=Path,
        default=Path(__file__).resolve().parent / "dataset_metrics_sim2real_report.txt",
        help="Path to a text report file to write.",
    )
    args = parser.parse_args()

    npz_files = sorted(args.dataset_root.rglob("log.npz"))
    if not npz_files:
        raise SystemExit(f"No log.npz files found under {args.dataset_root}")

    quiet_pos_res = []
    quiet_vel_res = []
    quiet_tq_res = []
    all_feedback_minus_command_ms = []
    lag_results = []
    by_motor = defaultdict(lambda: {"pos": [], "vel": [], "tq": [], "lat": []})
    by_group = defaultdict(lambda: {"pos": [], "vel": [], "tq": [], "lat": []})

    for path in npz_files:
        data = np.load(path)
        motor_id = data["motor_id"]
        cycle_idx = data["cycle_idx"]
        cmd = data["commanded_position_rad"].astype(np.float64)
        pos = data["actual_position_rad"].astype(np.float64)
        vel = data["motor_velocity_rads"].astype(np.float64)
        tq = data["motor_torque_nm"].astype(np.float64)
        cmd_ts = data["commanded_timestamp_ns"].astype(np.int64)
        fb_ts = data["feedback_timestamp_ns"].astype(np.int64)
        feedback_minus_command_ms = (fb_ts - cmd_ts) / 1e6
        all_feedback_minus_command_ms.append(feedback_minus_command_ms)

        for mid in np.unique(motor_id):
            mask = motor_id == mid
            order = np.argsort(cycle_idx[mask], kind="stable")
            c = cmd[mask][order]
            p = pos[mask][order]
            v = vel[mask][order]
            q = tq[mask][order]
            cts = cmd_ts[mask][order]
            fms = feedback_minus_command_ms[mask][order]

            dts = np.diff(cts) / 1e9
            dts = dts[(dts > 0) & np.isfinite(dts)]
            dt = float(np.median(dts)) if dts.size else (1.0 / 120.0)
            win = max(5, int(round(args.trend_window_s / dt)))

            cmd_rate = np.zeros_like(c)
            if c.size > 1:
                rate = np.abs(np.diff(c) / max(dt, 1e-9))
                cmd_rate[1:] = rate
                cmd_rate[0] = rate[0]

            quiet = (
                (cmd_rate < args.quiet_command_rate_rads)
                & (np.abs(v) < args.quiet_velocity_rads)
            )

            if quiet.any():
                pos_res = (p - moving_average(p, win))[quiet]
                vel_res = (v - moving_average(v, win))[quiet]
                tq_res = (q - moving_average(q, win))[quiet]
                quiet_pos_res.append(pos_res)
                quiet_vel_res.append(vel_res)
                quiet_tq_res.append(tq_res)
                by_motor[int(mid)]["pos"].append(pos_res)
                by_motor[int(mid)]["vel"].append(vel_res)
                by_motor[int(mid)]["tq"].append(tq_res)
                group_name = group_name_for_motor(int(mid))
                if group_name is not None:
                    by_group[group_name]["pos"].append(pos_res)
                    by_group[group_name]["vel"].append(vel_res)
                    by_group[group_name]["tq"].append(tq_res)

            by_motor[int(mid)]["lat"].append(fms)
            group_name = group_name_for_motor(int(mid))
            if group_name is not None:
                by_group[group_name]["lat"].append(fms)

            if c.size >= max(50, 2 * win):
                c0 = c - moving_average(c, win)
                p0 = p - moving_average(p, win)
                if np.std(c0) > 1e-3 and np.std(p0) > 1e-3:
                    max_lag = min(c0.size // 4, int(round(0.25 / dt)))
                    best_corr = -np.inf
                    best_k = None
                    for k in range(0, max_lag + 1):
                        x = c0[:-k] if k > 0 else c0
                        y = p0[k:] if k > 0 else p0
                        if x.size < 20:
                            continue
                        sx = np.std(x)
                        sy = np.std(y)
                        if sx < 1e-9 or sy < 1e-9:
                            continue
                        corr = float(np.corrcoef(x, y)[0, 1])
                        if np.isfinite(corr) and corr > best_corr:
                            best_corr = corr
                            best_k = k
                    if best_k is not None:
                        lag_results.append(
                            {
                                "file": path,
                                "motor_id": int(mid),
                                "lag_ms": best_k * dt * 1e3,
                                "corr": best_corr,
                            }
                        )

    quiet_pos_res = (
        concat_arrays(quiet_pos_res)
    )
    quiet_vel_res = (
        concat_arrays(quiet_vel_res)
    )
    quiet_tq_res = (
        concat_arrays(quiet_tq_res)
    )
    all_feedback_minus_command_ms = (
        concat_arrays(all_feedback_minus_command_ms)
    )

    reliable_lags = [
        item for item in lag_results if item["corr"] >= args.lag_corr_threshold
    ]
    lag_ms = (
        np.array([item["lag_ms"] for item in reliable_lags], dtype=np.float64)
        if reliable_lags
        else np.array([], dtype=np.float64)
    )

    lag_by_group = defaultdict(list)
    for item in reliable_lags:
        group_name = group_name_for_motor(item["motor_id"])
        if group_name is not None:
            lag_by_group[group_name].append(item["lag_ms"])

    lines: list[str] = []
    lines.append(f"Dataset root: {args.dataset_root}")
    lines.append(f"Runs analyzed: {len(npz_files)}")
    lines.append(
        "Quiet samples: command rate < "
        f"{args.quiet_command_rate_rads} rad/s and |measured velocity| < "
        f"{args.quiet_velocity_rads} rad/s"
    )
    lines.append(f"Trend window: {args.trend_window_s} s")
    lines.append(
        "Motor groups: "
        + ", ".join(
            f"{group_name}={motor_ids[0]}-{motor_ids[1]}"
            for group_name, motor_ids in MOTOR_GROUPS.items()
        )
    )
    lines.append("")

    append_summary(lines, "Overall position noise residual", summarize(quiet_pos_res), "rad")
    append_summary(lines, "Overall velocity noise residual", summarize(quiet_vel_res), "rad/s")
    append_summary(lines, "Overall torque noise residual", summarize(quiet_tq_res), "Nm")
    append_summary(
        lines,
        "Overall feedback minus command timestamp",
        summarize(all_feedback_minus_command_ms),
        "ms",
    )
    lines.append("")
    lines.append(
        f"Reliable actuation-lag fits: {len(reliable_lags)} / {len(lag_results)} "
        f"(corr >= {args.lag_corr_threshold})"
    )
    append_summary(lines, "Overall estimated actuation lag", summarize(lag_ms), "ms")
    lines.append("")
    lines.append("Group summaries")
    for group_name, motor_ids in MOTOR_GROUPS.items():
        lines.append(f"{group_name} motors={motor_ids[0]},{motor_ids[1]}")
        pos = concat_arrays(by_group[group_name]["pos"])
        vel = concat_arrays(by_group[group_name]["vel"])
        tq = concat_arrays(by_group[group_name]["tq"])
        lat = concat_arrays(by_group[group_name]["lat"])
        group_lag = np.array(lag_by_group[group_name], dtype=np.float64)
        append_summary(lines, "  Position noise residual", summarize(pos), "rad")
        append_summary(lines, "  Velocity noise residual", summarize(vel), "rad/s")
        append_summary(lines, "  Torque noise residual", summarize(tq), "Nm")
        append_summary(lines, "  Feedback minus command timestamp", summarize(lat), "ms")
        append_summary(lines, "  Estimated actuation lag", summarize(group_lag), "ms")
        lines.append("")

    lines.append("Per-group p95(|residual|) and p95 latency")
    lines.append("Group      Motors  Pos(rad)   Vel(rad/s)  Tq(Nm)    Fb-Cmd(ms)  ActLag(ms)")
    for group_name, motor_ids in MOTOR_GROUPS.items():
        pos = concat_arrays(by_group[group_name]["pos"])
        vel = concat_arrays(by_group[group_name]["vel"])
        tq = concat_arrays(by_group[group_name]["tq"])
        lat = concat_arrays(by_group[group_name]["lat"])
        group_lag = np.array(lag_by_group[group_name], dtype=np.float64)
        lat_p95 = float(np.percentile(lat, 95)) if lat.size else float("nan")
        lag_p95 = float(np.percentile(group_lag, 95)) if group_lag.size else float("nan")
        lines.append(
            f"{group_name:<10} {motor_ids[0]:>2},{motor_ids[1]:<2} "
            f"{format_float(p95_abs(pos)):>10} {format_float(p95_abs(vel)):>11} "
            f"{format_float(p95_abs(tq)):>9} {format_float(lat_p95):>12} "
            f"{format_float(lag_p95):>11}"
        )

    lines.append("")
    lines.append("Per-motor p95(|residual|) and timestamp latency p95")
    lines.append("ID  Pos(rad)   Vel(rad/s)  Tq(Nm)    Fb-Cmd(ms)")
    for mid in sorted(by_motor):
        pos = concat_arrays(by_motor[mid]["pos"])
        vel = concat_arrays(by_motor[mid]["vel"])
        tq = concat_arrays(by_motor[mid]["tq"])
        lat = concat_arrays(by_motor[mid]["lat"])
        lat_p95 = float(np.percentile(lat, 95)) if lat.size else float("nan")
        lines.append(
            f"{mid:<3} {p95_abs(pos):>9.6f} {p95_abs(vel):>11.6f} "
            f"{p95_abs(tq):>9.6f} {lat_p95:>12.6f}"
        )

    report_text = "\n".join(lines) + "\n"
    print(report_text, end="")
    args.report_out.write_text(report_text, encoding="utf-8")
    print(f"Report written to: {args.report_out}")


if __name__ == "__main__":
    main()
