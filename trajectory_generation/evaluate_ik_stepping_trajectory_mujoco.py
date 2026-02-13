#!/usr/bin/env python3
"""
Replay a generated IK stepping trajectory in MuJoCo and report tracking metrics.

Example:
  python evaluate_ik_stepping_trajectory_mujoco.py \
    --model scene.xml \
    --traj_csv traj_contact.csv \
    --out_csv traj_contact_eval.csv \
    --out_json traj_contact_eval_summary.json

Visualize only (no logs):
  python evaluate_ik_stepping_trajectory_mujoco.py \
    --model scene.xml \
    --traj_csv traj_contact.csv \
    --render --realtime --no_log
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path
from typing import List, Tuple

import mujoco
import numpy as np


DEFAULT_JOINT_ORDER = [
    "left_hip1_joint",
    "left_hip2_joint",
    "left_thigh_joint",
    "left_knee_joint",
    "left_ankle_joint",
    "right_hip1_joint",
    "right_hip2_joint",
    "right_thigh_joint",
    "right_knee_joint",
    "right_ankle_joint",
]


def load_trajectory_csv(path: Path, joint_order: List[str]) -> Tuple[np.ndarray, np.ndarray]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        raise ValueError(f"Trajectory CSV is empty: {path}")

    missing = [j for j in joint_order if j not in rows[0]]
    if missing:
        raise ValueError(f"CSV missing required joint columns: {missing}")
    if "t" not in rows[0]:
        raise ValueError("CSV missing required 't' column")

    t = np.array([float(r["t"]) for r in rows], dtype=float)
    q = np.array([[float(r[j]) for j in joint_order] for r in rows], dtype=float)

    if len(t) > 1 and np.any(np.diff(t) <= 0.0):
        raise ValueError("Trajectory time column must be strictly increasing")
    return t, q


def actuator_for_joint(model: mujoco.MjModel, joint_name: str) -> int:
    act_name = joint_name.replace("_joint", "_act")
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
    if act_id >= 0:
        return int(act_id)

    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise ValueError(f"Joint not found in model: {joint_name}")

    for i in range(model.nu):
        if model.actuator_trnid[i, 0] == joint_id:
            return i
    raise ValueError(f"No actuator found driving joint: {joint_name}")


def has_contact_pair(data: mujoco.MjData, geom_a: int, geom_b: int) -> bool:
    for i in range(data.ncon):
        c = data.contact[i]
        if (c.geom1 == geom_a and c.geom2 == geom_b) or (c.geom1 == geom_b and c.geom2 == geom_a):
            return True
    return False


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", type=str, default="scene.xml", help="MuJoCo XML model path")
    ap.add_argument("--traj_csv", type=str, required=True, help="CSV from generate_ik_stepping_trajectory.py")
    ap.add_argument("--traj_meta", type=str, default=None, help="Optional trajectory metadata JSON")
    ap.add_argument("--out_csv", type=str, default="traj_eval_mujoco.csv", help="Per-sample eval CSV")
    ap.add_argument("--out_json", type=str, default="traj_eval_mujoco_summary.json", help="Summary JSON")
    ap.add_argument("--start_keyframe", type=str, default="locomotion_standing_pose")
    ap.add_argument("--imu_site", type=str, default="imu", help="Site used for stability height metric")
    ap.add_argument("--fall_imu_z", type=float, default=0.20, help="Fall threshold on imu z [m]")
    ap.add_argument("--render", action="store_true", help="Show MuJoCo viewer during replay")
    ap.add_argument("--realtime", action="store_true", help="When rendering, run at wall-clock real-time")
    ap.add_argument(
        "--no_log",
        action="store_true",
        help="Replay only; skip writing eval CSV/JSON and metric summaries",
    )
    args = ap.parse_args()

    model_path = Path(args.model)
    traj_path = Path(args.traj_csv)
    out_csv = Path(args.out_csv)
    out_json = Path(args.out_json)

    if not model_path.exists():
        raise FileNotFoundError(model_path)
    if not traj_path.exists():
        raise FileNotFoundError(traj_path)

    joint_order = DEFAULT_JOINT_ORDER.copy()
    if args.traj_meta:
        meta_path = Path(args.traj_meta)
        if not meta_path.exists():
            raise FileNotFoundError(meta_path)
        meta = json.loads(meta_path.read_text())
        if "left_joint_order" in meta and "right_joint_order" in meta:
            joint_order = list(meta["left_joint_order"]) + list(meta["right_joint_order"])

    traj_t, traj_q = load_trajectory_csv(traj_path, joint_order)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    if args.start_keyframe:
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, args.start_keyframe)
        if key_id < 0:
            raise ValueError(f"Keyframe not found: {args.start_keyframe}")
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    else:
        mujoco.mj_resetData(model, data)

    qpos_adrs = []
    act_ids = []
    for jn in joint_order:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid < 0:
            raise ValueError(f"Joint in trajectory not present in model: {jn}")
        qpos_adrs.append(int(model.jnt_qposadr[jid]))
        act_ids.append(int(actuator_for_joint(model, jn)))

    qpos_adrs = np.array(qpos_adrs, dtype=int)
    act_ids = np.array(act_ids, dtype=int)

    # Start close to first trajectory sample to avoid large initial transients.
    data.qpos[qpos_adrs] = traj_q[0]
    data.ctrl[act_ids] = traj_q[0]
    mujoco.mj_forward(model, data)

    imu_site_id = -1
    if args.imu_site:
        imu_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, args.imu_site)
        if imu_site_id < 0:
            raise ValueError(f"IMU site not found: {args.imu_site}")

    collect_metrics = not args.no_log

    floor_id = -1
    left_foot_geom = -1
    right_foot_geom = -1
    if collect_metrics:
        floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        left_foot_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_foot_collision_box")
        right_foot_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_foot_collision_box")
        if floor_id < 0 or left_foot_geom < 0 or right_foot_geom < 0:
            raise ValueError("Expected geoms not found: floor/left_foot_collision_box/right_foot_collision_box")

    if len(traj_t) > 1:
        dt_traj = float(np.median(np.diff(traj_t)))
    else:
        dt_traj = 1.0 / 400.0
    dt_sim = float(model.opt.timestep)

    if collect_metrics:
        out_csv.parent.mkdir(parents=True, exist_ok=True)
        out_json.parent.mkdir(parents=True, exist_ok=True)

    headers = []
    if collect_metrics:
        headers = [
            "sample",
            "t_cmd",
            "sim_time",
            "imu_z",
            "left_contact",
            "right_contact",
            "rmse_step",
            "max_abs_step",
        ]
        for jn in joint_order:
            headers += [f"{jn}_target", f"{jn}_actual", f"{jn}_err"]

    records = []
    start_wall = time.perf_counter()

    viewer = None
    if args.render:
        import mujoco.viewer as mj_viewer

        viewer = mj_viewer.launch_passive(model, data)
        viewer.sync()

    for i in range(len(traj_t)):
        q_target = traj_q[i]
        data.ctrl[act_ids] = q_target

        if i < len(traj_t) - 1:
            t_end = float(traj_t[i + 1])
        else:
            t_end = float(traj_t[i] + dt_traj)

        while data.time + 1e-12 < t_end:
            sim_time_before = data.time
            mujoco.mj_step(model, data)

            if viewer is not None:
                viewer.sync()
                if args.realtime:
                    elapsed_sim = data.time - sim_time_before
                    elapsed_wall = time.perf_counter() - start_wall
                    lag = data.time - elapsed_wall
                    if lag > 0.0:
                        time.sleep(min(lag, elapsed_sim))

        if collect_metrics:
            q_actual = data.qpos[qpos_adrs].copy()
            q_err = q_actual - q_target
            rmse_step = float(np.sqrt(np.mean(np.square(q_err))))
            max_abs_step = float(np.max(np.abs(q_err)))
            imu_z = float(data.site_xpos[imu_site_id, 2]) if imu_site_id >= 0 else float("nan")
            left_contact = int(has_contact_pair(data, left_foot_geom, floor_id))
            right_contact = int(has_contact_pair(data, right_foot_geom, floor_id))

            row = [
                i,
                f"{traj_t[i]:.6f}",
                f"{data.time:.6f}",
                f"{imu_z:.6f}",
                left_contact,
                right_contact,
                f"{rmse_step:.9e}",
                f"{max_abs_step:.9e}",
            ]
            for j in range(len(joint_order)):
                row += [
                    f"{q_target[j]:.9f}",
                    f"{q_actual[j]:.9f}",
                    f"{q_err[j]:.9f}",
                ]
            records.append(row)

    if viewer is not None:
        viewer.close()

    if not collect_metrics:
        print("[OK] Replay complete (no logs written).")
        return

    with out_csv.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(headers)
        w.writerows(records)

    # Reconstruct arrays for summary stats.
    err_mat = np.array(
        [[float(records[i][8 + 3 * j + 2]) for j in range(len(joint_order))] for i in range(len(records))],
        dtype=float,
    )
    imu_z_vec = np.array([float(r[3]) for r in records], dtype=float)
    left_contact_vec = np.array([int(r[4]) for r in records], dtype=int)
    right_contact_vec = np.array([int(r[5]) for r in records], dtype=int)

    joint_rmse = np.sqrt(np.mean(np.square(err_mat), axis=0))
    joint_max_abs = np.max(np.abs(err_mat), axis=0)

    summary = {
        "model": str(model_path),
        "traj_csv": str(traj_path),
        "num_samples": int(len(records)),
        "traj_dt_nominal": dt_traj,
        "sim_dt": dt_sim,
        "sim_final_time": float(data.time),
        "joint_order": joint_order,
        "overall_rmse_rad": float(math.sqrt(float(np.mean(np.square(err_mat))))),
        "overall_max_abs_rad": float(np.max(np.abs(err_mat))),
        "joint_rmse_rad": {joint_order[i]: float(joint_rmse[i]) for i in range(len(joint_order))},
        "joint_max_abs_rad": {joint_order[i]: float(joint_max_abs[i]) for i in range(len(joint_order))},
        "imu_site": args.imu_site,
        "imu_z_min": float(np.min(imu_z_vec)),
        "imu_z_max": float(np.max(imu_z_vec)),
        "imu_z_mean": float(np.mean(imu_z_vec)),
        "fall_imu_z_threshold": float(args.fall_imu_z),
        "fell": bool(np.any(imu_z_vec < args.fall_imu_z)),
        "left_contact_ratio": float(np.mean(left_contact_vec)),
        "right_contact_ratio": float(np.mean(right_contact_vec)),
    }
    out_json.write_text(json.dumps(summary, indent=2))

    print(f"[OK] Wrote eval CSV: {out_csv}")
    print(f"[OK] Wrote summary JSON: {out_json}")
    print(f"[INFO] overall RMSE [rad]: {summary['overall_rmse_rad']:.6f}")
    print(f"[INFO] overall max abs err [rad]: {summary['overall_max_abs_rad']:.6f}")
    print(f"[INFO] imu z min/max [m]: {summary['imu_z_min']:.4f}/{summary['imu_z_max']:.4f}")
    print(f"[INFO] fell (imu_z < {args.fall_imu_z:.3f}): {summary['fell']}")


if __name__ == "__main__":
    main()
