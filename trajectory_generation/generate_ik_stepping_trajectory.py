#!/usr/bin/env python3
"""
Generate IK-based stepping joint trajectories from a URDF (no simulator required).
- Parses your URDF joint origins/axes/limits
- Builds per-leg kinematic chains
- Solves position IK (damped least squares) each timestep
- Exports a CSV of joint position commands at fixed rate (default 400 Hz)

Target joint names (from your URDF):
  left_hip1_joint, left_hip2_joint, left_thigh_joint, left_knee_joint, left_ankle_joint
  right_hip1_joint, right_hip2_joint, right_thigh_joint, right_knee_joint, right_ankle_joint
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import xml.etree.ElementTree as ET


# ---------------------------
# Math helpers
# ---------------------------

def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]], dtype=float)

def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]], dtype=float)

def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]], dtype=float)

def rpy_to_R(rpy: np.ndarray) -> np.ndarray:
    # URDF fixed-axis RPY convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    roll, pitch, yaw = rpy
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

def axis_angle_to_R(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    x, y, z = axis / n
    c = math.cos(angle)
    s = math.sin(angle)
    C = 1 - c
    return np.array([
        [c + x*x*C,     x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s,   c + y*y*C,   y*z*C - x*s],
        [z*x*C - y*s,   z*y*C + x*s, c + z*z*C  ],
    ], dtype=float)

def make_T(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3,  3] = p
    return T


# ---------------------------
# URDF parsing
# ---------------------------

@dataclass
class JointInfo:
    name: str
    origin_xyz: np.ndarray   # parent->joint translation
    origin_rpy: np.ndarray   # parent->joint rotation (rpy)
    axis: np.ndarray         # joint axis in joint frame
    lower: float
    upper: float

def parse_xyz(text: str | None, default=(0.0, 0.0, 0.0)) -> np.ndarray:
    if text is None:
        return np.array(default, dtype=float)
    vals = [float(v) for v in text.strip().split()]
    if len(vals) != 3:
        raise ValueError(f"Expected 3 values, got {vals}")
    return np.array(vals, dtype=float)

def parse_urdf_joints(urdf_path: Path) -> Dict[str, JointInfo]:
    tree = ET.parse(str(urdf_path))
    root = tree.getroot()
    out: Dict[str, JointInfo] = {}

    for j in root.findall("joint"):
        jname = j.attrib["name"]

        origin = j.find("origin")
        xyz = parse_xyz(origin.attrib.get("xyz") if origin is not None else None)
        rpy = parse_xyz(origin.attrib.get("rpy") if origin is not None else None)

        axis_el = j.find("axis")
        axis = parse_xyz(axis_el.attrib.get("xyz") if axis_el is not None else None, default=(1.0, 0.0, 0.0))

        limit_el = j.find("limit")
        if limit_el is not None and "lower" in limit_el.attrib and "upper" in limit_el.attrib:
            lower = float(limit_el.attrib["lower"])
            upper = float(limit_el.attrib["upper"])
        else:
            # fallback for joints without explicit limits
            lower, upper = -1e9, 1e9

        out[jname] = JointInfo(
            name=jname,
            origin_xyz=xyz,
            origin_rpy=rpy,
            axis=axis,
            lower=lower,
            upper=upper,
        )
    return out


# ---------------------------
# Leg kinematics + IK
# ---------------------------

class LegChain:
    def __init__(
        self,
        joint_infos: List[JointInfo],
        foot_offset_in_foot: np.ndarray | None = None,
    ):
        self.joints = joint_infos
        self.n = len(self.joints)
        self.lower = np.array([j.lower for j in self.joints], dtype=float)
        self.upper = np.array([j.upper for j in self.joints], dtype=float)
        self.foot_offset = np.zeros(3) if foot_offset_in_foot is None else np.asarray(foot_offset_in_foot, dtype=float)

    def fk(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Foot point pose in hip frame.
        Returns (position, rotation).
        """
        assert q.shape == (self.n,)
        T = np.eye(4, dtype=float)

        for i, j in enumerate(self.joints):
            R0 = rpy_to_R(j.origin_rpy)
            T0 = make_T(R0, j.origin_xyz)
            Rj = axis_angle_to_R(j.axis, float(q[i]))
            Tj = make_T(Rj, np.zeros(3))
            T = T @ T0 @ Tj

        p = T[:3, 3] + T[:3, :3] @ self.foot_offset
        R = T[:3, :3]
        return p, R

    def jacobian_pos_numeric(self, q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
        J = np.zeros((3, self.n), dtype=float)
        for i in range(self.n):
            qp = q.copy()
            qm = q.copy()
            qp[i] = np.clip(qp[i] + eps, self.lower[i], self.upper[i])
            qm[i] = np.clip(qm[i] - eps, self.lower[i], self.upper[i])
            pp, _ = self.fk(qp)
            pm, _ = self.fk(qm)
            J[:, i] = (pp - pm) / (2 * eps)
        return J

    def solve_ik_position(
        self,
        q_init: np.ndarray,
        p_target: np.ndarray,
        q_nominal: np.ndarray | None = None,
        max_iters: int = 30,
        tol: float = 1e-4,
        damping: float = 2e-2,
        alpha: float = 0.6,
        posture_gain: float = 0.02,
    ) -> Tuple[np.ndarray, bool, float, int]:
        """
        Damped least-squares IK with optional nullspace posture term.
        """
        q = np.clip(q_init.copy(), self.lower, self.upper)
        if q_nominal is None:
            q_nominal = q.copy()

        I3 = np.eye(3)
        In = np.eye(self.n)

        for it in range(max_iters):
            p, _ = self.fk(q)
            e = p_target - p
            err = float(np.linalg.norm(e))
            if err < tol:
                return q, True, err, it + 1

            J = self.jacobian_pos_numeric(q)
            A = J @ J.T + (damping ** 2) * I3
            # J_pinv = J^T (J J^T + λ²I)^-1
            J_pinv = J.T @ np.linalg.solve(A, I3)

            dq_task = J_pinv @ e
            dq_post = (In - J_pinv @ J) @ (q_nominal - q)

            q = q + alpha * (dq_task + posture_gain * dq_post)
            q = np.clip(q, self.lower, self.upper)

        p, _ = self.fk(q)
        err = float(np.linalg.norm(p_target - p))
        return q, False, err, max_iters


# ---------------------------
# Trajectory generator
# ---------------------------

def smooth_ramp(t: float, T: float, ramp_time: float) -> float:
    if ramp_time <= 1e-9:
        return 1.0
    up = min(1.0, max(0.0, t / ramp_time))
    down = min(1.0, max(0.0, (T - t) / ramp_time))
    return min(up, down)

def foot_target_periodic(
    t: float,
    p0: np.ndarray,
    f_hz: float,
    ax: float,
    az: float,
    gamma: float,
    phase: float,
    z_mode_offset: float,
    duration: float,
    ramp_time: float,
    forward_axis: str = "x",
) -> np.ndarray:
    theta = 2.0 * math.pi * f_hz * t + phase
    ramp = smooth_ramp(t, duration, ramp_time)

    s = ramp * ax * math.sin(theta)
    if forward_axis == "x":
        x = p0[0] + s
        y = p0[1]
    elif forward_axis == "y":
        x = p0[0]
        y = p0[1] + s
    else:
        raise ValueError(f"Unsupported forward_axis={forward_axis!r}; expected 'x' or 'y'.")

    lift = max(0.0, math.sin(theta))
    z = p0[2] + z_mode_offset + ramp * az * (lift ** gamma)
    return np.array([x, y, z], dtype=float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", type=str, required=True, help="Path to URDF")
    ap.add_argument("--out_csv", type=str, default="ik_stepping_trajectory.csv")
    ap.add_argument("--out_meta", type=str, default="ik_stepping_meta.json")

    ap.add_argument("--hz", type=float, default=400.0)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--step_freq", type=float, default=0.6, help="Stepping frequency [Hz]")
    ap.add_argument("--ax", type=float, default=0.02, help="Fore-aft amplitude [m]")
    ap.add_argument(
        "--forward_axis",
        choices=["x", "y"],
        default="x",
        help="Hip-frame axis treated as forward for ax",
    )
    ap.add_argument("--az", type=float, default=0.015, help="Foot lift amplitude [m]")
    ap.add_argument("--gamma", type=float, default=1.8, help="Lift shaping exponent")
    ap.add_argument("--mode", choices=["contact", "air"], default="contact")
    ap.add_argument("--air_clearance", type=float, default=0.03, help="Extra z offset in air mode [m]")
    ap.add_argument("--ramp_time", type=float, default=2.0)

    # IK params
    ap.add_argument("--ik_iters", type=int, default=30)
    ap.add_argument("--ik_tol", type=float, default=1e-4)
    ap.add_argument("--ik_damping", type=float, default=2e-2)
    ap.add_argument("--ik_alpha", type=float, default=0.6)
    ap.add_argument("--ik_posture_gain", type=float, default=0.02)

    # Optional: target point offset in foot link frame
    ap.add_argument("--left_foot_offset", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    ap.add_argument("--right_foot_offset", type=float, nargs=3, default=[0.0, 0.0, 0.0])

    args = ap.parse_args()

    urdf_path = Path(args.urdf)
    if not urdf_path.exists():
        raise FileNotFoundError(urdf_path)

    joints = parse_urdf_joints(urdf_path)

    left_names = [
        "left_hip1_joint",
        "left_hip2_joint",
        "left_thigh_joint",
        "left_knee_joint",
        "left_ankle_joint",
    ]
    right_names = [
        "right_hip1_joint",
        "right_hip2_joint",
        "right_thigh_joint",
        "right_knee_joint",
        "right_ankle_joint",
    ]

    missing = [n for n in (left_names + right_names) if n not in joints]
    if missing:
        raise ValueError(f"Missing expected joints in URDF: {missing}")

    left_chain = LegChain(
        [joints[n] for n in left_names],
        foot_offset_in_foot=np.array(args.left_foot_offset, dtype=float),
    )
    right_chain = LegChain(
        [joints[n] for n in right_names],
        foot_offset_in_foot=np.array(args.right_foot_offset, dtype=float),
    )

    # initial q = user-defined standing pose (in joint-name order)
    standing_pose = {
        "left_hip1_joint": 0.3,
        "left_hip2_joint": 0.0,
        "left_thigh_joint": 0.0,
        "left_knee_joint": -0.8,
        "left_ankle_joint": 0.5,
        "right_hip1_joint": 0.3,
        "right_hip2_joint": 0.0,
        "right_thigh_joint": 0.0,
        "right_knee_joint": -0.8,
        "right_ankle_joint": 0.5,
    }

    qL = np.array([standing_pose[n] for n in left_names], dtype=float)
    qR = np.array([standing_pose[n] for n in right_names], dtype=float)

    # keep inside limits
    qL = np.clip(qL, left_chain.lower, left_chain.upper)
    qR = np.clip(qR, right_chain.lower, right_chain.upper)

    # fixed posture reference for nullspace bias
    qL_posture = qL.copy()
    qR_posture = qR.copy()

    # neutral foot points in hip frame
    pL0, _ = left_chain.fk(qL)
    pR0, _ = right_chain.fk(qR)

    z_mode_offset = args.air_clearance if args.mode == "air" else 0.0

    dt = 1.0 / args.hz
    steps = int(round(args.duration * args.hz))

    out_csv = Path(args.out_csv)
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    out_meta = Path(args.out_meta)
    out_meta.parent.mkdir(parents=True, exist_ok=True)

    columns = [
        "t",
        "pL_tx", "pL_ty", "pL_tz",
        "pR_tx", "pR_ty", "pR_tz",
        "ik_err_L", "ik_err_R",
        "ik_ok_L", "ik_ok_R",
        *left_names,
        *right_names,
    ]

    with out_csv.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(columns)

        for k in range(steps):
            t = k * dt

            # alternating steps: right phase shifted by pi
            pL_t = foot_target_periodic(
                t=t, p0=pL0, f_hz=args.step_freq, ax=args.ax, az=args.az,
                gamma=args.gamma, phase=0.0, z_mode_offset=z_mode_offset,
                duration=args.duration, ramp_time=args.ramp_time,
                forward_axis=args.forward_axis,
            )
            pR_t = foot_target_periodic(
                t=t, p0=pR0, f_hz=args.step_freq, ax=args.ax, az=args.az,
                gamma=args.gamma, phase=math.pi, z_mode_offset=z_mode_offset,
                duration=args.duration, ramp_time=args.ramp_time,
                forward_axis=args.forward_axis,
            )

            qL, okL, errL, _ = left_chain.solve_ik_position(
                q_init=qL,
                p_target=pL_t,
                q_nominal=qL_posture,
                max_iters=args.ik_iters,
                tol=args.ik_tol,
                damping=args.ik_damping,
                alpha=args.ik_alpha,
                posture_gain=args.ik_posture_gain,
            )
            qR, okR, errR, _ = right_chain.solve_ik_position(
                q_init=qR,
                p_target=pR_t,
                q_nominal=qR_posture,
                max_iters=args.ik_iters,
                tol=args.ik_tol,
                damping=args.ik_damping,
                alpha=args.ik_alpha,
                posture_gain=args.ik_posture_gain,
            )

            writer.writerow([
                f"{t:.6f}",
                f"{pL_t[0]:.9f}", f"{pL_t[1]:.9f}", f"{pL_t[2]:.9f}",
                f"{pR_t[0]:.9f}", f"{pR_t[1]:.9f}", f"{pR_t[2]:.9f}",
                f"{errL:.9e}", f"{errR:.9e}",
                int(okL), int(okR),
                *[f"{v:.9f}" for v in qL.tolist()],
                *[f"{v:.9f}" for v in qR.tolist()],
            ])

    meta = {
        "urdf": str(urdf_path),
        "mode": args.mode,
        "hz": args.hz,
        "duration": args.duration,
        "dt": dt,
        "step_freq": args.step_freq,
        "ax": args.ax,
        "forward_axis": args.forward_axis,
        "az": args.az,
        "gamma": args.gamma,
        "air_clearance": args.air_clearance,
        "ramp_time": args.ramp_time,
        "ik": {
            "iters": args.ik_iters,
            "tol": args.ik_tol,
            "damping": args.ik_damping,
            "alpha": args.ik_alpha,
            "posture_gain": args.ik_posture_gain,
        },
        "left_joint_order": left_names,
        "right_joint_order": right_names,
        "left_neutral_foot": pL0.tolist(),
        "right_neutral_foot": pR0.tolist(),
        "left_foot_offset": args.left_foot_offset,
        "right_foot_offset": args.right_foot_offset,
    }
    out_meta.write_text(json.dumps(meta, indent=2))

    print(f"[OK] Wrote trajectory CSV: {out_csv}")
    print(f"[OK] Wrote metadata JSON: {out_meta}")
    print(f"Neutral foot positions (hip frame):")
    print(f"  Left : {pL0}")
    print(f"  Right: {pR0}")


if __name__ == "__main__":
    main()
