#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motor dataset collection for RobStride humanoid joints.

Implements campaign:
  - single_air/{clean,disturbed}: sine + chirp + random per motor
  - single_contact/{clean,disturbed}: sine + chirp + random per motor
  - multi_air/{clean,disturbed}: sine-only multi-motor
  - multi_contact/{clean,disturbed}: sine-only multi-motor
  - ik_contact/clean: replay IK trajectory across all selected motors

Logs at target 400 Hz (best-effort):
  commanded position, actual position, velocity, torque, temperature,
  commanded timestamp, actuator feedback receive timestamp, cycle index.

Notes:
  - External disturbances are manual (you apply pushes/holds during disturbed runs).
  - Non-active motors are held at their current position before each run.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import struct
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

# -------------------- RobStride SDK imports --------------------
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from robstride_dynamics import RobstrideBus, Motor, ParameterType, CommunicationType
except ImportError:
    try:
        from bus import RobstrideBus, Motor
        from protocol import ParameterType, CommunicationType
    except ImportError as e:
        raise SystemExit(f"Failed to import RobStride SDK: {e}")


# -------------------- User config copied from your script --------------------
INVERSION_ARRAY = [-1, 1, 1, 1, 1, 1, -1, -1, -1, -1]
INVERSION_BY_ID: Dict[int, int] = {i + 1: INVERSION_ARRAY[i] for i in range(len(INVERSION_ARRAY))}

JOINT_LIMITS: Dict[int, Tuple[float, float]] = {
    1: (-1.57, 1.57),
    2: (-1.57, 0.436332),
    3: (-0.785398, 0.785398),
    4: (-2.0944, 0.0),
    5: (-0.6, 0.6),
    6: (-1.57, 1.57),
    7: (-0.436332, 1.57),
    8: (-0.785398, 0.785398),
    9: (-2.0944, 0.0),
    10: (-0.6, 0.6),
}

MOTOR_MODEL_BY_ID: Dict[int, str] = {
    1: "rs-04",
    2: "rs-03",
    3: "rs-03",
    4: "rs-04",
    5: "rs-02",
    6: "rs-04",
    7: "rs-03",
    8: "rs-03",
    9: "rs-04",
    10: "rs-02",
}

TEMP_ABORT_C = 88.0
MAX_RANDOM_STEP_RAD = math.radians(20.0)  # <= 20 deg between random commanded positions


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def iso_now() -> str:
    return datetime.now().isoformat(timespec="seconds")


def parse_id_list(text: str) -> List[int]:
    vals = [int(x) for x in text.split() if x.strip()]
    if len(vals) == 0:
        raise ValueError("empty ID list")
    if len(set(vals)) != len(vals):
        raise ValueError("duplicate IDs")
    return vals


@dataclass
class MotorState:
    mid: int
    name: str
    model: str
    direction: int
    lim_lo: float
    lim_hi: float
    kp: float
    kd: float
    cmd_logical: float = 0.0
    pos: float = 0.0
    vel: float = 0.0
    tq: float = 0.0
    temp: float = 0.0


class RobstrideController:
    def __init__(
        self,
        motor_ids: List[int],
        channel: str,
        bitrate: int,
        kp: float,
        kd: float,
    ):
        self.channel = channel
        self.bitrate = bitrate
        self.motor_ids = sorted(motor_ids)

        self.states: Dict[int, MotorState] = {}
        for mid in self.motor_ids:
            lo, hi = JOINT_LIMITS.get(mid, (-math.inf, math.inf))
            st = MotorState(
                mid=mid,
                name=f"motor_{mid}",
                model=MOTOR_MODEL_BY_ID.get(mid, "rs-03"),
                direction=int(INVERSION_BY_ID.get(mid, 1)),
                lim_lo=lo,
                lim_hi=hi,
                kp=float(kp),
                kd=float(kd),
            )
            self.states[mid] = st

        self.bus: Optional[RobstrideBus] = None
        self.connected = False

    def _set_mode_raw(self, mid: int, mode: int = 0):
        assert self.bus is not None
        dev_id = self.bus.motors[f"motor_{mid}"].id
        param_id, _, _ = ParameterType.MODE
        value_buffer = struct.pack("<bBH", int(mode), 0, 0)
        data = struct.pack("<HH", param_id, 0x00) + value_buffer
        self.bus.transmit(CommunicationType.WRITE_PARAMETER, self.bus.host_id, dev_id, data)
        time.sleep(0.05)

    def connect(self):
        motors = {f"motor_{mid}": Motor(id=mid, model=self.states[mid].model) for mid in self.motor_ids}
        calib = {f"motor_{mid}": {"direction": 1, "homing_offset": 0.0} for mid in self.motor_ids}
        try:
            try:
                self.bus = RobstrideBus(self.channel, motors, calib, bitrate=self.bitrate)
            except TypeError:
                self.bus = RobstrideBus(self.channel, motors, calib)
            self.bus.connect(handshake=True)

            for mid in self.motor_ids:
                st = self.states[mid]
                self.bus.enable(st.name)
                time.sleep(0.2)
                self._set_mode_raw(mid, 0)

                pos, vel, tq, temp = self.bus.read_operation_frame(st.name)
                st.pos, st.vel, st.tq, st.temp = pos, vel, tq, temp
                logical = pos / float(st.direction)
                st.cmd_logical = logical

                # initial hold
                self.bus.write_operation_frame(st.name, logical * st.direction, st.kp, st.kd, 0.0, 0.0)
                time.sleep(0.01)

            self.connected = True
        except Exception as e:
            raise RuntimeError(f"connect failed: {e}")

    def read(self, mid: int) -> Tuple[float, float, float, float]:
        assert self.bus is not None
        st = self.states[mid]
        pos, vel, tq, temp = self.bus.read_operation_frame(st.name)
        st.pos, st.vel, st.tq, st.temp = pos, vel, tq, temp
        return pos, vel, tq, temp

    def write_logical(self, mid: int, logical_target: float):
        assert self.bus is not None
        st = self.states[mid]
        t = clamp(logical_target, st.lim_lo, st.lim_hi)
        st.cmd_logical = t
        phys = t * float(st.direction)
        self.bus.write_operation_frame(st.name, phys, st.kp, st.kd, 0.0, 0.0)

    def read_all_once(self):
        for mid in self.motor_ids:
            try:
                self.read(mid)
            except Exception:
                pass

    def hold_positions(self, mids: Optional[List[int]] = None):
        ids = self.motor_ids if mids is None else sorted(mids)
        self.read_all_once()
        for mid in ids:
            st = self.states[mid]
            logical = st.pos / float(st.direction)
            st.cmd_logical = clamp(logical, st.lim_lo, st.lim_hi)
            self.write_logical(mid, st.cmd_logical)

    def move_to_targets(self, targets: Dict[int, float], duration_s: float = 1.0, hz: float = 200.0):
        """Smoothly interpolate logical commands to targets (no logging)."""
        steps = max(1, int(duration_s * hz))
        self.read_all_once()
        start = {mid: self.states[mid].cmd_logical for mid in targets.keys()}
        for i in range(steps):
            a = (i + 1) / steps
            for mid, tgt in targets.items():
                cmd = (1.0 - a) * start[mid] + a * tgt
                self.write_logical(mid, cmd)
            for mid in targets.keys():
                try:
                    self.read(mid)
                except Exception:
                    pass
            time.sleep(max(0.0, 1.0 / hz))

    def max_temp(self, mids: List[int]) -> float:
        vals = [self.states[mid].temp for mid in mids]
        return max(vals) if vals else 0.0

    def shutdown(self):
        if not self.connected or self.bus is None:
            return
        try:
            self.hold_positions()
            time.sleep(0.2)
            for mid in self.motor_ids:
                try:
                    self.bus.disable(self.states[mid].name)
                except Exception:
                    pass
            self.bus.disconnect()
        finally:
            self.connected = False


def scan_motor_ids_or_parse(text: Optional[str], channel: str) -> List[int]:
    if text and text.strip():
        return parse_id_list(text)

    print(f"Scanning {channel} for motor IDs ...")
    found = RobstrideBus.scan_channel(channel, start_id=1, end_id=255)
    if not found:
        raise RuntimeError("No motors found on CAN")
    ids = sorted(found.keys())
    print("Found IDs:", ids)
    return ids


@dataclass
class Trajectory:
    name: str
    traj_type: str
    duration_s: float
    params: Dict[str, Any]
    fn: Callable[[float, int], Dict[int, float]]  # fn(t, cycle_idx) -> {mid: cmd_logical}


class RunLogger:
    def __init__(self, run_dir: Path, n_rows_est: int):
        self.run_dir = run_dir
        self.n = max(1, n_rows_est)
        self.i = 0
        self.cycle_idx = np.empty(self.n, dtype=np.int64)
        self.motor_id = np.empty(self.n, dtype=np.int16)
        self.commanded_position_rad = np.empty(self.n, dtype=np.float64)
        self.actual_position_rad = np.empty(self.n, dtype=np.float64)
        self.motor_velocity_rads = np.empty(self.n, dtype=np.float64)
        self.motor_torque_nm = np.empty(self.n, dtype=np.float64)
        self.motor_temperature_c = np.empty(self.n, dtype=np.float64)
        self.commanded_timestamp_ns = np.empty(self.n, dtype=np.int64)
        self.feedback_timestamp_ns = np.empty(self.n, dtype=np.int64)

    def append(
        self,
        cycle_idx: int,
        motor_id: int,
        cmd: float,
        pos: float,
        vel: float,
        tq: float,
        temp: float,
        cmd_ts_ns: int,
        fb_ts_ns: int,
    ):
        if self.i >= self.n:
            self._grow()

        j = self.i
        self.cycle_idx[j] = int(cycle_idx)
        self.motor_id[j] = int(motor_id)
        self.commanded_position_rad[j] = float(cmd)
        self.actual_position_rad[j] = float(pos)
        self.motor_velocity_rads[j] = float(vel)
        self.motor_torque_nm[j] = float(tq)
        self.motor_temperature_c[j] = float(temp)
        self.commanded_timestamp_ns[j] = int(cmd_ts_ns)
        self.feedback_timestamp_ns[j] = int(fb_ts_ns)
        self.i += 1

    def _grow(self):
        new_n = self.n * 2
        def g(arr, dtype):
            out = np.empty(new_n, dtype=dtype)
            out[: self.n] = arr
            return out

        self.cycle_idx = g(self.cycle_idx, np.int64)
        self.motor_id = g(self.motor_id, np.int16)
        self.commanded_position_rad = g(self.commanded_position_rad, np.float64)
        self.actual_position_rad = g(self.actual_position_rad, np.float64)
        self.motor_velocity_rads = g(self.motor_velocity_rads, np.float64)
        self.motor_torque_nm = g(self.motor_torque_nm, np.float64)
        self.motor_temperature_c = g(self.motor_temperature_c, np.float64)
        self.commanded_timestamp_ns = g(self.commanded_timestamp_ns, np.int64)
        self.feedback_timestamp_ns = g(self.feedback_timestamp_ns, np.int64)
        self.n = new_n

    def save_npz(self):
        np.savez_compressed(
            self.run_dir / "log.npz",
            cycle_idx=self.cycle_idx[: self.i],
            motor_id=self.motor_id[: self.i],
            commanded_position_rad=self.commanded_position_rad[: self.i],
            actual_position_rad=self.actual_position_rad[: self.i],
            motor_velocity_rads=self.motor_velocity_rads[: self.i],
            motor_torque_nm=self.motor_torque_nm[: self.i],
            motor_temperature_c=self.motor_temperature_c[: self.i],
            commanded_timestamp_ns=self.commanded_timestamp_ns[: self.i],
            feedback_timestamp_ns=self.feedback_timestamp_ns[: self.i],
        )

    def save_csv(self):
        csv_path = self.run_dir / "log.csv"
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(
                [
                    "cycle_idx",
                    "motor_id",
                    "commanded_position_rad",
                    "actual_position_rad",
                    "motor_velocity_rads",
                    "motor_torque_nm",
                    "motor_temperature_c",
                    "commanded_timestamp_ns",
                    "feedback_timestamp_ns",
                ]
            )
            for j in range(self.i):
                w.writerow(
                    [
                        int(self.cycle_idx[j]),
                        int(self.motor_id[j]),
                        float(self.commanded_position_rad[j]),
                        float(self.actual_position_rad[j]),
                        float(self.motor_velocity_rads[j]),
                        float(self.motor_torque_nm[j]),
                        float(self.motor_temperature_c[j]),
                        int(self.commanded_timestamp_ns[j]),
                        int(self.feedback_timestamp_ns[j]),
                    ]
                )


# -------------------- Trajectory builders --------------------
def make_single_sine(mid: int, center: float, amp: float, freq_hz: float, duration_s: float, lim: Tuple[float, float]) -> Trajectory:
    lo, hi = lim
    amp = max(0.0, min(amp, hi - center, center - lo))

    def fn(t: float, _k: int) -> Dict[int, float]:
        return {mid: clamp(center + amp * math.sin(2.0 * math.pi * freq_hz * t), lo, hi)}

    return Trajectory(
        name=f"sine_amp{math.degrees(amp):.1f}deg_f{freq_hz:.2f}Hz",
        traj_type="sine",
        duration_s=duration_s,
        params={"amp_rad": amp, "amp_deg": math.degrees(amp), "freq_hz": freq_hz},
        fn=fn,
    )


def make_single_chirp(mid: int, center: float, amp: float, f0: float, f1: float, duration_s: float, lim: Tuple[float, float]) -> Trajectory:
    lo, hi = lim
    amp = max(0.0, min(amp, hi - center, center - lo))
    k = (f1 - f0) / max(1e-9, duration_s)

    def fn(t: float, _k: int) -> Dict[int, float]:
        tt = clamp(t, 0.0, duration_s)
        phase = 2.0 * math.pi * (f0 * tt + 0.5 * k * tt * tt)
        return {mid: clamp(center + amp * math.sin(phase), lo, hi)}

    return Trajectory(
        name=f"chirp_amp{math.degrees(amp):.1f}deg_{f0:.2f}to{f1:.2f}Hz",
        traj_type="chirp",
        duration_s=duration_s,
        params={"amp_rad": amp, "amp_deg": math.degrees(amp), "f0_hz": f0, "f1_hz": f1},
        fn=fn,
    )


def make_single_random(
    mid: int,
    center: float,
    lim: Tuple[float, float],
    duration_s: float,
    hz: float,
    max_step_rad: float,
    seed: int,
    keyframe_hz: float = 20.0,
) -> Trajectory:
    lo, hi = lim
    n = max(1, int(round(duration_s * hz)))
    seq = np.empty(n, dtype=np.float64)
    seq[0] = clamp(center, lo, hi)

    rng = np.random.default_rng(seed)
    key_every = max(1, int(round(hz / keyframe_hz)))

    for i in range(1, n):
        if i % key_every == 0:
            delta = float(rng.uniform(-max_step_rad, max_step_rad))
            nxt = clamp(seq[i - 1] + delta, lo, hi)
            if abs(nxt - seq[i - 1]) > max_step_rad:
                nxt = seq[i - 1] + math.copysign(max_step_rad, nxt - seq[i - 1])
            seq[i] = clamp(nxt, lo, hi)
        else:
            seq[i] = seq[i - 1]

    def fn(_t: float, k: int) -> Dict[int, float]:
        kk = min(max(0, k), n - 1)
        return {mid: float(seq[kk])}

    return Trajectory(
        name=f"random_steple20deg_seed{seed}",
        traj_type="random",
        duration_s=duration_s,
        params={
            "max_step_rad": max_step_rad,
            "max_step_deg": math.degrees(max_step_rad),
            "seed": seed,
            "keyframe_hz": keyframe_hz,
        },
        fn=fn,
    )


def make_multi_sine(
    active_ids: List[int],
    centers: Dict[int, float],
    limits: Dict[int, Tuple[float, float]],
    amp_frac: float,
    freq_hz: float,
    duration_s: float,
) -> Trajectory:
    amps: Dict[int, float] = {}
    phases: Dict[int, float] = {}

    n = max(1, len(active_ids))
    for i, mid in enumerate(active_ids):
        lo, hi = limits[mid]
        c = centers[mid]
        span = 0.5 * (hi - lo)
        amp = amp_frac * span
        amp = min(amp, hi - c, c - lo)
        amps[mid] = max(0.0, amp)
        phases[mid] = (2.0 * math.pi * i) / n

    def fn(t: float, _k: int) -> Dict[int, float]:
        out: Dict[int, float] = {}
        for mid in active_ids:
            lo, hi = limits[mid]
            c = centers[mid]
            a = amps[mid]
            p = phases[mid]
            out[mid] = clamp(c + a * math.sin(2.0 * math.pi * freq_hz * t + p), lo, hi)
        return out

    return Trajectory(
        name=f"multi_sine_ampfrac{amp_frac:.2f}_f{freq_hz:.2f}Hz",
        traj_type="sine",
        duration_s=duration_s,
        params={"amp_fraction_of_half_span": amp_frac, "freq_hz": freq_hz},
        fn=fn,
    )


def resolve_ik_path(raw: str) -> Path:
    p = Path(raw)
    if p.exists():
        return p
    for ext in [".npz", ".npy", ".csv"]:
        q = Path(raw + ext)
        if q.exists():
            return q
    raise FileNotFoundError(f"IK file not found: {raw} (or .npz/.npy/.csv)")


def load_ik_trajectory(path: Path, active_ids: List[int], fallback_hz: float = 400.0) -> Tuple[np.ndarray, np.ndarray]:
    """
    Returns:
      t: shape [N]
      q: shape [N, M] where M=len(active_ids)
    Supported:
      - npz with keys {'t','q'} or {'time_s','q'}
      - npy as q only (uniform dt at fallback_hz)
      - csv with first col optional time, remaining M columns commands
    """
    suf = path.suffix.lower()
    M = len(active_ids)

    if suf == ".npz":
        d = np.load(path, allow_pickle=False)
        if "q" not in d.files:
            raise ValueError("NPZ must contain array 'q'")
        q = np.asarray(d["q"], dtype=np.float64)
        if q.ndim != 2:
            raise ValueError("q must be 2D [N,M]")
        if "t" in d.files:
            t = np.asarray(d["t"], dtype=np.float64)
        elif "time_s" in d.files:
            t = np.asarray(d["time_s"], dtype=np.float64)
        else:
            t = np.arange(q.shape[0], dtype=np.float64) / float(fallback_hz)

    elif suf == ".npy":
        q = np.asarray(np.load(path), dtype=np.float64)
        if q.ndim != 2:
            raise ValueError("NPY must be 2D [N,M]")
        t = np.arange(q.shape[0], dtype=np.float64) / float(fallback_hz)

    elif suf == ".csv":
        arr = np.genfromtxt(path, delimiter=",", names=True)
        cols = arr.dtype.names
        if cols is None:
            raise ValueError("CSV parse failed")

        # time column detection
        time_keys = [k for k in cols if k.lower() in ("t", "time", "time_s", "timestamp", "timestamp_s")]
        if len(time_keys) > 0:
            tk = time_keys[0]
            t = np.asarray(arr[tk], dtype=np.float64)
            q_cols = [k for k in cols if k != tk]
        else:
            t = np.arange(arr.shape[0], dtype=np.float64) / float(fallback_hz)
            q_cols = list(cols)

        if len(q_cols) < M:
            raise ValueError(f"CSV has {len(q_cols)} command cols, expected at least {M}")

        q = np.stack([np.asarray(arr[k], dtype=np.float64) for k in q_cols[:M]], axis=1)
    else:
        raise ValueError(f"Unsupported IK format: {path}")

    if q.shape[1] != M:
        if q.shape[1] > M:
            q = q[:, :M]
        else:
            raise ValueError(f"IK q has {q.shape[1]} cols but expected {M}")

    if len(t) != q.shape[0]:
        raise ValueError("Time and q length mismatch")
    if q.shape[0] < 2:
        raise ValueError("IK trajectory needs at least 2 rows")

    return t, q


def make_ik_traj(active_ids: List[int], t: np.ndarray, q: np.ndarray) -> Trajectory:
    t0 = float(t[0])
    t1 = float(t[-1])
    duration = max(0.0, t1 - t0)

    def fn(tt: float, _k: int) -> Dict[int, float]:
        # clamp into trajectory span
        x = clamp(tt + t0, t0, t1)
        j = int(np.searchsorted(t, x, side="right") - 1)
        j = max(0, min(j, len(t) - 2))
        a = (x - t[j]) / max(1e-12, (t[j + 1] - t[j]))
        cmd_vec = (1.0 - a) * q[j] + a * q[j + 1]
        return {mid: float(cmd_vec[i]) for i, mid in enumerate(active_ids)}

    return Trajectory(
        name="ik_locomotion",
        traj_type="ik",
        duration_s=duration,
        params={"source": "IK_trajectory", "samples": int(len(t))},
        fn=fn,
    )


def ensure_dir(p: Path):
    p.mkdir(parents=True, exist_ok=True)


def countdown(msg: str, sec: int = 3):
    print(msg)
    for k in range(sec, 0, -1):
        print(f"  starting in {k}...")
        time.sleep(1.0)


def run_one(
    ctrl: RobstrideController,
    run_root: Path,
    family: str,
    pose: str,
    condition: str,
    active_ids: List[int],
    traj: Trajectory,
    hz: float,
    save_csv: bool,
    notes: str = "",
) -> Dict[str, Any]:
    """
    Executes one trajectory run and logs motor data.
    """
    # Make sure folder exists
    ensure_dir(run_root)
    ts_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    ids_tag = "-".join(str(i) for i in active_ids)
    run_dir = run_root / f"{ts_tag}_{traj.name}_m{ids_tag}"
    ensure_dir(run_dir)

    print("\n" + "=" * 90)
    print(f"Run: {run_dir}")
    print(f"family={family} pose={pose} condition={condition} active={active_ids} traj={traj.name}")

    if condition == "disturbed":
        input("Disturbed run: be ready to apply external pushes/holds during the run. Press Enter when ready...")
    else:
        input("Clean run: ensure no external disturbances. Press Enter when ready...")

    # Freeze non-active motors once at run start
    non_active = [m for m in ctrl.motor_ids if m not in set(active_ids)]
    ctrl.hold_positions(non_active)

    # center active motors first if single/multi sine/chirp/random (not IK)
    if traj.traj_type in ("sine", "chirp", "random"):
        targets = {}
        for mid in active_ids:
            lo, hi = ctrl.states[mid].lim_lo, ctrl.states[mid].lim_hi
            # Midpoint start keeps random first-sample jump bounded by MAX_RANDOM_STEP_RAD.
            targets[mid] = clamp(0.5 * (lo + hi), lo, hi)
        ctrl.move_to_targets(targets, duration_s=1.5, hz=200.0)

    # refresh centers used by trajectory fn
    ctrl.read_all_once()

    # logger preallocation: rows = cycles * active motors
    n_cycles = max(1, int(round(traj.duration_s * hz)))
    logger = RunLogger(run_dir, n_rows_est=n_cycles * max(1, len(active_ids)))

    meta = {
        "started_at": iso_now(),
        "family": family,
        "pose": pose,
        "condition": condition,
        "active_motor_ids": active_ids,
        "trajectory_type": traj.traj_type,
        "trajectory_name": traj.name,
        "trajectory_params": traj.params,
        "duration_s_planned": traj.duration_s,
        "sample_hz_target": hz,
        "fields": [
            "commanded_position_rad",
            "actual_position_rad",
            "motor_velocity_rads",
            "motor_torque_nm",
            "motor_temperature_c",
            "commanded_timestamp_ns",
            "feedback_timestamp_ns",
            "cycle_idx",
            "motor_id",
        ],
        "joint_limits_rad": {str(mid): [ctrl.states[mid].lim_lo, ctrl.states[mid].lim_hi] for mid in active_ids},
        "motor_models": {str(mid): ctrl.states[mid].model for mid in active_ids},
        "notes": notes,
    }

    # start countdown
    countdown("Starting run", sec=3)

    dt = 1.0 / hz
    start_perf = time.perf_counter()
    next_t = start_perf
    overruns = 0
    early_stop_reason: Optional[str] = None

    for cyc in range(n_cycles):
        # wait until next cycle boundary
        while True:
            now = time.perf_counter()
            rem = next_t - now
            if rem <= 0.0:
                break
            if rem > 0.001:
                time.sleep(rem - 0.0005)
            else:
                # short spin to reduce jitter
                pass

        cyc_t = time.perf_counter() - start_perf
        cmd_map = traj.fn(cyc_t, cyc)

        # clamp + write active motors
        for mid in active_ids:
            cmd = cmd_map[mid] if mid in cmd_map else ctrl.states[mid].cmd_logical
            st = ctrl.states[mid]
            cmd = clamp(cmd, st.lim_lo, st.lim_hi)

            cmd_ts = time.time_ns()
            try:
                ctrl.write_logical(mid, cmd)
            except Exception as e:
                early_stop_reason = f"write failure on motor {mid}: {e}"
                break

            try:
                pos, vel, tq, temp = ctrl.read(mid)
                fb_ts = time.time_ns()
            except Exception as e:
                early_stop_reason = f"read failure on motor {mid}: {e}"
                break

            logger.append(
                cycle_idx=cyc,
                motor_id=mid,
                cmd=ctrl.states[mid].cmd_logical,
                pos=pos,
                vel=vel,
                tq=tq,
                temp=temp,
                cmd_ts_ns=cmd_ts,
                fb_ts_ns=fb_ts,
            )

        if early_stop_reason is not None:
            break

        # temp safety
        if ctrl.max_temp(active_ids) >= TEMP_ABORT_C:
            early_stop_reason = f"temperature abort: >= {TEMP_ABORT_C}C"
            break

        # schedule next cycle
        next_t += dt
        if time.perf_counter() > next_t:
            overruns += 1

    elapsed = time.perf_counter() - start_perf

    # always hold active motors at end
    ctrl.hold_positions(active_ids)

    # save data
    logger.save_npz()
    if save_csv:
        logger.save_csv()

    meta.update(
        {
            "ended_at": iso_now(),
            "duration_s_actual": elapsed,
            "cycles_executed": int(min(n_cycles, (logger.i // max(1, len(active_ids))))),
            "rows_logged": int(logger.i),
            "achieved_cycle_hz": float((min(n_cycles, (logger.i // max(1, len(active_ids)))) / elapsed) if elapsed > 0 else 0.0),
            "overrun_cycles": int(overruns),
            "early_stop_reason": early_stop_reason,
            "log_file_npz": "log.npz",
            "log_file_csv": "log.csv" if save_csv else None,
        }
    )

    with open(run_dir / "metadata.json", "w") as f:
        json.dump(meta, f, indent=2)

    print(f"Done: rows={meta['rows_logged']} achieved_hz={meta['achieved_cycle_hz']:.2f} overruns={overruns}")
    if early_stop_reason:
        print(f"Stopped early: {early_stop_reason}")

    return meta


def build_single_trajectories_for_motor(ctrl: RobstrideController, mid: int, hz: float, seed0: int) -> List[Trajectory]:
    st = ctrl.states[mid]
    lo, hi = st.lim_lo, st.lim_hi

    # center at midpoint for better ROM coverage
    center = 0.5 * (lo + hi)
    half_span = 0.5 * (hi - lo)

    # amplitudes as fractions of half-span, then clipped to limits
    amp_fracs = [0.20, 0.40, 0.60]
    freqs = [0.25, 0.5, 1.0]

    out: List[Trajectory] = []
    for af in amp_fracs:
        for f in freqs:
            amp = af * half_span
            out.append(make_single_sine(mid, center=center, amp=amp, freq_hz=f, duration_s=12.0, lim=(lo, hi)))

    out.append(make_single_chirp(mid, center=center, amp=0.35 * half_span, f0=0.2, f1=2.0, duration_s=15.0, lim=(lo, hi)))

    out.append(
        make_single_random(
            mid=mid,
            center=center,
            lim=(lo, hi),
            duration_s=20.0,
            hz=hz,
            max_step_rad=MAX_RANDOM_STEP_RAD,
            seed=seed0 + mid,
            keyframe_hz=20.0,
        )
    )

    return out


def build_multi_trajectories(ctrl: RobstrideController, active_ids: List[int]) -> List[Trajectory]:
    centers = {}
    limits = {}
    for mid in active_ids:
        st = ctrl.states[mid]
        centers[mid] = 0.5 * (st.lim_lo + st.lim_hi)
        limits[mid] = (st.lim_lo, st.lim_hi)

    out: List[Trajectory] = []
    for amp_frac, freq, dur in [
        (0.08, 0.25, 15.0),
        (0.10, 0.50, 15.0),
        (0.12, 0.75, 15.0),
    ]:
        out.append(make_multi_sine(active_ids, centers, limits, amp_frac=amp_frac, freq_hz=freq, duration_s=dur))
    return out


def campaign_collect(args):
    root = Path(args.root_dir).resolve()
    campaign_dir = root / datetime.now().strftime("motor_data_%Y%m%d_%H%M%S")

    # create canonical folders
    canonical = [
        "single_air/clean",
        "single_air/disturbed",
        "single_contact/clean",
        "single_contact/disturbed",
        "multi_air/clean",
        "multi_air/disturbed",
        "multi_contact/clean",
        "multi_contact/disturbed",
        "ik_contact/clean",
    ]
    for c in canonical:
        ensure_dir(campaign_dir / c)

    motor_ids = scan_motor_ids_or_parse(args.motor_ids, args.channel)

    if args.single_ids:
        single_ids = parse_id_list(args.single_ids)
    else:
        single_ids = motor_ids

    if args.multi_ids:
        multi_ids = parse_id_list(args.multi_ids)
    else:
        multi_ids = motor_ids

    # validate IDs
    for s in single_ids + multi_ids:
        if s not in motor_ids:
            raise ValueError(f"ID {s} not present in detected motor_ids {motor_ids}")

    ctrl = RobstrideController(
        motor_ids=motor_ids,
        channel=args.channel,
        bitrate=args.bitrate,
        kp=args.kp,
        kd=args.kd,
    )

    all_meta: List[Dict[str, Any]] = []

    def _shutdown(_sig=None, _frm=None):
        print("\nSignal received, shutting down...")
        ctrl.shutdown()
        # write partial summary
        try:
            with open(campaign_dir / "campaign_summary_partial.json", "w") as f:
                json.dump(all_meta, f, indent=2)
        finally:
            os._exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    ctrl.connect()
    print(f"Connected. Campaign folder: {campaign_dir}")
    print(f"Target logging rate: {args.hz:.1f} Hz")

    try:
        # ---------- SINGLE-MOTOR CAMPAIGN ----------
        if args.campaign in ("all", "single"):
            for pose in ["air", "contact"]:
                input(f"\nPrepare robot in '{pose.upper()}' condition, then press Enter...")
                for cond in ["clean", "disturbed"]:
                    base = campaign_dir / f"single_{pose}" / cond
                    for mid in single_ids:
                        trajs = build_single_trajectories_for_motor(ctrl, mid, hz=args.hz, seed0=args.random_seed)
                        for tr in trajs:
                            m = run_one(
                                ctrl=ctrl,
                                run_root=base,
                                family="single",
                                pose=pose,
                                condition=cond,
                                active_ids=[mid],
                                traj=tr,
                                hz=args.hz,
                                save_csv=args.save_csv,
                                notes="single motor excitation",
                            )
                            all_meta.append(m)

        # ---------- MULTI-MOTOR CAMPAIGN ----------
        if args.campaign in ("all", "multi"):
            for pose in ["air", "contact"]:
                input(f"\nPrepare robot in '{pose.upper()}' condition for MULTI runs, then press Enter...")
                for cond in ["clean", "disturbed"]:
                    base = campaign_dir / f"multi_{pose}" / cond
                    mtrajs = build_multi_trajectories(ctrl, multi_ids)
                    for tr in mtrajs:
                        m = run_one(
                            ctrl=ctrl,
                            run_root=base,
                            family="multi",
                            pose=pose,
                            condition=cond,
                            active_ids=multi_ids,
                            traj=tr,
                            hz=args.hz,
                            save_csv=args.save_csv,
                            notes="multi motor sine only",
                        )
                        all_meta.append(m)

        # ---------- IK LOCOMOTION REPLAY ----------
        if args.campaign in ("all", "ik"):
            ik_base = campaign_dir / "ik_contact" / "clean"
            input("\nPrepare robot in CONTACT condition for IK locomotion replay, then press Enter...")
            ik_path = resolve_ik_path(args.ik_path)
            t, q = load_ik_trajectory(ik_path, active_ids=multi_ids, fallback_hz=args.ik_fallback_hz)
            ik_traj = make_ik_traj(multi_ids, t, q)

            m = run_one(
                ctrl=ctrl,
                run_root=ik_base,
                family="ik",
                pose="contact",
                condition="clean",
                active_ids=multi_ids,
                traj=ik_traj,
                hz=args.hz,
                save_csv=args.save_csv,
                notes=f"IK replay from {ik_path.name}",
            )
            all_meta.append(m)

    finally:
        ctrl.shutdown()

    # campaign summary
    with open(campaign_dir / "campaign_summary.json", "w") as f:
        json.dump(all_meta, f, indent=2)

    print("\nCampaign complete.")
    print(f"Summary: {campaign_dir / 'campaign_summary.json'}")


def make_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="RobStride motor dataset collection campaign")
    p.add_argument("--root-dir", type=str, default="./motor_dataset", help="Root folder for campaign outputs")
    p.add_argument("--campaign", type=str, default="all", choices=["all", "single", "multi", "ik"])

    p.add_argument("--channel", type=str, default="can0")
    p.add_argument("--bitrate", type=int, default=1_000_000)

    p.add_argument("--motor-ids", type=str, default="", help="Space-separated IDs; empty => scan bus")
    p.add_argument("--single-ids", type=str, default="", help="Space-separated IDs for single-motor runs")
    p.add_argument("--multi-ids", type=str, default="", help="Space-separated IDs for multi-motor and IK runs")

    p.add_argument("--kp", type=float, default=10.0)
    p.add_argument("--kd", type=float, default=0.2)

    p.add_argument("--hz", type=float, default=400.0, help="Target control/logging frequency")
    p.add_argument("--save-csv", action="store_true", help="Also export CSV (slower + larger)")

    p.add_argument("--ik-path", type=str, default="IK_trajectory", help="IK trajectory path (npz/npy/csv)")
    p.add_argument("--ik-fallback-hz", type=float, default=400.0, help="Used if IK file has no explicit time")

    p.add_argument("--random-seed", type=int, default=1234)

    return p


def main():
    args = make_argparser().parse_args()
    campaign_collect(args)


if __name__ == "__main__":
    main()
