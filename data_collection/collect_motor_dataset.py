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
import threading
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
    from utils.actuation_safety import ActuationSafetyMonitor
except ImportError:
    try:
        from bus import RobstrideBus, Motor
        from protocol import ParameterType, CommunicationType
        from utils.actuation_safety import ActuationSafetyMonitor
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

# Mirrors run_policy_config.json -> default_joint_pos_real_rad_by_joint mapped by motor ID.
DEFAULT_JOINT_POS_BY_ID: Dict[int, float] = {
    1: 0.4,
    2: 0.0,
    3: 0.0,
    4: -0.8,
    5: 0.4,
    6: 0.4,
    7: 0.0,
    8: 0.0,
    9: -0.8,
    10: 0.4,
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

# Per-motor MIT gains from latest gain-tuner values.
# Note: ID 6 appeared twice in the provided list; this map uses the later entry (kp=200, kd=5).
GAINS_BY_ID: Dict[int, Tuple[float, float]] = {
    1: (250.0, 5.0),
    2: (250.0, 5.0),
    3: (100.0, 2.0),
    4: (150.0, 5.0),
    5: (120.0, 0.8),
    6: (250.0, 5.0),
    7: (250.0, 5.0),
    8: (100.0, 2.0),
    9: (150.0, 5.0),
    10: (120.0, 1.0),
}

TEMP_ABORT_C = 88.0
TEMP_VALID_MIN_C = -40.0
TEMP_VALID_MAX_C = 200.0
TEMP_MAX_STEP_C = 40.0
TEMP_SAFETY_ENABLED = False
MAX_RANDOM_STEP_RAD = math.radians(10.0)  # <= 10 deg between random commanded positions


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
    temp_error_count: int = 0
    last_temp_error: Optional[str] = None


class RobstrideController:
    def __init__(
        self,
        motor_ids: List[int],
        channel: str,
        bitrate: int,
        kp: float,
        kd: float,
        control_hz: float,
        safety_read_hz: float | None,
        safety_max_jump_deg: float,
        safety_enabled: bool = True,
    ):
        self.channel = channel
        self.bitrate = bitrate
        self.motor_ids = sorted(motor_ids)

        self.states: Dict[int, MotorState] = {}
        self.lock = threading.RLock()
        self.control_hz = float(control_hz)
        self.safety_enabled = bool(safety_enabled)
        self.safety_read_hz = None if safety_read_hz is None else float(safety_read_hz)
        self.safety_max_jump_deg = float(safety_max_jump_deg)
        self.safety_tripped = False
        self.safety_reason: Optional[str] = None
        self.safety_monitor: Optional[ActuationSafetyMonitor] = None
        for mid in self.motor_ids:
            lo, hi = JOINT_LIMITS.get(mid, (-math.inf, math.inf))
            tuned_kp, tuned_kd = GAINS_BY_ID.get(mid, (float(kp), float(kd)))
            st = MotorState(
                mid=mid,
                name=f"motor_{mid}",
                model=MOTOR_MODEL_BY_ID.get(mid, "rs-03"),
                direction=int(INVERSION_BY_ID.get(mid, 1)),
                lim_lo=lo,
                lim_hi=hi,
                kp=float(tuned_kp),
                kd=float(tuned_kd),
            )
            self.states[mid] = st

        self.bus: Optional[RobstrideBus] = None
        self.connected = False

    def _trip_safety(self, reason: str):
        if self.safety_tripped:
            return
        self.safety_tripped = True
        self.safety_reason = reason
        print(reason)

    def _assert_safe(self):
        if not self.safety_enabled:
            return
        if self.safety_tripped:
            raise RuntimeError(self.safety_reason or "Safety monitor tripped")

    def _read_logical_for_safety(self, mid: int) -> float:
        assert self.bus is not None
        st = self.states[mid]
        acquired = self.lock.acquire(timeout=0.001)
        if not acquired:
            raise TimeoutError("safety read skipped: control lock busy")
        try:
            pos, vel, tq, temp = self.bus.read_operation_frame(st.name, timeout=0.005)
            st.pos, st.vel, st.tq = pos, vel, tq
            st.temp = self._sanitize_temp_reading(st, temp)
            return float(pos) / float(st.direction)
        finally:
            self.lock.release()

    def _sanitize_temp_reading(self, st: MotorState, raw_temp: float) -> float:
        t = float(raw_temp)
        prev = float(st.temp)
        msg: Optional[str] = None
        if (not math.isfinite(t)) or t < TEMP_VALID_MIN_C or t > TEMP_VALID_MAX_C:
            msg = f"ignored invalid temp reading on motor {st.mid}: {t:.1f}C"
        elif TEMP_VALID_MIN_C <= prev <= TEMP_VALID_MAX_C and abs(t - prev) > TEMP_MAX_STEP_C:
            msg = f"ignored temp spike on motor {st.mid}: prev={prev:.1f}C new={t:.1f}C"

        if msg is not None:
            st.temp_error_count += 1
            st.last_temp_error = msg
            if st.temp_error_count <= 5 or (st.temp_error_count % 20 == 0):
                print(f"WARNING: {msg} (count={st.temp_error_count})")
            return prev

        st.last_temp_error = None
        return t

    def _start_safety_monitor(self):
        if not self.safety_enabled:
            print("[SAFETY] monitor disabled (--no-safety)")
            if not TEMP_SAFETY_ENABLED:
                print("[TEMP] thermal abort disabled; temperatures will still be logged.")
            return
        joint_limits = {mid: (self.states[mid].lim_lo, self.states[mid].lim_hi) for mid in self.motor_ids}
        self.safety_monitor = ActuationSafetyMonitor(
            name="collect_dataset",
            motor_ids=self.motor_ids,
            joint_limits_by_id=joint_limits,
            read_logical_pos_fn=self._read_logical_for_safety,
            halt_fn=self._trip_safety,
            control_hz=self.control_hz,
            read_hz=self.safety_read_hz,
            max_step_deg=self.safety_max_jump_deg,
        )
        self.safety_monitor.start()
        per_motor_hz = self.safety_monitor.read_hz / max(1, len(self.motor_ids))
        print(
            f"[SAFETY] monitor started: control_hz={self.control_hz:.1f}, "
            f"read_hz_total={self.safety_monitor.read_hz:.1f}, per_motor~{per_motor_hz:.1f}, "
            f"max_jump_deg={self.safety_max_jump_deg:.1f}"
        )
        if not TEMP_SAFETY_ENABLED:
            print("[TEMP] thermal abort disabled; temperatures will still be logged.")

    def _set_mode_raw(self, mid: int, mode: int = 0):
        assert self.bus is not None
        dev_id = self.bus.motors[f"motor_{mid}"].id
        param_id, _, _ = ParameterType.MODE
        value_buffer = struct.pack("<bBH", int(mode), 0, 0)
        data = struct.pack("<HH", param_id, 0x00) + value_buffer
        self.bus.transmit(CommunicationType.WRITE_PARAMETER, self.bus.host_id, dev_id, data)
        time.sleep(0.1)

    def _read_operation_frame_retry(self, motor_name: str, attempts: int = 3, delay_s: float = 0.03):
        assert self.bus is not None
        last_exc: Optional[Exception] = None
        for i in range(max(1, attempts)):
            try:
                return self.bus.read_operation_frame(motor_name)
            except Exception as e:
                last_exc = e
                if i + 1 < attempts:
                    time.sleep(delay_s)
        raise RuntimeError(f"read_operation_frame failed for {motor_name}: {last_exc}")

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
                with self.lock:
                    self.bus.enable(st.name)
                    time.sleep(0.25)
                    self._set_mode_raw(mid, 0)

                # Match gain_tuner behavior: do not fail whole connection if first read is noisy.
                try:
                    pos, vel, tq, temp = self._read_operation_frame_retry(st.name, attempts=3, delay_s=0.03)
                    st.pos, st.vel, st.tq = pos, vel, tq
                    st.temp = self._sanitize_temp_reading(st, temp)
                    logical = pos / float(st.direction)
                    st.cmd_logical = logical
                except Exception as e:
                    print(f"WARNING: initial read failed on {st.name}: {e}; holding at 0.0 rad")
                    st.pos = 0.0
                    st.vel = 0.0
                    st.tq = 0.0
                    st.temp = 0.0
                    st.cmd_logical = 0.0

                # initial hold
                with self.lock:
                    self.bus.write_operation_frame(
                        st.name,
                        st.cmd_logical * float(st.direction),
                        st.kp,
                        st.kd,
                        0.0,
                        0.0,
                    )
                time.sleep(0.05)

            self.connected = True
            self._start_safety_monitor()
        except Exception as e:
            raise RuntimeError(f"connect failed: {e}")

    def read(self, mid: int) -> Tuple[float, float, float, float]:
        self._assert_safe()
        assert self.bus is not None
        st = self.states[mid]
        last_exc: Optional[Exception] = None
        for _ in range(3):
            try:
                with self.lock:
                    pos, vel, tq, temp = self.bus.read_operation_frame(st.name, timeout=0.01)
                st.pos, st.vel, st.tq = pos, vel, tq
                st.temp = self._sanitize_temp_reading(st, temp)
                return pos, vel, tq, st.temp
            except Exception as exc:
                last_exc = exc
                time.sleep(0.001)
        raise RuntimeError(f"read failed on {st.name}: {last_exc}")

    def write_logical(self, mid: int, logical_target: float):
        self._assert_safe()
        assert self.bus is not None
        st = self.states[mid]
        t = clamp(logical_target, st.lim_lo, st.lim_hi)
        st.cmd_logical = t
        phys = t * float(st.direction)
        with self.lock:
            self.bus.write_operation_frame(st.name, phys, st.kp, st.kd, 0.0, 0.0)

    def write_logical_and_read(self, mid: int, logical_target: float, timeout: float = 0.01) -> Tuple[float, float, float, float]:
        self._assert_safe()
        assert self.bus is not None
        st = self.states[mid]
        t = clamp(logical_target, st.lim_lo, st.lim_hi)
        st.cmd_logical = t
        phys = t * float(st.direction)
        last_exc: Optional[Exception] = None
        for _ in range(3):
            try:
                with self.lock:
                    self.bus.write_operation_frame(st.name, phys, st.kp, st.kd, 0.0, 0.0)
                    pos, vel, tq, temp = self.bus.read_operation_frame(st.name, timeout=timeout)
                st.pos, st.vel, st.tq = pos, vel, tq
                st.temp = self._sanitize_temp_reading(st, temp)
                return pos, vel, tq, st.temp
            except Exception as exc:
                last_exc = exc
                time.sleep(0.001)
        raise RuntimeError(f"write/read failed on {st.name}: {last_exc}")

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
            try:
                self.write_logical_and_read(mid, st.cmd_logical, timeout=0.01)
            except Exception:
                pass

    def move_to_targets(self, targets: Dict[int, float], duration_s: float = 1.0, hz: float = 200.0):
        """Smoothly interpolate logical commands to targets (no logging, time-based)."""
        self._assert_safe()
        duration_s = max(0.0, float(duration_s))
        hz = max(1.0, float(hz))
        # Ramp from measured current joint position (not last commanded setpoint).
        # Using stale cmd_logical can cause an apparent "snap" at ramp start.
        start: Dict[int, float] = {}
        for mid in targets.keys():
            st = self.states[mid]
            try:
                pos, _, _, _ = self.read(mid)
                logical = pos / float(st.direction)
            except Exception:
                logical = st.cmd_logical
            logical = clamp(logical, st.lim_lo, st.lim_hi)
            st.cmd_logical = logical
            start[mid] = logical
        if duration_s <= 1e-6:
            for mid, tgt in targets.items():
                try:
                    self.write_logical_and_read(mid, tgt, timeout=0.01)
                except Exception:
                    pass
            return

        t0 = time.perf_counter()
        next_t = t0
        dt = 1.0 / hz

        while True:
            now = time.perf_counter()
            elapsed = now - t0
            a = clamp(elapsed / duration_s, 0.0, 1.0)
            for mid, tgt in targets.items():
                cmd = (1.0 - a) * start[mid] + a * tgt
                try:
                    self.write_logical_and_read(mid, cmd, timeout=0.01)
                except Exception:
                    pass
            if a >= 1.0:
                break

            next_t += dt
            now2 = time.perf_counter()
            if now2 < next_t:
                time.sleep(next_t - now2)
            else:
                next_t = now2

    def max_temp(self, mids: List[int]) -> float:
        vals = [self.states[mid].temp for mid in mids]
        return max(vals) if vals else 0.0

    def shutdown(self):
        if not self.connected or self.bus is None:
            return
        if self.safety_monitor is not None:
            self.safety_monitor.stop()
            self.safety_monitor = None
        try:
            for mid in self.motor_ids:
                try:
                    with self.lock:
                        self.bus.disable(self.states[mid].name)
                except Exception:
                    pass
            with self.lock:
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


def _run_key(family: str, pose: str, condition: str, active_ids: List[int], traj_name: str) -> str:
    ids_tag = ",".join(str(i) for i in sorted(active_ids))
    return f"{family}|{pose}|{condition}|{ids_tag}|{traj_name}"


def _save_json(path: Path, obj: Any):
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)


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
    command_max_vel_rad_s: float,
    max_read_failures: int,
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
    command_max_vel_rad_s = max(1e-6, float(command_max_vel_rad_s))
    start_perf = time.perf_counter()
    next_t = start_perf
    overruns = 0
    read_failures = 0
    early_stop_reason: Optional[str] = None
    prev_cmd: Dict[int, float] = {mid: float(ctrl.states[mid].cmd_logical) for mid in active_ids}

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

        # clamp + write/read active motors
        for mid in active_ids:
            cmd = cmd_map[mid] if mid in cmd_map else ctrl.states[mid].cmd_logical
            st = ctrl.states[mid]
            cmd = clamp(cmd, st.lim_lo, st.lim_hi)
            max_step = command_max_vel_rad_s * dt
            cmd = clamp(cmd, prev_cmd[mid] - max_step, prev_cmd[mid] + max_step)
            prev_cmd[mid] = cmd

            cmd_ts = time.time_ns()
            try:
                pos, vel, tq, temp = ctrl.write_logical_and_read(mid, cmd, timeout=0.01)
                fb_ts = time.time_ns()
            except Exception as e:
                read_failures += 1
                if read_failures <= 5 or (read_failures % 20 == 0):
                    print(f"WARNING: write/read failure on motor {mid}: {e} (count={read_failures})")
                if read_failures >= max_read_failures:
                    early_stop_reason = f"too many read failures ({read_failures})"
                    break
                continue

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

        # thermal abort intentionally disabled; keep logging temperature telemetry
        if TEMP_SAFETY_ENABLED and ctrl.max_temp(active_ids) >= TEMP_ABORT_C:
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
            "read_failures": int(read_failures),
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

    # Safer chirp: lower top frequency and longer duration.
    out.append(make_single_chirp(mid, center=center, amp=0.25 * half_span, f0=0.05, f1=0.50, duration_s=20.0, lim=(lo, hi)))

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
    if args.campaign_dir.strip():
        campaign_dir = Path(args.campaign_dir).expanduser().resolve()
    else:
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

    progress_path = campaign_dir / "campaign_progress.json"
    summary_path = campaign_dir / "campaign_summary.json"

    completed_keys: set[str] = set()
    all_meta: List[Dict[str, Any]] = []
    if args.resume and progress_path.exists():
        try:
            prog = json.loads(progress_path.read_text())
            completed_keys = set(str(x) for x in prog.get("completed_keys", []))
            all_meta = list(prog.get("all_meta", []))
            print(f"Resume enabled: loaded {len(completed_keys)} completed runs from {progress_path}")
        except Exception as e:
            print(f"WARNING: Failed to load progress file {progress_path}: {e}")
            print("Starting with empty progress.")

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
        control_hz=args.hz,
        safety_read_hz=args.safety_read_hz,
        safety_max_jump_deg=args.safety_max_jump_deg,
        safety_enabled=(not args.no_safety),
    )

    def _persist_progress():
        payload = {
            "updated_at": iso_now(),
            "campaign_dir": str(campaign_dir),
            "completed_keys": sorted(completed_keys),
            "all_meta": all_meta,
        }
        _save_json(progress_path, payload)
        _save_json(summary_path, all_meta)

    def _shutdown(_sig=None, _frm=None):
        print("\nSignal received, shutting down...")
        ctrl.shutdown()
        try:
            _persist_progress()
            _save_json(campaign_dir / "campaign_summary_partial.json", all_meta)
        finally:
            os._exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    ctrl.connect()
    print(f"Connected. Campaign folder: {campaign_dir}")
    print(f"Target logging rate: {args.hz:.1f} Hz")

    if not args.no_move_to_defaults:
        default_targets: Dict[int, float] = {}
        for mid in ctrl.motor_ids:
            if mid in DEFAULT_JOINT_POS_BY_ID:
                st = ctrl.states[mid]
                default_targets[mid] = clamp(DEFAULT_JOINT_POS_BY_ID[mid], st.lim_lo, st.lim_hi)
        if default_targets:
            print(
                "Moving controlled joints to default joint positions "
                f"(duration={args.default_move_duration_s:.2f}s, rate={args.default_move_hz:.1f}Hz)..."
            )
            ctrl.move_to_targets(
                default_targets,
                duration_s=float(args.default_move_duration_s),
                hz=float(args.default_move_hz),
            )
            print("Default joint positioning complete.")

    def _has_pending_single(pose: str, cond: str) -> bool:
        for mid in single_ids:
            trajs = build_single_trajectories_for_motor(ctrl, mid, hz=args.hz, seed0=args.random_seed)
            for tr in trajs:
                key = _run_key("single", pose, cond, [mid], tr.name)
                if key not in completed_keys:
                    return True
        return False

    def _has_pending_multi(pose: str, cond: str) -> bool:
        mtrajs = build_multi_trajectories(ctrl, multi_ids)
        for tr in mtrajs:
            key = _run_key("multi", pose, cond, multi_ids, tr.name)
            if key not in completed_keys:
                return True
        return False

    try:
        # ---------- SINGLE-MOTOR CAMPAIGN ----------
        if args.campaign in ("all", "single"):
            for pose in ["air", "contact"]:
                pose_pending = any(_has_pending_single(pose, cond) for cond in ["clean", "disturbed"])
                if args.resume and not pose_pending:
                    print(f"Skipping pose '{pose}' in single campaign (no pending runs).")
                    continue
                input(f"\nPrepare robot in '{pose.upper()}' condition, then press Enter...")
                for cond in ["clean", "disturbed"]:
                    if args.resume and not _has_pending_single(pose, cond):
                        print(f"Skipping single_{pose}/{cond} (no pending runs).")
                        continue
                    base = campaign_dir / f"single_{pose}" / cond
                    for mid in single_ids:
                        trajs = build_single_trajectories_for_motor(ctrl, mid, hz=args.hz, seed0=args.random_seed)
                        for tr in trajs:
                            key = _run_key("single", pose, cond, [mid], tr.name)
                            if args.resume and key in completed_keys:
                                print(f"Skipping completed run: {key}")
                                continue
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
                                command_max_vel_rad_s=args.command_max_vel_rad_s,
                                max_read_failures=args.max_read_failures,
                                notes="single motor excitation",
                            )
                            m["resume_key"] = key
                            all_meta.append(m)
                            completed_keys.add(key)
                            _persist_progress()

        # ---------- MULTI-MOTOR CAMPAIGN ----------
        if args.campaign in ("all", "multi"):
            for pose in ["air", "contact"]:
                pose_pending = any(_has_pending_multi(pose, cond) for cond in ["clean", "disturbed"])
                if args.resume and not pose_pending:
                    print(f"Skipping pose '{pose}' in multi campaign (no pending runs).")
                    continue
                input(f"\nPrepare robot in '{pose.upper()}' condition for MULTI runs, then press Enter...")
                for cond in ["clean", "disturbed"]:
                    if args.resume and not _has_pending_multi(pose, cond):
                        print(f"Skipping multi_{pose}/{cond} (no pending runs).")
                        continue
                    base = campaign_dir / f"multi_{pose}" / cond
                    mtrajs = build_multi_trajectories(ctrl, multi_ids)
                    for tr in mtrajs:
                        key = _run_key("multi", pose, cond, multi_ids, tr.name)
                        if args.resume and key in completed_keys:
                            print(f"Skipping completed run: {key}")
                            continue
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
                            command_max_vel_rad_s=args.command_max_vel_rad_s,
                            max_read_failures=args.max_read_failures,
                            notes="multi motor sine only",
                        )
                        m["resume_key"] = key
                        all_meta.append(m)
                        completed_keys.add(key)
                        _persist_progress()

        # ---------- IK LOCOMOTION REPLAY ----------
        if args.campaign in ("all", "ik"):
            ik_base = campaign_dir / "ik_contact" / "clean"
            input("\nPrepare robot in CONTACT condition for IK locomotion replay, then press Enter...")
            ik_path = resolve_ik_path(args.ik_path)
            t, q = load_ik_trajectory(ik_path, active_ids=multi_ids, fallback_hz=args.ik_fallback_hz)
            ik_traj = make_ik_traj(multi_ids, t, q)
            key = _run_key("ik", "contact", "clean", multi_ids, ik_traj.name)
            if args.resume and key in completed_keys:
                print(f"Skipping completed run: {key}")
            else:
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
                    command_max_vel_rad_s=args.command_max_vel_rad_s,
                    max_read_failures=args.max_read_failures,
                    notes=f"IK replay from {ik_path.name}",
                )
                m["resume_key"] = key
                all_meta.append(m)
                completed_keys.add(key)
                _persist_progress()

    finally:
        ctrl.shutdown()

    _persist_progress()

    print("\nCampaign complete.")
    print(f"Summary: {campaign_dir / 'campaign_summary.json'}")


def make_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="RobStride motor dataset collection campaign")
    p.add_argument("--root-dir", type=str, default="./motor_dataset", help="Root folder for campaign outputs")
    p.add_argument(
        "--campaign-dir",
        type=str,
        default="",
        help="Existing campaign directory to use/resume. Empty => create timestamped folder under --root-dir",
    )
    p.add_argument("--resume", action="store_true", help="Resume by skipping completed runs from campaign_progress.json")
    p.add_argument("--campaign", type=str, default="all", choices=["all", "single", "multi", "ik"])

    p.add_argument("--channel", type=str, default="can0")
    p.add_argument("--bitrate", type=int, default=1_000_000)

    p.add_argument("--motor-ids", type=str, default="", help="Space-separated IDs; empty => scan bus")
    p.add_argument("--single-ids", type=str, default="", help="Space-separated IDs for single-motor runs")
    p.add_argument("--multi-ids", type=str, default="", help="Space-separated IDs for multi-motor and IK runs")

    p.add_argument("--kp", type=float, default=10.0)
    p.add_argument("--kd", type=float, default=0.2)

    p.add_argument("--hz", type=float, default=400.0, help="Target control/logging frequency")
    p.add_argument(
        "--command-max-vel-rad-s",
        type=float,
        default=2.5,
        help="Per-motor command slew-rate limit applied in run loop.",
    )
    p.add_argument(
        "--max-read-failures",
        type=int,
        default=200,
        help="Abort run only after this many read failures (transient misses are tolerated).",
    )
    p.add_argument("--save-csv", action="store_true", help="Also export CSV (slower + larger)")

    p.add_argument("--ik-path", type=str, default="IK_trajectory", help="IK trajectory path (npz/npy/csv)")
    p.add_argument("--ik-fallback-hz", type=float, default=400.0, help="Used if IK file has no explicit time")

    p.add_argument("--random-seed", type=int, default=1234)
    p.add_argument(
        "--safety-read-hz",
        type=float,
        default=0.0,
        help="Safety monitor read rate. If <=0, uses max(120, 2x control hz).",
    )
    p.add_argument(
        "--safety-max-jump-deg",
        type=float,
        default=90.0,
        help="Hard stop if any per-sample logical position jump exceeds this angle.",
    )
    p.add_argument(
        "--no-safety",
        action="store_true",
        help="Disable safety monitor and safety-trip command guards (dangerous).",
    )
    p.add_argument(
        "--default-move-duration-s",
        type=float,
        default=2.5,
        help="Seconds to ramp controlled joints to default joint positions at startup.",
    )
    p.add_argument(
        "--default-move-hz",
        type=float,
        default=200.0,
        help="Interpolation rate (Hz) for startup ramp to default joint positions.",
    )
    p.add_argument(
        "--no-move-to-defaults",
        action="store_true",
        help="Disable startup move to default joint positions.",
    )

    return p


def main():
    args = make_argparser().parse_args()
    if args.safety_read_hz <= 0.0:
        args.safety_read_hz = None
    campaign_collect(args)


if __name__ == "__main__":
    main()
