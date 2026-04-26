#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Low-level robot I/O for run_policy:
- CAN connect/enable/disable
- MIT mode writes
- motor feedback reads
- ankle 4-bar mapping for ankle joints
"""

from __future__ import annotations

import math
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from run_policy_validation import require_key


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    from robstride_dynamics import RobstrideBus, Motor, ParameterType, CommunicationType
except ModuleNotFoundError as exc:
    if exc.name == "can":
        raise ModuleNotFoundError(
            "Missing dependency 'python-can'. Install it with: python -m pip install python-can"
        ) from exc
    raise
except ImportError:
    from robstride_dynamics.bus import RobstrideBus, Motor
    from robstride_dynamics.protocol import ParameterType, CommunicationType

try:
    from scipy.optimize import fsolve  # type: ignore

    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def to_scalar_float(x: Any) -> float:
    if x is None:
        return 0.0
    if isinstance(x, (float, int)):
        return float(x)
    arr = np.asarray(x)
    if arr.ndim == 0:
        return float(arr)
    if arr.size == 1:
        return float(arr.reshape(()).item())
    return float(arr.reshape(-1)[0])


def wrap_to_pi(x: float) -> float:
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def unwrap_to_near(x: float, ref: float) -> float:
    return ref + wrap_to_pi(x - ref)


def offset_to_pi(x: float) -> float:
    return x - wrap_to_pi(x)


def lookup_int_key(mapping: dict[str, Any] | dict[int, Any], key: int, default: Any = None) -> Any:
    if key in mapping:
        return mapping[key]  # type: ignore[index]
    return mapping.get(str(key), default)  # type: ignore[call-arg]


class AnkleMapper:
    def __init__(self, cfg: dict[str, Any]):
        lengths = require_key(cfg, "link_lengths", "ankle_mapping")
        if len(lengths) != 4:
            raise ValueError("ankle_mapping.link_lengths must be [L1, L2, L3, L4]")
        self.l1 = float(lengths[0])
        self.l2 = float(lengths[1])
        self.l3 = float(lengths[2])
        self.l4 = float(lengths[3])

        self.k1 = self.l1 / self.l4
        self.k2 = self.l1 / self.l2
        self.k3 = (self.l2**2 - self.l3**2 + self.l4**2 + self.l1**2) / (2.0 * self.l2 * self.l4)

        self.theta2_offset_rad = math.radians(float(require_key(cfg, "theta2_offset_deg", "ankle_mapping")))
        self.t4_to_motor_offset_rad = math.radians(float(require_key(cfg, "t4_to_motor_offset_deg", "ankle_mapping")))
        self.ankle_to_theta2_sign = float(require_key(cfg, "ankle_to_theta2_sign", "ankle_mapping"))

    def _solve_foot_to_motor_theta2(self, target_foot_deg: float, t2_guess_rad: float) -> float:
        theta4 = math.radians(float(target_foot_deg))

        def f(t2: float) -> float:
            return self.k1 * math.cos(theta4) - self.k2 * math.cos(t2) - math.cos(t2 - theta4) + self.k3

        if _HAS_SCIPY:
            sol = fsolve(lambda x: f(to_scalar_float(x)), [float(t2_guess_rad)], xtol=1e-10, maxfev=100)
            return float(sol[0])

        t2 = float(t2_guess_rad)
        for _ in range(50):
            ft = f(t2)
            dft = self.k2 * math.sin(t2) + math.sin(t2 - theta4)
            if abs(dft) < 1e-12:
                break
            step = ft / dft
            t2 -= step
            if abs(step) < 1e-12:
                break
        return float(t2)

    def _solve_motor_to_foot_theta4(self, t2_rad: float, t4_guess_rad: float) -> float:
        def f(t4: float) -> float:
            return self.k1 * math.cos(t4) - self.k2 * math.cos(t2_rad) - math.cos(t2_rad - t4) + self.k3

        if _HAS_SCIPY:
            sol = fsolve(lambda x: f(to_scalar_float(x)), [float(t4_guess_rad)], xtol=1e-10, maxfev=100)
            return float(sol[0])

        t4 = float(t4_guess_rad)
        for _ in range(50):
            ft = f(t4)
            dft = -self.k1 * math.sin(t4) - math.sin(t2_rad - t4)
            if abs(dft) < 1e-12:
                break
            step = ft / dft
            t4 -= step
            if abs(step) < 1e-12:
                break
        return float(t4)

    def ankle_rad_to_motor_logical_rad(self, ankle_rad: float, motor_guess_logical_rad: float) -> float:
        theta2_model = self.theta2_offset_rad + self.ankle_to_theta2_sign * float(ankle_rad)
        t4_guess_model = motor_guess_logical_rad - self.t4_to_motor_offset_rad
        t4_model_raw = self._solve_motor_to_foot_theta4(theta2_model, t4_guess_model)
        t4_model = unwrap_to_near(t4_model_raw, t4_guess_model)
        motor_target = t4_model + self.t4_to_motor_offset_rad
        return unwrap_to_near(motor_target, motor_guess_logical_rad)

    def motor_logical_rad_to_ankle_rad(self, motor_logical_rad: float, ankle_guess_rad: float) -> float:
        theta4_model = motor_logical_rad - self.t4_to_motor_offset_rad
        theta2_guess = self.theta2_offset_rad + self.ankle_to_theta2_sign * float(ankle_guess_rad)
        theta2_raw = self._solve_foot_to_motor_theta2(math.degrees(theta4_model), theta2_guess)
        theta2 = unwrap_to_near(theta2_raw, theta2_guess)
        return float((theta2 - self.theta2_offset_rad) / self.ankle_to_theta2_sign)


@dataclass
class JointMotorState:
    joint_name: str
    motor_id: int
    model: str
    direction: int
    limit_lo: float
    limit_hi: float
    kp: float
    kd: float
    max_vel_rad_s: float
    is_ankle: bool
    motor_name: str
    position_phys: float = 0.0
    velocity_phys: float = 0.0
    torque_nm: float = 0.0
    temp_c: float = 0.0
    joint_pos: float = 0.0
    joint_vel: float = 0.0
    commanded_joint: float = 0.0
    startup_motor_offset_rad: float = 0.0
    last_read_time: float = 0.0
    last_cmd_phys: float = 0.0
    last_error: str | None = None


class RobotInterface:
    def __init__(self, cfg: dict[str, Any], dry_run: bool = False):
        self.cfg = cfg
        self.dry_run = bool(dry_run)

        self.real_joint_order = list(require_key(cfg, "real_joint_order"))
        self.policy_joint_order = list(require_key(cfg, "policy_joint_order"))
        self.can_channel = str(require_key(cfg, "can_channel"))
        self.bitrate = int(require_key(cfg, "bitrate"))

        self.joint_to_motor_id = dict(require_key(cfg, "joint_to_motor_id"))
        self.motor_model_by_id = dict(require_key(cfg, "motor_model_by_id"))
        self.default_motor_model = str(require_key(cfg, "default_motor_model"))
        self.inversion_array = list(require_key(cfg, "inversion_array"))
        self.ankle_joint_names = set(require_key(cfg, "ankle_joint_names"))

        self.joint_limits_cfg = dict(require_key(cfg, "joint_limits_rad_by_joint"))
        self.default_joint_pos_cfg = dict(require_key(cfg, "default_joint_pos_real_rad_by_joint"))
        self.kp_by_joint = dict(require_key(cfg, "kp_by_joint"))
        self.kd_by_joint = dict(require_key(cfg, "kd_by_joint"))
        self.max_vel_by_joint = dict(require_key(cfg, "max_vel_rad_s_by_joint"))
        self.action_scale_by_joint = dict(require_key(cfg, "action_scale_by_joint"))

        self.kp_default = float(require_key(cfg, "kp"))
        self.kd_default = float(require_key(cfg, "kd"))
        self.max_vel_default = float(require_key(cfg, "max_vel_rad_s"))

        self.ankle_mapper = AnkleMapper(dict(require_key(cfg, "ankle_mapping")))

        self.states: list[JointMotorState] = []
        self._state_by_joint: dict[str, JointMotorState] = {}

        self.default_joint_pos_real = np.zeros(len(self.real_joint_order), dtype=float)
        self.joint_action_scales_real = np.ones(len(self.real_joint_order), dtype=float)
        self.max_vel_real = np.zeros(len(self.real_joint_order), dtype=float)
        joint_limits_real: list[tuple[float, float]] = []

        for idx, joint_name in enumerate(self.real_joint_order):
            if joint_name not in self.joint_to_motor_id:
                raise ValueError(f"joint_to_motor_id missing '{joint_name}'")
            motor_id = int(self.joint_to_motor_id[joint_name])

            model = str(lookup_int_key(self.motor_model_by_id, motor_id, self.default_motor_model))
            if not (1 <= motor_id <= len(self.inversion_array)):
                raise ValueError(
                    f"Motor ID {motor_id} for joint '{joint_name}' is out of inversion_array bounds "
                    f"(len={len(self.inversion_array)})."
                )
            direction_raw = self.inversion_array[motor_id - 1]
            direction = 1 if float(direction_raw) >= 0 else -1

            if joint_name not in self.joint_limits_cfg:
                raise ValueError(f"joint_limits_rad_by_joint missing '{joint_name}'")
            lo, hi = self.joint_limits_cfg[joint_name]
            lo = float(lo)
            hi = float(hi)

            kp = float(self.kp_by_joint.get(joint_name, self.kp_default))
            kd = float(self.kd_by_joint.get(joint_name, self.kd_default))
            max_vel = float(self.max_vel_by_joint.get(joint_name, self.max_vel_default))

            if joint_name not in self.default_joint_pos_cfg:
                raise ValueError(f"default_joint_pos_real_rad_by_joint missing '{joint_name}'")
            self.default_joint_pos_real[idx] = float(self.default_joint_pos_cfg[joint_name])
            self.joint_action_scales_real[idx] = float(self.action_scale_by_joint.get(joint_name, 1.0))
            self.max_vel_real[idx] = max_vel
            joint_limits_real.append((lo, hi))

            st = JointMotorState(
                joint_name=joint_name,
                motor_id=motor_id,
                model=model,
                direction=direction,
                limit_lo=lo,
                limit_hi=hi,
                kp=kp,
                kd=kd,
                max_vel_rad_s=max_vel,
                is_ankle=(joint_name in self.ankle_joint_names),
                motor_name=f"motor_{motor_id}",
            )
            self.states.append(st)
            self._state_by_joint[joint_name] = st

        self.joint_limits_real = np.asarray(joint_limits_real, dtype=float)

        self.bus: RobstrideBus | None = None
        self.connected = False
        self.lock = threading.RLock()
        self._state_by_mid: dict[int, JointMotorState] = {st.motor_id: st for st in self.states}

        self.safety_enabled = bool(cfg.get("safety_enabled", True))
        self.safety_read_hz = cfg.get("safety_read_hz", None)
        self.safety_max_jump_deg = float(cfg.get("safety_max_jump_deg", 90.0))
        self.safety_max_jump_rad = math.radians(self.safety_max_jump_deg)
        self.safety_tripped = False
        self.safety_reason: str | None = None

    def _trip_safety(self, reason: str) -> None:
        if self.safety_tripped:
            return
        self.safety_tripped = True
        self.safety_reason = reason
        print(reason)

    def _assert_safe(self) -> None:
        if self.safety_tripped:
            raise RuntimeError(self.safety_reason or "Safety monitor tripped")

    def _report_safety_mode(self, control_hz_guess: float = 60.0) -> None:
        if not self.safety_enabled:
            print("[SAFETY] synchronous joint-state checks disabled")
            return
        print(
            f"[SAFETY] synchronous joint-state checks enabled: control_hz={float(control_hz_guess):.1f}, "
            f"max_jump_deg={self.safety_max_jump_deg:.1f}"
        )

    def connect(self) -> bool:
        motors = {st.motor_name: Motor(id=st.motor_id, model=st.model) for st in self.states}
        calibration = {st.motor_name: {"direction": 1, "homing_offset": 0.0} for st in self.states}
        try:
            try:
                self.bus = RobstrideBus(self.can_channel, motors, calibration, bitrate=self.bitrate)
            except TypeError:
                self.bus = RobstrideBus(self.can_channel, motors, calibration)
            with self.lock:
                self.bus.connect(handshake=True)

            for st in self.states:
                print(
                    f"[CONNECT] {st.joint_name} id={st.motor_id} model={st.model} "
                    f"dir={st.direction:+d} limits=[{st.limit_lo:.3f},{st.limit_hi:.3f}]"
                )
                with self.lock:
                    self.bus.enable(st.motor_name)
                time.sleep(0.10)
                self._set_mode_zero(st)

                with self.lock:
                    p, v, tq, temp = self.bus.read_operation_frame(st.motor_name)
                st.position_phys = to_scalar_float(p)
                st.velocity_phys = to_scalar_float(v)
                st.torque_nm = to_scalar_float(tq)
                st.temp_c = to_scalar_float(temp)
                raw_motor_logical = st.position_phys / float(st.direction)
                st.startup_motor_offset_rad = offset_to_pi(raw_motor_logical)
                now = time.time()
                self._update_joint_from_motor(st, now, initialize=True)
                st.commanded_joint = st.joint_pos
                st.last_cmd_phys = st.position_phys

                if not self.dry_run:
                    with self.lock:
                        self.bus.write_operation_frame(st.motor_name, st.position_phys, st.kp, st.kd, 0.0, 0.0)
                time.sleep(0.03)

            self.connected = True
            self._report_safety_mode(control_hz_guess=float(self.cfg.get("control_hz", 60.0)))
            print(f"Robot interface connected on {self.can_channel} @ {self.bitrate}. dry_run={self.dry_run}")
            return True
        except Exception as exc:
            print(f"Robot interface connection failed: {exc}")
            self.connected = False
            return False

    def shutdown(self) -> None:
        if self.bus and self.connected:
            try:
                if not self.dry_run:
                    for st in self.states:
                        try:
                            with self.lock:
                                self.bus.write_operation_frame(st.motor_name, st.position_phys, st.kp, st.kd, 0.0, 0.0)
                        except Exception:
                            pass
                    time.sleep(0.05)
                    for st in self.states:
                        try:
                            with self.lock:
                                self.bus.disable(st.motor_name)
                        except Exception:
                            pass
                try:
                    with self.lock:
                        self.bus.disconnect(disable_torque=False)
                except Exception:
                    pass
            finally:
                self.connected = False

    def read_feedback(self) -> None:
        self._assert_safe()
        if not self.connected or self.bus is None:
            return
        now = time.time()
        for st in self.states:
            try:
                prev_joint_pos = float(st.joint_pos)
                had_prev = st.last_read_time > 0.0
                with self.lock:
                    p, v, tq, temp = self.bus.read_operation_frame(st.motor_name)
                st.position_phys = to_scalar_float(p)
                st.velocity_phys = to_scalar_float(v)
                st.torque_nm = to_scalar_float(tq)
                st.temp_c = to_scalar_float(temp)
                measured_joint = self._update_joint_from_motor(st, now, initialize=False)
                self._check_joint_state_safety(st, measured_joint, prev_joint_pos, had_prev)
                self._assert_safe()
                st.last_error = None
            except Exception as exc:
                st.last_error = str(exc)

    def write_joint_targets(self, joint_targets_real: np.ndarray) -> None:
        self._assert_safe()
        if not self.connected or self.bus is None:
            return
        targets = np.asarray(joint_targets_real, dtype=float).reshape(-1)
        if targets.shape[0] != len(self.states):
            raise ValueError(f"Expected {len(self.states)} targets, got {targets.shape[0]}")

        for idx, st in enumerate(self.states):
            joint_cmd = clamp(float(targets[idx]), st.limit_lo, st.limit_hi)
            try:
                physical_target = self._joint_command_to_motor_physical(st, joint_cmd)
                if not self.dry_run:
                    with self.lock:
                        self.bus.write_operation_frame(st.motor_name, physical_target, st.kp, st.kd, 0.0, 0.0)
                st.commanded_joint = joint_cmd
                st.last_cmd_phys = physical_target
                st.last_error = None
            except Exception as exc:
                st.last_error = str(exc)

    def joint_vectors_real(self) -> tuple[np.ndarray, np.ndarray]:
        joint_pos_real = np.asarray([st.joint_pos for st in self.states], dtype=float)
        joint_vel_real = np.asarray([st.joint_vel for st in self.states], dtype=float)
        return joint_pos_real, joint_vel_real

    def temperatures_real(self) -> np.ndarray:
        return np.asarray([st.temp_c for st in self.states], dtype=float)

    def get_joint_state(self, joint_name: str) -> JointMotorState | None:
        return self._state_by_joint.get(joint_name)

    def _set_mode_zero(self, st: JointMotorState) -> None:
        assert self.bus is not None
        param_id, _, _ = ParameterType.MODE
        value_buffer = struct.pack("<bBH", int(0), 0, 0)
        data = struct.pack("<HH", param_id, 0x00) + value_buffer
        device_id = self.bus.motors[st.motor_name].id
        with self.lock:
            self.bus.transmit(CommunicationType.WRITE_PARAMETER, self.bus.host_id, device_id, data)
        time.sleep(0.05)

    def _update_joint_from_motor(self, st: JointMotorState, now: float, initialize: bool = False) -> float:
        raw_motor_logical = st.position_phys / float(st.direction)
        motor_logical = raw_motor_logical - st.startup_motor_offset_rad
        if st.is_ankle:
            guess = st.joint_pos if not initialize else 0.0
            measured_joint = self.ankle_mapper.motor_logical_rad_to_ankle_rad(motor_logical, guess)
            joint_pos = clamp(measured_joint, st.limit_lo, st.limit_hi)
            if initialize or st.last_read_time <= 0.0:
                st.joint_vel = 0.0
            else:
                dt = now - st.last_read_time
                if dt > 1e-4:
                    st.joint_vel = (joint_pos - st.joint_pos) / dt
            st.joint_pos = joint_pos
        else:
            measured_joint = motor_logical
            st.joint_pos = clamp(motor_logical, st.limit_lo, st.limit_hi)
            st.joint_vel = st.velocity_phys / float(st.direction)
        st.last_read_time = now
        return float(measured_joint)

    def _check_joint_state_safety(
        self,
        st: JointMotorState,
        measured_joint: float,
        prev_joint_pos: float,
        had_prev: bool,
    ) -> None:
        if not self.safety_enabled:
            return
        if measured_joint < st.limit_lo or measured_joint > st.limit_hi:
            self._trip_safety(
                f"[SAFETY] {st.joint_name} (motor {st.motor_id}) out of limits: "
                f"pos={measured_joint:.4f} rad, limits=[{st.limit_lo:.4f},{st.limit_hi:.4f}]"
            )
            return
        if had_prev:
            jump = abs(measured_joint - prev_joint_pos)
            if jump > self.safety_max_jump_rad:
                self._trip_safety(
                    f"[SAFETY] {st.joint_name} (motor {st.motor_id}) jump too large: "
                    f"|dpos|={math.degrees(jump):.2f} deg (threshold={self.safety_max_jump_deg:.2f} deg)"
                )

    def _joint_command_to_motor_physical(self, st: JointMotorState, joint_cmd_rad: float) -> float:
        joint_cmd_rad = clamp(joint_cmd_rad, st.limit_lo, st.limit_hi)
        raw_motor_guess_logical = st.position_phys / float(st.direction)
        motor_guess_logical = raw_motor_guess_logical - st.startup_motor_offset_rad
        if st.is_ankle:
            motor_target_logical = self.ankle_mapper.ankle_rad_to_motor_logical_rad(joint_cmd_rad, motor_guess_logical)
        else:
            motor_target_logical = joint_cmd_rad
        raw_motor_target_logical = motor_target_logical + st.startup_motor_offset_rad
        return float(raw_motor_target_logical * float(st.direction))
