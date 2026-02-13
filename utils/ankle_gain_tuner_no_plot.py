#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ankle gain tuner (no plotting) for RobStride motors 5 and 10.

This is a CLI-only variant of gain_tuner:
- No matplotlib / no live plots
- Interactive commands for select/goto/step/sine/hold/kp/kd/status
- Uses the ankle mapping flow from utils/ankle_fv.py (Freudenstein linkage)
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

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
        print(f"Failed to import RobStride SDK: {e}")
        sys.exit(1)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def to_scalar_float(x) -> float:
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


# -------------------- Mapping constants (from ankle_fv.py) --------------------
L1 = 6.5625
L2 = 1.875
L3 = 6.5
L4 = 1.79

K1 = L1 / L4
K2 = L1 / L2
K3 = (L2 ** 2 - L3 ** 2 + L4 ** 2 + L1 ** 2) / (2 * L2 * L4)

THETA2_OFFSET_RAD = math.radians(90.0)
T4_TO_MOTOR_OFFSET_RAD = -math.radians(90.0)
ANKLE_TO_THETA2_SIGN = +1


_HAS_SCIPY = False
try:
    from scipy.optimize import fsolve  # type: ignore

    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


def solve_foot_to_motor(target_foot_deg: float, t2_guess_rad: float) -> float:
    theta4 = math.radians(float(target_foot_deg))

    def f(t2: float) -> float:
        return K1 * math.cos(theta4) - K2 * math.cos(t2) - math.cos(t2 - theta4) + K3

    if _HAS_SCIPY:
        sol = fsolve(lambda x: f(to_scalar_float(x)), [float(t2_guess_rad)], xtol=1e-10, maxfev=100)
        return float(sol[0])

    t2 = float(t2_guess_rad)
    for _ in range(50):
        ft = f(t2)
        dft = K2 * math.sin(t2) + math.sin(t2 - theta4)
        if abs(dft) < 1e-12:
            break
        step = ft / dft
        t2 -= step
        if abs(step) < 1e-12:
            break
    return float(t2)


def solve_motor_to_foot(t2_rad: float, t4_guess_rad: float) -> float:
    def f(t4: float) -> float:
        return K1 * math.cos(t4) - K2 * math.cos(t2_rad) - math.cos(t2_rad - t4) + K3

    if _HAS_SCIPY:
        sol = fsolve(lambda x: f(to_scalar_float(x)), [float(t4_guess_rad)], xtol=1e-10, maxfev=100)
        return float(sol[0])

    t4 = float(t4_guess_rad)
    for _ in range(50):
        ft = f(t4)
        dft = -K1 * math.sin(t4) - math.sin(t2_rad - t4)
        if abs(dft) < 1e-12:
            break
        step = ft / dft
        t4 -= step
        if abs(step) < 1e-12:
            break
    return float(t4)


# -------------------- Ankle hardware config --------------------
ANKLE_IDS = (5, 10)
MOTOR_MODEL_BY_ID: Dict[int, str] = {5: "rs-02", 10: "rs-02"}
INVERSION_ARRAY = [1, 1, 1, 1, 1, -1, -1, -1, -1, -1]
INVERSION_BY_ID: Dict[int, int] = {i + 1: INVERSION_ARRAY[i] for i in range(len(INVERSION_ARRAY))}
ANKLE_LIMITS_RAD: Dict[int, Tuple[float, float]] = {5: (-0.6, 0.6), 10: (-0.6, 0.6)}


@dataclass
class Excitation:
    mode: str = "none"
    amp_deg: float = 0.0
    freq_hz: float = 0.0
    t0: float = 0.0
    duration_s: Optional[float] = None
    center_deg: float = 0.0


@dataclass
class AnkleState:
    id: int
    name: str
    model: str
    direction: int
    limit_lo_deg: float
    limit_hi_deg: float
    kp: float = 10.0
    kd: float = 0.2
    ramp_deg_s: float = 30.0
    target_deg: float = 0.0
    commanded_deg: float = 0.0
    motor_pos_rad: float = 0.0
    motor_vel: float = 0.0
    motor_torque: float = 0.0
    motor_temp_c: float = 0.0
    last_cmd_motor_rad: float = 0.0
    last_error: Optional[str] = None
    excitation: Excitation = field(default_factory=Excitation)


class AnkleGainTunerNoPlot:
    def __init__(
        self,
        motor_ids: List[int],
        channel: str = "can0",
        bitrate: int = 1_000_000,
        hz: float = 60.0,
        ramp_deg_s: float = 30.0,
    ):
        self.channel = channel
        self.bitrate = int(bitrate)
        self.hz = float(hz)
        self.dt = 1.0 / self.hz
        self.lock = threading.Lock()

        self.states: Dict[int, AnkleState] = {}
        for mid in sorted(motor_ids):
            if mid not in ANKLE_IDS:
                raise ValueError(f"Unsupported motor id {mid}. Supported: {list(ANKLE_IDS)}")
            lo, hi = ANKLE_LIMITS_RAD[mid]
            self.states[mid] = AnkleState(
                id=mid,
                name=f"motor_{mid}",
                model=MOTOR_MODEL_BY_ID[mid],
                direction=int(INVERSION_BY_ID[mid]),
                limit_lo_deg=math.degrees(lo),
                limit_hi_deg=math.degrees(hi),
                ramp_deg_s=float(ramp_deg_s),
            )

        self.selected: Set[int] = set(self.states.keys())
        self.running = True
        self.connected = False
        self.bus: Optional[RobstrideBus] = None
        self._control_thread: Optional[threading.Thread] = None

    def _set_mode_0(self, motor_id: int):
        assert self.bus is not None
        param_id, _, _ = ParameterType.MODE
        value_buffer = struct.pack("<bBH", int(0), 0, 0)
        data = struct.pack("<HH", param_id, 0x00) + value_buffer
        device_id = self.bus.motors[f"motor_{motor_id}"].id
        self.bus.transmit(CommunicationType.WRITE_PARAMETER, self.bus.host_id, device_id, data)
        time.sleep(0.05)

    def _ankle_deg_to_motor_logical(self, ankle_deg: float, motor_guess_logical: float) -> float:
        theta2_model = THETA2_OFFSET_RAD + ANKLE_TO_THETA2_SIGN * math.radians(float(ankle_deg))
        t4_guess_model = motor_guess_logical - T4_TO_MOTOR_OFFSET_RAD
        t4_model_raw = solve_motor_to_foot(theta2_model, t4_guess_model)
        t4_model = unwrap_to_near(t4_model_raw, t4_guess_model)
        motor_target = t4_model + T4_TO_MOTOR_OFFSET_RAD
        return unwrap_to_near(motor_target, motor_guess_logical)

    def _motor_logical_to_ankle_deg(self, motor_logical: float, ankle_guess_deg: float = 0.0) -> float:
        theta4_model = motor_logical - T4_TO_MOTOR_OFFSET_RAD
        theta2_guess = THETA2_OFFSET_RAD + ANKLE_TO_THETA2_SIGN * math.radians(float(ankle_guess_deg))
        theta2_raw = solve_foot_to_motor(math.degrees(theta4_model), theta2_guess)
        theta2 = unwrap_to_near(theta2_raw, theta2_guess)
        ankle_rad = (theta2 - THETA2_OFFSET_RAD) / float(ANKLE_TO_THETA2_SIGN)
        return float(math.degrees(ankle_rad))

    def connect(self) -> bool:
        motors = {st.name: Motor(id=mid, model=st.model) for mid, st in self.states.items()}
        calib = {st.name: {"direction": 1, "homing_offset": 0.0} for st in self.states.values()}

        try:
            try:
                self.bus = RobstrideBus(self.channel, motors, calib, bitrate=self.bitrate)
            except TypeError:
                self.bus = RobstrideBus(self.channel, motors, calib)

            print(f"Connecting to {self.channel} (bitrate={self.bitrate})...")
            self.bus.connect(handshake=True)

            with self.lock:
                for mid in sorted(self.states.keys()):
                    st = self.states[mid]
                    self.bus.enable(st.name)
                    time.sleep(0.15)
                    self._set_mode_0(mid)

                    p, v, tq, temp = self.bus.read_operation_frame(st.name)
                    st.motor_pos_rad = to_scalar_float(p)
                    st.motor_vel = to_scalar_float(v)
                    st.motor_torque = to_scalar_float(tq)
                    st.motor_temp_c = to_scalar_float(temp)

                    motor_logical = st.motor_pos_rad / float(st.direction)
                    ankle_now = self._motor_logical_to_ankle_deg(motor_logical, ankle_guess_deg=0.0)
                    ankle_now = clamp(ankle_now, st.limit_lo_deg, st.limit_hi_deg)
                    st.target_deg = ankle_now
                    st.commanded_deg = ankle_now
                    st.excitation = Excitation()

                    self.bus.write_operation_frame(st.name, float(st.motor_pos_rad), st.kp, st.kd, 0.0, 0.0)
                    st.last_cmd_motor_rad = float(st.motor_pos_rad)

                    print(
                        f"[ID {mid}] dir={st.direction:+d} model={st.model} "
                        f"ankle={ankle_now:+.2f}deg limits=[{st.limit_lo_deg:+.1f},{st.limit_hi_deg:+.1f}]deg"
                    )

            self.connected = True
            self.running = True
            self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
            self._control_thread.start()
            print("Connected. Control loop started.")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            self.connected = False
            return False

    def _control_loop(self):
        next_t = time.perf_counter()
        while self.running and self.connected and self.bus is not None:
            now = time.perf_counter()
            if now < next_t:
                time.sleep(next_t - now)
            else:
                next_t = now
            self.control_step(self.dt)
            next_t += self.dt

    def control_step(self, dt: float):
        if not (self.running and self.connected and self.bus):
            return
        dt = float(clamp(dt, 0.0, 0.05))
        now = time.time()

        with self.lock:
            for st in self.states.values():
                ex = st.excitation
                if ex.mode == "sine":
                    if ex.duration_s is not None and (now - ex.t0) >= ex.duration_s:
                        st.excitation = Excitation()
                        st.target_deg = ex.center_deg
                    else:
                        st.target_deg = ex.center_deg + ex.amp_deg * math.sin(2.0 * math.pi * ex.freq_hz * (now - ex.t0))

                st.target_deg = clamp(st.target_deg, st.limit_lo_deg, st.limit_hi_deg)
                max_step = st.ramp_deg_s * dt
                delta = st.target_deg - st.commanded_deg
                if abs(delta) <= max_step:
                    st.commanded_deg = st.target_deg
                else:
                    st.commanded_deg += math.copysign(max_step, delta)
                st.commanded_deg = clamp(st.commanded_deg, st.limit_lo_deg, st.limit_hi_deg)

                motor_now_logical = st.motor_pos_rad / float(st.direction)
                try:
                    motor_target_logical = self._ankle_deg_to_motor_logical(st.commanded_deg, motor_now_logical)
                    physical_target = motor_target_logical * float(st.direction)

                    self.bus.write_operation_frame(st.name, float(physical_target), float(st.kp), float(st.kd), 0.0, 0.0)
                    st.last_cmd_motor_rad = float(physical_target)

                    p, v, tq, temp = self.bus.read_operation_frame(st.name)
                    st.motor_pos_rad = to_scalar_float(p)
                    st.motor_vel = to_scalar_float(v)
                    st.motor_torque = to_scalar_float(tq)
                    st.motor_temp_c = to_scalar_float(temp)
                    st.last_error = None
                except Exception as e:
                    st.last_error = str(e)

    # -------------------- Commands --------------------
    def select(self, motor_id: Optional[int]):
        if motor_id is None:
            self.selected = set(self.states.keys())
            print(f"Selected all: {sorted(self.selected)}")
            return
        if motor_id not in self.states:
            print(f"Motor {motor_id} not connected. Connected: {sorted(self.states.keys())}")
            return
        self.selected = {motor_id}
        print(f"Selected motor: {motor_id}")

    def set_kp(self, kp: float):
        kp = float(kp)
        if not (0.0 <= kp <= 5000.0):
            print("kp out of range (0..5000)")
            return
        with self.lock:
            for mid in self.selected:
                self.states[mid].kp = kp
        print(f"Set kp={kp:.2f} for {sorted(self.selected)}")

    def set_kd(self, kd: float):
        kd = float(kd)
        if not (0.0 <= kd <= 100.0):
            print("kd out of range (0..100)")
            return
        with self.lock:
            for mid in self.selected:
                self.states[mid].kd = kd
        print(f"Set kd={kd:.3f} for {sorted(self.selected)}")

    def set_ramp(self, ramp_deg_s: float):
        ramp_deg_s = float(clamp(ramp_deg_s, 1.0, 720.0))
        with self.lock:
            for mid in self.selected:
                self.states[mid].ramp_deg_s = ramp_deg_s
        print(f"Set ramp={ramp_deg_s:.1f} deg/s for {sorted(self.selected)}")

    def hold(self):
        with self.lock:
            for mid in self.selected:
                st = self.states[mid]
                st.excitation = Excitation()
                motor_logical = st.motor_pos_rad / float(st.direction)
                ankle_now = self._motor_logical_to_ankle_deg(motor_logical, ankle_guess_deg=st.commanded_deg)
                ankle_now = clamp(ankle_now, st.limit_lo_deg, st.limit_hi_deg)
                st.target_deg = ankle_now
                st.commanded_deg = ankle_now
        print(f"Hold at current pose for {sorted(self.selected)}")

    def goto(self, ankle_deg: float):
        with self.lock:
            for mid in self.selected:
                st = self.states[mid]
                st.excitation = Excitation()
                st.target_deg = clamp(float(ankle_deg), st.limit_lo_deg, st.limit_hi_deg)
        print(f"Goto ankle={ankle_deg:+.2f} deg (clamped) for {sorted(self.selected)}")

    def step(self, delta_deg: float):
        with self.lock:
            for mid in self.selected:
                st = self.states[mid]
                st.excitation = Excitation()
                st.target_deg = clamp(st.target_deg + float(delta_deg), st.limit_lo_deg, st.limit_hi_deg)
        print(f"Step {delta_deg:+.2f} deg for {sorted(self.selected)}")

    def sine(self, amp_deg: float, freq_hz: float, duration_s: Optional[float]):
        amp_deg = float(clamp(amp_deg, 0.0, 45.0))
        freq_hz = float(clamp(freq_hz, 0.1, 5.0))
        if duration_s is not None:
            duration_s = float(clamp(duration_s, 0.2, 120.0))

        with self.lock:
            now = time.time()
            for mid in self.selected:
                st = self.states[mid]
                center = clamp(st.commanded_deg, st.limit_lo_deg, st.limit_hi_deg)
                span = min(st.limit_hi_deg - center, center - st.limit_lo_deg)
                safe_amp = clamp(amp_deg, 0.0, max(0.0, span))
                st.target_deg = center
                st.excitation = Excitation(
                    mode="sine",
                    amp_deg=safe_amp,
                    freq_hz=freq_hz,
                    t0=now,
                    duration_s=duration_s,
                    center_deg=center,
                )
        dstr = f"{duration_s:.2f}s" if duration_s is not None else "infinite"
        print(f"Sine amp={amp_deg:.2f}deg freq={freq_hz:.2f}Hz duration={dstr} for {sorted(self.selected)}")

    def stop(self):
        with self.lock:
            for mid in self.selected:
                self.states[mid].excitation = Excitation()
        print(f"Stopped excitation for {sorted(self.selected)}")

    def status(self):
        with self.lock:
            print("-" * 120)
            print(
                f"{'ID':<4} {'Sel':<4} {'AnkPos':<10} {'AnkCmd':<10} {'MPos':<10} {'MCmd':<10} "
                f"{'Vel':<9} {'Tq':<9} {'Temp':<8} {'Kp':<8} {'Kd':<8} {'Dir':<4}"
            )
            print("-" * 120)
            for mid in sorted(self.states.keys()):
                st = self.states[mid]
                sel = "*" if mid in self.selected else ""
                motor_logical = st.motor_pos_rad / float(st.direction)
                ankle_pos = self._motor_logical_to_ankle_deg(motor_logical, ankle_guess_deg=st.commanded_deg)
                dir_str = "INV" if st.direction < 0 else "NOR"
                print(
                    f"{mid:<4} {sel:<4} {ankle_pos:<+10.2f} {st.commanded_deg:<+10.2f} "
                    f"{math.degrees(st.motor_pos_rad):<+10.2f} {math.degrees(st.last_cmd_motor_rad):<+10.2f} "
                    f"{st.motor_vel:<9.3f} {st.motor_torque:<9.3f} {st.motor_temp_c:<8.1f} "
                    f"{st.kp:<8.2f} {st.kd:<8.3f} {dir_str:<4}"
                )
                if st.last_error:
                    print(f"     error: {st.last_error}")
            print("-" * 120)

    def shutdown(self):
        print("Shutting down...")
        self.running = False
        if self._control_thread is not None:
            self._control_thread.join(timeout=1.0)

        if self.bus and self.connected:
            with self.lock:
                for st in self.states.values():
                    try:
                        self.bus.write_operation_frame(st.name, float(st.motor_pos_rad), float(st.kp), float(st.kd), 0.0, 0.0)
                    except Exception:
                        pass
                time.sleep(0.1)
                for st in self.states.values():
                    try:
                        self.bus.disable(st.name)
                    except Exception:
                        pass
            try:
                self.bus.disconnect()
            except Exception:
                pass
        self.connected = False
        print("Done.")


def command_loop(tuner: AnkleGainTunerNoPlot):
    print("\nCommands:")
    print("  select 5 | select 10 | select all")
    print("  kp <value> | kd <value> | ramp <deg_per_s>")
    print("  hold")
    print("  step <deg>")
    print("  goto <deg>")
    print("  sine <amp_deg> <freq_hz> [duration_s]")
    print("  stop")
    print("  status")
    print("  q\n")

    while True:
        try:
            sel = "ALL" if len(tuner.selected) == len(tuner.states) else ",".join(str(x) for x in sorted(tuner.selected))
            cmd = input(f"[{sel}] >> ").strip().lower()
            if not cmd:
                continue

            if cmd in ("q", "quit", "exit"):
                tuner.shutdown()
                os._exit(0)
            if cmd == "status":
                tuner.status()
                continue
            if cmd == "hold":
                tuner.hold()
                continue
            if cmd == "stop":
                tuner.stop()
                continue

            if cmd.startswith("select "):
                parts = cmd.split()
                if len(parts) != 2:
                    print("Usage: select 5|10|all")
                    continue
                if parts[1] == "all":
                    tuner.select(None)
                else:
                    tuner.select(int(parts[1]))
                continue

            if cmd.startswith("kp "):
                tuner.set_kp(float(cmd.split()[1]))
                continue
            if cmd.startswith("kd "):
                tuner.set_kd(float(cmd.split()[1]))
                continue
            if cmd.startswith("ramp "):
                tuner.set_ramp(float(cmd.split()[1]))
                continue
            if cmd.startswith("step "):
                tuner.step(float(cmd.split()[1]))
                continue
            if cmd.startswith("goto "):
                tuner.goto(float(cmd.split()[1]))
                continue
            if cmd.startswith("sine "):
                parts = cmd.split()
                if len(parts) not in (3, 4):
                    print("Usage: sine <amp_deg> <freq_hz> [duration_s]")
                    continue
                amp = float(parts[1])
                freq = float(parts[2])
                dur = float(parts[3]) if len(parts) == 4 else None
                tuner.sine(amp, freq, dur)
                continue

            print("Unknown command.")
        except KeyboardInterrupt:
            tuner.shutdown()
            os._exit(0)
        except Exception as e:
            print(f"[CLI] error: {e}")


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Ankle gain tuner without plotting (motors 5/10).")
    ap.add_argument("--can", default="can0", help="CAN interface (default: can0)")
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--hz", type=float, default=60.0, help="Control loop frequency")
    ap.add_argument("--ramp_deg_s", type=float, default=30.0, help="Ramp rate in ankle deg/s")
    ap.add_argument("--motors", type=int, nargs="+", default=[5, 10], help="Ankle motor IDs to connect (5 and/or 10)")
    return ap.parse_args()


def main():
    args = parse_args()
    motors = sorted(set(args.motors))
    if any(mid not in ANKLE_IDS for mid in motors):
        raise SystemExit("Only ankle motors 5 and 10 are supported.")
    if not motors:
        raise SystemExit("No motors selected.")

    tuner = AnkleGainTunerNoPlot(
        motor_ids=motors,
        channel=args.can,
        bitrate=args.bitrate,
        hz=args.hz,
        ramp_deg_s=args.ramp_deg_s,
    )

    def _sig(_signum=None, _frame=None):
        tuner.shutdown()
        os._exit(0)

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    if not tuner.connect():
        sys.exit(1)

    command_loop(tuner)


if __name__ == "__main__":
    main()
