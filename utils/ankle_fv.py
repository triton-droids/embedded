#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ankle_function_verification.py

Purpose
- Command ONLY motor ID 5 (ankle motor) using your 4-bar linkage mapping
- Read IMU roll (deg)
- Plot: desired ankle command (deg) vs IMU roll (deg) over time

Startup behavior (as requested)
1) Start IMU stream and WAIT for a valid roll sample
2) Tare IMU (make current roll = 0 deg)
3) Re-anchor ankle->motor mapping so ankle_cmd=0 deg maps to *current motor pose* (no jump)
4) Command ankle to 0 deg and hold briefly (settle) BEFORE any sine test is allowed

Notes
- This script intentionally avoids: delay measurement, torque plotting, temperature logic, multi-motor selection.
- It only controls motor ID 5.
"""

import os
import sys
import time
import math
import struct
import signal
import threading
from dataclasses import dataclass, field
from collections import deque
from typing import Optional

import numpy as np

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# -------------------- RobStride SDK imports --------------------
# Adjust path if needed (kept same pattern as your scripts)
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

# -------------------- IMU imports --------------------
try:
    from imu_read import iter_imu_samples
    IMU_AVAILABLE = True
except Exception as e:
    IMU_AVAILABLE = False
    print(f"ERROR: imu_read.py not available / failed import: {e}")


def to_scalar_float(x) -> float:
    """Robustly convert SDK returns (numpy scalars/arrays/lists) into a Python float."""
    if x is None:
        return 0.0
    if isinstance(x, (float, int)):
        return float(x)

    try:
        import numpy as np
        arr = np.asarray(x)
        if arr.ndim == 0:
            return float(arr)
        if arr.size == 1:
            return float(arr.reshape(()).item())
        # If SDK returns more than 1 value, take the first (and keep going)
        return float(arr.reshape(-1)[0])
    except Exception:
        # last resort
        return float(x)

def describe_branch(motor_now_logical: float, t4_candidate: float, name: str):
    d = math.radians(5.0)
    t2p_raw = solve_foot_to_motor(math.degrees(t4_candidate + d), t2_guess_rad=motor_now_logical)
    t2m_raw = solve_foot_to_motor(math.degrees(t4_candidate - d), t2_guess_rad=motor_now_logical)
    t2p = unwrap_to_near(t2p_raw, motor_now_logical)
    t2m = unwrap_to_near(t2m_raw, motor_now_logical)
    dp = t2p - motor_now_logical
    dm = t2m - motor_now_logical
    print(
        f"[BRANCH {name}] t4={math.degrees(t4_candidate):+.2f}deg | "
        f"dp(+5deg)={dp:+.4f}rad dm(-5deg)={dm:+.4f}rad | "
        f"opposite={dp*dm<0} minmag={min(abs(dp),abs(dm)):.4f} asym={abs(abs(dp)-abs(dm)):.4f}"
    )

# ==============================
# 4-bar linkage mapping (Freudenstein)
# ==============================
# Link lengths (your values)
L1 = 6.5625  # Ground
L2 = 1.79    # Input (Motor Crank)
L3 = 6.5     # Coupler
L4 = 1.875   # Output (Ankle)

K1 = L1 / L4
K2 = L1 / L2
K3 = (L2**2 - L3**2 + L4**2 + L1**2) / (2 * L2 * L4)

# Try SciPy fsolve if available; otherwise use Newton fallback
_HAS_SCIPY = False
try:
    from scipy.optimize import fsolve  # type: ignore
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


def solve_foot_to_motor(target_foot_deg: float, t2_guess_rad: float) -> float:
    """
    Given desired foot angle theta4 (deg), solve for motor angle theta2 (rad).
    Uses fsolve if available; otherwise Newton-Raphson with analytic derivative.
    """
    theta4 = math.radians(float(target_foot_deg))

    def f(t2: float) -> float:
        return K1 * math.cos(theta4) - K2 * math.cos(t2) - math.cos(t2 - theta4) + K3

    if _HAS_SCIPY:
        def f_wrapped(x):
            # fsolve passes x as an array (e.g., array([t2]))
            return f(to_scalar_float(x))

        sol = fsolve(f_wrapped, [float(t2_guess_rad)], xtol=1e-10, maxfev=100)
        return float(sol[0])


    # Newton fallback
    t2 = float(t2_guess_rad)
    for _ in range(30):
        ft = f(t2)
        # df/dt2 = K2*sin(t2) + sin(t2 - theta4)
        dft = (K2 * math.sin(t2)) + math.sin(t2 - theta4)
        if abs(dft) < 1e-10:
            break
        step = ft / dft
        t2 = t2 - step
        if abs(step) < 1e-10:
            break
    return float(t2)


def wrap_to_pi(x: float) -> float:
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def unwrap_to_near(x: float, ref: float) -> float:
    # choose the 2π-equivalent of x that is closest to ref
    return ref + wrap_to_pi(x - ref)


def solve_motor_to_foot(t2_rad: float, t4_guess_rad: float) -> float:
    """
    Given motor angle theta2 (rad), solve for foot/output angle theta4 (rad).
    Newton with analytic derivative.
    """
    def f(t4: float) -> float:
        return K1 * math.cos(t4) - K2 * math.cos(t2_rad) - math.cos(t2_rad - t4) + K3

    t4 = float(t4_guess_rad)
    for _ in range(40):
        ft = f(t4)
        # df/dt4 = -K1*sin(t4) - sin(t2 - t4)
        dft = -K1 * math.sin(t4) - math.sin(t2_rad - t4)
        if abs(dft) < 1e-12:
            break
        step = ft / dft
        t4 = t4 - step
        if abs(step) < 1e-12:
            break
    return float(t4)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


# ==============================
# Configuration (ONLY motor 5)
# ==============================
MOTOR_ID = 5
MOTOR_MODEL = "rs-02"  # from your map
INVERSION_ARRAY = [1, 1, 1, 1, 1, -1, -1, -1, -1, -1]
DIRECTION = INVERSION_ARRAY[MOTOR_ID - 1]  # ID 5 -> +1 in your array

# Joint limits (ankle joint space, radians)
ANKLE_LIMIT_LO_RAD = -0.6
ANKLE_LIMIT_HI_RAD = 0.6
ANKLE_LIMIT_LO_DEG = math.degrees(ANKLE_LIMIT_LO_RAD)
ANKLE_LIMIT_HI_DEG = math.degrees(ANKLE_LIMIT_HI_RAD)
ANKLE_TO_T4_SIGN = -1


@dataclass
class Excitation:
    mode: str = "none"  # "none" | "sine"
    amp_deg: float = 0.0
    freq_hz: float = 0.0
    t0: float = 0.0
    duration_s: Optional[float] = None
    center_deg: float = 0.0


@dataclass
class State:
    # ankle joint space (what you care about)
    target_deg: float = 0.0
    commanded_deg: float = 0.0

    # motor space helpers
    motor_offset_rad: float = 0.0
    foot_offset_rad: float = 0.0
    last_t2_guess_rad: Optional[float] = None

    # control
    kp: float = 10.0
    kd: float = 0.2
    ramp_deg_s: float = 30.0
    excitation: Excitation = field(default_factory=Excitation)

    # telemetry (minimal)
    motor_pos_rad: float = 0.0


class AnkleFunctionVerifier:
    def __init__(
        self,
        can_channel: str = "can0",
        bitrate: int = 1_000_000,
        control_hz: float = 60.0,
        imu_port: str = "/dev/ttyUSB0",
        imu_baud: int = 115200,
        imu_key: str = "roll_deg",
        imu_sign: int = -1,
        settle_s: float = 1.0,
    ):
        self.can_channel = can_channel
        self.bitrate = int(bitrate)
        self.control_hz = float(control_hz)
        self.dt = 1.0 / self.control_hz

        self.imu_port = imu_port
        self.imu_baud = int(imu_baud)
        self.imu_key = str(imu_key)
        self.imu_sign = int(imu_sign)
        self.settle_s = float(settle_s)

        self.lock = threading.Lock()
        self.running = True
        self.connected = False

        self.bus: Optional[RobstrideBus] = None
        self.st = State()

        # IMU shared state
        self.imu_connected = False
        self._imu_raw_deg = 0.0
        self._imu_offset_deg = 0.0  # raw - offset => zeroed
        self.imu_roll_deg = 0.0     # already zeroed + sign

        # startup gate: no sine until we do tare+zero routine
        self.ready_for_sine = False

    # ---------- IMU ----------
    def _imu_thread(self):
        if not IMU_AVAILABLE:
            return
        try:
            gen = iter_imu_samples(
                source="serial",
                port=self.imu_port,
                baud=self.imu_baud,
                rate_hz=50.0,
                include_all=True,
            )
            for sample in gen:
                if not self.running:
                    break
                raw = sample.get(self.imu_key)
                if raw is None:
                    continue
                raw = float(raw)

                with self.lock:
                    self.imu_connected = True
                    self._imu_raw_deg = raw
                    zeroed = (raw - self._imu_offset_deg) * float(self.imu_sign)
                    self.imu_roll_deg = float(zeroed)
        except Exception as e:
            print(f"[IMU] Thread crashed: {e}")

    def start_imu(self):
        if not IMU_AVAILABLE:
            raise RuntimeError("imu_read.py not available; cannot run IMU.")
        th = threading.Thread(target=self._imu_thread, daemon=True)
        th.start()

    def wait_for_imu(self, timeout_s: float = 5.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            with self.lock:
                if self.imu_connected:
                    return True
            time.sleep(0.01)
        return False

    def tare_imu_and_reanchor(self):
        """
        Make current IMU reading become 0 deg, and re-anchor mapping so ankle_cmd=0
        maps to current motor pose (no jump).
        """
        with self.lock:
            # tare IMU
            self._imu_offset_deg = self._imu_raw_deg
            self.imu_roll_deg = 0.0

            # re-anchor ankle mapping
            motor_now_logical = self.st.motor_pos_rad / float(DIRECTION)

            t4_a = solve_motor_to_foot(motor_now_logical, t4_guess_rad=0.0)
            t4_b = solve_motor_to_foot(motor_now_logical, t4_guess_rad=math.pi)

            def branch_metrics(t4_0: float):
                d = math.radians(5.0)
                t2p_raw = solve_foot_to_motor(math.degrees(t4_0 + d), t2_guess_rad=motor_now_logical)
                t2m_raw = solve_foot_to_motor(math.degrees(t4_0 - d), t2_guess_rad=motor_now_logical)
                t2p = unwrap_to_near(t2p_raw, motor_now_logical)
                t2m = unwrap_to_near(t2m_raw, motor_now_logical)
                dp = t2p - motor_now_logical
                dm = t2m - motor_now_logical
                opposite = (dp * dm) < 0
                minmag = min(abs(dp), abs(dm))
                asym = abs(abs(dp) - abs(dm))
                return opposite, minmag, asym, t4_0, dp, dm

            cands = [
                branch_metrics(t4_a),
                branch_metrics(t4_b),
            ]

            good = [c for c in cands if c[0]]
            if good:
                best = sorted(good, key=lambda x: (-x[1], x[2]))[0]
            else:
                best = sorted(cands, key=lambda x: (-x[1], x[2]))[0]

            _, _, _, chosen_t4, dp, dm = best
            self.st.foot_offset_rad = chosen_t4

            describe_branch(motor_now_logical, t4_a, "A")
            describe_branch(motor_now_logical, t4_b, "B")
            print(f"[ANCHOR] chosen foot_offset = {math.degrees(self.st.foot_offset_rad):+.2f} deg")

            self.st.last_t2_guess_rad = motor_now_logical
            self.st.motor_offset_rad = 0.0
            # reset ankle command to 0
            self.st.excitation = Excitation()
            self.st.target_deg = 0.0
            self.st.commanded_deg = 0.0

        print("[STARTUP] IMU tared (roll=0). Mapping re-anchored. Ankle target set to 0 deg.")

    # ---------- Motor / CAN ----------
    def _set_mode_0(self):
        # Mode parameter is int8 in your prior script
        param_id, _, _ = ParameterType.MODE
        value_buffer = struct.pack("<bBH", int(0), 0, 0)
        data = struct.pack("<HH", param_id, 0x00) + value_buffer

        motor_name = f"motor_{MOTOR_ID}"
        device_id = self.bus.motors[motor_name].id
        self.bus.transmit(CommunicationType.WRITE_PARAMETER, self.bus.host_id, device_id, data)

    def connect_motor(self) -> bool:
        motor_name = f"motor_{MOTOR_ID}"
        motors = {motor_name: Motor(id=MOTOR_ID, model=MOTOR_MODEL)}
        cal = {motor_name: {"direction": 1, "homing_offset": 0.0}}

        try:
            try:
                self.bus = RobstrideBus(self.can_channel, motors, cal, bitrate=self.bitrate)
            except TypeError:
                self.bus = RobstrideBus(self.can_channel, motors, cal)

            print(f"Connecting motor {MOTOR_ID} on {self.can_channel} (bitrate={self.bitrate}) ...")
            self.bus.connect(handshake=True)
            self.bus.enable(motor_name)
            time.sleep(0.1)
            self._set_mode_0()
            time.sleep(0.1)

            # Read initial motor pose
            p, v, tq, temp = self.bus.read_operation_frame(motor_name)
            p = to_scalar_float(p)
            with self.lock:
                self.st.motor_pos_rad = p
                motor_now_logical = float(p) / float(DIRECTION)

                t4_a = solve_motor_to_foot(motor_now_logical, t4_guess_rad=0.0)
                t4_b = solve_motor_to_foot(motor_now_logical, t4_guess_rad=math.pi)

                def branch_metrics(t4_0: float):
                    d = math.radians(5.0)
                    t2p_raw = solve_foot_to_motor(math.degrees(t4_0 + d), t2_guess_rad=motor_now_logical)
                    t2m_raw = solve_foot_to_motor(math.degrees(t4_0 - d), t2_guess_rad=motor_now_logical)
                    t2p = unwrap_to_near(t2p_raw, motor_now_logical)
                    t2m = unwrap_to_near(t2m_raw, motor_now_logical)
                    dp = t2p - motor_now_logical
                    dm = t2m - motor_now_logical
                    opposite = (dp * dm) < 0
                    minmag = min(abs(dp), abs(dm))
                    asym = abs(abs(dp) - abs(dm))
                    return opposite, minmag, asym, t4_0, dp, dm

                cands = [
                    branch_metrics(t4_a),
                    branch_metrics(t4_b),
                ]

                good = [c for c in cands if c[0]]
                if good:
                    best = sorted(good, key=lambda x: (-x[1], x[2]))[0]
                else:
                    best = sorted(cands, key=lambda x: (-x[1], x[2]))[0]

                _, _, _, chosen_t4, dp, dm = best
                self.st.foot_offset_rad = chosen_t4

                describe_branch(motor_now_logical, t4_a, "A")
                describe_branch(motor_now_logical, t4_b, "B")
                print(f"[ANCHOR] chosen foot_offset = {math.degrees(self.st.foot_offset_rad):+.2f} deg")

                self.st.last_t2_guess_rad = motor_now_logical
                self.st.motor_offset_rad = 0.0

                self.st.target_deg = 0.0
                self.st.commanded_deg = 0.0
                self.st.excitation = Excitation()

            # Send initial hold
            self._send_current_command()

            self.connected = True
            print("Motor connected and holding (ankle_cmd=0 deg).")
            return True
        except Exception as e:
            print(f"Motor connect failed: {e}")
            return False

    def _send_current_command(self):
        if not self.bus:
            return
        motor_name = f"motor_{MOTOR_ID}"

        # map ankle_cmd (deg) -> motor_rad target (rad)
        ankle_deg = clamp(self.st.commanded_deg, ANKLE_LIMIT_LO_DEG, ANKLE_LIMIT_HI_DEG)

        motor_now_logical = self.st.motor_pos_rad / float(DIRECTION)

        theta4_abs_rad = self.st.foot_offset_rad + ANKLE_TO_T4_SIGN * math.radians(ankle_deg)

        t2_raw = solve_foot_to_motor(math.degrees(theta4_abs_rad), t2_guess_rad=motor_now_logical)
        t2_target = unwrap_to_near(t2_raw, motor_now_logical)

        self.st.last_t2_guess_rad = t2_target

        physical_target = t2_target * float(DIRECTION)

        self.bus.write_operation_frame(
            motor_name,
            float(physical_target),
            float(self.st.kp),
            float(self.st.kd),
            0.0,
            0.0,
        )

        # update motor position (minimal telemetry) for continuity
        p, v, tq, temp = self.bus.read_operation_frame(motor_name)
        self.st.motor_pos_rad = to_scalar_float(p)

    def control_step(self, dt: float):
        if not (self.running and self.connected and self.bus):
            return

        dt = float(clamp(dt, 0.0, 0.05))
        now = time.time()

        with self.lock:
            ex = self.st.excitation
            if ex.mode == "sine":
                if ex.duration_s is not None and (now - ex.t0) >= ex.duration_s:
                    # stop sine -> hold at center
                    self.st.excitation = Excitation()
                    self.st.target_deg = ex.center_deg
                else:
                    self.st.target_deg = ex.center_deg + ex.amp_deg * math.sin(
                        2.0 * math.pi * ex.freq_hz * (now - ex.t0)
                    )

            # clamp in ankle space
            self.st.target_deg = clamp(self.st.target_deg, ANKLE_LIMIT_LO_DEG, ANKLE_LIMIT_HI_DEG)

            # ramp in ankle space
            max_step = float(self.st.ramp_deg_s) * dt
            delta = self.st.target_deg - self.st.commanded_deg
            if abs(delta) <= max_step:
                self.st.commanded_deg = self.st.target_deg
            else:
                self.st.commanded_deg += math.copysign(max_step, delta)

            self.st.commanded_deg = clamp(self.st.commanded_deg, ANKLE_LIMIT_LO_DEG, ANKLE_LIMIT_HI_DEG)

            # send command + read back
            try:
                self._send_current_command()
            except Exception as e:
                # keep going; plot can still run even if occasional read fails
                print(f"[CAN] control_step warning: {e}")

    # ---------- Commands ----------
    def goto(self, ankle_deg: float):
        with self.lock:
            self.st.excitation = Excitation()
            self.st.target_deg = float(ankle_deg)

            motor_now = self.st.motor_pos_rad / float(DIRECTION)
            theta4_abs = self.st.foot_offset_rad + ANKLE_TO_T4_SIGN * math.radians(
                clamp(ankle_deg, ANKLE_LIMIT_LO_DEG, ANKLE_LIMIT_HI_DEG)
            )
            t2_raw = solve_foot_to_motor(math.degrees(theta4_abs), t2_guess_rad=motor_now)
            t2 = unwrap_to_near(t2_raw, motor_now)
            print(f"[MAP] goto {ankle_deg:+.1f}deg -> motor_delta {(t2-motor_now):+.4f} rad")

        print(f"goto ankle={ankle_deg:.2f} deg (will clamp to limits)")

    def sine(self, amp_deg: float, freq_hz: float, duration_s: Optional[float]):
        with self.lock:
            if not self.ready_for_sine:
                print("Not ready for sine yet (startup tare+zero not finished).")
                return
            amp_deg = float(clamp(amp_deg, 0.0, 90.0))
            freq_hz = float(clamp(freq_hz, 0.1, 5.0))
            if duration_s is not None:
                duration_s = float(clamp(duration_s, 0.2, 60.0))
            center = float(self.st.commanded_deg)
            self.st.excitation = Excitation(
                mode="sine",
                amp_deg=amp_deg,
                freq_hz=freq_hz,
                t0=time.time(),
                duration_s=duration_s,
                center_deg=center,
            )
        dstr = f"{duration_s:.2f}s" if duration_s is not None else "infinite"
        print(f"sine amp={amp_deg:.2f} deg freq={freq_hz:.2f} Hz duration={dstr}")

    def stop(self):
        with self.lock:
            self.st.excitation = Excitation()
        print("stopped excitation")

    def shutdown(self):
        self.running = False
        try:
            if self.bus and self.connected:
                motor_name = f"motor_{MOTOR_ID}"
                try:
                    # final hold at current motor position
                    with self.lock:
                        p = self.st.motor_pos_rad
                    self.bus.write_operation_frame(motor_name, float(p), self.st.kp, self.st.kd, 0.0, 0.0)
                    time.sleep(0.05)
                except Exception:
                    pass
                try:
                    self.bus.disable(motor_name)
                except Exception:
                    pass
                try:
                    self.bus.disconnect()
                except Exception:
                    pass
        finally:
            self.connected = False
            print("Shutdown complete.")


# ==============================
# Plotter (main thread drives control)
# ==============================
class LivePlotter:
    def __init__(self, verifier: AnkleFunctionVerifier, window_s: float = 10.0, ui_hz: float = 60.0):
        self.v = verifier
        self.window_s = float(window_s)
        self.ui_interval_ms = int(1000.0 / float(ui_hz))

        maxlen = int(window_s * ui_hz) + 200
        self.t = deque(maxlen=maxlen)
        self.cmd = deque(maxlen=maxlen)
        self.imu = deque(maxlen=maxlen)

        self.fig, self.ax = plt.subplots(1, 1, figsize=(10, 5))
        try:
            self.fig.canvas.manager.set_window_title("Ankle Function Verification (Cmd vs IMU Roll)")
        except Exception:
            pass

        (self.l_cmd,) = self.ax.plot([], [], label="ankle cmd (deg)")
        (self.l_imu,) = self.ax.plot([], [], label="IMU roll (deg)")
        self.ax.set_xlabel("time (s)")
        self.ax.set_ylabel("deg")
        self.ax.grid(True)
        self.ax.legend(loc="upper right")

        self._t0 = time.time()
        self._last_update_t = time.time()
        self._accum = 0.0
        self._max_control_iters_per_ui = 20
        self._ani = None

        self.fig.canvas.mpl_connect("close_event", self._on_close)

    def _on_close(self, _evt):
        try:
            self.v.shutdown()
        finally:
            os._exit(0)

    def _update(self, _frame):
        # drive control
        now = time.time()
        dt_wall = clamp(now - self._last_update_t, 0.0, 0.1)
        self._last_update_t = now

        self._accum += dt_wall
        iters = 0
        while self._accum >= self.v.dt and iters < self._max_control_iters_per_ui:
            self.v.control_step(self.v.dt)
            self._accum -= self.v.dt
            iters += 1
        if iters >= self._max_control_iters_per_ui:
            self._accum = 0.0

        # sample
        t_s = time.time() - self._t0
        with self.v.lock:
            cmd_deg = float(self.v.st.commanded_deg)
            imu_deg = float(self.v.imu_roll_deg)

        self.t.append(t_s)
        self.cmd.append(cmd_deg)
        self.imu.append(imu_deg)

        if len(self.t) < 2:
            return (self.l_cmd, self.l_imu)

        x = list(self.t)
        self.l_cmd.set_data(x, list(self.cmd))
        self.l_imu.set_data(x, list(self.imu))

        xmax = x[-1]
        xmin = max(0.0, xmax - self.window_s)
        self.ax.set_xlim(xmin, xmax)

        # y autoscale from data
        ys = list(self.cmd) + list(self.imu)
        y_min = min(ys)
        y_max = max(ys)
        if y_min != y_max:
            pad = 0.05 * (y_max - y_min)
            self.ax.set_ylim(y_min - pad, y_max + pad)

        return (self.l_cmd, self.l_imu)

    def show(self):
        self._ani = FuncAnimation(
            self.fig,
            self._update,
            interval=self.ui_interval_ms,
            blit=False,
            cache_frame_data=False,
        )
        plt.tight_layout()
        plt.show()


# ==============================
# CLI (minimal)
# ==============================
def command_loop(v: AnkleFunctionVerifier):
    print("\nCommands:")
    print("  tare                       (tare IMU to 0, re-anchor mapping, set ankle target=0)")
    print("  goto <deg>                 (ankle joint command, degrees)")
    print("  sine <amp_deg> <freq_hz> [duration_s]")
    print("  stop")
    print("  q\n")

    while True:
        try:
            cmd = input(">> ").strip().lower()
            if not cmd:
                continue
            if cmd in ("q", "quit", "exit"):
                v.shutdown()
                os._exit(0)
            if cmd == "tare":
                v.tare_imu_and_reanchor()
                # keep it at zero after tare
                v.goto(0.0)
                continue
            if cmd == "stop":
                v.stop()
                continue
            if cmd.startswith("goto "):
                parts = cmd.split()
                if len(parts) != 2:
                    print("Usage: goto <deg>")
                    continue
                v.goto(float(parts[1]))
                continue
            if cmd.startswith("sine "):
                parts = cmd.split()
                if len(parts) not in (3, 4):
                    print("Usage: sine <amp_deg> <freq_hz> [duration_s]")
                    continue
                amp = float(parts[1])
                freq = float(parts[2])
                dur = float(parts[3]) if len(parts) == 4 else None
                v.sine(amp, freq, dur)
                continue

            if cmd.startswith("kp "):
                v.st.kp = float(cmd.split()[1])
                print(f"kp set to {v.st.kp}")
                continue
            if cmd.startswith("kd "):
                v.st.kd = float(cmd.split()[1])
                print(f"kd set to {v.st.kd}")
                continue


            print("Unknown command.")
        except KeyboardInterrupt:
            v.shutdown()
            os._exit(0)
        except Exception as e:
            print(f"[CLI] error: {e}")


# ==============================
# Main
# ==============================
def main():
    import argparse

    ap = argparse.ArgumentParser(description="Verify ankle mapping: ankle_cmd(deg) vs IMU roll(deg) for motor ID 5.")
    ap.add_argument("--can", default="can0")
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--control_hz", type=float, default=60.0)
    ap.add_argument("--imu_port", default="/dev/ttyUSB0")
    ap.add_argument("--imu_baud", type=int, default=115200)
    ap.add_argument("--imu_key", default="roll_deg", help="Key from imu_read samples (default: roll_deg)")
    ap.add_argument("--imu_sign", type=int, default=1, choices=[-1, 1], help="Flip IMU sign if needed")
    ap.add_argument("--window", type=float, default=10.0, help="Plot time window (seconds)")
    ap.add_argument("--ui_hz", type=float, default=60.0, help="Plot update rate (Hz)")
    ap.add_argument("--settle_s", type=float, default=1.0, help="Hold-at-zero settle time before allowing sine")
    args = ap.parse_args()

    v = AnkleFunctionVerifier(
        can_channel=args.can,
        bitrate=args.bitrate,
        control_hz=args.control_hz,
        imu_port=args.imu_port,
        imu_baud=args.imu_baud,
        imu_key=args.imu_key,
        imu_sign=args.imu_sign,
        settle_s=args.settle_s,
    )

    def _sig(_signum=None, _frame=None):
        v.shutdown()
        os._exit(0)

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    # IMU first, then motor, then startup tare+zero routine
    v.start_imu()
    if not v.wait_for_imu(timeout_s=6.0):
        print("ERROR: IMU not detected (no valid samples). Check port/baud and imu_key.")
        v.shutdown()
        sys.exit(1)

    if not v.connect_motor():
        v.shutdown()
        sys.exit(1)

    # Startup routine (requested)
    v.tare_imu_and_reanchor()
    v.goto(0.0)

    with v.lock:
        motor_now = v.st.motor_pos_rad / float(DIRECTION)
        t2p = unwrap_to_near(
            solve_foot_to_motor(math.degrees(v.st.foot_offset_rad + math.radians(10.0)), motor_now),
            motor_now
        )
        t2m = unwrap_to_near(
            solve_foot_to_motor(math.degrees(v.st.foot_offset_rad - math.radians(10.0)), motor_now),
            motor_now
        )
    print(f"[SANITY] motor_now={motor_now:.3f} rad | +10deg -> {t2p:.3f} | -10deg -> {t2m:.3f}")
    print(f"[SANITY] deltas: +{(t2p-motor_now):+.3f} rad, {(t2m-motor_now):+.3f} rad")

    # settle at zero BEFORE allowing sine
    t_end = time.time() + float(v.settle_s)
    while time.time() < t_end:
        v.control_step(v.dt)
        time.sleep(v.dt)

    v.ready_for_sine = True
    print("[STARTUP] Ready. You can run sine now.")

    threading.Thread(target=command_loop, args=(v,), daemon=True).start()

    plotter = LivePlotter(v, window_s=args.window, ui_hz=args.ui_hz)
    plotter.show()


if __name__ == "__main__":
    main()
