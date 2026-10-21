"""Simple gain tuning helper utilities.

These are general-purpose helpers (PD from natural frequency/damping, Ziegler-Nichols rule).
If you have the original `robstride_control/gain_tuner.py` to port, paste it and I will adapt precisely.
"""
from __future__ import annotations

from typing import Tuple


def pd_from_natural_freq(wn_rad_s: float, damping_ratio: float = 1.0, inertia: float = 1.0) -> Tuple[float, float]:
    """Compute PD gains for a second-order target: J*s^2 + Kd*s + Kp.

    Kp = J * wn^2
    Kd = 2 * zeta * J * wn

    Args:
        wn_rad_s: desired natural frequency (rad/s)
        damping_ratio: desired damping ratio (zeta)
        inertia: inertia or effective mass (J)

    Returns:
        (kp, kd)
    """
    kp = inertia * (wn_rad_s ** 2)
    kd = 2.0 * damping_ratio * inertia * wn_rad_s
    return kp, kd


def ziegler_nichols_pid(ku: float, tu: float) -> Tuple[float, float, float]:
    """Ziegler–Nichols ultimate gain method.

    Given ultimate gain `ku` and oscillation period `tu`, return PID (Kp, Ki, Kd).
    This implements the classic Ziegler–Nichols table for a PID controller.
    """
    kp = 0.6 * ku
    ki = 1.2 * ku / tu
    kd = 0.075 * ku * tu
    return kp, ki, kd


def suggest_initial_pd(omega_hz: float = 1.0, damping_ratio: float = 0.7, inertia: float = 1.0) -> Tuple[float, float]:
    """Convenience: specify desired bandwidth in Hz and get PD gains.

    Converts bandwidth (Hz) to natural frequency and calls `pd_from_natural_freq`.
    """
    wn = 2.0 * 3.141592653589793 * omega_hz
    return pd_from_natural_freq(wn, damping_ratio, inertia)


# --- RobStride-like temperature derate helpers (ported) ---
# Temperature protection thresholds (°C) — tune to your motor’s real safe limits
TEMP_DERATE_START_C = 65.0   # start slowing motion
TEMP_HOLD_C = 75.0           # freeze at current pose
TEMP_DISABLE_C = 85.0        # disable motor
TEMP_REENABLE_C = 70.0       # must cool below this to re-enable (hysteresis)

DERATE_MIN_SCALE = 0.20      # minimum motion scale at/above disable threshold


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def motion_scale_from_temp(temp_c: float) -> float:
    """Motion derating scale for ramp rate (NOT gains).

    1.0 until `TEMP_DERATE_START_C`, then linearly down to `DERATE_MIN_SCALE` at
    `TEMP_DISABLE_C`.
    """
    if temp_c <= TEMP_DERATE_START_C:
        return 1.0
    if temp_c >= TEMP_DISABLE_C:
        return DERATE_MIN_SCALE
    frac = (temp_c - TEMP_DERATE_START_C) / (TEMP_DISABLE_C - TEMP_DERATE_START_C)
    return clamp(1.0 - frac * (1.0 - DERATE_MIN_SCALE), DERATE_MIN_SCALE, 1.0)


def temp_state_from_temp(temp_c: float) -> str:
    """Return temperature state string: 'OK'|'DERATE'|'HOLD'|'DISABLED'."""
    if temp_c >= TEMP_DISABLE_C:
        return "DISABLED"
    if temp_c >= TEMP_HOLD_C:
        return "HOLD"
    if temp_c >= TEMP_DERATE_START_C:
        return "DERATE"
    return "OK"


# ----- High-level GainTuner that runs a control loop (uses generic client API) -----
import threading
import math
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, List, Tuple
import warnings

from .motor import MotorConfig


@dataclass
class Excitation:
    mode: str = "none"  # "none" | "sine"
    amp_rad: float = 0.0
    freq_hz: float = 0.0
    t0: float = 0.0
    duration_s: Optional[float] = None
    center_rad: float = 0.0


@dataclass
class MotorState:
    joint_name: str
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    temperature: float = 0.0
    kp: float | None = None
    kd: float | None = None
    target_rad: float = 0.0
    commanded_target_rad: float = 0.0
    enabled: bool = True
    temp_state: str = "OK"
    last_disable_t: float = 0.0


class GainTuner:
    """Generic gain-tuner control loop that can be driven by any motor client.

    The `client` must provide `get_motor_status(joint_names)` returning a reply
    with `.motors` iterable of objects having `joint_name`, `position_rad`,
    `velocity_radps`, `effort_nm`, `temperature_c`; and must provide
    `set_motor_position(joint_names, positions, velocities, kp, kd)`.
    """

    def __init__(
        self,
        client,
        joint_names: List[str],
        hz: float = 60.0,
        motor_configs: Dict[str, MotorConfig] | None = None,
    ):
        self.client = client
        self.joint_names = list(joint_names)
        self.motor_configs = motor_configs or {}
        self.hz = float(hz)
        self.dt = 1.0 / self.hz

        self._states: Dict[str, MotorState] = {}
        for jn in self.joint_names:
            cfg = self.motor_configs.get(jn)
            self._states[jn] = MotorState(
                joint_name=jn,
                kp=cfg.kp if cfg is not None else None,
                kd=cfg.kd if cfg is not None else None,
            )
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._missing_gain_warned = False

    @classmethod
    def from_client(
        cls,
        client,
        joint_names: List[str],
        hz: float = 60.0,
        motor_configs: Dict[str, MotorConfig] | None = None,
    ):
        return cls(client, joint_names, hz, motor_configs=motor_configs)

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _loop(self) -> None:
        while not self._stop.is_set():
            t0 = time.time()
            try:
                self.control_step()
            except Exception:
                pass
            elapsed = time.time() - t0
            to_sleep = max(0.0, self.dt - elapsed)
            time.sleep(to_sleep)

    def control_step(self) -> None:
        try:
            reply = self.client.get_motor_status(self.joint_names)
        except Exception:
            return

        now = time.time()
        with self._lock:
            # update telemetry
            for ms in reply.motors:
                name = ms.joint_name
                if name not in self._states:
                    continue
                s = self._states[name]
                s.position = ms.position_rad
                s.velocity = ms.velocity_radps
                s.torque = ms.effort_nm
                s.temperature = ms.temperature_c
                s.temp_state = temp_state_from_temp(s.temperature)

            # compute targets and apply ramp + derate
            for s in self._states.values():
                # handle sine excitation if set as attribute
                ex: Optional[Excitation] = getattr(s, "_excitation", None)
                if ex is not None and ex.mode == "sine":
                    if ex.duration_s is not None and (now - ex.t0) >= ex.duration_s:
                        s._excitation = None
                        s.target_rad = ex.center_rad
                    else:
                        s.target_rad = ex.center_rad + ex.amp_rad * math.sin(2.0 * math.pi * ex.freq_hz * (now - ex.t0))

                motion_scale = 1.0
                if s.temp_state == "DERATE":
                    motion_scale = motion_scale_from_temp(s.temperature)

                max_step = (math.radians(30.0) * self.dt) * motion_scale
                delta = s.target_rad - s.commanded_target_rad
                if abs(delta) <= max_step:
                    s.commanded_target_rad = s.target_rad
                else:
                    s.commanded_target_rad += math.copysign(max_step, delta)

            # send command
            positions = [self._states[jn].commanded_target_rad for jn in self.joint_names]
            kps = []
            kds = []
            for jn in self.joint_names:
                kp = self._states[jn].kp
                kd = self._states[jn].kd
                if kp is None or kd is None:
                    if not self._missing_gain_warned:
                        warnings.warn(
                            f"Joint '{jn}' is missing kp/kd in config and set_kp/set_kd was not called; skipping command send",
                            RuntimeWarning,
                            stacklevel=2,
                        )
                        self._missing_gain_warned = True
                    return
                kps.append(kp)
                kds.append(kd)
            try:
                self.client.set_motor_position(self.joint_names, positions, None, kps, kds)
            except Exception:
                pass

    # --- command API ---
    def hold(self) -> None:
        reply = self.client.get_motor_status(self.joint_names)
        with self._lock:
            for ms in reply.motors:
                if ms.joint_name in self._states:
                    s = self._states[ms.joint_name]
                    s.target_rad = ms.position_rad
                    s.commanded_target_rad = ms.position_rad

    def step(self, delta_deg: float) -> None:
        delta = math.radians(float(delta_deg))
        with self._lock:
            for s in self._states.values():
                next_target = s.target_rad + delta
                cfg = self.motor_configs.get(s.joint_name)
                if cfg is not None:
                    next_target = max(cfg.min_position, min(cfg.max_position, next_target))
                s.target_rad = next_target

    def goto(self, angle_deg: float) -> None:
        rad = math.radians(float(angle_deg))
        with self._lock:
            for s in self._states.values():
                cfg = self.motor_configs.get(s.joint_name)
                if cfg is not None:
                    s.target_rad = max(cfg.min_position, min(cfg.max_position, rad))
                else:
                    s.target_rad = rad

    def sine(self, amp_deg: float, freq_hz: float, duration_s: Optional[float] = None) -> None:
        amp = math.radians(float(amp_deg))
        now = time.time()
        with self._lock:
            for s in self._states.values():
                s._excitation = Excitation(mode="sine", amp_rad=amp, freq_hz=float(freq_hz), t0=now, duration_s=duration_s, center_rad=s.target_rad)

    def stop_excitation(self) -> None:
        with self._lock:
            for s in self._states.values():
                s._excitation = None

    def set_kp(self, kp: float) -> None:
        with self._lock:
            for s in self._states.values():
                s.kp = float(kp)

    def set_kd(self, kd: float) -> None:
        with self._lock:
            for s in self._states.values():
                s.kd = float(kd)

    def status(self) -> List[Tuple[str, MotorState]]:
        with self._lock:
            return [(jn, self._states[jn]) for jn in self.joint_names]
