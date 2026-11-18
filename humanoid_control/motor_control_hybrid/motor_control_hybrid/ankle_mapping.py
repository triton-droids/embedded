"""Four-bar ankle conversion from robstride_data 2728fe19."""
import math
from typing import Any
import numpy as np

def require_key(cfg, key, context):
    return cfg[key]

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


