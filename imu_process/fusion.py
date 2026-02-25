from collections import deque
from dataclasses import dataclass
from typing import Dict, Tuple

from math3d import norm3, unit3, dot, cross, clamp, mean, std
from quat import (
    q_mul, q_normalize, q_rotate_world_to_body, q_from_axis_angle, q_slerp,
    q_to_euler_deg, q_integrate_midpoint
)
import math

@dataclass
class StillnessCfg:
    sample_hz: float = 50.0
    win_sec: float = 0.6
    g_lo: float = 0.98
    g_hi: float = 1.02
    a_std: float = 0.010        # std(|a|) in g
    gyro_std: float = 0.08      # std(ω) in rad/s (~4.6 deg/s)

@dataclass
class FusionCfg:
    corr_alpha: float = 0.10    # slerp strength per update when still
    bias_alpha: float = 0.05    # bias EMA when still

class QuatFusion:
    """
    q: WORLD -> BODY
    gravity in world: (0,0,1)
    """
    def __init__(self, still: StillnessCfg, cfg: FusionCfg):
        self.still = still
        self.cfg = cfg

        self.N = max(10, int(still.win_sec * still.sample_hz))
        self.aN_buf = deque(maxlen=self.N)
        self.wx_buf = deque(maxlen=self.N)
        self.wy_buf = deque(maxlen=self.N)
        self.wz_buf = deque(maxlen=self.N)

        self.q = (0.0, 0.0, 0.0, 1.0)
        self.omega_prev = (0.0, 0.0, 0.0)

        self.bias = (0.0, 0.0, 0.0)  # rad/s
        self.have_bias = False

    def _is_still(self) -> bool:
        if len(self.aN_buf) < self.aN_buf.maxlen:
            return False
        a_m = mean(self.aN_buf)
        a_s = std(self.aN_buf)
        if not (self.still.g_lo <= a_m <= self.still.g_hi):
            return False
        if a_s > self.still.a_std:
            return False
        if std(self.wx_buf) > self.still.gyro_std:
            return False
        if std(self.wy_buf) > self.still.gyro_std:
            return False
        if std(self.wz_buf) > self.still.gyro_std:
            return False
        return True

    def step(self, dt: float,
             ax_g: float, ay_g: float, az_g: float,
             wx: float, wy: float, wz: float) -> Dict:

        # update buffers for stillness detection
        aN = norm3(ax_g, ay_g, az_g)
        self.aN_buf.append(aN)
        self.wx_buf.append(wx)
        self.wy_buf.append(wy)
        self.wz_buf.append(wz)

        still = self._is_still()

        # estimate/update bias when still (EMA on window mean)
        if still:
            mx = mean(self.wx_buf); my = mean(self.wy_buf); mz = mean(self.wz_buf)
            if not self.have_bias:
                self.bias = (mx, my, mz)
                self.have_bias = True
            else:
                a = self.cfg.bias_alpha
                bx, by, bz = self.bias
                self.bias = ((1-a)*bx + a*mx,
                             (1-a)*by + a*my,
                             (1-a)*bz + a*mz)

        bx, by, bz = self.bias
        omega_now = (wx - bx, wy - by, wz - bz)

        # propagate quaternion
        self.q = q_integrate_midpoint(self.q, self.omega_prev, omega_now, dt)
        self.omega_prev = omega_now

        # correction when still: align gravity prediction to accel direction
        if still:
            g_meas = unit3(ax_g, ay_g, az_g)  # static: accel points along +g
            if g_meas is not None:
                g_pred = q_rotate_world_to_body(self.q, (0.0, 0.0, 1.0))
                g_pred_u = unit3(*g_pred) or (0.0, 0.0, 1.0)

                v = cross(g_pred_u, g_meas)
                v_n = norm3(*v)
                c = clamp(dot(g_pred_u, g_meas), -1.0, 1.0)

                if v_n > 1e-8:
                    axis = (v[0]/v_n, v[1]/v_n, v[2]/v_n)
                    angle = math.atan2(v_n, c)

                    q_corr_full = q_from_axis_angle(axis, angle)
                    alpha = clamp(self.cfg.corr_alpha, 0.0, 1.0)
                    q_corr = q_slerp((0.0, 0.0, 0.0, 1.0), q_corr_full, alpha)

                    # body-frame correction
                    self.q = q_normalize(q_mul(q_corr, self.q))

        # derived outputs
        g_hat = q_rotate_world_to_body(self.q, (0.0, 0.0, 1.0))
        g_hat_u = unit3(*g_hat) or g_hat
        anti_g = (-g_hat_u[0], -g_hat_u[1], -g_hat_u[2])
        roll, pitch, yaw = q_to_euler_deg(self.q)

        return {
            "still": still,
            "a_norm_g": aN,
            "q": self.q,
            "bias": self.bias,
            "omega_corr": omega_now,
            "g_hat": g_hat_u,
            "anti_g": anti_g,
            "euler_deg": (roll, pitch, yaw),
        }
