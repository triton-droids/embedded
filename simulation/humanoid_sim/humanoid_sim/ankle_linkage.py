"""Ankle four-bar transmission for the simulated ankle motors (motor_control_hybrid ankle_mapping).

On the robot the ankle motor drives the foot through a four-bar. motor_control_hybrid converts only the commanded
position from ankle to motor angle (``AnkleMapper.ankle_rad_to_motor_logical_rad``); kp, kd, velocity and feedforward
torque go to the motor unchanged, and the motor's torque limit applies on the motor side. The simulator does the
same: the MIT law runs in motor coordinates and the foot receives

    tau_ankle = J * tau_motor,   J = d(motor angle) / d(ankle angle)

(virtual work). J is about 0.95 near the zero pose and falls to 0 at the motor's dead points, where the motor
can no longer drive the foot.

Same Freudenstein equation and parameters as ``motor_control_hybrid/ankle_mapping.py`` (theta2: heel lever, from
the ankle angle; theta4: crank, from the motor angle), solved in closed form instead of with fsolve, so the
physics loop can run it every step:

    k1 cos(t4) - k2 cos(t2) - cos(t2 - t4) + k3 = 0
"""
from __future__ import annotations

import math


class AnkleLinkage:
    def __init__(self, cfg: dict) -> None:
        lengths = cfg['link_lengths']
        if len(lengths) != 4:
            raise ValueError('ankle_mapping.link_lengths must be [L1, L2, L3, L4]')
        l1, l2, l3, l4 = (float(v) for v in lengths)
        self.k1 = l1 / l4
        self.k2 = l1 / l2
        self.k3 = (l2 ** 2 - l3 ** 2 + l4 ** 2 + l1 ** 2) / (2.0 * l2 * l4)
        self.theta2_offset = math.radians(float(cfg['theta2_offset_deg']))
        self.t4_to_motor_offset = math.radians(float(cfg['t4_to_motor_offset_deg']))
        self.sign = float(cfg['ankle_to_theta2_sign'])
        self._motor = 0.0  # last solution: picks the assembly branch, as AnkleCalibration's solver guess

    def motor(self, ankle: float, guess: float | None = None) -> tuple[float, float]:
        """Motor angle for an ankle angle and the ratio J = d motor / d ankle.

        The solution nearest ``guess`` (default: the last one). Returns (nan, 0) where the loop cannot close."""
        if guess is None:
            guess = self._motor
        t2 = self.theta2_offset + self.sign * ankle
        # (k1 - cos t2) cos t4 - sin t2 sin t4 = k2 cos t2 - k3
        a = self.k1 - math.cos(t2)
        b = -math.sin(t2)
        c = self.k2 * math.cos(t2) - self.k3
        r = math.hypot(a, b)
        if r < 1e-12 or abs(c) > r:
            return math.nan, 0.0
        base = math.atan2(b, a)
        spread = math.acos(c / r)
        t4_guess = guess - self.t4_to_motor_offset
        t4 = min((base + spread, base - spread),
                 key=lambda t: abs(math.remainder(t - t4_guess, 2.0 * math.pi)))
        t4 = t4_guess + math.remainder(t4 - t4_guess, 2.0 * math.pi)
        # implicit derivative of F(t2, t4) = 0
        f_t2 = self.k2 * math.sin(t2) + math.sin(t2 - t4)
        f_t4 = -self.k1 * math.sin(t4) - math.sin(t2 - t4)
        ratio = -f_t2 / f_t4 * self.sign if abs(f_t4) > 1e-12 else 0.0
        motor = t4 + self.t4_to_motor_offset
        self._motor = motor
        return motor, ratio

    def reset(self, ankle: float = 0.0) -> None:
        """Re-seed the branch from the zero pose, then follow to ``ankle`` in small steps."""
        self._motor = 0.0
        steps = max(1, int(abs(ankle) / 0.02))
        for i in range(1, steps + 1):
            self.motor(ankle * i / steps)
