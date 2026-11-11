# Vendored from triton-droids td_perception_camera, td_ball_tracker/td_ball_tracker/kf.py at cae8fb1
# ("P-33: ball tracker"). Keep in sync; local changes: none.
"""Constant-velocity Kalman filter with rolling friction for a ball on the ground (P-33).

State x = [px, py, vx, vy] in odom (world-fixed). Velocity decays as v(t) = v0·exp(-c·t) (c = friction
rate, 1/s), so position integrates (1 - e^{-c·dt})/c of the velocity. Process noise: white acceleration
with spectral density q (m²/s³) per axis, standard discretization.
"""

from __future__ import annotations

import math

import numpy as np

H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])


class BallKF:
    def __init__(self, z: np.ndarray, R: np.ndarray, t: float, init_vel_std: float):
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.P = np.zeros((4, 4))
        self.P[:2, :2] = R
        self.P[2, 2] = self.P[3, 3] = init_vel_std**2
        self.t = t

    @staticmethod
    def transition(dt: float, friction: float, q: float) -> tuple[np.ndarray, np.ndarray]:
        a = math.exp(-friction * dt)
        g = (1.0 - a) / friction if friction > 1e-9 else dt
        F = np.eye(4)
        F[0, 2] = F[1, 3] = g
        F[2, 2] = F[3, 3] = a
        q11, q12, q22 = q * dt**3 / 3, q * dt**2 / 2, q * dt
        Q = np.array([[q11, 0, q12, 0], [0, q11, 0, q12], [q12, 0, q22, 0], [0, q12, 0, q22]])
        return F, Q

    def predicted(self, t: float, friction: float, q: float) -> tuple[np.ndarray, np.ndarray]:
        """State and covariance at time t, without changing the filter."""
        dt = max(t - self.t, 0.0)
        F, Q = self.transition(dt, friction, q)
        return F @ self.x, F @ self.P @ F.T + Q

    def predict(self, t: float, friction: float, q: float) -> None:
        self.x, self.P = self.predicted(t, friction, q)
        self.t = max(t, self.t)

    def mahalanobis2(self, z: np.ndarray, R: np.ndarray) -> float:
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        return float(y @ np.linalg.solve(S, y))

    def update(self, z: np.ndarray, R: np.ndarray) -> None:
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        Kg = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + Kg @ y
        I_KH = np.eye(4) - Kg @ H
        self.P = I_KH @ self.P @ I_KH.T + Kg @ R @ Kg.T  # Joseph form: stays symmetric PSD
