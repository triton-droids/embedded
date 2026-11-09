"""Linear inverted pendulum MPC on one horizontal axis (CoP as the decision variable).

With the CoP p held constant over each interval of length dt, the LIPM
c'' = omega^2 (c - p) has the exact discrete form x_{k+1} = A x_k + B p_k with
x = (c, c'). The predicted CoM states are linear in the CoP sequence, so with
box bounds on every p_k (the support region of that interval) the problem is a
bounded linear least-squares problem, solved with scipy's lsq_linear.

Cost: CoP reference tracking, CoP smoothness, CoM reference tracking (weak),
and the terminal DCM (c + c'/omega) at the final CoP reference, which keeps
the end state capturable.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from scipy.optimize import lsq_linear


@dataclass
class MPCWeights:
    cop: float = 1.0
    cop_rate: float = 3.0
    com: float = 0.3
    terminal_dcm: float = 30.0


class LipmMPC:
    def __init__(self, omega: float, dt: float = 0.04, horizon: int = 25,
                 weights: MPCWeights | None = None):
        self.omega, self.dt, self.n = float(omega), float(dt), int(horizon)
        self.w = weights or MPCWeights()
        ch, sh = math.cosh(omega * dt), math.sinh(omega * dt)
        self.A = np.array([[ch, sh / omega], [omega * sh, ch]])
        self.B = np.array([1.0 - ch, -omega * sh])
        n = self.n
        # Prediction matrices: X_k = Phi_k x0 + Gamma_k p, for k = 1..n.
        self.Phi = np.zeros((n, 2, 2))
        self.Gamma = np.zeros((n, 2, n))
        Ak = np.eye(2)
        for k in range(n):
            Ak = self.A @ Ak
            self.Phi[k] = Ak
        for k in range(n):
            for j in range(k + 1):
                self.Gamma[k, :, j] = np.linalg.matrix_power(self.A, k - j) @ self.B
        self.last_solution = None

    def solve(self, x0, p_ref, lo, hi, c_ref=None, p_prev=None):
        """x0 = (c, c'); p_ref, lo, hi, c_ref: arrays of length n. Returns the CoP sequence."""
        n, w = self.n, self.w
        p_ref, lo, hi = (np.asarray(a, float) for a in (p_ref, lo, hi))
        lo = np.minimum(lo, hi - 1e-6)
        x0 = np.asarray(x0, float)
        rows, rhs = [], []
        # CoP tracking.
        rows.append(math.sqrt(w.cop) * np.eye(n))
        rhs.append(math.sqrt(w.cop) * p_ref)
        # CoP rate (first difference to the previously applied CoP).
        D = np.eye(n) - np.eye(n, k=-1)
        d0 = np.zeros(n)
        if p_prev is not None:
            d0[0] = p_prev
        rows.append(math.sqrt(w.cop_rate) * D)
        rhs.append(math.sqrt(w.cop_rate) * d0)
        # CoM position tracking.
        if c_ref is not None and w.com > 0:
            G = self.Gamma[:, 0, :]
            free = self.Phi[:, 0, :] @ x0
            rows.append(math.sqrt(w.com) * G)
            rhs.append(math.sqrt(w.com) * (np.asarray(c_ref, float) - free))
        # Terminal DCM.
        e = np.array([1.0, 1.0 / self.omega])
        g_end = e @ self.Gamma[-1]
        free_end = e @ self.Phi[-1] @ x0
        rows.append(math.sqrt(w.terminal_dcm) * g_end[None, :])
        rhs.append(np.array([math.sqrt(w.terminal_dcm) * (p_ref[-1] - free_end)]))
        M, b = np.vstack(rows), np.concatenate(rhs)
        result = lsq_linear(M, b, bounds=(lo, hi), method='bvls')
        self.last_solution = result.x
        return result.x

    def predict(self, x0, p):
        return np.array([self.Phi[k] @ x0 + self.Gamma[k] @ p for k in range(self.n)])
