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
        self._active = None          # previous active set (-1 lower, +1 upper, 0 free)
        self._matrices = {}          # with_com -> (M, H)
        self.fallbacks = 0           # solves handed to lsq_linear (see _box_qp)

    def _system(self, with_com):
        """Least-squares matrix M (depends only on the weights) and H = M^T M."""
        if with_com not in self._matrices:
            n, w = self.n, self.w
            rows = [math.sqrt(w.cop) * np.eye(n),
                    math.sqrt(w.cop_rate) * (np.eye(n) - np.eye(n, k=-1))]
            if with_com:
                rows.append(math.sqrt(w.com) * self.Gamma[:, 0, :])
            e = np.array([1.0, 1.0 / self.omega])
            rows.append(math.sqrt(w.terminal_dcm) * (e @ self.Gamma[-1])[None, :])
            M = np.vstack(rows)
            self._matrices[with_com] = (M, M.T @ M)
        return self._matrices[with_com]

    def solve(self, x0, p_ref, lo, hi, c_ref=None, p_prev=None):
        """x0 = (c, c'); p_ref, lo, hi, c_ref: arrays of length n. Returns the CoP sequence."""
        n, w = self.n, self.w
        p_ref, lo, hi = (np.asarray(a, float) for a in (p_ref, lo, hi))
        lo = np.minimum(lo, hi - 1e-6)
        x0 = np.asarray(x0, float)
        with_com = c_ref is not None and w.com > 0
        M, H = self._system(with_com)
        rhs = [math.sqrt(w.cop) * p_ref]
        # CoP rate (first difference to the previously applied CoP).
        d0 = np.zeros(n)
        if p_prev is not None:
            d0[0] = p_prev
        rhs.append(math.sqrt(w.cop_rate) * d0)
        # CoM position tracking.
        if with_com:
            free = self.Phi[:, 0, :] @ x0
            rhs.append(math.sqrt(w.com) * (np.asarray(c_ref, float) - free))
        # Terminal DCM at the final CoP reference.
        e = np.array([1.0, 1.0 / self.omega])
        free_end = e @ self.Phi[-1] @ x0
        rhs.append(np.array([math.sqrt(w.terminal_dcm) * (p_ref[-1] - free_end)]))
        b = np.concatenate(rhs)
        x = self._box_qp(H, M.T @ b, lo, hi)
        if x is None:
            self.fallbacks += 1
            x = lsq_linear(M, b, bounds=(lo, hi), method='bvls').x
            self._active = np.where(x <= lo + 1e-9, -1, np.where(x >= hi - 1e-9, 1, 0))
        self.last_solution = x
        return x

    def _box_qp(self, H, g, lo, hi, max_iter=30, tol=1e-9):
        """min 1/2 x'Hx - g'x s.t. lo <= x <= hi (H positive definite).

        Primal-dual active set, warm-started with the previous cycle's active
        set (the horizon shifts by under one interval per cycle). It can cycle
        when H is not an M-matrix, so a repeated active set, like the iteration
        limit, returns None and the caller uses lsq_linear (same optimum).
        """
        n = len(g)
        act = (self._active.copy() if self._active is not None and len(self._active) == n
               else np.zeros(n, dtype=int))
        x = np.empty(n)
        seen = set()
        for _ in range(max_iter):
            key = act.tobytes()
            if key in seen:
                return None
            seen.add(key)
            free = act == 0
            x[act < 0], x[act > 0] = lo[act < 0], hi[act > 0]
            if free.any():
                rhs = g[free] - H[np.ix_(free, ~free)] @ x[~free]
                x[free] = np.linalg.solve(H[np.ix_(free, free)], rhs)
            mu = H @ x - g          # gradient; must be >= 0 at lower, <= 0 at upper bounds
            new = np.zeros(n, dtype=int)
            new[(x < lo) | ((act < 0) & (mu > 0))] = -1
            new[(x > hi) | ((act > 0) & (mu < 0))] = 1
            if np.array_equal(new, act):
                ok = (np.all(x >= lo - tol) and np.all(x <= hi + tol)
                      and np.all(np.abs(mu[free]) <= 1e-7 * (1.0 + np.abs(g[free])))
                      and np.all(mu[act < 0] >= -tol) and np.all(mu[act > 0] <= tol))
                if not ok:
                    return None
                self._active = act
                return np.clip(x, lo, hi)
            act = new
        return None

    def predict(self, x0, p):
        return np.array([self.Phi[k] @ x0 + self.Gamma[k] @ p for k in range(self.n)])
