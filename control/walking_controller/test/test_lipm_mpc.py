"""LIPM MPC: the active-set box QP must return the lsq_linear (BVLS) optimum."""
from pathlib import Path
import math
import sys

import numpy as np
from scipy.optimize import lsq_linear

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from walking_controller.lipm_mpc import LipmMPC  # noqa: E402

OMEGA = math.sqrt(9.81 / 0.37)


def reference(mpc, x0, p_ref, lo, hi, c_ref=None, p_prev=None):
    """The original formulation, solved with lsq_linear."""
    n, w = mpc.n, mpc.w
    lo = np.minimum(lo, hi - 1e-6)
    rows = [math.sqrt(w.cop) * np.eye(n), math.sqrt(w.cop_rate) * (np.eye(n) - np.eye(n, k=-1))]
    d0 = np.zeros(n)
    if p_prev is not None:
        d0[0] = p_prev
    rhs = [math.sqrt(w.cop) * p_ref, math.sqrt(w.cop_rate) * d0]
    if c_ref is not None:
        rows.append(math.sqrt(w.com) * mpc.Gamma[:, 0, :])
        rhs.append(math.sqrt(w.com) * (c_ref - mpc.Phi[:, 0, :] @ x0))
    e = np.array([1.0, 1.0 / mpc.omega])
    rows.append(math.sqrt(w.terminal_dcm) * (e @ mpc.Gamma[-1])[None, :])
    rhs.append(np.array([math.sqrt(w.terminal_dcm) * (p_ref[-1] - e @ mpc.Phi[-1] @ x0)]))
    return lsq_linear(np.vstack(rows), np.concatenate(rhs), bounds=(lo, hi),
                      method='bvls', tol=1e-12).x


def random_problem(rng, n, width):
    centre = rng.uniform(-0.15, 0.15, n)
    lo, hi = centre - width / 2, centre + width / 2
    x0 = np.array([rng.uniform(-0.1, 0.1), rng.uniform(-0.6, 0.6)])
    p_ref = centre + rng.uniform(-0.05, 0.05, n)
    return x0, p_ref, lo, hi


def test_matches_lsq_linear_on_random_problems():
    rng = np.random.default_rng(0)
    mpc = LipmMPC(OMEGA, 0.04, 25)
    active_counts = set()
    for trial in range(400):
        width = rng.choice([1e-3, 0.02, 0.06, 0.3])  # from nearly fixed to all free
        x0, p_ref, lo, hi = random_problem(rng, mpc.n, width)
        c_ref = p_ref if trial % 2 else None
        p_prev = float(rng.uniform(-0.1, 0.1)) if trial % 3 else None
        x = mpc.solve(x0, p_ref, lo, hi, c_ref=c_ref, p_prev=p_prev)
        xr = reference(mpc, x0, p_ref, lo, hi, c_ref, p_prev)
        np.testing.assert_allclose(x, xr, atol=1e-8)
        active_counts.add(int(((x <= lo + 1e-9) | (x >= hi - 1e-9)).sum()))
    # Bounds that jump randomly every interval can make the active set cycle;
    # those few solves go to lsq_linear (checked above to give the same x).
    assert mpc.fallbacks <= 8
    # The trials cover no, some and all bounds active.
    assert 0 in active_counts and mpc.n in active_counts and len(active_counts) > 5


def stepping_supports(n, phase, foot_lat=0.155, half=0.04, ds=5, ss=8):
    """Piecewise-constant lateral supports of a DS/SS pattern, as stepping.py plans."""
    lo, hi, ref = np.zeros(n), np.zeros(n), np.zeros(n)
    side = 1.0
    for k in range(n):
        t = (k + phase) % (ds + ss)
        if t < ds:
            lo[k], hi[k], ref[k] = -foot_lat - half, foot_lat + half, 0.5 * side * foot_lat
        else:
            lo[k], hi[k], ref[k] = side * foot_lat - half, side * foot_lat + half, side * foot_lat
        if t == ds + ss - 1:
            side = -side
    return lo, hi, ref


def test_no_fallback_on_stepping_shaped_problems():
    """Realistic supports (as in walking): the active set converges every cycle."""
    rng = np.random.default_rng(3)
    mpc = LipmMPC(OMEGA, 0.04, 25)
    for cycle in range(300):
        lo, hi, ref = stepping_supports(mpc.n, cycle % 13)
        x0 = np.array([rng.uniform(-0.12, 0.12), rng.uniform(-0.5, 0.5)])
        p_prev = float(rng.uniform(-0.1, 0.1))
        x = mpc.solve(x0, ref, lo, hi, c_ref=np.zeros(mpc.n), p_prev=p_prev)
        np.testing.assert_allclose(x, reference(mpc, x0, ref, lo, hi, np.zeros(mpc.n), p_prev), atol=1e-8)
    assert mpc.fallbacks == 0


def test_warm_start_across_a_shifting_horizon():
    """Consecutive cycles reuse the active set; the result must not depend on it."""
    rng = np.random.default_rng(1)
    warm, cold = LipmMPC(OMEGA, 0.04, 25), LipmMPC(OMEGA, 0.04, 25)
    x0, p_ref, lo, hi = random_problem(rng, warm.n, 0.04)
    for _ in range(50):
        x0 = x0 + rng.normal(0, 0.01, 2)
        lo, hi, p_ref = (np.roll(a, -1) for a in (lo, hi, p_ref))
        x = warm.solve(x0, p_ref, lo, hi, c_ref=p_ref)
        cold._active = None
        np.testing.assert_allclose(x, cold.solve(x0, p_ref, lo, hi, c_ref=p_ref), atol=1e-8)
        np.testing.assert_allclose(x, reference(warm, x0, p_ref, lo, hi, p_ref), atol=1e-8)


def test_falls_back_to_lsq_linear_when_active_set_does_not_converge():
    rng = np.random.default_rng(2)
    mpc = LipmMPC(OMEGA, 0.04, 25)
    x0, p_ref, lo, hi = random_problem(rng, mpc.n, 0.03)
    mpc._box_qp = lambda *args, **kwargs: None
    x = mpc.solve(x0, p_ref, lo, hi, c_ref=p_ref)
    assert mpc.fallbacks == 1
    np.testing.assert_allclose(x, reference(mpc, x0, p_ref, lo, hi, p_ref), atol=1e-8)


def test_inverted_bounds_are_repaired_like_before():
    """lo > hi (degenerate support) is clamped to lo = hi - 1e-6, as before."""
    mpc = LipmMPC(OMEGA, 0.04, 25)
    hi = np.full(mpc.n, 0.05)
    lo = hi + 0.01
    x = mpc.solve(np.zeros(2), np.zeros(mpc.n), lo, hi)
    assert np.all(x <= hi + 1e-12) and np.all(x >= hi - 1e-6 - 1e-12)
