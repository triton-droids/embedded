"""LIPM MPC: exact discretisation, CoP bounds, terminal DCM, closed-loop capture."""
import math

import numpy as np
import pytest

from walking_controller.lipm_mpc import LipmMPC, MPCWeights

OMEGA = math.sqrt(9.81 / 0.37)
DT, N = 0.04, 25


def dcm(x, omega=OMEGA):
    return x[0] + x[1] / omega


def integrate(x0, p, dt, omega=OMEGA, substeps=400):
    """Fine RK4 of c'' = omega^2 (c - p) with p held over each interval."""
    out, x = [], np.array(x0, float)
    f = lambda s, pk: np.array([s[1], omega**2 * (s[0] - pk)])
    h = dt / substeps
    for pk in p:
        for _ in range(substeps):
            k1 = f(x, pk); k2 = f(x + h / 2 * k1, pk); k3 = f(x + h / 2 * k2, pk); k4 = f(x + h * k3, pk)
            x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        out.append(x.copy())
    return np.array(out)


def test_prediction_is_the_exact_lipm_solution():
    mpc = LipmMPC(OMEGA, DT, N)
    rng = np.random.default_rng(0)
    x0, p = np.array([0.03, -0.2]), rng.uniform(-0.1, 0.1, N)
    np.testing.assert_allclose(mpc.predict(x0, p), integrate(x0, p, DT), atol=1e-9)


def test_cop_stays_inside_the_support_bounds():
    mpc = LipmMPC(OMEGA, DT, N)
    rng = np.random.default_rng(1)
    for _ in range(100):
        lo = rng.uniform(-0.2, 0.1, N)
        hi = lo + rng.uniform(0.0, 0.1, N)
        x = mpc.solve([rng.uniform(-0.2, 0.2), rng.uniform(-1, 1)], (lo + hi) / 2 + 0.3, lo, hi)
        assert np.all(x >= np.minimum(lo, hi - 1e-6) - 1e-12) and np.all(x <= hi + 1e-12)


def test_terminal_dcm_reaches_the_final_reference_when_feasible():
    """Wide bounds: the end state is capturable at the last CoP reference."""
    mpc = LipmMPC(OMEGA, DT, N)
    x0, target = np.array([0.0, 0.25]), 0.04
    p_ref = np.full(N, target)
    lo, hi = np.full(N, -0.3), np.full(N, 0.3)
    p = mpc.solve(x0, p_ref, lo, hi)
    end = mpc.predict(x0, p)[-1]
    assert abs(dcm(end) - target) < 5e-3


def test_terminal_weight_pulls_the_dcm_towards_the_reference():
    """CoP tracking (towards 0) against the terminal DCM (towards the last
    reference, 0.04 m), feasible bounds: more weight, smaller DCM error."""
    x0, target = np.array([0.0, 0.1]), 0.04
    p_ref = np.zeros(N)
    p_ref[-1] = target
    lo, hi = np.full(N, -0.06), np.full(N, 0.06)
    errors = []
    for weight in (0.0, 3.0, 30.0, 300.0):
        mpc = LipmMPC(OMEGA, DT, N, MPCWeights(terminal_dcm=weight))
        p = mpc.solve(x0, p_ref, lo, hi)
        errors.append(abs(dcm(mpc.predict(x0, p)[-1]) - target))
    assert all(a >= b - 1e-12 for a, b in zip(errors, errors[1:]))
    assert errors[-1] < 0.25 * errors[0]


def test_dcm_outside_the_support_cannot_be_captured():
    """Physics check: with the DCM beyond every bound the MPC saturates the CoP."""
    mpc = LipmMPC(OMEGA, DT, N)
    x0 = np.array([0.0, 1.0])            # DCM = 0.19 m
    lo, hi = np.full(N, -0.05), np.full(N, 0.05)
    p = mpc.solve(x0, np.zeros(N), lo, hi)
    assert p[0] == pytest.approx(0.05)   # pushes as hard as the support allows
    assert dcm(mpc.predict(x0, p)[-1]) > dcm(x0)   # and still diverges


def test_closed_loop_lipm_stays_captured():
    """Receding horizon on the LIPM itself: a perturbed CoM settles over the support."""
    mpc = LipmMPC(OMEGA, DT, N)
    lo, hi = np.full(N, -0.06), np.full(N, 0.06)
    x, p_prev = np.array([0.02, 0.15]), None
    for _ in range(100):                 # 4 s, re-solved every interval
        p = mpc.solve(x, np.zeros(N), lo, hi, c_ref=np.zeros(N), p_prev=p_prev)
        p_prev = p[0]
        x = mpc.A @ x + mpc.B * p[0]
    assert abs(x[0]) < 0.01 and abs(x[1]) < 0.02
