import math

import numpy as np

from humanoid_sim.imu_nodes import _YawError


class _Param:
    def __init__(self, value):
        self.value = value


class _Node:
    def __init__(self, **params):
        self.params = params

    def declare_parameter(self, name, default):
        self.params.setdefault(name, default)

    def get_parameter(self, name):
        return _Param(self.params[name])


def _yaw(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _tilt(q):
    w, x, y, z = q
    return np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])  # body z in world


def test_disabled_by_default():
    q = np.array([0.9, 0.1, -0.2, 0.3]); q /= np.linalg.norm(q)
    assert np.array_equal(_YawError(_Node()).apply(q, 123), q)


def test_sine_heading_error_keeps_tilt():
    err = _YawError(_Node(yaw_error_amplitude_deg=5.0, yaw_error_period_s=40.0))
    q = np.array([0.9, 0.1, -0.2, 0.3]); q /= np.linalg.norm(q)
    start = 10**9
    assert np.allclose(err.apply(q, start), q)
    out = err.apply(q, start + 10 * 10**9)  # Quarter period: +amplitude.
    d = math.atan2(math.sin(_yaw(out) - _yaw(q)), math.cos(_yaw(out) - _yaw(q)))
    assert math.isclose(d, math.radians(5.0), abs_tol=1e-9)
    tilt_in, tilt_out = _tilt(q), _tilt(out)
    assert math.isclose(tilt_in[2], tilt_out[2], abs_tol=1e-12)  # Gravity direction unchanged.
    assert math.isclose(np.linalg.norm(out), 1.0, abs_tol=1e-12)
