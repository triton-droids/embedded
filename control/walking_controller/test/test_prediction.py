"""Latency compensation: constant-velocity state prediction in GaitController.step."""
import numpy as np
import pinocchio as pin
import pytest

from conftest import GAINS


@pytest.fixture
def controller(model):
    from walking_controller.gait import GaitController, GaitParams
    from walking_controller.wbc import WBCGains
    _, robot, limits = model
    return GaitController(robot, limits.torque, GaitParams(gains=WBCGains(**GAINS)))


def test_predict_integrates_base_and_joints_at_constant_velocity(controller, standing):
    q, v = standing['q'], standing['v'].copy()
    v[:] = 0.0
    v[0], v[5], v[6 + 3] = 0.2, 0.5, -1.0      # base forward, base yaw rate, one joint
    qp, vp = controller.predict(q, v, 0.02)
    np.testing.assert_allclose(vp, v)
    np.testing.assert_allclose(qp, pin.integrate(controller.rm.model, q, 0.02 * v))
    assert qp[7 + 3] == pytest.approx(q[7 + 3] - 0.02)
    assert np.linalg.norm(qp[3:7]) == pytest.approx(1.0)


def test_horizon_is_input_age_plus_predict_s_and_off_on_the_gantry(controller, standing):
    from walking_controller.gait import HOLD
    q, v = standing['q'], standing['v']
    calls = []
    controller.predict = lambda q, v, h: (calls.append(h), (q, v))[1]
    controller.step(0.0, q, v, np.zeros(2), base_held=True, age_s=0.004)
    assert calls == [] and controller.phase == HOLD          # held: no prediction
    controller.step(0.005, q, v, np.full(2, 80.0), base_held=False, age_s=0.004)
    assert calls == []                                       # still HOLD at the release cycle
    controller.step(0.010, q, v, np.full(2, 80.0), base_held=False, age_s=0.004)
    assert calls == [pytest.approx(0.004 + controller.p.predict_s)]
    assert controller.horizon_s == pytest.approx(0.014)
