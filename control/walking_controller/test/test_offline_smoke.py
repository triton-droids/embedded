"""Offline closed-loop smoke tests (MuJoCo + controller, no ROS).

Fast tests run by default (~20 s on the Jetson). The latency regressions are
marked slow (WALK_SLOW_TESTS=1): on 2026-11-08 the ROS chain fell after ~30 s
of walking at ~25 ms loop latency; without compensation offline 20 ms falls at
~35 s. With the default 10 ms state prediction (GaitParams.predict_s) 20 ms
walks and 30 ms falls (docs/model_based_walking_design.md, sections 12 and 14).
"""
import numpy as np
import pytest

from conftest import GAINS


def stepping(model, length, duration, delay_steps=2, switch=None):
    """Run SteppingController; switch=(t, length) changes the step length once."""
    from walking_controller.offline import run
    from walking_controller.stepping import StepParams, SteppingController
    from walking_controller.wbc import WBCGains
    paths, robot, limits = model
    ctl = SteppingController(robot, limits.torque, StepParams(step_length_m=length, gains=WBCGains(**GAINS)))
    start = {}

    def on_step(t, sim, c):
        if 'pos' not in start and getattr(c, 'fwd_axis', None) is not None:
            start['pos'] = sim.data.qpos[:3].copy()
        if switch is not None and t >= switch[0]:
            c.p.step_length_m = switch[1]
        return False

    res = run(ctl, duration, delay_steps=delay_steps, paths=paths, robot=robot, limits=limits,
              on_step=on_step)
    moved = res['sim'].data.qpos[:3] - start['pos']
    return ctl, res, float(moved[:2] @ ctl.fwd_axis[:2]), float(moved[:2] @ ctl.lat_axis[:2])


def test_stand_holds_the_squat(model, standing):
    from walking_controller.offline import run
    ctl = standing['controller']
    res = standing['result']
    assert not res['fault'] and not res['fallen']
    robot = model[1]
    robot.forward(standing['q'], standing['v'])
    assert abs(robot.com[2] - ctl.z0) < 0.005
    assert ctl._tilt() < 0.05
    # The knees reached the planned squat.
    knees = [i for i, n in enumerate(robot.joint_names) if 'knee' in n]
    np.testing.assert_allclose(standing['q'][7:][knees], ctl.p.knee_rad, atol=0.1)


def test_steps_in_place_without_drifting(model):
    ctl, res, fwd, lat = stepping(model, 0.0, 10.0)
    assert not res['fault'] and not res['fallen']
    assert ctl.steps >= 12
    assert abs(fwd) < 0.05 and abs(lat) < 0.05


def test_walks_forward(model):
    ctl, res, fwd, lat = stepping(model, 0.06, 12.0)
    assert not res['fault'] and not res['fallen']
    assert ctl.steps >= 18
    assert fwd > 0.5 and abs(lat) < 0.1


def test_realised_step_length_follows_the_command(model):
    """Touchdown is accepted from s > 0.6 and in practice comes right there
    (the swing foot clips the floor). With the horizontal swing finishing at
    s = 1 and no latency compensation the steps were 0.73 of 0.05 m; with the
    10 ms state prediction they would be ~0.6, swing_fwd_end = 0.85 gives ~0.78."""
    from walking_controller.offline import run
    from walking_controller.stepping import SS, StepParams, SteppingController
    from walking_controller.wbc import WBCGains
    paths, robot, limits = model
    length = 0.05
    ctl = SteppingController(robot, limits.torque, StepParams(step_length_m=length, gains=WBCGains(**GAINS)))
    marks, touchdown_s, phase = {}, [], {}

    def on_step(t, sim, c):
        steps = getattr(c, 'steps', 0)
        if steps == 4 and 'a' not in marks:
            marks['a'] = (sim.data.qpos[:2].copy(), steps)
        marks['b'] = (sim.data.qpos[:2].copy(), steps)
        if getattr(c, 'gphase', None) == SS:
            if phase.get('t0') != c.t_gphase and 's' in phase and steps > 4:
                touchdown_s.append(phase['s'])
            phase.update(t0=c.t_gphase, s=(t - c.t_gphase) / c.t_dur)
        return False

    res = run(ctl, 14.0, paths=paths, robot=robot, limits=limits, on_step=on_step)
    assert not res['fault'] and not res['fallen']
    (pa, sa), (pb, sb) = marks['a'], marks['b']
    per_step = float((pb - pa) @ ctl.fwd_axis[:2]) / (sb - sa)
    assert per_step > 0.7 * length, per_step
    assert per_step < 1.15 * length, per_step
    # The cause is unchanged: touchdown still right after s = 0.6.
    assert np.median(touchdown_s) < 0.7


@pytest.mark.slow
@pytest.mark.parametrize('delay_steps', [3, 4])
def test_walks_49_s_at_15_and_20_ms_delay(model, delay_steps):
    """In place for 9 s, then 0.06 m. The ROS chain is at ~15 ms; 20 ms (10 ms
    more than the default prediction) fell at ~30 s before the compensation."""
    ctl, res, fwd, _ = stepping(model, 0.0, 49.0, delay_steps=delay_steps, switch=(9.0, 0.06))
    assert not res['fault'] and not res['fallen'], res['fault']
    assert fwd > 2.0


@pytest.mark.slow
@pytest.mark.xfail(strict=True, reason='known limit: 30 ms with the 10 ms default prediction falls (section 14)')
def test_walks_49_s_at_30_ms_delay(model):
    ctl, res, _, _ = stepping(model, 0.0, 49.0, delay_steps=6, switch=(9.0, 0.06))
    assert not res['fault'] and not res['fallen'], res['fault']
