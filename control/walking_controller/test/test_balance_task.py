"""Balance reward and braking metrics without MuJoCo."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from tools.rl_tasks.balance import Task


def environment(task):
    env = SimpleNamespace(state=[{}, {}])
    for i in range(2):
        task.on_reset(env, i)
    return env


def measurement(**changes):
    meas = dict(other=False, z=0.62, up=1.0, speed=0.0, forces=[60.0, 60.0],
                err_max=0.0, leg_err=0.0, tilts=[0.0, 0.0], dq=np.zeros(2),
                ang_body=np.zeros(3), airborne=False, vz=0.0, power=0.0, speed_ratio=0.0)
    meas.update(changes)
    return meas


def step(task, env, i=0, **changes):
    return task.reward(env, i, measurement(**changes), np.zeros(2))


@pytest.mark.parametrize('changes,fell', [({}, False), ({'other': True}, True),
    ({'z': 0.379}, True), ({'z': 0.38}, False), ({'up': 0.599}, True), ({'up': 0.6}, False)])
def test_default_fall_check(changes, fell):
    task = Task()
    reward, done, success, info = step(task, environment(task), **changes)
    assert done == info['fell'] == fell
    assert not success
    assert math.isnan(info['brake_s'])
    if fell:
        assert reward == -5.0


def test_relaxed_fall_thresholds():
    task = Task(fall_up=0.45, fall_z=0.33)
    env = environment(task)
    assert not step(task, env, up=0.5, z=0.34)[1]
    assert step(task, env, up=0.44)[1]
    assert step(task, env, z=0.32)[1]
    assert step(task, env, other=True)[1]


def test_default_reward_and_success_unchanged():
    task = Task(hold_steps=2)
    env = environment(task)
    # Perfect stance: score=1, original step reward=1.5 + ready fraction / 2.
    r, done, success, _ = step(task, env)
    assert r == pytest.approx(1.75)
    assert not done and not success
    r, done, success, _ = step(task, env)
    assert r == pytest.approx(2.0 - 0.02 + task.bonus)
    assert done and success


def test_brake_reward_difference_and_first_step():
    task, baseline = Task(brake_w=2.0, fall_up=0.45), Task(fall_up=0.45)
    env, base_env = environment(task), environment(baseline)
    for speed, tilt, delta in [(2.0, 0.6, 0.0), (1.8, 0.6, 0.0),
                               (0.8, 0.4, 1.8), (1.0, 0.5, -0.6)]:
        r = step(task, env, speed=speed, up=math.cos(tilt))[0]
        b = step(baseline, base_env, speed=speed, up=math.cos(tilt))[0]
        assert r - b == pytest.approx(delta)
    task.on_reset(env, 0)
    baseline.on_reset(base_env, 0)
    assert step(task, env)[0] == step(baseline, base_env)[0]


def test_braking_time_hold_latch_reset_and_environment_isolation():
    task = Task()
    env = environment(task)
    step(task, env, speed=0.8, up=math.cos(0.5))
    for _ in range(4):
        assert math.isnan(step(task, env)[3]['brake_s'])
    step(task, env, speed=0.3)  # Strict speed threshold resets the hold.
    step(task, env, up=math.cos(0.4))  # Tilt must also satisfy the threshold.
    for _ in range(4):
        assert math.isnan(step(task, env)[3]['brake_s'])
    info = step(task, env)[3]
    assert info['brake_s'] == pytest.approx(12 * 0.02)
    assert info['start_speed'] == 0.8
    assert info['start_tilt'] == pytest.approx(0.5)
    assert step(task, env, other=True, speed=0.9)[3]['brake_s'] == info['brake_s']
    assert math.isnan(step(task, env, i=1)[3]['brake_s'])
    task.on_reset(env, 0)
    info = step(task, env, speed=0.7)[3]
    assert math.isnan(info['brake_s'])
    assert info['start_speed'] == 0.7 and info['start_tilt'] == 0.0


@pytest.mark.parametrize('up,tilt', [(1.000001, 0.0), (-1.000001, math.pi)])
def test_start_tilt_clipped_on_first_step_including_fall(up, tilt):
    task = Task()
    info = step(task, environment(task), up=up)[3]
    assert info['start_tilt'] == tilt
