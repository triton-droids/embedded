"""Get-up reward and start selection checks without loading MuJoCo."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from tools.rl_tasks.getup import Task


def environment(task, label='prone'):
    env = SimpleNamespace(state=[{}], label=[label], prev=np.zeros((1, 2)))
    task.on_reset(env, 0)
    return env


def measurement(**changes):
    meas = dict(feet_only=True, other=False, z=0.62, up=1.0, speed=0.0,
                airborne=False, power=0.0, vz=0.0, speed_ratio=0.0,
                dq=np.zeros(2), ang_body=np.zeros(3), tilts=[0.0, 0.0])
    meas.update(changes)
    return meas


def step(task, env, **changes):
    return task.reward(env, 0, measurement(**changes), np.zeros(2))


def test_optional_calm_lying_rise_bonus_once_then_success():
    task = Task(max_speed=0.3, max_tilt=0.3)
    env = environment(task)
    assert task.states is None and task.states_frac == 0.0
    _, done, success, info = step(task, env, feet_only=False, other=True, z=0.2, up=0.0)
    assert not done and not success and not info['rose'] and not info['fell']
    # Fast upright steps earn the lenient rise bonus, but cannot hand over.
    for _ in range(task.hold_steps - 1):
        _, done, success, info = step(task, env, speed=0.4)
        assert not done and not success and not info['rose']
    base = (task.gamma - 1) * task.potential(measurement(speed=0.4), False)
    r, done, success, info = step(task, env, speed=0.4)
    assert r == pytest.approx(base + 150.0)
    assert info['rose'] and not done and not success
    r, _, _, _ = step(task, env, speed=0.4)
    assert r == pytest.approx(base)
    for _ in range(task.hold_steps - 1):
        _, done, success, _ = step(task, env)
        assert not done and not success
    r, done, success, info = step(task, env)
    assert r == pytest.approx((task.gamma - 1) * task.potential(measurement(), True) + 150.0)
    assert done and success and not info['fell'] and not info['air_fail']
    assert info['rise_s'] == pytest.approx(env.state[0]['steps'] * 0.02)


def test_getup_end_no_rise_bonus_and_calm_hold():
    task = Task(hold_steps=3, max_speed=0.3, max_tilt=0.3)
    env = environment(task, 'getup_end')
    assert env.state[0]['rose']
    base = (task.gamma - 1) * task.potential(measurement(speed=0.4), False)
    for _ in range(4):
        r, done, success, info = step(task, env, speed=0.4)
        assert r == pytest.approx(base)
        assert not done and not success and not info['fell']
        assert np.isnan(info['rise_s'])
    for _ in range(2):
        _, done, success, _ = step(task, env)
        assert not done and not success
    r, done, success, info = step(task, env)
    assert r == pytest.approx((task.gamma - 1) * task.potential(measurement(), True) + 150.0)
    assert done and success and not info['fell']
    assert info['rise_s'] == pytest.approx(0.14)


@pytest.mark.parametrize('fall', [dict(other=True, feet_only=False), dict(z=0.34, feet_only=False)])
@pytest.mark.parametrize('label', ['prone', 'getup_end'])
def test_fall_only_after_rising(fall, label):
    task = Task(hold_steps=2, fall_penalty=37.0, max_speed=0.3, max_tilt=0.3)
    risen_env = environment(task, label)
    if label == 'prone':
        for _ in range(task.hold_steps):
            step(task, risen_env, speed=0.4)
    assert risen_env.state[0]['rose']
    # Same reward history, except the comparison robot has not yet risen.
    lying_env = environment(task)
    lying_env.state[0] = dict(risen_env.state[0], rose=False)
    r_before, done, success, info = step(task, lying_env, **fall)
    assert not done and not success and not info['fell']
    r_after, done, success, info = step(task, risen_env, **fall)
    assert r_after == pytest.approx(r_before - task.fall_penalty)
    assert done and not success and info['fell'] and not info['air_fail']


def test_success_precedes_fall_check():
    task = Task(hold_steps=1)
    # Deliberately inconsistent contacts exercise the precedence explicitly.
    _, done, success, info = step(task, environment(task, 'getup_end'), other=True)
    assert done and success and not info['fell']


def test_airborne_failure_unchanged():
    task = Task()
    env = environment(task)
    for _ in range(task.air_fail):
        _, done, success, info = step(task, env, airborne=True, feet_only=False)
    assert done and not success and info['air_fail'] and not info['fell']


@pytest.mark.parametrize('fraction,draw,restored', [(1.0, 0.99, True), (0.5, 0.49, True),
                                                  (0.5, 0.5, False), (0.0, 0.0, False)])
def test_saved_start_selection(tmp_path, fraction, draw, restored):
    path = tmp_path / 'states.npz'
    qpos, qvel = np.ones((3, 7)), np.zeros((3, 6))
    np.savez(path, qpos=qpos, qvel=qvel)
    task = Task(states_file=path, states_frac=fraction, crouch_frac=0.0)
    path.unlink()  # make_start must use the already-loaded states.
    env = SimpleNamespace(rng=Mock(), from_states=Mock(), lying=Mock(return_value='prone'))
    env.rng.random.return_value = draw
    assert task.make_start(env, 0) == ('getup_end' if restored else 'prone')
    if restored:
        env.from_states.assert_called_once_with(0, task.states, noise=0.02)
        np.testing.assert_array_equal(task.states['qpos'], qpos)
        np.testing.assert_array_equal(task.states['qvel'], qvel)
    else:
        env.from_states.assert_not_called()
        env.lying.assert_called_once_with(0)
    if fraction == 0.0:
        assert env.rng.random.call_count == 1


@pytest.mark.parametrize('draw,label', [(0.1, 'crouch'), (0.3, 'supine')])
def test_no_states_preserves_start_sampling(draw, label):
    task = Task(states_frac=1.0)
    env = SimpleNamespace(rng=Mock(), stance=Mock(), lying=Mock(return_value='supine'))
    env.rng.random.return_value = draw
    env.rng.uniform.return_value = 1.5
    assert task.make_start(env, 0) == label
    assert env.rng.random.call_count == 1
    if label == 'crouch':
        env.stance.assert_called_once_with(0, depth=1.5, noise=0.05, settle_steps=60)


@pytest.mark.parametrize('blocked', [dict(feet_only=False), dict(z=0.5), dict(up=0.85)])
def test_default_lenient_handover(blocked):
    task = Task(hold_steps=3)
    env = environment(task)
    assert task.max_speed == float('inf') and task.min_up == pytest.approx(0.85)
    step(task, env, speed=2.0, up=0.86)
    _, done, success, _ = step(task, env, **blocked)
    assert not done and not success
    for k in range(3):
        _, done, success, _ = step(task, env, speed=2.0, up=0.86)
        assert done == success == (k == 2)


def test_optional_tilt_limit_blocks_handover():
    task = Task(hold_steps=2, max_speed=0.3, max_tilt=0.3)
    env = environment(task)
    for _ in range(3):
        _, done, success, _ = step(task, env, speed=0.1, up=0.9)
        assert not done and not success
