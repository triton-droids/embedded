"""Reference extraction, circular interpolation and walking rewards without MuJoCo."""
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from walk_ref import build_reference
from rl_tasks.walk import Task, interpolate_reference


NAMES = [f'leg{i}' for i in range(12)] + [f'shoulder{i}' for i in range(6)]


def episode(path, phases, vx=0.08, active=True):
    phase = np.asarray(phases)
    frame = np.zeros((len(phase), 42))
    frame[:, 6:24] = phase[:, None]
    cmd = np.zeros((len(phase), 5))
    cmd[:, 0] = vx
    if active:
        cmd[:, 3:5] = np.column_stack((np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)))
    np.savez(path, frame=frame, command=cmd, q_nominal=np.arange(18) / 10,
             joint_names=NAMES, period=1.0, stance=0.7)


def test_binning_and_filters(tmp_path):
    a, b = tmp_path / 'a', tmp_path / 'b'
    a.mkdir()
    b.mkdir()
    episode(a / 'one.npz', [0.1] * 10 + [0.3] * 10)
    episode(b / 'two.npz', [0.6] * 10 + [0.8] * 10)
    episode(a / 'standing.npz', [0.1] * 30, active=False)
    episode(a / 'fast.npz', [0.1] * 30, vx=0.2)
    out = a / 'ref.npz'
    ref = build_reference([a, b], bins=2, out=out)
    np.testing.assert_array_equal(ref['count'], [20, 20])
    np.testing.assert_allclose(ref['phase'], [0.25, 0.75])
    np.testing.assert_allclose(ref['q_ref'], np.arange(18)[None, :] / 10 + np.array([[0.2], [0.7]]))
    np.testing.assert_allclose(ref['q_std'], 0.1)
    with np.load(out) as saved:
        assert set(saved.files) == {'phase', 'q_ref', 'q_std', 'count', 'q_nominal', 'joint_names',
                                   'period', 'stance', 'vx_range'}
    np.testing.assert_allclose(build_reference([a, b], bins=2, out=out)['q_ref'], ref['q_ref'])


@pytest.mark.parametrize('ticks', [0, 19])
def test_underfilled_bin(tmp_path, ticks):
    episode(tmp_path / 'ep.npz', [0.1] * 20 + [0.6] * ticks)
    with pytest.raises(ValueError, match='>= 20'):
        build_reference([tmp_path], bins=2)


def test_circular_interpolation():
    phases = np.array([0.25, 0.75])
    poses = np.array([[0.] * 18, [2.] * 18])
    np.testing.assert_allclose(interpolate_reference(0.99, phases, poses), 1.04)
    np.testing.assert_allclose(interpolate_reference(-0.01, phases, poses), 1.04)
    np.testing.assert_allclose(interpolate_reference(0.25, phases, poses), 0)
    np.testing.assert_allclose(interpolate_reference(0.5, phases, poses), 1)


@pytest.fixture
def ref_file(tmp_path):
    path = tmp_path / 'ref.npz'
    np.savez(path, phase=[0.25, 0.75], q_ref=np.zeros((2, 18)), joint_names=NAMES, period=1., stance=0.7)
    return str(path)


def fake_env(task, moving=True):
    data = SimpleNamespace(site_xpos=np.array([[-0.15, 0, 0], [0.15, 0, 0]]), qvel=np.zeros(18))
    env = SimpleNamespace(names=NAMES, leg=np.arange(18) < 12, state=[{}],
                          sims=[SimpleNamespace(data=data, qvel=np.arange(18))], sole_sites=[[0, 1]],
                          rng=np.random.default_rng(1), command=np.zeros((1, 5)), q_nom=np.zeros(18),
                          t=[0], steps_max=2)
    task.on_reset(env, 0)
    env.state[0].update(moving=moving, phase=0.99)
    env.command[0, :3] = [0.08 if moving else 0, 0, 0]
    task._clock(env, 0)
    meas = dict(other=False, z=0.565, up=1., vy=0.08, vx=0., yaw=0., dq=np.zeros(18), q=np.zeros(18),
                yaw_rate=0., vz=0., ang_body=np.zeros(3), airborne=False, power=0., speed_ratio=0.,
                left=True, right=True)
    return env, meas


@pytest.mark.parametrize('moving,active,expected', [(True, True, 2.5), (False, False, 0), (True, False, 0)])
def test_reward_and_episode_mean(ref_file, moving, active, expected):
    task = Task(ref_file=ref_file, ref_w=2.5, period=1., stance=0.7)
    baseline = Task(period=1., stance=0.7)
    env, meas = fake_env(task, moving)
    base_env, _ = fake_env(baseline, moving)
    if not active:
        env.command[0, 3:5] = base_env.command[0, 3:5] = 0
    r = task.reward(env, 0, meas, np.zeros(18))[0]
    base = baseline.reward(base_env, 0, meas, np.zeros(18))[0]
    assert r - base == pytest.approx(expected)
    # Second moving sample has error squared equal to sigma: mean is (1 + exp(-1))/2.
    if active:
        meas['q'][0] = 0.5
    env.t[0] = 1
    info = task.reward(env, 0, meas, np.zeros(18))[3]
    assert info['ref_match'] == pytest.approx((1 + np.exp(-1)) / 2 if active else (1 if moving else 0))
    task.on_reset(env, 0)
    assert env.state[0]['ref_n'] == 0


@pytest.mark.parametrize('legs_only', [True, False])
def test_joint_mask(ref_file, legs_only):
    task = Task(ref_file=ref_file, ref_w=1., period=1., stance=0.7, ref_legs_only=legs_only)
    env, meas = fake_env(task)
    meas['q'][-1] = 0.5
    assert task._reference_reward(env, 0, meas) == pytest.approx(1 if legs_only else np.exp(-1))


@pytest.mark.parametrize('kwargs,match', [({'period': 0.7, 'stance': 0.7}, 'period'),
                                        ({'period': 1., 'stance': 0.6}, 'stance'),
                                        ({'period': 1., 'stance': 0.7, 'ref_sigma': 0}, 'ref_sigma')])
def test_invalid_reference_settings(ref_file, kwargs, match):
    with pytest.raises(ValueError, match=match):
        Task(ref_file=ref_file, **kwargs)


def test_joint_order_and_default_info(ref_file):
    task = Task(ref_file=ref_file, period=1., stance=0.7)
    env, _ = fake_env(task)
    env.names = NAMES[::-1]
    with pytest.raises(ValueError, match='joint_names'):
        task.on_reset(env, 0)
    baseline = Task()
    env, meas = fake_env(baseline)
    env.t[0] = 1
    assert 'ref_match' not in baseline.reward(env, 0, meas, np.zeros(18))[3]


def test_falling_step_match(ref_file):
    task = Task(ref_file=ref_file, ref_w=1., period=1., stance=0.7)
    env, meas = fake_env(task)
    meas['other'] = True
    reward, done, success, info = task.reward(env, 0, meas, np.zeros(18))
    assert (reward, done, success) == (-9., True, False)
    assert info['ref_match'] == 1.
