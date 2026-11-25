"""Walking demonstration labels and BC observation compatibility (no torch)."""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from getup_bc import load_episode
from walk_demo import gait_command, gait_phase, torque_equivalent_action


def test_torque_equivalent_action():
    rng = np.random.default_rng(21)
    n = 18
    c_kp, kp = rng.uniform(10, 100, (2, n))
    c_kd, kd = rng.uniform(0.1, 3, (2, n))
    c_q_ref, c_dq_ref, c_tau_ff, q, dq, q_nom = rng.normal(size=(6, n))
    args = (c_kp, c_kd, c_q_ref, c_dq_ref, c_tau_ff, q, dq, kp, kd)
    action = torque_equivalent_action(*args, np.full(n, -np.inf), np.full(n, np.inf), q_nom)
    target = action + q_nom
    tau = c_kp * (c_q_ref - q) + c_kd * (c_dq_ref - dq) + c_tau_ff
    np.testing.assert_allclose(kp * (target - q) - kd * dq, tau, atol=1e-12)
    lo, hi = np.full(n, -0.1), np.full(n, 0.1)
    clipped = torque_equivalent_action(*args, lo, hi, q_nom) + q_nom
    np.testing.assert_allclose(clipped, np.clip(target, lo, hi), atol=1e-12)
    assert np.any(target < lo) and np.any(target > hi)


@pytest.mark.parametrize('with_command', [False, True])
@pytest.mark.parametrize('time_input', [False, True])
def test_load_episode_layout(tmp_path, with_command, time_input):
    n, history, ticks = 18, 5, 4
    width = 6 + 2 * n
    frames = np.arange(ticks * width, dtype=np.float32).reshape(ticks, width)
    prev = np.arange(ticks * n, dtype=np.float32).reshape(ticks, n)
    label = prev + 1
    command = np.array([[0.1, 0, -0.2], [0, 0, 0], [0.16, 0, 0.2], [0.08, 0, 0.1]], np.float32)
    command = np.column_stack([command, [[0, 1], [0, 0], [1, 0], [-1, 0]]])
    times = np.array([2, 3, 4, 30], np.float32)
    path = tmp_path / 'episode.npz'
    np.savez(path, frame=frames, prev=prev, label=label, t=times,
             q_nominal=np.zeros(n), scale=1.0, joint_names=[f'j{i}' for i in range(n)],
             **({'command': command} if with_command else {}))
    obs, actual_label, meta = load_episode(path, history, time_input)
    assert obs.shape == (ticks, history * width + n + int(time_input) + 5 * with_command)
    if with_command and not time_input:
        assert obs.shape[1] == 233
    assert meta['command_dim'] == 5 * with_command
    np.testing.assert_array_equal(actual_label, label)
    np.testing.assert_array_equal(obs[:, history * width:history * width + n], prev)
    for i in range(ticks):
        expected = np.stack([frames[max(0, j)] for j in range(i - history + 1, i + 1)])
        np.testing.assert_array_equal(obs[i, :history * width], expected.ravel())
    if time_input:
        np.testing.assert_allclose(obs[:, history * width + n], np.clip(times / 10, 0, 2))
    if with_command:
        np.testing.assert_array_equal(obs[:, -5:], command)


@pytest.mark.parametrize('gphase,stance,next_stance,start,span', [
    ('ds', None, 'left', 0.0, 0.2),
    ('ss', 'left', 'right', 0.2, 0.3),
    ('ds', None, 'right', 0.5, 0.2),
    ('ss', 'right', 'left', 0.7, 0.3),
])
@pytest.mark.parametrize('tau', [-0.5, 0.0, 0.5, 1.0, 1.5])
def test_gait_phase(gphase, stance, next_stance, start, span, tau):
    assert gait_phase(gphase, stance, next_stance, 2 + tau * 0.3, 2, 0.3, 'left') == pytest.approx(
        start + span * np.clip(tau, 0, 1))


def test_clock_standing_and_holding():
    np.testing.assert_array_equal(gait_command(np.zeros(3), 0.25), np.zeros(5))
    moving = np.array([0.1, 0, 0.2])
    for phase, holding in [(0.25, True), (None, False)]:
        np.testing.assert_array_equal(gait_command(moving, phase, holding), [0.1, 0, 0.2, 0, 0])
    assert gait_phase('hold', None, 'left', 2, 1, 0.2, 'left') is None
    np.testing.assert_allclose(gait_command(moving, 0.25), [0.1, 0, 0.2, 1, 0], atol=1e-12)
