import numpy as np
import pytest

from walking_controller.getup_policy import GetUpPolicy, frame, observation, quat_rotate_inverse


def test_gravity_in_body_frame():
    upright = frame(np.zeros(3), [1, 0, 0, 0], np.zeros(2), np.zeros(2), np.zeros(2))
    np.testing.assert_allclose(upright[3:6], [0, 0, -1], atol=1e-12)
    # Lying on the back: pitched 90 degrees about y, gravity along body +x... or -x.
    s = np.sqrt(0.5)
    pitched = quat_rotate_inverse([s, 0, s, 0], [0, 0, -1])
    np.testing.assert_allclose(np.abs(pitched), [1, 0, 0], atol=1e-12)


def test_policy_step_layout(tmp_path):
    n, history = 2, 3
    obs = history * (6 + 2 * n) + n + 1
    path = tmp_path / 'p.npz'
    np.savez(path, obs_mean=np.zeros(obs), obs_std=np.ones(obs), layers=2,
             w0=np.ones((4, obs)) * 0.01, b0=np.zeros(4), w1=np.ones((n, 4)), b1=np.zeros(n),
             q_nominal=np.array([0.1, -0.2]), scale=0.5, history=history, time_input=True,
             joint_names=np.array(['a', 'b']))
    policy = GetUpPolicy(path)
    f = frame(np.zeros(3), [1, 0, 0, 0], [0.1, -0.2], np.zeros(n), policy.q_nominal)
    action, target = policy.step(f, 3.0)
    expected = policy(observation([f] * history, np.zeros(n), 3.0))
    np.testing.assert_allclose(action, expected)
    np.testing.assert_allclose(target, policy.q_nominal + 0.5 * action)
    np.testing.assert_allclose(policy.prev, action)
    policy.reset()
    assert policy.frames is None and not policy.prev.any()


def _walk_policy(tmp_path, n=18, history=2):
    obs = history * (6 + 2 * n) + n + 3
    path = tmp_path / 'walk.npz'
    np.savez(path, obs_mean=np.zeros(obs), obs_std=np.ones(obs), layers=1, w0=np.zeros((n, obs)), b0=np.zeros(n),
             q_nominal=np.linspace(-0.1, 0.1, n), scale=1.0, history=history, command_dim=3, task='walk',
             joint_names=np.array([f'j{i}' for i in range(n)]))
    return path


def test_policy_walk_controller(tmp_path):
    from walking_controller.policy_walk import PolicyWalkController
    path = _walk_policy(tmp_path)
    names = [f'j{i}' for i in range(18)]
    c = PolicyWalkController(path, names, np.full(18, 50.0), np.full(18, 1.0), np.full(18, -1.0), np.full(18, 1.0))
    c.set_velocity(1.0, 0.1)
    np.testing.assert_allclose(c.command, [0.3, 0.0, 0.1])          # clipped to max_speed
    c.measured = ([1, 0, 0, 0], [0, 0, 0], c.q_nominal, np.zeros(18))
    held = c.step(0.0, None, None, None, base_held=True)
    np.testing.assert_allclose(held.q_ref, c.q_nominal)              # gantry: hold the stance
    cmd = c.step(0.1, None, None, None, base_held=False)
    np.testing.assert_allclose(cmd.q_ref, c.q_nominal)               # zero network -> stance
    assert c.phase == 'active'
    c.measured = ([0.8, 0.6, 0, 0], [0, 0, 0], c.q_nominal, np.zeros(18))
    assert c.step(0.2, None, None, None, base_held=False) is None and 'tilt' in c.fault


def test_policy_walk_controller_rejects_recovery_policies(tmp_path):
    from walking_controller.policy_walk import PolicyWalkController
    path = tmp_path / 'p.npz'
    n, obs = 18, 2 * (6 + 36) + 18
    np.savez(path, obs_mean=np.zeros(obs), obs_std=np.ones(obs), layers=1, w0=np.zeros((n, obs)), b0=np.zeros(n),
             q_nominal=np.zeros(n), scale=1.0, history=2, joint_names=np.array([f'j{i}' for i in range(n)]))
    with pytest.raises(ValueError):
        PolicyWalkController(path, [f'j{i}' for i in range(n)], np.ones(n), np.ones(n), -np.ones(n), np.ones(n))


def _walk_policy5(tmp_path, n=18, history=2):
    obs = history * (6 + 2 * n) + n + 5
    path = tmp_path / 'walk5.npz'
    np.savez(path, obs_mean=np.zeros(obs), obs_std=np.ones(obs), layers=1, w0=np.zeros((n, obs)), b0=np.zeros(n),
             q_nominal=np.zeros(n), scale=1.0, history=history, command_dim=5, task='walk',
             joint_names=np.array([f'j{i}' for i in range(n)]))
    return path


def test_policy_walk_gait_clock(tmp_path):
    from walking_controller.policy_walk import PolicyWalkController
    names = [f'j{i}' for i in range(18)]
    c = PolicyWalkController(_walk_policy5(tmp_path), names, np.full(18, 50.0), np.ones(18), -np.ones(18),
                             np.ones(18), period=1.0)
    c.measured = ([1, 0, 0, 0], [0, 0, 0], np.zeros(18), np.zeros(18))
    c.set_velocity(0.1, 0.0)
    c.step(0.0, None, None, None, base_held=False)
    assert c.gait_phase == 0.0
    np.testing.assert_allclose(c._policy_command(0.25), [0.1, 0, 0, 1.0, 0.0], atol=1e-9)  # phase 0.25
    assert abs(c.gait_phase - 0.25) < 1e-9
    c.set_velocity(0.0, 0.0)
    np.testing.assert_allclose(c._policy_command(0.5), np.zeros(5))
    assert c.gait_phase == 0.0                                     # zero command resets the clock
