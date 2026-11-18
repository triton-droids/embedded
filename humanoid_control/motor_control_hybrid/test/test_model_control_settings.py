"""Verify model PD/angle bounds without inference dependencies or hardware."""
import copy
import hashlib
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import sys

import numpy as np
import pytest
import yaml

from motor_control_hybrid.tracking_onnx import (
    TrackingPolicy, apply_runtime_policy, training_joint_limits, validate_tracking_registry)

PACKAGE = Path(__file__).resolve().parents[1]
MODEL = PACKAGE / 'models/20260914_170827/20260914_170827.onnx'


def settings():
    return yaml.safe_load((PACKAGE / 'config/motors.yaml').read_text())['motor_control_node']['ros__parameters']


def policy_from_contract(params):
    contract = params['model_contract']
    return SimpleNamespace(
        names=contract['joint_order'], offset=np.asarray(contract['default_joint_pos'], dtype=np.float32),
        scale=np.asarray(contract['action_scale'], dtype=np.float32),
        kp=np.asarray(contract['joint_stiffness'], dtype=np.float32),
        kd=np.asarray(contract['joint_damping'], dtype=np.float32),
        target_limits=training_joint_limits(MODEL, contract['joint_order']))


def test_registry_uses_model_gains_and_full_training_joint_limits():
    params = settings()
    policy = policy_from_contract(params)
    actuators = ET.parse(MODEL.parent / 'chrobot_16kg_actuated.xml').getroot().find('actuator')
    assert [a.get('joint') for a in actuators] == policy.names
    for i, actuator in enumerate(actuators):
        motor = params['motors'][policy.names[i]]
        assert motor['kp'] == float(actuator.get('kp')) == policy.kp[i]
        assert motor['kd'] == float(actuator.get('kv')) == policy.kd[i]
        np.testing.assert_array_equal(
            np.asarray([motor['min_position'], motor['max_position']], dtype=np.float32),
            [policy.target_limits[0][i], policy.target_limits[1][i]])
    assert params['hardware_verified'] is False


def test_legacy_runtime_cannot_override_model_action_or_pd():
    params = settings()
    policy = policy_from_contract(params)
    expected_kp, expected_kd = policy.kp.copy(), policy.kd.copy()
    expected_limits = tuple(value.copy() for value in policy.target_limits)
    for motor in params['motors'].values():
        motor['kp'], motor['kd'] = 300, 20
    params['runtime_policy'] = {'action_scale': 1, 'policy_action_clip': 1}
    with pytest.raises(ValueError, match='runtime_policy overrides are unsupported'):
        apply_runtime_policy(policy, params)
    np.testing.assert_array_equal(policy.kp, expected_kp)
    np.testing.assert_array_equal(policy.kd, expected_kd)
    np.testing.assert_array_equal(policy.target_limits, expected_limits)


@pytest.mark.parametrize('key', ['kp', 'kd', 'min_position', 'max_position'])
def test_registry_rejects_gain_and_limit_drift_even_with_runtime_override(key):
    params = settings()
    policy = policy_from_contract(params)
    name = policy.names[0]
    params['motors'][name][key] += 0.1
    with pytest.raises(ValueError, match='motor (gains|limits) differ'):
        validate_tracking_registry(policy, params, hashlib.sha256(MODEL.read_bytes()).hexdigest())


def test_decoded_targets_are_clipped_in_joint_space_without_polarity_change():
    params = settings()
    policy = policy_from_contract(params)
    before = copy.deepcopy(params['motors'])
    apply_runtime_policy(policy, params)
    raw_action = np.array([100, -100] * 5, dtype=np.float32)
    policy.session = SimpleNamespace(run=lambda *args: [raw_action[None, :]])
    action, target = TrackingPolicy.run(policy, np.zeros((1, 56), dtype=np.float32), 0)
    np.testing.assert_array_equal(action, raw_action)
    expected = np.clip(policy.offset + policy.scale * action, *policy.target_limits)
    np.testing.assert_array_equal(target, expected)
    assert [m['direction'] for m in params['motors'].values()] == [-1, 1, 1, 1, 1, 1, -1, -1, -1, -1]
    assert params['motors'] == before


def test_original_action_and_observation_contract_are_preserved():
    params = settings()
    policy = policy_from_contract(params)
    apply_runtime_policy(policy, params)
    np.testing.assert_array_equal(policy.offset, np.zeros(10))
    np.testing.assert_allclose(policy.scale, np.full(10, 0.2))
    raw = np.array([2., -3.] * 5, dtype=np.float32)
    policy.session = SimpleNamespace(run=lambda *args: [raw[None, :]])
    action, target = TrackingPolicy.run(policy, np.zeros((1, 56), dtype=np.float32), 0)
    np.testing.assert_array_equal(action, raw)
    np.testing.assert_allclose(target, np.clip(0.2 * raw, *policy.target_limits))
    policy.reference_q = policy.reference_dq = np.zeros((1, 10), dtype=np.float32)
    policy.frames = 1
    obs = TrackingPolicy.observation(policy, 0, np.zeros(3), np.array([0, 0, -1]),
                                     np.zeros(10), np.zeros(10), action)
    np.testing.assert_array_equal(obs[0, 23:33], np.zeros(10))
    np.testing.assert_array_equal(obs[0, 43:53], raw)


def test_sync_restores_model_pd_and_preserves_polarity(monkeypatch, tmp_path):
    """Test the real sync script using metadata fixtures, without ONNX Runtime."""
    params = settings()
    original = copy.deepcopy(params)
    for motor in params['motors'].values():
        motor.update(kp=300, kd=20)
    params['runtime_policy'] = {'action_scale': 1, 'policy_action_clip': 1}
    output = tmp_path / 'motors.yaml'
    output.write_text(yaml.safe_dump({'motor_control_node': {'ros__parameters': params}}))
    contract = params['model_contract']
    metadata = {key: ','.join(map(str, contract[key])) for key in (
        'default_joint_pos', 'action_scale', 'joint_stiffness', 'joint_damping')}
    metadata.update(joint_names=','.join(contract['joint_order']),
                    observation_names=contract['observation_names'])
    monkeypatch.setitem(sys.modules, 'onnx', SimpleNamespace(load=lambda path: SimpleNamespace(
        metadata_props=[SimpleNamespace(key=k, value=v) for k, v in metadata.items()])))
    script = PACKAGE.parents[1] / 'scripts/sync_tracking_motor_config.py'
    spec = importlib.util.spec_from_file_location('model_pd_sync_test', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(sys, 'argv', [str(script), '--model', str(MODEL), '--output', str(output)])
    module.main()
    updated = yaml.safe_load(output.read_text())['motor_control_node']['ros__parameters']
    assert updated['motors'] == original['motors']
    assert 'runtime_policy' not in updated
