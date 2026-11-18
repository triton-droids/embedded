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


def run_sync(monkeypatch, *argv):
    """Run the real sync script with ONNX metadata taken from the registry contract."""
    contract = settings()['model_contract']
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
    monkeypatch.setattr(sys, 'argv', [str(script), *map(str, argv)])
    module.main()


def read_registry(path):
    return yaml.safe_load(path.read_text())['motor_control_node']['ros__parameters']


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def alternate_export(tmp_path, candidate_xml=None):
    """Different bytes (so a different hash) beside the same saved training XML."""
    directory = tmp_path / 'alternate'
    directory.mkdir()
    (directory / 'alternate.onnx').write_bytes(MODEL.read_bytes() + b'alternate')
    (directory / 'chrobot_16kg_actuated.xml').write_bytes(
        (MODEL.parent / 'chrobot_16kg_actuated.xml').read_bytes())
    (directory / 'chrobot_16kg_candidate.xml').write_text(
        candidate_xml or (MODEL.parent / 'chrobot_16kg_candidate.xml').read_text())
    return directory / 'alternate.onnx'


def test_sync_restores_model_pd_and_preserves_polarity(monkeypatch, tmp_path):
    """Test the real sync script using metadata fixtures, without ONNX Runtime."""
    params = settings()
    original = copy.deepcopy(params)
    for motor in params['motors'].values():
        motor.update(kp=300, kd=20)
    params['runtime_policy'] = {'action_scale': 1, 'policy_action_clip': 1}
    output = tmp_path / 'motors.yaml'
    output.write_text(yaml.safe_dump({'motor_control_node': {'ros__parameters': params}}))
    run_sync(monkeypatch, '--model', MODEL, '--output', output)
    updated = read_registry(output)
    assert updated['motors'] == original['motors']
    assert 'runtime_policy' not in updated
    # The gain drift changed the contract, so no earlier approval survives.
    assert updated['model_contract']['allowed_model_sha256s'] == [sha256(MODEL)]


def test_registry_rejects_unapproved_export():
    params = settings()
    policy = policy_from_contract(params)
    with pytest.raises(ValueError, match='not approved'):
        validate_tracking_registry(policy, params, '0' * 64)
    del params['model_contract']['allowed_model_sha256s']
    params['model_contract']['model_sha256'] = sha256(MODEL)
    with pytest.raises(ValueError, match='resync'):
        validate_tracking_registry(policy, params, sha256(MODEL))


def test_approve_adds_matching_export_and_resync_keeps_it(monkeypatch, tmp_path):
    output = tmp_path / 'motors.yaml'
    output.write_text((PACKAGE / 'config/motors.yaml').read_text())
    alternate = alternate_export(tmp_path)
    run_sync(monkeypatch, '--approve-model', alternate, '--output', output)
    approved = read_registry(output)
    assert approved['model_contract']['allowed_model_sha256s'] == [sha256(MODEL), sha256(alternate)]
    assert approved['motors'] == settings()['motors']
    run_sync(monkeypatch, '--approve-model', alternate, '--output', output)
    assert read_registry(output) == approved
    run_sync(monkeypatch, '--model', MODEL, '--output', output)
    assert read_registry(output)['model_contract']['allowed_model_sha256s'] == [
        sha256(MODEL), sha256(alternate)]


def test_approve_rejects_export_with_different_limits(monkeypatch, tmp_path):
    output = tmp_path / 'motors.yaml'
    output.write_text((PACKAGE / 'config/motors.yaml').read_text())
    root = ET.parse(MODEL.parent / 'chrobot_16kg_candidate.xml').getroot()
    joint = next(j for j in root.iter('joint') if j.get('name') == settings()['model_contract']['joint_order'][0])
    low, high = map(float, joint.get('range').split())
    joint.set('range', f'{low} {high - 0.1}')
    alternate = alternate_export(tmp_path, ET.tostring(root, encoding='unicode'))
    with pytest.raises(ValueError, match='does not match the registry'):
        run_sync(monkeypatch, '--approve-model', alternate, '--output', output)
    assert output.read_text() == (PACKAGE / 'config/motors.yaml').read_text()
