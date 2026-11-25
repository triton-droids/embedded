#!/usr/bin/env python3
"""Sync simulation joint order/PD metadata without inventing hardware mappings.

--model rebuilds the registry from one export. --approve-model only adds another
export's SHA-256 to allowed_model_sha256s, so tracking_policy_node may preload it
for switching; it must reproduce the registry's contract, gains and limits exactly.
"""
import argparse
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import onnx
import yaml


def sha256(model):
    return hashlib.sha256(model.read_bytes()).hexdigest()


def allowed_hashes(contract):
    # Registries written before allowed_model_sha256s pinned a single model_sha256.
    return list(contract.get('allowed_model_sha256s') or [contract['model_sha256']])


def same_contract(params, other):
    """Everything an approved export must share; approvals themselves excluded."""
    def strip(contract):
        return {key: value for key, value in contract.items()
                if key not in ('model_sha256', 'allowed_model_sha256s')}
    return (strip(params['model_contract']) == strip(other['model_contract'])
            and params['motors'] == other['motors'])


def synced_params(model, previous):
    metadata = {item.key: item.value for item in onnx.load(model).metadata_props}
    names = metadata['joint_names'].split(',')

    def vector(key):
        values = [float(value) for value in metadata[key].split(',')]
        if len(values) == 1:
            values *= len(names)
        if len(values) != len(names):
            raise ValueError(f'{key} does not match joint count')
        return values

    kp, kd = vector('joint_stiffness'), vector('joint_damping')
    offsets, scales = vector('default_joint_pos'), vector('action_scale')
    actuators = ET.parse(model.parent / 'chrobot_16kg_actuated.xml').getroot().find('actuator')
    if names != [item.get('joint') for item in actuators]:
        raise ValueError('ONNX and XML actuator order differ')
    if kp != [float(item.get('kp')) for item in actuators] or kd != [float(item.get('kv')) for item in actuators]:
        raise ValueError('ONNX and XML gains differ')
    joints = {item.get('name'): item for item in
              ET.parse(model.parent / 'chrobot_16kg_candidate.xml').getroot().iter('joint')}
    hardware_keys = ('can_interface', 'master_id', 'motor_id', 'model', 'actuator_type',
                     'direction', 'encoder_offset_rad', 'max_torque_nm')
    motors = {}
    for index, name in enumerate(names):
        old = previous.get('motors', {}).get(name, {})
        limits = [float(value) for value in joints[name].get('range').split()]
        motors[name] = {key: old.get(key) for key in hardware_keys}
        motors[name].update(min_position=limits[0], max_position=limits[1],
                            kp=kp[index], kd=kd[index])
        if 'ankle_mapping' in old:
            motors[name]['ankle_mapping'] = old['ankle_mapping']
    params = {
        # A new simulation sync always requires hardware review before CAN use.
        'hardware_verified': False,
        'model_contract': {
            'format': 'tracking_onnx', 'allowed_model_sha256s': [sha256(model)],
            'control_hz': 50.0, 'joint_order': names, 'default_joint_pos': offsets,
            'action_scale': scales, 'joint_stiffness': kp, 'joint_damping': kd,
            'observation_names': metadata['observation_names'],
        },
        'MAX_VEL_RAD_S': previous.get('MAX_VEL_RAD_S', 4.5),
        'motors': motors,
    }
    for key in ('default_can_interface', 'default_master_id', 'KP', 'KD'):
        if key in previous:
            params[key] = previous[key]
    return params


def main():
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=repo / 'control/motor_control_hybrid/models/'
                        '20260914_170827/20260914_170827.onnx')
    parser.add_argument('--approve-model', type=Path,
                        help='add this export to allowed_model_sha256s without changing the contract')
    parser.add_argument('--output', type=Path, default=repo / 'control/'
                        'motor_control_hybrid/config/motors.yaml')
    args = parser.parse_args()
    previous = {}
    if args.output.exists():
        previous = yaml.safe_load(args.output.read_text())['motor_control_node']['ros__parameters']
    if args.approve_model:
        if not previous:
            raise ValueError(f'{args.output} does not exist; sync it with --model first')
        model = args.approve_model.resolve()
        if not same_contract(synced_params(model, previous), previous):
            raise ValueError(f'{model} does not match the registry contract, gains or limits; '
                             'it cannot be switched to under this registry')
        params = previous
        allowed = allowed_hashes(params['model_contract'])
        if sha256(model) not in allowed:
            allowed.append(sha256(model))
        params['model_contract'].pop('model_sha256', None)
        params['model_contract']['allowed_model_sha256s'] = allowed
        # Approving a policy is a change that needs the same review as a resync.
        params['hardware_verified'] = False
        message = f'Approved {model.name} in {args.output}'
    else:
        model = args.model.resolve()
        params = synced_params(model, previous)
        # Approvals stay valid only while the contract they were checked against is unchanged.
        if previous and same_contract(params, previous):
            params['model_contract']['allowed_model_sha256s'] += [
                digest for digest in allowed_hashes(previous['model_contract'])
                if digest not in params['model_contract']['allowed_model_sha256s']]
        message = f'Synced {len(params["motors"])} joints to {args.output}'
    header = ('# Synced from the tracking ONNX export and its saved training XML.\n'
              '# Gains and limits are simulation values, not validated hardware values.\n'
              '# null hardware fields must be calibrated; action indices are not CAN IDs.\n')
    args.output.write_text(header + yaml.safe_dump(
        {'motor_control_node': {'ros__parameters': params}}, sort_keys=False))
    print(f'{message}; hardware_verified=false')


if __name__ == '__main__':
    main()
