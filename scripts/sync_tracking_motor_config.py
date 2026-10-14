#!/usr/bin/env python3
"""Sync simulation joint order/PD metadata without inventing hardware mappings."""
import argparse
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import onnx
import yaml


def main():
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=repo.parent / 'simulation/logs/legs_tracking/'
                        '20260914_170827/20260914_170827.onnx')
    parser.add_argument('--output', type=Path, default=repo / 'humanoid_control/'
                        'motor_control_hybrid/config/motors.yaml')
    args = parser.parse_args()
    model = args.model.resolve()
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
    previous = {}
    if args.output.exists():
        previous = yaml.safe_load(args.output.read_text())['motor_control_node']['ros__parameters']
    hardware_keys = ('can_interface', 'master_id', 'motor_id', 'model', 'actuator_type',
                     'direction', 'encoder_offset_rad', 'max_torque_nm')
    motors = {}
    for index, name in enumerate(names):
        old = previous.get('motors', {}).get(name, {})
        limits = [float(value) for value in joints[name].get('range').split()]
        motors[name] = {key: old.get(key) for key in hardware_keys}
        motors[name].update(min_position=limits[0], max_position=limits[1],
                            kp=old.get('kp', kp[index]) if 'runtime_policy' in previous else kp[index],
                            kd=old.get('kd', kd[index]) if 'runtime_policy' in previous else kd[index])
        if 'ankle_mapping' in old:
            motors[name]['ankle_mapping'] = old['ankle_mapping']
    params = {
        # A new simulation sync always requires hardware review before CAN use.
        'hardware_verified': False,
        'model_contract': {
            'format': 'tracking_onnx', 'model_sha256': hashlib.sha256(model.read_bytes()).hexdigest(),
            'control_hz': 50.0, 'joint_order': names, 'default_joint_pos': offsets,
            'action_scale': scales, 'joint_stiffness': kp, 'joint_damping': kd,
            'observation_names': metadata['observation_names'],
        },
        'MAX_VEL_RAD_S': previous.get('MAX_VEL_RAD_S', 4.5),
        'motors': motors,
    }
    for key in ('runtime_policy', 'default_can_interface', 'default_master_id', 'KP', 'KD'):
        if key in previous:
            params[key] = previous[key]
    header = ('# Synced from the tracking ONNX export and its saved training XML.\n'
              '# Gains and limits are simulation values, not validated hardware values.\n'
              '# null hardware fields must be calibrated; action indices are not CAN IDs.\n')
    args.output.write_text(header + yaml.safe_dump(
        {'motor_control_node': {'ros__parameters': params}}, sort_keys=False))
    print(f'Synced {len(names)} joints to {args.output}; hardware_verified=false')


if __name__ == '__main__':
    main()
