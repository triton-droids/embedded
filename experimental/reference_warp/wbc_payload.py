"""Generate an experimental WBC model with the camera's neutral rigid payload.

Head yaw/pitch remain real moving simulation joints. The ten-leg-joint WBC
approximates them as a payload fixed at zero joint angles; their servo reaction
and changing inertia are not represented. Original description files are untouched.
"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pinocchio as pin
import yaml

from humanoid_sim.model_builder import WorldOptions, build_model
from humanoid_sim.motor_model import limits_from_registry

ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT/'humanoid_leg_description/urdf/human_offset_corrected.urdf'


def camera_model():
    hybrid = ROOT/'humanoid_control/motor_control_hybrid'
    params = yaml.safe_load((hybrid/'config/motors.yaml').read_text())['motor_control_node']['ros__parameters']
    limits = limits_from_registry(params)
    return build_model(WorldOptions(
        str(hybrid/'models/20260914_170827/chrobot_16kg_candidate.xml'),
        str(ROOT/'humanoid_leg_description/meshes/robot_meshes'),
        {n:lim.max_torque for n,lim in limits.items()}, add_target=False,
        camera_resolution=(424,240), camera_fovy_deg=58,
        scene_builder='chase_scene:build_scene'))


def prepare():
    model = camera_model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    torso = data.body('torso')
    world_from_torso = pin.SE3(torso.xmat.reshape(3,3), torso.xpos)
    payload = pin.Inertia.Zero()
    bodies = ('d435_mount_base', 'head', 'd435_pitch')
    for name in bodies:
        body = model.body(name).id
        world_from_com = pin.SE3(data.ximat[body].reshape(3,3), data.xipos[body])
        torso_from_com = world_from_torso.inverse()*world_from_com
        inertia = pin.Inertia(float(model.body_mass[body]), np.zeros(3), np.diag(model.body_inertia[body]))
        payload += torso_from_com.act(inertia)
    tree = ET.parse(URDF)
    link = ET.SubElement(tree.getroot(), 'link', name='experimental_camera_payload')
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'origin', xyz=' '.join(f'{v:.17g}' for v in payload.lever), rpy='0 0 0')
    ET.SubElement(inertial, 'mass', value=f'{payload.mass:.17g}')
    ET.SubElement(inertial, 'inertia', **{key:f'{payload.inertia[i,j]:.17g}' for key,i,j in
        [('ixx',0,0), ('iyy',1,1), ('izz',2,2), ('ixy',0,1), ('ixz',0,2), ('iyz',1,2)]})
    joint = ET.SubElement(tree.getroot(), 'joint', name='experimental_camera_payload_fixed', type='fixed')
    ET.SubElement(joint, 'parent', link='torso')
    ET.SubElement(joint, 'child', link='experimental_camera_payload')
    ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='0 0 0')
    output = Path(__file__).resolve().parent/'generated'/'camera_payload.urdf'
    output.parent.mkdir(exist_ok=True)
    tree.write(output, encoding='unicode', xml_declaration=True)
    metadata = json.loads(URDF.with_suffix('.dynamics.json').read_text())
    metadata['camera_payload'] = dict(mass_kg=payload.mass, torso_com_m=payload.lever.tolist(),
        bodies=bodies, head_yaw_rad=0.0, head_pitch_rad=0.0,
        approximation='Rigid neutral payload; moving head inertia and servo reactions omitted',
        source_urdf_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest())
    output.with_suffix('.dynamics.json').write_text(json.dumps(metadata, indent=2)+'\n')
    return output


if __name__ == '__main__':
    print(prepare())
