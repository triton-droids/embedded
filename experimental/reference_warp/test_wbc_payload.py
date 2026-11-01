import json

import mujoco
import numpy as np
import pinocchio as pin
import pytest

from wbc_control_node.controller import WBCController
from wbc_payload import camera_model, prepare


@pytest.fixture(scope='module')
def models():
    path = prepare()
    return WBCController(path), camera_model(), path


def state_and_map(controller, model, data):
    base = data.body('floating_base')
    q, v = controller.state_from_sim(base.xpos, np.roll(data.qpos[3:7], -1),
        np.zeros(3), np.zeros(3),
        [data.qpos[model.joint(n).qposadr[0]] for n in controller.joint_names], np.zeros(10))
    transform = controller.dynamics['mujoco_base_to_pin_root']
    rotation = np.array(transform['rotation'])
    offset = np.array(transform['translation'])
    base_rotation = base.xmat.reshape(3,3)
    mapping = np.zeros((model.nv,16))
    mapping[:3,:3] = base_rotation@rotation
    mapping[:3,3:6] = base_rotation@pin.skew(offset)@rotation
    mapping[3:6,3:6] = rotation
    for i,n in enumerate(controller.joint_names):
        mapping[model.joint(n).dofadr[0],6+i] = 1
    return q, v, mapping


@pytest.mark.parametrize('seed', [0,1,2])
def test_neutral_camera_payload_mass_com_gravity_and_mass_matrix(models, seed):
    controller, model, path = models
    data = mujoco.MjData(model)
    rng = np.random.default_rng(seed)
    data.qpos[3:7] = np.roll(pin.Quaternion(pin.rpy.rpyToMatrix(*rng.uniform(-.2,.2,3))).coeffs(),1)
    for n in controller.joint_names:
        j = model.joint(n)
        data.qpos[j.qposadr[0]] = rng.uniform(max(j.range[0],-.3), min(j.range[1],.3))
    mujoco.mj_forward(model,data)
    q,v,mapping = state_and_map(controller, model, data)
    pd = controller.model.createData()
    controller.robot.computeAllTerms(pd,q,v)
    mass = np.zeros((model.nv,model.nv))
    mujoco.mj_fullM(model,mass,data.qM)
    np.testing.assert_allclose(controller.robot.mass(pd),mapping.T@mass@mapping,atol=1e-9)
    np.testing.assert_allclose(pd.com[0], data.subtree_com[model.body('floating_base').id],atol=1e-10)
    np.testing.assert_allclose(pin.computeGeneralizedGravity(controller.model,pd,q),mapping.T@data.qfrc_bias,atol=1e-9)
    expected = sum(model.body_mass[model.body(n).id] for n in ('head','d435_mount_base','d435_pitch'))
    assert sum(i.mass for i in controller.model.inertias) == pytest.approx(16+expected)
    assert json.loads(path.with_suffix('.dynamics.json').read_text())['camera_payload']['mass_kg'] == pytest.approx(expected)


def test_moving_head_approximation_is_explicit_and_quantified(models):
    controller, model, path = models
    data = mujoco.MjData(model)
    errors = []
    for yaw in (-1.0,0.0,1.0):
        for pitch in (-.6,0.0,.6):
            data.qpos[model.joint('head_yaw_joint').qposadr[0]] = yaw
            data.qpos[model.joint('head_pitch_joint').qposadr[0]] = pitch
            mujoco.mj_forward(model,data)
            q,v,mapping = state_and_map(controller,model,data)
            pd = controller.model.createData()
            gravity = pin.computeGeneralizedGravity(controller.model,pd,q)
            errors.append(float(np.max(np.abs(gravity-mapping.T@data.qfrc_bias))))
    assert np.isfinite(errors).all() and max(errors) > 1e-5
    metadata_path = path.with_suffix('.dynamics.json')
    metadata = json.loads(metadata_path.read_text())
    metadata['camera_payload']['measured_max_generalized_gravity_error'] = max(errors)
    metadata_path.write_text(json.dumps(metadata,indent=2)+'\n')
    print('Moving-head static-model max generalized gravity error:',max(errors))
