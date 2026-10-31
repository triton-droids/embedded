import hashlib
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "simulation/humanoid_sim"))

from experimental.kinematics import WBCController
from humanoid_sim.model_builder import WorldOptions, build_model
from humanoid_sim.motor_model import limits_from_registry

URDF = ROOT / "humanoid_leg_description/urdf/human_offset_corrected.urdf"
MJCF = ROOT / "humanoid_control/motor_control_hybrid/models/20260914_170827/chrobot_16kg_candidate.xml"


def make_models():
    params = yaml.safe_load(
        (ROOT / "humanoid_control/motor_control_hybrid/config/motors.yaml").read_text()
    )["motor_control_node"]["ros__parameters"]
    limits = limits_from_registry(params)
    model = build_model(WorldOptions(
        str(MJCF), str(ROOT / "humanoid_leg_description/meshes/robot_meshes"),
        {name: limit.max_torque for name, limit in limits.items()},
        add_head=False, add_target=False,
    ))
    data = mujoco.MjData(model)
    data.eq_active[:] = 0
    mujoco.mj_forward(model, data)
    return WBCController(URDF), model, data, limits


def state(controller, model, data):
    base = data.body("floating_base")
    rotation = base.xmat.reshape(3, 3)
    names = controller.joint_names
    return controller.state_from_sim(
        base.xpos, np.roll(data.qpos[3:7], -1), data.qvel[:3],
        rotation @ data.qvel[3:6],
        [data.qpos[model.joint(n).qposadr[0]] for n in names],
        [data.qvel[model.joint(n).dofadr[0]] for n in names],
    )


def velocity_map(controller, data):
    transform = controller.dynamics["mujoco_base_to_pin_root"]
    offset_rotation = np.array(transform["rotation"])
    offset = np.array(transform["translation"])
    base_rotation = data.body("floating_base").xmat.reshape(3, 3)
    root_rotation = base_rotation @ offset_rotation
    mapping = np.eye(16)
    mapping[:3, :3] = root_rotation
    mapping[:3, 3:6] = base_rotation @ pin.skew(offset) @ offset_rotation
    mapping[3:6, 3:6] = offset_rotation
    return mapping


@pytest.mark.parametrize("seed", range(6))
def test_model_equivalence(seed):
    controller, model, data, _ = make_models()
    rng = np.random.default_rng(seed)
    data.qpos[:3] += rng.uniform(-.1, .1, 3)
    data.qpos[3:7] = np.roll(
        pin.Quaternion(pin.rpy.rpyToMatrix(*rng.uniform(-.3, .3, 3))).coeffs(), 1
    )
    for name in controller.joint_names:
        jid = model.joint(name).id
        lo, hi = model.jnt_range[jid]
        data.qpos[model.jnt_qposadr[jid]] = rng.uniform(max(lo, -.5), min(hi, .5))
    data.qvel[:] = rng.uniform(-.2, .2, model.nv)
    mujoco.mj_forward(model, data)
    q, v = state(controller, model, data)
    mapping = velocity_map(controller, data)
    np.testing.assert_allclose(mapping @ v, data.qvel, atol=1e-12)

    pd = controller.model.createData()
    controller.robot.computeAllTerms(pd, q, v)
    mass = controller.robot.mass(pd)
    mj_mass = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, mj_mass, data.qM)
    np.testing.assert_allclose(mass, mapping.T @ mj_mass @ mapping, atol=1e-9)
    np.testing.assert_allclose(pd.com[0], data.subtree_com[model.body("floating_base").id], atol=1e-10)
    assert abs(sum(i.mass for i in controller.model.inertias) - 16.0) < 1e-9

    for side in ("left", "right"):
        name = side + "_foot"
        frame = pd.oMf[controller.model.getFrameId(name)]
        ankle = model.joint(side + "_ankle_joint").id
        np.testing.assert_allclose(frame.translation, data.xanchor[ankle], atol=1e-10)
        points = np.array(controller.dynamics["contact_points"][name])
        geom = model.geom(side + "_foot_collision_box").id
        local = data.geom_xmat[geom].reshape(3, 3).T @ (
            frame.rotation @ points + frame.translation[:, None] - data.geom_xpos[geom, :, None]
        )
        np.testing.assert_allclose(local[2], -model.geom_size[geom, 2], atol=1e-10)
        np.testing.assert_allclose(np.abs(local[:2]), np.repeat(model.geom_size[geom, :2, None], 4, axis=1), atol=1e-10)

    acceleration = rng.uniform(-.3, .3, 16)
    offset = np.array(controller.dynamics["mujoco_base_to_pin_root"]["translation"])
    base_rotation = data.body("floating_base").xmat.reshape(3, 3)
    root_rotation = pin.Quaternion(q[3:7]).matrix()
    derivative_term = np.zeros(16)
    derivative_term[:3] = (
        root_rotation @ np.cross(v[3:6], v[:3])
        + base_rotation @ np.cross(data.qvel[3:6], np.cross(offset, data.qvel[3:6]))
    )
    mj_acceleration = mapping @ acceleration + derivative_term
    expected = mapping.T @ (mj_mass @ mj_acceleration + data.qfrc_bias - data.qfrc_passive)
    actual = pin.rnea(controller.model, pd, q, v, acceleration).copy()
    actual[6:] += controller.robot.rotor_inertias * acceleration[6:]
    actual[6:] += controller.joint_damping * v[6:]
    np.testing.assert_allclose(actual, expected, atol=1e-8)

    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    gravity = pin.computeGeneralizedGravity(controller.model, pd, q)
    np.testing.assert_allclose(gravity, mapping.T @ data.qfrc_bias, atol=1e-9)


def test_only_inertials_changed():
    before = ET.fromstring(subprocess.check_output(
        ["git", "show", "HEAD:humanoid_leg_description/urdf/human_offset_corrected.urdf"], cwd=ROOT
    ))
    after = ET.parse(URDF).getroot()
    for tree in (before, after):
        for element in tree.iter():
            element.text = None
            element.tail = None
        for link in tree.findall("link"):
            inertial = link.find("inertial")
            if inertial is not None:
                link.remove(inertial)
    assert ET.tostring(before) == ET.tostring(after)


def test_source_metadata_matches():
    controller, _, _, _ = make_models()
    assert controller.dynamics["source_sha256"] == hashlib.sha256(MJCF.read_bytes()).hexdigest()


@pytest.mark.parametrize("bend", [0.0, .1])
def test_headless_double_support(bend):
    controller, model, data, limits = make_models()
    for side in ("left", "right"):
        for suffix, angle in [("hip1", bend), ("knee", -2 * bend), ("ankle", bend)]:
            name = f"{side}_{suffix}_joint"
            data.qpos[model.joint(name).qposadr[0]] = angle
    mujoco.mj_forward(model, data)
    data.qpos[2] -= min(data.site(side + "_foot").xpos[2] for side in ("left", "right"))
    mujoco.mj_forward(model, data)
    q, v = state(controller, model, data)
    q_ref = q[7:].copy()
    for side in ("left", "right"):
        controller.add_foot_contact(side + "_foot", q)
    base_start = data.body("floating_base").xpos.copy()
    com_start = pin.centerOfMass(controller.model, controller.data, q).copy()
    max_torque = 0.0
    for step in range(2000):
        q, v = state(controller, model, data)
        tau = controller.compute(q, v, q_ref, step * model.opt.timestep)
        max_torque = max(max_torque, np.max(np.abs(tau)))
        for i, name in enumerate(controller.joint_names):
            assert abs(tau[i]) <= limits[name].max_torque
            data.ctrl[model.actuator(name + "_motor").id] = tau[i]
        data.xfrc_applied[model.body("floating_base").id, 0] = (
            5.0 if 400 <= step < 420 else 0.0
        )
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    q, _ = state(controller, model, data)
    com_end = pin.centerOfMass(controller.model, controller.data, q).copy()
    rotation = data.body("floating_base").xmat.reshape(3, 3)
    assert np.linalg.norm(data.body("floating_base").xpos - base_start) < .02
    assert np.linalg.norm(com_end - com_start) < .02
    assert np.linalg.norm(pin.log3(rotation)) < .05
    assert not np.any(data.eq_active)
    print(f"10s stance, bend={bend}, 5N/0.1s push: CoM drift={np.linalg.norm(com_end-com_start):.6g} m, max torque={max_torque:.6g} Nm")


@pytest.mark.parametrize("priority", [0, 1])
def test_contact_removal_preserves_hard_constraint_count(priority):
    controller, model, data, limits = make_models()
    q, _ = state(controller, model, data)
    controller.set_torque_limits([limits[n].max_torque for n in controller.joint_names])
    baseline = (controller.form.nVar, controller.form.nEq, controller.form.nIn)
    for _ in range(10):
        for frame in ("left_foot", "right_foot"):
            controller.add_foot_contact(frame, q, motion_priority=priority)
        assert controller.form.nEq == baseline[1] + (12 if priority == 0 else 0)
        for remaining, frame in [(1, "left_foot"), (0, "right_foot")]:
            name = frame + "_contact"
            # Assert dimensions before passing them to the native solver: older
            # TSID releases underflow nEq here and corrupt the solver's memory.
            assert controller.form.removeRigidContact(name, 0.0)
            assert controller.form.nEq == baseline[1] + (6 * remaining if priority == 0 else 0)
            del controller.contacts[name]
            controller._resize_solver()
        assert (controller.form.nVar, controller.form.nEq, controller.form.nIn) == baseline


def test_contact_switching_solves_after_each_transition():
    controller, model, data, limits = make_models()
    q, v = state(controller, model, data)
    controller.set_torque_limits([limits[n].max_torque for n in controller.joint_names])
    for cycle in range(10):
        for active in [
            {"left_foot", "right_foot"}, {"left_foot"}, set(),
            {"right_foot"}, {"left_foot", "right_foot"},
        ]:
            # Fail safely on the dimension regression before update_contacts
            # resizes the native solver on an unpatched installation.
            for frame in list(controller.contacts):
                if frame.removesuffix("_contact") not in active:
                    assert controller.form.removeRigidContact(frame, 0.0)
                    assert controller.form.nEq == 6
                    del controller.contacts[frame]
                    controller._resize_solver()
            controller.update_contacts(active, q)
            tau = controller.compute(q, v, q[7:], cycle * .1)
            assert controller.form.nEq == 6
            assert np.isfinite(tau).all()
            assert all(abs(tau[i]) <= limits[name].max_torque + 1e-6
                       for i, name in enumerate(controller.joint_names))


def test_policy_torque_reference_is_bounded_and_tracks_nominal():
    controller, model, data, limits = make_models()
    q, v = state(controller, model, data)
    bound = np.array([limits[n].max_torque for n in controller.joint_names])
    controller.set_torque_limits(bound)
    controller.enable_policy_torque_tracking()
    controller.update_contacts({'left_foot', 'right_foot'}, q)
    nominal = np.linspace(-2., 2., controller.robot.na)
    tau = controller.compute(q, v, q[7:], torque_ref=nominal)
    assert np.max(np.abs(tau - nominal)) < 1.0
    tau = controller.compute(q, v, q[7:], torque_ref=100 * bound)
    assert np.all(np.abs(tau) <= bound + 1e-5)
    with pytest.raises(ValueError, match='Nominal torque'):
        controller.compute(q, v, q[7:], torque_ref=np.full(10, np.nan))


def test_policy_torque_correction_bound_includes_damping_compensation():
    controller, model, data, limits = make_models()
    q, v = state(controller, model, data)
    v[6:] = np.linspace(-.2, .2, controller.robot.na)
    controller.set_torque_limits([limits[n].max_torque for n in controller.joint_names])
    controller.enable_policy_torque_tracking(max_correction=.25)
    controller.update_contacts({'left_foot', 'right_foot'}, q)
    nominal = np.linspace(-2., 2., controller.robot.na)
    tau = controller.compute(q, v, q[7:], torque_ref=nominal)
    assert np.max(np.abs(tau - nominal)) <= .25 + 1e-5
