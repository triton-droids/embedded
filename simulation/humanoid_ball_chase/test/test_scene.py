"""Compile real robot + CAD and check physical ball and optical extrinsics."""
from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml
from humanoid_sim import model_builder as mb
from humanoid_sim.motor_model import limits_from_registry

REPO = Path(__file__).resolve().parents[3]


def _build(**extra):
    hybrid = REPO / 'humanoid_control/motor_control_hybrid'
    limits = limits_from_registry(yaml.safe_load((hybrid/'config/motors.yaml').read_text())['motor_control_node']['ros__parameters'])
    return mb.build_model(mb.WorldOptions(
        robot_xml=str(hybrid/'models/20260914_170827/chrobot_16kg_candidate.xml'),
        mesh_dir=str(REPO/'humanoid_leg_description/meshes/robot_meshes'),
        joint_torque_limits={name: lim.max_torque for name,lim in limits.items()},
        camera_resolution=(424,240), camera_fovy_deg=58,
        scene_builder='humanoid_ball_chase.scene:build_scene', **extra))


@pytest.fixture
def model():
    return _build()


def test_scene_has_cad_and_physical_ball(model):
    assert model.geom('d435_REF_D435_envelope').contype == 0
    assert model.body('chase_ball').mass[0] == pytest.approx(0.2)
    assert model.joint('chase_ball_free').type[0] == mujoco.mjtJoint.mjJNT_FREE
    assert mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'face_target') == -1
    data = mujoco.MjData(model)
    for _ in range(200):
        mujoco.mj_step(model,data)
    assert data.body('chase_ball').xpos[2] == pytest.approx(0.10,abs=0.002)
    assert any(model.geom('chase_ball').id in (c.geom1,c.geom2) for c in data.contact)


def test_rev_l_imu_follows_camera_carrier(model):
    # RevL adds the board, insulating pad and header to the pitch group.
    pitch_body = model.body('d435_pitch').id
    for name in ('REF_IMU_PCB_24x20_ASSUMED', 'REF_IMU_insulating_pad',
                 'REF_IMU_header_ASSUMED'):
        geom = model.geom('d435_' + name)
        assert geom.bodyid[0] == pitch_body
        assert geom.contype[0] == 0 and geom.conaffinity[0] == 0
    imu = model.site('head_imu')
    assert imu.pos == pytest.approx([0, 0, 0.0228])
    data = mujoco.MjData(model)
    for yaw, pitch in ((0, 0), (0.5, 0.3), (-0.5, -0.3)):
        data.qpos[model.joint(mb.HEAD_JOINT).qposadr[0]] = yaw
        data.qpos[model.joint('head_pitch_joint').qposadr[0]] = pitch
        mujoco.mj_forward(model, data)
        body = data.body('d435_pitch')
        expected = body.xpos + body.xmat.reshape(3, 3) @ np.array([0, 0, 0.0228])
        assert data.site('head_imu').xpos == pytest.approx(expected)


def test_optical_extrinsics_follow_actual_yaw(model):
    from humanoid_sim.camera_node import SimCameraNode
    from builtin_interfaces.msg import Time
    from types import SimpleNamespace
    data = mujoco.MjData(model)
    node = SimpleNamespace(world=SimpleNamespace(model=model,data=data),_frame_id='camera_color_optical_frame',
                           _body_from_base=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]]))
    j = model.joint(mb.HEAD_JOINT).qposadr[0]
    for yaw in (0.0,0.5,-0.5):
        data.qpos[j]=yaw
        mujoco.mj_forward(model,data)
        transform=SimCameraNode._camera_transform(node,Time())
        r=transform.transform.rotation
        rotation=np.zeros(9)
        mujoco.mju_quat2Mat(rotation,np.array([r.w,r.x,r.y,r.z]))
        forward=rotation.reshape(3,3)[:,2]
        assert np.arctan2(forward[1],forward[0]) == pytest.approx(yaw)
        assert forward[2] == pytest.approx(-0.5)
        assert transform.header.frame_id=='base_link'
        assert transform.child_frame_id=='camera_color_optical_frame'
    pitch=model.joint('head_pitch_joint').qposadr[0]
    for angle in (-.3,.3):
        data.qpos[pitch]=angle
        mujoco.mj_forward(model,data)
        transform=SimCameraNode._camera_transform(node,Time())
        r=transform.transform.rotation
        mujoco.mju_quat2Mat(rotation,np.array([r.w,r.x,r.y,r.z]))
        assert rotation.reshape(3,3)[2,2] == pytest.approx(-np.sin(np.pi/6+angle))


def test_pitch_servo_moves_and_release_removes_drive(model):
    from humanoid_sim.world import SimWorld,HeadServoConfig
    from humanoid_sim.motor_model import MotorBank
    hybrid=REPO/'humanoid_control/motor_control_hybrid'
    limits=limits_from_registry(yaml.safe_load((hybrid/'config/motors.yaml').read_text())['motor_control_node']['ros__parameters'])
    world=SimWorld(model,MotorBank(limits),head=HeadServoConfig(),settle_s=.2)
    joint=model.joint('head_pitch_joint').qposadr[0]
    world.set_pitch_target(.2)
    for i in range(150):world.step(i*.005)
    assert world.data.qpos[joint] == pytest.approx(.2,abs=.015)
    world.set_pitch_target(-.2)
    for i in range(200):world.step(1+i*.005)
    assert world.data.qpos[joint] == pytest.approx(-.2,abs=.015)
    world.set_pitch_target(None);world.step(3)
    act=model.actuator('head_pitch_servo').id
    assert world.data.ctrl[act]==0
    assert np.all(model.actuator_biasprm[act,1:3]==0)
    world.reset()
    assert world._pitch_target==0


def test_yolo_recognizes_rendered_football(model):
    from humanoid_ball_chase.football_detector import FootballDetector
    from humanoid_sim.camera_node import pinhole_info
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    with mujoco.Renderer(model,240,424) as renderer:
        renderer.update_scene(data,camera='head_camera');rgb=renderer.render()
        renderer.enable_depth_rendering();depth=renderer.render()
    detector=FootballDetector()
    result=detector.detect(rgb,depth,pinhole_info(424,240,58,'camera').k)
    assert result is not None
    assert detector.last_confidence>.3
    assert result[4]==pytest.approx(1.83,abs=.15)


def test_camera_faces_toes_and_ball_is_in_front(model):
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    # The saved foot shape extends 130 mm toward +Y from its ankle,
    # versus 80 mm toward -Y. This independent geometry check fixes front/back.
    ankle=data.xanchor[model.joint('left_ankle_joint').id]
    foot=model.geom('left_foot_collision_box').id
    toe_y=data.geom_xpos[foot,1]+model.geom_size[foot,1]
    heel_y=data.geom_xpos[foot,1]-model.geom_size[foot,1]
    assert toe_y-ankle[1] > ankle[1]-heel_y
    forward=-data.cam_xmat[model.camera('head_camera').id].reshape(3,3)[:,2]
    assert forward[1]>0
    lenses=[data.geom_xpos[model.geom('d435_VIS_lens_'+name).id]
            for name in ('32.5','11.5','-17.5','-32.5')]
    housing=data.geom_xpos[model.geom('d435_REF_D435_envelope').id]
    assert np.dot(np.mean(lenses,axis=0)-housing,forward)>0
    camera=data.cam_xpos[model.camera('head_camera').id]
    assert np.dot(data.body('chase_ball').xpos-camera,forward)>0


def test_hard_scene_adds_only_visual_clutter():
    hybrid = REPO / 'humanoid_control/motor_control_hybrid'
    limits = limits_from_registry(yaml.safe_load((hybrid/'config/motors.yaml').read_text())['motor_control_node']['ros__parameters'])
    def build(builder):
        return mb.build_model(mb.WorldOptions(
            robot_xml=str(hybrid/'models/20260914_170827/chrobot_16kg_candidate.xml'),
            mesh_dir=str(REPO/'humanoid_leg_description/meshes/robot_meshes'),
            joint_torque_limits={name: lim.max_torque for name,lim in limits.items()},
            camera_resolution=(424,240), camera_fovy_deg=58,
            scene_builder='humanoid_ball_chase.scene:' + builder))
    clean, hard = build('build_scene'), build('build_scene_hard')
    assert hard.ngeom > clean.ngeom + 100
    old = {clean.geom(i).name for i in range(clean.ngeom)}
    added = [hard.geom(i) for i in range(hard.ngeom) if hard.geom(i).name not in old]
    assert len(added) == hard.ngeom - clean.ngeom
    assert all(g.contype[0] == 0 and g.conaffinity[0] == 0 for g in added)
    assert hard.nbody == clean.nbody and hard.nv == clean.nv and hard.nu == clean.nu
    assert np.allclose(hard.body_mass, clean.body_mass)
    assert hard.light('low_sun').castshadow


def test_stereo_rig_to_head_imu_extrinsic_matches_kinematics():
    """The static TF cuVSLAM fuses with: position and gyro axes at any head pose."""
    from humanoid_sim.stereo_camera_node import (CAMERA_FROM_OPTICAL, RIG_FROM_OPTICAL,
                                                 rig_from_head_imu)
    model = _build(stereo_baseline_m=0.05)
    translation, rotation = rig_from_head_imu(model)
    # Board centre: 25 mm right of the left camera, 13 mm behind its optical centre
    # (rig x forward, y left, z up), 22.8 mm above it.
    assert translation == pytest.approx([-0.013, -0.025, 0.0228], abs=1e-6)
    data = mujoco.MjData(model)
    left = model.camera(mb.STEREO_CAMERAS[0]).id
    yaw_dof = model.joint(mb.HEAD_JOINT).dofadr[0]
    pitch_dof = model.joint('head_pitch_joint').dofadr[0]
    for yaw, pitch in ((0, 0), (0.5, 0.3), (-0.5, -0.3)):
        data.qpos[model.joint(mb.HEAD_JOINT).qposadr[0]] = yaw
        data.qpos[model.joint('head_pitch_joint').qposadr[0]] = pitch
        data.qvel[:] = 0
        data.qvel[yaw_dof], data.qvel[pitch_dof] = 0.7, -0.4
        mujoco.mj_forward(model, data)
        world_from_rig = data.cam_xmat[left].reshape(3, 3) @ CAMERA_FROM_OPTICAL @ RIG_FROM_OPTICAL.T
        site = data.site('head_imu')
        assert site.xpos == pytest.approx(data.cam_xpos[left] + world_from_rig @ translation, abs=1e-9)
        assert site.xmat.reshape(3, 3) == pytest.approx(world_from_rig @ rotation, abs=1e-9)
        # The gyro (head_imu_link axes) rotated into the rig is the rig's own rate.
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_CAMERA, left, velocity, 1)
        rig_rate = (CAMERA_FROM_OPTICAL @ RIG_FROM_OPTICAL.T).T @ velocity[:3]
        assert rotation @ data.sensor(mb.HEAD_GYRO).data == pytest.approx(rig_rate, abs=1e-9)
