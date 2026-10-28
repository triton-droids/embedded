"""Compose the MuJoCo simulation world around the deployed robot model.

The robot MJCF (``chrobot_16kg_candidate.xml`` saved beside the tracking model)
is loaded unchanged; everything the simulator needs is added with MjSpec so the
robot description is not duplicated:

* a floor and light,
* one MIT-law actuator per leg joint (see ``motor_model.py``),
* body IMU sensors on the existing ``imu`` site,
* a head with one yaw servo, head IMU and a forward camera on the ``top`` site,
* a weld that can hold the floating base in place (a virtual gantry),
* an optional mocap "face" target for the head-tracking detector.

The head, camera mount and target are simulation fixtures. Their geometry and
mass are placeholders, not measurements of the robot's head assembly.

Robot frame convention (from the MJCF): +Z up, robot forward is -Y, robot left
is +X. The ``imu`` site is identity-aligned with ``floating_base``, matching the
policy's ``imu_to_body_rotation`` identity default.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import mujoco
import numpy as np

# Names shared with world.py and the ROS nodes.
BASE_BODY = 'floating_base'
BASE_WELD = 'base_hold'
IMU_GYRO = 'imu_gyro'
IMU_ACCEL = 'imu_accel'
IMU_QUAT = 'imu_quat'
HEAD_JOINT = 'head_yaw_joint'
HEAD_ACTUATOR = 'head_yaw_servo'
HEAD_GYRO = 'head_imu_gyro'
HEAD_ACCEL = 'head_imu_accel'
HEAD_CAMERA = 'head_camera'
TARGET_BODY = 'face_target'
FOOT_SITES = ('left_foot', 'right_foot')  # Sole centers in the robot MJCF.


def actuator_name(joint: str) -> str:
    return f'{joint}_motor'


@dataclass
class WorldOptions:
    """Simulation fixtures; defaults are placeholders, not measured values."""

    robot_xml: str
    mesh_dir: str
    joint_torque_limits: dict[str, float]
    timestep: float = 0.0  # 0 keeps the robot MJCF's timestep.
    # Gantry height. At ~0 the limp legs fold against the floor and keep
    # shaking the base; 5 cm lets them hang still.
    foot_clearance_m: float = 0.05
    add_head: bool = True
    head_mass_kg: float = 0.05
    head_range_rad: float = 1.0
    head_servo_torque_nm: float = 2.5
    camera_fovy_deg: float = 42.5
    camera_resolution: tuple[int, int] = (640, 480)
    add_target: bool = True
    target_distance_m: float = 1.5
    target_image: str = ''  # PNG only: MuJoCo decodes no other image format.


def build_model(options: WorldOptions) -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(options.robot_xml)
    spec.meshdir = options.mesh_dir
    if options.timestep > 0.0:
        spec.option.timestep = options.timestep
    spec.visual.global_.offwidth = max(options.camera_resolution[0], 1280)
    spec.visual.global_.offheight = max(options.camera_resolution[1], 720)

    _add_environment(spec)
    _add_leg_actuators(spec, options.joint_torque_limits)
    _add_body_imu(spec)
    _add_base_hold(spec)
    if options.add_head:
        _add_head(spec, options)
    if options.add_target:
        _add_target(spec, options)
    _lift_base_to_clearance(spec, options.foot_clearance_m)
    return spec.compile()


def _lift_base_to_clearance(spec: mujoco.MjSpec, clearance: float) -> None:
    """Move the spawn pose so the lowest foot sole sits `clearance` above the floor.

    The saved MJCF spawns the soles about 3 cm below z=0. The base weld holds
    the spawn pose, so it must be corrected in the model (qpos0), not in data.
    """
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    lowest = min(data.site(name).xpos[2] for name in FOOT_SITES)
    base = spec.body(BASE_BODY)
    base.pos = [base.pos[0], base.pos[1], base.pos[2] + clearance - lowest]
    target = spec.body(TARGET_BODY)
    if target is not None:
        target.pos = [target.pos[0], target.pos[1], target.pos[2] + clearance - lowest]


def _add_environment(spec: mujoco.MjSpec) -> None:
    spec.add_texture(name='floor_checker', type=mujoco.mjtTexture.mjTEXTURE_2D,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                     rgb1=[0.25, 0.28, 0.32], rgb2=[0.35, 0.38, 0.42], width=512, height=512)
    floor = spec.add_material(name='floor', texrepeat=[8, 8], reflectance=0.1)
    floor.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'floor_checker'
    spec.add_texture(name='sky', type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=[0.6, 0.75, 0.9], rgb2=[0.1, 0.1, 0.15], width=256, height=256)
    world = spec.worldbody
    world.add_geom(name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05],
                   material='floor', contype=1, conaffinity=1)
    world.add_light(name='sun', pos=[0, 0, 4], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
                    diffuse=[0.7, 0.7, 0.7], specular=[0.2, 0.2, 0.2])


def _add_leg_actuators(spec: mujoco.MjSpec, torque_limits: dict[str, float]) -> None:
    """Affine actuators: force = ctrl + bias1*q + bias2*qdot, clamped to forcerange.

    world.py writes ctrl = kp*q_ref + kd*dq_ref + tau_ff, bias1 = -kp and
    bias2 = -kd every step, which is the MIT law. Keeping the gains inside the
    actuator lets the implicitfast integrator treat the damping term implicitly,
    like the position actuators used in training.
    """
    for joint, torque in torque_limits.items():
        if spec.joint(joint) is None:
            raise ValueError(f'Registry joint {joint} is not in the robot model')
        actuator = spec.add_actuator(
            name=actuator_name(joint), target=joint, trntype=mujoco.mjtTrn.mjTRN_JOINT,
            gaintype=mujoco.mjtGain.mjGAIN_FIXED, biastype=mujoco.mjtBias.mjBIAS_AFFINE,
            forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE, forcerange=[-torque, torque])
        actuator.gainprm[0] = 1.0


def _add_body_imu(spec: mujoco.MjSpec) -> None:
    site = mujoco.mjtObj.mjOBJ_SITE
    spec.add_sensor(name=IMU_GYRO, type=mujoco.mjtSensor.mjSENS_GYRO, objtype=site, objname='imu')
    spec.add_sensor(name=IMU_ACCEL, type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                    objtype=site, objname='imu')
    spec.add_sensor(name=IMU_QUAT, type=mujoco.mjtSensor.mjSENS_FRAMEQUAT,
                    objtype=site, objname='imu')


def _add_base_hold(spec: mujoco.MjSpec) -> None:
    # Weld to the world at the initial pose; world.py toggles it at runtime.
    # The default weld is soft enough to tilt the base a few degrees under the
    # offset centre of mass; use the stiffest time constant MuJoCo allows (2*dt)
    # and near-rigid impedance so the held IMU reads upright.
    timestep = spec.option.timestep
    spec.add_equality(name=BASE_WELD, type=mujoco.mjtEq.mjEQ_WELD,
                      objtype=mujoco.mjtObj.mjOBJ_BODY, name1=BASE_BODY,
                      solref=[2.0 * timestep, 1.0], solimp=[0.99, 0.999, 0.001, 0.5, 2.0])


def _add_head(spec: mujoco.MjSpec, options: WorldOptions) -> None:
    top = spec.site('top')
    torso = spec.body('torso')
    head = torso.add_body(name='head', pos=top.pos)
    limit = options.head_range_rad
    # Positive yaw turns the camera toward robot left (counter-clockwise from above).
    head.add_joint(name=HEAD_JOINT, type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1],
                   range=[-limit, limit], damping=0.05, armature=0.001)
    head.add_geom(name='head_neck', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.012, 0.03, 0],
                  pos=[0, 0, -0.025], rgba=[0.2, 0.2, 0.22, 1], group=2,
                  contype=0, conaffinity=0, mass=0.0)
    head.add_geom(name='head_shell', type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.05, 0.04, 0.035],
                  pos=[0, 0, 0.04], rgba=[0.2, 0.2, 0.22, 1], group=2,
                  contype=0, conaffinity=0, mass=options.head_mass_kg)
    head.add_geom(name='head_lens', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.012, 0.005, 0],
                  pos=[0, -0.045, 0.04], quat=_quat_from_axes([1, 0, 0], [0, 0, -1]),
                  rgba=[0.05, 0.05, 0.05, 1], group=2, contype=0, conaffinity=0, mass=0.0)
    head.add_site(name='head_imu', pos=[0, 0, 0.04])
    width, height = options.camera_resolution
    # MuJoCo cameras look along their -Z with +Y up: x=-X_body, y=+Z_body gives -Y forward.
    head.add_camera(name=HEAD_CAMERA, pos=[0, -0.05, 0.04], fovy=options.camera_fovy_deg,
                    quat=_quat_from_axes([-1, 0, 0], [0, 0, 1]), resolution=[width, height])

    spec.add_actuator(
        name=HEAD_ACTUATOR, target=HEAD_JOINT, trntype=mujoco.mjtTrn.mjTRN_JOINT,
        gaintype=mujoco.mjtGain.mjGAIN_FIXED, biastype=mujoco.mjtBias.mjBIAS_AFFINE,
        forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE,
        forcerange=[-options.head_servo_torque_nm, options.head_servo_torque_nm],
    ).gainprm[0] = 1.0
    site = mujoco.mjtObj.mjOBJ_SITE
    spec.add_sensor(name=HEAD_GYRO, type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=site, objname='head_imu')
    spec.add_sensor(name=HEAD_ACCEL, type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                    objtype=site, objname='head_imu')


def _add_target(spec: mujoco.MjSpec, options: WorldOptions) -> None:
    """A mocap board in front of the robot, optionally textured with a face image."""
    top = spec.site('top').pos
    torso_pos = spec.body('torso').pos
    base_pos = spec.body(BASE_BODY).pos
    height = float(base_pos[2] + torso_pos[2] + top[2])
    center_x = float(base_pos[0] + torso_pos[0] + top[0])
    center_y = float(base_pos[1] + torso_pos[1] + top[1])
    body = spec.worldbody.add_body(name=TARGET_BODY, mocap=True,
                                   pos=[center_x, center_y - options.target_distance_m, height])
    material = 'target_plain'
    if options.target_image:
        if not options.target_image.lower().endswith('.png'):
            raise ValueError(f'target_image must be a PNG file: {options.target_image}')
        spec.add_texture(name='target_image', type=mujoco.mjtTexture.mjTEXTURE_2D,
                         file=options.target_image)
        textured = spec.add_material(name='target_textured')
        textured.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'target_image'
        material = 'target_textured'
    else:
        spec.add_material(name='target_plain', rgba=[0.9, 0.6, 0.2, 1])
    # Thin board facing +Y (toward the robot); the image spans its X-Z face.
    body.add_geom(name='target_board', type=mujoco.mjtGeom.mjGEOM_BOX,
                  size=[0.12, 0.002, 0.12], material=material, contype=0, conaffinity=0)


def _quat_from_axes(x_axis, y_axis) -> list[float]:
    """Quaternion (w, x, y, z) of a frame given its X and Y axes in the parent."""
    x = np.asarray(x_axis, dtype=float)
    y = np.asarray(y_axis, dtype=float)
    z = np.cross(x, y)
    rotation = np.column_stack([x, y, z]).reshape(-1)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, rotation)
    if not math.isclose(np.linalg.norm(quat), 1.0, rel_tol=1e-6):
        raise ValueError('Axes do not form a rotation')
    return quat.tolist()
