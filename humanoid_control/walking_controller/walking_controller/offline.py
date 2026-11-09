"""Offline closed loop without ROS, for development and tests.

Uses humanoid_sim's model builder, its gantry weld and the same MIT actuator
law as humanoid_sim.world.SimWorld.step, with a configurable command delay
standing in for the ROS/C++ path.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from .robot import FEET, RobotModel, load_limits


@dataclass
class Command:
    """tau = kp * (q_ref - q) + kd * (dq_ref - dq) + tau_ff, per joint."""
    q_ref: np.ndarray
    dq_ref: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    tau_ff: np.ndarray


def default_paths():
    from ament_index_python.packages import get_package_share_directory as share
    hybrid, leg = Path(share('motor_control_hybrid')), Path(share('humanoid_leg_description'))
    return {
        'robot_xml': hybrid / 'models/20260914_170827/chrobot_16kg_candidate.xml',
        'mesh_dir': leg / 'meshes/robot_meshes',
        'registry': hybrid / 'config/motors.yaml',
        'urdf': leg / 'urdf/human_offset_corrected.urdf',
    }


def load(paths=None):
    paths = paths or default_paths()
    robot = RobotModel(paths['urdf'])
    return paths, robot, load_limits(paths['registry'], robot.joint_names)


class OfflineSim:
    def __init__(self, robot: RobotModel, limits, paths, delay_steps=2, friction=None):
        from humanoid_sim import model_builder as mb
        options = mb.WorldOptions(
            robot_xml=str(paths['robot_xml']), mesh_dir=str(paths['mesh_dir']),
            joint_torque_limits=dict(zip(robot.joint_names, limits.torque)), timestep=0.005,
            foot_clearance_m=0.02, add_head=False, head_mass_kg=0.3, head_range_rad=1.0,
            head_servo_torque_nm=1.0, camera_fovy_deg=42.5, camera_resolution=(64, 48),
            add_target=False, target_distance_m=2.0, target_image='', scene_builder='')
        self.model = m = mb.build_model(options)
        self.data = mujoco.MjData(m)
        self.robot = robot
        self.qpos = [m.jnt_qposadr[m.joint(n).id] for n in robot.joint_names]
        self.qvel = [m.jnt_dofadr[m.joint(n).id] for n in robot.joint_names]
        self.act = [m.actuator(mb.actuator_name(n)).id for n in robot.joint_names]
        self.weld = m.eq('base_hold').id
        self.base_body = m.body('floating_base').id
        self.floor = m.geom('floor').id
        self.foot_geoms = [m.geom(f'{s}_foot_collision_box').id for s in ('left', 'right')]
        self.foot_bodies = [m.body(f).id for f in FEET]
        if friction is not None:
            m.geom('floor').friction[0] = friction
        self.dt = float(m.opt.timestep)
        self.delay_steps = int(delay_steps)
        self.queue, self.command = [], None
        mujoco.mj_resetData(m, self.data)
        self.data.eq_active[self.weld] = 1
        mujoco.mj_forward(m, self.data)

    @property
    def time(self):
        return float(self.data.time)

    @property
    def base_held(self):
        return bool(self.data.eq_active[self.weld])

    def release(self):
        self.data.eq_active[self.weld] = 0

    def state(self):
        d = self.data
        ang = np.zeros(3)
        mujoco.mju_rotVecQuat(ang, d.qvel[3:6], d.qpos[3:7])
        return self.robot.state_from_sim(d.qpos[:3], np.roll(d.qpos[3:7], -1), d.qvel[:3], ang,
                                         d.qpos[self.qpos], d.qvel[self.qvel])

    def foot_forces(self):
        forces, wrench = np.zeros(2), np.zeros(6)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            pair = {c.geom1, c.geom2}
            if self.floor in pair:
                for k, g in enumerate(self.foot_geoms):
                    if g in pair:
                        mujoco.mj_contactForce(self.model, self.data, i, wrench)
                        forces[k] += max(0.0, wrench[0])
        return forces

    def foot_side_tilt_deg(self):
        """Sideways tilt of each foot about its toe axis (degrees)."""
        out = []
        for b in self.foot_bodies:
            R = self.data.xmat[b].reshape(3, 3)
            out.append(float(np.degrees(np.arcsin(np.clip(R[2, 0], -1, 1)))))
        return out

    def fallen(self):
        """Any non-foot geom touching the floor."""
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            pair = {c.geom1, c.geom2}
            if self.floor in pair and not (pair & set(self.foot_geoms)):
                return True
        return False

    def step(self, command: Command | None, push=None):
        self.queue.append(command)
        if len(self.queue) > self.delay_steps:
            self.command = self.queue.pop(0)
        m, d, c = self.model, self.data, self.command
        for i, act in enumerate(self.act):
            if c is None:
                d.ctrl[act] = 0.0
                m.actuator_biasprm[act, 1:3] = 0.0
            else:
                d.ctrl[act] = c.kp[i] * c.q_ref[i] + c.kd[i] * c.dq_ref[i] + c.tau_ff[i]
                m.actuator_biasprm[act, 1] = -c.kp[i]
                m.actuator_biasprm[act, 2] = -c.kd[i]
        d.xfrc_applied[self.base_body, :3] = 0.0 if push is None else push
        mujoco.mj_step(m, d)


def run(controller, duration_s, release_s=1.0, delay_steps=2, paths=None, robot=None, limits=None,
        push=None, on_step=None, friction=None):
    """Drive controller.step(t, q, v, forces, held) -> Command | None.

    push: optional callable t -> force (3,) on the pelvis. on_step: callable
    (t, sim, controller) called after each physics step; return True to stop.
    """
    if robot is None:
        paths, robot, limits = load(paths)
    sim = OfflineSim(robot, limits, paths, delay_steps, friction)
    result = {'fault': '', 'fallen': False, 't_end': 0.0}
    for k in range(int(duration_s / sim.dt)):
        t = k * sim.dt
        if sim.base_held and t >= release_s:
            sim.release()
        q, v = sim.state()
        command = controller.step(t, q, v, sim.foot_forces(), sim.base_held)
        if command is None:
            result['fault'] = getattr(controller, 'fault', 'no command')
            break
        sim.step(command, push(t) if push else None)
        result['t_end'] = t
        if sim.fallen():
            result['fallen'] = True
            break
        if on_step is not None and on_step(t, sim, controller):
            break
    result['base_z'] = float(sim.data.qpos[2])
    result['sim'] = sim
    return result
