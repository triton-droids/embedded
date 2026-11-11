"""Leg odometry: stance-foot anchoring (stubbed kinematics) and an offline walk vs MuJoCo ground truth."""
import math

import numpy as np
import pytest

from conftest import GAINS
from walking_controller.leg_odometry import (LegOdometry, body_from_base, footprint, inverse,
                                             matrix_to_quat_xyzw, quat_xyzw_to_matrix, rigid)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


class StubOdometry(LegOdometry):
    """Feet given directly (world axes, relative to the base): no Pinocchio needed."""

    def __init__(self, **kw):
        self.robot = None
        self.body_from_base = body_from_base('positive_y')
        self.imu_to_body = np.eye(3)
        self.margin = 0.01
        self.reset()

    def feet(self, joint_positions, world_from_body):
        return joint_positions  # the test passes the feet dict here


def test_quaternion_round_trip():
    R = rot_z(2.0) @ np.array([[1, 0, 0], [0, math.cos(.3), -math.sin(.3)], [0, math.sin(.3), math.cos(.3)]])
    assert quat_xyzw_to_matrix(matrix_to_quat_xyzw(R)) == pytest.approx(R)
    with pytest.raises(ValueError):
        quat_xyzw_to_matrix([0, 0, 0, 0])


def test_stance_anchor_moves_the_base_by_the_step_length():
    """Body forward is +y (positive_y). Each step the swing foot lands 0.1 m ahead of the stance foot."""
    odo = StubOdometry()
    up = (0, 0, 0, 1)
    h = -0.6
    # Start: both feet down, left slightly lower -> left is stance.
    est = odo.update({'left_foot': np.array([-0.1, 0.0, h]), 'right_foot': np.array([0.1, 0.0, h + 0.005])}, up)
    assert est.stance == 'left_foot' and est.odom_from_base[:3, 3] == pytest.approx([0, 0, 0.6])
    # Base moves 0.05 m forward over the stance foot; right swings to 0.1 m ahead and lands lower.
    est = odo.update({'left_foot': np.array([-0.1, -0.05, h]), 'right_foot': np.array([0.1, 0.05, h + 0.04])}, up)
    assert est.stance == 'left_foot'
    assert est.odom_from_base[:2, 3] == pytest.approx([0.05, 0.0])  # odom x = robot forward
    est = odo.update({'left_foot': np.array([-0.1, -0.05, h + 0.02]), 'right_foot': np.array([0.1, 0.05, h])}, up)
    assert est.stance == 'right_foot' and est.switches == 1
    # Base moves on over the right foot: total progress 0.1 m.
    est = odo.update({'left_foot': np.array([-0.1, -0.1, h + 0.04]), 'right_foot': np.array([0.1, 0.0, h])}, up)
    assert est.odom_from_base[:2, 3] == pytest.approx([0.1, 0.0])
    assert est.odom_from_footprint[2, 3] == 0.0 and est.footprint_from_base[2, 3] == pytest.approx(0.6)


def test_hysteresis_keeps_the_stance_in_double_support():
    odo = StubOdometry()
    up = (0, 0, 0, 1)
    odo.update({'left_foot': np.array([-0.1, 0, -0.6]), 'right_foot': np.array([0.1, 0, -0.598])}, up)
    for dz in (0.004, -0.004, 0.009, -0.009):  # within the 1 cm margin
        est = odo.update({'left_foot': np.array([-0.1, 0, -0.6]), 'right_foot': np.array([0.1, 0, -0.6 + dz])}, up)
        assert est.stance == 'left_foot' and est.switches == 0


def test_odom_starts_at_the_first_footprint_and_follows_imu_yaw():
    odo = StubOdometry()
    yaw0 = 0.7
    q0 = matrix_to_quat_xyzw(rot_z(yaw0))
    feet = {'left_foot': np.array([0.0, 0.0, -0.6]), 'right_foot': np.array([0.2, 0.0, -0.59])}
    est = odo.update(feet, q0)
    assert est.odom_from_footprint == pytest.approx(np.eye(4))  # IMU yaw at start is odom's zero
    est = odo.update(feet, matrix_to_quat_xyzw(rot_z(yaw0 + 0.3)))
    assert math.atan2(est.odom_from_footprint[1, 0], est.odom_from_footprint[0, 0]) == pytest.approx(0.3)


def test_offline_walk_tracks_ground_truth(model):
    """Closed-loop offline arc walk (MuJoCo, TSID): leg odometry fed with the simulated joint angles
    and the exact base attitude (an ideal IMU) stays close to the true base pose."""
    from walking_controller.offline import run
    from walking_controller.stepping import SS, StepParams, SteppingController
    from walking_controller.wbc import WBCGains
    paths, robot, limits = model
    ctl = SteppingController(robot, limits.torque, StepParams(gains=WBCGains(**GAINS)))
    odo = LegOdometry(robot, 'positive_y')
    B = body_from_base('positive_y')
    rec = {'est': [], 'true': []}
    state = {'world_from_odom': None, 'next': 4.0}

    def on_step(t, sim, c):
        if abs(t - 5.0) < sim.dt / 2:
            c.p.turn_rad, c.p.step_length_m = 0.08, 0.06
        if t + 1e-9 < state['next']:
            return False
        state['next'] += 0.02
        d = sim.data
        quat = np.roll(d.qpos[3:7], -1)
        est = odo.update(d.qpos[sim.qpos], quat, c.stance if c.gphase == SS else odo.stance)
        true = rigid(quat_xyzw_to_matrix(quat) @ B, d.qpos[:3].copy())
        if state['world_from_odom'] is None:
            state['world_from_odom'] = footprint(true)
        rec['est'].append(est.odom_from_base.copy())
        rec['true'].append(inverse(state['world_from_odom']) @ true)
        return False

    res = run(ctl, 20.0, paths=paths, robot=robot, limits=limits, on_step=on_step)
    assert not res['fallen'] and not res['fault']
    est, true = np.array(rec['est']), np.array(rec['true'])
    walked = float(np.sum(np.linalg.norm(np.diff(true[:, :2, 3], axis=0), axis=1)))
    err = np.linalg.norm(est[:, :2, 3] - true[:, :2, 3], axis=1)
    print(f'walked {walked:.2f} m, {odo.switches} stance switches, final error {err[-1]:.3f} m, '
          f'max {err.max():.3f} m')
    assert walked > 1.0
    assert odo.switches > 20
    # Dead reckoning drifts with distance; a few cm per metre is what the ball tracker can live with.
    assert err[-1] < 0.05 * walked + 0.03, (err[-1], walked)
    yaw_err = [math.remainder(math.atan2(e[1, 0], e[0, 0]) - math.atan2(g[1, 0], g[0, 0]), 2 * math.pi)
               for e, g in zip(est, true)]
    assert max(abs(y) for y in yaw_err) < 1e-6  # yaw comes straight from the (ideal) IMU


def test_given_stance_overrides_foot_heights():
    """The walking controller's stance foot wins even when the swing foot is barely higher."""
    odo = StubOdometry()
    up = (0, 0, 0, 1)
    feet = {'left_foot': np.array([-0.1, 0.0, -0.6]), 'right_foot': np.array([0.1, 0.1, -0.598])}
    assert odo.update(feet, up, 'left_foot').stance == 'left_foot'
    est = odo.update(feet, up, 'right_foot')  # within the height margin, but the controller says so
    assert est.stance == 'right_foot' and est.switches == 1
    with pytest.raises(ValueError):
        odo.update(feet, up, 'hand')
