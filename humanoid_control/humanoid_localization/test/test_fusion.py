import math

import numpy as np

from humanoid_localization.fusion import (FusionConfig, VioFusion, invert, matrix_to_quat,
                                          quat_to_matrix, rot_z, transform)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])


def simulate(fusion, n=200, dt=1/15, odom_offset=None, imu_yaw_bias=0.4):
    """Base walks an arc while the camera head pans; VO sees the rig in an arbitrary frame."""
    odom_offset = odom_offset if odom_offset is not None else transform(rot_z(1.0) @ rot_x(0.3), (5., -2., 1.))
    truth = []
    for k in range(n):
        t = k * dt
        yaw = 0.3 * t
        world_base = transform(rot_z(yaw), (0.2 * math.sin(yaw) / 0.3, 0.2 * (1 - math.cos(yaw)) / 0.3, 0.6))
        head = 0.5 * math.sin(2 * t)  # Pan, so base->rig changes every frame.
        base_rig = transform(rot_z(head) @ rot_x(-0.5), (0.05, 0.0, 0.3))
        world_rig = world_base @ base_rig
        if k == 0:
            odom_world = odom_offset @ invert(world_rig)  # VO frame starts at an arbitrary pose.
        # Game rotation vector: same attitude, yaw relative to power-on (bias).
        imu = rot_z(imu_yaw_bias) @ world_base[:3, :3]
        fusion.add_imu(t, matrix_to_quat(imu))
        fusion.add_vo(t, odom_world @ world_rig, base_rig)
        truth.append(world_base)
    return truth


def test_quaternion_round_trip():
    r = rot_z(2.5) @ rot_x(-1.1)
    assert np.allclose(quat_to_matrix(matrix_to_quat(r)), r)


def test_recovers_pose_with_moving_head_and_imu_yaw_offset():
    fusion = VioFusion(FusionConfig(initial_position=(0.0, 0.0, 0.6), initial_yaw=0.0))
    truth = simulate(fusion)
    assert fusion.accepted == len(truth) - 1
    assert np.allclose(fusion.position, truth[-1][:3, 3], atol=1e-6)
    assert np.allclose(fusion.orientation(), truth[-1][:3, :3], atol=1e-6)


def test_rejects_jump_and_keeps_position():
    fusion = VioFusion(FusionConfig(initial_position=(0.0, 0.0, 0.6)))
    simulate(fusion, n=20)
    before = fusion.position.copy()
    t = 20 / 15
    fusion.add_imu(t, matrix_to_quat(rot_z(0.4)))
    jump = transform(np.eye(3), (50.0, 0.0, 0.0))
    assert fusion.add_vo(t, jump, np.eye(4)) == 'rejected'
    assert np.allclose(fusion.position, before)


def test_waits_for_imu():
    fusion = VioFusion()
    assert fusion.add_vo(0.0, np.eye(4), np.eye(4)) == 'no_imu'
    assert not fusion.initialized
