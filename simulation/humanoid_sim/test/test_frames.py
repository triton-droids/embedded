import math

import numpy as np
import pytest

from humanoid_sim import frames


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])


def quat_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def test_body_from_base_maps_forward_to_the_scene_axis():
    assert frames.body_from_base('positive_y') @ [1, 0, 0] == pytest.approx([0, 1, 0])
    assert frames.body_from_base('negative_y') @ [1, 0, 0] == pytest.approx([0, -1, 0])
    with pytest.raises(ValueError):
        frames.body_from_base('x')


@pytest.mark.parametrize('R', [np.eye(3), rot_z(2.9), rot_z(-1.0) @ rot_y(0.3), rot_y(math.pi),
                               rot_z(math.pi / 2) @ rot_y(-2.5)])
def test_quaternion_round_trip(R):
    q = frames.quaternion_xyzw(R)
    assert np.linalg.norm(q) == pytest.approx(1.0)
    assert q[3] >= 0.0
    assert quat_matrix(*q) == pytest.approx(R, abs=1e-12)


def test_footprint_keeps_yaw_and_drops_tilt_and_height():
    T = frames.rigid(rot_z(0.7) @ rot_y(0.2), np.array([1.0, -2.0, 0.45]))
    fp = frames.footprint(T)
    assert fp[:3, 3] == pytest.approx([1.0, -2.0, 0.0])
    assert frames.yaw_of(fp) == pytest.approx(0.7)
    assert fp[2, :3] == pytest.approx([0, 0, 1])


def test_split_odometry_composes_back_and_starts_at_identity():
    start = frames.rigid(rot_z(1.2) @ rot_y(0.05), np.array([0.3, 1.8, 0.5]))
    world_from_odom = frames.footprint(start)
    odom_fp, fp_base = frames.split_odometry(world_from_odom, start)
    assert odom_fp == pytest.approx(np.eye(4), abs=1e-12)  # odom origin = start footprint
    assert fp_base[:3, 3] == pytest.approx([0, 0, 0.5])

    # Robot walks 0.4 m along its own heading and turns 0.3 rad.
    moved = frames.rigid(rot_z(1.5) @ rot_y(-0.04),
                         np.array([0.3 + 0.4 * math.cos(1.2), 1.8 + 0.4 * math.sin(1.2), 0.48]))
    odom_fp, fp_base = frames.split_odometry(world_from_odom, moved)
    assert odom_fp[:3, 3] == pytest.approx([0.4, 0.0, 0.0])  # forward in odom = start heading
    assert frames.yaw_of(odom_fp) == pytest.approx(0.3)
    assert world_from_odom @ odom_fp @ fp_base == pytest.approx(moved, abs=1e-12)


def test_world_point_is_static_in_odom_while_robot_moves():
    world_from_odom = frames.planar(2.0, -1.0, 0.8)
    ball_world = np.array([3.0, 0.5, 0.11, 1.0])
    for x, y, yaw in [(2.0, -1.0, 0.8), (2.3, -0.7, 1.4), (1.5, 0.2, -2.0)]:
        base = frames.rigid(rot_z(yaw), np.array([x, y, 0.5]))
        odom_fp, fp_base = frames.split_odometry(world_from_odom, base)
        ball_base = frames.inverse(base) @ ball_world  # what the robot measures
        ball_odom = odom_fp @ fp_base @ ball_base
        assert ball_odom == pytest.approx(frames.inverse(world_from_odom) @ ball_world, abs=1e-12)


def test_footprint_twist_is_in_robot_axes():
    base = frames.rigid(rot_z(math.pi / 2), np.array([0., 0., 0.5]))
    v, w = frames.footprint_twist(base, [0.0, 0.2, 0.0], [0.0, 0.0, 0.3])
    assert v == pytest.approx([0.2, 0.0, 0.0])  # world +y = robot forward here
    assert w == pytest.approx([0.0, 0.0, 0.3])
