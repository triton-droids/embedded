"""Pinocchio backend against the real leg URDF and collision meshes."""
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('pinocchio')

from humanoid_safety.self_collision.checker import (  # noqa: E402
    SelfCollisionChecker,
    create_backend,
    urdf_adjacent_link_pairs,
    urdf_joint_limits,
)

REPO = Path(__file__).resolve().parents[4]
URDF = str(REPO / 'humanoid_leg_description/urdf/human_offset_corrected.urdf')
JOINTS = tuple(f'{side}_{joint}_joint' for side in ('left', 'right')
               for joint in ('hip1', 'hip2', 'thigh', 'knee', 'ankle'))


def backend(name='pinocchio'):
    return create_backend(name, URDF, [str(REPO)], urdf_adjacent_link_pairs(URDF))


@pytest.fixture(scope='module')
def convex():
    return backend()


def pose(**overrides):
    positions = dict.fromkeys(JOINTS, 0.0)
    positions.update(overrides)
    return positions


def test_joints_match_the_policy_joints(convex):
    assert convex.joint_names == JOINTS


def test_adjacent_links_are_not_checked(convex):
    adjacent = urdf_adjacent_link_pairs(URDF)
    assert adjacent.isdisjoint(convex.link_pairs)
    assert ('left_leg3', 'left_leg4') in adjacent


def test_standing_pose_is_clear(convex):
    result = SelfCollisionChecker(convex, 0.01).check_pose(pose())
    assert result.level == 'clear'
    # Closest at zero angles: torso box to an upper hip link, about 2 cm.
    assert 0.015 < result.min_distance_m < 0.03
    assert 'torso' in result.closest_pair


def test_legs_swung_inward_collide(convex):
    crossed = pose(left_hip2_joint=0.436332, right_hip2_joint=-0.436332)
    result = SelfCollisionChecker(convex, 0.01).check_pose(crossed)
    assert result.level == 'collision'
    assert ('left_leg4', 'right_leg4') in result.colliding_pairs
    assert result.min_distance_m < 0


def test_path_into_collision_is_flagged(convex):
    crossed = pose(left_hip2_joint=0.436332, right_hip2_joint=-0.436332)
    result = SelfCollisionChecker(convex, 0.01).check_path(pose(), crossed, 0.05, 100)
    assert result.level == 'collision'
    assert result.poses_checked == 10


def test_convex_matches_exact_mesh_distance(convex):
    """The *_ch.stl meshes are convex hulls, so both backends should agree."""
    mesh = backend('pinocchio_mesh')
    assert mesh.link_pairs == convex.link_pairs
    lower, upper = urdf_joint_limits(URDF, JOINTS)
    for positions in np.random.default_rng(0).uniform(lower, upper, size=(5, len(JOINTS))):
        a, b = convex.distances(positions), mesh.distances(positions)
        assert np.array_equal(a <= 0, b <= 0)
        clear = (a > 0) & (b > 0)
        assert np.max(np.abs(a - b)[clear]) < 1e-4
