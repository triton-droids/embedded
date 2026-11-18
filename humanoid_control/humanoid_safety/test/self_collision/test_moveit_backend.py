"""MoveIt backend against the real leg URDF, and agreement with Pinocchio."""
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('moveit')
ament = pytest.importorskip('ament_index_python.packages')
try:
    # MoveIt loads package:// meshes through the ament index.
    ament.get_package_share_directory('humanoid_leg_description')
except ament.PackageNotFoundError:
    pytest.skip('humanoid_leg_description must be installed and sourced', allow_module_level=True)

from humanoid_safety.self_collision.checker import (  # noqa: E402
    SelfCollisionChecker,
    create_backend,
    urdf_adjacent_link_pairs,
)

REPO = Path(__file__).resolve().parents[4]
URDF = str(REPO / 'humanoid_leg_description/urdf/human_offset_corrected.urdf')
ALLOWED = urdf_adjacent_link_pairs(URDF) | {('hip', 'left_leg2'), ('hip', 'right_leg2')}
JOINTS = tuple(f'{side}_{joint}_joint' for side in ('left', 'right')
               for joint in ('hip1', 'hip2', 'thigh', 'knee', 'ankle'))


@pytest.fixture(scope='module')
def moveit():
    return create_backend('moveit', URDF, [str(REPO)], ALLOWED)


def pose(**overrides):
    positions = dict.fromkeys(JOINTS, 0.0)
    positions.update(overrides)
    return positions


def test_joints_and_pairs(moveit):
    assert moveit.joint_names == JOINTS
    assert ALLOWED.isdisjoint(moveit.link_pairs)
    assert len(moveit.link_pairs) == 53


def test_standing_pose_is_clear(moveit):
    result = SelfCollisionChecker(moveit, 0.01).check_pose(pose())
    assert result.level == 'clear'
    assert result.min_distance_m == pytest.approx(0.0215, abs=1e-3)


def test_crossed_legs_collide_with_zero_distance(moveit):
    crossed = pose(left_hip2_joint=0.436332, right_hip2_joint=-0.436332)
    result = SelfCollisionChecker(moveit, 0.01).check_pose(crossed)
    assert ('left_leg4', 'right_leg4') in result.colliding_pairs
    # MoveIt's Python bindings give no penetration depth.
    assert result.min_distance_m == 0.0


def test_single_query_matches_per_pair_minimum(moveit):
    for positions in (pose(), pose(left_hip2_joint=0.3), pose(left_hip2_joint=0.436332)):
        q = np.asarray([positions[name] for name in moveit.joint_names])
        assert moveit.min_distance(q) == pytest.approx(moveit.distances(q).min(), abs=1e-9)


def test_matches_pinocchio(moveit):
    pytest.importorskip('pinocchio')
    pinocchio = create_backend('pinocchio', URDF, [str(REPO)], ALLOWED)
    assert pinocchio.link_pairs == moveit.link_pairs
    rng = np.random.default_rng(1)
    for _ in range(5):
        q = rng.uniform(-0.4, 0.4, size=len(JOINTS))
        a, b = pinocchio.distances(q), moveit.distances(q)
        assert np.array_equal(a <= 0, b <= 0)
        clear = (a > 0) & (b > 0)
        assert np.max(np.abs(a - b)[clear]) < 1e-4
