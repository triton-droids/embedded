"""Backend-independent self-collision logic, using a fake backend."""
from pathlib import Path

import numpy as np
import pytest

from humanoid_safety.self_collision.checker import (
    PoseSample,
    SelfCollisionChecker,
    evaluate,
    parse_link_pairs,
    sample_problem,
    srdf_disabled_link_pairs,
    urdf_adjacent_link_pairs,
)

REPO = Path(__file__).resolve().parents[4]
URDF = REPO / 'humanoid_leg_description/urdf/human_offset_corrected.urdf'
SRDF = REPO / 'humanoid_leg_moveit_config/config/humanoid_leg.srdf'


class FakeBackend:
    """Pair (x, y) is |a - 0.5| - 0.01 from colliding; (x, z) is b metres clear."""
    name = 'fake'
    joint_names = ('a', 'b')
    link_pairs = (('x', 'y'), ('x', 'z'))

    def __init__(self):
        self.calls = []

    def distances(self, positions):
        self.calls.append(positions.copy())
        return np.array([abs(positions[0] - 0.5) - 0.01, positions[1]])


def checker(margin=0.05):
    return SelfCollisionChecker(FakeBackend(), margin)


def test_pose_levels_follow_the_margin():
    clear = checker().check_pose({'a': 0.0, 'b': 1.0})
    assert clear.level == 'clear'
    assert clear.closest_pair == ('x', 'y')
    assert clear.min_distance_m == pytest.approx(0.49)

    close = checker().check_pose({'a': 0.0, 'b': 0.02})
    assert close.level == 'close'
    assert close.close_pairs == (('x', 'z'),)

    hit = checker().check_pose({'a': 0.5, 'b': 0.02})
    assert hit.level == 'collision'
    assert hit.colliding_pairs == (('x', 'y'),)
    assert hit.min_distance_m == pytest.approx(-0.01)


def test_missing_or_nonfinite_joints_are_rejected():
    with pytest.raises(ValueError, match='Missing joints: b'):
        checker().check_pose({'a': 0.0})
    with pytest.raises(ValueError, match='finite'):
        checker().check_pose({'a': float('nan'), 'b': 1.0})


def test_path_finds_a_collision_between_clear_endpoints():
    path_checker = checker()
    start, end = {'a': 0.0, 'b': 1.0}, {'a': 1.0, 'b': 1.0}
    assert path_checker.check_pose(start).level == 'clear'
    assert path_checker.check_pose(end).level == 'clear'

    result = path_checker.check_path(start, end, max_joint_step_rad=0.05, max_poses=100)
    assert result.level == 'collision'
    assert result.poses_checked == 21  # 20 steps of 0.05 rad, both endpoints included
    samples = path_checker.backend.calls[-21:]
    assert samples[0][0] == 0.0 and samples[-1][0] == 1.0


def test_path_sample_cap_is_reported():
    result = checker().check_path({'a': 0.0, 'b': 1.0}, {'a': 1.0, 'b': 1.0},
                                  max_joint_step_rad=0.05, max_poses=5)
    assert result.poses_checked == 5


def test_invalid_settings_are_rejected():
    with pytest.raises(ValueError):
        SelfCollisionChecker(FakeBackend(), -0.01)
    with pytest.raises(ValueError):
        checker().check_path({'a': 0, 'b': 0}, {'a': 1, 'b': 0}, 0.0, 10)


def sample(received_at=10.0, stamp=100.0, positions=None):
    return PoseSample(positions or {'a': 0.0, 'b': 1.0}, received_at, stamp)


@pytest.mark.parametrize('pose, problem', [
    (None, 'no input'),
    (sample(stamp=0.0), 'unstamped'),
    (sample(stamp=99.8), 'stale'),  # source is 0.2 s old
    (sample(stamp=100.05), 'stale'),  # source is in the future
    (sample(received_at=9.8), 'received'),  # held locally too long
    (sample(positions={'a': 0.0}), 'missing: b'),
])
def test_unusable_samples_explain_why(pose, problem):
    assert problem in sample_problem(pose, ('a', 'b'), 10.0, 100.0, 0.1)


def test_fresh_complete_sample_is_usable():
    assert sample_problem(sample(), ('a', 'b'), 10.0, 100.0, 0.1) == ''


def test_stale_target_reports_problem_instead_of_old_clearance():
    reports = evaluate(checker(), sample(), sample(stamp=99.0),
                       now_monotonic=10.0, now_ros=100.0, timeout_s=0.1,
                       max_joint_step_rad=0.05, max_path_poses=100)
    by_check = {report.check: report for report in reports}
    assert by_check['measured'].result.level == 'clear'
    for check in ('target', 'path'):
        assert by_check[check].result is None
        assert 'stale' in by_check[check].problem


def test_backend_failure_is_reported_not_raised():
    failing = checker()
    failing.backend.distances = lambda positions: np.array([np.nan, 0.0])
    reports = evaluate(failing, sample(), sample(), 10.0, 100.0, 0.1, 0.05, 100)
    assert all(report.problem.startswith('check failed') for report in reports)


def test_parse_link_pairs_is_order_independent():
    assert parse_link_pairs(' b:a , c:d,') == {('a', 'b'), ('c', 'd')}
    with pytest.raises(ValueError):
        parse_link_pairs('a:b:c')


def test_urdf_adjacency_matches_srdf_adjacent_pairs():
    """The SRDF's Adjacent entries are exactly the URDF's parent/child links."""
    from_urdf = urdf_adjacent_link_pairs(str(URDF))
    from_srdf = srdf_disabled_link_pairs(str(SRDF))
    assert from_srdf <= from_urdf
    # The only URDF-only pair is the fixed world attachment.
    assert all('world' in pair for pair in from_urdf - from_srdf)
