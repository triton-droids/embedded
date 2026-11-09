"""Edge contacts in the WBC: corner choice, per-foot force cap, lateral CoM authority.

Without ankle roll the lateral CoM can only be accelerated in double support
when both feet roll on parallel edges (README, "How it works"). Measured from
the offline standing state: achieved / requested lateral CoM acceleration is
0.006 with both feet flat, 0.009 with one foot on an edge and 0.97-0.98 with
both on parallel edges.
"""
import numpy as np
import pinocchio as pin
import pytest

from conftest import GAINS


@pytest.fixture
def setup(model, standing):
    from walking_controller.robot import FEET
    paths, robot, limits = model
    q, v = standing['q'], standing['v']
    robot.forward(q, v)
    lat = robot.sole_center(FEET[0]) - robot.sole_center(FEET[1])
    lat[2] = 0.0
    return robot, limits, q, v, lat / np.linalg.norm(lat), standing['controller']


def solve(setup, modes, com_acc_lat=0.0, cap=None):
    from walking_controller.wbc import WBCGains, WholeBodyController
    robot, limits, q, v, lat, stand = setup
    wbc = WholeBodyController(robot, limits.torque, WBCGains(**GAINS))
    for foot, mode in modes.items():
        wbc.set_mode(foot, mode, q)
    if cap is not None:
        wbc.set_max_force(*cap)
    robot.forward(q, v)
    wbc.set_torso_yaw(stand.yaw)
    wbc.set_posture(stand.q_nominal)
    wbc.set_com(robot.com.copy(), np.zeros(3), com_acc_lat * lat)
    result = wbc.solve(q, v)
    data = pin.Data(robot.model)
    pin.centerOfMass(robot.model, data, q, v, result.ddq)
    return wbc, result, float(data.acom[0] @ lat)


def test_edge_mode_uses_two_points_on_the_requested_edge(setup):
    from walking_controller.robot import FEET
    from walking_controller.wbc import INNER, OUTER
    robot, _, q, _, lat, _ = setup
    wbc, _, _ = solve(setup, {FEET[0]: INNER, FEET[1]: OUTER})
    left_pts, right_pts = robot.sole_points(FEET[0]), robot.sole_points(FEET[1])
    for foot, points, sign, mode in ((FEET[0], left_pts, -1, INNER), (FEET[1], right_pts, -1, OUTER)):
        names = wbc.contacts[foot]
        assert len(names) == 2 and all(n.endswith('_point') for n in names)
        used = sorted(int(n.split('_c')[-1].split('_')[0]) for n in names)
        # Left inner edge and right outer edge both lie on the -lat side of their foot.
        side = lat @ (points[:, used].mean(axis=1) - points.mean(axis=1))
        assert sign * side > 0.0, (foot, mode, used)


@pytest.mark.parametrize('mode', ['flat', 'inner'])
def test_max_force_caps_the_whole_foot(setup, mode):
    """set_max_force limits the foot's total normal force, flat (one Contact6d)
    or on an edge (split over its two points)."""
    from walking_controller.robot import FEET
    other_mode = 'flat' if mode == 'flat' else 'outer'
    for cap in (0.0, 30.0, 60.0):
        _, result, _ = solve(setup, {FEET[0]: mode, FEET[1]: other_mode}, cap=(FEET[0], cap))
        assert result.forces[FEET[0]] <= cap + 1e-3
        assert result.forces[FEET[1]] > 40.0       # the load goes to the other foot


def test_lateral_com_is_locked_unless_both_feet_roll(setup):
    from walking_controller.robot import FEET
    from walking_controller.wbc import FLAT, INNER, OUTER
    ratios = {}
    for name, modes in {'flat': {FEET[0]: FLAT, FEET[1]: FLAT},
                        'one_edge': {FEET[0]: FLAT, FEET[1]: OUTER},
                        'lean_right': {FEET[0]: INNER, FEET[1]: OUTER},
                        'lean_left': {FEET[0]: OUTER, FEET[1]: INNER}}.items():
        for acc in (0.5, -0.5):
            _, _, achieved = solve(setup, modes, com_acc_lat=acc)
            ratios[name, acc] = achieved / acc
    for acc in (0.5, -0.5):
        assert abs(ratios['flat', acc]) < 0.03
        assert abs(ratios['one_edge', acc]) < 0.03
        assert ratios['lean_right', acc] > 0.8
        assert ratios['lean_left', acc] > 0.8
