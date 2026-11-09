"""velocity.step_command: velocity -> step length / turn per step (no ROS)."""
import pytest

from walking_controller.velocity import step_command


def test_plain_conversion_and_caps():
    assert step_command(0.08, 0.10, 0.5, 0.06, 0.08) == pytest.approx((0.04, 0.05))
    assert step_command(0.20, -0.50, 0.5, 0.06, 0.08) == pytest.approx((0.06, -0.08))
    assert step_command(-0.2, 0.0, 0.5, 0.06, 0.08)[0] == 0.0


@pytest.mark.parametrize('w, step', [(0.0, 0.06), (0.08, 0.06), (0.14, 0.03), (0.30, 0.02), (-0.30, 0.02)])
def test_turn_first_scaling(w, step):
    assert step_command(0.2, w, 0.5, 0.06, 0.08, 0.04, 0.10)[0] == pytest.approx(step)


def test_turn_first_stops_at_the_stop_turn():
    assert step_command(0.2, 0.5, 0.5, 0.06, 0.15, 0.04, 0.10)[0] == 0.0
