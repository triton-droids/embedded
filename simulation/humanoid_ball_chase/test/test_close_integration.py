import time
from unittest.mock import Mock
import numpy as np
import pytest
import rclpy
from humanoid_ball_chase.vision_node import BallChaseVision
from humanoid_ball_chase.last_ball import LastBall
from humanoid_ball_chase.kick import WalkState, KickDecision

@pytest.fixture
def node():
    rclpy.init(args=['--ros-args','-p','detector:=orange_test','-p','world_model:=false'])
    n = BallChaseVision()
    n.kick_enabled = n.shoot_enabled = True
    n._walk = WalkState(True, None, 0, time.monotonic())
    n._foot_xy = Mock(return_value=(.03,.16))
    n._kick = Mock(state='approach')
    n._kick.update.return_value = KickDecision(True)
    n._kick_anchor = (np.array([.35,.16]), .02)
    n._kick_frozen = True
    n._lookup_transform = Mock(return_value=(np.eye(3), np.zeros(3)))
    yield n
    n.destroy_node()
    rclpy.shutdown()

def observe(n, t, point):
    for dt in (0., .11, .22):
        n._last_ball.observe(t+dt, point, .02, .9)

def test_far_anchor_alone_cannot_authorize_close_approach(node):
    d = node._kick_decision(None, np.eye(3), np.zeros(3), 2.)
    assert d.state == 'observe' and d.vx == 0 and d.kick_foot is None
    node._kick.update.assert_not_called()

def test_visible_updates_and_occluded_point_uses_current_base_transform(node):
    observe(node, 1., (.35,.16))
    node._kick_decision(None, np.eye(3), np.zeros(3), 1.3)
    assert node._last_ball.close_seen
    observe(node, 1.4, (.36,.16))
    # Body moved 5cm forward: fixed-world last image becomes 31cm in base.
    node._kick_decision(None, np.eye(3), np.array([-.05,0.,0.]), 2.)
    assert node._kick.update.call_args.args[1] == pytest.approx((.31,.16))
    node._kick.update.reset_mock()
    d = node._kick_decision(None, np.eye(3), np.zeros(3), 15.)
    assert d.state == 'observe'
    node._kick.update.assert_not_called()
