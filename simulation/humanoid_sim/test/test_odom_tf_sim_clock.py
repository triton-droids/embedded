"""Odometry TF restamping must stay on the simulated clock."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from humanoid_sim.odom_tf_node import SimOdomTfNode

PERIOD_NS = 20_000_000


def _node(sim_clock, sample_ns, clock_ns):
    world = SimpleNamespace(sim_clock=sim_clock, clock_ns=clock_ns)
    node = SimpleNamespace(
        world=world, _period_ns=PERIOD_NS, _last_stamp_ns=sample_ns, publish_robot=True,
        _world_from_odom=np.eye(4), _reset_count=0, _tf=Mock(), _odom_pub=None,
        odom_frame='odom', footprint_frame='base_footprint', base_frame='base_link',
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: SimpleNamespace(sec=0, nanosec=0))),
        _sample=lambda: (np.eye(4), np.zeros(3), np.zeros(3), sample_ns, 0))
    return node


def _stamp(node):
    msg = node._tf.sendTransform.call_args.args[0][0]
    return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec


def test_repeated_sample_is_restamped_with_simulated_time_not_host_time():
    sample = 1_000 * 10**9
    node = _node(True, sample, clock_ns=sample + 3 * PERIOD_NS)  # Physics behind the timer.
    SimOdomTfNode._tick(node)
    assert _stamp(node) == sample + 3 * PERIOD_NS  # Host time would be ~1.8e18 ns.


def test_paused_simulation_repeats_nothing():
    sample = 1_000 * 10**9
    node = _node(True, sample, clock_ns=sample)  # Clock frozen with the physics.
    SimOdomTfNode._tick(node)
    node._tf.sendTransform.assert_not_called()
