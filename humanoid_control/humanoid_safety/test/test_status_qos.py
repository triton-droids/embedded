"""Diagnostics may drop old samples; the emergency-stop channel stays reliable."""
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from humanoid_safety.safety_node import SafetyNode


@pytest.mark.parametrize('mode,expected,depth', [
    ('reliable', ReliabilityPolicy.RELIABLE, 2),
    ('best_effort', ReliabilityPolicy.BEST_EFFORT, 2),
])
def test_status_qos_does_not_change_estop_contract(mode, expected, depth):
    rclpy.init(args=['--ros-args', '-p', f'status_reliability:={mode}'])
    node = None
    try:
        node = SafetyNode()
        assert node._status_pub.qos_profile.reliability == expected
        assert node._status_pub.qos_profile.depth == depth
        assert node._estop_pub.qos_profile.depth == 2
        assert node._estop_pub.qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert node._estop_pub.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert node.get_parameter('trip_on_imu_timeout').value
        assert node.get_parameter('imu_timeout_s').value == 0.25
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()
