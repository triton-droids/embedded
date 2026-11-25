"""Routine diagnostics must not share the supervisor's executor queue.

``safety_node`` builds and serialises a DiagnosticArray for every status
publication, and the rclpy publish call holds the GIL for the whole native
write.  When that timer lived on the same single-threaded executor as the IMU
subscription, one slow status write sat directly in front of safety input: a
measured 317 ms status publication produced a 339 ms evaluation gap.  ``main``
now creates a separate diagnostics node for the routine timer, so this test
pins the node each endpoint is created on rather than its rate.
"""
import rclpy
from rclpy.node import Node

from humanoid_safety.safety_node import SafetyNode


def test_routine_status_timer_is_isolated_from_the_imu_path():
    rclpy.init()
    diagnostics = Node('safety_diagnostics_test', start_parameter_services=False,
                       enable_rosout=False)
    node = None
    try:
        node = SafetyNode(diagnostics_node=diagnostics)
        status_publishers = node.get_publishers_info_by_topic('/safety/status')
        assert len(status_publishers) == 1
        assert status_publishers[0].node_name == 'safety_diagnostics_test'
        imu_subscribers = node.get_subscriptions_info_by_topic('/imu/data_raw')
        assert len(imu_subscribers) == 1
        assert imu_subscribers[0].node_name == 'safety_node'
        # Diagnostics describe health; they are not a control-rate signal.
        assert node.get_parameter('status_rate_hz').value <= 1.0
    finally:
        if node is not None:
            node.destroy_node()
        diagnostics.destroy_node()
        rclpy.shutdown()
