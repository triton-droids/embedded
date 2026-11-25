from types import SimpleNamespace
from unittest.mock import Mock
from rclpy._rclpy_pybind11 import RCLError
from humanoid_safety.safety_node import SafetyNode


def test_estop_failure_stays_pending_and_retries_same_assertion():
    node = SimpleNamespace(supervisor=SimpleNamespace(latched=True),
        _last_published_estop=False, _estop_pub=Mock(), get_logger=lambda: Mock())
    node._estop_pub.publish.side_effect = [RCLError('injected'), None]
    SafetyNode._publish_estop(node)
    assert node._last_published_estop is False
    SafetyNode._publish_estop(node)
    assert node._last_published_estop is True
    assert all(c.args[0].data is True for c in node._estop_pub.publish.call_args_list)
