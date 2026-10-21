"""Safety-estop gate for the Torch policy bridge.

The bridge used to be the only command source that published without consulting
`/safety/estop`. These tests pin the contract using a bare node object: no ROS
init, no JSON config, no torch policy and no serial/CAN I/O.
"""
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('motor_control_interfaces.msg')

from std_msgs.msg import Bool  # noqa: E402

from motor_control_hybrid.policy_bridge_node import PolicyBridgeNode  # noqa: E402


class FakeLogger:
    """Collect (level, message) pairs instead of writing to rosout."""

    def __init__(self):
        self.records = []

    def error(self, message):
        self.records.append(('error', message))

    def warn(self, message):
        self.records.append(('warn', message))

    def info(self, message):
        self.records.append(('info', message))

    def levels(self):
        return [level for level, _ in self.records]


def make_bridge(inputs_ready=False):
    """Build the minimum node state the estop gate touches."""
    node = PolicyBridgeNode.__new__(PolicyBridgeNode)
    node.estop = False
    node.estop_warned = False
    node.logger = FakeLogger()
    node.get_logger = lambda: node.logger
    node.inputs_checked = 0

    def _inputs_ready():
        node.inputs_checked += 1
        return inputs_ready

    node._inputs_ready = _inputs_ready
    return node


def test_bridge_starts_with_the_gate_open():
    node = make_bridge()
    assert node.estop is False
    assert node.estop_warned is False


def test_asserted_estop_blocks_tick_before_inputs_are_read():
    node = make_bridge()
    node._estop_callback(Bool(data=True))
    assert node.estop is True
    assert node.logger.records == [
        ('error', 'Safety estop asserted: policy output suspended, no motor commands published')
    ]
    node._tick()
    assert node.inputs_checked == 0, 'a latched estop must not reach the policy pipeline'


def test_latched_estop_warns_once_per_assertion():
    node = make_bridge()
    node._estop_callback(Bool(data=True))
    node._tick()
    node._tick()
    node._tick()
    errors = [message for level, message in node.logger.records if level == 'error']
    assert len(errors) == 2, 'assertion + one "still latched" notice only'
    assert 'still latched' in errors[-1]


def test_repeated_true_messages_do_not_re_log():
    node = make_bridge()
    node._estop_callback(Bool(data=True))
    node._estop_callback(Bool(data=True))
    assert node.logger.levels() == ['error']


def test_clearing_the_latch_resumes_input_evaluation():
    node = make_bridge()
    node._estop_callback(Bool(data=True))
    node._estop_callback(Bool(data=False))
    assert node.estop is False
    node._tick()
    assert node.inputs_checked == 1, 'cleared latch must evaluate inputs again'
    assert node.logger.records[-1][0] == 'warn'
