"""Drop failed sensor/diagnostic writes; never retry or re-stamp an old sample."""
from rclpy._rclpy_pybind11 import RCLError


def publish_sample(node, publisher, message, topic):
    try:
        publisher.publish(message)
        return True
    except RCLError as error:
        if not hasattr(node, '_publish_failures'):
            node._publish_failures = {}
        previous = node._publish_failures.get(topic, {})
        node._publish_failures[topic] = dict(count=previous.get('count', 0)+1, error=str(error))
        # Logging can itself wait on ROS/DDS logging locks while holding the GIL.
        # Expose the counters in /imu/status, /sim/status and /can_timing instead
        # of introducing another blocking publication into the acquisition path.
        return False
