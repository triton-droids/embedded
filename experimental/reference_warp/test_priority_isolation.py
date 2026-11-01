"""Real executor regressions for diagnostics blocking the IMU critical path."""
import threading
import time
import pytest

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu

from humanoid_safety.safety_node import SafetyNode
from humanoid_sim.imu_nodes import SimBodyImuNode
from humanoid_sim.main import make_executors
from test_world import make_world


def test_sim_imu_keeps_publishing_while_its_status_publish_is_blocked():
    rclpy.init(args=[],domain_id=219)
    world = make_world()
    diag = Node('isolated_sim_diagnostics_test')
    imu = SimBodyImuNode(world, diagnostics_node=diag)
    entered, release = threading.Event(), threading.Event()
    def slow_status():
        entered.set()
        release.wait(2)
    imu._publish_status = slow_status
    timer = diag.create_timer(.01,slow_status)
    executors = make_executors([imu,diag],dedicated_imu=True)
    threads = [threading.Thread(target=e.spin) for e in executors]
    try:
        world.start()
        for thread in threads:
            thread.start()
        assert entered.wait(2)
        before = imu._published
        deadline = time.monotonic()+.25
        while time.monotonic()<deadline and imu._published-before<5:
            time.sleep(.005)
        assert imu._published-before>=5
    finally:
        release.set()
        timer.cancel()
        for e in executors:
            e.shutdown(timeout_sec=2)
        for t in threads:
            t.join(timeout=2)
        world.stop()
        imu.destroy_node()
        diag.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('isolated',[False,True])
def test_safety_receives_fresh_imu_during_blocked_diagnostics_and_still_trips_on_loss(isolated):
    rclpy.init(args=['--ros-args','-p','trip_on_imu_stuck:=false'],domain_id=219)
    diag = Node('isolated_safety_diagnostics_test',use_global_arguments=False)
    safety = SafetyNode(diagnostics_node=diag if isolated else None)
    source = Node('safety_priority_imu_source')
    pub = source.create_publisher(Imu,'/imu/data_raw',10)
    entered, release = threading.Event(), threading.Event()
    original_publish = safety._status_pub.publish
    def slow_publish(msg):
        entered.set()
        release.wait(2)
        original_publish(msg)
    safety._status_pub.publish = slow_publish
    executors = [SingleThreadedExecutor(),SingleThreadedExecutor()]
    executors[0].add_node(safety)
    executors[1].add_node(diag)
    threads = [threading.Thread(target=e.spin) for e in executors]
    try:
        for thread in threads:
            thread.start()
        assert entered.wait(2)
        before = safety._imu_received
        deadline = time.monotonic()+.45
        while time.monotonic()<deadline:
            msg = Imu()
            msg.header.stamp = source.get_clock().now().to_msg()
            msg.orientation.w = 1.0
            pub.publish(msg)
            time.sleep(.01)
        if not isolated:
            assert safety._imu_received==before, 'shared executor should reproduce receipt starvation'
            return
        assert safety._imu_received-before>=10
        assert not safety.supervisor.latched
        # Continue holding diagnostics, but actually stop IMU source now.
        deadline = time.monotonic()+.6
        while not safety.supervisor.latched and time.monotonic()<deadline:
            time.sleep(.01)
        assert safety.supervisor.latched
        assert 'imu_freshness' in safety.supervisor.reason
    finally:
        release.set()
        for e in executors:
            e.shutdown(timeout_sec=2)
        for t in threads:
            t.join(timeout=2)
        safety.destroy_node()
        diag.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
