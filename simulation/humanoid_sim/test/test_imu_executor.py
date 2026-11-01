"""A slow non-IMU callback must not starve the real simulated IMU timer."""
import threading
import time
import gc

import rclpy
from rclpy.node import Node
from humanoid_sim.imu_nodes import SimBodyImuNode
from humanoid_sim.main import make_executors
from test_world import make_world


def test_imu_timer_continues_while_feedback_executor_is_blocked():
    rclpy.init(args=[], domain_id=197)
    world = make_world()
    slow = Node('blocked_feedback_test')
    imu = SimBodyImuNode(world)
    assert imu.imu_pub.qos_profile.depth == 2
    assert imu.status_pub.qos_profile.depth == 2
    entered, release = threading.Event(), threading.Event()
    executors, threads = [], []
    timer = None
    try:
        def blocked_feedback():
            entered.set()
            release.wait(1.5)
        timer = slow.create_timer(.01, blocked_feedback)
        executors = make_executors([slow,imu])
        world.start()
        threads = [threading.Thread(target=e.spin) for e in executors]
        for t in threads:
            t.start()
        assert entered.wait(2)
        before = imu._published
        end = time.monotonic()+.25
        while time.monotonic()<end and imu._published-before<5:
            time.sleep(.005)
        assert imu._published-before>=5, 'shared feedback callback starved IMU publication'
        assert imu._publish_native_tid in [t.native_id for t in threads]
        assert imu._publish_native_tid != threading.get_native_id()
    finally:
        release.set()
        if timer is not None:
            timer.cancel()
        for e in executors:
            e.shutdown(timeout_sec=2)
        for t in threads:
            t.join(timeout=2)
        world.stop()
        imu.destroy_node()
        slow.destroy_node()
        rclpy.shutdown()


def test_optional_gc_monitor_is_removed_on_node_destruction():
    rclpy.init(args=['--ros-args','-p','gc_diagnostics:=true'], domain_id=197)
    before = list(gc.callbacks)
    imu = None
    try:
        imu = SimBodyImuNode(make_world())
        assert imu._gc_callback in gc.callbacks
        imu.destroy_node()
        imu = None
        assert gc.callbacks == before
    finally:
        if imu is not None:
            imu.destroy_node()
        rclpy.shutdown()
