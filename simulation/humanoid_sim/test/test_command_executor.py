"""Slow feedback work must not prevent admission of reliable lifecycle commands."""
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import yaml
from motor_control_interfaces.msg import MotorCommand
from humanoid_sim.motor_backend_node import SimMotorBackendNode
from test_world import make_world


def test_command_and_estop_group_can_run_while_feedback_is_busy():
    # Never publish test ENABLE traffic into a robot's normal ROS domain.
    rclpy.init(args=[], domain_id=193)
    backend = sender = receiver = executor = command_executor = thread = command_thread = None
    release, entered = threading.Event(), threading.Event()
    try:
        world = make_world()
        root = Path(__file__).resolve().parents[3]
        registry = yaml.safe_load((root / 'humanoid_control/motor_control_hybrid/config/motors.yaml').read_text())['motor_control_node']['ros__parameters']
        receiver = Node("test_motor_receiver", start_parameter_services=False)
        backend = SimMotorBackendNode(world, registry, command_node=receiver)
        sender = Node('test_command_arrival')
        pub = sender.create_publisher(MotorCommand, '/motor_commands', 10)
        executor = SingleThreadedExecutor()
        command_executor = SingleThreadedExecutor()
        command_executor.add_node(receiver)
        command_thread = threading.Thread(target=command_executor.spin)
        command_thread.start()
        executor.add_node(backend)
        executor.add_node(sender)
        thread = threading.Thread(target=executor.spin)
        thread.start()
        end = time.monotonic() + 3.
        while pub.get_subscription_count() == 0 and time.monotonic() < end:
            time.sleep(.01)
        assert pub.get_subscription_count() > 0
        def busy_feedback():
            entered.set()
            release.wait(2.)
        timer = backend.create_timer(.01, busy_feedback)
        assert entered.wait(2.)
        msg = MotorCommand()
        msg.header.stamp = sender.get_clock().now().to_msg()
        msg.joint_name = list(world.joint_names)
        msg.mode = [MotorCommand.MODE_ENABLE]
        pub.publish(msg)
        end = time.monotonic() + .4
        enabled = False
        while time.monotonic() < end:
            with world.lock:
                enabled = all(world.motors.is_enabled(n) for n in world.joint_names)
            if enabled:break
            time.sleep(.005)
        assert enabled, 'feedback callback blocked command admission'
        command_group = next(s.callback_group for s in receiver.subscriptions if s.topic_name == '/motor_commands')
        estop_group = next(s.callback_group for s in receiver.subscriptions if s.topic_name == '/safety/estop')
        assert command_group is estop_group
        assert command_group is not receiver.default_callback_group
        timer.cancel()
    finally:
        release.set()
        if executor is not None:executor.shutdown(timeout_sec=3.)
        if command_executor is not None:command_executor.shutdown(timeout_sec=3.)
        if thread is not None:thread.join(timeout=3.)
        if command_thread is not None:command_thread.join(timeout=3.)
        if receiver is not None:receiver.destroy_node()
        if backend is not None:backend.destroy_node()
        if sender is not None:sender.destroy_node()
        rclpy.shutdown()
