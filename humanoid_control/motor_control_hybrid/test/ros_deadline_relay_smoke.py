"""Isolated DDS relay test: no CAN process or hardware topics."""
import os
from pathlib import Path
import subprocess
import time
import rclpy
from rclpy.node import Node
from motor_control_interfaces.msg import MotorCommand
from sensor_msgs.msg import JointState

if os.environ.get('ROS_DOMAIN_ID') != '181':
    raise SystemExit('Run this software-only smoke test with ROS_DOMAIN_ID=181')
rclpy.init()
node = Node('deadline_relay_probe')
outputs = []
node.create_subscription(MotorCommand, '/deadline_probe/output', outputs.append, 20)
state_pub = node.create_publisher(JointState, '/deadline_probe/state', 1)
cmd_pub = node.create_publisher(MotorCommand, '/deadline_probe/input', 1)
exe = Path(__file__).resolve().parents[3] / 'install/motor_control_hybrid/lib/motor_control_hybrid/cpp_control_node'
with open('/tmp/deadline-relay.log', 'w') as log:
    child = subprocess.Popen([str(exe), '--ros-args', '-p', 'joint_names:=[a,b]',
        '-r', '/joint_states:=/deadline_probe/state',
        '-r', '/desired_motor_subset:=/deadline_probe/input',
        '-r', '/desired_velocity_subset:=/deadline_probe/velocity',
        '-r', '/motor_commands:=/deadline_probe/output',
        '-r', '/safety/estop:=/deadline_probe/estop'], stdout=log, stderr=log)
    try:
        limit = time.monotonic() + 4
        while time.monotonic() < limit and cmd_pub.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=.02)
        assert cmd_pub.get_subscription_count(), 'No isolated DDS discovery'
        for _ in range(10):
            state = JointState(name=['a','b'], position=[0.,0.], velocity=[0.,0.])
            state.header.stamp = node.get_clock().now().to_msg()
            state_pub.publish(state)
            rclpy.spin_once(node, timeout_sec=.01)
        outputs.clear()
        now_ns = node.get_clock().now().nanoseconds
        cmd = MotorCommand(joint_name=['a','b'], mode=[MotorCommand.MODE_MOTION],
                           position=[.1,.1])
        cmd.header.stamp.sec, cmd.header.stamp.nanosec = divmod(now_ns - 10_000_000, 10**9)
        cmd.valid_until.sec, cmd.valid_until.nanosec = divmod(now_ns + 100_000_000, 10**9)
        cmd_pub.publish(cmd)
        limit = time.monotonic() + .3
        while time.monotonic() < limit:
            state.header.stamp = node.get_clock().now().to_msg()
            state_pub.publish(state)
            rclpy.spin_once(node, timeout_sec=.005)
        motion = [m for m in outputs if MotorCommand.MODE_MOTION in m.mode]
        assert motion, 'No forwarded motion command'
        stamp_ns = lambda t: t.sec * 10**9 + t.nanosec
        assert all(abs(stamp_ns(m.header.stamp) - stamp_ns(cmd.header.stamp)) < 1000 for m in motion)
        assert all(abs(stamp_ns(m.valid_until) - stamp_ns(cmd.valid_until)) < 1000 for m in motion)
        assert any(MotorCommand.MODE_DISABLE in m.mode for m in outputs), 'No deadline disable'
        assert list(outputs[-1].mode) == [MotorCommand.MODE_DISABLE], 'Last output must disable'
        print(f'PASS: {len(motion)} relay samples preserve source/expiry; explicit deadline disables with fresh joint feedback')
    finally:
        child.terminate()
        child.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
