"""Live fake-motor -> monitor -> robot_state_publisher regression test."""
import math
import os
import signal
import subprocess
import tempfile
import time

import pytest
import rclpy
from motor_control_interfaces.msg import MotorCommand
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


@pytest.mark.skipif(os.environ.get('RUN_ROS_INTEGRATION') != '1',
                    reason='Requires DDS sockets; set RUN_ROS_INTEGRATION=1 in a sourced workspace')
def test_fake_motor_pose_reaches_tf():
    # Run with a dedicated ROS_DOMAIN_ID and a sourced workspace.
    processes = []
    rclpy.init()
    node = rclpy.create_node('pose_monitor_integration_test')
    source, output, transforms = [], [], []
    node.create_subscription(JointState, '/pose_test/feedback', source.append, qos_profile_sensor_data)
    node.create_subscription(JointState, '/pose_test/display', output.append, qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf', transforms.append, qos_profile_sensor_data)
    publisher = node.create_publisher(MotorCommand, '/pose_test/motor_commands', 10)

    def wait_for(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError('Timed out waiting for fake motor / joint relay / TF')

    with tempfile.TemporaryFile(mode='w+') as log:
        try:
            for args in (
                ['motor_control_hybrid', 'joint_state_monitor.launch.py',
                 'use_fake_motor:=true', 'source_topic:=/pose_test/feedback',
                 'output_topic:=/pose_test/display'],
                ['humanoid_leg_description', 'display.launch.py',
                 'use_joint_state_gui:=false', 'use_rviz:=false',
                 'joint_states_topic:=/pose_test/display'],
            ):
                processes.append(subprocess.Popen(['ros2', 'launch', *args],
                    stdout=log, stderr=log, start_new_session=True))
            wait_for(lambda: source and output and transforms and publisher.get_subscription_count())
            assert len(output[-1].name) == 10
            assert 'left_hip1_joint' in output[-1].name
            msg = MotorCommand(joint_name=['left_hip1_joint'], mode=[MotorCommand.MODE_ENABLE])
            publisher.publish(msg)
            wait_for(lambda: any('Enabled left_hip1_joint' in line for line in read_log(log)))
            publisher.publish(MotorCommand(joint_name=['left_hip1_joint'],
                mode=[MotorCommand.MODE_POSITION], position=[0.4], velocity=[1.0]))
            wait_for(lambda: abs(output[-1].position[output[-1].name.index('left_hip1_joint')] - 0.4) < 1e-5)
            wait_for(lambda: any(t.child_frame_id == 'left_leg1' and
                abs(abs(t.transform.rotation.x) - math.sin(0.2)) < 1e-5
                for batch in transforms for t in batch.transforms))
            by_stamp = {(m.header.stamp.sec, m.header.stamp.nanosec): m for m in source}
            matched = [m for m in output if (m.header.stamp.sec, m.header.stamp.nanosec) in by_stamp]
            assert len(matched) > 10
            assert all(m == by_stamp[(m.header.stamp.sec, m.header.stamp.nanosec)] for m in matched)
            assert all(p.poll() is None for p in processes)
        except Exception:
            print(''.join(read_log(log)))
            raise
        finally:
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            node.destroy_node()
            rclpy.shutdown()


def read_log(log):
    log.seek(0)
    return log.readlines()
