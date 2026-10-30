"""Unified fake control/safety/Web/model launch; no physical CAN or GUI needed."""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
from urllib.error import URLError
from urllib.request import urlopen

import pytest
import rclpy
from motor_control_interfaces.msg import AbstractCommand, MotorCommand
from motor_control_interfaces.srv import SubmitAbstractCommand
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from tf2_msgs.msg import TFMessage


@pytest.mark.skipif(os.environ.get('RUN_ROS_INTEGRATION') != '1', reason='Opt-in DDS/HTTP test')
def test_unified_fake_control_web_safety_and_tf():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0')
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    rclpy.init()
    node = rclpy.create_node('unified_control_probe')
    displayed, transforms, commands = [], [], []
    node.create_subscription(JointState, '/safety/visualization/joint_states', displayed.append, qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf', transforms.append, qos_profile_sensor_data)
    node.create_subscription(MotorCommand, '/motor_commands', commands.append, 10)
    # Match the body micro-ROS default publisher and the safety subscriber's
    # reliable QoS; a best-effort fixture cannot satisfy that subscription.
    imu_pub = node.create_publisher(Imu, '/imu/data_raw', 10)
    desired_pub = node.create_publisher(MotorCommand, '/desired_motor_subset', 10)
    client = node.create_client(SubmitAbstractCommand, '/control_core/submit_abstract_command')
    mode = [None]
    def inputs():
        imu = Imu(); imu.header.stamp = node.get_clock().now().to_msg(); imu.orientation.w = 1.
        imu_pub.publish(imu)
        if mode[0] is not None:
            command = MotorCommand(joint_name=['left_hip1_joint'], mode=[mode[0]], position=[.4])
            command.header.stamp = node.get_clock().now().to_msg()
            desired_pub.publish(command)
    timer = node.create_timer(.02, inputs)
    url = f'http://127.0.0.1:{port}/api/status'
    def status():
        try:
            with urlopen(url, timeout=.5) as stream:
                return json.load(stream)
        except (URLError, TimeoutError):
            return None
    def wait(predicate, timeout=18):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return
        raise AssertionError('Timed out waiting for unified stack')
    process = None
    with_rviz = os.environ.get('CONTROL_STACK_TEST_RVIZ') == '1'
    with tempfile.TemporaryFile(mode='w+') as log:
        try:
            process = subprocess.Popen(['ros2', 'launch', 'motor_control_hybrid', 'control_test.launch.py',
                'enable_fake_motor:=true', f'use_rviz:={str(with_rviz).lower()}', f'port:={port}'],
                stdout=log, stderr=log, start_new_session=True)
            wait(lambda: (data := status()) and data['connected'] and data['estop'] is False
                 and displayed and transforms and client.service_is_ready())
            data = status()
            assert len(data['motors']) == 10
            assert len(data['config']['parameters']['motors']) == 10
            assert Path(data['config']['path']).is_file()
            expected_nodes = ('safety_node', 'motor_web_node', 'joint_state_monitor_node',
                              'robot_state_publisher', 'cpp_control_node', 'fake_motor_node')
            wait(lambda: all(name in [n for n, _ in node.get_node_names_and_namespaces()]
                             for name in expected_nodes))
            if with_rviz:
                wait(lambda: any(name.startswith('rviz') for name, _ in node.get_node_names_and_namespaces()))
            nodes = [name for name, _ in node.get_node_names_and_namespaces()]
            for expected in expected_nodes:
                assert nodes.count(expected) == 1, nodes
            assert 'python_can_node' not in nodes  # Mutually exclusive fake mode.
            assert 'joint_state_publisher_gui' not in nodes
            assert not any(MotorCommand.MODE_ENABLE in command.mode for command in commands)
            request = SubmitAbstractCommand.Request()
            request.command = AbstractCommand(request_id='probe-zero', joint_name='left_hip1_joint',
                                              operation=AbstractCommand.SET_MECHANICAL_ZERO)
            request.command.header.stamp = node.get_clock().now().to_msg()
            future = client.call_async(request)
            wait(future.done)
            assert not future.result().accepted
            assert 'disabled' in future.result().message
            assert future.result().status.request_id == 'probe-zero'
            # Enable only the simulated joint, then drive the complete path via core.
            mode[0] = MotorCommand.MODE_ENABLE
            wait(lambda: any(MotorCommand.MODE_ENABLE in command.mode for command in commands))
            mode[0] = MotorCommand.MODE_MOTION
            wait(lambda: (data := status()) and abs(data['motors'][0]['position_rad'] - .4) < 1e-5)
            wait(lambda: any(t.child_frame_id == 'left_leg1' and
                 abs(abs(t.transform.rotation.x) - math.sin(.2)) < 1e-5
                 for batch in transforms for t in batch.transforms))
            assert displayed[-1].name[0] == 'left_hip1_joint'
            assert len(displayed[-1].position) == 10
            assert data['domain_id'] == os.environ['ROS_DOMAIN_ID']
            assert process.poll() is None
        except Exception:
            log.seek(0); print(log.read()[-14000:]); raise
        finally:
            mode[0] = None
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait()
            node.destroy_timer(timer); node.destroy_node(); rclpy.shutdown()
