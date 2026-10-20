"""Isolated DDS -> safety Web/TF integration with synthetic feedback only."""
import json
import math
import os
import signal
import socket
import subprocess
import tempfile
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


@pytest.mark.skipif(os.environ.get('RUN_ROS_INTEGRATION') != '1',
                    reason='Opt-in isolated DDS and localhost HTTP sockets required')
def test_safety_monitor_web_and_tf():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use an isolated non-default domain'
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    rclpy.init()
    node = rclpy.create_node('safety_monitor_probe')
    relay, transforms = [], []
    node.create_subscription(JointState, '/monitor_probe/display', relay.append, qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf', transforms.append, qos_profile_sensor_data)
    publisher = node.create_publisher(JointState, '/monitor_probe/feedback', qos_profile_sensor_data)
    names = [f'{side}_{joint}_joint' for side in ('left', 'right')
             for joint in ('hip1', 'hip2', 'thigh', 'knee', 'ankle')]
    msg = JointState(name=names, position=[.4] + [0.] * 9,
                     velocity=[0.] * 10, effort=[0.] * 10)
    def feedback():
        msg.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(msg)
    timer = node.create_timer(.02, feedback)
    url = f'http://127.0.0.1:{port}'
    def status():
        with urlopen(url + '/api/status', timeout=.5) as response:
            return json.load(response)
    process = None
    with tempfile.TemporaryFile(mode='w+') as log:
        try:
            process = subprocess.Popen(['ros2', 'launch', 'humanoid_safety', 'safety.launch.py',
                'mode:=monitor', 'use_rviz:=false', f'port:={port}',
                'source_topic:=/monitor_probe/feedback', 'display_topic:=/monitor_probe/display'],
                stdout=log, stderr=log, start_new_session=True)
            deadline = time.monotonic() + 15
            data = None
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
                try:
                    data = status()
                except (URLError, TimeoutError):
                    continue
                moved_tf = any(t.child_frame_id == 'left_leg1' and
                    abs(abs(t.transform.rotation.x) - math.sin(.2)) < 1e-5
                    for batch in transforms for t in batch.transforms)
                if data['connected'] and relay and moved_tf:
                    break
            else:
                raise AssertionError('Timed out waiting for Web, safety-owned relay and TF')
            assert data['domain_id'] == os.environ['ROS_DOMAIN_ID']
            assert data['source_topic'] == '/monitor_probe/feedback'
            assert data['motors'][0]['position_rad'] == .4
            assert relay[-1].position[0] == .4
            assert data['estop'] is None  # monitor mode must not invent/start a supervisor
            assert node.count_publishers('/motor_commands') == 0
            with urlopen(url + '/', timeout=1) as response:
                assert b'Motor Web UI' in response.read()
            try:
                urlopen(Request(url + '/api/command', data=b'{}', method='POST'), timeout=1)
            except HTTPError as error:
                assert error.code == 405
            else:
                raise AssertionError('Command endpoint must be unavailable')
            timer.cancel()
            deadline = time.monotonic() + .8
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
            assert not status()['connected'], 'HTTP polling must not renew stale feedback'
            assert process.poll() is None
        except Exception:
            log.seek(0)
            print(log.read())
            raise
        finally:
            if process and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=10)
            node.destroy_node()
            rclpy.shutdown()
