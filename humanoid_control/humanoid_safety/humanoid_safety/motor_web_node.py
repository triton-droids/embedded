#!/usr/bin/env python3
"""Read-only HTTP view of DDS JointState feedback; no SDK or CAN ownership.

ROS callbacks update a bounded per-joint cache. HTTP runs separately and never
waits for ROS messages. Source and local receipt ages both determine freshness.
"""
import json
import math
import mimetypes
import os
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import yaml

from .joint_state_monitor_node import valid_joint_state


class MotorStatusCache:
    """Thread-safe telemetry only. Missing velocity/effort remains unknown."""
    def __init__(self, registry=None, timeout=0.5):
        self.registry = registry or {}
        self.timeout = timeout
        self.lock = threading.Lock()
        self.samples = {}
        self.temperatures = {}
        self.estop = None

    def observe(self, msg, monotonic_now, ros_now):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (not valid_joint_state(msg) or stamp <= 0
                or not -0.01 <= ros_now - stamp <= self.timeout):
            return False
        with self.lock:
            for i, name in enumerate(msg.name):
                if self.registry and name not in self.registry:
                    continue
                if name not in self.samples and len(self.samples) >= 256:
                    continue
                if name in self.samples and stamp <= self.samples[name][1]:
                    continue
                self.samples[name] = (monotonic_now, stamp, float(msg.position[i]),
                    float(msg.velocity[i]) if len(msg.velocity) else None,
                    float(msg.effort[i]) if len(msg.effort) else None)
        return True

    def observe_temperatures(self, text, now):
        """Legacy motor_status keys are registry enumeration indices, NOT IDs.

        This unstamped ancillary topic provides receipt freshness only; it never
        makes joint feedback online and cannot provide a hardware capture time.
        """
        try:
            values = json.loads(text)
            if not isinstance(values, dict):
                return
            updates = {}
            for index, name in enumerate(self.registry):
                value = values.get(str(index), {}).get('temperature')
                if isinstance(value, (float, int)) and math.isfinite(value):
                    updates[name] = (now, float(value))
        except (ValueError, TypeError, AttributeError):
            return
        with self.lock:
            self.temperatures.update(updates)

    def snapshot(self, monotonic_now, ros_now):
        with self.lock:
            motors = []
            for name in (self.registry or self.samples):
                cfg = self.registry.get(name, {})
                sample = self.samples.get(name)
                age = None if sample is None else ros_now - sample[1]
                online = (sample is not None and 0 <= monotonic_now - sample[0] <= self.timeout
                          and -0.01 <= age <= self.timeout)
                temp = self.temperatures.get(name)
                motors.append(dict(joint_name=name, motor_id=cfg.get('motor_id'),
                    model=cfg.get('model'), online=online,
                    source_age_ms=None if age is None else age * 1000,
                    position_rad=None if sample is None else sample[2],
                    velocity_radps=None if sample is None else sample[3],
                    effort_nm=None if sample is None else sample[4],
                    temperature_c=(temp[1] if temp and online and
                                   0 <= monotonic_now - temp[0] <= self.timeout else None)))
            return dict(connected=any(m['online'] for m in motors),
                        estop=self.estop, motors=motors)


def make_handler(snapshot, static_root):
    """Expose a fixed GET-only surface; filesystem paths are never user supplied."""
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/api/status':
                payload = json.dumps(snapshot(), allow_nan=False).encode()
                content_type = 'application/json'
            elif self.path in ('/', '/index.html', '/styles.css', '/app.js'):
                path = static_root / ('index.html' if self.path == '/' else self.path[1:])
                payload = path.read_bytes()
                content_type = mimetypes.guess_type(str(path))[0] or 'application/octet-stream'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', content_type + '; charset=utf-8')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(payload)

        def do_POST(self):
            self.send_error(405, 'Read-only monitor')

        def log_message(self, *args):
            pass  # Frequent dashboard polling must not flood ROS logs.
    return Handler


class MonitorHTTPServer(HTTPServer):
    def get_request(self):
        sock, address = super().get_request()
        sock.settimeout(2.0)  # A stalled browser cannot hold HTTP shutdown forever.
        return sock, address


class MotorWebNode(Node):
    def __init__(self):
        super().__init__('motor_web_node')
        try:
            default_registry = str(Path(get_package_share_directory('motor_control_hybrid')) / 'config/motors.yaml')
        except PackageNotFoundError:
            default_registry = ''
        defaults = dict(source_topic='/joint_states', motor_status_topic='/motor_status',
                        motor_config_file=default_registry, host='127.0.0.1', port=8080,
                        state_timeout_s=0.5)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        param = lambda key: self.get_parameter(key).value
        timeout = float(param('state_timeout_s'))
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('state_timeout_s must be finite and positive')
        registry = {}
        if param('motor_config_file'):
            registry = yaml.safe_load(Path(param('motor_config_file')).read_text())['motor_control_node']['ros__parameters']['motors']
        self.cache = MotorStatusCache(registry, timeout)
        self.source_topic = str(param('source_topic'))
        self.create_subscription(JointState, self.source_topic, self.on_state, qos_profile_sensor_data)
        self.create_subscription(String, str(param('motor_status_topic')),
            lambda msg: self.cache.observe_temperatures(msg.data, time.monotonic()), qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/safety/estop', self.on_estop, qos)
        root = Path(get_package_share_directory('humanoid_safety')) / 'web_ui'
        self.server = MonitorHTTPServer((str(param('host')), int(param('port'))),
                                       make_handler(self.snapshot, root))
        self.http_thread = threading.Thread(target=self.server.serve_forever,
                                            name='motor-status-http', daemon=True)
        self.http_thread.start()
        self.get_logger().info(f'Read-only motor UI: http://{param("host")}:{self.server.server_port}; '
                               f'DDS source={self.source_topic}, domain={os.getenv("ROS_DOMAIN_ID", "0")}')

    def on_state(self, msg):
        self.cache.observe(msg, time.monotonic(), self.get_clock().now().nanoseconds * 1e-9)

    def on_estop(self, msg):
        with self.cache.lock:
            self.cache.estop = bool(msg.data)

    def snapshot(self):
        result = self.cache.snapshot(time.monotonic(), self.get_clock().now().nanoseconds * 1e-9)
        return dict(result, domain_id=os.getenv('ROS_DOMAIN_ID', '0'), source_topic=self.source_topic)

    def destroy_node(self):
        self.server.shutdown()
        self.server.server_close()
        self.http_thread.join(timeout=3)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorWebNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
