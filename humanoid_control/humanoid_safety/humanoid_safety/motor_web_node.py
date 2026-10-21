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
from diagnostic_msgs.msg import DiagnosticArray
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
        self.estop_received = None
        self.supervisor = None

    def replace_registry(self, registry):
        """Swap display configuration atomically; never reuse a changed CAN identity."""
        with self.lock:
            for name in list(self.samples.keys() | self.temperatures.keys()):
                before, after = self.registry.get(name), registry.get(name)
                if after is None or before is None or any(before.get(k) != after.get(k)
                        for k in ('can_interface', 'motor_id', 'model', 'direction',
                                  'encoder_offset_rad', 'ankle_mapping')):
                    self.samples.pop(name, None)
                    self.temperatures.pop(name, None)
            self.registry = registry

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

    def observe_temperatures(self, text, now, ros_now):
        """Accept v1 named telemetry only; legacy enumeration has no safe identity.

        Identity is joined by joint name + CAN interface/ID + model, independent
        of registry order. Replayed stamps never refresh local receipt freshness.
        """
        try:
            with self.lock:
                registry = self.registry
            values = json.loads(text)
            if (not isinstance(values, dict) or values.get('schema_version') != 1
                    or not isinstance(values.get('motors'), list)
                    or len(values['motors']) > 256):
                return False
            updates = {}
            for record in values['motors']:
                name = record['joint_name']
                if not isinstance(name, str) or name in updates:
                    return False
                cfg = registry.get(name)
                if cfg is not None and any(record.get(key) != cfg.get(key)
                        for key in ('can_interface', 'motor_id', 'model')):
                    continue
                if registry and cfg is None:
                    continue
                if (not isinstance(record.get('can_interface'), str)
                        or type(record.get('motor_id')) is not int):
                    continue
                value, stamp = record.get('temperature_c'), record.get('source_stamp')
                if (type(value) not in (int, float) or not math.isfinite(value)
                        or type(stamp) not in (int, float) or not math.isfinite(stamp)
                        or stamp <= 0 or not -0.01 <= ros_now - stamp <= self.timeout):
                    continue
                updates[name] = (now, stamp, float(value), record)
        except (ValueError, TypeError, KeyError, AttributeError):
            return False
        with self.lock:
            for name, sample in updates.items():
                cfg = self.registry.get(name)
                if self.registry and (cfg is None or any(sample[3].get(k) != cfg.get(k)
                        for k in ('can_interface', 'motor_id', 'model'))):
                    continue
                if name not in self.temperatures and len(self.temperatures) >= 256:
                    continue
                previous = self.temperatures.get(name)
                if previous is None or sample[1] > previous[1]:
                    self.temperatures[name] = sample
        return True

    def observe_estop(self, value, now):
        with self.lock:
            self.estop, self.estop_received = bool(value), now

    def observe_supervisor(self, msg, now, ros_now):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        summaries = [s for s in msg.status if s.name == 'humanoid_safety/supervisor'
                     and s.hardware_id == 'humanoid']
        if (len(summaries) != 1 or stamp <= 0
                or not -0.01 <= ros_now - stamp <= self.timeout):
            return False
        summary = summaries[0]
        with self.lock:
            if self.supervisor is not None and stamp <= self.supervisor[1]:
                return False
            level = summary.level[0] if isinstance(summary.level, bytes) else int(summary.level)
            self.supervisor = (now, stamp, level, summary.message)
        return True

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
                if not self.registry and temp:
                    cfg = temp[3]
                motors.append(dict(joint_name=name, motor_id=cfg.get('motor_id'),
                    model=cfg.get('model'), can_interface=cfg.get('can_interface'), online=online,
                    source_age_ms=None if age is None else age * 1000,
                    position_rad=None if sample is None else sample[2],
                    velocity_radps=None if sample is None else sample[3],
                    effort_nm=None if sample is None else sample[4],
                    temperature_c=(temp[2] if temp and online and
                                   0 <= monotonic_now - temp[0] <= self.timeout and
                                   -0.01 <= ros_now - temp[1] <= self.timeout else None),
                    temperature_source_age_ms=None if temp is None else (ros_now - temp[1]) * 1000))
            supervisor = self.supervisor
            supervisor_online = (supervisor is not None and
                0 <= monotonic_now - supervisor[0] <= self.timeout and
                -0.01 <= ros_now - supervisor[1] <= self.timeout)
            estop_fresh = (self.estop_received is not None and
                0 <= monotonic_now - self.estop_received <= self.timeout)
            return dict(connected=any(m['online'] for m in motors),
                        estop=self.estop if supervisor_online and estop_fresh else None,
                        last_reported_estop=self.estop, supervisor_online=supervisor_online,
                        supervisor_level=supervisor[2] if supervisor_online else None,
                        supervisor_message=supervisor[3] if supervisor_online else None,
                        motors=motors)


class LiveMotorConfig:
    """Reload the selected local YAML at most once per second, for display only.

    Invalid/partially written files keep the last valid table and expose an error.
    This does not reconfigure the running motor backend or prove calibration.
    """
    def __init__(self, path, cache):
        self.path = Path(path).expanduser().resolve() if path else None
        self.cache = cache
        self.checked_at = None
        self.signature = None
        self.parameters = None
        self.loaded_at = None
        self.error = None

    def refresh(self, now):
        if self.checked_at is not None and now - self.checked_at < 1.0:
            return
        self.checked_at = now
        if self.path is None:
            return
        try:
            stat = self.path.stat()
            signature = (stat.st_mtime_ns, stat.st_ctime_ns, stat.st_size, stat.st_ino)
            if signature == self.signature and self.error is None:
                return
            if stat.st_size > 1024 * 1024:
                raise ValueError('Motor config exceeds 1 MiB')
            params = yaml.safe_load(self.path.read_text())['motor_control_node']['ros__parameters']
            motors = params['motors']
            if not isinstance(motors, dict) or not 1 <= len(motors) <= 256:
                raise ValueError('Expected 1–256 configured motors')
            registry = {}
            for name, cfg in motors.items():
                if not isinstance(name, str) or not name or not isinstance(cfg, dict):
                    raise ValueError('Invalid joint name or motor entry')
                entry = dict(cfg, can_interface=cfg.get('can_interface',
                    params.get('default_can_interface', 'can0')))
                if (type(entry.get('motor_id')) is not int or
                        not isinstance(entry.get('can_interface'), str) or
                        not isinstance(entry.get('model'), str)):
                    raise ValueError(f'Invalid CAN identity for {name}')
                registry[name] = entry
            json.dumps(params, allow_nan=False)  # Reject non-JSON and non-finite YAML values.
            self.cache.replace_registry(registry)
            self.parameters = params
            self.signature = signature
            self.loaded_at = time.time()
            self.error = None
        except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
            self.error = str(error)

    def snapshot(self):
        return dict(path=None if self.path is None else str(self.path),
                    loaded_at=self.loaded_at, error=self.error, parameters=self.parameters,
                    reload_interval_s=1.0)


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
        defaults = dict(source_topic='/joint_states', motor_status_topic='/motor_telemetry',
                        supervisor_status_topic='/safety/status',
                        motor_config_file=default_registry, host='127.0.0.1', port=8080,
                        state_timeout_s=0.5)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        param = lambda key: self.get_parameter(key).value
        timeout = float(param('state_timeout_s'))
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('state_timeout_s must be finite and positive')
        self.cache = MotorStatusCache(timeout=timeout)
        self.live_config = LiveMotorConfig(str(param('motor_config_file')), self.cache)
        self.live_config.refresh(time.monotonic())
        self.source_topic = str(param('source_topic'))
        self.create_subscription(JointState, self.source_topic, self.on_state, qos_profile_sensor_data)
        self.create_subscription(String, str(param('motor_status_topic')),
            lambda msg: self.cache.observe_temperatures(msg.data, time.monotonic(),
                self.get_clock().now().nanoseconds * 1e-9), qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/safety/estop', self.on_estop, qos)
        self.create_subscription(DiagnosticArray, str(param('supervisor_status_topic')),
            lambda msg: self.cache.observe_supervisor(msg, time.monotonic(),
                self.get_clock().now().nanoseconds * 1e-9), qos_profile_sensor_data)
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
        self.cache.observe_estop(msg.data, time.monotonic())

    def snapshot(self):
        self.live_config.refresh(time.monotonic())
        result = self.cache.snapshot(time.monotonic(), self.get_clock().now().nanoseconds * 1e-9)
        return dict(result, domain_id=os.getenv('ROS_DOMAIN_ID', '0'), source_topic=self.source_topic,
                    config=self.live_config.snapshot())

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
