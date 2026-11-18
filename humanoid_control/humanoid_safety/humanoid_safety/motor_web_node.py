#!/usr/bin/env python3
"""HTTP view of DDS feedback with bounded control_core service admission.

ROS callbacks update a bounded per-joint cache. HTTP runs separately and never
waits for ROS messages. Source and local receipt ages both determine freshness.
"""
import json
import math
import mimetypes
import queue
import uuid
from urllib.parse import urlsplit
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
from motor_control_interfaces.srv import SubmitAbstractCommand
from motor_control_interfaces.msg import AbstractCommand, AbstractCommandStatus
from rclpy.duration import Duration
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
                    params.get('default_can_interface', 'can1')))
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


class WebActionBridge:
    """Bounded HTTP-to-ROS queue; only the executor touches the ROS client."""
    def __init__(self, client, clock):
        self.client, self.clock = client, clock
        self.lock = threading.Lock()
        self.pending = queue.Queue(maxsize=1)
        self.available = False
        self.result = None
        self.active = False
        self.deadline = None
        self.future = None
        self.current_id = None
        self.batch = []
        self.batch_action = None
        self.all_home = False

    def snapshot(self):
        with self.lock:
            return dict(connected=self.available, home_supported=True,
                        zero_supported=True, busy=self.active,
                        result=None if self.result is None else json.loads(json.dumps(self.result)))

    def submit(self, body, status):
        if (not isinstance(body, dict) or set(body) != {'action', 'joint_name'}
                or body.get('action') not in ('zero', 'home', 'enable', 'disable') or not isinstance(body.get('joint_name'), str)):
            raise ValueError('Expected action=zero, home, enable or disable and joint_name')
        all_motors = body['joint_name'] == '*'
        selected = status['motors'] if all_motors else [m for m in status['motors'] if m['joint_name'] == body['joint_name']]
        if not selected or len(selected) > 256:
            raise ValueError('Unknown joint or empty/oversized motor list')
        disabling = body['action'] == 'disable'
        if not disabling and (not all(m['online'] for m in selected) or status['estop'] is not False):
            raise ValueError('Fresh feedback for every selected joint and a live, released safety estop are required')
        with self.lock:
            if not self.available or (self.active and not disabling):
                raise ValueError('Control service unavailable or an action is already pending')
            if disabling:
                try:
                    self.pending.get_nowait()
                except queue.Empty:
                    pass
                if self.future is not None and not self.future.done():
                    self.client.remove_pending_request(self.future)
                self.deadline = None
            request_id = str(uuid.uuid4())
            self.result = dict(request_id=request_id, joint_name=body['joint_name'],
                               state='queued', message='Queued for ROS admission; not completed',
                               total=len(selected), completed=0, results=[])
            self.active = True
            self.batch = [m['joint_name'] for m in selected]
            self.batch_action = body['action']
            self.all_home = all_motors and body['action'] in ('home', 'enable', 'disable')
            self._queue_next()
            return dict(self.result)

    def _queue_next(self):
        index = self.result['completed']
        self.current_id = self.result['request_id'] if self.result['total'] == 1 or self.all_home else f"{self.result['request_id']}:{index}"
        joint = '*' if self.all_home else self.batch[index]
        self.result.update(state='queued', current_joint=joint)
        self.pending.put_nowait((self.current_id, joint, self.batch_action))

    def _terminal(self, state, message):
        names = self.batch if self.all_home else [self.result['current_joint']]
        self.result['results'].extend(dict(joint_name=name, state=state, message=message) for name in names)
        self.result.update(state=state, message=message)
        self.deadline = None
        if state == 'completed':
            self.result['completed'] += self.result['total'] if self.all_home else 1
            if self.result['completed'] < self.result['total']:
                self._queue_next()
                return
        self.active = False

    def tick(self):
        with self.lock:
            self.available = self.client.service_is_ready()
            if self.active and self.deadline is not None and time.monotonic() > self.deadline:
                if self.future is not None:
                    self.client.remove_pending_request(self.future)
                self.result.update(state='unknown', message='Service response timed out; execution outcome unknown. Do not automatically retry.')
                # Keep admission blocked: timeout is not evidence that ROS did not execute.
                self.deadline = None
            try:
                request_id, joint, action = self.pending.get_nowait()
            except queue.Empty:
                return
            request = SubmitAbstractCommand.Request()
            request.command.request_id, request.command.joint_name = request_id, joint
            request.command.operation = {'home': AbstractCommand.HOME, 'zero': AbstractCommand.SET_MECHANICAL_ZERO,
                                         'enable': AbstractCommand.ENABLE, 'disable': AbstractCommand.DISABLE}[action]
            now = self.clock.now()
            request.command.header.stamp = now.to_msg()
            request.command.valid_until = (now + Duration(seconds=0.5)).to_msg()
            try:
                self.future = self.client.call_async(request)
                self.deadline = time.monotonic() + 2.0
            except Exception as error:
                self.result.update(state='failed', message=str(error))
                self.active = False
                return
        self.future.add_done_callback(lambda future, ident=request_id: self.on_response(future, ident))

    def on_response(self, future, request_id=None):
        with self.lock:
            if request_id is not None and request_id != self.current_id:
                return
            try:
                response = future.result()
                if self.result['state'] in ('completed', 'failed', 'cancelled', 'rejected'):
                    return
                self.result.update(state='accepted' if response.accepted else 'rejected',
                                   message=response.message, stage=response.status.stage)
                if not response.accepted:
                    self._terminal('rejected', response.message)
                if response.accepted:
                    self.result['message'] += ' (admitted; completion not yet verified)'
            except Exception as error:
                self.result.update(state='unknown', message=str(error))
                # An uncertain response must not permit duplicate execution.
            self.deadline = None

    def on_status(self, msg):
        with self.lock:
            if self.result is None or msg.request_id != self.current_id:
                return
            if self.result['state'] in ('completed', 'failed', 'cancelled', 'rejected'):
                return
            self.result.update(stage=msg.stage, message=msg.message)
            if msg.stage in (AbstractCommandStatus.COMPLETED, AbstractCommandStatus.FAILED,
                             AbstractCommandStatus.CANCELLED, AbstractCommandStatus.REJECTED):
                self._terminal({6: 'completed', 7: 'failed', 8: 'cancelled', 9: 'rejected'}[msg.stage], msg.message)


def make_handler(snapshot, static_root, submit=None):
    """Expose status/assets and an optional bounded same-origin action endpoint."""
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
            if submit is None:
                self.send_error(405, 'Read-only monitor')
                return
            if self.path != '/api/action':
                self.send_error(405, 'Unsupported command endpoint')
                return
            origin = self.headers.get('Origin')
            if ((origin and urlsplit(origin).netloc != self.headers.get('Host'))
                    or self.headers.get('Sec-Fetch-Site') == 'cross-site'):
                self.send_error(403, 'Same-origin requests required')
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4096 or self.headers.get_content_type() != 'application/json':
                    raise ValueError('Expected a JSON body of at most 4096 bytes')
                result = submit(json.loads(self.rfile.read(length)))
                code = 202
            except (ValueError, TypeError, UnicodeError) as error:
                result, code = dict(error=str(error)), 400
            payload = json.dumps(result, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(payload)

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
        self.action_client = self.create_client(SubmitAbstractCommand, '/control_core/submit_abstract_command')
        self.actions = WebActionBridge(self.action_client, self.get_clock())
        self.create_timer(0.05, self.actions.tick)
        self.create_subscription(AbstractCommandStatus, '/control_core/abstract_command_status',
                                 self.actions.on_status, qos)
        root = Path(get_package_share_directory('humanoid_safety')) / 'web_ui'
        self.server = MonitorHTTPServer((str(param('host')), int(param('port'))),
                                       make_handler(self.snapshot, root, lambda body: self.actions.submit(body, self.snapshot())))
        self.http_thread = threading.Thread(target=self.server.serve_forever,
                                            name='motor-status-http', daemon=True)
        self.http_thread.start()
        self.get_logger().info(f'Motor UI with ROS admission: http://{param("host")}:{self.server.server_port}; '
                               f'DDS source={self.source_topic}, domain={os.getenv("ROS_DOMAIN_ID", "0")}')

    def on_state(self, msg):
        self.cache.observe(msg, time.monotonic(), self.get_clock().now().nanoseconds * 1e-9)

    def on_estop(self, msg):
        self.cache.observe_estop(msg.data, time.monotonic())

    def snapshot(self):
        self.live_config.refresh(time.monotonic())
        result = self.cache.snapshot(time.monotonic(), self.get_clock().now().nanoseconds * 1e-9)
        return dict(result, domain_id=os.getenv('ROS_DOMAIN_ID', '0'), source_topic=self.source_topic,
                    config=self.live_config.snapshot(), control=self.actions.snapshot())

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
