#!/usr/bin/env python3
"""Advisory fall predictor: scores the body IMU and publishes a warning.

Read-only with respect to Safety: it subscribes to the body IMU and the latched
estop, never publishes on the estop topic, never calls services and never
commands motors. Consumers may use /safety/fall_warning to prepare (log, slow
down, brace); the deterministic safety_node remains the only estop authority.

The model was trained in simulation only (old RL tracking-policy path) and its
scores are uncalibrated. Run it in shadow first (log_path) to collect real data.

Outputs:
  warning_topic     std_msgs/Bool, latched; True while warning_head is in alarm.
  prediction_topic  std_msgs/String JSON per scored sample ('' disables).
  log_path          optional JSONL in the evaluation shadow-log format.
"""
import hashlib
import json
import math
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

from .core import FallPredictor, OnnxScorer, imu_features, load_spec

DEFAULT_SPEC = os.path.join('models', 'fall_predictor', 'dual_v2_trial03.json')


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def _head_key(name):
    """'600ms' -> '600' for the shadow-log field names p600/alarm600."""
    return name[:-2] if name.endswith('ms') else name


class FallPredictorNode(Node):
    def __init__(self):
        super().__init__('fall_predictor')
        p = {name: self.declare_parameter(name, default).value for name, default in (
            ('model_spec', ''),
            ('imu_topic', '/imu/data_raw'),
            ('estop_topic', '/safety/estop'),
            ('warning_topic', '/safety/fall_warning'),
            ('prediction_topic', '/safety/fall_prediction'),
            ('warning_head', '600ms'),
            ('log_path', ''),
        )}
        spec_path = p['model_spec'] or os.path.join(
            get_package_share_directory('humanoid_safety'), DEFAULT_SPEC)
        self.spec = load_spec(spec_path)
        if self.spec.model_sha256 and _sha256(self.spec.model_path) != self.spec.model_sha256:
            raise RuntimeError(f'{self.spec.model_path} does not match model_sha256 in {spec_path}')
        self.warning_head = self.spec.head(p['warning_head']).name
        self.predictor = FallPredictor(self.spec, OnnxScorer(self.spec))
        self.estop = False
        self.warning = None
        self.invalid_samples = 0
        self.log_file = open(p['log_path'], 'a', buffering=1) if p['log_path'] else None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.warning_pub = self.create_publisher(Bool, p['warning_topic'], latched)
        self.prediction_pub = (self.create_publisher(String, p['prediction_topic'], 10)
                               if p['prediction_topic'] else None)
        self.create_subscription(Imu, p['imu_topic'], self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Bool, p['estop_topic'], self.on_estop, latched)
        self.publish_warning(False)
        heads = {h.name: (h.threshold, h.confirm_frames) for h in self.spec.heads}
        self.log(kind='start', model=self.spec.name, policy=heads, warning_head=self.warning_head)
        self.get_logger().info(
            f'Advisory fall predictor {self.spec.name}: warning from head {self.warning_head} '
            f'on {p["warning_topic"]}; heads (threshold, confirm frames) {heads}. '
            'Never trips Safety.')

    def log(self, **row):
        if self.log_file is None:
            return
        row['recv_mono'] = time.monotonic()
        row['recv_wall_ns'] = time.time_ns()
        self.log_file.write(json.dumps(row) + '\n')

    def publish_warning(self, value):
        if value != self.warning:
            self.warning = value
            self.warning_pub.publish(Bool(data=value))

    def on_estop(self, msg):
        if bool(msg.data) != self.estop:
            self.estop = bool(msg.data)
            self.log(kind='estop', value=self.estop)

    def on_imu(self, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        q = msg.orientation
        quat = (q.w, q.x, q.y, q.z)
        g, a = msg.angular_velocity, msg.linear_acceleration
        features = imu_features(quat, (g.x, g.y, g.z), (a.x, a.y, a.z))
        norm = math.sqrt(sum(v * v for v in quat))
        if not all(map(math.isfinite, features)) or abs(norm - 1.0) > 0.1:
            # No usable orientation (or NaN): never score across it.
            self.invalid_samples += 1
            self.predictor.reset()
            self.publish_warning(False)
            self.get_logger().warn('Invalid IMU sample (orientation or non-finite values); '
                                   'history reset', throttle_duration_sec=5.0)
            return
        resets = self.predictor.resets
        t0 = time.perf_counter()
        try:
            result = self.predictor.update(stamp, features, self.estop)
        except Exception as exc:  # advisory: report, never take the process down
            self.predictor.reset()
            self.publish_warning(False)
            self.get_logger().error(f'Fall predictor inference failed: {exc}',
                                    throttle_duration_sec=5.0)
            return
        infer_ms = (time.perf_counter() - t0) * 1e3
        if self.predictor.resets != resets:
            self.log(kind='reset', stamp_ns=stamp)
        if result is None:
            self.publish_warning(False)
            return
        self.publish_warning(result.alarms[self.warning_head])
        row = dict(stamp_ns=stamp, f=features, eligible=result.eligible, infer_ms=round(infer_ms, 4))
        for name, score in result.scores.items():
            row['p' + _head_key(name)] = score
            row['alarm' + _head_key(name)] = result.alarms[name]
        self.log(kind='score', **row)
        if self.prediction_pub is not None:
            self.prediction_pub.publish(String(data=json.dumps({
                'stamp_ns': stamp, 'scores': result.scores, 'alarms': result.alarms,
                'eligible': result.eligible, 'warning': result.alarms[self.warning_head]})))
        for name in result.onsets:
            self.log(kind='alarm', head=name, stamp_ns=stamp, p=result.scores[name])
            self.get_logger().warn(f'Advisory fall warning ({name}), score={result.scores[name]:.3f}')

    def destroy_node(self):
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None
        return super().destroy_node()


def main():
    rclpy.init()
    node = FallPredictorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            # ros2 launch can deliver a second SIGINT while cleanup is in progress.
            pass


if __name__ == '__main__':
    main()
