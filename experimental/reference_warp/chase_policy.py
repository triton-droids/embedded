"""Experimental chase consumer using the unchanged tracking-policy worker.

Only reference observations are warped. Parent decoding, inference, expiry,
motor messages and gains are preserved. Turning is deliberately not synthesized.
Loss/zero/turn demand after arming latches a fault and stops refreshing commands;
the existing C++/backend expiry path stops the simulation. This is not a
balanced stand transition and requires restarting the experiment to resume.
"""
import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

from motor_control_hybrid.tracking_policy_node import TrackingPolicyNode
from prepare import warp


def request_reason(command, now_ros, now_wall, sim_receipt):
    """Strict stamped planar-forward request validation, independent of ROS."""
    if sim_receipt is None or not 0 <= now_wall-sim_receipt <= 1.5:
        return 'waiting_for_fresh_simulator'
    if command is None:
        return 'waiting_for_chase'
    receipt, stamp, frame, values = command
    if frame != 'base_link' or not all(math.isfinite(v) for v in (*values, stamp, receipt)):
        return 'invalid_command'
    if not -0.01 <= now_ros-stamp <= 0.25 or not 0 <= now_wall-receipt <= 0.25:
        return 'stale_chase_command'
    vx, vy, vz, wx, wy, wz = values
    if not (0 <= vx <= 0.200001 and abs(wz) <= 0.500001
            and all(abs(v) <= 1e-8 for v in (vy, vz, wx, wy))):
        return 'unsupported_command'
    if vx <= 0.001:
        return 'zero_request_target_lost_or_close'
    if abs(wz) > 0.25:
        return 'turn_required_not_implemented'
    return None


def dynamic_reference(q, dq, names, scale, scale_rate):
    out_q, out_dq = warp(q, dq, names, scale)
    t = np.minimum(np.arange(len(q), dtype=np.float64)/50, 1)
    blend = 10*t**3-15*t**4+6*t**5
    selected = [i for i, n in enumerate(names)
                if n.endswith(('_hip1_joint', '_knee_joint', '_ankle_joint'))]
    out_dq[:, selected] += scale_rate*blend[:, None]*(q[:, selected]-q[0, selected])
    return out_q, out_dq


class ChaseTrackingPolicy(TrackingPolicyNode):
    def __init__(self):
        # The parent starts its worker during construction; defer it until the
        # experimental state/subscriptions are fully initialized.
        self._experiment_ready = threading.Event()
        super().__init__()
        if not self.wait_for_sim_release or self.feedback_mode != 'joint_states':
            raise ValueError('Experimental chase requires simulation-release gating and joint feedback')
        self.declare_parameter('experiment_max_walk_s', 8.0)
        self.max_walk_s = float(self.get_parameter('experiment_max_walk_s').value)
        if not math.isfinite(self.max_walk_s) or not 0 < self.max_walk_s <= 30:
            raise ValueError('experiment_max_walk_s must be in (0, 30]')
        self.original_q = self.policy.reference_q.copy()
        self.original_dq = self.policy.reference_dq.copy()
        self.chase_command = None
        self.sim_receipt = None
        self.sim_fault = None
        self.armed = False
        self.scale = 1.0
        self.scale_rate = 0.0
        self.requested_scale = 1.0
        self.last_warp_tick = None
        self.chase_reason = 'waiting_for_chase'
        self.experiment_pub = self.create_publisher(String, '/experimental/chase_status', 2)
        self.create_subscription(TwistStamped, '/ball_chase/cmd_vel', self.on_chase, 2)
        self.create_timer(0.1, self.publish_experiment)
        self._experiment_ready.set()

    def run_worker(self):
        while not self._experiment_ready.wait(0.05):
            if self.stopping.is_set():
                return
        super().run_worker()

    def latch_fault(self, reason, details=None):
        """Preserve the initiating fault when expiry/estop follows a chase stop."""
        with self.state_lock:
            if self.fault is None:
                super().latch_fault(reason, details)

    def on_sim_status(self, msg):
        super().on_sim_status(msg)
        try:
            value = json.loads(msg.data)
            if (not isinstance(value.get('base_held'), bool)
                    or not math.isfinite(value['sim_time_s'])):
                return
        except (KeyError, TypeError, ValueError):
            return
        with self.state_lock:
            self.sim_receipt = time.monotonic()
            self.sim_fault = value.get('control_fault') or ('simulator_estop' if value.get('estop_active') else None)

    def on_chase(self, msg):
        t = msg.twist
        values = (t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z)
        with self.state_lock:
            self.chase_command = (time.monotonic(), self.stamp_seconds(msg), msg.header.frame_id, values)
            reason = request_reason(self.chase_command, self.ros_seconds(), time.monotonic(), self.sim_receipt)
            if self.armed and reason and self.fault is None:
                # Also invalidates an inference already in flight: the parent
                # rechecks fault before publishing its result.
                self.chase_reason = reason
                self.latch_fault('Experimental chase stopped: '+reason)

    def tick(self):
        with self.state_lock:
            if self.fault or self.stopping.is_set():
                return  # Preserve the initiating reason after downstream expiry/estop.
            now = time.monotonic()
            reason = self.sim_fault or request_reason(
                self.chase_command, self.ros_seconds(), now, self.sim_receipt)
            if self.reference_started is not None and now-self.reference_started >= self.max_walk_s:
                reason = 'experiment_walk_time_limit'
            if reason:
                self.chase_reason = reason
                if self.armed:
                    self.latch_fault('Experimental chase stopped: '+reason)
                return
            self.chase_reason = 'forward_reference_warp'
            vx = self.chase_command[3][0]
            self.requested_scale = 0.95+0.10*min(1.0, vx/0.2)
            dt = 0.02 if self.last_warp_tick is None else min(0.04, max(1e-6, now-self.last_warp_tick))
            self.last_warp_tick = now
            change = float(np.clip(self.requested_scale-self.scale, -0.1*dt, 0.1*dt))
            self.scale += change
            self.scale_rate = change/dt
            q, dq = dynamic_reference(self.original_q, self.original_dq, self.policy.names,
                                      self.scale, self.scale_rate)
            if np.any(q < self.policy.target_limits[0]) or np.any(q > self.policy.target_limits[1]):
                self.latch_fault('Experimental reference exceeds original joint limits')
                return
            self.policy.reference_q, self.policy.reference_dq = q, dq
            count_before = self.count
        super().tick()
        with self.state_lock:
            # Startup discovery/calibration may have no usable input yet.
            # A transient chase loss must not latch a never-commanded robot.
            if self.count > count_before:
                self.armed = True
                reason = request_reason(self.chase_command, self.ros_seconds(),
                                        time.monotonic(), self.sim_receipt)
                if reason:
                    self.chase_reason = reason
                    self.latch_fault('Experimental chase stopped: '+reason)

    def publish_experiment(self):
        with self.state_lock:
            message = String(data=json.dumps(dict(
                armed=self.armed, reason=self.chase_reason, fault=self.fault,
                reference_scale=self.scale, requested_scale=self.requested_scale,
                scale_rate_per_s=self.scale_rate, turning_supported=False,
                velocity_calibrated=False, hardware_output=False,
                command=None if self.chase_command is None else self.chase_command[3])))
        try:
            self.experiment_pub.publish(message)
        except RCLError as error:
            self.get_logger().error(f'Experiment diagnostics publication failed: {error}', throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = ChaseTrackingPolicy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
