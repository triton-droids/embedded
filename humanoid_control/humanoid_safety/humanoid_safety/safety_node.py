#!/usr/bin/env python3
"""ROS 2 safety supervisor with a latched emergency-stop output."""
# Reset: ros2 service call /safety/reset std_srvs/srv/Trigger "{}"
from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .checks import (
    FallDetectionCheck,
    ImuFreshnessCheck,
    ImuObservation,
    SafetyCheck,
    SafetySupervisor,
    quaternion_to_roll_pitch,
)


class SafetyNode(Node):
    """Own the safety checks, latch, diagnostics, and estop signal."""

    def __init__(self) -> None:
        super().__init__("safety_node")

        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("estop_topic", "/safety/estop")
        self.declare_parameter("status_topic", "/safety/status")
        self.declare_parameter("evaluation_rate_hz", 100.0)
        self.declare_parameter("fall_detection_enabled", True)
        self.declare_parameter("fall_roll_deg", 40.0)
        self.declare_parameter("fall_pitch_deg", 40.0)
        self.declare_parameter("fall_confirm_samples", 3)
        self.declare_parameter("reset_roll_deg", 30.0)
        self.declare_parameter("reset_pitch_deg", 30.0)
        self.declare_parameter("trip_on_imu_timeout", True)
        self.declare_parameter("imu_timeout_s", 0.25)
        self.declare_parameter("imu_future_tolerance_s", 0.01)
        self.declare_parameter("startup_grace_s", 3.0)

        imu_topic = str(self.get_parameter("imu_topic").value)
        estop_topic = str(self.get_parameter("estop_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        evaluation_rate_hz = float(self.get_parameter("evaluation_rate_hz").value)
        self._imu_timeout_s = float(self.get_parameter("imu_timeout_s").value)
        self._imu_future_tolerance_s = float(
            self.get_parameter("imu_future_tolerance_s").value
        )
        # Keep source validation bounded even when legacy timeout_s<=0 disables
        # the receipt watchdog; fall detection still requires current attitude.
        if self._imu_timeout_s <= 0.0:
            self._imu_timeout_s = 0.25
        self._last_imu_stamp_ns = None
        started_at = self._now_seconds()

        checks: list[SafetyCheck] = []
        if bool(self.get_parameter("fall_detection_enabled").value):
            checks.append(
                FallDetectionCheck(
                    roll_limit_deg=float(self.get_parameter("fall_roll_deg").value),
                    pitch_limit_deg=float(self.get_parameter("fall_pitch_deg").value),
                    reset_roll_deg=float(self.get_parameter("reset_roll_deg").value),
                    reset_pitch_deg=float(self.get_parameter("reset_pitch_deg").value),
                    confirm_samples=int(
                        self.get_parameter("fall_confirm_samples").value
                    ),
                )
            )
        checks.append(
            ImuFreshnessCheck(
                timeout_s=float(self.get_parameter("imu_timeout_s").value),
                startup_grace_s=float(
                    self.get_parameter("startup_grace_s").value
                ),
                started_at=started_at,
                enabled=bool(self.get_parameter("trip_on_imu_timeout").value),
            )
        )
        self.supervisor = SafetySupervisor(checks)

        estop_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._estop_pub = self.create_publisher(Bool, estop_topic, estop_qos)
        self._status_pub = self.create_publisher(
            DiagnosticArray, status_topic, 10
        )
        self._imu_sub = self.create_subscription(
            Imu, imu_topic, self._imu_callback, 20
        )
        self._reset_srv = self.create_service(
            Trigger, "/safety/reset", self._reset_callback
        )

        period = 1.0 / max(1.0, evaluation_rate_hz)
        # A ROS-time timer would stop evaluating when /clock pauses. Both the
        # watchdog timer and receipt/reset/grace accounting use steady time.
        self._watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            period, self._evaluate, clock=self._watchdog_clock
        )
        self._publish_estop()
        self.get_logger().info(
            f"Safety node started: imu={imu_topic}, estop={estop_topic}, "
            f"checks={[check.name for check in checks]}"
        )

    def _now_seconds(self) -> float:
        return time.monotonic()

    def _imu_callback(self, msg: Imu) -> None:
        # Source age is measured in the publisher's synchronized ROS epoch;
        # receipt age is measured independently on the host's steady clock.
        # Rejected packets never reach either safety check or renew freshness.
        received_at = self._now_seconds()
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        source_age_s = (self.get_clock().now().nanoseconds - stamp_ns) * 1.0e-9
        if (stamp_ns <= 0
                or not 0 <= msg.header.stamp.nanosec < 1_000_000_000
                or source_age_s < -self._imu_future_tolerance_s
                or source_age_s > self._imu_timeout_s
                or (self._last_imu_stamp_ns is not None
                    and stamp_ns <= self._last_imu_stamp_ns)):
            return
        self._last_imu_stamp_ns = stamp_ns
        orientation_unavailable = (
            len(msg.orientation_covariance) > 0
            and msg.orientation_covariance[0] < 0.0
        )
        roll, pitch, valid = quaternion_to_roll_pitch(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        observation = ImuObservation(
            received_at=received_at,
            source_age_s=max(0.0, source_age_s),
            roll_rad=roll,
            pitch_rad=pitch,
            valid=valid and not orientation_unavailable,
        )
        self.supervisor.observe_imu(observation)

    def _evaluate(self) -> None:
        newly_tripped = self.supervisor.evaluate(self._now_seconds())
        if newly_tripped:
            self.get_logger().fatal(
                f"SAFETY ESTOP LATCHED: {self.supervisor.reason}"
            )
        self._publish_estop()
        self._publish_status()

    def _publish_estop(self) -> None:
        msg = Bool()
        msg.data = self.supervisor.latched
        self._estop_pub.publish(msg)

    def _publish_status(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        summary = DiagnosticStatus()
        summary.name = "humanoid_safety/supervisor"
        summary.hardware_id = "humanoid"
        if self.supervisor.latched:
            summary.level = DiagnosticStatus.ERROR
            summary.message = self.supervisor.reason
        elif any(not result.healthy for result in self.supervisor.results):
            summary.level = DiagnosticStatus.WARN
            summary.message = "safety checks initializing or unhealthy"
        else:
            summary.level = DiagnosticStatus.OK
            summary.message = "all safety checks clear"
        summary.values = [
            KeyValue(key="latched", value=str(self.supervisor.latched)),
            KeyValue(key="reason", value=self.supervisor.reason),
        ]
        array.status.append(summary)

        for result in self.supervisor.results:
            status = DiagnosticStatus()
            status.name = f"humanoid_safety/{result.name}"
            status.hardware_id = "humanoid"
            if result.active:
                status.level = DiagnosticStatus.ERROR
            elif not result.healthy:
                status.level = DiagnosticStatus.WARN
            else:
                status.level = DiagnosticStatus.OK
            status.message = result.reason
            status.values = [
                KeyValue(key="active", value=str(result.active)),
                KeyValue(key="healthy", value=str(result.healthy)),
                KeyValue(key="clearable", value=str(result.clearable)),
            ]
            array.status.append(status)
        self._status_pub.publish(array)

    def _reset_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        success, message = self.supervisor.try_reset(self._now_seconds())
        response.success = success
        response.message = message
        if success:
            self.get_logger().warn(message)
        else:
            self.get_logger().error(message)
        self._publish_estop()
        self._publish_status()
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            # Publish a final fail-closed state only while the ROS context is valid.
            if rclpy.ok():
                node.supervisor.latched = True
                if not node.supervisor.reason:
                    node.supervisor.reason = "safety node shutting down"
                node._publish_estop()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            # ros2 launch can deliver a second SIGINT while cleanup is in progress.
            pass


if __name__ == "__main__":
    main()
