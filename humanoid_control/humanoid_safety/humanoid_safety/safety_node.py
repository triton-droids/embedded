#!/usr/bin/env python3
"""ROS 2 safety supervisor with a latched emergency-stop output."""
# Reset: ros2 service call /safety/reset std_srvs/srv/Trigger "{}"
from __future__ import annotations

import json
import math
import time
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .checks import (
    BatteryVoltageCheck,
    CanLinkCheck,
    CanObservation,
    FallDetectionCheck,
    ImuFreshnessCheck,
    ImuObservation,
    ImuStuckCheck,
    JointObservation,
    JointPositionLimitCheck,
    JointVelocityLimitCheck,
    MotorFaultCheck,
    MotorFeedbackCheck,
    MotorObservation,
    MotorTemperatureCheck,
    MotorTorqueCheck,
    PowerObservation,
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
        self.declare_parameter("fall_glitch_jump_deg", 20.0)
        self.declare_parameter("fall_glitch_max_gyro_rad_s", 0.5)
        self.declare_parameter("fall_glitch_max_samples", 10)
        self.declare_parameter("trip_on_imu_stuck", True)
        self.declare_parameter("imu_stuck_samples", 50)
        self.declare_parameter("trip_on_imu_timeout", True)
        self.declare_parameter("imu_timeout_s", 0.25)
        self.declare_parameter("imu_future_tolerance_s", 0.01)
        self.declare_parameter("startup_grace_s", 3.0)

        # Motor, joint, CAN and battery checks. All are off by default.
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("motor_telemetry_topic", "/motor_telemetry")
        self.declare_parameter("can_timing_topic", "/can_timing")
        self.declare_parameter("battery_topic", "/battery_state")
        self.declare_parameter("motor_config_file", "")
        self.declare_parameter("motor_temperature_enabled", False)
        self.declare_parameter("motor_max_temp_c", 80.0)
        self.declare_parameter("motor_reset_temp_c", 70.0)
        self.declare_parameter("motor_fault_enabled", False)
        self.declare_parameter("motor_torque_enabled", False)
        self.declare_parameter("motor_max_torque_nm", 60.0)
        self.declare_parameter("motor_torque_confirm_samples", 5)
        self.declare_parameter("motor_feedback_enabled", False)
        self.declare_parameter("motor_feedback_timeout_s", 0.25)
        self.declare_parameter("joint_position_enabled", False)
        self.declare_parameter("joint_position_margin_rad", 0.05)
        self.declare_parameter("joint_velocity_enabled", False)
        self.declare_parameter("joint_max_velocity_rad_s", 8.0)
        self.declare_parameter("joint_velocity_confirm_samples", 3)
        self.declare_parameter("can_link_enabled", False)
        self.declare_parameter("can_timeout_s", 3.0)
        self.declare_parameter("battery_enabled", False)
        self.declare_parameter("battery_min_voltage_v", 42.0)
        self.declare_parameter("battery_reset_voltage_v", 44.0)
        self.declare_parameter("battery_confirm_samples", 10)

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
                    glitch_jump_deg=float(
                        self.get_parameter("fall_glitch_jump_deg").value
                    ),
                    glitch_max_gyro_rad_s=float(
                        self.get_parameter("fall_glitch_max_gyro_rad_s").value
                    ),
                    glitch_max_samples=int(
                        self.get_parameter("fall_glitch_max_samples").value
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
        checks.append(
            ImuStuckCheck(
                stuck_samples=int(self.get_parameter("imu_stuck_samples").value),
                enabled=bool(self.get_parameter("trip_on_imu_stuck").value),
            )
        )
        checks.extend(self._build_hardware_checks(started_at))
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
        # Only listen to streams that an enabled check uses.
        def uses(hook: str) -> bool:
            return any(getattr(type(check), hook) is not getattr(SafetyCheck, hook)
                       for check in checks)

        if uses("observe_joints"):
            self.create_subscription(
                JointState, str(self.get_parameter("joint_states_topic").value),
                self._joint_state_callback, 20)
        if uses("observe_motors"):
            self.create_subscription(
                String, str(self.get_parameter("motor_telemetry_topic").value),
                self._motor_telemetry_callback, 10)
        if uses("observe_can"):
            self.create_subscription(
                String, str(self.get_parameter("can_timing_topic").value),
                self._can_timing_callback, 10)
        if uses("observe_power"):
            self.create_subscription(
                BatteryState, str(self.get_parameter("battery_topic").value),
                self._battery_callback, 10)
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

    def _param(self, name: str):
        return self.get_parameter(name).value

    def _build_hardware_checks(self, started_at: float) -> list[SafetyCheck]:
        checks: list[SafetyCheck] = []
        grace = float(self._param("startup_grace_s"))
        needs_motor_config = any(self._param(name) for name in (
            "motor_torque_enabled", "motor_feedback_enabled", "joint_position_enabled"))
        motors = self._load_motor_config() if needs_motor_config else {}

        if self._param("motor_temperature_enabled"):
            checks.append(MotorTemperatureCheck(
                max_temp_c=float(self._param("motor_max_temp_c")),
                reset_temp_c=float(self._param("motor_reset_temp_c"))))
        if self._param("motor_fault_enabled"):
            checks.append(MotorFaultCheck())
        if self._param("motor_torque_enabled"):
            checks.append(MotorTorqueCheck(
                default_limit_nm=float(self._param("motor_max_torque_nm")),
                limits_nm={joint: cfg["max_torque_nm"] for joint, cfg in motors.items()
                           if cfg.get("max_torque_nm") is not None},
                confirm_samples=int(self._param("motor_torque_confirm_samples"))))
        if self._param("motor_feedback_enabled"):
            checks.append(MotorFeedbackCheck(
                expected_joints=list(motors),
                timeout_s=float(self._param("motor_feedback_timeout_s")),
                startup_grace_s=grace,
                started_at=started_at))
        if self._param("joint_position_enabled"):
            checks.append(JointPositionLimitCheck(
                limits_rad={joint: (cfg["min_position"], cfg["max_position"])
                            for joint, cfg in motors.items()
                            if cfg.get("min_position") is not None
                            and cfg.get("max_position") is not None},
                margin_rad=float(self._param("joint_position_margin_rad"))))
        if self._param("joint_velocity_enabled"):
            checks.append(JointVelocityLimitCheck(
                max_velocity_rad_s=float(self._param("joint_max_velocity_rad_s")),
                confirm_samples=int(self._param("joint_velocity_confirm_samples"))))
        if self._param("can_link_enabled"):
            checks.append(CanLinkCheck(
                timeout_s=float(self._param("can_timeout_s")),
                startup_grace_s=grace,
                started_at=started_at))
        if self._param("battery_enabled"):
            checks.append(BatteryVoltageCheck(
                min_voltage_v=float(self._param("battery_min_voltage_v")),
                reset_voltage_v=float(self._param("battery_reset_voltage_v")),
                confirm_samples=int(self._param("battery_confirm_samples"))))
        return checks

    def _load_motor_config(self) -> dict:
        """Return the motors: table from motor_control_hybrid's motors.yaml."""
        import yaml

        path = str(self._param("motor_config_file"))
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = str(Path(get_package_share_directory("motor_control_hybrid"))
                       / "config" / "motors.yaml")
        data = yaml.safe_load(Path(path).read_text()) or {}
        for node_config in data.values():
            motors = (node_config or {}).get("ros__parameters", {}).get("motors")
            if motors:
                return dict(motors)
        # Fail loudly: an enabled check with no joints would silently pass.
        raise ValueError(f"no motors table found in {path}")

    def _source_age_s(self, stamp) -> float:
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= 0:
            return 0.0
        return max(0.0, (self.get_clock().now().nanoseconds - stamp_ns) * 1.0e-9)

    def _joint_state_callback(self, msg: JointState) -> None:
        def by_name(values) -> dict:
            return {name: float(value) for name, value in zip(msg.name, values)}

        self.supervisor.observe(JointObservation(
            received_at=self._now_seconds(),
            positions=by_name(msg.position),
            velocities=by_name(msg.velocity),
            efforts=by_name(msg.effort),
            source_age_s=self._source_age_s(msg.header.stamp),
        ))

    def _motor_telemetry_callback(self, msg: String) -> None:
        # python_can_node publishes {"schema_version": 1, "motors": [{...}]}.
        try:
            motors = json.loads(msg.data)["motors"]
            temperatures = {m["joint_name"]: float(m.get("temperature_c", math.nan))
                            for m in motors}
            faults = {m["joint_name"]: int(m["fault_bits"])
                      for m in motors if "fault_bits" in m}
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warn(f"ignoring bad motor telemetry: {exc}",
                                   throttle_duration_sec=1.0)
            return
        self.supervisor.observe(MotorObservation(
            received_at=self._now_seconds(),
            temperatures_c=temperatures,
            fault_bits=faults,
        ))

    def _can_timing_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            tx_errors = sum(int(iface.get("tx_errors", 0))
                            for iface in payload.get("interfaces", {}).values())
            transport_fault = bool(payload.get("transport_fault", False))
        except (ValueError, AttributeError, TypeError) as exc:
            self.get_logger().warn(f"ignoring bad CAN status: {exc}",
                                   throttle_duration_sec=1.0)
            return
        self.supervisor.observe(CanObservation(
            received_at=self._now_seconds(),
            transport_fault=transport_fault,
            tx_errors=tx_errors,
        ))

    def _battery_callback(self, msg: BatteryState) -> None:
        percentage = float(msg.percentage)
        self.supervisor.observe(PowerObservation(
            received_at=self._now_seconds(),
            voltage_v=float(msg.voltage),
            percentage=percentage if math.isfinite(percentage) else None,
        ))

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
        # A covariance[0] of -1 means the field is not reported (REP 145).
        gyro_unavailable = (
            len(msg.angular_velocity_covariance) > 0
            and msg.angular_velocity_covariance[0] < 0.0
        )
        accel_unavailable = (
            len(msg.linear_acceleration_covariance) > 0
            and msg.linear_acceleration_covariance[0] < 0.0
        )
        observation = ImuObservation(
            received_at=received_at,
            source_age_s=max(0.0, source_age_s),
            roll_rad=roll,
            pitch_rad=pitch,
            valid=valid and not orientation_unavailable,
            angular_velocity=None if gyro_unavailable else (
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ),
            linear_acceleration=None if accel_unavailable else (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ),
        )
        self.supervisor.observe(observation)

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
