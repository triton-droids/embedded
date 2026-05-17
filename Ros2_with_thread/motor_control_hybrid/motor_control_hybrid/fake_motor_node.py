#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import time

import rclpy
from motor_control_interfaces.msg import MotorCommand
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class FakeMotorNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_motor_node")

        self.declare_parameter("joint_names", ["test_joint", "test_joint2"])
        self.declare_parameter("publish_rate_hz", 50.0)

        joint_names_param = self.get_parameter("joint_names").value
        self.joint_names = [str(name) for name in joint_names_param]
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self._enabled = {name: False for name in self.joint_names}
        self._mode = {name: MotorCommand.MODE_DISABLE for name in self.joint_names}
        self._position = {name: 0.0 for name in self.joint_names}
        self._velocity = {name: 0.0 for name in self.joint_names}
        self._effort = {name: 0.0 for name in self.joint_names}
        self._temperature = {name: 28.0 for name in self.joint_names}
        self._target_position = {name: 0.0 for name in self.joint_names}
        self._kp = {name: 40.0 for name in self.joint_names}
        self._kd = {name: 1.5 for name in self.joint_names}
        self._disabled_warning_time = {name: 0.0 for name in self.joint_names}
        self._last_update = time.monotonic()

        self._joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self._motor_status_pub = self.create_publisher(String, "motor_status", 10)
        self._command_sub = self.create_subscription(
            MotorCommand,
            "motor_commands",
            self._command_callback,
            10,
        )

        period = 1.0 / publish_rate_hz if publish_rate_hz > 0.0 else 0.02
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Fake motor node started for joints: {', '.join(self.joint_names)}"
        )

    def _command_callback(self, msg: MotorCommand) -> None:
        if not msg.joint_name:
            return

        if len(msg.mode) == 1:
            modes = [msg.mode[0]] * len(msg.joint_name)
        elif len(msg.mode) == 0:
            modes = [MotorCommand.MODE_VELOCITY] * len(msg.joint_name)
        else:
            modes = list(msg.mode)

        for index, joint_name in enumerate(msg.joint_name):
            if joint_name not in self._enabled:
                self._add_joint(joint_name)

            mode = modes[index] if index < len(modes) else MotorCommand.MODE_VELOCITY
            self._mode[joint_name] = int(mode)

            if mode == MotorCommand.MODE_ENABLE:
                self._enabled[joint_name] = True
                self._velocity[joint_name] = 0.0
                self.get_logger().info(f"Enabled {joint_name}")
                continue

            if mode == MotorCommand.MODE_DISABLE:
                self._enabled[joint_name] = False
                self._velocity[joint_name] = 0.0
                self.get_logger().info(f"Disabled {joint_name}")
                continue

            if not self._enabled[joint_name]:
                now = time.monotonic()
                last_warn = self._disabled_warning_time.get(joint_name, 0.0)
                if now - last_warn >= 1.0:
                    self.get_logger().warn(f"Ignoring command for disabled joint: {joint_name}")
                    self._disabled_warning_time[joint_name] = now
                continue

            if mode == MotorCommand.MODE_VELOCITY:
                self._velocity[joint_name] = self._value_at(msg.velocity, index, 0.0)
                continue

            if mode == MotorCommand.MODE_POSITION:
                self._target_position[joint_name] = self._value_at(
                    msg.position, index, self._position[joint_name]
                )
                self._velocity[joint_name] = abs(self._value_at(msg.velocity, index, 0.5))
                self._kp[joint_name] = self._value_at(msg.kp, index, self._kp[joint_name])
                self._kd[joint_name] = self._value_at(msg.kd, index, self._kd[joint_name])
                continue

            if mode == MotorCommand.MODE_MOTION:
                self._target_position[joint_name] = self._value_at(
                    msg.position, index, self._position[joint_name]
                )
                self._velocity[joint_name] = self._value_at(msg.velocity, index, 0.0)
                self._effort[joint_name] = self._value_at(msg.torque, index, 0.0)
                self._kp[joint_name] = self._value_at(msg.kp, index, self._kp[joint_name])
                self._kd[joint_name] = self._value_at(msg.kd, index, self._kd[joint_name])

    def _tick(self) -> None:
        now = time.monotonic()
        dt = max(0.0, now - self._last_update)
        self._last_update = now

        for joint_name in self.joint_names:
            if not self._enabled[joint_name]:
                self._effort[joint_name] = 0.0
                continue

            mode = self._mode[joint_name]
            if mode == MotorCommand.MODE_VELOCITY:
                self._position[joint_name] += self._velocity[joint_name] * dt
                self._effort[joint_name] = abs(self._velocity[joint_name]) * 0.1
            elif mode in (MotorCommand.MODE_POSITION, MotorCommand.MODE_MOTION):
                error = self._target_position[joint_name] - self._position[joint_name]
                max_step = abs(self._velocity[joint_name]) * dt
                if max_step <= 0.0:
                    max_step = 0.5 * dt
                step = math.copysign(min(abs(error), max_step), error) if error else 0.0
                self._position[joint_name] += step
                self._effort[joint_name] = abs(error) * 0.2 + abs(self._velocity[joint_name]) * 0.05

            self._temperature[joint_name] += abs(self._effort[joint_name]) * 0.001

        self._publish_joint_states()
        self._publish_motor_status()

    def _publish_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for joint_name in self.joint_names:
            msg.name.append(joint_name)
            msg.position.append(self._position[joint_name])
            msg.velocity.append(self._velocity[joint_name] if self._enabled[joint_name] else 0.0)
            msg.effort.append(self._effort[joint_name])

        self._joint_state_pub.publish(msg)

    def _publish_motor_status(self) -> None:
        status = {
            str(index): {
                "temperature": float(self._temperature[joint_name]),
                "torque": float(self._effort[joint_name]),
            }
            for index, joint_name in enumerate(self.joint_names)
        }
        msg = String()
        msg.data = json.dumps(status)
        self._motor_status_pub.publish(msg)

    def _add_joint(self, joint_name: str) -> None:
        self.joint_names.append(joint_name)
        self._enabled[joint_name] = False
        self._mode[joint_name] = MotorCommand.MODE_DISABLE
        self._position[joint_name] = 0.0
        self._velocity[joint_name] = 0.0
        self._effort[joint_name] = 0.0
        self._temperature[joint_name] = 28.0
        self._target_position[joint_name] = 0.0
        self._kp[joint_name] = 40.0
        self._kd[joint_name] = 1.5
        self._disabled_warning_time[joint_name] = 0.0

    @staticmethod
    def _value_at(values, index: int, default: float) -> float:
        if len(values) == 0:
            return default
        if len(values) == 1:
            return float(values[0])
        if index < len(values):
            return float(values[index])
        return default


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeMotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
