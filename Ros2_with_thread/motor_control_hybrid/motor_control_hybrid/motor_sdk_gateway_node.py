#!/usr/bin/env python3
from __future__ import annotations

import json
import threading
import time
from concurrent import futures
from typing import Iterable

import grpc
import rclpy
from motor_control_interfaces.msg import MotorCommand
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from motor_control_hybrid import robot_sdk_pb2, robot_sdk_pb2_grpc


GRPC_ADDR = "127.0.0.1:50052"


class MotorSdkGatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("motor_sdk_gateway_node")

        self.declare_parameter("grpc_addr", GRPC_ADDR)
        self.grpc_addr = str(self.get_parameter("grpc_addr").value)

        self._lock = threading.Lock()
        self._joint_state_by_name: dict[str, dict[str, float]] = {}
        self._temperature_by_index: dict[int, float] = {}
        self._motor_index_by_name: dict[str, int] = {}

        self._command_pub = self.create_publisher(MotorCommand, "motor_commands", 10)
        self._joint_state_sub = self.create_subscription(
            JointState,
            "joint_states",
            self._joint_state_callback,
            10,
        )
        self._motor_status_sub = self.create_subscription(
            String,
            "motor_status",
            self._motor_status_callback,
            10,
        )

        self._grpc_server = grpc.server(futures.ThreadPoolExecutor(max_workers=8))
        robot_sdk_pb2_grpc.add_MotorControlServicer_to_server(
            MotorControlService(self),
            self._grpc_server,
        )
        self._grpc_server.add_insecure_port(self.grpc_addr)
        self._grpc_thread = threading.Thread(target=self._serve_grpc, daemon=True)
        self._grpc_thread.start()

        self.get_logger().info(f"Motor SDK gRPC gateway listening on {self.grpc_addr}")

    def publish_command(
        self,
        joint_names: Iterable[str],
        mode: int,
        *,
        position: Iterable[float] = (),
        velocity: Iterable[float] = (),
        acceleration: Iterable[float] = (),
        torque: Iterable[float] = (),
        kp: Iterable[float] = (),
        kd: Iterable[float] = (),
    ) -> list[str]:
        names = [str(name) for name in joint_names if str(name)]
        if not names:
            raise ValueError("joint_names must not be empty")

        count = len(names)
        msg = MotorCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_name = names
        msg.mode = [int(mode)]
        msg.position = self._expand_numeric_field("position", position, count)
        msg.velocity = self._expand_numeric_field("velocity", velocity, count)
        msg.acceleration = self._expand_numeric_field("acceleration", acceleration, count)
        msg.torque = self._expand_numeric_field("torque", torque, count)
        msg.kp = self._expand_numeric_field("kp", kp, count)
        msg.kd = self._expand_numeric_field("kd", kd, count)
        self._command_pub.publish(msg)
        return names

    def _expand_numeric_field(
        self,
        field_name: str,
        values: Iterable[float],
        count: int,
    ) -> list[float]:
        result = [float(value) for value in values]
        if len(result) in (0, count):
            return result
        if len(result) == 1:
            return result * count
        raise ValueError(
            f"{field_name} must contain 0, 1, or {count} value(s); got {len(result)}"
        )

    def get_motor_statuses(self, joint_names: Iterable[str]) -> list[dict[str, float | str]]:
        requested = [str(name) for name in joint_names if str(name)]
        with self._lock:
            names = requested or list(self._joint_state_by_name.keys())
            result = []
            for name in names:
                state = self._joint_state_by_name.get(name)
                if state is None:
                    continue

                index = self._motor_index_by_name.get(name)
                temperature = self._temperature_by_index.get(index, 0.0) if index is not None else 0.0
                result.append(
                    {
                        "joint_name": name,
                        "position_rad": state["position_rad"],
                        "velocity_radps": state["velocity_radps"],
                        "effort_nm": state["effort_nm"],
                        "temperature_c": temperature,
                        "stamp_unix_s": state["stamp_unix_s"],
                    }
                )
            return result

    def _serve_grpc(self) -> None:
        self._grpc_server.start()
        self._grpc_server.wait_for_termination()

    def _joint_state_callback(self, msg: JointState) -> None:
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp <= 0.0:
            stamp = time.time()

        with self._lock:
            for index, name in enumerate(msg.name):
                self._motor_index_by_name[name] = index
                self._joint_state_by_name[name] = {
                    "position_rad": msg.position[index] if index < len(msg.position) else 0.0,
                    "velocity_radps": msg.velocity[index] if index < len(msg.velocity) else 0.0,
                    "effort_nm": msg.effort[index] if index < len(msg.effort) else 0.0,
                    "stamp_unix_s": stamp,
                }

    def _motor_status_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Invalid motor_status JSON: {exc}")
            return

        with self._lock:
            for key, status in payload.items():
                try:
                    index = int(key)
                    self._temperature_by_index[index] = float(status.get("temperature", 0.0))
                except (TypeError, ValueError, AttributeError):
                    continue

    def destroy_node(self) -> None:
        self._grpc_server.stop(grace=None)
        super().destroy_node()


class MotorControlService(robot_sdk_pb2_grpc.MotorControlServicer):
    def __init__(self, node: MotorSdkGatewayNode) -> None:
        self._node = node

    def EnableMotors(self, request, context):
        return self._publish_simple(request, MotorCommand.MODE_ENABLE, "enable command published")

    def DisableMotors(self, request, context):
        return self._publish_simple(request, MotorCommand.MODE_DISABLE, "disable command published")

    def SetMotorVelocity(self, request, context):
        return self._publish(
            request.joint_names,
            MotorCommand.MODE_VELOCITY,
            "velocity command published",
            velocity=request.velocity_radps,
            acceleration=request.acceleration_radps2,
        )

    def SetMotorPosition(self, request, context):
        return self._publish(
            request.joint_names,
            MotorCommand.MODE_POSITION,
            "position command published",
            position=request.position_rad,
            velocity=request.velocity_radps,
            kp=request.kp,
            kd=request.kd,
        )

    def SetMotorMit(self, request, context):
        return self._publish(
            request.joint_names,
            MotorCommand.MODE_MOTION,
            "mit command published",
            position=request.position_rad,
            velocity=request.velocity_radps,
            torque=request.torque_nm,
            kp=request.kp,
            kd=request.kd,
        )

    def GetMotorStatus(self, request, context):
        statuses = self._node.get_motor_statuses(request.joint_names)
        return robot_sdk_pb2.MotorStatusReply(
            motors=[
                robot_sdk_pb2.MotorStatus(
                    joint_name=str(status["joint_name"]),
                    position_rad=float(status["position_rad"]),
                    velocity_radps=float(status["velocity_radps"]),
                    effort_nm=float(status["effort_nm"]),
                    temperature_c=float(status["temperature_c"]),
                    stamp_unix_s=float(status["stamp_unix_s"]),
                )
                for status in statuses
            ]
        )

    def _publish_simple(self, request, mode: int, message: str):
        return self._publish(request.joint_names, mode, message)

    def _publish(self, joint_names, mode: int, message: str, **fields):
        try:
            names = self._node.publish_command(joint_names, mode, **fields)
        except ValueError as exc:
            return robot_sdk_pb2.MotorCommandReply(
                accepted=False,
                message=str(exc),
                joint_names=[],
            )

        return robot_sdk_pb2.MotorCommandReply(
            accepted=True,
            message=message,
            joint_names=names,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorSdkGatewayNode()

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
