#!/usr/bin/env python3
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from motor_control_interfaces.msg import MotorCommand


def _now_ros_time(node: Node):
    return node.get_clock().now().to_msg()


def _as_list(x):
    if x is None:
        return []
    return x if isinstance(x, list) else [x]


class TargetGatewayNode(Node):
    """
    HTTP -> ROS topic gateway
    External: HTTP POST /target (JSON)
    Internal: publish motor_control_interfaces/MotorCommand to /desired_motor_subset
    """

    def __init__(self):
        super().__init__("target_gateway_node")

        # Params
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8080)
        self.declare_parameter("topic_out", "/desired_motor_subset")

        self.declare_parameter("default_kp", 40.0)
        self.declare_parameter("default_kd", 1.5)
        self.declare_parameter("default_mode", "velocity")  # velocity/position/motion/enable/disable

        self.host = self.get_parameter("host").get_parameter_value().string_value
        self.port = int(self.get_parameter("port").get_parameter_value().integer_value)
        self.topic_out = self.get_parameter("topic_out").get_parameter_value().string_value

        self.default_kp = float(self.get_parameter("default_kp").value)
        self.default_kd = float(self.get_parameter("default_kd").value)
        self.default_mode = self.get_parameter("default_mode").value

        self.pub = self.create_publisher(MotorCommand, self.topic_out, 10)

        self._lock = threading.Lock()
        self._last_publish_ok = True
        self._last_error = ""

        # Start HTTP server thread
        self._server = None
        self._thread = threading.Thread(target=self._run_http, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"TargetGatewayNode started: http://{self.host}:{self.port}  -> publish {self.topic_out}"
        )

    # -------- mode mapping --------
    def _mode_str_to_const(self, s: str) -> int:
        s = (s or "").strip().lower()
        if s in ("vel", "velocity"):
            return MotorCommand.MODE_VELOCITY
        if s in ("pos", "position"):
            return MotorCommand.MODE_POSITION
        if s in ("motion",):
            return MotorCommand.MODE_MOTION
        if s in ("enable", "on"):
            return MotorCommand.MODE_ENABLE
        if s in ("disable", "off"):
            return MotorCommand.MODE_DISABLE
        # fallback
        return MotorCommand.MODE_VELOCITY

    # -------- HTTP server --------
    def _run_http(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def _send_json(self, code: int, obj: Dict[str, Any]):
                data = json.dumps(obj).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                if self.path in ("/health", "/"):
                    with node._lock:
                        ok = node._last_publish_ok
                        err = node._last_error
                    self._send_json(
                        200,
                        {
                            "ok": True,
                            "gateway_publish_ok": ok,
                            "last_error": err,
                            "topic_out": node.topic_out,
                        },
                    )
                else:
                    self._send_json(404, {"ok": False, "error": "not found"})

            def do_POST(self):
                if self.path != "/target":
                    self._send_json(404, {"ok": False, "error": "not found"})
                    return

                length = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(length) if length > 0 else b"{}"

                try:
                    payload = json.loads(raw.decode("utf-8"))
                except Exception as e:
                    self._send_json(400, {"ok": False, "error": f"invalid json: {e}"})
                    return

                try:
                    msg = node._payload_to_motor_command(payload)
                    node.pub.publish(msg)
                    with node._lock:
                        node._last_publish_ok = True
                        node._last_error = ""
                    self._send_json(200, {"ok": True, "published_joints": len(msg.joint_name)})
                except Exception as e:
                    with node._lock:
                        node._last_publish_ok = False
                        node._last_error = str(e)
                    self._send_json(400, {"ok": False, "error": str(e)})

            # Quiet default logging
            def log_message(self, format, *args):
                return

        try:
            self._server = HTTPServer((self.host, self.port), Handler)
            self._server.serve_forever()
        except Exception as e:
            with self._lock:
                self._last_publish_ok = False
                self._last_error = f"HTTP server failed: {e}"
            self.get_logger().error(self._last_error)

    # -------- JSON -> MotorCommand --------
    def _payload_to_motor_command(self, payload: Dict[str, Any]) -> MotorCommand:
        """
        Supports:
        A) {"commands": {"joint": {"mode":"velocity","velocity":..,"kp":..,"kd":..}, ...}}
        B) {"joint_name":[...], "mode":[...], "velocity":[...], "position":[...], ...}
        """

        if not isinstance(payload, dict):
            raise ValueError("payload must be a JSON object")

        # --- form A: dict commands ---
        if "commands" in payload:
            commands = payload["commands"]
            if not isinstance(commands, dict) or not commands:
                raise ValueError("'commands' must be a non-empty object")

            joint_name: List[str] = []
            mode: List[int] = []
            position: List[float] = []
            velocity: List[float] = []
            acceleration: List[float] = []
            torque: List[float] = []
            kp: List[float] = []
            kd: List[float] = []

            for jn, cmd in commands.items():
                if not isinstance(jn, str) or not jn:
                    continue
                if not isinstance(cmd, dict):
                    raise ValueError(f"commands['{jn}'] must be an object")

                ms = cmd.get("mode", self.default_mode)
                joint_name.append(jn)
                mode.append(self._mode_str_to_const(ms))

                position.append(float(cmd.get("position", 0.0)))
                velocity.append(float(cmd.get("velocity", 0.0)))
                acceleration.append(float(cmd.get("acceleration", 0.0)))
                torque.append(float(cmd.get("torque", 0.0)))
                kp.append(float(cmd.get("kp", self.default_kp)))
                kd.append(float(cmd.get("kd", self.default_kd)))

            if not joint_name:
                raise ValueError("no valid joints in 'commands'")

            msg = MotorCommand()
            msg.header.stamp = _now_ros_time(self)
            msg.joint_name = joint_name
            msg.mode = mode
            msg.position = position
            msg.velocity = velocity
            msg.acceleration = acceleration
            msg.torque = torque
            msg.kp = kp
            msg.kd = kd
            return msg

        # --- form B: array fields ---
        joint_name = payload.get("joint_name", None)
        if not joint_name:
            raise ValueError("missing 'commands' or 'joint_name'")

        joint_name = _as_list(joint_name)
        if not all(isinstance(x, str) and x for x in joint_name):
            raise ValueError("'joint_name' must be a list of non-empty strings")

        n = len(joint_name)

        def get_arr(key: str, default: float) -> List[float]:
            arr = _as_list(payload.get(key, []))
            out = []
            for i in range(n):
                if i < len(arr):
                    out.append(float(arr[i]))
                else:
                    out.append(float(default))
            return out

        mode_in = _as_list(payload.get("mode", self.default_mode))
        mode: List[int] = []
        for i in range(n):
            m = mode_in[i] if i < len(mode_in) else mode_in[-1]
            if isinstance(m, str):
                mode.append(self._mode_str_to_const(m))
            else:
                mode.append(int(m))

        msg = MotorCommand()
        msg.header.stamp = _now_ros_time(self)
        msg.joint_name = joint_name
        msg.mode = mode
        msg.position = get_arr("position", 0.0)
        msg.velocity = get_arr("velocity", 0.0)
        msg.acceleration = get_arr("acceleration", 0.0)
        msg.torque = get_arr("torque", 0.0)
        msg.kp = get_arr("kp", self.default_kp)
        msg.kd = get_arr("kd", self.default_kd)
        return msg


def main():
    rclpy.init()
    node = TargetGatewayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
