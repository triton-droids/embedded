#!/usr/bin/env python3
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any, Dict, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState, Imu
from motor_control_interfaces.msg import MotorCommand


SUPPORTED_MSG_TYPES = {
    "std_msgs/msg/String": String,
    "std_msgs/msg/Float64": Float64,
    "sensor_msgs/msg/JointState": JointState,
    "sensor_msgs/msg/Imu": Imu,
    "motor_control_interfaces/msg/MotorCommand": MotorCommand,
}


def ros_msg_to_dict(msg):
    if isinstance(msg, String):
        return {"data": msg.data}

    if isinstance(msg, Float64):
        return {"data": float(msg.data)}

    if isinstance(msg, JointState):
        return {
            "header": {
                "stamp": {
                    "sec": int(msg.header.stamp.sec),
                    "nanosec": int(msg.header.stamp.nanosec),
                },
                "frame_id": msg.header.frame_id,
            },
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort),
        }

    if isinstance(msg, Imu):
        return {
            "header": {
                "stamp": {
                    "sec": int(msg.header.stamp.sec),
                    "nanosec": int(msg.header.stamp.nanosec),
                },
                "frame_id": msg.header.frame_id,
            },
            "orientation": {
                "x": float(msg.orientation.x),
                "y": float(msg.orientation.y),
                "z": float(msg.orientation.z),
                "w": float(msg.orientation.w),
            },
            "angular_velocity": {
                "x": float(msg.angular_velocity.x),
                "y": float(msg.angular_velocity.y),
                "z": float(msg.angular_velocity.z),
            },
            "linear_acceleration": {
                "x": float(msg.linear_acceleration.x),
                "y": float(msg.linear_acceleration.y),
                "z": float(msg.linear_acceleration.z),
            },
        }

    if isinstance(msg, MotorCommand):
        return {
            "header": {
                "stamp": {
                    "sec": int(msg.header.stamp.sec),
                    "nanosec": int(msg.header.stamp.nanosec),
                }
            },
            "joint_name": list(msg.joint_name),
            "mode": list(msg.mode),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "acceleration": list(msg.acceleration),
            "torque": list(msg.torque),
            "kp": list(msg.kp),
            "kd": list(msg.kd),
        }

    raise TypeError(f"Unsupported message instance: {type(msg)}")


def dict_to_ros_msg(msg_type_str: str, payload: Dict[str, Any], node: Node):
    cls = SUPPORTED_MSG_TYPES[msg_type_str]
    msg = cls()

    if cls is String:
        msg.data = str(payload.get("data", ""))
        return msg

    if cls is Float64:
        msg.data = float(payload.get("data", 0.0))
        return msg

    if cls is JointState:
        if "header" in payload and "stamp" in payload["header"]:
            stamp = payload["header"]["stamp"]
            msg.header.stamp.sec = int(stamp.get("sec", 0))
            msg.header.stamp.nanosec = int(stamp.get("nanosec", 0))
        else:
            msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = str(payload.get("header", {}).get("frame_id", ""))

        msg.name = [str(x) for x in payload.get("name", [])]
        msg.position = [float(x) for x in payload.get("position", [])]
        msg.velocity = [float(x) for x in payload.get("velocity", [])]
        msg.effort = [float(x) for x in payload.get("effort", [])]
        return msg

    if cls is Imu:
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = str(payload.get("header", {}).get("frame_id", ""))

        ori = payload.get("orientation", {})
        msg.orientation.x = float(ori.get("x", 0.0))
        msg.orientation.y = float(ori.get("y", 0.0))
        msg.orientation.z = float(ori.get("z", 0.0))
        msg.orientation.w = float(ori.get("w", 1.0))

        av = payload.get("angular_velocity", {})
        msg.angular_velocity.x = float(av.get("x", 0.0))
        msg.angular_velocity.y = float(av.get("y", 0.0))
        msg.angular_velocity.z = float(av.get("z", 0.0))

        la = payload.get("linear_acceleration", {})
        msg.linear_acceleration.x = float(la.get("x", 0.0))
        msg.linear_acceleration.y = float(la.get("y", 0.0))
        msg.linear_acceleration.z = float(la.get("z", 0.0))
        return msg

    if cls is MotorCommand:
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.joint_name = [str(x) for x in payload.get("joint_name", [])]
        msg.mode = [int(x) for x in payload.get("mode", [])]
        msg.position = [float(x) for x in payload.get("position", [])]
        msg.velocity = [float(x) for x in payload.get("velocity", [])]
        msg.acceleration = [float(x) for x in payload.get("acceleration", [])]
        msg.torque = [float(x) for x in payload.get("torque", [])]
        msg.kp = [float(x) for x in payload.get("kp", [])]
        msg.kd = [float(x) for x in payload.get("kd", [])]
        return msg

    raise TypeError(f"Unsupported msg_type: {msg_type_str}")


class GenericGatewayNode(Node):
    def __init__(self):
        super().__init__("generic_gateway_node")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8080)
        self.declare_parameter("repeat_publish_hz", 0.0)
        self.declare_parameter("repeat_topics", ["/desired_motor_subset"])

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.repeat_publish_hz = float(self.get_parameter("repeat_publish_hz").value)
        self.repeat_topics = {
            str(topic) for topic in self.get_parameter("repeat_topics").value
        }

        self._lock = threading.Lock()

        self._pub_map: Dict[Tuple[str, str], Any] = {}
        self._sub_map: Dict[Tuple[str, str], Any] = {}
        self._last_msg_map: Dict[Tuple[str, str], Dict[str, Any]] = {}
        self._repeat_payload_map: Dict[Tuple[str, str], Dict[str, Any]] = {}

        self._server = None
        self._thread = threading.Thread(target=self._run_http, daemon=True)
        self._thread.start()
        self._repeat_timer = None
        if self.repeat_publish_hz > 0.0:
            self._repeat_timer = self.create_timer(
                1.0 / self.repeat_publish_hz,
                self._repeat_last_published,
            )

        self.get_logger().info(
            f"Generic gateway started at http://{self.host}:{self.port}"
        )

    def _should_repeat_topic(self, topic: str) -> bool:
        return "*" in self.repeat_topics or topic in self.repeat_topics

    def _remember_repeat_payload(self, topic: str, msg_type: str, payload: Dict[str, Any]):
        if self.repeat_publish_hz <= 0.0 or not self._should_repeat_topic(topic):
            return

        with self._lock:
            self._repeat_payload_map[(topic, msg_type)] = dict(payload)

    def _repeat_last_published(self):
        with self._lock:
            items = list(self._repeat_payload_map.items())

        for (topic, msg_type), payload in items:
            try:
                pub = self._get_or_create_publisher(topic, msg_type)
                msg = dict_to_ros_msg(msg_type, payload, self)
                pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(
                    f"Failed to repeat publish {topic} ({msg_type}): {e}"
                )

    def _get_or_create_publisher(self, topic: str, msg_type: str):
        key = (topic, msg_type)
        with self._lock:
            if key in self._pub_map:
                return self._pub_map[key]

            if msg_type not in SUPPORTED_MSG_TYPES:
                raise ValueError(f"Unsupported msg_type: {msg_type}")

            pub = self.create_publisher(SUPPORTED_MSG_TYPES[msg_type], topic, 10)
            self._pub_map[key] = pub
            return pub

    def _get_or_create_subscriber(self, topic: str, msg_type: str):
        key = (topic, msg_type)
        with self._lock:
            if key in self._sub_map:
                return self._sub_map[key]

            if msg_type not in SUPPORTED_MSG_TYPES:
                raise ValueError(f"Unsupported msg_type: {msg_type}")

            msg_cls = SUPPORTED_MSG_TYPES[msg_type]

            def cb(msg, topic=topic, msg_type=msg_type):
                with self._lock:
                    self._last_msg_map[(topic, msg_type)] = ros_msg_to_dict(msg)

            sub = self.create_subscription(msg_cls, topic, cb, 10)
            self._sub_map[key] = sub
            return sub

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
                if self.path == "/" or self.path == "/health":
                    self._send_json(200, {"ok": True})
                    return

                if self.path == "/topics":
                    with node._lock:
                        pubs = [
                            {"topic": t, "msg_type": m}
                            for (t, m) in node._pub_map.keys()
                        ]
                        subs = [
                            {"topic": t, "msg_type": m}
                            for (t, m) in node._sub_map.keys()
                        ]
                        repeaters = [
                            {"topic": t, "msg_type": m}
                            for (t, m) in node._repeat_payload_map.keys()
                        ]
                    self._send_json(
                        200,
                        {
                            "ok": True,
                            "publishers": pubs,
                            "subscribers": subs,
                            "repeat_publish_hz": node.repeat_publish_hz,
                            "repeaters": repeaters,
                        },
                    )
                    return

                if self.path.startswith("/message?"):
                    try:
                        query = self.path.split("?", 1)[1]
                        params = {}
                        for item in query.split("&"):
                            if "=" in item:
                                k, v = item.split("=", 1)
                                params[k] = v

                        topic = params.get("topic", "")
                        msg_type = params.get("msg_type", "")

                        if not topic or not msg_type:
                            self._send_json(400, {"ok": False, "error": "topic and msg_type required"})
                            return

                        with node._lock:
                            msg = node._last_msg_map.get((topic, msg_type))

                        self._send_json(200, {"ok": True, "topic": topic, "msg_type": msg_type, "message": msg})
                    except Exception as e:
                        self._send_json(400, {"ok": False, "error": str(e)})
                    return

                self._send_json(404, {"ok": False, "error": "not found"})

            def do_POST(self):
                length = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(length) if length > 0 else b"{}"

                try:
                    data = json.loads(raw.decode("utf-8"))
                except Exception as e:
                    self._send_json(400, {"ok": False, "error": f"invalid json: {e}"})
                    return

                if self.path == "/publish":
                    try:
                        topic = data["topic"]
                        msg_type = data["msg_type"]
                        payload = data.get("payload", {})

                        pub = node._get_or_create_publisher(topic, msg_type)
                        msg = dict_to_ros_msg(msg_type, payload, node)
                        pub.publish(msg)
                        node._remember_repeat_payload(topic, msg_type, payload)

                        self._send_json(200, {"ok": True, "topic": topic, "msg_type": msg_type})
                    except Exception as e:
                        self._send_json(400, {"ok": False, "error": str(e)})
                    return

                if self.path == "/subscribe":
                    try:
                        topic = data["topic"]
                        msg_type = data["msg_type"]
                        node._get_or_create_subscriber(topic, msg_type)

                        self._send_json(200, {"ok": True, "topic": topic, "msg_type": msg_type})
                    except Exception as e:
                        self._send_json(400, {"ok": False, "error": str(e)})
                    return

                self._send_json(404, {"ok": False, "error": "not found"})

            def log_message(self, format, *args):
                return

        self._server = HTTPServer((self.host, self.port), Handler)
        self._server.serve_forever()


def main():
    rclpy.init()
    node = GenericGatewayNode()
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
