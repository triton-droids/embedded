#!/usr/bin/python3.10

import argparse
import json
import re
from typing import Dict, Any

import requests

SERVICE_NAME = "/robstride_joint_control"


def compile_via_http(compile_url: str, instruction: str, timeout_s: float) -> str:
    """
    等价于：
    curl -s http://127.0.0.1:8000/compile -H "Content-Type: application/json" -d '{"instruction":"..."}'
    """
    r = requests.post(
        compile_url,
        headers={"Content-Type": "application/json"},
        json={"instruction": instruction},
        timeout=timeout_s,
    )
    if r.status_code != 200:
        raise RuntimeError(f"Compile server HTTP {r.status_code}: {r.text}")
    data = r.json()
    if "command" not in data:
        raise RuntimeError(f"Compile server response missing 'command': {data}")
    return str(data["command"]).strip()


def parse_fields_from_command(cmd: str) -> Dict[str, Any]:
    """
    从 ros2 service call 命令里解析出 joint_name / command_type / 数值字段
    兼容 joint_name 无引号/单引号/双引号。
    """
    # joint_name
    m = re.search(r"joint_name\s*:\s*['\"]?([a-zA-Z0-9_]+)['\"]?", cmd)
    if not m:
        raise ValueError("joint_name not found in command")
    joint = m.group(1)

    def get_num(name: str, default: float = 0.0) -> float:
        mm = re.search(rf"\b{name}\b\s*:\s*([-+]?\d+(?:\.\d+)?)", cmd)
        return float(mm.group(1)) if mm else default

    fields = {
        "joint_name": joint,
        "command_type": int(get_num("command_type", 0.0)),
        "position": get_num("position", 0.0),
        "velocity": get_num("velocity", 0.0),
        "torque": get_num("torque", 0.0),
        "iq": get_num("iq", 0.0),
        "id": get_num("id", 0.0),
        "acceleration": get_num("acceleration", 0.0),
        "kp": get_num("kp", 0.0),
        "kd": get_num("kd", 0.0),
    }
    return fields


def call_ros2_service(fields: Dict[str, Any], timeout_s: float) -> Dict[str, Any]:
    """
    直接用 rclpy 调用 /robstride_joint_control
    返回 dict：success/message + 回传状态（position/velocity/torque/temperature）
    """
    import rclpy
    from rclpy.node import Node
    from motor_control_interfaces.srv import RobStrideJointControl

    class Caller(Node):
        def __init__(self):
            super().__init__("robstride_compile_and_call")
            self.cli = self.create_client(RobStrideJointControl, SERVICE_NAME)

    rclpy.init()
    node = Caller()

    if not node.cli.wait_for_service(timeout_sec=timeout_s):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(f"Service {SERVICE_NAME} not available (timeout {timeout_s}s).")

    req = RobStrideJointControl.Request()
    req.joint_name = str(fields["joint_name"])
    req.command_type = int(fields["command_type"])
    req.position = float(fields["position"])
    req.velocity = float(fields["velocity"])
    req.torque = float(fields["torque"])
    req.iq = float(fields["iq"])
    req.id = float(fields["id"])
    req.acceleration = float(fields["acceleration"])
    req.kp = float(fields["kp"])
    req.kd = float(fields["kd"])

    fut = node.cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_s)

    if (not fut.done()) or fut.result() is None:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(f"Service call timed out after {timeout_s}s.")

    res = fut.result()
    out = {
        "success": bool(res.success),
        "message": str(res.message),
        "position": float(res.position),
        "velocity": float(res.velocity),
        "torque": float(res.torque),
        "temperature": float(res.temperature),
    }

    node.destroy_node()
    rclpy.shutdown()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--compile-url", default="http://127.0.0.1:8000/compile")
    ap.add_argument("--compile-timeout", type=float, default=20.0)
    ap.add_argument("--service-timeout", type=float, default=3.0)
    ap.add_argument("--print-only", action="store_true", help="只编译并打印，不调用ROS service")
    args = ap.parse_args()

    print("Client: instruction -> (HTTP compile) -> (rclpy service call). 输入 quit 退出.")
    print(f"- compile_url: {args.compile_url}")
    print(f"- print_only: {args.print_only}")

    while True:
        try:
            instr = input("\nInstruction> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nBye.")
            return
        if not instr:
            continue
        if instr.lower() in {"quit", "exit"}:
            print("Bye.")
            return

        try:
            cmd = compile_via_http(args.compile_url, instr, args.compile_timeout)
            print("\n=== Compiled Command ===")
            print(cmd)

            if args.print_only:
                continue

            fields = parse_fields_from_command(cmd)
            print("\n=== Parsed Fields ===")
            print(json.dumps(fields, indent=2))

            res = call_ros2_service(fields, args.service_timeout)
            print("\n=== ROS Service Response ===")
            print(json.dumps(res, indent=2))

        except Exception as e:
            print(f"\nERROR: {e}")


if __name__ == "__main__":
    main()
