from __future__ import annotations

import json
import threading
import time

import zmq

from .grpc_client import GrpcRobotClient
from .zmq_state import STATE_ADDR


class HybridRobotClient:
    """Commands over gRPC, state monitoring over ZeroMQ PUB/SUB."""

    def __init__(self) -> None:
        self._commands = GrpcRobotClient()
        self._context = zmq.Context.instance()
        self._state = self._context.socket(zmq.SUB)
        self._state.connect(STATE_ADDR)
        self._state.setsockopt_string(zmq.SUBSCRIBE, "robot_state")

    def enable_robot(self):
        return self._commands.enable_robot()

    def disable_robot(self):
        return self._commands.disable_robot()

    def set_mode(self, mode: str):
        return self._commands.set_mode(mode)

    def load_policy(self, policy_id: str, uri: str):
        return self._commands.load_policy(policy_id, uri)

    def start_policy(self, policy_id: str):
        return self._commands.start_policy(policy_id)

    def stop_policy(self):
        return self._commands.stop_policy()

    def set_velocity_command(self, vx_mps: float, vy_mps: float, wz_radps: float, timeout_s: float = 0.25):
        return self._commands.set_velocity_command(vx_mps, vy_mps, wz_radps, timeout_s)

    def get_robot_status(self):
        return self._commands.get_robot_status()

    def stream_robot_state(self):
        while True:
            topic, payload = self._state.recv_string().split(" ", 1)
            if topic == "robot_state":
                yield json.loads(payload)


def print_state_stream(robot: HybridRobotClient) -> None:
    for index, state in enumerate(robot.stream_robot_state()):
        print("state", json.dumps(state["status"], indent=2))
        if index >= 5:
            return


def main() -> None:
    robot = HybridRobotClient()
    thread = threading.Thread(target=print_state_stream, args=(robot,), daemon=True)
    thread.start()

    actions = [
        robot.enable_robot,
        lambda: robot.set_mode("velocity"),
        lambda: robot.set_velocity_command(vx_mps=0.2, vy_mps=0.0, wz_radps=0.1),
        lambda: robot.load_policy("walk_v1", "/opt/policies/walk_v1.onnx"),
        lambda: robot.start_policy("walk_v1"),
        robot.stop_policy,
        robot.disable_robot,
    ]

    for action in actions:
        print(action())
        time.sleep(0.15)

    thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
