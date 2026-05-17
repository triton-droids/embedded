from __future__ import annotations

import json
import time

import zmq

from .model import RobotSimulator


STATE_ADDR = "tcp://127.0.0.1:5556"


def publish_state(context: zmq.Context, robot: RobotSimulator, period_s: float = 0.1) -> None:
    socket = context.socket(zmq.PUB)
    socket.bind(STATE_ADDR)
    print(f"ZeroMQ state stream publishing on {STATE_ADDR}")

    while True:
        payload = json.dumps(robot.state_dict(), separators=(",", ":"))
        socket.send_string(f"robot_state {payload}")
        time.sleep(period_s)
