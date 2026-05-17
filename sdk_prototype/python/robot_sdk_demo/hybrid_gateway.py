from __future__ import annotations

import threading
from concurrent import futures

import grpc
import zmq

from .grpc_server import GRPC_ADDR, RobotControlService
from .model import RobotSimulator
from . import robot_sdk_pb2_grpc
from .zmq_state import publish_state


def serve_grpc_commands(robot: RobotSimulator) -> None:
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=8))
    robot_sdk_pb2_grpc.add_RobotControlServicer_to_server(RobotControlService(robot), server)
    server.add_insecure_port(GRPC_ADDR)
    server.start()
    print(f"gRPC command endpoint listening on {GRPC_ADDR}")
    server.wait_for_termination()


def serve_zmq_state(robot: RobotSimulator) -> None:
    context = zmq.Context.instance()
    publish_state(context, robot)


def main() -> None:
    robot = RobotSimulator()
    threading.Thread(target=serve_zmq_state, args=(robot,), daemon=True).start()
    serve_grpc_commands(robot)


if __name__ == "__main__":
    main()
