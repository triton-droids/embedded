from __future__ import annotations

from concurrent import futures

import grpc

from . import robot_sdk_pb2
from . import robot_sdk_pb2_grpc
from .model import (
    MODE_ESTOP,
    MODE_IDLE,
    MODE_MANUAL,
    MODE_POLICY,
    MODE_VELOCITY,
    RobotSimulator,
)


GRPC_ADDR = "127.0.0.1:50051"


PROTO_TO_MODE = {
    robot_sdk_pb2.ROBOT_MODE_IDLE: MODE_IDLE,
    robot_sdk_pb2.ROBOT_MODE_MANUAL: MODE_MANUAL,
    robot_sdk_pb2.ROBOT_MODE_VELOCITY: MODE_VELOCITY,
    robot_sdk_pb2.ROBOT_MODE_POLICY: MODE_POLICY,
    robot_sdk_pb2.ROBOT_MODE_ESTOP: MODE_ESTOP,
}

MODE_TO_PROTO = {value: key for key, value in PROTO_TO_MODE.items()}
HEALTH_TO_PROTO = {
    "ok": robot_sdk_pb2.ROBOT_HEALTH_OK,
    "warn": robot_sdk_pb2.ROBOT_HEALTH_WARN,
    "fault": robot_sdk_pb2.ROBOT_HEALTH_FAULT,
}


class RobotControlService(robot_sdk_pb2_grpc.RobotControlServicer):
    def __init__(self, robot: RobotSimulator | None = None) -> None:
        self._robot = robot or RobotSimulator()

    def EnableRobot(self, request, context):
        return self._reply(self._robot.enable_robot())

    def DisableRobot(self, request, context):
        return self._reply(self._robot.disable_robot())

    def SetMode(self, request, context):
        return self._reply(self._robot.set_mode(PROTO_TO_MODE.get(request.mode, "")))

    def LoadPolicy(self, request, context):
        return self._reply(self._robot.load_policy(request.policy_id, request.uri))

    def StartPolicy(self, request, context):
        return self._reply(self._robot.start_policy(request.policy_id))

    def StopPolicy(self, request, context):
        return self._reply(self._robot.stop_policy())

    def SetVelocityCommand(self, request, context):
        return self._reply(
            self._robot.set_velocity_command(
                request.vx_mps,
                request.vy_mps,
                request.wz_radps,
                request.timeout_s,
            )
        )

    def GetRobotStatus(self, request, context):
        return self._status(self._robot.status_dict())

    def _reply(self, data: dict):
        return robot_sdk_pb2.CommandReply(
            accepted=data["accepted"],
            message=data["message"],
            status=self._status(data["status"]),
        )

    def _status(self, data: dict):
        return robot_sdk_pb2.RobotStatus(
            enabled=data["enabled"],
            mode=MODE_TO_PROTO.get(data["mode"], robot_sdk_pb2.ROBOT_MODE_UNSPECIFIED),
            health=HEALTH_TO_PROTO.get(data["health"], robot_sdk_pb2.ROBOT_HEALTH_UNSPECIFIED),
            active_policy_id=data["active_policy_id"],
            state_sequence=data["state_sequence"],
            stamp_unix_s=data["stamp_unix_s"],
        )

def main() -> None:
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=8))
    robot_sdk_pb2_grpc.add_RobotControlServicer_to_server(RobotControlService(), server)
    server.add_insecure_port(GRPC_ADDR)
    server.start()
    print(f"gRPC RobotControl listening on {GRPC_ADDR}")
    server.wait_for_termination()


if __name__ == "__main__":
    main()
