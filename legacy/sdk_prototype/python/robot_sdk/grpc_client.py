from __future__ import annotations

import uuid

import grpc

from . import robot_sdk_pb2
from . import robot_sdk_pb2_grpc


GRPC_ADDR = "127.0.0.1:50051"


MODE_TO_PROTO = {
    "idle": robot_sdk_pb2.ROBOT_MODE_IDLE,
    "manual": robot_sdk_pb2.ROBOT_MODE_MANUAL,
    "velocity": robot_sdk_pb2.ROBOT_MODE_VELOCITY,
    "policy": robot_sdk_pb2.ROBOT_MODE_POLICY,
    "estop": robot_sdk_pb2.ROBOT_MODE_ESTOP,
}


class GrpcRobotClient:
    def __init__(self, addr: str = GRPC_ADDR) -> None:
        self._channel = grpc.insecure_channel(addr)
        self._stub = robot_sdk_pb2_grpc.RobotControlStub(self._channel)

    def enable_robot(self):
        return self._stub.EnableRobot(robot_sdk_pb2.EnableRobotRequest(request_id=self._request_id()))

    def disable_robot(self):
        return self._stub.DisableRobot(robot_sdk_pb2.DisableRobotRequest(request_id=self._request_id()))

    def set_mode(self, mode: str):
        return self._stub.SetMode(
            robot_sdk_pb2.SetModeRequest(
                request_id=self._request_id(),
                mode=MODE_TO_PROTO[mode],
            )
        )

    def load_policy(self, policy_id: str, uri: str):
        return self._stub.LoadPolicy(
            robot_sdk_pb2.LoadPolicyRequest(
                request_id=self._request_id(),
                policy_id=policy_id,
                uri=uri,
            )
        )

    def start_policy(self, policy_id: str):
        return self._stub.StartPolicy(
            robot_sdk_pb2.StartPolicyRequest(request_id=self._request_id(), policy_id=policy_id)
        )

    def stop_policy(self):
        return self._stub.StopPolicy(robot_sdk_pb2.StopPolicyRequest(request_id=self._request_id()))

    def set_velocity_command(self, vx_mps: float, vy_mps: float, wz_radps: float, timeout_s: float = 0.25):
        return self._stub.SetVelocityCommand(
            robot_sdk_pb2.VelocityCommand(
                request_id=self._request_id(),
                vx_mps=vx_mps,
                vy_mps=vy_mps,
                wz_radps=wz_radps,
                timeout_s=timeout_s,
            )
        )

    def get_robot_status(self):
        return self._stub.GetRobotStatus(robot_sdk_pb2.GetRobotStatusRequest())

    def _request_id(self) -> str:
        return str(uuid.uuid4())


class MotorGrpcClient:
    def __init__(self, addr: str = "127.0.0.1:50052") -> None:
        self._channel = grpc.insecure_channel(addr)
        self._stub = robot_sdk_pb2_grpc.MotorControlStub(self._channel)

    def enable_motors(self, joint_names: list[str]):
        return self._stub.EnableMotors(
            robot_sdk_pb2.MotorSelection(
                request_id=self._request_id(),
                joint_names=joint_names,
            )
        )

    def disable_motors(self, joint_names: list[str]):
        return self._stub.DisableMotors(
            robot_sdk_pb2.MotorSelection(
                request_id=self._request_id(),
                joint_names=joint_names,
            )
        )

    def set_motor_velocity(
        self,
        joint_names: list[str],
        velocity_radps: list[float],
        acceleration_radps2: list[float] | None = None,
    ):
        return self._stub.SetMotorVelocity(
            robot_sdk_pb2.MotorVelocityCommand(
                request_id=self._request_id(),
                joint_names=joint_names,
                velocity_radps=velocity_radps,
                acceleration_radps2=acceleration_radps2 or [],
            )
        )

    def set_motor_position(
        self,
        joint_names: list[str],
        position_rad: list[float],
        velocity_radps: list[float] | None = None,
        kp: list[float] | None = None,
        kd: list[float] | None = None,
    ):
        return self._stub.SetMotorPosition(
            robot_sdk_pb2.MotorPositionCommand(
                request_id=self._request_id(),
                joint_names=joint_names,
                position_rad=position_rad,
                velocity_radps=velocity_radps or [],
                kp=kp or [],
                kd=kd or [],
            )
        )

    def set_motor_mit(
        self,
        joint_names: list[str],
        position_rad: list[float],
        velocity_radps: list[float],
        torque_nm: list[float] | None = None,
        kp: list[float] | None = None,
        kd: list[float] | None = None,
    ):
        return self._stub.SetMotorMit(
            robot_sdk_pb2.MotorMitCommand(
                request_id=self._request_id(),
                joint_names=joint_names,
                position_rad=position_rad,
                velocity_radps=velocity_radps,
                torque_nm=torque_nm or [],
                kp=kp or [],
                kd=kd or [],
            )
        )

    def get_motor_status(self, joint_names: list[str] | None = None):
        return self._stub.GetMotorStatus(
            robot_sdk_pb2.MotorSelection(
                request_id=self._request_id(),
                joint_names=joint_names or [],
            )
        )

    def _request_id(self) -> str:
        return str(uuid.uuid4())


def main() -> None:
    robot = GrpcRobotClient()

    actions = [
        robot.enable_robot,
        lambda: robot.set_mode("velocity"),
        lambda: robot.set_velocity_command(vx_mps=0.2, vy_mps=0.0, wz_radps=0.1),
        lambda: robot.load_policy("walk_v1", "/opt/policies/walk_v1.onnx"),
        lambda: robot.start_policy("walk_v1"),
    ]

    for action in actions:
        reply = action()
        print(reply)

    print(robot.get_robot_status())

    print(robot.stop_policy())
    print(robot.disable_robot())


if __name__ == "__main__":
    main()
