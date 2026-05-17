from __future__ import annotations

import time

from .grpc_client import MotorGrpcClient


def main() -> None:
    motors = MotorGrpcClient("127.0.0.1:50052")
    joints = ["test_joint", "test_joint2"]

    print(motors.enable_motors(joints))
    time.sleep(0.2)

    print(motors.set_motor_velocity(joints, [0.2, -0.2], [10.0, 10.0]))
    time.sleep(2.0)

    print(motors.get_motor_status(joints))

    print(motors.set_motor_velocity(joints, [0.0, 0.0], [10.0, 10.0]))
    time.sleep(0.2)

    print(motors.disable_motors(joints))


if __name__ == "__main__":
    main()
