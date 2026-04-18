#!/usr/bin/env python3
import time
from motor_command_client import GatewayClient, MotorCommandClient


def main():
    gw = GatewayClient(host="127.0.0.1", port=8080)
    motor = MotorCommandClient(
        gw,
        topic="/motor_commands",
        default_kp=10.0,
        default_kd=0.2,
    )

    joints = [
        "left_knee_joint",
        "right_knee_joint",
        "left_ankle_joint",
        "right_ankle_joint",
    ]

    # Enable all
    print("Enabling motors...")
    motor.enable(joints)
    time.sleep(1.0)

    # Move all together
    print("Move to pose 1")
    motor.motion(
        joint_name=joints,
        position=[-0.4, -0.4, 0.1, 0.1],
        velocity=[1.0, 1.0, 1.0, 1.0],
        kp=[10.0, 10.0, 8.0, 8.0],
        kd=[0.2, 0.2, 0.15, 0.15],
    )
    time.sleep(2.0)

    # Hold
    print("Holding pose")
    motor.hold(
        joint_name=joints,
        position=[-0.4, -0.4, 0.1, 0.1],
        kp=[12.0, 12.0, 10.0, 10.0],
        kd=[0.25, 0.25, 0.2, 0.2],
    )
    time.sleep(2.0)

    # Move back
    print("Move to zero")
    motor.motion(
        joint_name=joints,
        position=[0.0, 0.0, 0.0, 0.0],
        velocity=[1.0, 1.0, 1.0, 1.0],
        kp=[10.0, 10.0, 8.0, 8.0],
        kd=[0.2, 0.2, 0.15, 0.15],
    )
    time.sleep(2.0)

    # Disable all
    print("Disabling motors...")
    motor.disable(joints)
    print("Done.")


if __name__ == "__main__":
    main()