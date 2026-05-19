"""Double-pendulum SDK demo.

This demo shows how an external Python user drives the ROS2 double-pendulum
setup through the SDK. It does not import ROS2. The expected runtime path is:

    this script -> RobotSDK -> MotorGrpcClient -> ROS2 motor_sdk_gateway_node
    -> /motor_commands -> fake_motor_node or real CAN node

Run the ROS2 side first, for example with fake motors and the browser UI:

    cd Ros2_with_thread
    source install/setup.bash
    ros2 launch motor_control_hybrid hybrid_control.launch.py \
      enable_fake_motor:=true \
      enable_sdk_gateway:=true \
      enable_websocket_ui:=true \
      sdk_grpc_addr:=0.0.0.0:50052

Then run this file from the repository root:

    python3 -m sdk_prototype.demo.robot_sdk_demo.model --host <robot-ip>
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path


def _add_repo_root_to_path() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    repo_root_str = str(repo_root)
    if repo_root_str not in sys.path:
        sys.path.insert(0, repo_root_str)


_add_repo_root_to_path()


JOINT_1 = "test_joint"
JOINT_2 = "test_joint2"
DEFAULT_KP = 40.0
DEFAULT_KD = 1.5
DEFAULT_VELOCITY = 1.0


def default_motor_config_path() -> Path:
    repo_root = Path(__file__).resolve().parents[3]
    return repo_root / "Ros2_with_thread" / "motor_control_hybrid" / "config" / "motors.yaml"


@dataclass(frozen=True)
class DoublePendulumCommand:
    theta1_rad: float
    theta2_rad: float


class DoublePendulumSdkDemo:
    """Small user-facing example that controls two ROS2 motor joints via SDK."""

    def __init__(
        self,
        sdk,
        joint_1: str = JOINT_1,
        joint_2: str = JOINT_2,
        kp: float = DEFAULT_KP,
        kd: float = DEFAULT_KD,
        velocity: float = DEFAULT_VELOCITY,
    ) -> None:
        self.joint_1 = joint_1
        self.joint_2 = joint_2
        self.kp = float(kp)
        self.kd = float(kd)
        self.velocity = float(velocity)
        self.motor_1 = sdk.motor(joint_1)
        self.motor_2 = sdk.motor(joint_2)

    def enable(self) -> None:
        self.motor_1.enable()
        self.motor_2.enable()

    def disable(self) -> None:
        self.motor_1.disable()
        self.motor_2.disable()

    def hold_zero(self) -> None:
        self.set_position(DoublePendulumCommand(0.0, 0.0))

    def set_position(self, command: DoublePendulumCommand) -> None:
        self.motor_1.set_position(command.theta1_rad, velocity=self.velocity, kp=self.kp, kd=self.kd)
        self.motor_2.set_position(command.theta2_rad, velocity=self.velocity, kp=self.kp, kd=self.kd)

    def sine(
        self,
        duration_s: float,
        hz: float = 0.5,
        amplitude_rad: float = 0.6,
        command_period_s: float = 0.05,
    ) -> None:
        start = time.monotonic()
        while True:
            elapsed = time.monotonic() - start
            if elapsed >= duration_s:
                break

            phase = 2.0 * math.pi * hz * elapsed
            command = DoublePendulumCommand(
                theta1_rad=amplitude_rad * math.sin(phase),
                theta2_rad=amplitude_rad * math.sin(phase + math.pi / 2.0),
            )
            self.set_position(command)
            time.sleep(command_period_s)


def build_sdk(config_path: Path, motor_grpc_addr: str):
    from sdk_prototype.python.robot_sdk import RobotSDK, load_motor_configs_from_yaml

    motor_configs = load_motor_configs_from_yaml(config_path) if config_path.exists() else {}
    return RobotSDK(motor_configs=motor_configs, motor_grpc_addr=motor_grpc_addr)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Control the ROS2 double-pendulum demo through RobotSDK.")
    parser.add_argument("--config", type=Path, default=default_motor_config_path())
    parser.add_argument("--host", default="127.0.0.1", help="ROS2 gateway host or robot IP.")
    parser.add_argument("--motor-grpc-addr", default=None, help="Full motor gateway address, e.g. 192.168.1.20:50052.")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--hz", type=float, default=0.5)
    parser.add_argument("--amplitude", type=float, default=0.6)
    parser.add_argument("--kp", type=float, default=DEFAULT_KP)
    parser.add_argument("--kd", type=float, default=DEFAULT_KD)
    parser.add_argument("--velocity", type=float, default=DEFAULT_VELOCITY)
    parser.add_argument("--no-disable", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    motor_grpc_addr = args.motor_grpc_addr or f"{args.host}:50052"
    sdk = build_sdk(args.config, motor_grpc_addr)
    demo = DoublePendulumSdkDemo(
        sdk,
        kp=args.kp,
        kd=args.kd,
        velocity=args.velocity,
    )

    demo.enable()
    try:
        demo.hold_zero()
        time.sleep(0.5)
        demo.sine(args.duration, hz=args.hz, amplitude_rad=args.amplitude)
        demo.hold_zero()
    finally:
        if not args.no_disable:
            demo.disable()


if __name__ == "__main__":
    main()
