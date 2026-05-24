"""Small SDK demo that moves one arm joint and returns it to zero.

Run the ROS2 side first so the gRPC motor gateway and fake motor node are
available, then run this script from the repository root.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


def _add_repo_root_to_path() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    repo_root_str = str(repo_root)
    if repo_root_str not in sys.path:
        sys.path.insert(0, repo_root_str)


_add_repo_root_to_path()


DEFAULT_JOINT = "base_to_shoulder_joint"
DEFAULT_TARGET_RAD = 0.45
DEFAULT_RETURN_RAD = 0.0
DEFAULT_HOLD_S = 1.5
DEFAULT_KP = 40.0
DEFAULT_KD = 1.5
DEFAULT_VELOCITY = 1.0
DEFAULT_TOLERANCE_RAD = 0.03
DEFAULT_TIMEOUT_S = 10.0
ARM_JOINTS = [
    "base_to_shoulder_joint",
    "shoulder_to_upper_arm_joint",
    "upper_arm_to_lower_arm_joint",
    "lower_arm_to_wrist_joint",
]


def default_motor_config_path() -> Path:
    repo_root = Path(__file__).resolve().parents[3]
    return repo_root / "humanoid_control" / "motor_control_hybrid" / "config" / "control_config.yaml"


class SingleJointArmDemo:
    def __init__(
        self,
        sdk,
        joint_name: str = DEFAULT_JOINT,
        target_rad: float = DEFAULT_TARGET_RAD,
        return_rad: float = DEFAULT_RETURN_RAD,
        hold_s: float = DEFAULT_HOLD_S,
        kp: float = DEFAULT_KP,
        kd: float = DEFAULT_KD,
        velocity: float = DEFAULT_VELOCITY,
        tolerance_rad: float = DEFAULT_TOLERANCE_RAD,
        timeout_s: float = DEFAULT_TIMEOUT_S,
    ) -> None:
        self.joint_name = str(joint_name)
        self.target_rad = float(target_rad)
        self.return_rad = float(return_rad)
        self.hold_s = float(hold_s)
        self.kp = float(kp)
        self.kd = float(kd)
        self.velocity = float(velocity)
        self.tolerance_rad = float(tolerance_rad)
        self.timeout_s = float(timeout_s)
        self.motor = sdk.motor(self.joint_name)
        self.sdk = sdk
        self.arm_joints = list(ARM_JOINTS)

    def enable(self) -> None:
        self.sdk.motor_client.enable_motors(self.arm_joints)
        self.motor.enable()

    def disable(self) -> None:
        self.sdk.motor_client.disable_motors(self.arm_joints)
        self.motor.disable()

    def move_to(self, position_rad: float) -> None:
        self.motor.set_position(position_rad, velocity=self.velocity, kp=self.kp, kd=self.kd)

    def seed_home_pose(self) -> None:
        positions = [0.0] * len(self.arm_joints)
        velocities = [self.velocity] * len(self.arm_joints)
        gains_kp = [self.kp] * len(self.arm_joints)
        gains_kd = [self.kd] * len(self.arm_joints)
        self.sdk.motor_client.set_motor_position(
            self.arm_joints,
            positions,
            velocities,
            gains_kp,
            gains_kd,
        )

    def wait_until_reached(self, target_rad: float) -> bool:
        deadline = time.monotonic() + self.timeout_s
        stable_until = None

        while time.monotonic() < deadline:
            reply = self.sdk.motor_client.get_motor_status([self.joint_name])
            status = next((m for m in reply.motors if m.joint_name == self.joint_name), None)
            if status is None and reply.motors:
                status = reply.motors[0]
            if status is None:
                time.sleep(0.05)
                continue

            error = abs(float(status.position_rad) - float(target_rad))
            if error <= self.tolerance_rad:
                if stable_until is None:
                    stable_until = time.monotonic() + 0.25
                elif time.monotonic() >= stable_until:
                    return True
            else:
                stable_until = None

            time.sleep(0.05)

        return False


def build_sdk(config_path: Path):
    from sdk_prototype.python.robot_sdk import RobotSDK, load_motor_configs_from_yaml

    motor_configs = load_motor_configs_from_yaml(config_path) if config_path.exists() else {}
    return RobotSDK(motor_configs=motor_configs)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Move one humanoid arm joint through the SDK.")
    parser.add_argument("--config", type=Path, default=default_motor_config_path())
    parser.add_argument("--joint", type=str, default=DEFAULT_JOINT)
    parser.add_argument("--target", type=float, default=DEFAULT_TARGET_RAD)
    parser.add_argument("--return-angle", type=float, default=DEFAULT_RETURN_RAD)
    parser.add_argument("--hold", type=float, default=DEFAULT_HOLD_S)
    parser.add_argument("--kp", type=float, default=DEFAULT_KP)
    parser.add_argument("--kd", type=float, default=DEFAULT_KD)
    parser.add_argument("--velocity", type=float, default=DEFAULT_VELOCITY)
    parser.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE_RAD)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S)
    parser.add_argument("--return-after", action="store_true")
    parser.add_argument("--no-disable", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    sdk = build_sdk(args.config)
    demo = SingleJointArmDemo(
        sdk,
        joint_name=args.joint,
        target_rad=args.target,
        return_rad=args.return_angle,
        hold_s=args.hold,
        kp=args.kp,
        kd=args.kd,
        velocity=args.velocity,
        tolerance_rad=args.tolerance,
        timeout_s=args.timeout,
    )

    demo.enable()
    try:
        demo.seed_home_pose()
        demo.wait_until_reached(args.return_angle)
        demo.move_to(args.target)
        reached = demo.wait_until_reached(args.target)
        if not reached:
            print(
                f"Warning: joint {args.joint} did not reach {args.target:.3f} rad "
                f"within {args.timeout:.1f}s"
            )
        else:
            time.sleep(args.hold)
        if args.return_after:
            demo.seed_home_pose()
            demo.move_to(args.return_angle)
            demo.wait_until_reached(args.return_angle)
            time.sleep(args.hold)
    finally:
        if not args.no_disable:
            demo.disable()


if __name__ == "__main__":
    main()
