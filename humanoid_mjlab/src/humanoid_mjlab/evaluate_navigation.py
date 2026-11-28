"""Benchmark an mjlab checkpoint or follow an RRT* route in simulation."""

import argparse
import json
import math
from pathlib import Path

from humanoid_mjlab.path_follower import PathFollower, Twist, angle_error
from humanoid_mjlab.plan_path import PlanningMap, Point, add_planning_args, plan_path


def _benchmark_cases(speed: float, yaw: float, stop_at: float):
    straight = (speed, 0.0, 0.0)
    stop = (0.0, 0.0, 0.0)
    return {
        "straight": [(0.0, straight)],
        "slow": [(0.0, (speed / 2, 0.0, 0.0))],
        "stand": [(0.0, stop)],
        "walk_then_stop": [(0.0, straight), (stop_at, stop)],
        "walk_left": [(0.0, (speed, 0.0, yaw))],
        "walk_right": [(0.0, (speed, 0.0, -yaw))],
        "pivot_left": [(0.0, (0.0, 0.0, yaw))],
        "pivot_right": [(0.0, (0.0, 0.0, -yaw))],
    }


def run(args, map_: PlanningMap | None, path: list[Point] | None) -> dict:
    from dataclasses import asdict

    import torch
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import RslRlVecEnvWrapper
    from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

    from humanoid_mjlab.env_cfg import FixedVelocityCommand, FixedVelocityCommandCfg
    from humanoid_mjlab.env_cfg import FORWARD_SPEED, NAV_YAW_RATE, make_env_cfg
    from humanoid_mjlab.rl_cfg import make_rl_cfg

    follower: PathFollower | None = None
    schedule: list[tuple[float, Twist]] = []
    elapsed = 0.0

    def pose(robot, env) -> tuple[Point, float]:
        local_x, local_y = (
            robot.data.root_link_pos_w[0, :2] - env.scene.env_origins[0, :2]
        ).tolist()
        start_x, start_y = map_.start if map_ else (0.0, 0.0)
        return (local_x + start_x, local_y + start_y), float(robot.data.heading_w[0])

    class ReplayCommand(FixedVelocityCommand):
        def _update_command(self, env_ids=None) -> None:
            if follower is not None:
                command = follower.update(*pose(self.robot, self._env))
            else:
                command = schedule[0][1] if schedule else (0.0, 0.0, 0.0)
                for start, candidate in schedule[1:]:
                    if elapsed >= start:
                        command = candidate
            self.vel_command_b[0, :] = self.vel_command_b.new_tensor(command)

    class ReplayCommandCfg(FixedVelocityCommandCfg):
        def build(self, env) -> ReplayCommand:
            return ReplayCommand(self, env)

    navigation = args.profile == "navigation"
    cfg = make_env_cfg(play=True, navigation=navigation)
    cfg.auto_reset = False
    cfg.scale_rewards_by_dt = False  # Report weighted tracking terms on their original scale.
    cfg.seed = args.seed
    cfg.commands["twist"] = ReplayCommandCfg(
        entity_name="robot",
        resampling_time_range=(args.duration + 1.0, args.duration + 1.0),
        ranges=FixedVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 0.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(0.0, 0.0)
        ),
    )
    agent_cfg = make_rl_cfg(navigation=navigation)
    env = ManagerBasedRlEnv(cfg=cfg, device=args.device)
    wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    def episode(case: list[tuple[float, Twist]]) -> dict:
        nonlocal elapsed, schedule
        elapsed, schedule = 0.0, case
        obs, _ = wrapped.reset()
        robot = env.scene["robot"]
        start, initial_heading = pose(robot, env)
        previous, previous_heading = start, initial_heading
        yaw_displacement = 0.0
        distance = 0.0
        linear_sq, yaw_sq = 0.0, 0.0
        linear_reward, yaw_reward = 0.0, 0.0
        velocity_sum = [0.0, 0.0, 0.0]
        lateral_peak = 0.0
        samples = 0
        status = "completed"
        for tick in range(math.ceil(args.duration / env.step_dt)):
            if follower and follower.status != "tracking":
                status = follower.status
                break
            actions = policy(obs)
            if not bool(torch.isfinite(actions).all()):
                status = "invalid_action"
                break

            command = env.command_manager.get_command("twist")[0].tolist()
            elapsed = (tick + 1) * env.step_dt
            obs, _, done, _ = wrapped.step(actions)
            point, heading = pose(robot, env)
            vx, vy = robot.data.root_link_lin_vel_b[0, :2].tolist()
            yaw = float(robot.data.root_link_ang_vel_b[0, 2])
            if not all(map(math.isfinite, (*point, heading, vx, vy, yaw))):
                status = "invalid_state"
                break
            linear_sq += (vx - command[0]) ** 2 + (vy - command[1]) ** 2
            yaw_sq += (yaw - command[2]) ** 2
            linear_reward += float(env.reward_manager.get_term("track_linear_velocity")[0])
            yaw_reward += float(env.reward_manager.get_term("track_angular_velocity")[0])
            for i, value in enumerate((vx, vy, yaw)):
                velocity_sum[i] += value
            samples += 1
            distance += math.dist(previous, point)
            yaw_displacement += angle_error(heading, previous_heading)
            dx, dy = point[0] - start[0], point[1] - start[1]
            lateral = -math.sin(initial_heading) * dx + math.cos(initial_heading) * dy
            lateral_peak = max(lateral_peak, abs(lateral))
            previous, previous_heading = point, heading
            if bool(done[0]):
                status = "terminated"
                break
            if follower and follower.status != "tracking":
                status = follower.status
                break
        if follower and status == "completed":
            status = "timeout"
        return {
            "status": status,
            "seconds": elapsed,
            "initial_xy_m": start,
            "final_xy_m": previous,
            "traveled_m": distance,
            "peak_lateral_displacement_m": lateral_peak,
            "heading_change_deg": math.degrees(yaw_displacement),
            "planar_velocity_rmse_mps": math.sqrt(linear_sq / samples) if samples else None,
            "yaw_rate_rmse_radps": math.sqrt(yaw_sq / samples) if samples else None,
            "mean_weighted_tracking_reward": {
                "planar_velocity": linear_reward / samples if samples else None,
                "yaw_rate": yaw_reward / samples if samples else None,
            },
            "mean_twist_body": [value / samples for value in velocity_sum] if samples else None,
            "last_twist_body": (vx, vy, yaw) if samples else None,
        }

    try:
        runner = VelocityOnPolicyRunner(wrapped, asdict(agent_cfg), device=args.device)
        runner.load(str(args.checkpoint), load_cfg={"actor": True}, strict=True, map_location=args.device)
        policy = runner.get_inference_policy(device=args.device)
        with torch.inference_mode():
            if map_ is not None and path is not None:
                follower = PathFollower(
                    map_, path, speed=args.speed, yaw_rate=args.yaw_rate,
                    tolerance=args.goal_tolerance, max_tracking_error=args.max_tracking_error,
                )
                report = episode([])
                report.update(
                    waypoints_xy_m=path,
                    goal_error_m=math.dist(report["final_xy_m"], map_.goal),
                    peak_tracking_error_m=follower.peak_tracking_error,
                    waypoint_index=follower.next_index,
                )
                return report
            cases = _benchmark_cases(FORWARD_SPEED, NAV_YAW_RATE / 2, args.duration / 2)
            return {name: {"command_schedule": case, **episode(case)} for name, case in cases.items()}
    finally:
        wrapped.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--checkpoint", type=Path, required=True)
    common.add_argument("--profile", choices=("forward", "navigation"), default="navigation")
    common.add_argument("--device", default="cuda:0")
    common.add_argument("--output", type=Path, required=True)
    modes = parser.add_subparsers(dest="mode", required=True)
    benchmark = modes.add_parser("benchmark", parents=[common], help="Run the fixed-command tracking suite")
    benchmark.add_argument("--duration", type=float, default=10.0)
    benchmark.add_argument("--seed", type=int, default=42)
    navigate = modes.add_parser("navigate", parents=[common], help="Plan and follow an RRT* route")
    add_planning_args(navigate)
    navigate.add_argument("--duration", type=float, default=60.0)
    navigate.add_argument("--speed", type=float, default=0.35)
    navigate.add_argument("--yaw-rate", type=float, default=0.6)
    navigate.add_argument("--goal-tolerance", type=float, default=0.07)
    navigate.add_argument("--max-tracking-error", type=float, default=0.18)
    args = parser.parse_args()
    if not args.checkpoint.is_file():
        parser.error(f"checkpoint not found: {args.checkpoint}")
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("duration must be finite and positive")
    map_, path = None, None
    try:
        if args.mode == "navigate":
            if not (0 < args.speed <= 0.5 and 0 < args.yaw_rate <= 0.6):
                raise ValueError("speed and yaw rate must be within the navigation training ranges")
            if not (
                0 < args.goal_tolerance <= args.max_tracking_error
                and math.isfinite(args.max_tracking_error)
            ):
                raise ValueError("tracking error limit must be finite and at least the positive goal tolerance")
            map_ = PlanningMap.load(args.map, args.clearance)
            path = plan_path(map_, step_size=args.step_size, iterations=args.iterations, seed=args.seed)
        report = (
            {"status": "search_exhausted"}
            if map_ is not None and path is None
            else run(args, map_, path)
        )
        result = {
            "checkpoint": str(args.checkpoint),
            "profile": args.profile,
            "seed": args.seed,
            "mode": args.mode,
            "result": report,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(args.output)
    except (OSError, ValueError, OverflowError) as exc:
        parser.error(str(exc))
    return 0 if args.mode == "benchmark" or report.get("status") == "reached_goal" else 1


if __name__ == "__main__":
    raise SystemExit(main())
