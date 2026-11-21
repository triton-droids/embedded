#!/usr/bin/env python3
"""Plan (and optionally execute) from the current state to an SRDF group state for every group.

Run against a live move_group, e.g.:
  ros2 launch humanoid_full_body_moveit_config inspect.launch.py use_rviz:=false joint_sliders:=false
  ros2 run humanoid_full_body_moveit_config test_plan.py --state preview

With the MuJoCo backend (backend:=mujoco) add --execute to also run each plan.

Exit code 0 only if every group produced a plan (and, with --execute, executed it).
"""

import argparse
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_path
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient


def group_states(state_name):
    srdf = get_package_share_path("humanoid_full_body_moveit_config") / "config/full_body.srdf"
    states = {}
    for state in ET.parse(srdf).getroot().findall("group_state"):
        if state.attrib["name"] == state_name:
            states[state.attrib["group"]] = {
                joint.attrib["name"]: float(joint.attrib["value"]) for joint in state.findall("joint")
            }
    return states


def plan(client, group, joints, planning_time, execute):
    goal = MoveGroup.Goal()
    request = goal.request
    request.group_name = group
    request.num_planning_attempts = 1
    request.allowed_planning_time = planning_time
    request.max_velocity_scaling_factor = 1.0
    request.max_acceleration_scaling_factor = 1.0
    request.start_state.is_diff = True  # start from move_group's current state
    request.goal_constraints.append(Constraints(joint_constraints=[
        JointConstraint(joint_name=name, position=value, tolerance_above=1e-3,
                        tolerance_below=1e-3, weight=1.0)
        for name, value in joints.items()
    ]))
    goal.planning_options.plan_only = not execute

    node = client._node
    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        return None, "goal rejected"
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=planning_time + 60.0)
    if result_future.result() is None:
        return None, "timed out waiting for result"
    result = result_future.result().result
    return result, result.error_code.val


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--state", default="preview", help="SRDF group_state name to plan to")
    parser.add_argument("--groups", nargs="*", help="subset of groups (default: all with that state)")
    parser.add_argument("--execute", action="store_true", help="execute each plan (needs backend:=mujoco)")
    parser.add_argument("--planning-time", type=float, default=5.0, help="seconds per group")
    args = parser.parse_args()

    states = group_states(args.state)
    groups = args.groups or list(states)
    rclpy.init()
    node = rclpy.create_node("full_body_plan_check")
    client = ActionClient(node, MoveGroup, "move_action")
    if not client.wait_for_server(timeout_sec=60.0):
        print("move_action server not available", file=sys.stderr)
        return 2

    failures = 0
    for group in groups:
        result, code = plan(client, group, states[group], args.planning_time, args.execute)
        if result is not None and code == MoveItErrorCodes.SUCCESS:
            points = len(result.planned_trajectory.joint_trajectory.points)
            print(f"OK    {group:10s} {points} points" + (", executed" if args.execute else ""))
        else:
            failures += 1
            print(f"FAIL  {group:10s} error {code}")
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
