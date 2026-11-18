import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return file.read()


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to start RViz2.",
    )
    use_ros2_control_arg = DeclareLaunchArgument(
        "use_ros2_control",
        default_value="true",
        description="Whether to start ros2_control controllers.",
    )

    robot_description = {
        "robot_description": load_file(
            "humanoid_leg_description",
            "urdf/human_offset_corrected.urdf",
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": load_file(
            "humanoid_leg_moveit_config",
            "config/humanoid_leg.srdf",
        )
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "humanoid_leg_moveit_config",
            "config/kinematics.yaml",
        )
    }
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "humanoid_leg_moveit_config",
            "config/joint_limits.yaml",
        )
    }
    ompl_planning = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": load_yaml("humanoid_leg_moveit_config", "config/ompl_planning.yaml"),
    }
    trajectory_execution = load_yaml("humanoid_leg_moveit_config", "config/moveit_controllers.yaml")
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    ros2_controllers_path = os.path.join(
        get_package_share_directory("humanoid_leg_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_ros2_control")),
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[ros2_controllers_path, robot_description],
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
    )

    left_leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_leg_controller", "-c", "/controller_manager"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
    )

    right_leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_leg_controller", "-c", "/controller_manager"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning,
            trajectory_execution,
            planning_scene_monitor_parameters,
        ],
    )

    rviz_base = os.path.join(
        get_package_share_directory("humanoid_leg_moveit_config"),
        "launch",
    )
    rviz_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning,
        ],
    )

    immediate_moveit = GroupAction(
        actions=[move_group, rviz_node],
        condition=UnlessCondition(LaunchConfiguration("use_ros2_control")),
    )

    delayed_moveit = TimerAction(
        period=5.0,
        actions=[move_group, rviz_node],
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
    )

    delayed_spawners = TimerAction(
        period=2.0,
        actions=[
            joint_state_broadcaster_spawner,
            left_leg_controller_spawner,
            right_leg_controller_spawner,
        ],
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            use_ros2_control_arg,
            robot_state_publisher,
            joint_state_publisher_gui,
            ros2_control_node,
            delayed_spawners,
            immediate_moveit,
            delayed_moveit,
        ]
    )
