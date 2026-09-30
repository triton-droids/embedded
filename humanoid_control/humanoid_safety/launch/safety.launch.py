from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "safety_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("humanoid_safety"), "config", "safety.yaml"]
        ),
        description="Safety node parameter file",
    )
    node = Node(
        package="humanoid_safety",
        executable="safety_node",
        name="safety_node",
        output="screen",
        parameters=[LaunchConfiguration("safety_config")],
    )
    return LaunchDescription([config_arg, node])
