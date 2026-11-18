"""Advisory self-collision monitor for measured and policy target leg poses."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution(
            [FindPackageShare('humanoid_safety'), 'config', 'self_collision.yaml'])),
        DeclareLaunchArgument('backend', default_value='pinocchio'),
        # Bench mode: use /policy/joint_states for measured feedback.
        DeclareLaunchArgument('measured_topic', default_value='/joint_states'),
        DeclareLaunchArgument('target_topic', default_value='/policy/target_angles'),
        Node(package='humanoid_safety', executable='self_collision_monitor_node',
             name='self_collision_monitor_node', output='screen',
             parameters=[LaunchConfiguration('config'), {
                 'backend': LaunchConfiguration('backend'),
                 'measured_topic': LaunchConfiguration('measured_topic'),
                 'target_topic': LaunchConfiguration('target_topic'),
             }]),
    ])
