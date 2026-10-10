"""Publish selected feedback or target angles for a remote RViz computer."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    registry_path = Path(get_package_share_directory('motor_control_hybrid')) / 'config/motors.yaml'
    registry = yaml.safe_load(registry_path.read_text())['motor_control_node']['ros__parameters']
    return LaunchDescription([
        DeclareLaunchArgument('use_fake_motor', default_value='false',
                              description='Start isolated fake motors with robot joint names'),
        Node(package='motor_control_hybrid', executable='fake_motor_node',
             namespace='pose_test', output='screen',
             condition=IfCondition(LaunchConfiguration('use_fake_motor')),
             parameters=[{'joint_names': registry['model_contract']['joint_order'],
                          'publish_rate_hz': 50.0}],
             remappings=[('joint_states', LaunchConfiguration('source_topic'))]),
        DeclareLaunchArgument('source_topic', default_value='/policy/joint_states',
                              description='Feedback topic; use /policy/target_angles for commanded pose'),
        DeclareLaunchArgument('output_topic', default_value='/joint_states'),
        Node(package='motor_control_hybrid', executable='joint_state_monitor_node',
             output='screen', parameters=[{
                 'source_topic': LaunchConfiguration('source_topic'),
                 'output_topic': LaunchConfiguration('output_topic'),
             }]),
    ])
