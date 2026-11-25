"""Interactive self-collision test: sliders pose the legs, RViz shows flagged links.

Publishes no motor commands. joint_state_publisher_gui drives /joint_states,
which feeds both TF (robot_state_publisher) and the monitor's measured and
target inputs, so all three checks see the slider pose.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('humanoid_safety')
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='pinocchio'),
        DeclareLaunchArgument('joint_states_topic', default_value='/joint_states'),
        DeclareLaunchArgument('use_gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        # robot_state_publisher only; this launch starts its own sliders and RViz.
        # Scoped: include arguments otherwise leak into this file, and its
        # use_rviz:=false would also disable the RViz node below.
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('humanoid_leg_description'), 'launch', 'display.launch.py'])),
            launch_arguments={'use_joint_state_gui': 'false', 'use_rviz': 'false',
                              'joint_states_topic': LaunchConfiguration('joint_states_topic')}.items())]),
        # 30 Hz keeps slider samples well inside the monitor's input timeout.
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(LaunchConfiguration('use_gui')),
             parameters=[{'rate': 30}],
             remappings=[('/joint_states', LaunchConfiguration('joint_states_topic'))],
             output='screen'),
        Node(package='humanoid_safety', executable='self_collision_monitor_node',
             name='self_collision_monitor_node', output='screen',
             parameters=[PathJoinSubstitution([share, 'config', 'self_collision.yaml']), {
                 'backend': LaunchConfiguration('backend'),
                 'measured_topic': LaunchConfiguration('joint_states_topic'),
                 'target_topic': LaunchConfiguration('joint_states_topic'),
                 'publish_markers': True,
                 'marker_check': 'measured',
             }]),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'self_collision.rviz'])]),
    ])
