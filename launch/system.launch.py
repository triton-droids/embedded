"""Start active control, vision and leg display modules without MoveIt."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package, filename, arguments, enabled=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', filename,
        ])),
        launch_arguments=arguments.items(),
        condition=IfCondition(LaunchConfiguration(enabled)) if enabled else None,
    )


def generate_launch_description():
    defaults = [
        ('enable_rt', 'true', 'Request RR scheduling; permission failure logs a warning'),
        ('can_tx_rt_priority', '20', 'Event-driven CAN TX priority, 1-99'),
        ('can_rx_rt_priority', '21', 'Continuous per-interface CAN RX priority, 1-99'),
        ('control_rt_priority', '10', 'C++ executor priority, 1-99'),
        ('control_rate_hz', '50.0', 'CAN and C++ control rate'),
        ('enable_fake_motor', 'false', 'Use simulated motors instead of CAN'),
        ('enable_vision', 'true', 'Start vision subsystem'),
        ('start_camera', 'true', 'Start RealSense camera'),
        ('start_agent', 'true', 'Start micro-ROS Agent'),
        ('start_detector', 'true', 'Start detector; false for TRACK_ONLY builds'),
        ('start_tracker', 'true', 'Start head tracking'),
        ('serial_port', '/dev/ttyACM0', 'Head MCU serial port'),
        ('agent_mode', 'native', 'micro-ROS Agent mode: native or docker'),
        ('enable_display', 'true', 'Start leg state publisher and RViz'),
        ('use_joint_state_gui', 'false', 'Use manual joint states instead of live states'),
        ('joint_states_topic', '/joint_states', 'Leg joint state input'),
    ]
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=value, description=description)
        for name, value, description in defaults
    ] + [
        include('motor_control_hybrid', 'hybrid_control.launch.py', {
            'enable_cpp_control': 'true',
            'enable_sdk_gateway': 'false',
            'enable_policy_bridge': 'false',
            'enable_fake_motor': arg('enable_fake_motor'),
            'enable_websocket_ui': 'false',
            'control_rate_hz': arg('control_rate_hz'),
            'can_rt_enabled': arg('enable_rt'),
            'can_tx_rt_priority': arg('can_tx_rt_priority'),
            'can_rx_rt_priority': arg('can_rx_rt_priority'),
            'control_rt_enabled': arg('enable_rt'),
            'control_rt_priority': arg('control_rt_priority'),
        }),
        include('humanoid_vision', 'head_tracking.launch.py', {
            name: arg(name) for name in (
                'start_camera', 'start_agent', 'start_detector', 'start_tracker',
                'serial_port', 'agent_mode',
            )
        }, 'enable_vision'),
        include('humanoid_leg_description', 'display.launch.py', {
            'use_joint_state_gui': arg('use_joint_state_gui'),
            'joint_states_topic': arg('joint_states_topic'),
        }, 'enable_display'),
    ])
