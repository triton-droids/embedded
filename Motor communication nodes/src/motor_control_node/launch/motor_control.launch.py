from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch argument: mode = default | test | demo
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='default',
        description="Which set of nodes to launch: 'default', 'test', or 'demo'"
    )

    mode = LaunchConfiguration('mode')

    # Path to YAML config (must exist inside the package)
    config_file = os.path.join(
        get_package_share_directory('motor_control_node'),
        'config',
        'robstride_motors.yaml'
    )

    # Helper: condition "mode == '<target>'"
    def mode_is(target: str):
        # Builds expression like: 'default' == 'default'
        return IfCondition(PythonExpression(["'", mode, f"' == '{target}'"]))

    # ========= Nodes =========

    # Low-level CAN / service node (motor_service) in "default" mode
    motor_service_node = Node(
    package='motor_control_node',
    executable='motor_service',
    name='motor_control_node',
    output='screen',
    parameters=[
        {'motor_config_file': config_file},   # ⬅ 把路径作为参数传进去
    ],
)

    # High-level controller node (motor_control) in "default" mode
    motor_control_node = Node(
        package='motor_control_node',
        executable='motor_control',          # <--- uses motor_control
        name='controller_node',
        output='screen',
        parameters=[{'default_mode': 'position_pp'}],
        condition=mode_is('default'),
    )

    # Test node in "test" mode
    test_node = Node(
        package='motor_control_node',
        executable='motor_control_test',
        name='ros2test_node',
        output='screen',
        condition=mode_is('test'),
    )

    # Demo node in "demo" mode
    demo_node = Node(
        package='motor_control_node',
        executable='motor_control_demo',
        name='demo_node',
        output='screen',
        parameters=[config_file],
        condition=mode_is('demo'),
    )

    return LaunchDescription([
        mode_arg,
        motor_service_node,
        motor_control_node,
        test_node,
        demo_node,
    ])
