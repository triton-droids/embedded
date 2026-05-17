from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ---- args ----
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='default',
        description="Which set of nodes to launch: 'default', 'test', or 'demo'"
    )
    enable_can_arg = DeclareLaunchArgument(
        'enable_can',
        default_value='false',
        description="true/false: bring up can0 and start motor nodes"
    )
    enable_imu_i2c_arg = DeclareLaunchArgument(
        'enable_imu_i2c',
        default_value='false',
        description="true/false: start IMU DR node reading /dev/i2c-*"
    )

    mode = LaunchConfiguration('mode')
    enable_can = LaunchConfiguration('enable_can')
    enable_imu_i2c = LaunchConfiguration('enable_imu_i2c')

    pkg_share = get_package_share_directory('motor_control_node')

    config_file = os.path.join(pkg_share, 'config', 'robstride_motors.yaml')
    setup_can_script = os.path.join(pkg_share, 'launch', 'setup_can0.sh')

    # ---- conditions (string compare) ----
    enable_can_cond = IfCondition(PythonExpression(["'", enable_can, "' == 'true'"]))
    enable_imu_i2c_cond = IfCondition(PythonExpression(["'", enable_imu_i2c, "' == 'true'"]))

    def mode_and_can(target: str):
        return IfCondition(PythonExpression([
            "('", mode, "' == '", target, "') and ('", enable_can, "' == 'true')"
        ]))

    # ---- CAN bring-up (only when enable_can==true) ----
    setup_can = ExecuteProcess(
        cmd=['sudo', 'bash', setup_can_script],
        output='screen',
        condition=enable_can_cond
    )

    # ---- motor nodes (only when enable_can==true) ----
    motor_service_node = Node(
        package='motor_control_node',
        executable='motor_service',
        name='motor_control_node_debug',
        output='screen',
        parameters=[{'motor_config_file': config_file}],
        condition=enable_can_cond,
    )

    motor_control_node = Node(
        package='motor_control_node',
        executable='motor_control',
        name='controller_node',
        output='screen',
        parameters=[{'default_mode': 'position_pp'}],
        condition=mode_and_can('default'),
    )

    test_node = Node(
        package='motor_control_node',
        executable='motor_control_test',
        name='ros2test_node',
        output='screen',
        condition=mode_and_can('test'),
    )

    demo_node = Node(
        package='motor_control_node',
        executable='motor_control_demo',
        name='demo_node',
        output='screen',
        parameters=[config_file],
        condition=mode_and_can('demo'),
    )

    # ---- IMU DR node (only when enable_imu_i2c==true) ----
    imu_dr_node = Node(
        package='motor_control_node',
        executable='imu_dr_rk4_node',
        name='imu_dr_rk4',
        output='screen',
        parameters=[{
            'source': 'i2c',
            'i2c_bus': 1,
            'i2c_addr': 104,      # 0x68
            'i2c_rate_hz': 200.0,
            'acc_includes_gravity': True,
            'orientation_correction_gain': 0.0,
        }],
        condition=enable_imu_i2c_cond,
    )

    # ---- start motor nodes after setup_can exits ----
    nodes_to_start_after_can = [
        motor_service_node,
        motor_control_node,
        test_node,
        demo_node,
    ]

    start_nodes_after_can = RegisterEventHandler(
        OnProcessExit(
            target_action=setup_can,
            on_exit=nodes_to_start_after_can
        ),
        condition=enable_can_cond,
    )

    return LaunchDescription([
        mode_arg,
        enable_can_arg,
        enable_imu_i2c_arg,

        setup_can,
        start_nodes_after_can,

        imu_dr_node,
    ])
