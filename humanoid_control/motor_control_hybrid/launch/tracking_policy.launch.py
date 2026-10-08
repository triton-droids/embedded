"""IMU -> ONNX tracking -> monitor topics, with an optional isolated C++ bench."""
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    registry_path = Path(get_package_share_directory('motor_control_hybrid')) / 'config/motors.yaml'
    registry = yaml.safe_load(registry_path.read_text())['motor_control_node']['ros__parameters']
    contract = registry['model_contract']
    defaults = {
        'model_path': str(Path(__file__).resolve().parents[3].parent / 'simulation/logs/legs_tracking/'
                          '20260914_170827/20260914_170827.onnx'),
        'port': '/dev/ttyACM0', 'baud': '460800', 'input_format': 'json',
        'start_imu_reader': 'true', 'imu_topic': '/imu/data_raw',
        'joint_states_topic': '/policy/joint_states',
        'joint_feedback_mode': 'zero', 'calibration_seconds': '2.0',
        'use_fake_joint_states': 'false', 'run_cpp_control': 'false',
    }
    args = [DeclareLaunchArgument(name, default_value=value)
            for name, value in defaults.items()]
    cfg = LaunchConfiguration
    names = contract['joint_order']
    reader = Node(
        package='attitude_sensing_pkg', executable='imu_reader_node', output='screen',
        condition=IfCondition(cfg('start_imu_reader')),
        parameters=[{
            'port': cfg('port'), 'baud': ParameterValue(cfg('baud'), value_type=int),
            'frame_id': 'imu_link', 'input_format': cfg('input_format'),
            'acc_units': 'm/s^2', 'gyro_units': 'rad/s',
            'use_rk4_orientation': False, 'use_sensor_orientation': False,
            'publish_odom': False,
        }], remappings=[('/imu/data_raw', cfg('imu_topic'))])
    policy = Node(
        package='motor_control_hybrid', executable='tracking_policy_node', output='screen',
        parameters=[{
            'model_path': cfg('model_path'), 'control_rate_hz': contract['control_hz'],
            'motor_config_file': str(registry_path),
            'imu_topic': cfg('imu_topic'), 'joint_states_topic': cfg('joint_states_topic'),
            'joint_feedback_mode': cfg('joint_feedback_mode'),
            'calibration_seconds': ParameterValue(cfg('calibration_seconds'), value_type=float),
        }])
    fake = Node(
        package='motor_control_hybrid', executable='fake_motor_node', namespace='policy',
        output='screen', condition=IfCondition(cfg('use_fake_joint_states')),
        parameters=[{'joint_names': names, 'publish_rate_hz': 50.0}],
        remappings=[('joint_states', cfg('joint_states_topic'))])
    control = Node(
        package='motor_control_hybrid', executable='cpp_control_node', output='screen',
        condition=IfCondition(cfg('run_cpp_control')),
        parameters=[{'control_rate_hz': 50.0}],
        remappings=[('/joint_states', cfg('joint_states_topic')),
                    ('/desired_motor_subset', '/policy/desired_motor_subset'),
                    ('/desired_velocity_subset', '/policy/desired_velocity_subset'),
                    ('/motor_commands', '/policy/motor_commands')])
    # No CAN, SDK gateway, websocket or automatic MODE_ENABLE is launched here.
    return LaunchDescription(args + [reader, policy, fake, control])
