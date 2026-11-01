"""RGB-D -> bounded reference warp -> WBC -> C++ -> MuJoCo experiment."""
from pathlib import Path
import os
import shutil
import subprocess
import json
import shlex
import importlib.util

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            SetEnvironmentVariable, SetLaunchConfiguration, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import yaml


def scheduling_actions(context):
    spec = importlib.util.spec_from_file_location('chase_priority_profile', Path(__file__).resolve().parent/'priority_profile.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    mode = LaunchConfiguration('priority_mode').perform(context)
    profile = module.make_profile(mode)
    print('[priority-profile] resolved='+json.dumps(profile), flush=True)
    return [SetEnvironmentVariable('HUMANOID_PRIORITY_CONFIG', json.dumps(profile) if profile else ''),
            SetEnvironmentVariable('HUMANOID_THREAD_SCHEDULER', 'priority_profile' if profile else ''),
            SetLaunchConfiguration('launch-prefix', shlex.join([
                shutil.which('python3'), str(Path(__file__).resolve().parent/'priority_profile.py'), '--']) if profile else '')]


def generate_launch_description():
    here = Path(__file__).resolve().parent
    ball = Path(get_package_share_directory('humanoid_ball_chase'))
    motor = Path(get_package_share_directory('motor_control_hybrid'))
    wbc = Path(get_package_share_directory('wbc_control_node'))
    # ros2 launch itself uses system Python; native Pinocchio lives in rosenv.
    subprocess.run([shutil.which('python3'), str(here/'wbc_payload.py')],
                   check=True, capture_output=True, text=True)
    urdf = here/'generated/camera_payload.urdf'
    registry = str(motor/'config/motors.yaml')
    config = yaml.safe_load((ball/'config/scene.yaml').read_text())
    config['mujoco_sim']['ros__parameters'].update(
        foot_clearance_m=0.0, scene_nodes='chase_scene:create_nodes',
        publish_feet_ground_truth=True, ground_truth_rate_hz=100.0,
        allow_torque_only_motion=True, dedicated_command_executor=True,
        dedicated_imu_executor=True)
    config['sim_motor_backend']['ros__parameters']['publish_rate_hz'] = 200.0
    config['sim_body_imu'] = {'ros__parameters': {'gc_diagnostics': True}}
    generated = here/'generated'
    generated.mkdir(exist_ok=True)
    scene_config = generated/'chase_scene.yaml'
    scene_config.write_text(yaml.safe_dump(config, sort_keys=False))
    arg = LaunchConfiguration
    wrapper = [shutil.which('python3'), str(here/'priority_profile.py'), '--']
    return LaunchDescription([
        DeclareLaunchArgument('priority_mode', default_value='auto', choices=['auto','nice','fifo','off']),
        DeclareLaunchArgument('start_chase', default_value='false',
                              description='Enable through C++ and release only after valid chase commands'),
        DeclareLaunchArgument('use_viewer', default_value='true'),
        DeclareLaunchArgument('render_camera', default_value='true',
                              description='Disable only for timing diagnosis; chase then has no images'),
        DeclareLaunchArgument('dds_publication_mode', default_value='ASYNCHRONOUS',
                             choices=['ASYNCHRONOUS', 'SYNCHRONOUS']),
        DeclareLaunchArgument('localhost_only', default_value='0',
                              description='0 allows DDS discovery beyond this host; 1 confines it'),
        DeclareLaunchArgument('ros_domain_id', default_value='182'),
        DeclareLaunchArgument('web_port', default_value='8182'),
        DeclareLaunchArgument('max_walk_s', default_value='8.0'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', arg('ros_domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', arg('localhost_only')),
        SetEnvironmentVariable('PYTHONPATH', [str(here), os.pathsep,
                                             EnvironmentVariable('PYTHONPATH', default_value='')]),
        SetEnvironmentVariable('RMW_FASTRTPS_PUBLICATION_MODE', arg('dds_publication_mode')),
        OpaqueFunction(function=scheduling_actions),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('humanoid_sim'))/'launch/sim.launch.py')),
            launch_arguments={
                'enable_policy': 'false', 'connect_policy': 'false', 'auto_start': 'false',
                'enable_wbc': 'false',  # Experimental WBC with camera payload is assembled below.
                'enable_vision': arg('render_camera'), 'enable_depth': 'true', 'start_detector': 'false',
                'vision_target_timeout_s': '1.0',
                'control_rate_hz': '200.0',
                'sim_config_file': str(scene_config),
                'scene_builder': 'chase_scene:build_scene',
                'use_viewer': arg('use_viewer'), 'ros_domain_id': arg('ros_domain_id'),
                'localhost_only': arg('localhost_only'),
                'dds_publication_mode': arg('dds_publication_mode'),
                'web_port': arg('web_port'),
            }.items()),
        ExecuteProcess(cmd=wrapper+[shutil.which('python3'), str(here/'chase_policy.py'), '--ros-args',
                            '-p', 'motor_config_file:='+registry,
                            '-p', 'model_path:='+str(motor/'models/20260914_170827/20260914_170827.onnx'),
                            '-p', 'joint_feedback_mode:=joint_states',
                            '-p', 'wait_for_sim_release:=true',
                            '-p', 'reference_end_behavior:=loop',
                            '-p', ['experiment_max_walk_s:=', arg('max_walk_s')]], prefix='', output='screen'),
        Node(package='wbc_control_node', executable='wbc_control_node',
             prefix=shlex.join(wrapper+[shutil.which('python3')]), output='screen',
             additional_env={'OPENBLAS_NUM_THREADS':'1', 'OMP_NUM_THREADS':'1', 'MKL_NUM_THREADS':'1'},
             parameters=[str(wbc/'config/wbc.yaml'), {'motor_config_file':registry,
                 'urdf_path':str(urdf), 'output_topic':'/desired_motor_subset',
                 'tracking_mode':'policy_impedance', 'max_policy_torque_correction_nm':0.02}]),
        Node(package='humanoid_ball_chase', executable='ball_chase_vision', output='screen',
             parameters=[{'target_timeout_s':1.0, 'stop_distance':1.0}]),
        Node(package='humanoid_vision', executable='track_node', name='pitch_tracker',
             output='screen', parameters=[{'axis':'pitch', 'command_topic':'/head/tilt_target_rad',
                'direction':1.0, 'initial_angle_rad':0.0, 'target_timeout_s':1.0,
                'max_speed_rad_s':0.3, 'max_accel_rad_s2':0.6}]),
        Node(package='humanoid_sim', executable='sim_autostart', output='screen',
             condition=IfCondition(arg('start_chase')), parameters=[{
                 'enable_via_control_core':True, 'policy_topic':'/desired_motor_subset',
                 'startup_timeout_s':10.0, 'release_delay_s':0.1}]),
    ])
