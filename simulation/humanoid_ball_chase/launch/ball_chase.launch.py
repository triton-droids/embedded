"""Camera/ball pursuit integration.

controller:=policy (default): the bundled tracking policy does not consume
velocity commands. controller:=model_walk: the model-based walking controller
follows /ball_chase/cmd_vel (forward speed and yaw rate, converted per step).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg = LaunchConfiguration
    # With the world model the head follows the detection when seen and the prediction while coasting.
    head_target = PythonExpression(["'/ball_chase/head_target' if '", arg('world_model'),
                                    "' == 'true' else '/vision/target_center'"])
    return LaunchDescription([
        DeclareLaunchArgument('use_viewer', default_value='true'),
        DeclareLaunchArgument('controller', default_value='policy',
                              description='policy (no pursuit locomotion) or model_walk (follows /ball_chase/cmd_vel)'),
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='Run bundled reference motion; does not enable velocity pursuit'),
        DeclareLaunchArgument('pd_profile', default_value='tuned',
                              description='Policy/C++ PD settings passed to sim.launch.py: tuned or original'),
        DeclareLaunchArgument('ros_domain_id', default_value='61'),
        DeclareLaunchArgument('web_port', default_value='8081'),
        DeclareLaunchArgument('dds_tuning', default_value='on',
                              description='Bound Fast DDS publication blocking in simulation'),
        DeclareLaunchArgument('imu_split', default_value='true',
                              description='Publish body IMU from a separate process'),
        DeclareLaunchArgument('enable_astar', default_value='false',
                              description='Plan paths from external OccupancyGrid and PoseStamped goals'),
        DeclareLaunchArgument('enable_vio', default_value='false',
                              description='Render the passive stereo pair and run vio_localizer (body IMU + '
                                          'cuVSLAM from scripts/run_cuvslam.sh); A* then uses /localization/odom'),
        DeclareLaunchArgument('imu_yaw_error_deg', default_value='0.0',
                              description='Body IMU heading error amplitude (sine, degrees; 0 = off)'),
        DeclareLaunchArgument('imu_yaw_error_period_s', default_value='60.0',
                              description='Period of the body IMU heading error'),
        DeclareLaunchArgument('remote_viewer', default_value='false',
                              description='Stream read-only state to humanoid_sim remote_viewer.py on another computer'),
        DeclareLaunchArgument('remote_viewer_host', default_value='127.0.0.1',
                              description='Stream bind address; use an SSH tunnel or a trusted LAN'),
        DeclareLaunchArgument('remote_viewer_port', default_value='8765'),
        DeclareLaunchArgument('remote_viewer_rate_hz', default_value='15.0'),
        DeclareLaunchArgument('walk_contact_kp', default_value='100.0',
                              description='controller:=model_walk: WBC contact task gain'),
        # Faster than the walking controller's own defaults (0.06 m / 0.08 rad): with the GPU detector
        # (~0.15 s image latency) and the world model the heading loop no longer limit-cycles at
        # 0.15 rad/step; 2026-11-10 Jetson runs: no falls, ball circling at 0.4 m/s kept in view.
        # With vision_device:=cpu (0.4 s+ latency) use 0.06 / 0.08.
        DeclareLaunchArgument('odometry', default_value='ground_truth', choices=['ground_truth', 'leg'],
                              description='Source of odom -> base_footprint -> base_link: ground_truth '
                                          '(sim_odom_tf) or leg (walking_controller leg_odometry: body IMU '
                                          'attitude + stance-foot kinematics, what the robot has)'),
        DeclareLaunchArgument('kick', default_value='false', choices=['true', 'false'],
                              description='Near a still ball line a foot up, stand and kick it (/walk/kick; '
                                          'needs controller:=model_walk and world_model:=true)'),
        DeclareLaunchArgument('walk_max_step_m', default_value='0.08',
                              description='controller:=model_walk: step cap per 0.5 s step (m; node max 0.10)'),
        DeclareLaunchArgument('walk_max_turn_rad', default_value='0.15',
                              description='controller:=model_walk: turn cap per step (rad; node max 0.15)'),
        DeclareLaunchArgument('vision_target_timeout_s', default_value='1.0',
                              description='Maximum image/receipt age for CPU football inference; seconds'),
        DeclareLaunchArgument('scene', default_value='clean', choices=['clean', 'hard'],
                              description='clean: ball on a plain pitch. hard: visual-only clutter (robots, '
                                          'people, lines, goal, boards, shadows) plus camera blur/noise/'
                                          'exposure/JPEG; physics unchanged'),
        DeclareLaunchArgument('vision_device', default_value='gpu', choices=['gpu', 'cpu'],
                              description='gpu: TensorRT FP16 engine next to the ONNX (build with '
                                          'scripts/build_ball_chase_engines.sh); cpu: ONNX Runtime, one thread'),
        DeclareLaunchArgument('vision_model_path', default_value='',
                              description='YOLO11 ONNX for ball detection (class 0 = ball); empty = assets/td-perception.onnx'),
        DeclareLaunchArgument('world_model', default_value='true', choices=['true', 'false'],
                              description='true: track the ball in odom (coast through misses, chase a '
                                          'moving ball a little ahead, head looks at the prediction); '
                                          'false: stateless per-frame chase'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', arg('ros_domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('humanoid_sim'), 'launch', 'sim.launch.py'])), launch_arguments={
                'scene_builder': PythonExpression(["'humanoid_ball_chase.scene:build_scene' + "
                                                   "('_hard' if '", arg('scene'), "' == 'hard' else '')"]),
                'sim_config_file': PathJoinSubstitution([
                    FindPackageShare('humanoid_ball_chase'), 'config',
                    PythonExpression(["'scene_hard.yaml' if '", arg('scene'), "' == 'hard' else 'scene.yaml'"])]),
                'enable_vision': 'true', 'enable_depth': 'true', 'start_detector': 'false',
                'vision_target_timeout_s': arg('vision_target_timeout_s'),
                'head_target_topic': head_target,
                'use_viewer': arg('use_viewer'), 'auto_start': arg('auto_start'),
                'pd_profile': arg('pd_profile'), 'controller': arg('controller'),
                'walk_cmd_vel_topic': PythonExpression(["'/ball_chase/cmd_vel' if '", arg('controller'),
                                                        "' == 'model_walk' else ''"]),
                'walk_contact_kp': arg('walk_contact_kp'),
                'walk_max_step_m': arg('walk_max_step_m'), 'walk_max_turn_rad': arg('walk_max_turn_rad'),
                'odom_tf_robot': PythonExpression(["'true' if '", arg('odometry'), "' == 'ground_truth' else 'false'"]),
                'ros_domain_id': arg('ros_domain_id'), 'web_port': arg('web_port'),
                'dds_tuning': arg('dds_tuning'), 'imu_split': arg('imu_split'),
                'remote_viewer': arg('remote_viewer'), 'remote_viewer_host': arg('remote_viewer_host'),
                'remote_viewer_port': arg('remote_viewer_port'),
                'remote_viewer_rate_hz': arg('remote_viewer_rate_hz'),
                'enable_stereo': arg('enable_vio'),
                'imu_yaw_error_deg': arg('imu_yaw_error_deg'),
                'imu_yaw_error_period_s': arg('imu_yaw_error_period_s'),
            }.items()),
        # Start pose = the spawn placement (known, like a kickoff position), not ground truth.
        Node(package='humanoid_localization', executable='vio_localizer', output='screen',
             condition=IfCondition(arg('enable_vio')),
             parameters=[{'initial_x': 0.0, 'initial_y': 0.0, 'initial_yaw': 0.0}]),
        Node(package='humanoid_ball_chase', executable='ball_chase_vision', output='screen',
             parameters=[{'target_timeout_s':ParameterValue(arg('vision_target_timeout_s'), value_type=float),
                         'model_path':ParameterValue(arg('vision_model_path'), value_type=str),
                         'device':ParameterValue(arg('vision_device'), value_type=str),
                         'world_model':ParameterValue(arg('world_model'), value_type=bool),
                         'kick':ParameterValue(arg('kick'), value_type=bool),
                         # Measured in this scene (2026-11-10): the MuJoCo ball's speed decays at
                         # ~0.06/s, far less than turf (~0.4/s, the tracker default).
                         'tracker.friction':0.06}]),
        Node(package='walking_controller', executable='leg_odometry', prefix='python3', output='screen',
             condition=IfCondition(PythonExpression(["'", arg('odometry'), "' == 'leg'"])),
             parameters=[{'base_forward_axis': 'positive_y'}]),
        Node(package='humanoid_ball_chase', executable='ball_chase_astar',
             condition=IfCondition(arg('enable_astar')), output='screen',
             parameters=[{'odom_topic': PythonExpression(["'/localization/odom' if '", arg('enable_vio'),
                                                          "' == 'true' else '/sim/ground_truth/odom'"])}]),
        Node(package='humanoid_vision', executable='track_node', name='pitch_tracker',
             parameters=[{'axis':'pitch', 'command_topic':'/head/tilt_target_rad',
                          'target_topic':head_target,
                          'direction':1.0, 'initial_angle_rad':0.0,
                          'target_timeout_s':ParameterValue(arg('vision_target_timeout_s'), value_type=float),
                          'max_speed_rad_s':0.3,
                          'max_accel_rad_s2':0.6}], output='screen'),
    ])
