from pathlib import Path
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _build_rviz_config() -> str:
    return """Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 1
      Class: rviz_default_plugins/Grid
      Name: Grid
      Plane Cell Count: 20
      Plane Normal: Z
      Reference Frame: <Fixed Frame>
      Enabled: true
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Visual Enabled: true
      Value: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: world
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.5
      Focal Point:
        X: 0
        Y: 0
        Z: 0.3
      Name: Current View
      Pitch: 0.5
      Yaw: 0.8
Window Geometry:
  Height: 900
  Width: 1600
"""


def _launch_setup(context, *args, **kwargs):
    model_path = LaunchConfiguration("model").perform(context)
    rviz_config_path = Path(tempfile.gettempdir()) / "humanoid_leg_description_display.rviz"
    rviz_config_path.write_text(_build_rviz_config(), encoding="utf-8")

    robot_description = {
        "robot_description": ParameterValue(Command(["cat ", model_path]), value_type=str)
    }

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("use_joint_state_gui")),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_config_path)],
            output="screen",
        ),
    ]


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("humanoid_leg_description"),
                "urdf",
                "human_offset_corrected.urdf",
            ]
        ),
        description="Absolute path to the robot URDF file",
    )

    use_joint_state_gui_arg = DeclareLaunchArgument(
        "use_joint_state_gui",
        default_value="true",
        description="Launch joint_state_publisher_gui for manual joint tweaking",
    )

    return LaunchDescription([
        model_arg,
        use_joint_state_gui_arg,
        OpaqueFunction(function=_launch_setup),
    ])
