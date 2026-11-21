"""Inspect the supplied URDF in MoveIt 2 with local, synthetic joint states."""

from math import acos, atan2, hypot
import xml.etree.ElementTree as ET

from ament_index_python.packages import PackageNotFoundError, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def missing_meshes(robot):
    packages = {}
    missing = []
    for uri in sorted({mesh.attrib["filename"] for mesh in robot.findall(".//mesh")}):
        package, relative_path = uri.removeprefix("package://").split("/", 1)
        if package not in packages:
            try:
                packages[package] = get_package_share_path(package)
            except PackageNotFoundError:
                packages[package] = None
        share = packages[package]
        if share is None or not (share / relative_path).is_file():
            missing.append(uri)
    return missing


def add_visual(link, name, primitive, dimensions, xyz, rpy, rgba):
    visual = ET.SubElement(link, "visual", name=name)
    ET.SubElement(visual, "origin", xyz=xyz, rpy=rpy)
    ET.SubElement(ET.SubElement(visual, "geometry"), primitive, **dimensions)
    material = ET.SubElement(visual, "material", name=name)
    ET.SubElement(material, "color", rgba=rgba)


def add_skeleton(robot):
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    for name, link in links.items():
        for element in list(link):
            if element.tag in {"visual", "collision"}:
                link.remove(element)
        add_visual(link, name + "_origin", "sphere", {"radius": "0.017"},
                   "0 0 0", "0 0 0", "0.95 0.72 0.22 1")

    for joint in robot.findall("joint"):
        name = joint.attrib["name"]
        parent = links[joint.find("parent").attrib["link"]]
        x, y, z = map(float, joint.find("origin").attrib["xyz"].split())
        length = hypot(x, y, z)
        if length < 0.001:
            continue
        color = "0.62 0.65 0.69 1"
        if name.startswith("left_"):
            color = "0.16 0.70 0.95 1"
        elif name.startswith("right_"):
            color = "0.99 0.43 0.23 1"
        add_visual(parent, name + "_segment", "cylinder",
                   {"radius": "0.010", "length": str(length)},
                   f"{x / 2} {y / 2} {z / 2}",
                   f"0 {acos(z / length)} {atan2(y, x)}", color)


def launch_inspection(context):
    config = get_package_share_path("humanoid_full_body_moveit_config") / "config"
    urdf_name = URDF_FILES[LaunchConfiguration("model").perform(context)]
    urdf = get_package_share_path("humanoid_full_body_description") / "urdf" / urdf_name
    robot = ET.parse(urdf).getroot()
    missing = missing_meshes(robot)
    geometry = LaunchConfiguration("geometry").perform(context)
    if geometry == "cad" and missing:
        samples = ", ".join(missing[:3])
        raise RuntimeError(f"CAD mode requires all STL files. Missing {len(missing)}. Examples: {samples}")

    skeleton = geometry == "skeleton" or bool(missing)
    if skeleton:
        add_skeleton(robot)
        mode = f"Skeleton preview. Missing STL files: {len(missing)}. Self-collision geometry unavailable."
    else:
        mode = "CAD preview. Using the original URDF mesh geometry."

    def read_yaml(name):
        return yaml.safe_load((config / name).read_text(encoding="utf-8"))

    description = {"robot_description": ParameterValue(ET.tostring(robot, encoding="unicode"), value_type=str)}
    model_parameters = {
        **description,
        "robot_description_semantic": ParameterValue(
            (config / "full_body.srdf").read_text(encoding="utf-8"), value_type=str
        ),
        "robot_description_planning": read_yaml("joint_limits.yaml"),
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": read_yaml("ompl_planning.yaml"),
    }
    sliders = LaunchConfiguration("joint_sliders")
    mujoco = LaunchConfiguration("backend").perform(context) == "mujoco"
    use_rviz = LaunchConfiguration("use_rviz").perform(context) == "true"
    rviz = Node(
        package="rviz2", executable="rviz2", name="full_body_rviz", output="screen",
        arguments=["-d", str(config / ("skeleton.rviz" if skeleton else "cad.rviz"))],
        parameters=[model_parameters],
    )
    actions = [
        LogInfo(msg=mode),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[description]),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[model_parameters, {
                 "allow_trajectory_execution": mujoco,
                 "moveit_manage_controllers": False,
                 "publish_robot_description_semantic": True,
                 "publish_planning_scene": True,
                 "publish_geometry_updates": True,
                 "publish_state_updates": True,
                 "publish_transforms_updates": True,
             }, *([read_yaml("moveit_controllers.yaml")] if mujoco else [])]),
    ]
    if mujoco:
        # MuJoCo owns /joint_states and executes trajectories; no joint sliders.
        sim = Node(
            package="humanoid_full_body_moveit_config", executable="mujoco_sim.py", output="screen",
            parameters=[{"viewer": LaunchConfiguration("mujoco_viewer").perform(context) == "true",
                         "urdf": urdf_name}])
        # Closing the MuJoCo viewer also ends the session.
        actions += [sim, RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=[Shutdown()]))]
    else:
        actions += [
            Node(package="joint_state_publisher", executable="joint_state_publisher",
                 condition=UnlessCondition(sliders), output="screen", parameters=[description]),
            Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
                 condition=IfCondition(sliders), output="screen", parameters=[description]),
        ]
    if use_rviz:
        # Closing RViz ends the whole inspection session.
        actions += [rviz, RegisterEventHandler(OnProcessExit(target_action=rviz, on_exit=[Shutdown()]))]
    return actions


# simplified: one decimated visual + one per-part convex collision mesh per link (about 1 GB for the stack).
# detailed: the original 1,086 CAD part meshes (several GB per process; MuJoCo needs about 3 GB).
URDF_FILES = {"simplified": "full_body.urdf", "detailed": "full_body_detailed.urdf"}


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="simplified", choices=list(URDF_FILES),
                              description="simplified meshes, or the detailed per-part CAD meshes"),
        DeclareLaunchArgument("geometry", default_value="auto", choices=["auto", "cad", "skeleton"]),
        DeclareLaunchArgument("joint_sliders", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("backend", default_value="fake", choices=["fake", "mujoco"],
                              description="fake: joint sliders, plan only; mujoco: MuJoCo executes plans"),
        DeclareLaunchArgument("mujoco_viewer", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"],
                              description="false runs move_group headless, e.g. for test_plan.py"),
        OpaqueFunction(function=launch_inspection),
    ])
