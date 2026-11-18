# humanoid_leg_description

Humanoid leg description package extracted from `triton-droids/simulation`.

## Contents

- `urdf/`: upstream URDF variants from the Isaac Lab branch
- `meshes/robot_meshes/`: collision and visual STL/OBJ assets
- `launch/display.launch.py`: simple `robot_state_publisher` + RViz demo
- `rviz/display.rviz`: default visualization config

## Default model

The launch file defaults to:

- `urdf/human_offset_corrected.urdf`

That file has a `world` root link, which makes it convenient for RViz.

For live policy or motor joint states from another ROS 2 computer, see
[remote RViz setup](../docs/remote_rviz.md). Disable the manual joint GUI with
`use_joint_state_gui:=false` and select the stream with `joint_states_topic`.
