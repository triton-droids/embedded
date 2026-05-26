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
