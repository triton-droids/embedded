# humanoid_leg_moveit_config

MoveIt 2 configuration for `humanoid_leg_description`.

Default demo loads:

- `../humanoid_leg_description/urdf/human_offset_corrected.urdf`
- `config/humanoid_leg.srdf`

Planning groups:

- `left_leg`
- `right_leg`

## Build

Build from this package directory:

```bash
source /opt/ros/jazzy/setup.bash
source ../humanoid_leg_description/install/setup.bash
colcon build --symlink-install
```

Launch:

```bash
source ../humanoid_leg_description/install/setup.bash
source install/setup.bash
ros2 launch humanoid_leg_moveit_config demo.launch.py
```
