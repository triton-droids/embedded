# humanoid_leg_moveit_config (legacy)

Retained MoveIt 2 configuration for `humanoid_leg_description`. The source is
under `legacy/`; the ROS package name remains `humanoid_leg_moveit_config`.

The demo loads the installed leg URDF
(`humanoid_leg_description/urdf/human_offset_corrected.urdf`) and this package's
`config/humanoid_leg.srdf`. Planning groups are `left_leg` and `right_leg`.

## Build and launch

Run from the repository root with the required ROS/MoveIt packages installed:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to humanoid_leg_moveit_config --cmake-clean-cache
source install/setup.bash
ros2 launch humanoid_leg_moveit_config demo.launch.py
```

`--cmake-clean-cache` refreshes source-directory paths in any existing build
cache after relocation. This standalone demo is outside the active policy/CAN
launch composition.
