# humanoid_full_body_description

Full-body CAD URDF (`urdf/full_body.urdf`): the upper body plus the articulated lower body at the nominal five-hole
pelvis mount. Every link uses its original STL meshes (`meshes/`, 1,086 files) for both visual and collision geometry.
The root link is `torso_link`.

For MoveIt 2 / RViz2 inspection, see [humanoid_full_body_moveit_config](../humanoid_full_body_moveit_config/README.md).

## Joint zero

At `q = 0` the robot stands upright: legs straight, knee axes lateral, hip-yaw axes vertical, soles flat,
toes forward (−y in `torso_link`), and arms hanging straight down.
In the source CAD, the legs were posed rather than straight. The right knee was bent about 29°, the left leg
was yawed about 33° and pitched about 20°, and the right hip was rolled about 8°. These offsets were moved into
the `rpy` of the 10 leg joint origins, so that the old pose equals the new joint values below:

| Joint | Old pose in new coordinates (deg) |
|---|---|
| left_pelvis_hip_connector / hip / thigh / shank / foot | −19.7 / 0.0 / 32.5 / 29.5 / 5.7 |
| right_pelvis_hip_connector / hip / thigh / shank / foot | −1.9 / 7.8 / 0.9 / −29.2 / 2.5 |

Link geometry and joint positions are unchanged. Only the child frames are rotated about each joint axis.
The arm joints were already at a sensible zero (hanging down), so they were not changed.
