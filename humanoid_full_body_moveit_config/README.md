# humanoid_full_body_moveit_config

MoveIt 2 (Jazzy) inspection setup for the full-body CAD URDF in
[`humanoid_full_body_description`](../humanoid_full_body_description/urdf/full_body.urdf.xacro):
RViz2 MotionPlanning and joint-space planning. Plans execute in MuJoCo when `backend:=mujoco` is set.

- Fixed base: `torso_link` is the URDF root, no virtual joint.
- 20 active joints. Planning groups: `left_arm`, `right_arm`, `left_leg`, `right_leg`,
  `both_arms`, `both_legs`, `whole_body`. Group states: `zero`, `preview`.
- Joint-space goals only. The arms have 5 DOF, so no IK / pose goals (no `kinematics.yaml`).
- Trajectory timing uses placeholder limits of 0.5 rad/s and 0.5 rad/s² (`config/joint_limits.yaml`), not hardware limits.
- The default `backend:=fake` is plan only: joint sliders, `allow_trajectory_execution` false.
  `backend:=mujoco` executes plans in MuJoCo (see below).

## Run

```bash
colcon build --packages-select humanoid_full_body_description humanoid_full_body_moveit_config
source install/setup.bash
ros2 launch humanoid_full_body_moveit_config inspect.launch.py
```

To plan in RViz: in **MotionPlanning → Planning**, set Planning Group, set Start State to `<current>`
and Goal State to `preview`, then click **Plan**.

| Argument | Default | Meaning |
|---|---|---|
| `model` | `simplified` | `simplified` = one decimated visual and convex-hull collision mesh per link; `detailed` = the original 1,086 CAD part meshes (several GB per process) |
| `geometry` | `auto` | `cad` = the link meshes (fails if any are missing); `skeleton` = spheres and cylinders at the joint frames, with no collision geometry; `auto` = CAD when every mesh is present |
| `joint_sliders` | `true` | `joint_state_publisher_gui` sliders; `false` = plain `joint_state_publisher` |
| `backend` | `fake` | `fake` = joint sliders, plan only; `mujoco` = MuJoCo publishes `/joint_states` and executes plans |
| `mujoco_viewer` | `true` | MuJoCo viewer window (with `backend:=mujoco`) |
| `use_rviz` | `true` | `false` runs move_group headless |

The URDF has one simplified visual mesh and one convex-hull collision mesh per link.
The whole stack (RViz, move_group, MuJoCo) uses about 1 GB.

## Execute in MuJoCo

```bash
source scripts/setup_rosenv.sh      # provides the mujoco Python module
source install/setup.bash
ros2 launch humanoid_full_body_moveit_config inspect.launch.py backend:=mujoco
```

In RViz, **Plan & Execute** (or **Plan**, then **Execute**) moves the robot in the MuJoCo window.
Closing either window ends the session.
[`scripts/mujoco_sim.py`](scripts/mujoco_sim.py) does the following:

- Loads the same URDF with `torso_link` welded to the world.
  It copies the SRDF `<disable_collisions>` pairs into MuJoCo `<exclude>` pairs, so the robot is
  contact-free at the zero pose. Other self-contacts stay active.
- Drives every joint with a position servo (`kp` 200 N·m/rad, `kv` 8 N·m·s/rad) and applies gravity compensation (`gravity_compensation`).
  The joint armature of 0.01 kg·m² and damping of 0.5 N·m·s/rad are placeholders, not motor data.
  Uses the `implicitfast` integrator, because the small distal inertias diverge with explicit damping.
- Serves `control_msgs/FollowJointTrajectory` on `<group>_controller/follow_joint_trajectory` for the
  four limbs ([`config/moveit_controllers.yaml`](config/moveit_controllers.yaml)). MoveIt splits
  `both_*` and `whole_body` trajectories across them. A goal succeeds once every joint is within
  `goal_tolerance` (0.02 rad) of the last point.
- Runs physics in wall-clock real time and publishes `/joint_states` at 100 Hz. It publishes no `/clock`.

## Check that planning works

```bash
ros2 launch humanoid_full_body_moveit_config inspect.launch.py use_rviz:=false joint_sliders:=false &
ros2 run humanoid_full_body_moveit_config test_plan.py            # every group -> preview
ros2 run humanoid_full_body_moveit_config test_plan.py --state zero --groups left_arm
```

The script exits non-zero if any group fails to plan.
With `backend:=mujoco`, add `--execute` to also execute each plan:

```bash
ros2 launch humanoid_full_body_moveit_config inspect.launch.py backend:=mujoco use_rviz:=false mujoco_viewer:=false &
ros2 run humanoid_full_body_moveit_config test_plan.py --execute
```

## Self-collision matrix

The `<disable_collisions>` entries in `config/full_body.srdf` come from MoveIt's
`collisions_updater`: 21 adjacent pairs plus 101 pairs that never collided in 20,000 random samples
(computed on the convex-hull collision meshes). Every adjacent pair touches at the zero pose. Without these entries,
`CheckStartStateCollision` rejects every plan request, reporting "21 contact(s) detected".
Regenerate the matrix after any change to the URDF geometry:

```bash
collisions_updater=/opt/ros/$ROS_DISTRO/lib/moveit_setup_assistant/collisions_updater
$collisions_updater \
  --urdf install/humanoid_full_body_description/share/humanoid_full_body_description/urdf/full_body.urdf \
  --srdf humanoid_full_body_moveit_config/config/full_body.srdf \
  --output humanoid_full_body_moveit_config/config/full_body.srdf \
  --default --always --trials 20000
```

## Known limitations

- Collision shapes are one convex hull per link, so they fill concave regions (for example, the torso frame is a solid
  block). This is conservative: a plan may be rejected for a contact the real parts would not make.
- 16 of the 20 joints are `continuous` in the URDF, including the hips, knees and ankles,
  so the planner treats them as able to rotate without limit. Plans are kinematic previews only, not motions the robot can make.
  The MuJoCo servo gains and gravity compensation are simulation choices, not the real actuators.
- move_group segfaults on shutdown in `rclcpp::CallbackGroup::~CallbackGroup()` (exit code -11).
  This is a MoveIt/Jazzy shutdown issue. It happens after the session ends and does not affect planning.
