# Self-collision monitor (advisory)

Tracks issue #46. `humanoid_safety/self_collision_monitor_node` checks leg poses
against the robot's own collision geometry and publishes **warnings only**. It
never publishes commands or an estop, and it is not registered with the latched
`SafetySupervisor`, which would turn a warning into an estop. Model PD gains,
action scaling, polarity, trajectory and enable behavior are unchanged.

## What it checks

Every `check_rate_hz` (default 20 Hz) it checks the latest sample of:

| Check | Input | Meaning |
|---|---|---|
| `measured` | `/joint_states` | Where the legs are |
| `target` | `/policy/target_angles` | Where the policy is pulling them |
| `path` | both | Poses sampled on the straight joint-space line from measured to target |

The policy can't be assumed to follow its reference motion, so both the
measured and target poses are checked. The path check samples poses so that no
joint moves more than `max_joint_step_rad` (0.05 rad) between samples, up to
`max_path_poses`. It is **sampled**, and the straight line only approximates the
real PD motion, so a clear path result is not a proof of continuous-path safety.

Each check is published as a `DiagnosticStatus` on `/safety/self_collision`:

| Level | Meaning |
|---|---|
| `OK` | Every checked pair is at least `clearance_margin_m` apart |
| `WARN` | A pair is closer than the margin |
| `ERROR` | A pair touches or overlaps (distance ≤ 0; negative = penetration) |
| `STALE` | No result: missing, stale, future-stamped or incomplete input, or a failed check |

Inputs must be stamped, and both source age and local receipt age must be within
`input_timeout_s` (0.1 s), so old data is never reported as a clearance figure.
Values include `min_distance_m`, `closest_pair`, `colliding_pairs`,
`close_pairs`, `poses_checked`, `compute_ms` and `input_age_ms`.

Subscriptions keep only the latest sample per topic; nothing queues behind a
slow check. The node runs in its own process, so it cannot block the CAN node or
the policy. A check that overruns its period is logged.

## Geometry and allowed pairs

- Robot: `humanoid_leg_description/urdf/human_offset_corrected.urdf`,
  fixed base. Ground and environment contact are out of scope.
- Adjacent links (each URDF joint's parent and child) are exempt. These are
  exactly the "Adjacent" entries in `humanoid_leg_moveit_config`'s SRDF, which a
  test enforces. The SRDF isn't read by default because that package depends on
  all of MoveIt. `srdf_path` can add one.
- Further exemptions go in `extra_allowed_pairs` in
  `humanoid_safety/config/self_collision.yaml`, each with a written reason.
  Unknown link names stop the node at startup. Currently excluded:
  `hip:left_leg2` and `hip:right_leg2`. They meet around each hip2 joint and
  come within ~1 mm in ordinary poses. Excluded pairs are never reported, even
  if they really collide.

## Running

```bash
ros2 launch humanoid_safety self_collision.launch.py
ros2 topic echo /safety/self_collision
```

In bench mode (`joint_feedback_mode: zero`) the policy's joint input is not real
feedback; pass `measured_topic:=/policy/joint_states` if that is the stream you
want checked.

### Interactive test in RViz

```bash
ros2 launch humanoid_safety self_collision_rviz.launch.py
```

This opens joint sliders and RViz, and publishes no motor commands. The sliders
drive `/joint_states`, which feeds TF and both monitor inputs. In RViz, links in
collision are red and links closer than the margin are yellow. The closest pair
and distance are on `/safety/self_collision` and in the log warnings. Coloured
shapes are the URDF collision geometry, which is what the check uses. They and
the robot model are opaque, because RViz sorts transparent objects as a whole and
overlapping transparent geometry flickers. Shapes are drawn 2% larger about
their centres (`marker_inflation`) so they enclose the visual mesh; many hull
faces lie exactly on it and would otherwise z-fight. The inflation is drawing
only. Parts of a visual mesh that lie outside its hull can still show through.
Markers come from the `measured` check only, because RViz shows one robot state. The monitor publishes them when
`publish_markers` is true, with any backend.

The backend needs `ros-humble-pinocchio` (or the Jazzy equivalent), which also
installs the Coal collision library: `rosdep install --from-paths humanoid_control/humanoid_safety`.

## Backends and benchmarking

Backends implement `CollisionBackend` in `humanoid_safety/self_collision/checker.py` and
are registered in `BACKENDS`; select one with the `backend` parameter. They are
imported only when selected.

| Backend | Method | Runtime dependency |
|---|---|---|
| `pinocchio` (default) | Coal GJK on convex shapes | `ros-humble-pinocchio` |
| `pinocchio_mesh` | Coal BVH mesh-to-mesh distance (reference) | `ros-humble-pinocchio` |
| `moveit` | MoveIt PlanningScene (FCL) via `moveit_py`, one query per pair | `ros-humble-moveit-py` (not in `package.xml`; comparison only) |

The collision meshes are `*_ch.stl` convex hulls, so the convex backend is exact
for them. It is only an approximation for any future non-convex mesh.

The `moveit` backend exists to compare against MoveIt. Humble's Python bindings
return only the minimum distance over all allowed pairs, not per-pair
distances, so it measures each pair separately by allowing all the others in a
collision matrix. A colliding pair reports 0.0 because the bindings cannot
return contact depth, so its "closest pair" in collision is the first colliding
pair, not the deepest. It builds its own minimal SRDF and applies the same
allowed pairs as the other backends, and loads package:// meshes through the
ament index, so `humanoid_leg_description` must be installed and sourced.

```bash
ros2 run humanoid_safety benchmark_self_collision --poses 200 \
  --extra-allowed-pairs "hip:left_leg2, hip:right_leg2"
```

Results in the Humble devcontainer (Intel i7-1355U, 200 random poses within URDF
limits, 53 pairs, two runs agree):

| Backend | Mean | p99 | Collision flags vs `pinocchio` | Clear distances vs `pinocchio` |
|---|---|---|---|---|
| `pinocchio` | 0.13 ms | 0.16 ms | reference | reference |
| `pinocchio_mesh` | 21.3 ms | 31.3 ms | identical | within 1.4 µm |
| `moveit` | 23.3 ms | 35.6 ms | identical | within 7.1 µm |
| `moveit`, one query (overall minimum only) | 1.3 ms | 2.7 ms | — | matches per-pair minimum |

In the monitor node at 20 Hz, a 10-pose path check took 1.1 ms with `pinocchio`
and 230 ms with `moveit`, which dropped the status rate to 5.8 Hz. With a
41-pose path, `moveit` fell to 1.7 Hz and inputs went stale between checks,
because a long check blocks message processing.

The speed difference is convex versus mesh distance, not the library:
`moveit` costs about the same as `pinocchio_mesh`, because MoveIt checks the
meshes as BVH meshes. Even one MoveIt query over all pairs, which cannot say
which pairs are close, costs about 10 times one `pinocchio` evaluation.
`pinocchio` remains the default.

Timing on the robot computer has not been measured.

## Known poses

| Pose | Result |
|---|---|
| All joints at zero | Clear; closest pair is torso to each upper hip link, 0.0215 m |
| Both `hip2` joints at their inward limit (±0.436 rad) | Collision; shins overlap by 0.0955 m |

The standing clearance of 2.15 cm means a margin above that warns constantly.
The 0.01 m default is a placeholder until the margin and allowed-pair audit.

## Not yet done (issue #46)

- Verify joint names, axes, zero positions, mesh units/origins and collision
  geometry against the physical robot.
- Confirm the URDF matches the MuJoCo XML the policy was trained on; if not, the
  same angles describe different poses.
- Audit allowed-contact pairs and choose the clearance margin, e.g. by sweeping
  the policy's reference motion.
- Measure timing on the robot computer and against any other candidate backend.
