# humanoid_full_body_description

Full-body humanoid URDF: the upper body plus the articulated lower body at the nominal five-hole pelvis mount.
22 links, 20 actuated joints. Fixed base at `torso_link`.

Two variants with identical kinematics and inertials, a light **simplified** model (default) and the original
**detailed** CAD meshes. See [MODELS.md](MODELS.md) for the comparison and when to use which.

## Layout

| Path | What |
|---|---|
| [`urdf/full_body.urdf.xacro`](urdf/full_body.urdf.xacro) | **Source of truth**, about 200 lines: one `cad_link` and one joint macro call per link/joint, grouped by torso / pelvis / arms / legs |
| `urdf/full_body.urdf` (installed only) | Generated at build time by CMake (`xacro`), so consumers keep loading `share/humanoid_full_body_description/urdf/full_body.urdf` |
| `meshes/visual/<link>.stl` | The link's CAD parts, each decimated separately and then concatenated (427k triangles in total). Each part stays within 1 mm of the original (95th percentile); the torso is held to 2 mm so it fits under MuJoCo's 200k-face STL limit. Simple parts such as the two torso battery packs are kept unchanged |
| `meshes/collision/<link>.stl` | One convex piece per CAD part, concatenated (about 2,500 pieces, 140k triangles): screws/nuts (< 5 cm³) as an oriented box, ordinary parts as their convex hull, strongly concave parts (brackets, extrusions) split with CoACD. See "Collision model" below |
| `cad_source/` | Original per-part export: `full_body_parts.urdf` (1,086 `<visual>`/`<collision>` parts, 3.06M triangles) and its STLs. Installed as the **detailed** model `urdf/full_body_detailed.urdf` (CMake option `INSTALL_DETAILED_MODEL`, default ON) |
| [`scripts/simplify_meshes.py`](scripts/simplify_meshes.py) | Regenerates `meshes/` from `cad_source/` |

Before simplifying, the robot had 1,086 separate meshes with 3M triangles. RViz, move_group and MuJoCo each
needed 2–3 GB, and two copies running at once took down a 15 GB desktop. Now the whole MoveIt + MuJoCo stack
uses about 1 GB. Joint origins, axes and inertials are unchanged: link frames and the CAD inertials were kept,
and only the mesh files were replaced. The colours are simplified to one material per link.

Both models have identical links, joints and inertials. Pick one with
`ros2 launch humanoid_full_body_moveit_config inspect.launch.py model:=simplified|detailed`. The detailed model
needs about 2 GB for move_group and about 2.5 GB for MuJoCo.

Regenerate the meshes after a CAD change. These tools are offline only, not runtime dependencies:

```bash
python3 -m venv /tmp/meshenv
/tmp/meshenv/bin/pip install trimesh==4.4.9 fast-simplification==0.1.9 rtree==1.3.0 coacd==1.0.14 scipy
cd humanoid_full_body_description && OMP_NUM_THREADS=1 /tmp/meshenv/bin/python scripts/simplify_meshes.py
```

Then regenerate the MoveIt self-collision matrix (see the MoveIt config README), because the collision shapes changed.

## Collision model

Same idea as the SO-101 work in `lerobot_test`: convex shapes per *part*, not per link. A single hull per link filled the gaps
between parts (the pelvis became a solid block) and wrongly rejected 1.7–4% of random poses. Compared against the detailed
per-part meshes in MoveIt over 900 random poses (±0.8, ±1.6 and ±π rad):

| Collision model | False positives (only the simplified model collides) | Missed collisions |
|---|---|---|
| One convex hull per link (previous) | 4.0% / 1.7% / 3.0% | 0 |
| One hull per part | 3.7% / 1.0% / 0.7% | 0 |
| **Per part + CoACD on concave parts (current)** | **0.7% / 0% / 0.3%** | **0** |

CoACD is approximate: with each piece capped at 64 vertices, a decomposed part's surface can lie up to 2.6 mm outside its pieces
(the script prints this per part). No missed collision showed up in the comparison, but treat contacts closer than ~3 mm as
unresolved. MoveIt's `default_robot_padding` does not apply to self-collision, so it cannot close this gap. One part
(`lower_body_link_002_001`) crashes CoACD on every seed and keeps its plain hull. MuJoCo convexifies each collision STL as a whole,
so in MuJoCo every link is still a single hull.

## Frames

`pelvis_link` is fixed to `torso_link` (`torso_to_pelvis`). The pelvis top face sits flush against the torso
bottom (z = 0), and the hip-pitch axes are exactly ±x. Torso and pelvis together form the reference frame for
the joint zero below.

The STL names in `cad_source/meshes/` are unique across all four folders. Names that clashed carry a
`left_arm_` / `right_arm_` / `lower_body_` prefix. MuJoCo's URDF importer names mesh assets by file basename
and silently reuses the first mesh loaded under a duplicate name. Before the rename, the pelvis rendered as
torso parts and the right arm as left-arm parts.

For MoveIt 2 / RViz2 inspection, see [humanoid_full_body_moveit_config](../humanoid_full_body_moveit_config/README.md).

## Joint zero

At `q = 0` the robot stands upright: legs straight, knee axes lateral, hip-yaw axes vertical, soles flat,
arms hanging straight down. In `torso_link`, +z is up and +y is forward (knee flexion swings the foot toward −y); the
`left_*` joints are on the +x side. This matches the motor registry and the tracking policy's MJCF. With +y forward and +z up,
that side is the robot's anatomical right, so check the left/right labels against the hardware.
In the source CAD, the legs were posed rather than straight. The right knee was bent about 29°, the left leg
was yawed about 33° and pitched about 20°, and the right hip was rolled about 8°. These offsets were moved into
the `rpy` of the 10 leg joint origins, so that the old pose equals the new joint values below:

| Joint | Old pose in new coordinates (deg) |
|---|---|
| left hip1 / hip2 / thigh / knee / ankle | 22.1 / −1.5 / 32.5 / −31.8 / 5.5 |
| right hip1 / hip2 / thigh / knee / ankle | −2.1 / 8.8 / −0.9 / 29.7 / 2.2 |

Link geometry and joint positions are unchanged. Only the child frames are rotated about each joint axis.
The arm joints were already at a sensible zero (hanging down), so they were not changed.

## Mass properties (from the CAD export, not measured)

Total 31.9 kg: torso 9.0, pelvis 3.5, each arm 3.3, each leg 6.4. Height at the zero pose 1.05 m (sole to torso top, no head).

The shoulder mounts (`base_link_333` / `base_link_314`, the three-lobed brackets bolted to the torso frame) were changed
from 6061 aluminium to printed ASA, which made the robot 1.0 kg lighter (−500 g each, 652 → 152 g). The torso mass, COM and inertia were recomputed
from the two parts' geometry (density difference times the part's own volume integrals, then moved to the new COM with the parallel-axis theorem):
mass 10.00 → 9.00 kg, COM z 0.192 → 0.181 m, ixx/iyy/izz 0.148/0.275/0.162 → 0.133/0.231/0.129 kg·m².
The inner round part at each shoulder (`base_link_332` / `313`) is the shoulder motor, not the mount, and was not changed.

The ball and block at the end of each forearm (`link_004_004`, 209 cm³, and `link_004_003`, 142 cm³) were exported as steel; they are
printed PLA at 20% infill. They alone made up 2.75 of the forearm's 3.19 kg. Effective density = 1.24 g/cm³ × (solid shell + 20% of
the core), with the shell taken as surface area × 1.2 mm (3 perimeters / top-bottom layers): 0.44 g/cm³ for the block (63 g) and
0.41 g/cm³ for the ball (85 g). Same delta method as the torso (the forearm's CAD inertial matches its geometry to 2 mm): each forearm
3.19 → 0.59 kg, COM z −0.098 → −0.058 m. Each arm 5.9 → 3.3 kg.
For a robot about 1.13 m tall without a head this is on the heavy side. For comparison, a 1.3 m commercial humanoid weighs about 35 kg.
Every inertia tensor is positive definite, satisfies the triangle inequality, and has its COM inside the link's hull,
so the values are physically consistent. Some values still need checking. Weigh the real parts before using these
for dynamics or control:

| Link | Mass | Implied density (mass / CAD volume) | Concern |
|---|---|---|---|
| `*_link_004` (forearm) | 0.59 kg (was 3.19) | 1.4 g/cm³ | Ball and block corrected from steel to PLA, 20% infill (85 + 63 g). Weigh one to confirm |
| `*_link_002` | 0.024 kg | 8.7 g/cm³ | Tiny coaxial disc between the two yaw joints. Brass/copper density |
| `*_link_001` | 0.73 kg | 4.9 g/cm³ | Mixed steel parts |
| `torso_link` | 9.0017 kg (was 10.0016) | 3.0 g/cm³ | Includes the two battery packs (170×60×46 mm). The original 10.0016 kg looked like a manual round number; the ASA shoulder-mount change is applied on top of it. Weighing the real torso (without pelvis) would settle it |
| legs, pelvis | 0.57–3.5 kg | 2.1–2.9 g/cm³ | Close to aluminium (2.7). Motors are probably not at their real density, so the leg masses may be low |

With the forearm correction each arm (3.3 kg) is about half of each leg (6.4 kg).
With that mass at the end of each arm, arm motions shift the whole-body COM noticeably, which matters for balance control.

## Joint names, signs and limits

The leg joints use the motor registry / tracking policy names, axis signs and ranges
(`motor_control_hybrid/models/*/chrobot_16kg_candidate.xml`). Every leg axis points the same way as in that MJCF, so the
same joint angle bends both models the same way. The knee flexes negative (−120°..0°).

| Leg joint (was) | Name | Range |
|---|---|---|
| `*_pelvis_hip_connector_joint` | `*_hip1_joint` (hip pitch) | ±90° |
| `*_hip_joint` | `*_hip2_joint` (hip roll) | left −90..25°, right −25..90° |
| `*_thigh_joint` | `*_thigh_joint` (hip yaw) | ±45° |
| `*_shank_joint` | `*_knee_joint` | −120..0° |
| `*_foot_joint` | `*_ankle_joint` (ankle pitch) | ±34° |

The arms are renamed to `*_shoulder_pitch_joint` (was `*_shoulder_abduction`), `*_shoulder_roll_joint` (`*_joint_001_10`),
`*_shoulder_yaw_joint` (`*_joint_002_16`), `*_upper_arm_twist_joint` (`*_joint_003_45`, coaxial with the yaw) and
`*_elbow_joint` (`*_joint_004_52`). Their axis signs are the CAD export's, and only the CAD limits are set.

## MuJoCo simulation (humanoid_sim)

[`mjcf/full_body_sim.xml`](mjcf/full_body_sim.xml) is the robot for `humanoid_sim`:
`ros2 launch humanoid_sim sim.launch.py robot:=full_body` (the default `robot:=legs` keeps the policy's 16 kg leg model).
It is generated from the simplified URDF by [`scripts/make_sim_mjcf.py`](scripts/make_sim_mjcf.py). The script adds a floating
base (`floating_base` + `imu` site at the torso origin), the `top` site for the simulated head, sole boxes and sites
(`*_foot_collision_box`, `*_foot`), the policy MJCF's leg damping and armature, position servos that hold the arms at
ctrl = 0, and the SRDF's disabled collision pairs as exclusions. Rerun the script after any URDF change.

First test with the existing tracking policy (2026-11-21, headless, `auto_start:=true`; runs where startup tripped
`imu_freshness` before release are left out):

| Robot | Walking reference | Holding frame 0 (standing) |
|---|---|---|
| `legs` (policy MJCF, 16 kg) | stayed up for 37 s, 15–17 m | stayed up |
| `full_body` (31.9 kg) | fell sideways 1.6 s after release (2 of 2) | fell 1.6 s after release |
| `full_body` with torso and arms made nearly massless (16.3 kg) | fell 2.2 s after release (2 of 2), 1.2 m | – |

So the policy did not transfer, and **mass was not the only reason**. The full-body CAD legs differed from the policy MJCF.
With all joints at zero, each ankle sat 2–3 cm further out, the feet were 3.5 cm further apart, and the foot extended mostly
*behind* the ankle (sole centre 2 cm back) where the policy's foot extends mostly in front (2.5 cm forward).

**After that test, both feet were turned 180° about the vertical through the ankle** (the CAD foot was mounted the wrong way
round). The ankle axis is negated, so its world direction and sign convention are unchanged. Now the sole centre is 2.0–3.2 cm in
front of the hip, as in the policy MJCF (2.4 cm), and the COM sits at the same fore-aft place over the feet as in the 16 kg model.
Still different from the policy MJCF:
- Each ankle sits 2–3 cm further out.
- The two feet are the same, unmirrored part, so the left sole is 1.1 cm outboard of its ankle and the right one 1.4 cm inboard.
  The policy MJCF has both about 1 cm inboard.

Rerun after the foot fix (same policy and launch; runs where startup tripped `imu_freshness` were retried):

| Robot | Walking reference | Holding frame 0 (standing) |
|---|---|---|
| `legs` | no fall in 38 s, 14.6 m | – |
| `full_body` (31.9 kg) | fell after 2.0 / 2.2 / 2.5 s, 0.35–0.59 m, always rolling to +41° | fell after 1.7 / 1.7 s |
| `full_body` with near-massless torso and arms (16.3 kg) | fell after 3.1 / 3.1 / 4.1 s, 1.5–1.7 m | fell after 2.4 s |

The foot fix helps a little (+0.5–1 s), but the policy still does not carry over, even at 16 kg, and even when holding still.
The ankle sat 2–3 cm further out than in the policy MJCF.

**Leg zero matched to the policy MJCF.** The leg joint zeros were then shifted by at most 2.4° (left hip1 −2.4°, hip2 +1.5°,
knee +2.3°; right hip2 −1.0°, the rest under 0.5°), so that at q = 0 the ankle sits where the policy MJCF puts it, while the
sole stays nearly flat (tilt 1.5° / 0.95°). Without an ankle roll joint, both cannot hold exactly. The ankle is now 8.6–9.4 cm
out from hip1 (policy 7.8 cm), and its fore-aft offset dropped from 1.2 cm to under 1 mm. Retest:

| Robot | Walking reference | Holding frame 0 (standing) |
|---|---|---|
| `full_body` (31.9 kg) | fell after 2.1 / 2.3 / 2.5 s, 0.6–0.7 m; now pitching (−44°), no longer always rolling to one side | fell after 1.8 / 1.9 s |
| `full_body`, 16.3 kg variant | fell after 2.7 / 3.1 / 3.4 s, 1.3–1.7 m | fell after 3.2 s |

Matching the zero removed the one-sided roll but not the falls, so the leg zero was not the cause. What still differs from the
policy MJCF: each leg link's mass and inertia distribution, a 13% smaller sole (box half-sizes 3.5 × 9.6 cm against 4.0 × 10.5 cm),
the joint positions along the chain (for example, hip2 sits 1 cm further out), and, at full weight, the 15.6 kg upper body.
**The existing tracking policy does not run this robot; it needs a policy trained on this model.** Which CAD matches the built
robot still has to be checked on the hardware.
The SRDF collision matrix was regenerated after the fix: 21 adjacent pairs and 170 never colliding (the leg ranges exclude
more poses now).
