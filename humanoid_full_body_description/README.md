# humanoid_full_body_description

Full-body humanoid URDF: the upper body plus the articulated lower body at the nominal five-hole pelvis mount.
22 links, 20 actuated joints. Fixed base at `torso_link`.

## Layout

| Path | What |
|---|---|
| [`urdf/full_body.urdf.xacro`](urdf/full_body.urdf.xacro) | **Source of truth**, about 200 lines: one `cad_link` and one joint macro call per link/joint, grouped by torso / pelvis / arms / legs |
| `urdf/full_body.urdf` (installed only) | Generated at build time by CMake (`xacro`), so consumers keep loading `share/humanoid_full_body_description/urdf/full_body.urdf` |
| `meshes/visual/<link>.stl` | The link's CAD parts, each decimated separately and then concatenated (427k triangles in total). Each part stays within 1 mm of the original (95th percentile); the torso is held to 2 mm so it fits under MuJoCo's 200k-face STL limit. Simple parts such as the two torso battery packs are kept unchanged |
| `meshes/collision/<link>.stl` | Convex hull of the link's CAD parts (17k triangles in total) |
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
python3 -m venv /tmp/meshenv && /tmp/meshenv/bin/pip install trimesh==4.4.9 fast-simplification==0.1.9 scipy
cd humanoid_full_body_description && /tmp/meshenv/bin/python scripts/simplify_meshes.py
```

Then regenerate the MoveIt self-collision matrix (see the MoveIt config README), because the collision shapes changed.

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
toes forward (−y in `torso_link`; +z up, +x to the robot's left), and arms hanging straight down.
In the source CAD, the legs were posed rather than straight. The right knee was bent about 29°, the left leg
was yawed about 33° and pitched about 20°, and the right hip was rolled about 8°. These offsets were moved into
the `rpy` of the 10 leg joint origins, so that the old pose equals the new joint values below:

| Joint | Old pose in new coordinates (deg) |
|---|---|
| left_pelvis_hip_connector / hip / thigh / shank / foot | −19.7 / 0.0 / 32.5 / 29.5 / 5.7 |
| right_pelvis_hip_connector / hip / thigh / shank / foot | −1.9 / 7.8 / 0.9 / −29.2 / 2.5 |

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
