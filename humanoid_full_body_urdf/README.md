# Full-body humanoid URDF

The full-body humanoid as two URDFs, a light **simplified** model and the original **detailed** CAD meshes.
Both have the same kinematics and mass properties; only the meshes differ.
The folder is a ROS 2 package (`humanoid_full_body_description`), but the URDFs also work without ROS.

| File | Model |
|---|---|
| [`urdf/full_body.urdf.xacro`](urdf/full_body.urdf.xacro) | Simplified, **editable source**, about 210 lines: one macro call per link and joint |
| [`urdf/full_body.urdf`](urdf/full_body.urdf) | Simplified, expanded from the xacro. After editing the xacro, regenerate it with `xacro urdf/full_body.urdf.xacro -o urdf/full_body.urdf` |
| [`urdf/full_body_detailed.urdf`](urdf/full_body_detailed.urdf) | Detailed: the CAD export with one visual/collision per part, about 14,500 lines |
| `meshes/visual/`, `meshes/collision/` | Simplified meshes, one of each per link |
| `meshes/cad/` | Detailed meshes, 1,086 CAD parts |

The MoveIt 2 / RViz2 / MuJoCo setup, and the script that generates the simplified meshes from the CAD parts, are on the
`sim-experimental-cyclonedds` branch (`humanoid_full_body_moveit_config`, `humanoid_full_body_description/scripts/`).

## Use

```bash
# ROS 2: put this folder in a workspace's src/ and build it
colcon build --packages-select humanoid_full_body_description && source install/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat \
  install/humanoid_full_body_description/share/humanoid_full_body_description/urdf/full_body.urdf)"
```

Without ROS (MuJoCo, PyBullet, Isaac, ...), replace `package://humanoid_full_body_description/` in the mesh paths with
this folder's absolute path.
MuJoCo rejects STL files over 200,000 faces, and every simplified mesh is under that limit.
MuJoCo also names mesh assets by file name, and every mesh file name is unique.

## Simplified vs detailed

### Identical in both

- 22 links and 20 joints with the same joint origins and axes, so forward kinematics agree to 0.01 µm.
- The joint zero: at `q = 0` the robot stands upright, with legs straight, soles flat and arms hanging down.
- Mass, COM and inertia of every link. The total is 31.9 kg (details below).

### Different

| | **simplified** | **detailed** |
|---|---|---|
| Visual | 22 meshes, one per link, 427k triangles. Each CAD part was decimated on its own and stays within 1 mm of the original (2 mm on the torso, 95th percentile). Simple parts such as the battery packs are unchanged | 1,086 meshes, one per CAD part, 3.06M triangles |
| Collision | 22 meshes built from about 2,500 convex pieces, 140k triangles. Screws and nuts are boxes, ordinary parts their convex hull, and concave parts are split with CoACD | The same 1,086 CAD part meshes |
| Colours | One material per link | The original colour of every part |
| Memory with MoveIt + MuJoCo + RViz | **about 1 GB** | **about 5–8 GB**. Two copies at once froze a 15 GB desktop |
| MuJoCo model build | about 0.7 s | about 10 s |
| Size | 27 MB | 150 MB |

The simplified collision model was compared with the detailed meshes in MoveIt over 900 random poses (300 each with
every joint in ±0.8, ±1.6 and ±π rad). It wrongly reported contact in **0.7% / 0% / 0.3%** of the poses and missed **no**
collision. For comparison, one convex hull per link gave 4.0% / 1.7% / 3.0%. CoACD pieces are approximate: up to 2.6 mm of a
decomposed part's surface can lie outside its pieces, so treat contacts closer than about 3 mm as unresolved.
MuJoCo makes each collision STL convex as a whole, so in MuJoCo every link is a single hull.

**Use simplified** for everyday work: planning, simulation, RL or control, Jetson-class machines, and parallel runs.
**Use detailed** to check the CAD (part placement, colours), for interference questions below about 3 mm, and as the
reference when validating the simplified model. Run only one detailed instance per machine.

## Frames and joints

- Fixed base at `torso_link`. Its origin is the centre of the torso's bottom face, with +z up, +x to the robot's left and −y forward.
- `pelvis_link` is fixed to the torso. Its top face sits flush on the torso bottom.
- The legs have hip pitch (`*_pelvis_hip_connector_joint`), hip roll (`*_hip_joint`), hip yaw (`*_thigh_joint`), knee
  (`*_shank_joint`) and ankle pitch (`*_foot_joint`).
- The arms have shoulder pitch (`*_shoulder_abduction`), roll (`*_joint_001_10`), two coaxial yaw joints
  (`*_joint_002_16`, `*_joint_003_45`) and elbow (`*_joint_004_52`).
- At zero pose the robot is 1.05 m tall from sole to torso top (no head) and 0.68 m wide across the arms.

The joint zero was re-defined from the CAD pose, in which the legs were posed rather than straight. The offsets moved
into the leg joint origins, in degrees, are:

| Old CAD pose expressed in the new coordinates | hip pitch / roll / yaw / knee / ankle |
|---|---|
| left leg | −19.7 / 0.0 / 32.5 / 29.5 / 5.7 |
| right leg | −1.9 / 7.8 / 0.9 / −29.2 / 2.5 |

## Mass properties

Total **31.9 kg**: torso 9.0 kg (with the two battery packs), pelvis 3.5 kg, each arm 3.3 kg, each leg 6.4 kg.
These values come from the CAD export, with two material corrections, and nothing has been weighed:

- **Shoulder mounts** (`base_link_333` / `314`, the three-lobed brackets on the torso sides) changed from 6061 to printed ASA,
  which removes 1.0 kg for the pair. The torso mass, COM and inertia were recomputed from the parts' geometry
  (10.0 → 9.0 kg, COM z 0.192 → 0.181 m).
- **Forearm ball and block** (`link_004_004`, `link_004_003`) changed from steel to PLA at 20% infill (about 0.41–0.44 g/cm³
  effective with a 1.2 mm solid shell). Each forearm went from 3.19 to 0.59 kg.

Every inertia tensor is positive definite and satisfies the triangle inequality.

## Not done yet

- **Joint polarity** has not been checked against the motors. The axis directions come from the CAD, and left/right are
  mirrored for some joints but not others.
- **Joint limits**: 16 of the 20 joints are `continuous`. Only the shoulder roll (−15°..105°) and one upper-arm yaw (±180°)
  have CAD limits, and effort/velocity are 0.
- **Encoder zero** and the model zero have not been reconciled.
- The **masses** have not been weighed, and the motor masses are whatever the CAD assigned.
