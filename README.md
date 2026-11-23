# Full-body humanoid URDF

The full-body humanoid as two URDFs, a light **simplified** model and the original **detailed** CAD meshes.
Both have the same kinematics and mass properties; only the meshes differ. Each folder is a self-contained ROS 2 package,
and the two have different package names, so they can be built side by side. The URDFs also work without ROS.

| Folder | Package | Contents |
|---|---|---|
| [`simplified/`](simplified) | `humanoid_full_body_simplified` | [`urdf/full_body.urdf.xacro`](simplified/urdf/full_body.urdf.xacro), the **editable source** (about 210 lines: one macro call per link and joint); [`urdf/full_body.urdf`](simplified/urdf/full_body.urdf), expanded from the xacro; `meshes/visual/` and `meshes/collision/`, one mesh of each per link |
| [`detailed/`](detailed) | `humanoid_full_body_detailed` | [`urdf/full_body.urdf`](detailed/urdf/full_body.urdf), the CAD export with one visual/collision per part (about 14,500 lines); `meshes/cad/`, 1,086 CAD part meshes |
| [`config/`](config) | – | [`motors_full_body.yaml`](config/motors_full_body.yaml), the motor registry for the full body (simulation): 10 leg motors and 8 arm motors, see "Motors" below |

After editing the xacro, regenerate the expanded file with `cd simplified && xacro urdf/full_body.urdf.xacro -o urdf/full_body.urdf`.
Changes to joints or masses must be made in **both** folders.

The MoveIt 2 / RViz2 / MuJoCo setup, and the script that generates the simplified meshes from the CAD parts, are on the
`sim-experimental-cyclonedds` branch (`humanoid_full_body_moveit_config`, `humanoid_full_body_description/scripts/`).

## Use

```bash
# ROS 2: put one or both folders in a workspace's src/ and build
colcon build --packages-select humanoid_full_body_simplified humanoid_full_body_detailed && source install/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat \
  install/humanoid_full_body_simplified/share/humanoid_full_body_simplified/urdf/full_body.urdf)"
# detailed: install/humanoid_full_body_detailed/share/humanoid_full_body_detailed/urdf/full_body.urdf
```

Without ROS (MuJoCo, PyBullet, Isaac, ...), replace `package://humanoid_full_body_simplified/` (or `..._detailed/`) in the
mesh paths with the folder's absolute path.
MuJoCo rejects STL files over 200,000 faces, and every simplified mesh is under that limit.
MuJoCo also names mesh assets by file name, and every mesh file name is unique.

## MuJoCo (simplified, with motors)

```bash
cd simplified
python3 scripts/make_mjcf.py           # urdf/full_body.urdf -> mjcf/full_body.xml (rerun after URDF changes)
python3 scripts/stand.py --headless    # 10 s standing test; without --headless opens the viewer (macOS: mjpython)
```

- [`mjcf/full_body.xml`](simplified/mjcf/full_body.xml) (generated) is the robot on a free joint with one `<motor>` per motor,
  `gear="1"` (ctrl = torque in N·m) and `ctrlrange` = the motor's **peak** torque (not continuous):

  | Actuator | Joint | Motor | ctrlrange (N·m) |
  |---|---|---|---|
  | `*_hip1_motor`, `*_knee_motor` | hip pitch, knee | RS-04 | ±120 |
  | `*_hip2_motor`, `*_thigh_motor` | hip roll, hip yaw | RS-03 | ±60 |
  | `*_ankle_motor` | **`*_ankle_crank_joint`** (motor axis) | RS-02 | ±17 |
  | `*_shoulder_pitch_motor`, `*_shoulder_roll_motor` | arm | RS-03 | ±60 |
  | `*_shoulder_yaw_motor`, `*_elbow_motor` | arm | RS-02 | ±17 |

- **Ankle.** The RS-02 turns the four-bar crank, not the foot. Crank and rod are passive hinges, and an equality `connect`
  pins the rod's lower eye to the heel lever. MuJoCo therefore passes the 17 N·m through the real linkage geometry, so the foot
  torque varies with the ankle angle. Its PD acts on the crank angle (crank ≈ 0.98 × ankle near zero). The loop uses a
  near-hard `solimp` (0.99, 0.999); with the default it stretched about 6 mm under body weight and the robot fell.
- `*_upper_arm_twist_joint` is welded. Each sole is a 15 mm box contact. The robot collides only with the floor, with no
  self-collision.
- [`mjcf/scene.xml`](simplified/mjcf/scene.xml) adds the floor and lights.
- [`scripts/stand.py`](simplified/scripts/stand.py) holds the zero pose with joint PD, using the registry gains for every joint except the ankle.
  Standing still with no balance feedback is an inverted pendulum on the ankles (m·g·h ≈ 200 N·m/rad). The registry's
  ankle kp 20 tips the robot over within 2 s, so `stand.py` uses `--ankle-kp 150 --ankle-kd 3`. It then stands with about
  2° lean, using under 35% of peak torque.

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

- Fixed base at `torso_link`. Its origin is the centre of the torso's bottom face, with **+z up and +y forward** (the toes
  point +y). The `left_*` joints are on the **−x side, the robot's left**.
- The CAD export faces −y. `torso_link` is the CAD torso frame turned 180° about z: the torso mesh, its inertial and its
  three child joints (pelvis, both shoulders) carry the turn. All other links keep their CAD frames.
- `pelvis_link` is fixed to the torso. Its top face sits flush on the torso bottom.
- At zero pose the robot stands upright, with legs straight, soles flat and arms hanging down. It is 1.05 m tall from sole to
  torso top (no head) and 0.68 m wide across the arms.

Every joint sign was checked by forward kinematics (`humanoid_full_body_description/scripts/reorient.py` on
`sim-experimental-cyclonedds`). The meanings are those of the motor registry (`motor_control_hybrid/config/motors.yaml`),
so its direction flags still apply. **Fusion shows the opposite sign for hip pitch and ankle.**

| Joint | Positive direction | Range | Earlier CAD name |
|---|---|---|---|
| `*_hip1_joint` | hip pitch, **flexion** (foot forward) | left −17.5..+120°, right −15..+120° (see below) | `*_pelvis_hip_connector_joint` |
| `*_hip2_joint` | hip roll; left − / right + abducts | left −90..25°, right −25..90° | `*_hip_joint` |
| `*_thigh_joint` | hip yaw, toe outward | ±45° | `*_thigh_joint` |
| `*_knee_joint` | knee; **flexion is negative** (foot back) | −120..0° (mechanically free both ways) | `*_shank_joint` |
| `*_ankle_joint` | ankle pitch, **toe up** | −120 (toe down) .. +96.6° (toe up), measured | `*_foot_joint` |
| `*_shoulder_pitch_joint` | arm forward | none (continuous) | `*_shoulder_abduction` |
| `*_shoulder_roll_joint` | abduction | −15..105° (CAD) | `*_joint_001_10` |
| `*_shoulder_yaw_joint` | – | ±180° (CAD) | `*_joint_002_16` |
| `*_upper_arm_twist_joint` | twist, coaxial with the yaw; **no motor** (see "Motors") | none (continuous) | `*_joint_003_45` |
| `*_elbow_joint` | flexion (assumed forward, unverified) | 0..130° (Fusion) | `*_joint_004_52` |

- **Hip pitch.** Extending the thigh backward, the hip connector bracket grazes a pelvis bracket at 19.5° (left) and
  17.0° (right). The overlap is under 1 mm and clears again from 25°. An exact mesh sweep of the CAD parts found it; the
  design is being changed. A second contact (about 3 × 3 × 5 mm) follows at 61–63.5° of extension. The limits keep 2° clear of
  the first contact. Flexion is free to beyond 120° (Fusion limit 120°).
- **Ankle.** The foot is driven through a heel four-bar (`ankle_mapping` in the motor registry). The motor drives the foot
  with at least 10 Nm between about −65° and +75°. The linkage has dead points near −82° and +87°. Beyond them the foot folds
  passively to the measured −120° / +96.6°. `config/motors_full_body.yaml` limits the ankle to the driven range.
  In the URDF the four-bar has its own links: `*_ankle_crank_link` turns about the motor axis and `*_ankle_rod_link`
  hangs on the crank pin (a `*_ankle_rod_end` frame marks the lower eye). Both joints are `<mimic>` joints of the ankle, a
  linear fit within 0.9° (crank) / 1.6° (rod) over −65..+75°. A URDF tree cannot close the loop; a simulator can, by
  pinning `*_ankle_rod_end` to the heel lever (MuJoCo: equality `connect`). CAD pins: ground 166.24 mm, crank 45.00, rod
  166.29, heel lever 44.40 (the `ankle_mapping` lengths differ, see `humanoid_full_body_description/README.md` on
  sim-experimental-cyclonedds).
- **Earlier versions of this branch were wrong** (until eb00df9c). They kept the CAD frame, matched the legs to the
  tracking policy's +y-forward leg model, and turned both feet 180°. That reversed the robot front to back: the feet, the
  knee direction and the hip range were all mirrored.

## Motors

[`config/motors_full_body.yaml`](config/motors_full_body.yaml) has the same format as `motor_control_hybrid/config/motors.yaml`.
The 10 leg entries are copied from that file. Peak torques are RS-04 120 Nm, RS-03 60 Nm and RS-02 17 Nm.

| Joint | Motor |
|---|---|
| `*_hip1_joint` (hip pitch), `*_knee_joint` | RS-04 |
| `*_hip2_joint` (hip roll), `*_thigh_joint` (hip yaw) | RS-03 |
| `*_ankle_joint` | RS-02 |
| `*_shoulder_pitch_joint` (on the torso), `*_shoulder_roll_joint` | RS-03 |
| `*_shoulder_yaw_joint`, `*_elbow_joint` | RS-02 |

Each arm has 4 motors: shoulder pitch, then shoulder roll, shoulder yaw and elbow towards the wrist.
`*_upper_arm_twist_joint` is a CAD artefact: it is coaxial with the shoulder yaw, the link between them weighs 24 g, and it has no
motor. The URDFs keep it, but the simulation and controllers treat it as fixed at 0. The arm entries' CAN ids, directions,
zero offsets and gains are **placeholders**. Their position ranges are the URDF's, ±180° where the joint is continuous. Only the
motor models are from the hardware.

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

- **Arm motors**: the arm motor entries in `config/` need their CAN ids, directions and zero offsets; the elbow flexion direction is assumed.
- **Effort/velocity limits** are 0 in the URDF.
- **Encoder zero** and the model zero have not been reconciled.
- The **masses** have not been weighed, and the motor masses are whatever the CAD assigned.
