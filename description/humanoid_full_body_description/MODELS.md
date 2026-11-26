# Simplified vs detailed model

`humanoid_full_body_description` ships the same robot twice. The two URDFs share kinematics and mass properties and
differ only in their meshes.

```bash
ros2 launch humanoid_full_body_moveit_config inspect.launch.py model:=simplified   # default
ros2 launch humanoid_full_body_moveit_config inspect.launch.py model:=detailed
```

## What is identical

- 27 links and 22 movable joints (with the pan-tilt head), with the same joint origins and axes, so forward kinematics agree to 0.01 µm.
- The joint zero: at `q = 0` the robot stands upright, with legs straight, soles flat and arms hanging down.
- Mass, COM and inertia of every link (total 32.5 kg with the 0.6 kg head). Both files carry the same corrections: ASA shoulder mounts and
  PLA forearm parts at 20% infill.
- The MoveIt configuration: planning groups, group states and the SRDF self-collision matrix.

## What differs

| | **simplified** (default) | **detailed** |
|---|---|---|
| Installed file | `urdf/full_body.urdf` | `urdf/full_body_detailed.urdf` |
| Edited in | [`urdf/full_body.urdf.xacro`](urdf/full_body.urdf.xacro) (about 210 lines; CMake expands it) | [`cad_source/full_body_parts.urdf`](cad_source/full_body_parts.urdf) (about 14,500 lines, the CAD export) |
| Visual geometry | 22 meshes, one per link, 427k triangles. Each CAD part was decimated separately and stays within 1 mm of the original (2 mm on the torso, 95th percentile). Simple parts such as the battery packs are unchanged | 1,086 meshes, one per CAD part, 3.06M triangles |
| Collision geometry | 22 meshes built from about 2,500 convex pieces (140k triangles). Screws and nuts are boxes, ordinary parts their convex hull, and concave parts are split with CoACD | The same 1,086 CAD part meshes |
| Colours | One material per link | The original colour of every part |
| Memory: move_group | about 0.1–0.2 GB | about 2 GB |
| Memory: MuJoCo (`backend:=mujoco`) | about 0.5 GB | about 2.3–3 GB |
| Memory: whole stack with RViz | **about 1 GB** | **about 5–8 GB**. Two copies at once froze a 15 GB desktop |
| MuJoCo model build | about 0.7 s | about 10 s |
| Package size installed / zipped | 14 MB / 11 MB | +150 MB / 58 MB |

### Collision accuracy of the simplified model

Measured in MoveIt against the detailed meshes, over the same 900 random poses (300 each with every joint in
±0.8, ±1.6 and ±π rad):

| Simplified collision model | False positives (only the simplified model reports contact) | Missed collisions |
|---|---|---|
| One convex hull per link (earlier version) | 4.0% / 1.7% / 3.0% | 0 |
| **Per-part pieces with CoACD (current)** | **0.7% / 0% / 0.3%** | **0** |

Every convex hull contains its part. CoACD pieces are approximate: with each piece capped at 64 vertices, up to
2.6 mm of a decomposed part's surface can lie outside its pieces. No missed collision showed up in the test, but treat
contacts closer than about 3 mm as unresolved. MuJoCo makes each collision STL convex as a whole, so inside MuJoCo every
link is a single hull in both models.

## Which to use

- **simplified**: everyday work, including planning, MuJoCo execution, RL or controller work, Jetson-class machines,
  and anything that runs for a long time or more than once in parallel.
- **detailed**: checking the CAD (part placement, colours, which part is which), close-clearance or interference
  questions below about 3 mm, presentation screenshots, and serving as the reference when validating the simplified
  model.

Use only one model on a machine at a time. On a 15–16 GB desktop, cap memory for the detailed model:

```bash
systemd-run --user --scope -p MemoryMax=9G -p MemorySwapMax=0 \
  ros2 launch humanoid_full_body_moveit_config inspect.launch.py backend:=mujoco model:=detailed
```

To skip installing the detailed model on small targets, build with `--cmake-args -DINSTALL_DETAILED_MODEL=OFF`.

## How the simplified model is produced

The detailed URDF is also the input for the simplified meshes. After a CAD change, update
`cad_source/` and regenerate:

```bash
python3 -m venv /tmp/meshenv
/tmp/meshenv/bin/pip install trimesh==4.4.9 fast-simplification==0.1.9 rtree==1.3.0 coacd==1.0.14 scipy
cd humanoid_full_body_description
OMP_NUM_THREADS=1 /tmp/meshenv/bin/python scripts/simplify_meshes.py                # visual + collision
OMP_NUM_THREADS=1 /tmp/meshenv/bin/python scripts/simplify_meshes.py --only visual  # about 1.5 min
```

Generating the collision meshes takes about 30 minutes, because CoACD is slow on the torso extrusions. Use
`--links <name> ...` to redo only some links. If the collision shapes grow, regenerate the SRDF collision matrix as
described in the MoveIt config README. Changes to masses or joints go into **both** URDFs, which must stay identical
in everything except meshes.
