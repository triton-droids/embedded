# WBC simulation prototype

`kinematics.py` re-exports the controller from `wbc_control_node.controller` for the saved
`chrobot_16kg_candidate.xml`, without the optional simulated head or target.
The ROS package provides an optional policy → WBC → C++ → MuJoCo launch path;
see `humanoid_control/wbc_control_node/README.md`. These isolated tests do not use ROS.
The candidate XML is the simulation reference, not a validated hardware model.

The URDF retains its original joint transforms, geometry, and fixed world offset.
Its link masses, local centers of mass, and inertia tensors are synchronized from
the compiled MuJoCo robot. Run from the repository root to regenerate:

```bash
rosenv/bin/python experimental/sync_wbc_model.py
```

The adjacent `human_offset_corrected.dynamics.json` records the source SHA256,
joint armatures/damping, the transform between MuJoCo's floating base and
Pinocchio's root, and sole corner coordinates in each URDF foot frame. URDF alone
cannot encode the MuJoCo joint armature; TSID's rotor inertia and gear ratio
settings supply it. Returned motor torques compensate MuJoCo's passive joint
damping. Do not apply that compensation twice in an external controller.

Use `state_from_sim(...)` with MuJoCo floating-base position, quaternion in **xyzw**
order, **world-frame** linear/angular velocity, and joint arrays ordered according
to `controller.joint_names`. These match the existing `/sim/ground_truth/odom`
pose/twist conventions after quaternion field ordering. Direct MuJoCo free-joint
angular velocity is body-frame and must first be rotated into the world frame.
The method also accounts for the different base origins and their velocity shift.

After initializing the feet on the ground, use:

```python
controller.add_foot_contact("left_foot", q)
controller.add_foot_contact("right_foot", q)
tau = controller.compute(q, dq, q_ref, t)
```

Contact motion uses priority 1 with weight 1000 by default. Double-support
kinematic equalities can be rank-deficient for this five-joint-per-leg geometry;
the fast solver rejects redundant hard equalities. Soft motion permits small
tracking errors while floating-base dynamics, friction inequalities, and normal
force bounds remain hard. Passing `motion_priority=0` requests hard motion
constraints and can reproduce the redundant-equality failure. Do not interpret
adding a contact as measuring one.

The current locally built TSID 1.7.1 required the accompanying
`tsid-1.7.1-rotor-setters.patch`: its original non-const Eigen reference arguments
prevented NumPy arrays from reaching the rotor/gear setters. The installed rosenv
binding has been rebuilt with that patch; recreating the environment requires
applying it before building TSID.

Run the model-equivalence and isolated stance tests with:

```bash
PYTHONDONTWRITEBYTECODE=1 rosenv/bin/python -m pytest experimental/test_wbc_model.py -q -s -p no:cacheprovider
```

Tests compare FK, foot geometry, CoM, gravity, mass matrices including armature,
and moving inverse dynamics including coordinate acceleration and damping.
The stance test disables the gantry, drives MuJoCo actuators directly with TSID
torques, and does not use ROS, the original motor PD, the optional head, or RL.
It therefore does not validate the existing ROS control/safety path or walking.
Torque magnitudes are checked against the motor registry in this isolated test.
The ROS bridge additionally configures hard actuator torque bounds. Its default
policy-impedance mode retains fast motor PD and adds a small WBC torque
correction through the C++ scheduler and safety path.

### TSID 1.7.1 soft-contact removal fix

Also apply `tsid-1.7.1-soft-contact-removal.patch` to the TSID 1.7.1 source
before building. `addRigidContact` counts motion equalities only at priority 0,
but the original `removeRigidContact` decrements that count at every priority.
Removing soft contacts can therefore underflow the equality count and cause
native solver allocation failure.
The patch checks the motion task's actual level before removing it, and only
decrements the hard equality count for a level-0 task. It does not alter contact
forces, weights, friction constraints or the floating-base dynamics.

For the existing local build (which already includes the rotor-setter patch):

```bash
patch -d /path/to/tsid-1.7.1 -p1 < experimental/tsid-1.7.1-soft-contact-removal.patch
cmake --build /path/to/tsid-build -j1
cmake --install /path/to/tsid-build
source scripts/activate_rosenv.sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.:simulation/humanoid_sim:humanoid_control/wbc_control_node:$PYTHONPATH rosenv/bin/python -m pytest experimental/test_wbc_model.py -k contact_removal -q -p no:cacheprovider
```

The regression checks the real native formulation counts before calling solver
resize, for hard and soft contacts across repeated add/remove cycles. This keeps
an unpatched TSID installation from crashing the test process.

### Nominal policy torque task Python binding

`tsid-1.7.1-actuation-equality-binding.patch` exposes TSID's existing C++
`TaskActuationEquality` and the matching formulation `addActuationTask` overload
to Python. Apply it with `patch -p1` in the same TSID source tree, then rebuild
and install as above. The installed rosenv includes all three patches.
The new default WBC policy-impedance mode requires this binding; a missing
binding raises an explicit startup error rather than falling back to PD.
