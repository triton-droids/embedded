"""The full-body humanoid (simplified/mjcf/full_body.xml) as an mjlab EntityCfg.

What comes from the XML unchanged:
- meshes (visual and collision), masses and inertias (31.93 kg),
- joint ranges, damping and armature (the RS-02 rotor armature sits on the ankle crank),
- the ankle four-bar: crank and rod are passive hinges, closed by an equality connect
  per side (`*_ankle_linkage`, near-hard solimp),
- collision bits: robot geoms contype=2 / conaffinity=1, so the robot touches a
  terrain with the default contype=conaffinity=1 but never itself; the sole boxes
  carry the foot contacts.

What this file replaces: the 18 <motor>s of the XML. They are deleted in `get_spec`
and mjlab's IdealPdActuatorCfg adds one <motor> per joint again, computing
tau = clamp(kp * (q_ref - q) + kd * (dq_ref - dq), -peak, peak) like
simplified/scripts/stand.py. Without the deletion every joint would have two motors.

Joints: 22 hinges + the free joint. 18 are motorised; the 4 others are passive,
driven through the four-bar: `*_ankle_joint` (the foot) and `*_ankle_rod_joint`.
The motorless upper-arm twist is already welded in the XML.

Run the checkpoint: mjlab/.venv/bin/python -m humanoid_mjlab.robot
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
from mjlab.actuator import IdealPdActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg

EMBEDDED = Path(__file__).resolve().parents[3]  # .../embedded
ROBOT_XML = EMBEDDED / "simplified" / "mjcf" / "full_body.xml"

# Base (free joint at the torso_link origin) height: soles 1 mm above z = 0 at the
# zero pose, as in the XML.
BASE_HEIGHT = 0.666
TOTAL_MASS = 31.933  # kg, README "Mass properties"

PEAK_TORQUE = {"rs-02": 17.0, "rs-03": 60.0, "rs-04": 120.0}  # N*m, peak

# Joint groups. Names are matched with re.fullmatch.
HIP_PITCH = (".*_hip1_joint",)
HIP_ROLL_YAW = (".*_hip2_joint", ".*_thigh_joint")
KNEE = (".*_knee_joint",)
# The RS-02 ankle motor turns the four-bar crank, not the foot.
ANKLE_MOTOR = (".*_ankle_crank_joint",)
SHOULDER_PITCH_ROLL = (".*_shoulder_pitch_joint", ".*_shoulder_roll_joint")
SHOULDER_YAW_ELBOW = (".*_shoulder_yaw_joint", ".*_elbow_joint")
# Driven through the four-bar, no motor of their own.
PASSIVE_JOINTS = (
    "left_ankle_joint",
    "left_ankle_rod_joint",
    "right_ankle_joint",
    "right_ankle_rod_joint",
)

# Gains: config/motors_full_body.yaml (kp / kd), except the ankle. Standing with no
# balance feedback needs about m*g*h = 200 N*m/rad at the ankles; the registry's
# kp 20 per ankle tips the robot over, so the standing setup uses 150 / 3
# (simplified/scripts/stand.py). Crank-to-foot ratio is about 0.98 near zero.
HIP_PITCH_ACTUATOR = IdealPdActuatorCfg(
    target_names_expr=HIP_PITCH,
    stiffness=100.0,
    damping=1.0,
    effort_limit=PEAK_TORQUE["rs-04"],
)
HIP_ROLL_YAW_ACTUATOR = IdealPdActuatorCfg(
    target_names_expr=HIP_ROLL_YAW,
    stiffness=100.0,
    damping=1.0,
    effort_limit=PEAK_TORQUE["rs-03"],
)
KNEE_ACTUATOR = IdealPdActuatorCfg(
    target_names_expr=KNEE,
    stiffness=80.0,
    damping=1.0,
    effort_limit=PEAK_TORQUE["rs-04"],
)
ANKLE_ACTUATOR = IdealPdActuatorCfg(
    target_names_expr=ANKLE_MOTOR,
    stiffness=150.0,
    damping=3.0,
    effort_limit=PEAK_TORQUE["rs-02"],
)
SHOULDER_PITCH_ROLL_ACTUATOR = IdealPdActuatorCfg(
    target_names_expr=SHOULDER_PITCH_ROLL,
    stiffness=40.0,
    damping=2.0,
    effort_limit=PEAK_TORQUE["rs-03"],
)
SHOULDER_YAW_ELBOW_ACTUATOR = IdealPdActuatorCfg(
    target_names_expr=SHOULDER_YAW_ELBOW,
    stiffness=40.0,
    damping=2.0,
    effort_limit=PEAK_TORQUE["rs-02"],
)
# armature / frictionloss / viscous_damping are left None: the XML values stay.

ACTUATORS = (
    HIP_PITCH_ACTUATOR,
    HIP_ROLL_YAW_ACTUATOR,
    KNEE_ACTUATOR,
    ANKLE_ACTUATOR,
    SHOULDER_PITCH_ROLL_ACTUATOR,
    SHOULDER_YAW_ELBOW_ACTUATOR,
)

# Zero pose: standing, legs straight, soles flat, arms down. Every joint is 0,
# including the passive crank and rod: the four-bar closes exactly there. Any other
# ankle angle needs consistent crank/rod angles (the equality pulls them together).
ZERO_POSE = EntityCfg.InitialStateCfg(
    pos=(0.0, 0.0, BASE_HEIGHT),
    joint_pos={".*": 0.0},
    joint_vel={".*": 0.0},
)


def get_spec() -> mujoco.MjSpec:
    """The robot XML without its <motor>s (mjlab adds them from ACTUATORS)."""
    spec = mujoco.MjSpec.from_file(str(ROBOT_XML))
    for actuator in list(spec.actuators):
        spec.delete(actuator)
    return spec


# No CollisionCfg: it would rewrite contype/conaffinity of the matched geoms and
# zero all others, and the XML's collision geoms (from the URDF) are unnamed. The
# XML's own collision setup is kept as described above.
ROBOT_CFG = EntityCfg(
    spec_fn=get_spec,
    init_state=ZERO_POSE,
    articulation=EntityArticulationInfoCfg(actuators=ACTUATORS),
)


def get_robot_cfg() -> EntityCfg:
    return ROBOT_CFG


def checkpoint() -> None:
    """Compile the entity and check mass, actuators, joints and the ankle loop."""
    entity = ROBOT_CFG.build()
    model = entity.spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("init_state").id)
    mujoco.mj_forward(model, data)

    def check(ok: bool, text: str) -> None:
        print(f"  [{'ok' if ok else 'FAIL'}] {text}")
        if not ok:
            raise SystemExit(1)

    print(f"Entity from {ROBOT_XML}")

    mass = float(model.body_mass.sum())
    check(abs(mass - TOTAL_MASS) < 0.01, f"mass {mass:.3f} kg (expected {TOTAL_MASS})")
    check(not entity.is_fixed_base, "floating base (free joint)")

    # Actuators: exactly one motor per motorised joint, none on passive joints.
    hinge_names = [
        model.joint(j).name
        for j in range(model.njnt)
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE
    ]
    targets = [model.joint(model.actuator_trnid[a, 0]).name for a in range(model.nu)]
    check(model.nu == 18, f"{model.nu} actuators (expected 18)")
    check(len(set(targets)) == len(targets), "no joint has two motors")
    passive = sorted(set(hinge_names) - set(targets))
    check(passive == sorted(PASSIVE_JOINTS), f"passive joints: {', '.join(passive)}")
    check(
        all(
            model.actuator_biastype[a] == mujoco.mjtBias.mjBIAS_NONE
            for a in range(model.nu)
        )
        and np.all(model.actuator_gear[:, 0] == 1.0),
        "all actuators are torque motors with gear 1",
    )
    print("  actuated joints (forcerange N*m, kp, kd):")
    for act in entity.actuators:
        cfg = act.cfg
        for name in act.target_names:
            a = targets.index(name)
            lo, hi = model.actuator_forcerange[a]
            ok = model.actuator_forcelimited[a] and hi == cfg.effort_limit and lo == -hi
            print(
                f"    {name:28s} ±{hi:5.0f}  kp {cfg.stiffness:5.0f}  kd {cfg.damping:3.0f}"
                + ("" if ok else "  <- FAIL")
            )
            if not ok:
                raise SystemExit(1)
    ankle_targets = [t for t in targets if "ankle" in t]
    check(
        all(t.endswith("_ankle_crank_joint") for t in ankle_targets)
        and len(ankle_targets) == 2,
        "ankle motors drive the cranks; the foot joints are passive",
    )

    # Ankle four-bar loop.
    eq_names = [model.equality(e).name for e in range(model.neq)]
    check(
        sorted(eq_names) == ["left_ankle_linkage", "right_ankle_linkage"]
        and all(model.eq_type[e] == mujoco.mjtEq.mjEQ_CONNECT for e in range(model.neq))
        and all(model.eq_active0),
        f"ankle loops present and active: {', '.join(eq_names)}",
    )
    check(
        bool(np.all(model.eq_solimp[:, 0] >= 0.99)),
        f"ankle loops near-hard (solimp {model.eq_solimp[0, :3].round(3).tolist()})",
    )
    eq_rows = data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
    gap = float(np.abs(data.efc_pos[eq_rows]).max()) if eq_rows.any() else 0.0
    check(gap < 1e-6, f"loops closed at the zero pose (gap {gap * 1000:.2g} mm)")

    # Joint limits of the motorised leg joints.
    for name in ("left_hip1_joint", "left_knee_joint", "left_ankle_joint"):
        lo, hi = np.degrees(model.joint(name).range)
        check(
            model.joint(name).limited[0] == 1,
            f"{name} limited to {lo:.1f}..{hi:.1f} deg",
        )

    # Collisions: no self-contact, sole boxes reach the floor bit.
    robot_geoms = [g for g in range(model.ngeom) if model.geom_contype[g]]
    check(
        all(
            model.geom_contype[g] == 2 and model.geom_conaffinity[g] == 1
            for g in robot_geoms
        ),
        f"{len(robot_geoms)} collision geoms, contype 2 / conaffinity 1 (no self-contact)",
    )
    soles = [model.geom(f"{s}_foot_collision_box").id for s in ("left", "right")]
    sole_bottom = [
        data.geom_xpos[g][2] - model.geom_size[g][2] for g in soles
    ]  # boxes are level at the zero pose
    check(
        all(0.0 < z < 0.003 for z in sole_bottom),
        f"soles {', '.join(f'{z * 1000:.1f}' for z in sole_bottom)} mm above z = 0 at init",
    )
    check(data.ncon == 0, "no contacts at the zero pose")
    crank = model.joint("left_ankle_crank_joint").armature[0]
    check(crank == 0.01, f"XML armature kept (ankle crank {crank} kg*m^2)")
    print("Checkpoint passed.")


if __name__ == "__main__":
    checkpoint()
