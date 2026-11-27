"""Validate walking reset and target clipping with native MuJoCo.

Run: mjlab/.venv/bin/python -m humanoid_mjlab.check_crouch
Uses the task physics settings and the same PD law; no GPU or Warp compilation.
"""

import re

import mujoco
import numpy as np
from mjlab.scene import Scene

from humanoid_mjlab.crouch import JOINT_TARGET_MARGIN
from humanoid_mjlab.env_cfg import make_env_cfg


def main() -> None:
    cfg = make_env_cfg()
    cfg.scene.num_envs = 1
    scene = Scene(cfg.scene, device="cpu")
    m = scene.compile()
    cfg.sim.mujoco.apply(m)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("init_state").id)
    ref = d.qpos.copy()
    gains = {
        n: (a.cfg.stiffness, a.cfg.damping, a.cfg.effort_limit)
        for a in scene["robot"].actuators
        for n in a.target_names
    }
    mujoco.mj_forward(m, d)
    initial_gap = np.max(
        abs(d.efc_pos[d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY])
    )
    assert initial_gap < 1e-6
    for side in ("left", "right"):
        g = m.geom("robot/" + side + "_foot_collision_box").id
        rot = d.geom_xmat[g].reshape(3, 3)
        assert np.linalg.norm(rot[:2, 2]) < 1e-5
        assert abs(d.geom_xpos[g, 2] - m.geom_size[g, 2] - 0.001) < 1e-6
    for j in range(m.njnt):
        if m.jnt_limited[j]:
            lo, hi = m.jnt_range[j]
            assert (
                lo + JOINT_TARGET_MARGIN
                <= ref[m.jnt_qposadr[j]]
                <= hi - JOINT_TARGET_MARGIN
            )
    # Exercise the actual action processing method, without allocating a Warp scene.
    from types import SimpleNamespace

    import torch
    from mjlab.envs.mdp.actions.actions import JointPositionAction

    bounds = cfg.actions["joint_pos"].clip
    names = list(bounds)
    action = object.__new__(JointPositionAction)
    action.cfg = SimpleNamespace(clip=bounds)
    action._raw_actions = torch.zeros((3, len(names)))
    action._offset = torch.tensor(
        [[cfg.scene.entities["robot"].init_state.joint_pos[n] for n in names]]
    )
    action._scale = torch.tensor(
        [
            [
                next(
                    v
                    for p, v in cfg.actions["joint_pos"].scale.items()
                    if re.fullmatch(p, n)
                )
                for n in names
            ]
        ]
    )
    action._clip = torch.tensor([[bounds[n] for n in names]])
    action.process_actions(
        torch.tensor([[-1e6] * len(names), [0.0] * len(names), [1e6] * len(names)])
    )
    assert torch.all(action._processed_actions >= action._clip[:, :, 0])
    assert torch.all(action._processed_actions <= action._clip[:, :, 1])
    assert torch.allclose(action._processed_actions[1], action._offset[0])
    print(
        "PASS: reset geometry, joint margins, extreme-action clipping, and zero-action offset"
    )
    worst = 0
    gap = 0
    for k in range(5000):
        for a in range(m.nu):
            j = m.actuator_trnid[a, 0]
            kp, kd, lim = gains[m.joint(j).name.removeprefix("robot/")]
            d.ctrl[a] = np.clip(
                kp * (ref[m.jnt_qposadr[j]] - d.qpos[m.jnt_qposadr[j]])
                - kd * d.qvel[m.jnt_dofadr[j]],
                -lim,
                lim,
            )
        mujoco.mj_step(m, d)
        if k > 250:
            worst = max(worst, np.max(abs(d.ctrl) / m.actuator_forcerange[:, 1]))
            gap = max(
                gap,
                np.max(
                    abs(d.efc_pos[d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY])
                ),
            )
        if k % 500 == 499:
            tilt = np.degrees(
                np.arccos(np.clip(d.xmat[m.body("robot/floating_base").id, 8], -1, 1))
            )
            print(round(d.time, 1), "height", d.qpos[2], "tilt", tilt, flush=True)
    print("max torque fraction", worst, "loop gap", gap)
    assert d.qpos[2] > 0.6 and tilt < 10 and gap < 0.001 and worst < 0.9
    print("PASS: native MuJoCo 10-second crouch hold (not a Warp rollout)")


if __name__ == "__main__":
    main()
