"""Minimal standing in mjlab: one robot on a flat floor, joint PD holding the zero pose.

No weld, no policy, no RL environment: an mjlab Scene (plane terrain + ROBOT_CFG) is
stepped by mjlab's Simulation (MuJoCo Warp). Every physics step the PD actuators of
robot.py get the same fixed joint targets (all 0) and compute
tau = clamp(kp * (0 - q) - kd * dq, -peak, peak).

  mjlab/.venv/bin/python -m humanoid_mjlab.stand --headless   # checkpoint, 10 s
  mjlab/.venv/bin/mjpython -m humanoid_mjlab.stand            # viewer (macOS: mjpython)

The viewer is MuJoCo's passive viewer on a CPU copy of the scene model: each frame
the Warp state (qpos, qvel, ctrl) is copied into it and mj_forward recomputes contacts,
constraints and actuator forces. The checkpoint uses the same copy to check that both
soles touch the floor, that the ankle loops stay closed, and how much of each motor's
peak torque is used (compare with simplified/scripts/stand.py).
"""

from __future__ import annotations

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np
import torch
from mjlab.scene import Scene, SceneCfg
from mjlab.sim import Simulation, SimulationCfg
from mjlab.terrains import TerrainEntityCfg

from humanoid_mjlab.robot import ROBOT_CFG

PRINT_EVERY_S = 1.0


def build(device: str) -> tuple[Scene, Simulation]:
    scene_cfg = SceneCfg(
        num_envs=1,
        terrain=TerrainEntityCfg(terrain_type="plane"),
        entities={"robot": ROBOT_CFG},
    )
    scene = Scene(scene_cfg, device=device)
    sim_cfg = SimulationCfg()
    sim = Simulation(num_envs=1, cfg=sim_cfg, model=scene.compile(), device=device)
    scene.initialize(sim.mj_model, sim.model, sim.data)
    return scene, sim


def reset_to_zero_pose(scene: Scene, sim: Simulation) -> None:
    """Root at BASE_HEIGHT, all joints 0 (the ankle loops are closed there)."""
    robot = scene["robot"]
    sim.reset()
    scene.reset()
    root = robot.data.default_root_state.clone()
    root[:, 0:3] += scene.env_origins
    robot.write_root_state_to_sim(root)
    robot.write_joint_state_to_sim(
        robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
    )
    sim.forward()


def copy_to_cpu(sim: Simulation, data: mujoco.MjData) -> None:
    """Warp state of env 0 into a MuJoCo MjData, then forward (contacts, efc, forces)."""
    data.qpos[:] = sim.data.qpos[0].cpu().numpy()
    data.qvel[:] = sim.data.qvel[0].cpu().numpy()
    data.ctrl[:] = sim.data.ctrl[0].cpu().numpy()
    data.time = float(sim.data.time[0]) if sim.data.time.ndim else float(sim.data.time)
    mujoco.mj_forward(sim.mj_model, data)


class Monitor:
    """Standing checks on the CPU copy: height, tilt, sole contacts, loops, torque."""

    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self.base = model.body("robot/floating_base").id
        self.floor = model.geom("terrain").id
        self.soles = {
            side: model.geom(f"robot/{side}_foot_collision_box").id
            for side in ("left", "right")
        }
        self.peak = model.actuator_forcerange[:, 1]  # peak torque, set by robot.py
        self.max_loop_gap = 0.0
        self.max_torque_ratio = 0.0
        self.all_soles_down = True
        self.other_contacts = 0

    def sample(self, data: mujoco.MjData) -> dict:
        m = self.model
        tilt = np.degrees(np.arccos(np.clip(data.xmat[self.base][8], -1.0, 1.0)))
        touching = {side: False for side in self.soles}
        others = 0
        for c in data.contact[: data.ncon]:
            pair = {c.geom1, c.geom2}
            if self.floor not in pair:
                continue
            for side, g in self.soles.items():
                if g in pair:
                    touching[side] = True
                    break
            else:
                others += 1
        eq = data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
        gap = float(np.abs(data.efc_pos[eq]).max()) if eq.any() else 0.0
        tau = data.actuator_force  # the clamped PD torque mjlab wrote to ctrl
        ratio = float(np.max(np.abs(tau) / self.peak))
        return {
            "z": data.xpos[self.base][2],
            "tilt": tilt,
            "left": touching["left"],
            "right": touching["right"],
            "others": others,
            "gap_mm": gap * 1000,
            "ratio": ratio,
            "worst": m.actuator(int(np.argmax(np.abs(tau) / self.peak))).name,
        }

    def record(self, s: dict, settled: bool) -> None:
        if settled:  # ignore the first 0.5 s, while the feet settle onto the floor
            self.max_loop_gap = max(self.max_loop_gap, s["gap_mm"])
            self.max_torque_ratio = max(self.max_torque_ratio, s["ratio"])
            self.all_soles_down &= s["left"] and s["right"]
            self.other_contacts = max(self.other_contacts, s["others"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument(
        "--device", default="cuda:0" if torch.cuda.is_available() else "cpu"
    )
    args = parser.parse_args()

    scene, sim = build(args.device)
    robot = scene["robot"]
    dt = sim.mj_model.opt.timestep
    reset_to_zero_pose(scene, sim)

    # Fixed targets for every joint (only the 18 motorised ones have a PD).
    q_ref = torch.zeros_like(robot.data.default_joint_pos)
    zero = torch.zeros_like(q_ref)

    def physics_step() -> None:
        robot.set_joint_position_target(q_ref)
        robot.set_joint_velocity_target(zero)
        robot.set_joint_effort_target(zero)
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)

    cpu = mujoco.MjData(sim.mj_model)
    monitor = Monitor(sim.mj_model)
    steps_per_print = round(PRINT_EVERY_S / dt)

    if args.headless:
        print(
            f"mjlab stand: {sim.mj_model.nu} motors, dt {dt * 1000:.0f} ms, "
            f"device {args.device}"
        )
        n_steps = round(args.duration / dt)
        for k in range(1, n_steps + 1):
            physics_step()
            if k % 10 == 0 or k % steps_per_print == 0:
                copy_to_cpu(sim, cpu)
                s = monitor.sample(cpu)
                monitor.record(s, settled=k * dt >= 0.5)
                if k % steps_per_print == 0:
                    print(
                        f"t={k * dt:5.1f} s  base z={s['z']:.3f} m  "
                        f"tilt={s['tilt']:5.2f} deg  "
                        f"soles L/R={int(s['left'])}/{int(s['right'])}  "
                        f"loop gap={s['gap_mm']:.3f} mm  "
                        f"max |tau|/peak={s['ratio']:.2f} ({s['worst']})"
                    )
        s = monitor.sample(cpu)
        standing = s["z"] > 0.6 and s["tilt"] < 10.0
        print()
        print(f"  [{'ok' if standing else 'FAIL'}] standing without a weld")
        print(
            f"  [{'ok' if monitor.all_soles_down else 'FAIL'}] both soles on the floor "
            f"(other robot-floor contacts: {monitor.other_contacts})"
        )
        loops_ok = monitor.max_loop_gap < 1.0
        print(
            f"  [{'ok' if loops_ok else 'FAIL'}] ankle loops closed "
            f"(max gap {monitor.max_loop_gap:.3f} mm)"
        )
        print(f"  max |tau|/peak after settling: {monitor.max_torque_ratio:.2f}")
        if not (standing and monitor.all_soles_down and loops_ok):
            raise SystemExit(1)
        return

    copy_to_cpu(sim, cpu)
    with mujoco.viewer.launch_passive(sim.mj_model, cpu) as viewer:
        viewer.cam.lookat[:] = [0.0, 0.0, 0.5]
        viewer.cam.distance = 2.5
        viewer.cam.azimuth = 90.0  # robot faces +y: look at its front
        viewer.cam.elevation = -10.0
        while viewer.is_running():
            start = time.time()
            physics_step()
            copy_to_cpu(sim, cpu)
            viewer.sync()
            time.sleep(max(0.0, dt - (time.time() - start)))


if __name__ == "__main__":
    main()
