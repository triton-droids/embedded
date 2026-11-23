#!/usr/bin/env python3
"""Hold the zero pose (standing, legs straight, arms down) in MuJoCo with joint PD on the torque motors.

    python3 scripts/stand.py            # interactive viewer (macOS: mjpython scripts/stand.py)
    python3 scripts/stand.py --headless # run 10 s and print the base height and tilt

tau = kp * (q_ref - q) - kd * dq, clamped by each motor's ctrlrange (peak torque). The gains are the registry's
(config/motors_full_body.yaml). The ankle motor acts on the four-bar crank, so its PD runs on the crank angle:
the crank reference is the crank angle at the ankle reference (0 at the zero pose).

Ankle gains: standing still with no balance feedback, the robot is an inverted pendulum on its ankles
(31.9 kg, COM 0.64 m up: m*g*h = 200 N*m/rad). The registry's ankle kp of 20 (two ankles: 40) cannot hold it and the
robot tips backward within 2 s; on hardware the policy balances it. This script therefore uses --ankle-kp 150
(two ankles: 300 > 200). The crank-to-foot ratio is about 0.98 near zero, so the foot sees about the same stiffness.
"""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parent.parent
SCENE = PACKAGE / "mjcf/scene.xml"
REGISTRY = PACKAGE.parent / "config/motors_full_body.yaml"


def build_controller(model, ankle_kp, ankle_kd):
    motors = yaml.safe_load(REGISTRY.read_text())["motor_control_node"]["ros__parameters"]["motors"]
    qadr, dadr, kp, kd = [], [], [], []
    for i in range(model.nu):
        joint_id = model.actuator_trnid[i, 0]
        joint = model.joint(joint_id).name
        entry = motors[joint.replace("_crank_joint", "_joint")]
        qadr.append(model.jnt_qposadr[joint_id])
        dadr.append(model.jnt_dofadr[joint_id])
        is_ankle = joint.endswith("_ankle_crank_joint")
        kp.append(ankle_kp if is_ankle else entry["kp"])
        kd.append(ankle_kd if is_ankle else entry["kd"])
    return np.array(qadr), np.array(dadr), np.array(kp), np.array(kd)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--ankle-kp", type=float, default=150.0, help="N*m/rad on the crank (registry: 20)")
    parser.add_argument("--ankle-kd", type=float, default=3.0, help="N*m*s/rad on the crank (registry: 1)")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    qadr, dadr, kp, kd = build_controller(model, args.ankle_kp, args.ankle_kd)
    q_ref = np.zeros(model.nu)  # zero pose; every motor joint (crank included) is 0 there

    def control(m, d):
        d.ctrl[:] = kp * (q_ref - d.qpos[qadr]) - kd * d.qvel[dadr]  # MuJoCo clamps to ctrlrange

    mujoco.set_mjcb_control(control)
    if args.headless:
        base = model.body("floating_base").id
        while data.time < args.duration:
            mujoco.mj_step(model, data)
            if round(data.time / model.opt.timestep) % round(1.0 / model.opt.timestep) == 0:
                tilt = np.degrees(np.arccos(np.clip(data.xmat[base][8], -1, 1)))
                print(f"t={data.time:5.1f} s  base z={data.xpos[base][2]:.3f} m  tilt={tilt:5.2f} deg  "
                      f"max |tau|/peak={np.max(np.abs(data.ctrl) / model.actuator_ctrlrange[:, 1]):.2f}")
        return
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            start = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - (time.time() - start)))


if __name__ == "__main__":
    main()
