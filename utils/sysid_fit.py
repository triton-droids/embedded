#!/usr/bin/env python3
"""
Fit per-joint actuator parameters to a sysid_logger.py session.

Run with the CPU fitting environment (mujoco + scipy), not the robot .venv:

    ~/Documents/simulation/.venv-sysid/bin/python utils/sysid_fit.py logs/system_id/<session>

What it does, per motor:
  1. Builds the robot from the simulation repo's MJCF (read with `git show`, so
     the checkout is not touched), contacts off. The base is free but welded to
     a mocap target replaying the torso orientation the IMU measured: the robot
     hangs from a gantry and swings when a leg moves.
  2. Replays every trial: each joint gets an MIT-style PD actuator
     tau = kp*(cmd - q) - kd_scale*kd*qdot, and cmd is the logged post-slew,
     post-guard target, applied at its logged send time plus a delay. The ankle
     gains are mapped through the linkage ratio measured in the log (x r^2).
  3. Fits the moving joint's armature, Coulomb friction, kd scale and command
     delay to the `fit` trials by least squares on the measured position at each
     reply (receive time minus half the CAN round trip). kp stays at the
     commanded value; the motors' torque replies give an independent gain check.
  4. Scores the `holdout` trials with the fitted values and with the current
     training defaults, so the improvement is measured on data the fit never saw.

Output in <session>/fit/:
    sysid_params.json   fitted values, holdout scores, suggested randomisation ranges
    m<id>_<joint>.png   measured vs simulated, one step and one sine holdout trial
"""

from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

SIM_REPO = Path.home() / "Documents/simulation"
MJCF_REF = "origin/codex/hardware-tracking-retrain:cad/chrobot_hardware_candidate.xml"
SIM_DT = 0.0005
# Base motion replayed from the IMU: "fixed" (no motion), "yaw" (integrated gyro z
# only; the robot yaws almost freely on the gantry strap when a leg swings) or
# "imu" (yaw plus roll/pitch from the gravity estimate, which jitters by 1-2 deg).
BASE_MODE = "yaw"

# Training defaults in the codex branch (HARDWARE_TRACKING_CHANGES.md), used as the baseline.
NOMINAL = {"armature": 0.01, "frictionloss": 0.0, "damping": 0.1, "delay_s": 0.0, "kp_scale": 1.0,
           "kd_scale": 1.0}
# Other joints in a trial are only holding; give them plausible fixed values.
HELD = {"armature": 0.01, "frictionloss": 0.3, "damping": 0.1}


def load_mjcf() -> str:
    xml = subprocess.run(["git", "-C", str(SIM_REPO), "show", MJCF_REF],
                         capture_output=True, text=True, check=True).stdout
    # The robot hangs from a gantry and its torso swings when a leg moves (the
    # IMU shows ~0.9 correlation with hip velocity). Keep the free base and weld
    # it to a mocap target that replays the measured torso orientation.
    xml = xml.replace('<body name="floating_base"',
                      '<body name="base_target" mocap="true" pos="0 0 0.6846"/><body name="floating_base"', 1)
    xml = re.sub(r"\s*<mesh [^>]*/>", "", xml)                    # no mesh assets
    xml = re.sub(r"\s*<geom mesh=[^>]*/>", "", xml)               # no mesh geoms
    xml = re.sub(r'meshdir="[^"]*"', "", xml)
    actuators = "".join(
        f'<general name="{j}" joint="{j}" gaintype="fixed" biastype="affine" gainprm="0" biasprm="0 0 0"/>'
        for j in re.findall(r'<joint name="([^"]+)"', xml))
    weld = '<equality><weld body1="base_target" body2="floating_base" solref="0.02 1"/></equality>'
    return xml.replace("</worldbody>", f"</worldbody>{weld}<actuator>{actuators}</actuator>")


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_z(angle: float) -> np.ndarray:
    return np.array([math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2)])


def quat_from_gravity(g_body: np.ndarray) -> np.ndarray:
    """Smallest rotation R (body to world) with R @ g_body = (0, 0, -1)."""
    g = np.asarray(g_body, dtype=float)
    g = g / np.linalg.norm(g)
    down = np.array([0.0, 0.0, -1.0])
    axis = np.cross(g, down)
    s = np.linalg.norm(axis)
    c = float(np.dot(g, down))
    if s < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    angle = math.atan2(s, c)
    return np.concatenate([[math.cos(angle / 2)], axis / s * math.sin(angle / 2)])


class Trial:
    """One logged trial, in the simulator's joint convention and time base."""

    def __init__(self, path: Path):
        z = np.load(path)
        self.meta = json.loads(str(z["meta"]))
        self.name = self.meta["run"]
        self.file = path.name
        self.tag = (self.meta.get("trial") or {}).get("tag", "")
        info = self.meta["trial_info"]
        self.j = int(info["active_index"])
        self.joint_names = self.meta["joint_names"]
        sign = np.asarray(self.meta["joint_sign"], dtype=float)
        self.kp = np.asarray(self.meta["kp"], dtype=float)
        self.kd = np.asarray(self.meta["kd"], dtype=float)

        sent = z["sent"]
        rows = np.flatnonzero(np.isfinite(sent).all(axis=1))   # every step that wrote targets
        # Row 0 drained replies left over from connect(), so the state starts at
        # row 1: the reply to row 0's command, received at rx_ts[1].
        start = 1
        self.t0 = float(z["rx_ts"][start, self.j])
        self.cmd_t = z["tx_wall"][rows] - self.t0           # (n, 10) send time per motor
        self.cmd = sent[rows] * sign                         # (n, 10) sim convention
        q = z["joint_pos_unclamped"] * sign
        self.q_init = q[start]
        fresh = z["feedback_fresh"][:, self.j] > 0.5
        meas = np.flatnonzero(fresh & (np.arange(len(fresh)) > start))
        # A motor replies the moment a command reaches it, with its position at
        # that instant. rx_ts is the kernel receive time, later by the return trip
        # through CAN and USB; take that as half the measured round trip.
        rtt = z["rx_ts"][meas, self.j] - z["tx_wall"][meas - 1, self.j]
        self.half_rtt = 0.5 * float(np.median(rtt[np.isfinite(rtt) & (rtt > 0) & (rtt < 0.015)]))
        self.meas_t = z["rx_ts"][meas, self.j] - self.half_rtt - self.t0
        self.meas_q = q[meas, self.j]
        self.meas_cmd = sent[meas, self.j] * sign[self.j]

        # Torso orientation over time, from the IMU logged at each step. The IMU's
        # base frame is the model's (+X right, +Y forward, +Z up).
        self.base_t, self.base_quat = None, None
        if "proj_gravity" in z.files and np.isfinite(z["proj_gravity"][rows]).all():
            g = z["proj_gravity"][rows]
            w = z["ang_vel"][rows]
            bt = z["tx_wall"][rows, 0] - self.t0 - 0.001     # read just before the writes
            yaw = np.concatenate([[0.0], np.cumsum(0.5 * (w[1:, 2] + w[:-1, 2]) * np.diff(bt))])
            self.base_t = bt
            self.base_quat = np.stack([
                quat_mul(quat_z(a), quat_from_gravity(gi) if BASE_MODE == "imu" else np.array([1.0, 0, 0, 0]))
                for a, gi in zip(yaw, g)])
            self.base_tilt_max_deg = float(np.degrees(np.max(np.arccos(np.clip(-g[:, 2] / np.linalg.norm(g, axis=1), -1, 1)))))

        # Linkage ratio dmotor/djoint, from the motor and joint angles in the log.
        # 1 for direct-drive joints; about 0.95 for the ankles.
        self.ratio = np.ones(len(sign))
        mp, jp = z["motor_pos"], z["joint_pos_unclamped"]
        for i in range(len(sign)):
            dj = jp[rows, i] - jp[rows, i].mean()
            if np.ptp(jp[rows, i]) > math.radians(2.0):
                dm = mp[rows, i] - mp[rows, i].mean()
                self.ratio[i] = abs(float(np.dot(dm, dj) / np.dot(dj, dj)))
        self.duration = float(self.meas_t[-1])


class Simulator:
    def __init__(self):
        import mujoco
        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_string(load_mjcf())
        self.model.opt.timestep = SIM_DT
        self.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)   # hanging, no contacts
        self.data = mujoco.MjData(self.model)
        self.jid = {self.model.joint(i).name: i for i in range(self.model.njnt)}
        self.aid = {self.model.actuator(i).name: i for i in range(self.model.nu)}
        assert self.model.nu == 10 and self.model.njnt == 11, "expected a free base and 10 hinge joints"
        self.hinges = [n for n in self.jid if self.model.jnt_type[self.jid[n]] == mujoco.mjtJoint.mjJNT_HINGE]
        self.base_qadr = self.model.jnt_qposadr[self.jid[self.model.joint(0).name]]

    def run(self, trial: Trial, params: dict, ratio_override: dict | None = None) -> np.ndarray:
        """Simulated position of the moving joint at every measurement time."""
        mj, m, d = self.mj, self.model, self.data
        mj.mj_resetData(m, d)
        for i, name in enumerate(trial.joint_names):
            jid = self.jid[name]
            dof = m.jnt_dofadr[jid]
            p = params if i == trial.j else HELD
            m.dof_armature[dof] = p["armature"]
            m.dof_frictionloss[dof] = p["frictionloss"]
            m.dof_damping[dof] = p.get("damping", 0.0)
            r2 = trial.ratio[i] ** 2
            kp = trial.kp[i] * r2 * (params["kp_scale"] if i == trial.j else 1.0)
            kv = trial.kd[i] * r2 * (params.get("kd_scale", 1.0) if i == trial.j else 1.0)
            aid = self.aid[name]
            m.actuator_gainprm[aid, 0] = kp
            m.actuator_biasprm[aid, :3] = (0.0, -kp, -kv)
            d.qpos[m.jnt_qposadr[jid]] = trial.q_init[i]
        order = [self.aid[n] for n in trial.joint_names]
        base_q = (trial.base_quat if (trial.base_quat is not None and BASE_MODE != "fixed"
                                       and params.get("base_motion", True)) else None)
        quat0 = base_q[0] if base_q is not None else np.array([1.0, 0.0, 0.0, 0.0])
        d.qpos[self.base_qadr + 3:self.base_qadr + 7] = quat0
        d.mocap_quat[0] = quat0

        # Event timeline: command updates (per step, at the moving motor's send
        # time plus the delay; all ten are sent within ~1 ms) and measurements.
        delay = params["delay_s"]
        cmd_times = trial.cmd_t[:, trial.j] + delay
        ci = max(1, int(np.searchsorted(cmd_times, 0.0, side="right")))
        d.ctrl[order] = trial.cmd[ci - 1]          # the command in effect at t = 0
        mj.mj_forward(m, d)
        qadr = m.jnt_qposadr[self.jid[trial.joint_names[trial.j]]]
        out = np.empty(len(trial.meas_t))
        mi, bi, t = 0, 1, 0.0
        n_base = len(trial.base_t) if base_q is not None else 0
        while mi < len(trial.meas_t):
            next_cmd = cmd_times[ci] if ci < len(cmd_times) else math.inf
            next_base = trial.base_t[bi] if bi < n_base else math.inf
            next_meas = trial.meas_t[mi]
            t_next = min(next_cmd, next_meas, next_base)
            n = int(round((t_next - t) / SIM_DT))
            if n > 0:
                mj.mj_step(m, d, nstep=n)
                t += n * SIM_DT
            if next_meas <= next_cmd and next_meas <= next_base:
                out[mi] = d.qpos[qadr]
                mi += 1
            elif next_base <= next_cmd:
                d.mocap_quat[0] = base_q[bi]
                bi += 1
            else:
                d.ctrl[order] = trial.cmd[ci]
                ci += 1
        return out


def rms_deg(sim: Simulator, trials: list[Trial], params: dict) -> float:
    err = np.concatenate([sim.run(t, params) - t.meas_q for t in trials])
    return math.degrees(float(np.sqrt(np.mean(err ** 2))))


# The delay is searched on a grid: a finite-difference step on it would be
# smaller than one simulation step and leave it stuck at its starting value.
DELAY_GRID_MS = [0, 2, 4, 7, 12]
# kp stays at the commanded value: in a position response, kp trades off against
# friction and armature (the first fit ran them into their bounds), and the
# motors' own torque replies show the applied gain at or below nominal.
# The viscous term is fitted as a scale on the commanded kd: the torque replies
# put the applied velocity gain near 0.5-1 Nm s/rad against the 2 sent, and an
# added joint damping (>= 0) could only ever make the model more damped.
CONT = ["armature", "frictionloss", "kd_scale"]
# The optimiser works on p / SCALE so every parameter is order 1 and the
# finite-difference steps are sensible (armature 0.0005, friction 0.005 Nm, ...).
SCALE = np.array([0.01, 0.1, 1.0])
C0 = np.array([0.01, 0.3, 1.0]) / SCALE
CLO = np.array([0.0005, 0.0, 0.1]) / SCALE
CHI = np.array([0.3, 3.0, 3.0]) / SCALE


def fit_trials(sim: "Simulator", fit: list["Trial"]) -> tuple[dict, dict]:
    """Least squares on the continuous parameters at each grid delay; keep the best."""
    from scipy.optimize import least_squares

    def solve(delay_s: float, u0: np.ndarray):
        def residual(u: np.ndarray) -> np.ndarray:
            p = dict(zip(CONT, u * SCALE), delay_s=delay_s, kp_scale=1.0, damping=0.0)
            return np.degrees(np.concatenate([sim.run(t, p) - t.meas_q for t in fit]))
        return least_squares(residual, u0, bounds=(CLO, CHI), diff_step=0.05, max_nfev=40)

    grid = {}
    best = None
    for ms in DELAY_GRID_MS:
        res = solve(ms / 1000.0, C0 if best is None else best[1].x)
        grid[ms] = float(res.cost)
        if best is None or res.cost < best[1].cost:
            best = (ms, res)
    ms, res = best
    for step in (1.0, 0.5):                               # refine down to the 0.5 ms sim step
        centre = ms
        for nb in (centre - step, centre + step):
            if nb >= 0 and nb not in grid:
                r = solve(nb / 1000.0, res.x)
                grid[nb] = float(r.cost)
                if r.cost < res.cost:
                    ms, res = nb, r
    params = dict(zip(CONT, (float(v) for v in res.x * SCALE)), delay_s=ms / 1000.0, kp_scale=1.0,
                  damping=0.0)
    info = {"cost": float(res.cost), "delay_grid_cost": {str(k): v for k, v in sorted(grid.items())}}
    return params, info


def torque_reply_gains(files: list[str]) -> dict:
    """Regress each reply's torque on (command - position) and velocity, motor space.
    A reply carries the state just before the new command is applied, so the
    torque belongs to the command sent two steps earlier. Pooled over the trials."""
    E, V, T, kp, kd = [], [], [], None, None
    for f in files:
        z = np.load(f)
        meta = json.loads(str(z["meta"]))
        j = meta["trial_info"]["active_index"]
        kp, kd = meta["kp"][j], meta["kd"][j]
        fresh = z["feedback_fresh"][:, j] > 0.5
        cmd, pos, vel, tau = (z[k][:, j] for k in ("motor_cmd", "motor_pos", "motor_vel", "motor_torque"))
        k = np.arange(2, len(tau))
        ok = fresh[k] & np.isfinite(cmd[k - 2])
        E.append((cmd[k - 2] - pos[k])[ok]); V.append(vel[k][ok]); T.append(tau[k][ok])
    E, V, T = np.concatenate(E), np.concatenate(V), np.concatenate(T)
    A = np.stack([E, V, np.ones_like(E)], axis=1)
    coef, *_ = np.linalg.lstsq(A, T, rcond=None)
    r2 = 1.0 - float(np.sum((T - A @ coef) ** 2) / np.sum((T - T.mean()) ** 2))
    return {"kp_ratio": float(coef[0] / kp), "kd_measured": float(-coef[1]), "kd_nominal": float(kd),
            "r2": r2, "samples": int(len(T)),
            "note": "reported torque depends on the motor's own torque-constant calibration"}


def fit_motor(args: tuple[int, list[str], list[str]]) -> dict:
    motor_id, fit_files, hold_files = args
    sim = Simulator()
    fit = [Trial(Path(f)) for f in fit_files]
    hold = [Trial(Path(f)) for f in hold_files]
    t0 = time.time()
    params, info = fit_trials(sim, fit)
    tr = fit[0]
    return {
        "motor_id": motor_id,
        "joint": tr.joint_names[tr.j],
        "model": tr.meta["motor_models"][tr.j],
        "kp": float(tr.kp[tr.j]),
        "kd": float(tr.kd[tr.j]),
        "linkage_ratio": float(np.median([t.ratio[t.j] for t in fit + hold])),
        "can_half_round_trip_s": float(np.median([t.half_rtt for t in fit + hold])),
        "fitted": params,
        "rms_deg": {
            "fit_trials_fitted": rms_deg(sim, fit, params),
            "fit_trials_nominal": rms_deg(sim, fit, NOMINAL),
            "holdout_fitted": rms_deg(sim, hold, params) if hold else None,
            "holdout_nominal": rms_deg(sim, hold, NOMINAL) if hold else None,
            "holdout_fitted_base_fixed": rms_deg(sim, hold, {**params, "base_motion": False}) if hold else None,
        },
        "torque_reply": torque_reply_gains(fit_files + hold_files),
        "base_tilt_max_deg": float(max(getattr(t, "base_tilt_max_deg", 0.0) for t in fit + hold)),
        "trials": {"fit": [t.file for t in fit], "holdout": [t.file for t in hold]},
        "per_holdout_trial_deg": {t.name + "|" + t.file: {"fitted": rms_deg(sim, [t], params),
                                                         "nominal": rms_deg(sim, [t], NOMINAL)}
                                  for t in hold},
        "optimizer": {**info, "seconds": round(time.time() - t0, 1)},
    }


def selftest_motor(args: tuple[int, list[str], dict]) -> dict:
    """Synthetic recovery: simulate the real fit trials' commands with known
    parameters, add encoder-sized noise, fit, and compare."""
    motor_id, fit_files, truth = args
    sim = Simulator()
    fit = [Trial(Path(f)) for f in fit_files]
    rng = np.random.default_rng(motor_id)
    for t in fit:
        t.meas_q = sim.run(t, truth) + rng.normal(0.0, math.radians(0.02), len(t.meas_t))
    params, info = fit_trials(sim, fit)
    return {"motor_id": motor_id, "truth": truth, "recovered": params,
            "rms_deg": rms_deg(sim, fit, params)}


def signcheck_motor(args: tuple[int, list[str]]) -> dict:
    """Fit with the moving joint's hardware->sim sign as configured and flipped.
    Gravity only pulls one way, so the right sign should fit clearly better."""
    motor_id, fit_files = args
    sim = Simulator()
    out = {"motor_id": motor_id}
    for label, flip in (("configured", False), ("flipped", True)):
        fit = [Trial(Path(f)) for f in fit_files]
        if flip:
            for t in fit:
                t.cmd[:, t.j] *= -1
                t.q_init[t.j] *= -1
                t.meas_q = -t.meas_q
                t.meas_cmd = -t.meas_cmd
        params, info = fit_trials(sim, fit)
        out[label] = {"rms_deg": rms_deg(sim, fit, params), "params": params}
    return out


def plot_motor(result: dict, out_dir: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    sim = Simulator()
    hold = [Trial(Path(out_dir.parent / f)) for f in result["trials"]["holdout"]]
    picks = [t for t in hold if "step" in t.name][:1] + [t for t in hold if "sine" in t.name][:1]
    if not picks:
        return
    fig, axes = plt.subplots(len(picks), 1, figsize=(11, 3.6 * len(picks)), squeeze=False)
    for ax, t in zip(axes[:, 0], picks):
        fitted = sim.run(t, result["fitted"])
        nominal = sim.run(t, NOMINAL)
        ax.plot(t.meas_t, np.degrees(t.meas_cmd), color="0.6", lw=1, label="command sent (post slew)")
        ax.plot(t.meas_t, np.degrees(t.meas_q), "k", lw=1.6, label="measured")
        ax.plot(t.meas_t, np.degrees(nominal), "--", color="tab:red", lw=1.2,
                label=f"sim, training defaults ({math.degrees(np.sqrt(np.mean((nominal - t.meas_q) ** 2))):.2f} deg RMS)")
        ax.plot(t.meas_t, np.degrees(fitted), color="tab:blue", lw=1.2,
                label=f"sim, fitted ({math.degrees(np.sqrt(np.mean((fitted - t.meas_q) ** 2))):.2f} deg RMS)")
        ax.set_title(f"m{result['motor_id']} {result['joint']}  holdout {t.name}")
        ax.set_xlabel("s")
        ax.set_ylabel("deg (sim convention)")
        ax.legend(fontsize=8, loc="best")
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / f"m{result['motor_id']}_{result['joint']}.png", dpi=110)
    plt.close(fig)


SKIPPED: list[str] = []
EXCLUDED: dict[str, str] = {}


def collect(session: Path) -> dict[int, tuple[list[str], list[str]]]:
    manifest = json.loads((session / "manifest.json").read_text())
    exclude_path = session / "fit_exclude.json"
    exclude = json.loads(exclude_path.read_text())["exclude"] if exclude_path.is_file() else {}
    groups: dict[int, tuple[list[str], list[str]]] = {}
    for run in manifest["runs"]:
        if run.get("mode") != "trial" or run.get("stop_reason") != "completed":
            continue      # skips connect failures, Ctrl+C and the battery-floor stop
        if run.get("file") in exclude:
            EXCLUDED[run["file"]] = exclude[run["file"]]
            continue
        path = session / run["file"]
        if not path.is_file() or path.stat().st_size == 0:
            SKIPPED.append(run["file"])
            continue
        mid = int(run["trial"]["motor_id"])
        fit, hold = groups.setdefault(mid, ([], []))
        (fit if run["trial"]["tag"] == "fit" else hold).append(str(session / run["file"]))
    return groups


# Prior ranges used when a fitted value sits on its bound: the data could not pin
# it (e.g. hip armature, where the hanging robot moves under the leg).
PRIOR_RANGES = {"armature": [0.005, 0.03], "frictionloss": [0.1, 1.0], "kd_scale": [0.5, 1.5]}
FIT_BOUNDS = {"armature": (0.0005, 0.3), "frictionloss": (0.0, 3.0), "kd_scale": (0.1, 3.0)}


def joint_type(joint: str) -> str:
    return joint.split("_")[1]      # left_hip1_joint -> hip1


# On the gantry the robot moves under a swinging leg, so the hip fits describe the
# hanging robot as much as the motor: hip1 (RS-04) fits ~0.02 Nm friction with
# doubled damping while the knee, the same motor, measures 0.6-0.9 Nm. For these
# joint types the ranges are widened to cover the same motor's clean fit, and the
# fitted nominal values are not used.
GANTRY_CONFOUNDED = {"hip1": "knee", "hip2": "thigh"}


def suggest_ranges(results: list[dict]) -> dict:
    """Randomisation ranges per joint type (hip1, hip2, thigh, knee, ankle): the
    spread of the left and right fits, widened by 30% on each side. A value on
    its fit bound is replaced by the prior range and flagged."""
    by_type: dict[str, list[dict]] = {}
    for r in results:
        by_type.setdefault(joint_type(r["joint"]), []).append(r["fitted"])
    out = {}
    for jtype, fits in by_type.items():
        out[jtype] = {"at_bound": []}
        for key in ("armature", "frictionloss", "kd_scale"):
            vals = [f[key] for f in fits]
            lo_b, hi_b = FIT_BOUNDS[key]
            on_bound = [v for v in vals if v <= lo_b * 1.05 + 1e-9 or v >= hi_b * 0.95]
            if on_bound:
                out[jtype][key] = list(PRIOR_RANGES[key])
                out[jtype]["at_bound"].append(key)
            else:
                out[jtype][key] = [round(min(vals) * 0.7, 4), round(max(vals) * 1.3, 4)]
        delays = [f["delay_s"] for f in fits]
        out[jtype]["delay_s"] = [0.0, round(max(0.005, max(delays) * 1.3), 4)]
    for jtype, reference in GANTRY_CONFOUNDED.items():
        if jtype in out and reference in out:
            for key in ("armature", "frictionloss", "kd_scale"):
                if key in out[jtype]["at_bound"]:
                    continue        # already the prior range
                a, b = out[jtype][key], out[reference][key]
                out[jtype][key] = [min(a[0], b[0]), max(a[1], b[1])]
            out[jtype]["untrusted"] = ["armature", "frictionloss", "kd_scale"]
            out[jtype]["note"] = (f"gantry-confounded; range = own fit U {reference} fit (same motor model), "
                                  f"nominal from the codex estimates")
    return out


def main() -> None:
    global BASE_MODE
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("session")
    p.add_argument("--motors", default="", help="comma-separated motor IDs (default: all)")
    p.add_argument("--workers", type=int, default=8)
    p.add_argument("--base", choices=("fixed", "yaw", "imu"), default=BASE_MODE,
                   help="base motion replayed from the IMU (default yaw)")
    p.add_argument("--out", default="fit", help="output directory name inside the session")
    p.add_argument("--ranges-only", action="store_true",
                   help="recompute suggested_randomisation in an existing <out>/sysid_params.json")
    p.add_argument("--selftest", action="store_true",
                   help="Synthetic recovery check: fit data simulated with known parameters.")
    p.add_argument("--signcheck", action="store_true",
                   help="Fit each joint with its hardware->sim sign as configured and flipped.")
    args = p.parse_args()
    session = Path(args.session).expanduser().resolve()
    BASE_MODE = args.base
    out_dir = session / args.out
    out_dir.mkdir(exist_ok=True)
    if args.ranges_only:
        path = out_dir / "sysid_params.json"
        data = json.loads(path.read_text())
        data["suggested_randomisation"] = suggest_ranges(data["joints"])
        path.write_text(json.dumps(data, indent=2))
        print(json.dumps(data["suggested_randomisation"], indent=1))
        return
    groups = collect(session)
    if args.motors:
        keep = {int(m) for m in args.motors.split(",")}
        groups = {k: v for k, v in groups.items() if k in keep}
    if SKIPPED:
        print(f"[FIT] skipping {len(SKIPPED)} empty or missing files: {SKIPPED}")
    if EXCLUDED:
        print(f"[FIT] excluding {len(EXCLUDED)} trials listed in fit_exclude.json")
    print(f"[FIT] {len(groups)} motors, {sum(len(f) + len(h) for f, h in groups.values())} trials, "
          f"{args.workers} workers")
    if args.selftest:
        truth = {"armature": 0.03, "frictionloss": 0.6, "damping": 0.0, "delay_s": 0.004, "kp_scale": 1.0,
                 "kd_scale": 0.6}
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            res = list(pool.map(selftest_motor, [(m, f, truth) for m, (f, _) in sorted(groups.items())]))
        print(f"[SELFTEST] truth: {truth}")
        for r in res:
            print(f"  m{r['motor_id']:<2} " + "  ".join(f"{k} {v:.4g}" for k, v in r["recovered"].items())
                  + f"  | residual {r['rms_deg']:.3f} deg")
        (out_dir / "selftest.json").write_text(json.dumps(res, indent=2))
        return
    if args.signcheck:
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            res = list(pool.map(signcheck_motor, [(m, f) for m, (f, _) in sorted(groups.items())]))
        for r in res:
            c, fl = r["configured"]["rms_deg"], r["flipped"]["rms_deg"]
            print(f"  m{r['motor_id']:<2} configured {c:.3f} deg  flipped {fl:.3f} deg  -> "
                  f"{'configured sign fits better' if c < fl else 'FLIPPED FITS BETTER'} ({fl / c:.2f}x)")
        (out_dir / "signcheck.json").write_text(json.dumps(res, indent=2))
        return
    jobs = [(mid, f, h) for mid, (f, h) in sorted(groups.items())]
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results = list(pool.map(fit_motor, jobs))
    results.sort(key=lambda r: r["motor_id"])
    for r in results:
        f, s = r["fitted"], r["rms_deg"]
        print(f"  m{r['motor_id']:<2} {r['joint']:<18} arm {f['armature']:.4f}  fric {f['frictionloss']:.3f} Nm  "
              f"kd x{f['kd_scale']:.2f}  delay {f['delay_s'] * 1000:4.1f} ms | "
              + (f"holdout RMS {s['holdout_nominal']:.2f} -> {s['holdout_fitted']:.2f} deg "
                 f"(base fixed {s['holdout_fitted_base_fixed']:.2f})" if s["holdout_fitted"] is not None
                 else f"fit RMS {s['fit_trials_nominal']:.2f} -> {s['fit_trials_fitted']:.2f} deg (no holdout trials)")
              + f" | torque-reply kp x{r['torque_reply']['kp_ratio']:.2f} (R2 {r['torque_reply']['r2']:.2f})"
              + f"  ({r['optimizer']['seconds']} s)")
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        list(pool.map(plot_motor, results, [out_dir] * len(results)))
    sim_head = subprocess.run(["git", "-C", str(SIM_REPO), "rev-parse", MJCF_REF.split(":")[0]],
                              capture_output=True, text=True).stdout.strip()
    out = {
        "session": session.name,
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "mjcf": {"ref": MJCF_REF, "commit": sim_head, "base_mode": BASE_MODE, "sim_dt": SIM_DT},
        "model": "tau = kp*r^2*(cmd(t - delay) - q) - kd_scale*kd*r^2*qdot with the commanded kp; armature and "
                 "frictionloss (Coulomb, MuJoCo) on the joint; cmd = logged post-slew "
                 "target; free base welded to the torso orientation measured by the IMU (gantry swing)",
        "nominal_baseline": NOMINAL,
        "joints": results,
        "suggested_randomisation": suggest_ranges(results),
        "skipped_files": SKIPPED,
        "excluded_trials": EXCLUDED,
        "seconds": round(time.time() - t0, 1),
    }
    (out_dir / "sysid_params.json").write_text(json.dumps(out, indent=2))
    print(f"[FIT] {out_dir / 'sysid_params.json'}  ({out['seconds']} s)")


if __name__ == "__main__":
    main()
