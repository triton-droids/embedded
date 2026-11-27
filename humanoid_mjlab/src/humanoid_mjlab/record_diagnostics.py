"""Record a deterministic rollout as an MP4 with live gait-diagnostic panels.

Layout (1920 x 1080): sim camera top-left; right column = gait stats text, knee,
ankle crank, hip pitch, forward speed; bottom row = foot height + contact,
sole pitch/roll, |torque| / peak. Plots show a rolling window ending "now".
Left leg is blue, right leg orange. Solid = measured, dashed = policy target,
dotted grey = target clip bounds.

Also writes <output>.json (summary gait stats, steady state after --settle s)
and <output>_telemetry.npz (every step) for offline analysis.

Run on the Antioch session from /workspace/project:
  embedded/humanoid_mjlab/.venv/bin/python -m humanoid_mjlab.record_diagnostics \
      --checkpoint logs/rsl_rl/humanoid_forward/<run>/model_4999.pt \
      --output outputs/diag_model_4999.mp4
"""

from __future__ import annotations

import argparse
import json
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import asdict
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mediapy as media  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from mjlab.actuator import IdealPdActuatorCfg  # noqa: E402
from mjlab.envs import ManagerBasedRlEnv  # noqa: E402
from mjlab.rl import RslRlVecEnvWrapper  # noqa: E402
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner  # noqa: E402

from humanoid_mjlab import robot as robot_module  # noqa: E402
from humanoid_mjlab.env_cfg import ACTION_SCALE, make_env_cfg  # noqa: E402
from humanoid_mjlab.rl_cfg import make_rl_cfg  # noqa: E402

W, H = 1920, 1080
VW, VH = 1280, 720
SIDES = ("left", "right")
COLOR = {"left": "#1f77b4", "right": "#ff7f0e"}
DEG = 180.0 / np.pi


# --------------------------------------------------------------------- helpers
def quat_to_mat(q: np.ndarray) -> np.ndarray:
    """(..., 4) wxyz -> (..., 3, 3)."""
    w, x, y, z = np.moveaxis(q, -1, 0)
    return np.stack(
        [
            np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
            np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
            np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1),
        ],
        -2,
    )


def peak_torques(names: list[str]) -> np.ndarray:
    cfgs = [v for v in vars(robot_module).values() if isinstance(v, IdealPdActuatorCfg)]
    out = []
    for n in names:
        lim = next(
            c.effort_limit for c in cfgs if any(re.fullmatch(p, n) for p in c.target_names_expr)
        )
        out.append(float(lim))
    return np.array(out)


def xml_joint_ranges() -> dict[str, tuple[float, float]]:
    ranges = {}
    for j in ET.parse(robot_module.ROBOT_XML).getroot().findall(".//worldbody//joint"):
        if "range" in j.attrib and j.get("limited") != "false":
            lo, hi = map(float, j.attrib["range"].split())
            ranges[j.attrib["name"]] = (lo, hi)
    return ranges


# --------------------------------------------------------------------- rollout
def rollout(checkpoint: Path, seconds: float, seed: int):
    cfg = make_env_cfg(play=True)
    cfg.seed = seed
    cfg.scene.num_envs = 1
    cfg.viewer.width, cfg.viewer.height = VW, VH
    agent_cfg = make_rl_cfg()
    env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0", render_mode="rgb_array")
    wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    try:
        runner = VelocityOnPolicyRunner(wrapped, asdict(agent_cfg), device="cuda:0")
        runner.load(str(checkpoint), load_cfg={"actor": True}, strict=True, map_location="cuda:0")
        policy = runner.get_inference_policy(device="cuda:0")
        obs, _ = wrapped.reset()
        robot = env.scene["robot"]
        contact_sensor = env.scene["feet_ground_contact"]
        action_term = env.action_manager.get_term("joint_pos")
        foot_body = [robot.body_names.index(f"{s}_foot_link") for s in SIDES]
        foot_site = [robot.site_names.index(f"{s}_foot") for s in SIDES]
        contact_order = [list(contact_sensor.primary_names).index(f"{s}_foot_link") for s in SIDES]

        def a(t):
            return t[0].detach().float().cpu().numpy().copy()

        keys = (
            "joint_pos joint_vel torque target raw_action contact force foot_quat "
            "foot_pos root_pos root_quat root_vel root_ang_vel command reward done"
        ).split()
        rec = {k: [] for k in keys}
        frames = []
        steps = int(round(seconds / env.step_dt))
        with torch.inference_mode():
            for i in range(steps):
                actions = policy(obs)
                if not torch.isfinite(actions).all():
                    raise RuntimeError("Policy produced non-finite actions")
                obs, reward, done, _ = wrapped.step(actions)
                frame = env.render()
                if frame.ndim == 4:
                    frame = frame[0]
                frames.append(np.asarray(frame)[:, :, :3].copy())
                d = robot.data
                rec["joint_pos"].append(a(d.joint_pos))
                rec["joint_vel"].append(a(d.joint_vel))
                rec["torque"].append(a(d.actuator_force))
                rec["target"].append(a(action_term._processed_actions))
                rec["raw_action"].append(a(actions))
                rec["contact"].append(a(contact_sensor.data.found)[contact_order] > 0)
                rec["force"].append(a(contact_sensor.data.force)[contact_order])
                rec["foot_quat"].append(a(d.body_link_quat_w)[foot_body])
                rec["foot_pos"].append(a(d.site_pos_w)[foot_site])
                rec["root_pos"].append(a(d.root_link_pos_w))
                rec["root_quat"].append(a(d.root_link_quat_w))
                rec["root_vel"].append(a(d.root_link_lin_vel_b))
                rec["root_ang_vel"].append(a(d.root_link_ang_vel_b))
                rec["command"].append(a(env.command_manager.get_command("twist")))
                rec["reward"].append(float(reward[0]))
                rec["done"].append(bool(done[0]))
                if (i + 1) % 100 == 0:
                    print(f"rollout {i + 1}/{steps}", flush=True)
        tel = {k: np.array(v) for k, v in rec.items()}
        meta = {
            "joint_names": list(robot.joint_names),
            "actuator_names": list(robot.actuator_names),
            "target_names": list(action_term.target_names),
            "dt": float(env.step_dt),
        }
        return tel, meta, frames
    finally:
        wrapped.close()


# --------------------------------------------------------------------- derived signals
def derive(tel, meta):
    dt = meta["dt"]
    n = len(tel["done"])
    t = np.arange(1, n + 1) * dt
    jn, an, tn = meta["joint_names"], meta["actuator_names"], meta["target_names"]
    bounds = {name: tuple(b) for name, b in _target_bounds().items()}
    peak = peak_torques(an)

    def jpos(name):
        return tel["joint_pos"][:, jn.index(name)]

    def tgt(name):
        return tel["target"][:, tn.index(name)]

    def tau_ratio(name):
        k = an.index(name)
        return np.abs(tel["torque"][:, k]) / peak[k]

    # Heading frame: yaw of the root.
    rq = tel["root_quat"]
    yaw = np.arctan2(2 * (rq[:, 0] * rq[:, 3] + rq[:, 1] * rq[:, 2]), 1 - 2 * (rq[:, 2] ** 2 + rq[:, 3] ** 2))
    fwd = np.stack([np.cos(yaw), np.sin(yaw)], -1)
    left = np.stack([-np.sin(yaw), np.cos(yaw)], -1)

    # Sole tilt relative to the first-step foot orientation (feet start flat).
    R = quat_to_mat(tel["foot_quat"])  # (n, 2, 3, 3)
    up_local = np.einsum("sji,j->si", R[0], np.array([0.0, 0.0, 1.0]))  # R0^T z
    up = np.einsum("nsij,sj->nsi", R, up_local)  # world "sole up" vector
    up_f = np.einsum("nsi,ni->ns", up[..., :2], fwd)
    up_l = np.einsum("nsi,ni->ns", up[..., :2], left)
    sole_pitch = np.arctan2(-up_f, up[..., 2]) * DEG  # + = toe up
    sole_roll = np.arctan2(up_l, up[..., 2]) * DEG

    sig = {"t": t, "yaw": yaw}
    for s in SIDES:
        for j in ("knee", "ankle_crank", "hip1", "hip2", "thigh"):
            name = f"{s}_{j}_joint"
            sig[f"{s}_{j}"] = jpos(name) * DEG
            sig[f"{s}_{j}_tgt"] = tgt(name) * DEG
            sig[f"{s}_{j}_tau"] = tau_ratio(name)
            lo, hi = bounds[name]
            sig[f"{s}_{j}_atbound"] = (np.abs(tgt(name) - lo) < 1e-4) | (np.abs(tgt(name) - hi) < 1e-4)
        sig[f"{s}_ankle"] = jpos(f"{s}_ankle_joint") * DEG
    k = {s: i for i, s in enumerate(SIDES)}
    for s in SIDES:
        sig[f"{s}_contact"] = tel["contact"][:, k[s]]
        sig[f"{s}_height"] = tel["foot_pos"][:, k[s], 2]
        sig[f"{s}_sole_pitch"] = sole_pitch[:, k[s]]
        sig[f"{s}_sole_roll"] = sole_roll[:, k[s]]
        sig[f"{s}_fz"] = tel["force"][:, k[s], 2]
    sig["vx"] = tel["root_vel"][:, 0]
    sig["vy"] = tel["root_vel"][:, 1]
    sig["cmd_vx"] = tel["command"][:, 0]

    # Gait events per foot.
    events = {}
    fp = tel["foot_pos"][..., :2]
    for s in SIDES:
        c = sig[f"{s}_contact"].astype(int)
        o = "right" if s == "left" else "left"
        td = np.flatnonzero(np.diff(c) == 1) + 1
        lo_ = np.flatnonzero(np.diff(c) == -1) + 1
        swings, lengths, widths = [], [], []
        for i in td:
            prev_lift = lo_[lo_ < i]
            swings.append((i - prev_lift[-1]) * dt if len(prev_lift) else np.nan)
            rel = fp[i, k[s]] - fp[i, k[o]]
            lengths.append(float(rel @ fwd[i]))
            widths.append(float(abs(rel @ left[i])))
        events[s] = {
            "touchdown": td,
            "liftoff": lo_,
            "swing": np.array(swings),
            "step_len": np.array(lengths),
            "step_width": np.array(widths),
        }
    return sig, events, bounds


def _target_bounds():
    from humanoid_mjlab.crouch import joint_target_bounds

    return joint_target_bounds(tuple(ACTION_SCALE))


def window_stats(sig, events, i0, i1, dt):
    """Gait statistics over samples [i0, i1]."""
    sl = slice(i0, i1 + 1)
    span = max((i1 - i0 + 1) * dt, dt)
    cl, cr = sig["left_contact"][sl], sig["right_contact"][sl]
    out = {
        "vx_mean": float(np.mean(sig["vx"][sl])),
        "vy_mean": float(np.mean(sig["vy"][sl])),
        "double_support_pct": float(np.mean(cl & cr) * 100),
        "flight_pct": float(np.mean(~cl & ~cr) * 100),
    }
    n_td = 0
    for s in SIDES:
        e = events[s]
        m = (e["touchdown"] >= i0) & (e["touchdown"] <= i1)
        n_td += int(m.sum())
        c = sig[f"{s}_contact"][sl]
        out[s] = {
            "steps": int(m.sum()),
            "step_len_cm": float(np.nanmean(e["step_len"][m]) * 100) if m.any() else np.nan,
            "step_width_cm": float(np.nanmean(e["step_width"][m]) * 100) if m.any() else np.nan,
            "swing_s": float(np.nanmean(e["swing"][m])) if m.any() else np.nan,
            "stance_pct": float(np.mean(c) * 100),
            "knee_deg_min": float(np.min(sig[f"{s}_knee"][sl])),
            "knee_deg_max": float(np.max(sig[f"{s}_knee"][sl])),
            "ankle_crank_deg_range": float(np.ptp(sig[f"{s}_ankle_crank"][sl])),
            "hip_pitch_deg_range": float(np.ptp(sig[f"{s}_hip1"][sl])),
            "knee_tgt_at_bound_pct": float(np.mean(sig[f"{s}_knee_atbound"][sl]) * 100),
            "ankle_tgt_at_bound_pct": float(np.mean(sig[f"{s}_ankle_crank_atbound"][sl]) * 100),
            "hip_tgt_at_bound_pct": float(np.mean(sig[f"{s}_hip1_atbound"][sl]) * 100),
            "stance_sole_pitch_deg": float(np.mean(sig[f"{s}_sole_pitch"][sl][c])) if c.any() else np.nan,
            "peak_knee_tau_ratio": float(np.max(sig[f"{s}_knee_tau"][sl])),
            "peak_ankle_tau_ratio": float(np.max(sig[f"{s}_ankle_crank_tau"][sl])),
            "max_foot_height_cm": float(np.max(sig[f"{s}_height"][sl]) * 100),
        }
    out["cadence_steps_per_s"] = n_td / span
    return out


# --------------------------------------------------------------------- video
def px(x, y, w, h):
    """Axes rect from pixel box with y measured from the top."""
    return [x / W, 1 - (y + h) / H, w / W, h / H]


def lim(*arrays, pad=0.08, extra=()):
    v = np.concatenate([np.ravel(a) for a in arrays] + [np.ravel(extra)])
    v = v[np.isfinite(v)]
    lo, hi = float(v.min()), float(v.max())
    span = max(hi - lo, 1e-3)
    return lo - pad * span, hi + pad * span


def compose(frames, sig, events, bounds, meta, output: Path, window: float, label: str):
    dt = meta["dt"]
    fps = round(1 / dt)
    n = len(frames)
    nw = int(round(window / dt))
    xs = (np.arange(nw) - (nw - 1)) * dt  # -window .. 0

    plt.rcParams.update({"font.size": 8, "axes.titlesize": 9, "axes.titleweight": "bold"})
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100, facecolor="white")
    animated = []  # (artist, updater)

    def axes(rect, title, ylabel, ylim, hlines=(), xlabels=True):
        ax = fig.add_axes(rect)
        if not xlabels:
            ax.tick_params(labelbottom=False)
        ax.set_title(title, loc="left", pad=3)
        ax.set_ylabel(ylabel)
        ax.set_xlim(xs[0], 0)
        ax.set_ylim(*ylim)
        ax.grid(alpha=0.3)
        ax.tick_params(labelsize=7)
        for y, kw in hlines:
            ax.axhline(y, **kw)
        return ax

    def line(ax, key, **kw):
        (ln,) = ax.plot([], [], animated=True, **kw)
        animated.append((ln, key))
        return ln

    bound_kw = {"color": "0.45", "ls": ":", "lw": 1}

    def joint_plot(rect, joint, title):  # no x labels: stacked column
        names = [f"{s}_{joint}_joint" for s in SIDES]
        bl = [np.array(bounds[nm]) * DEG for nm in names]
        keys = [f"{s}_{joint}{suf}" for s in SIDES for suf in ("", "_tgt")]
        ax = axes(
            rect, title, "deg", lim(*(sig[k] for k in keys)),
            hlines=[(b, bound_kw) for b in np.unique(np.round(np.concatenate(bl), 6))],
            xlabels=False,
        )
        for s in SIDES:
            line(ax, f"{s}_{joint}", color=COLOR[s], lw=1.6, label=f"{s[0].upper()} meas")
            line(ax, f"{s}_{joint}_tgt", color=COLOR[s], lw=1.0, ls="--", label=f"{s[0].upper()} target")
        return ax

    # Right column.
    tx = fig.add_axes(px(1290, 4, 625, 262))
    tx.axis("off")
    stats_text = tx.text(0, 1, "", va="top", ha="left", family="monospace", fontsize=8.2, animated=True)
    animated.append((stats_text, "__stats__"))

    ax_knee = joint_plot(px(1340, 290, 565, 160), "knee", "Knee  (solid meas, dashed target, dotted = target clip)")
    ax_knee.legend(loc="upper left", ncol=4, fontsize=6.5, framealpha=0.7)
    joint_plot(px(1340, 478, 565, 160), "ankle_crank", "Ankle crank (motor)")
    joint_plot(px(1340, 666, 565, 160), "hip1", "Hip pitch (hip1)")
    ax_v = axes(
        px(1340, 854, 565, 172), "Forward speed (body frame)", "m/s",
        lim(sig["vx"], sig["cmd_vx"], sig["vy"]),
    )
    line(ax_v, "vx", color="k", lw=1.4, label="vx")
    line(ax_v, "vy", color="0.6", lw=1.0, label="vy")
    line(ax_v, "cmd_vx", color="g", lw=1.0, ls="--", label="cmd vx")
    ax_v.legend(loc="upper left", ncol=3, fontsize=6.5, framealpha=0.7)
    ax_v.set_xlabel("time relative to now (s)")

    # Bottom row.
    hmax = max(sig["left_height"].max(), sig["right_height"].max())
    ax_h = axes(px(55, 762, 380, 278), "Foot site height; bars = ground contact", "m", (-0.06, hmax * 1.1 + 0.01))
    for s, y in (("left", -0.02), ("right", -0.042)):
        line(ax_h, f"{s}_height", color=COLOR[s], lw=1.5)
        line(ax_h, f"{s}_contact_bar", color=COLOR[s], lw=6, solid_capstyle="butt")
        ax_h.text(xs[0] + 0.05, y, s[0].upper(), va="center", fontsize=7, color=COLOR[s])
    ax_h.set_xlabel("time relative to now (s)")

    ax_s = axes(
        px(495, 762, 380, 278), "Sole tilt vs start (solid pitch +toe-up, dashed roll)", "deg",
        lim(*(sig[f"{s}_sole_{a}"] for s in SIDES for a in ("pitch", "roll"))), hlines=[(0, {"color": "0.3", "lw": 0.8})],
    )
    for s in SIDES:
        line(ax_s, f"{s}_sole_pitch", color=COLOR[s], lw=1.5)
        line(ax_s, f"{s}_sole_roll", color=COLOR[s], lw=1.0, ls="--")
    ax_s.set_xlabel("time relative to now (s)")

    ax_t = axes(
        px(935, 762, 335, 278), "|torque|/peak: knee -, ankle --, hip :", "ratio",
        (0, 1.05), hlines=[(1.0, {"color": "r", "lw": 0.8, "ls": ":"})],
    )
    for s in SIDES:
        line(ax_t, f"{s}_knee_tau", color=COLOR[s], lw=1.4)
        line(ax_t, f"{s}_ankle_crank_tau", color=COLOR[s], lw=1.0, ls="--")
        line(ax_t, f"{s}_hip1_tau", color=COLOR[s], lw=1.0, ls=":")
    ax_t.set_xlabel("time relative to now (s)")


    for s in SIDES:
        bar = np.where(sig[f"{s}_contact"], -0.02 if s == "left" else -0.042, np.nan)
        sig[f"{s}_contact_bar"] = bar

    canvas = fig.canvas
    canvas.draw()
    background = canvas.copy_from_bbox(fig.bbox)

    with media.VideoWriter(str(output), shape=(H, W), fps=fps) as video:
        for i in range(n):
            j0 = max(0, i - nw + 1)
            k = i - j0 + 1
            canvas.restore_region(background)
            for art, key in animated:
                if key == "__stats__":
                    art.set_text(label + "\n" + stats_block(sig, events, j0, i, dt))
                else:
                    art.set_data(xs[-k:], sig[key][j0 : i + 1])
                art.axes.draw_artist(art)
            img = np.asarray(canvas.buffer_rgba())[:, :, :3].copy()
            img[:VH, :VW] = frames[i]
            video.add_image(img)
            if (i + 1) % 100 == 0:
                print(f"composed {i + 1}/{n}", flush=True)
    plt.close(fig)


def stats_block(sig, events, j0, i, dt):
    w = window_stats(sig, events, j0, i, dt)
    L, R = w["left"], w["right"]

    def row(name, key, fmt="{:7.1f}"):
        f = lambda v: "    n/a" if v is None or not np.isfinite(v) else fmt.format(v)  # noqa: E731
        return f"{name:<24}{f(L[key])}  {f(R[key])}"

    lines = [
        f"t {sig['t'][i]:5.2f}s   cmd vx {sig['cmd_vx'][i]:.2f}   vx now {sig['vx'][i]:+.2f}  win {w['vx_mean']:+.2f} m/s",
        f"cadence {w['cadence_steps_per_s']:4.2f} steps/s   double supp {w['double_support_pct']:4.0f}%   flight {w['flight_pct']:3.0f}%",
        f"{'(rolling window)':<24}{'LEFT':>7}  {'RIGHT':>7}",
        row("step length cm", "step_len_cm"),
        row("step width cm", "step_width_cm"),
        row("swing time s", "swing_s", "{:7.2f}"),
        row("stance %", "stance_pct", "{:7.0f}"),
        row("knee min deg", "knee_deg_min"),
        row("knee max deg", "knee_deg_max"),
        row("ankle crank range deg", "ankle_crank_deg_range"),
        row("hip pitch range deg", "hip_pitch_deg_range"),
        row("knee tgt @clip %", "knee_tgt_at_bound_pct", "{:7.0f}"),
        row("ankle tgt @clip %", "ankle_tgt_at_bound_pct", "{:7.0f}"),
        row("stance sole pitch deg", "stance_sole_pitch_deg"),
        row("max foot height cm", "max_foot_height_cm"),
        row("peak knee |tau|/peak", "peak_knee_tau_ratio", "{:7.2f}"),
    ]
    return "\n".join(lines)


# --------------------------------------------------------------------- main
def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--checkpoint", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True, help="output .mp4")
    p.add_argument("--seconds", type=float, default=20.0)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--window", type=float, default=3.0, help="rolling plot window, s")
    p.add_argument("--settle", type=float, default=2.0, help="ignore first s in summary")
    args = p.parse_args()

    tel, meta, frames = rollout(args.checkpoint, args.seconds, args.seed)
    sig, events, bounds = derive(tel, meta)
    dt = meta["dt"]
    label = f"{args.checkpoint.parent.name}/{args.checkpoint.name}   seed {args.seed}"
    compose(frames, sig, events, bounds, meta, args.output, args.window, label)

    n = len(frames)
    summary = window_stats(sig, events, min(int(args.settle / dt), n - 1), n - 1, dt)
    report = {
        "checkpoint": str(args.checkpoint),
        "video": args.output.name,
        "seed": args.seed,
        "duration_s": n * dt,
        "fps": round(1 / dt),
        "resolution": [W, H],
        "episode_resets": int(tel["done"].sum()),
        "mean_reward": float(tel["reward"].mean()),
        "min_root_height_m": float(tel["root_pos"][:, 2].min()),
        f"steady_state_after_{args.settle:g}s": summary,
        "telemetry": args.output.with_name(args.output.stem + "_telemetry.npz").name,
    }
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2, default=float) + "\n")
    np.savez_compressed(
        args.output.with_name(args.output.stem + "_telemetry.npz"),
        **tel,
        **{f"meta_{k}": np.array(v) for k, v in meta.items()},
    )
    print(json.dumps(report, default=float), flush=True)


if __name__ == "__main__":
    main()
