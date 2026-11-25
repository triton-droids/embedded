"""Render a record_scenario.py recording: tracking view + mode-manager state panel + mode timeline.

  MUJOCO_GL=egl python3 render_scenario.py REC.npz OUT.mp4
One frame per 1/30 s of simulated time, so the video plays at 1x simulated time.
Aid of Magic: while the TEST-ONLY simulator assist is on (walking_controller recovery_sim_assist, /sim/assist:
a horizontal damping force and an uprighting torque on the base, not the policies' own work) a banner says so
and the timeline marks the span in gold. The spans come from the recording ('walks' entries with a 5th value,
sim_assist) or, for older recordings, an 'aids' list [[start_ns, end_ns], ...] added to the JSON by hand.
Needs mujoco, numpy, Pillow and ffmpeg.
"""
import json
import subprocess
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H = 1280, 720
FONTS = ['/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
         '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf']
MODE_COLORS = {
    'boot': (120, 120, 120), 'disabled': (90, 90, 140), 'enabling': (70, 120, 200), 'homing': (70, 120, 200),
    'gantry': (60, 150, 200), 'stand': (40, 170, 90), 'walk': (30, 200, 60), 'kick': (200, 200, 40),
    'squat': (150, 200, 40), 'falling': (240, 140, 20), 'fallen': (230, 90, 20), 'getup': (220, 60, 160),
    'damping': (200, 80, 80), 'fault': (220, 30, 30), 'estop': (255, 0, 0)}
PHASES = {'start': 'Start', 'ready': 'Ready', 'walk': 'Walk', 'fall': 'Fall', 'getup': 'Get up',
          'settle': 'Settle', 'walk_again': 'Walk again', 'stop': 'Stop'}
RECOVERY = {'walk': 'walking', 'rise': 'get-up: rise', 'straight': 'get-up: straighten'}
MAGIC = (255, 200, 40)
MAGIC_LINES = ('Simulator holds the torso: it damps sideways speed',
               'and twists the base upright (test only).',
               'What you see is not the policies\' own work.')
HELPING = {'rise': 'helping: get-up', 'straight': 'helping: balance', 'walk': 'helping: walking'}


def aid_spans(side, t0, t1):
    """[(start, end)] of the simulator assist, from the walk-status records or a hand-written 'aids' list."""
    if side.get('aids'):
        return [(max(a, t0), min(b if b else t1, t1)) for a, b, *_ in side['aids']]
    spans, start = [], None
    for w in side['walks']:
        on = len(w) > 4 and bool(w[4])
        if on and start is None:
            start = w[0]
        elif not on and start is not None:
            spans.append((max(start, t0), min(w[0], t1)))
            start = None
    if start is not None:
        spans.append((max(start, t0), t1))
    return [(a, b) for a, b in spans if b > a]


def latest(items, stamp):
    best = None
    for item in items:
        if item[0] > stamp:
            break
        best = item
    return best


def font(size):
    for path in FONTS:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def main():
    rec_path, out = sys.argv[1], sys.argv[2]
    rec = np.load(rec_path)
    side = json.load(open(rec_path.replace('.npz', '.json')))
    modes, walks, phases = side['modes'], side['walks'], side['phases']
    model = mujoco.MjModel.from_binary_path(rec_path.replace('.npz', '.mjb'))
    model.vis.global_.offwidth, model.vis.global_.offheight = W, H
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, H, W)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.elevation, cam.azimuth = 2.6, -12.0, 135.0
    stamps = rec['stamp']
    t0, t1 = stamps[0], stamps[-1]
    f_big, f_mid, f_small = font(54), font(28), font(20)
    aids = aid_spans(side, t0, t1)

    # Mode spans for the timeline.
    spans = []
    for i, (t, mode, *_rest) in enumerate(modes):
        end = modes[i + 1][0] if i + 1 < len(modes) else t1
        if min(end, t1) > max(t, t0):  # Modes before the first recorded frame are dropped.
            spans.append((max(t, t0), min(end, t1), mode))

    ff = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                           '-s', f'{W}x{H}', '-r', '30', '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                           '-crf', '20', out], stdin=subprocess.PIPE)
    lookat = None
    # One output frame per 1/30 s of simulated time, showing the latest recorded state (1x speed).
    times = np.arange(t0, t1 + 1, int(1e9 / 30))
    indices = np.searchsorted(stamps, times, side='right') - 1
    for k, stamp in zip(indices, times):
        data.qpos[:] = rec['qpos'][k]
        if model.nmocap:
            data.mocap_pos[:] = rec['mocap_pos'][k]
            data.mocap_quat[:] = rec['mocap_quat'][k]
        mujoco.mj_kinematics(model, data)
        target = np.array([data.qpos[0], data.qpos[1], 0.45])
        lookat = target if lookat is None else lookat + 0.08 * (target - lookat)  # Smooth follow.
        cam.lookat[:] = lookat
        renderer.update_scene(data, cam)
        img = Image.fromarray(renderer.render())
        draw = ImageDraw.Draw(img, 'RGBA')

        m = latest(modes, stamp)
        w = latest(walks, stamp)
        p = latest(phases, stamp)
        mode = m[1] if m else 'boot'
        color = MODE_COLORS.get(mode, (200, 200, 200))
        # State panel.
        draw.rounded_rectangle((20, 20, 520, 250), 14, fill=(0, 0, 0, 170))
        draw.text((40, 28), 'MODE MANAGER STATE', font=f_small, fill=(200, 200, 200))
        draw.text((40, 52), mode.upper(), font=f_big, fill=color)
        draw.text((40, 122), f'reason: {m[2] if m else "-"}'[:44], font=f_small, fill=(230, 230, 230))
        draw.text((40, 150), f'scenario: {PHASES.get(p[1], p[1]) if p else "-"}', font=f_mid, fill=(255, 255, 255))
        rec_mode = w[2] if w else None
        draw.text((40, 188), f'controller: {RECOVERY.get(rec_mode, rec_mode or "-")}',
                  font=f_small, fill=(230, 230, 230))
        draw.text((40, 214), f'get-up attempts {m[3] if m else 0}   sim t = {(stamp - t0) * 1e-9:5.1f} s',
                  font=f_small, fill=(230, 230, 230))
        # Timeline.
        x0, x1, y0, y1 = 20, W - 20, H - 70, H - 40
        draw.rectangle((x0 - 2, y0 - 26, x1 + 2, y1 + 22), fill=(0, 0, 0, 150))
        scale = (x1 - x0) / max(1, t1 - t0)
        for a, b, md in spans:
            draw.rectangle((x0 + (a - t0) * scale, y0, x0 + (b - t0) * scale, y1), fill=MODE_COLORS.get(md))
        right = -1e9   # Right edge of the previous label: crowded labels move right, never overlap.
        for t, name in phases:
            x = x0 + (max(t, t0) - t0) * scale
            draw.line((x, y0 - 4, x, y1 + 4), fill=(255, 255, 255), width=2)
            label = PHASES.get(name, name)
            lx = max(x + 3, right + 8)
            draw.text((lx, y0 - 26), label, font=f_small, fill=(255, 255, 255))
            right = lx + draw.textlength(label, font=f_small)
        right = -1e9
        for a, b, md in spans:
            lx = max(x0 + (a - t0) * scale + 4, right + 8)
            if lx + draw.textlength(md, font=f_small) <= x0 + (b - t0) * scale + 40:
                draw.text((lx, y1 + 0), md, font=f_small, fill=(255, 255, 255))
                right = lx + draw.textlength(md, font=f_small)
        for a, b in aids:   # Aid of Magic spans: a gold strip over the mode bar.
            draw.rectangle((x0 + (a - t0) * scale, y0, x0 + (b - t0) * scale, y0 + 6), fill=MAGIC)
        if any(a <= stamp < b for a, b in aids):
            bx0, by0, bx1 = W - 560, 20, W - 20
            draw.rounded_rectangle((bx0, by0, bx1, by0 + 200), 14, fill=(40, 20, 70, 200), outline=MAGIC, width=3)
            draw.text((bx0 + 20, by0 + 8), 'AID OF MAGIC', font=f_big, fill=MAGIC)
            for i, line in enumerate(MAGIC_LINES):
                draw.text((bx0 + 20, by0 + 72 + 22 * i), line, font=f_small, fill=(255, 240, 200))
            helping = HELPING.get(rec_mode)
            if helping:
                draw.text((bx0 + 20, by0 + 72 + 22 * len(MAGIC_LINES) + 6), helping, font=f_mid, fill=MAGIC)
        cx = x0 + (stamp - t0) * scale
        draw.rectangle((cx - 2, y0 - 6, cx + 2, y1 + 6), fill=(255, 255, 255))
        ff.stdin.write(np.asarray(img).tobytes())
    ff.stdin.close()
    ff.wait()
    print(f'{out}: {len(times)} frames, {(t1 - t0) * 1e-9:.1f} s')


if __name__ == '__main__':
    main()
