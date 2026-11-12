"""Render a recorded VIO trial: chase view + stereo-left inset + top-down truth/estimate map.

  MUJOCO_GL=egl python3 render_route.py REC.npz ERRORS.csv OUT.mp4 [TITLE]

Ground truth and the ball come from the recorded qpos; the estimate from
localization_eval's CSV (estimate x, y; estimate yaw = truth yaw + yaw error).
"""
import math
import subprocess
import sys

import cv2
import mujoco
import numpy as np

W, H = 1280, 720
VIEW_W, VIEW_H = 800, 720
MAP_W = W - VIEW_W
SPEEDUP = 2  # 15 Hz recording played at 30 fps.


def main():
    rec_path, csv_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
    title = sys.argv[4] if len(sys.argv) > 4 else 'IMU + cuVSLAM stereo VO'
    rec = np.load(rec_path)
    model = mujoco.MjModel.from_binary_path(rec_path.replace('.npz', '.mjb'))
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, VIEW_W)
    model.vis.global_.offheight = max(model.vis.global_.offheight, VIEW_H)
    data = mujoco.MjData(model)
    view = mujoco.Renderer(model, VIEW_H, VIEW_W)
    inset = mujoco.Renderer(model, 240, 320)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.elevation, cam.azimuth = 2.6, -28.0, 200.0
    base = model.body('floating_base').id

    stamp, qpos = rec['stamp'], rec['qpos']
    bq = model.jnt_qposadr[model.body('floating_base').jntadr[0]]
    w, x, y, z = (qpos[:, bq + 3 + k] for k in range(4))
    truth = np.column_stack([qpos[:, bq], qpos[:, bq + 1],
                             np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))])
    kq = model.jnt_qposadr[model.joint('chase_ball_free').id]
    balls = qpos[:, kq:kq + 2]
    rows = np.loadtxt(csv_path, delimiter=',', skiprows=1)
    est = np.full((len(stamp), 3), np.nan)
    ts = stamp * 1e-9
    j = np.clip(np.searchsorted(rows[:, 0], ts), 1, len(rows) - 1)
    j = np.where(np.abs(rows[j - 1, 0] - ts) < np.abs(rows[j, 0] - ts), j - 1, j)
    ok = np.abs(rows[j, 0] - ts) < 0.05
    est[ok, 0], est[ok, 1] = rows[j[ok], 1], rows[j[ok], 2]
    est[ok, 2] = truth[ok, 2] + rows[j[ok], 6]
    valid = ~np.isnan(est[:, 0])
    first = int(np.argmax(valid))
    xs = np.concatenate([truth[:, 0], est[valid, 0]])
    ys = np.concatenate([truth[:, 1], est[valid, 1]])
    lo = np.array([xs.min(), ys.min()]) - 0.6
    hi = np.array([xs.max(), ys.max()]) + 0.6
    scale = min((MAP_W - 60) / (hi[0] - lo[0]), (H - 260) / (hi[1] - lo[1]))
    origin = np.array([30, 200])

    def px(x, y):
        return (int(origin[0] + (x - lo[0]) * scale), int(H - 60 - (y - lo[1]) * scale))

    ff = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
                           '-s', f'{W}x{H}', '-r', '30', '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                           '-crf', '23', '-preset', 'veryfast', out_path], stdin=subprocess.PIPE)
    path_len = 0.0
    for i in range(first, len(stamp)):
        data.qpos[:] = rec['qpos'][i]
        if model.nmocap:
            data.mocap_pos[:] = rec['mocap_pos'][i]
            data.mocap_quat[:] = rec['mocap_quat'][i]
        mujoco.mj_kinematics(model, data)
        mujoco.mj_camlight(model, data)
        cam.lookat[:] = data.xpos[base]
        cam.lookat[2] = 0.45
        view.update_scene(data, camera=cam)
        frame = np.zeros((H, W, 3), np.uint8)
        frame[:, :VIEW_W] = cv2.cvtColor(view.render(), cv2.COLOR_RGB2BGR)
        inset.update_scene(data, camera='stereo_left')
        small = cv2.cvtColor(cv2.cvtColor(inset.render(), cv2.COLOR_RGB2GRAY), cv2.COLOR_GRAY2BGR)
        frame[H - 250:H - 10, 10:330] = small
        cv2.rectangle(frame, (9, H - 251), (330, H - 9), (255, 255, 255), 1)
        cv2.putText(frame, 'stereo left (VO input)', (16, H - 232), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, title, (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)

        # Map panel.
        m = frame[:, VIEW_W:]
        m[:] = (32, 32, 32)
        for gx in range(math.floor(lo[0]), math.ceil(hi[0]) + 1):
            a, b = px(gx, lo[1]), px(gx, hi[1])
            cv2.line(m, (a[0], a[1]), (b[0], b[1]), (70, 70, 70), 1)
        for gy in range(math.floor(lo[1]), math.ceil(hi[1]) + 1):
            a, b = px(lo[0], gy), px(hi[0], gy)
            cv2.line(m, (a[0], a[1]), (b[0], b[1]), (70, 70, 70), 1)
        t_pts = np.array([px(*p) for p in truth[first:i + 1, :2]], np.int32)
        e_idx = [k for k in range(first, i + 1) if valid[k]]
        e_pts = np.array([px(*est[k, :2]) for k in e_idx], np.int32)
        if len(t_pts) > 1:
            cv2.polylines(m, [t_pts], False, (255, 255, 255), 2, cv2.LINE_AA)
        if len(e_pts) > 1:
            cv2.polylines(m, [e_pts], False, (0, 160, 255), 2, cv2.LINE_AA)
        for pose, color in ((truth[i], (255, 255, 255)), (est[i], (0, 160, 255))):
            if np.isnan(pose[0]):
                continue
            c = px(pose[0], pose[1])
            # Base forward axis is +y: heading arrow at yaw + 90 deg.
            h = pose[2] + math.pi / 2
            tip = (int(c[0] + 22 * math.cos(h)), int(c[1] - 22 * math.sin(h)))
            cv2.circle(m, c, 6, color, -1, cv2.LINE_AA)
            cv2.arrowedLine(m, c, tip, color, 2, cv2.LINE_AA, tipLength=0.35)
        ball = balls[i]
        if not np.isnan(ball[0]) and lo[0] <= ball[0] <= hi[0] and lo[1] <= ball[1] <= hi[1]:
            cv2.circle(m, px(*ball), 5, (60, 220, 60), -1, cv2.LINE_AA)
        if i - first >= 15 and (i - first) % 15 == 0:  # 1 Hz: leave out the walking sway.
            path_len += float(np.linalg.norm(truth[i, :2] - truth[i - 15, :2]))
        err = float(np.linalg.norm(est[i, :2] - truth[i, :2])) if valid[i] else math.nan
        yerr = math.degrees(math.atan2(math.sin(est[i, 2] - truth[i, 2]), math.cos(est[i, 2] - truth[i, 2])))
        lines = [('Top-down (m)', (255, 255, 255)),
                 ('white: ground truth', (255, 255, 255)),
                 ('orange: /localization/odom', (0, 160, 255)),
                 ('green: ball', (60, 220, 60)),
                 (f't = {(stamp[i] - stamp[first]) * 1e-9:5.1f} s   ({SPEEDUP}x speed)', (220, 220, 220)),
                 (f'path {path_len:5.2f} m   pos err {err * 100:5.1f} cm', (220, 220, 220)),
                 (f'yaw err {yerr:+5.2f} deg  (IMU +/-3 deg, 60 s)', (220, 220, 220))]
        for k, (text, color) in enumerate(lines):
            cv2.putText(m, text, (20, 32 + 24 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
        ff.stdin.write(frame.tobytes())
    ff.stdin.close()
    ff.wait()
    print(f'wrote {len(stamp) - first} frames to {out_path}')


if __name__ == '__main__':
    main()
