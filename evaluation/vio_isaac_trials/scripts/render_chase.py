"""Render recorded chases: chase view + head camera with Isaac detections + top-down map.

  MUJOCO_GL=egl python3 render_chase.py OUT.mp4 REC1.npz "TITLE 1" [REC2.npz "TITLE 2" ...]
Each recording becomes one clip (2x speed); clips are joined.
"""
import json
import math
import subprocess
import sys

import cv2
import mujoco
import numpy as np

W, H = 1280, 720
VIEW_W = 760
CAM_W, CAM_H = 520, 294          # Head camera panel (424x240 scaled).
SRC_W, SRC_H = 424, 240
NAMES = ['ball', 'goalpost', 'robot', 'L-int', 'T-int', 'X-int', 'penalty']
COLORS = [(60, 220, 60), (255, 255, 255), (60, 60, 230), (230, 200, 0), (230, 120, 0), (200, 0, 200), (0, 200, 255)]
# Static scene objects (humanoid_ball_chase/scene.py, scene:=hard).
ROBOTS = [(-0.58, 2.18), (-0.24, 2.14), (0.1, 2.2), (1.2, 3.0), (-1.5, 3.4)]
CAMERA_FROM_OPTICAL = np.diag([1.0, -1.0, -1.0])  # MuJoCo camera axes -> optical (x right, y down, z fwd).


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def network_to_image(cx, cy, bw, bh):
    scale = min(640 / SRC_W, 640 / SRC_H)
    px, py = (640 - SRC_W * scale) / 2, (640 - SRC_H * scale) / 2
    return (cx - px) / scale, (cy - py) / scale, bw / scale, bh / scale


def latest(items, stamp):
    """Last item with stamp <= stamp (items sorted by stamp), or None."""
    best = None
    for item in items:
        if item[0] > stamp:
            break
        best = item
    return best


def clip(ff, path, title):
    rec = np.load(path)
    side = json.load(open(path.replace('.npz', '.json')))
    dets = sorted(side['detections'], key=lambda d: d[0])
    tgts = sorted(side['targets'], key=lambda t: t[0])
    poses = sorted(side.get('poses', []), key=lambda t: t[0])     # Robot's own pose (/localization/odom).
    points = sorted(side.get('points', []), key=lambda t: t[0])   # Chased target, camera optical frame.
    model = mujoco.MjModel.from_binary_path(path.replace('.npz', '.mjb'))
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, VIEW_W)
    model.vis.global_.offheight = max(model.vis.global_.offheight, H)
    data = mujoco.MjData(model)
    view = mujoco.Renderer(model, H, VIEW_W)
    head = mujoco.Renderer(model, SRC_H, SRC_W)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.elevation, cam.azimuth = 2.6, -30.0, 90.0  # Behind the robot (it walks towards +y).
    base = model.body('floating_base').id
    bq = model.jnt_qposadr[model.body('floating_base').jntadr[0]]
    kq = model.jnt_qposadr[model.joint('chase_ball_free').id]
    qp = rec['qpos'][int(np.nonzero(np.diff(rec['stamp']) > 2e9)[0][-1]) + 1 if (np.diff(rec['stamp']) > 2e9).any() else 0:]
    xs = np.concatenate([qp[:, bq], qp[:, kq], [r[0] for r in ROBOTS]] + ([np.array(poses)[:, 1]] if poses else []))
    ys = np.concatenate([qp[:, bq + 1], qp[:, kq + 1], [r[1] for r in ROBOTS]]
                        + ([np.array(poses)[:, 2]] if poses else []))
    LO = np.array([xs.min(), ys.min()]) - 0.4
    HI = np.array([xs.max(), ys.max()]) + 0.4
    map_h = H - CAM_H - 20
    scale = min((W - VIEW_W - 40) / (HI[0] - LO[0]), (map_h - 40) / (HI[1] - LO[1]))

    def px(x, y):
        return (int(VIEW_W + 20 + (x - LO[0]) * scale), int(CAM_H + 20 + map_h - 20 - (y - LO[1]) * scale))

    stamp, qpos = rec['stamp'], rec['qpos']
    # A state file left by an earlier run can supply the first frames: start after the last >2 s gap.
    gaps = np.nonzero(np.diff(stamp) > 2e9)[0]
    first = int(gaps[-1]) + 1 if len(gaps) else 0
    stamp, qpos = stamp[first:], qpos[first:]
    mocap_pos, mocap_quat = rec['mocap_pos'][first:], rec['mocap_quat'][first:]
    path_pts, est_pts, belief_trail = [], [], []
    head_cam = model.camera('head_camera').id
    for i in range(len(stamp)):
        data.qpos[:] = qpos[i]
        if model.nmocap:
            data.mocap_pos[:], data.mocap_quat[:] = mocap_pos[i], mocap_quat[i]
        mujoco.mj_kinematics(model, data)
        mujoco.mj_camlight(model, data)
        cam.lookat[:] = data.xpos[base]
        cam.lookat[2] = 0.45
        view.update_scene(data, camera=cam)
        frame = np.zeros((H, W, 3), np.uint8)
        frame[:, :VIEW_W] = cv2.cvtColor(view.render(), cv2.COLOR_RGB2BGR)
        cv2.putText(frame, title, (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, f't = {(stamp[i] - stamp[0]) * 1e-9:5.1f} s  (2x)', (16, 64),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1, cv2.LINE_AA)
        # Head camera with the detections of this (or the latest earlier) image.
        head.update_scene(data, camera='head_camera')
        img = cv2.cvtColor(head.render(), cv2.COLOR_RGB2BGR)
        det = latest(dets, stamp[i])
        tgt = latest(tgts, stamp[i])
        if det is not None and stamp[i] - det[0] < 0.4e9:
            for cls, score, cx, cy, bw, bh in det[1]:
                u, v, w, h = network_to_image(cx, cy, bw, bh)
                c = COLORS[cls % len(COLORS)]
                cv2.rectangle(img, (int(u - w / 2), int(v - h / 2)), (int(u + w / 2), int(v + h / 2)), c, 1)
                cv2.putText(img, f'{NAMES[cls] if cls < len(NAMES) else cls} {score:.2f}',
                            (int(u - w / 2), max(9, int(v - h / 2) - 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.3, c, 1,
                            cv2.LINE_AA)
        if tgt is not None and stamp[i] - tgt[0] < 0.4e9:
            cv2.drawMarker(img, (int(tgt[1]), int(tgt[2])), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
        panel = frame[0:CAM_H, VIEW_W:]
        panel[:] = cv2.resize(img, (W - VIEW_W, CAM_H))
        cv2.putText(panel, 'head camera + Isaac YOLO (yellow cross: chased)', (8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        # Top-down map.
        m = frame[CAM_H:, VIEW_W:]
        m[:] = (32, 32, 32)
        for gx in range(math.ceil(LO[0]), math.floor(HI[0]) + 1):
            cv2.line(frame, px(gx, LO[1]), px(gx, HI[1]), (60, 60, 60), 1)
        for gy in range(math.ceil(LO[1]), math.floor(HI[1]) + 1):
            cv2.line(frame, px(LO[0], gy), px(HI[0], gy), (60, 60, 60), 1)
        for x, y in ROBOTS:
            cv2.rectangle(frame, (px(x, y)[0] - 6, px(x, y)[1] - 6), (px(x, y)[0] + 6, px(x, y)[1] + 6),
                          (60, 60, 230), -1)
        # Ground truth (simulation): white path/arrow, green ball.
        bx, by = qpos[i, kq], qpos[i, kq + 1]
        cv2.circle(frame, px(bx, by), 6, (60, 220, 60), -1, cv2.LINE_AA)
        path_pts.append(px(qpos[i, bq], qpos[i, bq + 1]))
        if len(path_pts) > 1:
            cv2.polylines(frame, [np.array(path_pts, np.int32)], False, (255, 255, 255), 2, cv2.LINE_AA)
        w_, x_, y_, z_ = qpos[i, bq + 3:bq + 7]
        yaw = math.atan2(2 * (w_ * z_ + x_ * y_), 1 - 2 * (y_ * y_ + z_ * z_)) + math.pi / 2  # Forward = +y.
        c = path_pts[-1]
        cv2.arrowedLine(frame, c, (int(c[0] + 25 * math.cos(yaw)), int(c[1] - 25 * math.sin(yaw))),
                        (255, 255, 255), 2, cv2.LINE_AA, tipLength=0.35)
        # What the robot believes: orange own path/arrow, yellow target from its own pose.
        lines = []
        pose = latest(poses, stamp[i])
        if pose is not None:
            est_pts.append(px(pose[1], pose[2]))
            cv2.polylines(frame, [np.array(est_pts, np.int32)], False, (0, 150, 255), 2, cv2.LINE_AA)
            r_est = quat_to_matrix(pose[4:8])
            fwd = r_est[:2, 1]
            e = est_pts[-1]
            cv2.arrowedLine(frame, e, (int(e[0] + 25 * fwd[0]), int(e[1] - 25 * fwd[1])),
                            (0, 150, 255), 2, cv2.LINE_AA, tipLength=0.35)
            err = math.hypot(pose[1] - qpos[i, bq], pose[2] - qpos[i, bq + 1])
            lines.append((f'own position error {err * 100:5.1f} cm', (0, 150, 255)))
            pt = latest(points, stamp[i])
            if pt is not None and stamp[i] - pt[0] < 0.4e9:
                # Camera pose relative to the body from head kinematics, then the robot's own body pose.
                r_wb = data.xmat[base].reshape(3, 3)
                r_wc = data.cam_xmat[head_cam].reshape(3, 3) @ CAMERA_FROM_OPTICAL
                p_bc = r_wb.T @ (data.cam_xpos[head_cam] - data.xpos[base])
                p_body = p_bc + r_wb.T @ r_wc @ np.array(pt[1:4])
                belief = np.array(pose[1:4]) + r_est @ p_body
                belief_trail.append(px(belief[0], belief[1]))
                cv2.drawMarker(frame, belief_trail[-1], (0, 255, 255), cv2.MARKER_TILTED_CROSS, 16, 3)
                lines.append((f'target belief vs true ball {math.hypot(belief[0] - bx, belief[1] - by) * 100:5.1f} cm',
                              (0, 255, 255)))
        for p_ in belief_trail[-40:]:
            cv2.circle(frame, p_, 2, (0, 200, 200), -1)
        cv2.putText(frame, 'top-down  white: true path  orange: robot own estimate (IMU + VO)',
                    (VIEW_W + 10, CAM_H + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(frame, 'green: true ball  yellow x: where the robot thinks its target is',
                    (VIEW_W + 10, CAM_H + 42), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1, cv2.LINE_AA)
        for k_, (text, color) in enumerate(lines):
            cv2.putText(frame, text, (VIEW_W + 10, H - 40 + 20 * k_), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
                        cv2.LINE_AA)
        ff.stdin.write(frame.tobytes())
    return len(stamp)


def main():
    out = sys.argv[1]
    pairs = list(zip(sys.argv[2::2], sys.argv[3::2]))
    ff = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
                           '-s', f'{W}x{H}', '-r', '30', '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                           '-crf', '23', '-preset', 'veryfast', out], stdin=subprocess.PIPE)
    total = sum(clip(ff, path, title) for path, title in pairs)
    ff.stdin.close()
    ff.wait()
    print(f'wrote {total} frames to {out}')


if __name__ == '__main__':
    main()
