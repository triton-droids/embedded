#!/usr/bin/env python3
"""Fisheye calibration for the ELP camera -> stella_vslam config values.

Runs natively on the Mac (Docker Desktop cannot reach USB cameras). Needs:
    pip install opencv-python numpy

1) Capture checkerboard images (SPACE = save, Q = quit). Aim for 30-50 images,
   board tilted and moved into every part of the image, especially the edges:
       python3 calibrate_fisheye.py capture --camera 0 --out calib_imgs
2) Calibrate and print the YAML block to paste into config/elp_fisheye.yaml:
       python3 calibrate_fisheye.py calibrate --images calib_imgs

--cols/--rows are INNER corners of the board (a 10x7-square board has 9x6 inner corners).
Use the same resolution the robot uses (default 640x480). Don't touch the lens focus afterwards.
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np


def capture(args):
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    os.makedirs(args.out, exist_ok=True)
    n = len(glob.glob(os.path.join(args.out, "*.png")))
    while True:
        ok, frame = cap.read()
        if not ok:
            sys.exit("could not read from camera %s" % args.camera)
        h, w = frame.shape[:2]
        if (w, h) != (args.width, args.height):
            sys.exit("camera gave %dx%d, wanted %dx%d" % (w, h, args.width, args.height))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (args.cols, args.rows), cv2.CALIB_CB_FAST_CHECK)
        view = frame.copy()
        cv2.drawChessboardCorners(view, (args.cols, args.rows), corners, found)
        cv2.putText(view, "saved: %d  SPACE=save Q=quit" % n, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("capture", view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" ") and found:
            cv2.imwrite(os.path.join(args.out, "img_%03d.png" % n), frame)
            n += 1
    cap.release()
    cv2.destroyAllWindows()


def calibrate(args):
    paths = sorted(glob.glob(os.path.join(args.images, "*.png")) + glob.glob(os.path.join(args.images, "*.jpg")))
    if not paths:
        sys.exit("no images in " + args.images)
    objp = np.zeros((1, args.cols * args.rows, 3), np.float64)
    objp[0, :, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    obj_pts, img_pts, size = [], [], None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    for p in paths:
        gray = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if size is None:
            size = gray.shape[::-1]
        elif gray.shape[::-1] != size:
            sys.exit("%s has a different resolution; use one resolution only" % p)
        found, corners = cv2.findChessboardCorners(gray, (args.cols, args.rows), None)
        if not found:
            print("skip (no board):", p)
            continue
        corners = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), crit)
        obj_pts.append(objp)
        img_pts.append(corners.reshape(1, -1, 2).astype(np.float64))
    if len(obj_pts) < 10:
        sys.exit("only %d usable images; need at least 10 (30-50 recommended)" % len(obj_pts))

    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND | cv2.fisheye.CALIB_FIX_SKEW
    rms, K, D, _, _ = cv2.fisheye.calibrate(
        obj_pts, img_pts, size, K, D, flags=flags,
        criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100, 1e-6))

    print("images used: %d / %d" % (len(obj_pts), len(paths)))
    print("RMS reprojection error: %.3f px %s" % (rms, "(OK)" if rms < 0.5 else "(target is < 0.5 px)"))
    print("\n# paste into the Camera: section of config/elp_fisheye.yaml")
    print("  fx: %.6f\n  fy: %.6f\n  cx: %.6f\n  cy: %.6f" % (K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
    for i in range(4):
        print("  k%d: %.10e" % (i + 1, D[i, 0]))
    print("  cols: %d\n  rows: %d" % size)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cols", type=int, default=9, help="inner corners per row")
    ap.add_argument("--rows", type=int, default=6, help="inner corners per column")
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture")
    c.add_argument("--camera", type=int, default=0)
    c.add_argument("--width", type=int, default=640)
    c.add_argument("--height", type=int, default=480)
    c.add_argument("--out", default="calib_imgs")
    k = sub.add_parser("calibrate")
    k.add_argument("--images", default="calib_imgs")
    k.add_argument("--square", type=float, default=1.0, help="square size (any unit; does not affect intrinsics)")
    args = ap.parse_args()
    capture(args) if args.cmd == "capture" else calibrate(args)


if __name__ == "__main__":
    main()
