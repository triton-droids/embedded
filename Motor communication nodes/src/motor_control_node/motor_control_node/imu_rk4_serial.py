#!/usr/bin/env python3
import argparse
import math
import time
import serial

G = 9.80665
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

def wrap_pi(a):
    # wrap to [-pi, pi)
    while a >= math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def rot_matrix_from_rpy(roll, pitch, yaw):
    # R = Rz(yaw)*Ry(pitch)*Rx(roll)  (body -> world)
    cr = math.cos(roll);  sr = math.sin(roll)
    cp = math.cos(pitch); sp = math.sin(pitch)
    cy = math.cos(yaw);   sy = math.sin(yaw)

    R = [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ]
    return R

def mat_vec(R, v):
    return [
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    ]

def parse_line(line: str):
    # returns dict or None
    parts = [p.strip() for p in line.split(",")]
    if len(parts) < 10:
        return None
    try:
        t_ms = float(parts[0])
        ax_g = float(parts[1]); ay_g = float(parts[2]); az_g = float(parts[3])
        gx_dps = float(parts[4]); gy_dps = float(parts[5]); gz_dps = float(parts[6])
        roll_deg = float(parts[7]); pitch_deg = float(parts[8])
        # tempC = float(parts[9])  
        return {
            "t_ms": t_ms,
            "a_g": (ax_g, ay_g, az_g),
            "g_dps": (gx_dps, gy_dps, gz_dps),
            "rp_acc_deg": (roll_deg, pitch_deg),
        }
    except Exception:
        return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--alpha", type=float, default=0.98, help="complementary filter alpha for roll/pitch")
    ap.add_argument("--calib_sec", type=float, default=2.0, help="seconds for gyro bias calibration at start (keep still)")
    ap.add_argument("--zupt_acc_g", type=float, default=0.03, help="| |a|-1g | < thresh => still candidate")
    ap.add_argument("--zupt_gyro_dps", type=float, default=2.0, help="gyro magnitude < thresh => still candidate")
    ap.add_argument("--zupt_hold", type=int, default=8, help="consecutive samples to trigger ZUPT")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1.0)

    # state
    last_t = None

    # orientation (rad)
    roll = 0.0
    pitch = 0.0
    yaw_cont = 0.0

    # gyro bias (rad/s)
    bgx = bgy = bgz = 0.0

    # vel/pos in world (m/s, m)
    vx = vy = vz = 0.0
    px = py = pz = 0.0

    # prev linear accel for trapezoid
    prev_aw = None

    # ZUPT counter
    still_count = 0

    # --- gyro bias calibration (keep IMU still) ---
    print(f"# Calibrating gyro bias for {args.calib_sec}s... keep IMU still")
    t_start = time.time()
    n = 0
    s_gx = s_gy = s_gz = 0.0
    while time.time() - t_start < args.calib_sec:
        line = ser.readline().decode(errors="ignore").strip()
        d = parse_line(line)
        if not d:
            continue
        gx_dps, gy_dps, gz_dps = d["g_dps"]
        s_gx += gx_dps * DEG2RAD
        s_gy += gy_dps * DEG2RAD
        s_gz += gz_dps * DEG2RAD
        n += 1
    if n > 0:
        bgx = s_gx / n
        bgy = s_gy / n
        bgz = s_gz / n
    print(f"# gyro_bias(rad/s): bgx={bgx:.6f}, bgy={bgy:.6f}, bgz={bgz:.6f}")
    print("# columns: t(s), yaw_now(deg), yaw_cont(deg), roll(deg), pitch(deg), "
          "vx,vy,vz(m/s), px,py,pz(m), axw,ayw,azw(m/s^2)")

    # main loop
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        d = parse_line(line)
        if not d:
            continue

        t = d["t_ms"] * 1e-3
        if last_t is None:
            last_t = t
            prev_aw = (0.0, 0.0, 0.0)
            continue
        dt = t - last_t
        if dt <= 0 or dt > 0.5:
            # 时间戳异常：跳过
            last_t = t
            prev_aw = (0.0, 0.0, 0.0)
            continue
        last_t = t

        # sensors
        ax_g, ay_g, az_g = d["a_g"]
        gx_dps, gy_dps, gz_dps = d["g_dps"]
        roll_acc_deg, pitch_acc_deg = d["rp_acc_deg"]

        # convert
        ax = ax_g * G
        ay = ay_g * G
        az = az_g * G

        gx = gx_dps * DEG2RAD - bgx
        gy = gy_dps * DEG2RAD - bgy
        gz = gz_dps * DEG2RAD - bgz

        # --- complementary filter for roll/pitch ---
        # gyro integration
        roll_gyro = roll + gx * dt
        pitch_gyro = pitch + gy * dt

        # accel roll/pitch (use your Arduino computed values, convert to rad)
        roll_acc = roll_acc_deg * DEG2RAD
        pitch_acc = pitch_acc_deg * DEG2RAD

        alpha = args.alpha
        roll = alpha * roll_gyro + (1 - alpha) * roll_acc
        pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc

        # --- yaw integration (no absolute reference, will drift) ---
        yaw_cont += gz * dt
        yaw_now = wrap_pi(yaw_cont)

        # --- linear acceleration in world ---
        # body accel vector (m/s^2)
        ab = [ax, ay, az]

        # rotate to world
        R = rot_matrix_from_rpy(roll, pitch, yaw_cont)
        aw = mat_vec(R, ab)

        # subtract gravity (world z up, gravity points -z)
        # If you define world z up: gravity vector = [0,0,-g], and accelerometer measures +g when stationary upward?
        # With our rotation convention, stationary should give aw ≈ [0,0, +g], so subtract +g on z.
        a_lin_w = [aw[0], aw[1], aw[2] - G]

        # --- ZUPT (simple still detection) ---
        a_mag_g = math.sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g)
        gyro_mag = math.sqrt(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps)
        if abs(a_mag_g - 1.0) < args.zupt_acc_g and gyro_mag < args.zupt_gyro_dps:
            still_count += 1
        else:
            still_count = 0

        # --- integrate v and p (trapezoid) ---
        # v += 0.5*(a_k + a_{k-1})*dt
        axw, ayw, azw = a_lin_w
        paxw, payw, pazw = prev_aw
        vx += 0.5 * (axw + paxw) * dt
        vy += 0.5 * (ayw + payw) * dt
        vz += 0.5 * (azw + pazw) * dt

        # ZUPT: if still for some samples, clamp velocities to 0
        if still_count >= args.zupt_hold:
            vx = vy = vz = 0.0

        px += vx * dt
        py += vy * dt
        pz += vz * dt

        prev_aw = (axw, ayw, azw)

        print(f"{t:10.3f}, {yaw_now*RAD2DEG:8.2f}, {yaw_cont*RAD2DEG:9.2f}, "
              f"{roll*RAD2DEG:7.2f}, {pitch*RAD2DEG:7.2f}, "
              f"{vx:8.3f},{vy:8.3f},{vz:8.3f}, "
              f"{px:8.3f},{py:8.3f},{pz:8.3f}, "
              f"{axw:7.3f},{ayw:7.3f},{azw:7.3f}")

if __name__ == "__main__":
    main()
