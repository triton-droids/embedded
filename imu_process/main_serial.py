import argparse
import math
import time

import serial

from fusion import QuatFusion, StillnessCfg, FusionCfg


def parse_line(line: str):
    parts = [p.strip() for p in line.split(",")]
    if len(parts) < 7:
        return None
    try:
        t_ms = float(parts[0])
        ax = float(parts[1]); ay = float(parts[2]); az = float(parts[3])
        gx_dps = float(parts[4]); gy_dps = float(parts[5]); gz_dps = float(parts[6])
        return t_ms, ax, ay, az, gx_dps, gy_dps, gz_dps
    except ValueError:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=1.0)

    ap.add_argument("--sample_hz", type=float, default=50.0)
    ap.add_argument("--win_sec", type=float, default=0.6)
    ap.add_argument("--corr_alpha", type=float, default=0.10)
    ap.add_argument("--bias_alpha", type=float, default=0.05)

    # stillness thresholds (optional)
    ap.add_argument("--g_lo", type=float, default=0.98)
    ap.add_argument("--g_hi", type=float, default=1.02)
    ap.add_argument("--a_std", type=float, default=0.010)
    ap.add_argument("--gyro_std", type=float, default=0.08)  # rad/s

    args = ap.parse_args()

    still = StillnessCfg(
        sample_hz=args.sample_hz,
        win_sec=args.win_sec,
        g_lo=args.g_lo,
        g_hi=args.g_hi,
        a_std=args.a_std,
        gyro_std=args.gyro_std,
    )
    cfg = FusionCfg(corr_alpha=args.corr_alpha, bias_alpha=args.bias_alpha)
    fusion = QuatFusion(still, cfg)

    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    time.sleep(1.0)
    ser.reset_input_buffer()

    d2r = math.pi / 180.0
    last_t_s = None

    print("t_ms,ax_g,ay_g,az_g,wx,wy,wz,wx_corr,wy_corr,wz_corr,a_norm_g,still,"
          "qx,qy,qz,qw,ghx,ghy,ghz,ugx,ugy,ugz,roll,pitch,yaw")

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            if line.startswith("t_ms,ax_g") or "serial_ok" in line or "MPU found" in line or "ping_" in line:
                continue
            if "read_fail" in line:
                continue

            parsed = parse_line(line)
            if parsed is None:
                continue

            t_ms, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps = parsed

            t_s = t_ms * 1e-3
            if last_t_s is None:
                last_t_s = t_s
                continue

            dt = t_s - last_t_s
            last_t_s = t_s
            if dt <= 0.0 or dt > 0.2:
                dt = 1.0 / args.sample_hz

            wx = gx_dps * d2r
            wy = gy_dps * d2r
            wz = gz_dps * d2r

            out = fusion.step(dt, ax_g, ay_g, az_g, wx, wy, wz)

            wxc, wyc, wzc = out["omega_corr"]
            qx, qy, qz, qw = out["q"]
            ghx, ghy, ghz = out["g_hat"]
            ugx, ugy, ugz = out["anti_g"]
            roll, pitch, yaw = out["euler_deg"]

            print(f"{t_ms:.0f},{ax_g:.5f},{ay_g:.5f},{az_g:.5f},"
                  f"{wx:.6f},{wy:.6f},{wz:.6f},"
                  f"{wxc:.6f},{wyc:.6f},{wzc:.6f},"
                  f"{out['a_norm_g']:.4f},{1 if out['still'] else 0},"
                  f"{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f},"
                  f"{ghx:.6f},{ghy:.6f},{ghz:.6f},"
                  f"{ugx:.6f},{ugy:.6f},{ugz:.6f},"
                  f"{roll:.2f},{pitch:.2f},{yaw:.2f}")

    except KeyboardInterrupt:
        print("\n# stopped")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
