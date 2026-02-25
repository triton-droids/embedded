import time
import struct
import can
import math

CAN_ID_RSP1 = 0x101
CAN_ID_RSP2 = 0x102

# MPU6050 default config used above:
ACC_S = 16384.0   # LSB/g (±2g)
GYR_S = 131.0     # LSB/(deg/s) (±250 dps)

def be16(b0, b1):
    return struct.unpack(">h", bytes([b0, b1]))[0]

def unit3(x, y, z, eps=1e-12):
    n = math.sqrt(x*x + y*y + z*z)
    if n < eps:
        return None
    return (x/n, y/n, z/n)

def main():
    bus = can.interface.Bus(channel="can0", bustype="socketcan")

    print("t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,ugx,ugy,ugz,seq")

    # cache
    got1 = False
    got2 = False
    ax=ay=az=gx=gy=gz=0
    seq = None

    while True:
        msg = bus.recv(timeout=1.0)
        if msg is None:
            continue
        if msg.is_extended_id:
            continue

        d = msg.data
        if msg.arbitration_id == CAN_ID_RSP1 and len(d) == 8:
            ax = be16(d[0], d[1])
            ay = be16(d[2], d[3])
            az = be16(d[4], d[5])
            gx = be16(d[6], d[7])
            got1 = True

        elif msg.arbitration_id == CAN_ID_RSP2 and len(d) == 8:
            gy = be16(d[0], d[1])
            gz = be16(d[2], d[3])
            seq = d[7]
            got2 = True

        if got1 and got2:
            t_ms = int(time.time() * 1000)

            ax_g = ax / ACC_S
            ay_g = ay / ACC_S
            az_g = az / ACC_S

            gx_dps = gx / GYR_S
            gy_dps = gy / GYR_S
            gz_dps = gz / GYR_S

            # “反重力方向”单位矢量：用加速度方向近似重力方向（静止/低动态）
            g_hat = unit3(ax_g, ay_g, az_g)
            if g_hat is None:
                ugx=ugy=ugz=float("nan")
            else:
                ugx, ugy, ugz = (-g_hat[0], -g_hat[1], -g_hat[2])

            print(f"{t_ms},{ax_g:.5f},{ay_g:.5f},{az_g:.5f},"
                  f"{gx_dps:.3f},{gy_dps:.3f},{gz_dps:.3f},"
                  f"{ugx:.6f},{ugy:.6f},{ugz:.6f},{seq}")

            got1 = got2 = False

if __name__ == "__main__":
    main()
