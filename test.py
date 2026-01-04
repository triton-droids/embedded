#!/usr/bin/env python3
"""
MIT position movement test by CAN_ID (RobStride protocol, raw python-can).

Requires:
  pip install python-can numpy

Run (Linux):
  sudo ip link set can0 type can bitrate 1000000
  sudo ip link set up can0
  python3 mit_position_test_raw.py --id 10 --model rs-02 --deg 30 --kp 30 --kd 0.5 --hz 50

Notes:
- This uses SocketCAN (Linux only).
- Start with SMALL angles and SMALL gains.
"""

import argparse
import struct
import time
import signal
import sys

import can
import numpy as np

# --- Protocol constants (from your repo) ---
class CommunicationType:
    GET_DEVICE_ID       = 0
    OPERATION_CONTROL   = 1
    OPERATION_STATUS    = 2
    ENABLE              = 3
    DISABLE             = 4
    SET_ZERO_POSITION   = 6
    SET_DEVICE_ID       = 7
    READ_PARAMETER      = 17
    WRITE_PARAMETER     = 18
    FAULT_REPORT        = 21

class ParameterType:
    MODE = (0x7005, np.int8, "run_mode")

MODEL_MIT_POSITION_TABLE = {
    "rs-00": 4 * np.pi,
    "rs-01": 4 * np.pi,
    "rs-02": 4 * np.pi,
    "rs-03": 4 * np.pi,
    "rs-04": 4 * np.pi,
    "rs-05": 4 * np.pi,
    "rs-06": 4 * np.pi,
}
MODEL_MIT_VELOCITY_TABLE = {
    "rs-00": 50, "rs-01": 44, "rs-02": 44, "rs-03": 50, "rs-04": 15, "rs-05": 33, "rs-06": 20
}
MODEL_MIT_TORQUE_TABLE = {
    "rs-00": 17, "rs-01": 17, "rs-02": 17, "rs-03": 60, "rs-04": 120, "rs-05": 17, "rs-06": 60
}
MODEL_MIT_KP_TABLE = {
    "rs-00": 500.0, "rs-01": 500.0, "rs-02": 500.0, "rs-03": 5000.0, "rs-04": 5000.0, "rs-05": 500.0, "rs-06": 5000.0
}
MODEL_MIT_KD_TABLE = {
    "rs-00": 5.0, "rs-01": 5.0, "rs-02": 5.0, "rs-03": 100.0, "rs-04": 100.0, "rs-05": 5.0, "rs-06": 100.0
}

HOST_ID = 0xFF  # same idea as your bus.py: host ID high


def build_ext_id(comm_type: int, extra_data: int, device_id: int) -> int:
    # ext_id = (communication_type << 24) | (extra_data << 8) | (device_id)
    return ((comm_type & 0x1F) << 24) | ((extra_data & 0xFFFF) << 8) | (device_id & 0xFF)


def send_frame(bus: can.Bus, comm_type: int, extra_data: int, device_id: int, data: bytes = b""):
    msg = can.Message(
        arbitration_id=build_ext_id(comm_type, extra_data, device_id),
        is_extended_id=True,
        data=data,
    )
    bus.send(msg)


def write_parameter_mode(bus: can.Bus, device_id: int, mode: int):
    # Matches your bus.write(): data = <HH> + <bBH> for int8
    param_id, _, _ = ParameterType.MODE
    value_buffer = struct.pack("<bBH", int(mode), 0, 0)
    data = struct.pack("<HH", param_id, 0x00) + value_buffer
    send_frame(bus, CommunicationType.WRITE_PARAMETER, HOST_ID, device_id, data)


def encode_mit_command(model: str, p_des_rad: float, v_des: float, kp: float, kd: float, t_ff: float) -> tuple[int, bytes]:
    """
    Encodes MIT OPERATION_CONTROL frame like your RobstrideBus.write_operation_frame():
      - CAN ext_id extra_data = torque_u16
      - data = >HHHH(position_u16, velocity_u16, kp_u16, kd_u16)
    """
    # clamp & scale
    p_lim = MODEL_MIT_POSITION_TABLE[model]
    v_lim = MODEL_MIT_VELOCITY_TABLE[model]
    t_lim = MODEL_MIT_TORQUE_TABLE[model]
    kp_lim = MODEL_MIT_KP_TABLE[model]
    kd_lim = MODEL_MIT_KD_TABLE[model]

    p = float(np.clip(p_des_rad, -p_lim, p_lim))
    v = float(np.clip(v_des, -v_lim, v_lim))
    kp = float(np.clip(kp, 0.0, kp_lim))
    kd = float(np.clip(kd, 0.0, kd_lim))
    t = float(np.clip(t_ff, -t_lim, t_lim))

    position_u16 = int(((p / p_lim) + 1.0) * 0x7FFF)
    velocity_u16 = int(((v / v_lim) + 1.0) * 0x7FFF)
    kp_u16 = int((kp / kp_lim) * 0xFFFF)
    kd_u16 = int((kd / kd_lim) * 0xFFFF)
    torque_u16 = int(((t / t_lim) + 1.0) * 0x7FFF)

    # clip to uint16 range
    position_u16 = int(np.clip(position_u16, 0, 0xFFFF))
    velocity_u16 = int(np.clip(velocity_u16, 0, 0xFFFF))
    kp_u16 = int(np.clip(kp_u16, 0, 0xFFFF))
    kd_u16 = int(np.clip(kd_u16, 0, 0xFFFF))
    torque_u16 = int(np.clip(torque_u16, 0, 0xFFFF))

    data = struct.pack(">HHHH", position_u16, velocity_u16, kp_u16, kd_u16)
    return torque_u16, data


def decode_status(model: str, extra_data: int, data: bytes):
    """
    Matches your receive_status_frame():
      data = >HHHH (position_u16, velocity_u16, torque_i16, temperature_u16)
      normalize each into physical units.
    """
    if len(data) < 8:
        return None

    pos_u16, vel_u16, tq_u16, temp_u16 = struct.unpack(">HHHH", data[:8])

    p_lim = MODEL_MIT_POSITION_TABLE[model]
    v_lim = MODEL_MIT_VELOCITY_TABLE[model]
    t_lim = MODEL_MIT_TORQUE_TABLE[model]

    pos = (float(pos_u16) / 0x7FFF - 1.0) * p_lim
    vel = (float(vel_u16) / 0x7FFF - 1.0) * v_lim
    tq  = (float(tq_u16) / 0x7FFF - 1.0) * t_lim
    temp = float(temp_u16) * 0.1

    # device_id appears in low 8 bits of extra_data in your code
    device_id = (extra_data >> 0) & 0xFF
    return device_id, pos, vel, tq, temp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--id", type=int, required=True, help="Motor CAN_ID (1..255)")
    ap.add_argument("--model", type=str, required=True, choices=sorted(MODEL_MIT_POSITION_TABLE.keys()))
    ap.add_argument("--deg", type=float, default=10.0, help="Target position in degrees")
    ap.add_argument("--kp", type=float, default=10.0)
    ap.add_argument("--kd", type=float, default=0.2)
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--set_mode0", action="store_true", help="Write MODE=0 (MIT) before control loop")
    ap.add_argument("--duration", type=float, default=5.0, help="Seconds to run, then disable")
    ap.add_argument("--print_hz", type=float, default=10.0, help="Status print rate")
    args = ap.parse_args()

    device_id = int(args.id)
    model = args.model.lower()
    target_rad = np.deg2rad(args.deg)

    # Basic safety clamps (keep it reasonable)
    # (You can adjust; just don't start with huge commands.)
    target_rad = float(np.clip(target_rad, -np.deg2rad(90), np.deg2rad(90)))

    running = True

    def shutdown(_sig=None, _frm=None):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    bus = can.interface.Bus(interface="socketcan", channel=args.channel, bitrate=args.bitrate)

    try:
        print(f"[+] Using {args.channel} @ {args.bitrate} bps")
        print(f"[+] Motor ID={device_id}, model={model}")
        print("[+] Enabling motor…")
        send_frame(bus, CommunicationType.ENABLE, HOST_ID, device_id, b"")

        time.sleep(0.15)

        if args.set_mode0:
            print("[+] Setting MODE=0 (MIT)…")
            write_parameter_mode(bus, device_id, 0)
            time.sleep(0.15)

        dt = 1.0 / float(args.hz)
        t_end = time.time() + float(args.duration)
        next_print = time.time()

        print(f"[+] Commanding step to {args.deg:.2f} deg ({target_rad:.3f} rad) at {args.hz:.1f} Hz for {args.duration:.2f}s")
        print("[+] Ctrl+C to stop.")

        while running and time.time() < t_end:
            # Send MIT command
            torque_u16, data = encode_mit_command(
                model=model,
                p_des_rad=target_rad,
                v_des=0.0,
                kp=args.kp,
                kd=args.kd,
                t_ff=0.0
            )
            send_frame(bus, CommunicationType.OPERATION_CONTROL, torque_u16, device_id, data)

            # Drain one response (best-effort)
            msg = bus.recv(timeout=0.01)
            if msg and msg.is_extended_id:
                comm_type = (msg.arbitration_id >> 24) & 0x1F
                extra = (msg.arbitration_id >> 8) & 0xFFFF

                if comm_type == CommunicationType.OPERATION_STATUS:
                    decoded = decode_status(model, extra, msg.data)
                    if decoded and time.time() >= next_print:
                        mid, pos, vel, tq, temp = decoded
                        print(f"ID={mid:3d}  pos={pos:+.3f} rad  vel={vel:+.3f} rad/s  tq={tq:+.2f} Nm  T={temp:.1f} C")
                        next_print = time.time() + 1.0 / float(args.print_hz)

                elif comm_type == CommunicationType.FAULT_REPORT:
                    print("[!] FAULT_REPORT received. Stopping.")
                    break

            time.sleep(dt)

    finally:
        print("[+] Disabling motor…")
        try:
            send_frame(bus, CommunicationType.DISABLE, HOST_ID, device_id, b"")
            time.sleep(0.05)
        except Exception:
            pass
        try:
            bus.shutdown()
        except Exception:
            pass
        print("[+] Done.")


if __name__ == "__main__":
    main()
