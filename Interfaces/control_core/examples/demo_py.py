#!/usr/bin/env python3
"""demo_py.py (NEW CONTROL STRUCTURE)

This demo is **NOT** a control_core unit test anymore.
It is an **external client** that talks to target_gateway_node over HTTP.

Usage examples:
  # Health check
  python3 demo_py.py --health

  # Enable then command one joint
  python3 demo_py.py --enable shoulder_pitch --mode velocity --vel 0.8

  # Sweep velocity (+ then -) at 20 Hz for 5 seconds
  python3 demo_py.py --enable shoulder_pitch --mode velocity --vel 0.8 --sweep --rate 20 --secs 5

Expected pipeline:
  demo_py.py  ->  target_gateway_node (HTTP /target)  ->  /desired_motor_subset
             ->  cpp_control_node (50 Hz)            ->  /motor_commands
             ->  python_can_node (SocketCAN)         ->  can0/vcan0
"""

import argparse
import json
import sys
import time
import urllib.request


def http_json(method: str, url: str, payload=None, timeout: float = 0.5):
    data = None
    headers = {"Accept": "application/json"}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"

    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        body = resp.read().decode("utf-8")
        try:
            return resp.status, json.loads(body)
        except Exception:
            return resp.status, {"raw": body}


def build_payload(joint: str, mode: str, pos: float, vel: float, acc: float, tq: float, kp: float, kd: float):
    cmd = {"mode": mode}
    if mode in ("velocity", "vel"):
        cmd["velocity"] = vel
    elif mode in ("position", "pos"):
        cmd["position"] = pos
        cmd["velocity"] = vel
        cmd["acceleration"] = acc
    elif mode == "motion":
        cmd.update({
            "position": pos,
            "velocity": vel,
            "acceleration": acc,
            "torque": tq,
            "kp": kp,
            "kd": kd,
        })
    elif mode in ("enable", "disable"):
        pass
    else:
        raise ValueError(f"unknown mode: {mode}")

    return {"commands": {joint: cmd}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--timeout", type=float, default=0.5)

    ap.add_argument("--health", action="store_true", help="GET /health and exit")
    ap.add_argument("--enable", metavar="JOINT", help="Send MODE_ENABLE for JOINT before other commands")

    ap.add_argument("--joint", default="shoulder_pitch")
    ap.add_argument("--mode", default="velocity", choices=["velocity", "position", "motion", "enable", "disable"])

    ap.add_argument("--pos", type=float, default=0.0)
    ap.add_argument("--vel", type=float, default=0.0)
    ap.add_argument("--acc", type=float, default=0.0)
    ap.add_argument("--tq", type=float, default=0.0)
    ap.add_argument("--kp", type=float, default=40.0)
    ap.add_argument("--kd", type=float, default=1.5)

    ap.add_argument("--sweep", action="store_true", help="Alternate +vel and -vel")
    ap.add_argument("--rate", type=float, default=20.0, help="Send rate for sweep (Hz)")
    ap.add_argument("--secs", type=float, default=2.0, help="Duration for sweep (seconds)")

    args = ap.parse_args()
    base = f"http://{args.host}:{args.port}"

    if args.health:
        status, obj = http_json("GET", base + "/health", timeout=args.timeout)
        print(status, json.dumps(obj, indent=2))
        return 0

    if args.enable:
        payload = build_payload(args.enable, "enable", 0.0, 0.0, 0.0, 0.0, args.kp, args.kd)
        status, obj = http_json("POST", base + "/target", payload=payload, timeout=args.timeout)
        print("ENABLE:", status, obj)
        time.sleep(0.1)

    if not args.sweep:
        payload = build_payload(args.joint, args.mode, args.pos, args.vel, args.acc, args.tq, args.kp, args.kd)
        status, obj = http_json("POST", base + "/target", payload=payload, timeout=args.timeout)
        print("CMD:", status, obj)
        return 0

    if args.mode != "velocity":
        print("--sweep currently supports --mode velocity only", file=sys.stderr)
        return 2

    dt = 1.0 / max(args.rate, 1e-6)
    t_end = time.time() + args.secs
    sign = 1.0
    sent = 0

    while time.time() < t_end:
        payload = build_payload(args.joint, "velocity", 0.0, sign * args.vel, 0.0, 0.0, args.kp, args.kd)
        status, obj = http_json("POST", base + "/target", payload=payload, timeout=args.timeout)
        if status != 200:
            print("ERROR:", status, obj)
            return 1
        sent += 1
        sign *= -1.0
        time.sleep(dt)

    print(f"sweep done: sent {sent} messages")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
