#!/usr/bin/env python3
import json
import math
import sys
import time
from urllib import request, error


class GatewayGainTuner:
    def __init__(self, host="127.0.0.1", port=8080, timeout=2.0):
        self.base_url = f"http://{host}:{port}"
        self.target_url = f"{self.base_url}/target"
        self.health_url = f"{self.base_url}/health"
        self.timeout = timeout

    def health_check(self):
        try:
            req = request.Request(self.health_url, method="GET")
            with request.urlopen(req, timeout=self.timeout) as resp:
                data = json.loads(resp.read().decode("utf-8"))
            return True, data
        except Exception as e:
            return False, str(e)

    def send_joint_command(
        self,
        joint_name,
        mode="motion",
        position=0.0,
        velocity=0.0,
        acceleration=0.0,
        torque=0.0,
        kp=10.0,
        kd=0.2,
    ):
        payload = {
            "commands": {
                joint_name: {
                    "mode": mode,
                    "position": float(position),
                    "velocity": float(velocity),
                    "acceleration": float(acceleration),
                    "torque": float(torque),
                    "kp": float(kp),
                    "kd": float(kd),
                }
            }
        }
        return self._post_json(payload)

    def send_multi_joint_command(self, commands_dict):
        """
        commands_dict example:
        {
            "left_knee_joint": {"mode":"motion","position":-0.3,"velocity":1.0,"kp":10,"kd":0.2},
            "right_knee_joint": {"mode":"motion","position":-0.3,"velocity":1.0,"kp":10,"kd":0.2}
        }
        """
        payload = {"commands": commands_dict}
        return self._post_json(payload)

    def enable_joint(self, joint_name):
        return self.send_joint_command(joint_name, mode="enable")

    def disable_joint(self, joint_name):
        return self.send_joint_command(joint_name, mode="disable")

    def hold_joint(self, joint_name, position, kp=10.0, kd=0.2):
        return self.send_joint_command(
            joint_name=joint_name,
            mode="motion",
            position=position,
            velocity=0.0,
            torque=0.0,
            kp=kp,
            kd=kd,
        )

    def _post_json(self, payload):
        data = json.dumps(payload).encode("utf-8")
        req = request.Request(
            self.target_url,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with request.urlopen(req, timeout=self.timeout) as resp:
                body = resp.read().decode("utf-8")
                return True, json.loads(body) if body else {}
        except error.HTTPError as e:
            try:
                msg = e.read().decode("utf-8")
            except Exception:
                msg = str(e)
            return False, msg
        except Exception as e:
            return False, str(e)


def print_help():
    print("\nCommands:")
    print("  health")
    print("  enable <joint>")
    print("  disable <joint>")
    print("  hold <joint> <pos_rad> <kp> <kd>")
    print("  motion <joint> <pos_rad> <vel_rad_s> <kp> <kd>")
    print("  sweep_kp <joint> <pos_rad> <vel_rad_s> <kd> <kp_start> <kp_end> <kp_step>")
    print("  sweep_kd <joint> <pos_rad> <vel_rad_s> <kp> <kd_start> <kd_end> <kd_step>")
    print("  q")


def main():
    host = "127.0.0.1"
    port = 8080

    if len(sys.argv) >= 2:
        host = sys.argv[1]
    if len(sys.argv) >= 3:
        port = int(sys.argv[2])

    tuner = GatewayGainTuner(host=host, port=port)

    ok, info = tuner.health_check()
    if not ok:
        print(f"[ERROR] Gateway not reachable: {info}")
        print(f"Expected gateway at http://{host}:{port}")
        return
    print(f"[OK] Gateway reachable: {info}")

    print_help()

    while True:
        try:
            s = input("\n[gain-tuner] >> ").strip()
            if not s:
                continue
            parts = s.split()
            cmd = parts[0].lower()

            if cmd in ("q", "quit", "exit"):
                break

            elif cmd == "health":
                ok, info = tuner.health_check()
                print("[OK]" if ok else "[ERR]", info)

            elif cmd == "enable" and len(parts) == 2:
                joint = parts[1]
                ok, resp = tuner.enable_joint(joint)
                print("[OK]" if ok else "[ERR]", resp)

            elif cmd == "disable" and len(parts) == 2:
                joint = parts[1]
                ok, resp = tuner.disable_joint(joint)
                print("[OK]" if ok else "[ERR]", resp)

            elif cmd == "hold" and len(parts) == 5:
                joint = parts[1]
                pos = float(parts[2])
                kp = float(parts[3])
                kd = float(parts[4])
                ok, resp = tuner.hold_joint(joint, pos, kp, kd)
                print("[OK]" if ok else "[ERR]", resp)

            elif cmd == "motion" and len(parts) == 6:
                joint = parts[1]
                pos = float(parts[2])
                vel = float(parts[3])
                kp = float(parts[4])
                kd = float(parts[5])
                ok, resp = tuner.send_joint_command(
                    joint_name=joint,
                    mode="motion",
                    position=pos,
                    velocity=vel,
                    kp=kp,
                    kd=kd,
                )
                print("[OK]" if ok else "[ERR]", resp)

            elif cmd == "sweep_kp" and len(parts) == 8:
                joint = parts[1]
                pos = float(parts[2])
                vel = float(parts[3])
                kd = float(parts[4])
                kp_start = float(parts[5])
                kp_end = float(parts[6])
                kp_step = float(parts[7])

                kp = kp_start
                while kp <= kp_end + 1e-9:
                    ok, resp = tuner.send_joint_command(
                        joint_name=joint,
                        mode="motion",
                        position=pos,
                        velocity=vel,
                        kp=kp,
                        kd=kd,
                    )
                    print(f"kp={kp:.4f} ->", "[OK]" if ok else "[ERR]", resp)
                    kp += kp_step
                    time.sleep(0.8)

            elif cmd == "sweep_kd" and len(parts) == 8:
                joint = parts[1]
                pos = float(parts[2])
                vel = float(parts[3])
                kp = float(parts[4])
                kd_start = float(parts[5])
                kd_end = float(parts[6])
                kd_step = float(parts[7])

                kd = kd_start
                while kd <= kd_end + 1e-9:
                    ok, resp = tuner.send_joint_command(
                        joint_name=joint,
                        mode="motion",
                        position=pos,
                        velocity=vel,
                        kp=kp,
                        kd=kd,
                    )
                    print(f"kd={kd:.4f} ->", "[OK]" if ok else "[ERR]", resp)
                    kd += kd_step
                    time.sleep(0.8)

            else:
                print("Invalid command.")
                print_help()

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"[ERROR] {e}")


if __name__ == "__main__":
    main()