#!/usr/bin/env python3
import socket
import time
import subprocess
import platform


UDP_IP = "0.0.0.0"
UDP_PORT = 5005
ESP32_AP_SUBNET_PREFIX = "192.168.4."


def get_local_ips():
    ips = set()

    # Cross-platform socket method
    try:
        hostname = socket.gethostname()
        for info in socket.getaddrinfo(hostname, None):
            ip = str(info[4][0])
            if "." in ip and not ip.startswith("127."):
                ips.add(ip)
    except Exception:
        pass

    # Linux ip command fallback
    if platform.system().lower() == "linux":
        try:
            out = subprocess.check_output(
                ["ip", "-4", "addr"],
                text=True,
                stderr=subprocess.DEVNULL
            )
            for line in out.splitlines():
                line = line.strip()
                if line.startswith("inet "):
                    ip = line.split()[1].split("/")[0]
                    if not ip.startswith("127."):
                        ips.add(ip)
        except Exception:
            pass

    return sorted(ips)


def check_esp32_network():
    ips = get_local_ips()

    print("Local IPv4 addresses:")
    for ip in ips:
        print(f"  {ip}")

    esp32_ips = [ip for ip in ips if ip.startswith(ESP32_AP_SUBNET_PREFIX)]

    if esp32_ips:
        print("\nOK: You seem connected to the ESP32 AP network.")
        print(f"ESP32 client IP: {esp32_ips[0]}")
        print("Expected ESP32 AP IP: 192.168.4.1")
        return True

    print("\nWARNING: You do not seem connected to the ESP32 AP network.")
    print("Expected your computer IP to look like: 192.168.4.x")
    print("Connect to WiFi SSID: ESP32S3_IMU")
    print("Password: 12345678")
    return False


def parse_imu_message(msg: str) -> dict:
    data = {}

    for part in msg.strip().split(","):
        if "=" not in part:
            continue

        key, value = part.split("=", 1)
        key = key.strip()
        value = value.strip()

        try:
            if key == "seq":
                data[key] = int(value)
            else:
                data[key] = float(value)
        except ValueError:
            data[key] = value

    return data


def main():
    check_esp32_network()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"\nListening UDP on {UDP_IP}:{UDP_PORT}")
    print("Press Ctrl+C to stop.\n")

    last_time = None

    while True:
        data, addr = sock.recvfrom(4096)
        now = time.time()

        if last_time is None:
            dt = 0.0
        else:
            dt = now - last_time
        last_time = now

        msg = data.decode("utf-8", errors="replace").strip()
        parsed = parse_imu_message(msg)

        print("=" * 60)
        print(f"From: {addr[0]}:{addr[1]}")
        print(f"dt: {dt:.3f} s")
        print(f"raw: {msg}")
        print(f"parsed: {parsed}")


if __name__ == "__main__":
    main()