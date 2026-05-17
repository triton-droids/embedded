# imu_read.py
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Dict, Generator, Optional, Tuple

import numpy as np

try:
    import serial  # pyserial
except Exception:
    serial = None


def _normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v if n < eps else (v / n)


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    # q = [w, x, y, z]
    w1, x1, y1, z1 = map(float, q1)
    w2, x2, y2, z2 = map(float, q2)
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=np.float64)


def quat_conj(q: np.ndarray) -> np.ndarray:
    w, x, y, z = map(float, q)
    return np.array([w, -x, -y, -z], dtype=np.float64)


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    # v' = q * [0,v] * q_conj
    vq = np.array([0.0, float(v[0]), float(v[1]), float(v[2])], dtype=np.float64)
    return quat_mul(quat_mul(q, vq), quat_conj(q))[1:]


def quat_from_omega(omega_rad_s: np.ndarray) -> np.ndarray:
    # 用在 qdot = 0.5 * q ⊗ [0, omega]
    return np.array([0.0, float(omega_rad_s[0]), float(omega_rad_s[1]), float(omega_rad_s[2])], dtype=np.float64)


def qdot(q: np.ndarray, omega_rad_s: np.ndarray) -> np.ndarray:
    return 0.5 * quat_mul(q, quat_from_omega(omega_rad_s))


def rk4_quat_step(q: np.ndarray, omega_rad_s: np.ndarray, dt: float) -> np.ndarray:
    # omega 这里假设在 dt 内常值（IMU 采样通常够用）
    k1 = qdot(q, omega_rad_s)
    k2 = qdot(q + 0.5*dt*k1, omega_rad_s)
    k3 = qdot(q + 0.5*dt*k2, omega_rad_s)
    k4 = qdot(q + dt*k3, omega_rad_s)
    q_next = q + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)
    return _normalize(q_next)


@dataclass
class RK4DeadReckoner:
    gravity_world: Tuple[float, float, float] = (0.0, 0.0, 9.80665)
    q_wb: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)  # world<-body
    vel_w: np.ndarray = np.zeros(3, dtype=np.float64)
    pos_w: np.ndarray = np.zeros(3, dtype=np.float64)

    # 可选：简单的陀螺零偏（你也可以外面做校准后塞进来）
    gyro_bias_rad_s: np.ndarray = np.zeros(3, dtype=np.float64)

    def reset(self) -> None:
        self.q_wb = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.vel_w = np.zeros(3, dtype=np.float64)
        self.pos_w = np.zeros(3, dtype=np.float64)
        self.gyro_bias_rad_s = np.zeros(3, dtype=np.float64)

    def step(
        self,
        acc_body_m_s2: np.ndarray,
        gyro_body_rad_s: np.ndarray,
        dt: float,
    ) -> Dict[str, np.ndarray]:
        # 1) 姿态：RK4 积分陀螺
        omega = gyro_body_rad_s - self.gyro_bias_rad_s
        self.q_wb = rk4_quat_step(self.q_wb, omega, dt)

        # 2) 加速度转世界系并减重力
        acc_w = quat_rotate(self.q_wb, acc_body_m_s2)
        g_w = np.array(self.gravity_world, dtype=np.float64)
        lin_acc_w = acc_w - g_w

        # 3) 速度/位置积分（这里用简单欧拉；你也能改成 RK4/梯形）
        self.vel_w = self.vel_w + lin_acc_w * dt
        self.pos_w = self.pos_w + self.vel_w * dt

        return {
            "q_wb": self.q_wb.copy(),
            "acc_w": acc_w,
            "lin_acc_w": lin_acc_w,
            "vel_w": self.vel_w.copy(),
            "pos_w": self.pos_w.copy(),
        }


def iter_imu_samples(
    source: str = "serial",
    port: str = "/dev/ttyUSB0",
    baud: int = 115200,
    rate_hz: Optional[float] = None,
    include_all: bool = True,
    integrator: Optional[RK4DeadReckoner] = None,
    acc_units: str = "m/s^2",   # "m/s^2" or "g"
    gyro_units: str = "rad/s",  # "rad/s" or "deg/s"
) -> Generator[Dict, None, None]:
    """
    读取 IMU 数据，输出 dict：
      - t_wall, dt
      - acc_m_s2 (3,), gyro_rad_s (3,)
      - 可选：rpy_deg, lin_pos_m, lin_vel_m_s
    """
    if source != "serial":
        raise ValueError("目前只实现 source='serial'")

    if serial is None:
        raise RuntimeError("缺少 pyserial：pip install pyserial")

    ser = serial.Serial(port, baud, timeout=1.0)

    last_t = None
    target_dt = (1.0 / rate_hz) if rate_hz else None

    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        # 支持：一行 JSON
        try:
            msg = json.loads(line)
        except Exception:
            continue

        t_now = time.time()
        if last_t is None:
            dt = target_dt if target_dt else 0.0
        else:
            dt = t_now - last_t
        last_t = t_now

        acc = np.array(msg.get("acc", [0, 0, 0]), dtype=np.float64)
        gyro = np.array(msg.get("gyro", [0, 0, 0]), dtype=np.float64)

        # 单位换算
        if acc_units.lower() in ["g", "grav", "gravity"]:
            acc = acc * 9.80665
        elif acc_units.lower() in ["m/s^2", "mps2", "mps^2"]:
            pass
        else:
            raise ValueError("acc_units 只能是 'm/s^2' 或 'g'")

        if gyro_units.lower() in ["deg/s", "dps", "degps"]:
            gyro = gyro * (math.pi / 180.0)
        elif gyro_units.lower() in ["rad/s", "rads"]:
            pass
        else:
            raise ValueError("gyro_units 只能是 'rad/s' 或 'deg/s'")

        out = {
            "t_wall": t_now,
            "dt": float(dt),
            "acc_m_s2": acc,
            "gyro_rad_s": gyro,
        }

        if integrator is not None and dt > 0:
            st = integrator.step(acc, gyro, dt)
            q = st["q_wb"]
            # 方便调试：rpy
            w, x, y, z = map(float, q)
            # ZYX yaw-pitch-roll
            yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
            pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
            roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
            out.update({
                "quat_wb": q,
                "rpy_deg": np.array([roll, pitch, yaw]) * 180.0 / math.pi,
                "lin_vel_m_s": st["vel_w"],
                "lin_pos_m": st["pos_w"],
                "lin_acc_w_m_s2": st["lin_acc_w"],
            })

        if include_all:
            out["raw"] = msg

        yield out