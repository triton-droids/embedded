# imu_read.py
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Generator, Optional, Tuple

import numpy as np

try:
    import serial  # pyserial
except Exception:
    serial = None

G_M_S2 = 9.80665
INPUT_FORMATS = ("auto", "json", "bno085_csv")


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


def quat_from_rpy(roll_rad: float, pitch_rad: float, yaw_rad: float = 0.0) -> np.ndarray:
    """Return a [w, x, y, z] quaternion for ZYX yaw-pitch-roll."""
    cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
    cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
    cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ], dtype=np.float64)


def _to_si(
    acc: np.ndarray,
    gyro: np.ndarray,
    *,
    acc_units: str,
    gyro_units: str,
) -> Tuple[np.ndarray, np.ndarray]:
    if acc_units.lower() in ("g", "grav", "gravity"):
        acc = acc * G_M_S2
    elif acc_units.lower() not in ("m/s^2", "mps2", "mps^2"):
        raise ValueError("acc_units 只能是 'm/s^2' 或 'g'")

    if gyro_units.lower() in ("deg/s", "dps", "degps"):
        gyro = gyro * (math.pi / 180.0)
    elif gyro_units.lower() not in ("rad/s", "rads"):
        raise ValueError("gyro_units 只能是 'rad/s' 或 'deg/s'")
    return acc, gyro


def parse_imu_line(
    line: str,
    *,
    input_format: str = "auto",
    acc_units: str = "m/s^2",
    gyro_units: str = "rad/s",
) -> Optional[Dict[str, Any]]:
    """Parse one generic JSON or BNO085 firmware CSV sample.

    BNO085 CSV format:
      t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg[,temp_c]

    Returned acceleration and angular velocity values always use ROS SI units.
    The firmware exposes roll and pitch but not yaw, so its quaternion uses
    yaw=0 and is marked as a partial orientation.
    """
    line = line.strip()
    if not line:
        return None

    selected_format = input_format.lower().strip()
    if selected_format not in INPUT_FORMATS:
        raise ValueError(f"input_format must be one of {INPUT_FORMATS}, got {input_format!r}")
    if selected_format == "auto":
        selected_format = "json" if line.startswith("{") else "bno085_csv"

    if selected_format == "json":
        try:
            msg = json.loads(line)
        except (json.JSONDecodeError, TypeError):
            return None
        if not isinstance(msg, dict):
            return None
        try:
            acc = np.asarray(msg.get("acc", [0, 0, 0]), dtype=np.float64)
            gyro = np.asarray(msg.get("gyro", [0, 0, 0]), dtype=np.float64)
            if acc.shape != (3,) or gyro.shape != (3,):
                return None
            acc, gyro = _to_si(
                acc, gyro, acc_units=acc_units, gyro_units=gyro_units
            )
        except (TypeError, ValueError):
            return None
        return {
            "acc_m_s2": acc,
            "gyro_rad_s": gyro,
            "sensor_time_s": None,
            "raw": msg,
            "source": "json",
        }

    if line.startswith(("serial_ok", "BNO085", "t_ms,")):
        return None
    parts = [part.strip() for part in line.split(",")]
    if len(parts) < 9:
        return None
    try:
        values = [float(value) for value in parts[:9]]
        temp_c = float(parts[9]) if len(parts) >= 10 else None
    except ValueError:
        return None
    numeric_values = values + ([] if temp_c is None else [temp_c])
    if not all(math.isfinite(value) for value in numeric_values):
        return None

    t_ms, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, roll_deg, pitch_deg = values
    return {
        "acc_m_s2": np.array([ax_g, ay_g, az_g], dtype=np.float64) * G_M_S2,
        "gyro_rad_s": np.radians(
            np.array([gx_dps, gy_dps, gz_dps], dtype=np.float64)
        ),
        "sensor_time_s": t_ms * 1e-3,
        "sensor_quat_wb": quat_from_rpy(
            math.radians(roll_deg), math.radians(pitch_deg)
        ),
        "sensor_rpy_deg": np.array([roll_deg, pitch_deg, 0.0], dtype=np.float64),
        "sensor_orientation_partial": True,
        "temperature_c": temp_c,
        "raw": line,
        "source": "bno085_csv",
    }


@dataclass
class RK4DeadReckoner:
    gravity_world: Tuple[float, float, float] = (0.0, 0.0, 9.80665)
    q_wb: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    )  # world<-body
    vel_w: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    pos_w: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))

    # 可选：简单的陀螺零偏（你也可以外面做校准后塞进来）
    gyro_bias_rad_s: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))

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
    input_format: str = "auto",  # "auto", "json", or "bno085_csv"
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
    input_format = input_format.lower().strip()
    if input_format not in INPUT_FORMATS:
        raise ValueError(f"input_format must be one of {INPUT_FORMATS}, got {input_format!r}")

    ser = serial.Serial(port, baud, timeout=1.0)
    last_wall_t = None
    last_sensor_t = None
    target_dt = (1.0 / rate_hz) if rate_hz else None

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            parsed = parse_imu_line(
                line,
                input_format=input_format,
                acc_units=acc_units,
                gyro_units=gyro_units,
            )
            if parsed is None:
                continue

            t_now = time.time()
            sensor_t = parsed.get("sensor_time_s")
            if sensor_t is not None and last_sensor_t is not None:
                dt = float(sensor_t) - float(last_sensor_t)
                if dt < -1.0:  # Arduino millis() rollover (~49.7 days)
                    dt += (2**32) * 1e-3
            elif last_wall_t is not None:
                dt = t_now - last_wall_t
            else:
                dt = target_dt if target_dt else 0.0
            last_wall_t = t_now
            if sensor_t is not None:
                last_sensor_t = float(sensor_t)

            acc, gyro = parsed["acc_m_s2"], parsed["gyro_rad_s"]
            out = dict(parsed)
            out.update({"t_wall": t_now, "dt": float(dt)})

            # Reject long gaps to avoid a single large dead-reckoning step.
            if integrator is not None and 0.0 < dt <= 0.5:
                st = integrator.step(acc, gyro, dt)
                q = st["q_wb"]
                w, x, y, z = map(float, q)
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

            if not include_all:
                out.pop("raw", None)
            yield out
    finally:
        ser.close()
