"""
imu_stream.py

Importable IMU streaming + optional RK4 dead-reckoning integrator.

What you get
------------
1) Serial mode:
   - Parses Arduino/ESP32 CSV lines:
     t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,temp_C
     (temp_C optional)

2) I2C mode:
   - Reads from imu_i2c_reader.MPU6050Reader(bus_id, addr).read()
     returning (acc_ms2, gyro_rads)

3) Optional integrator (RK4):
   - Integrates orientation quaternion with gyro (RK4).
   - Rotates body acceleration into world frame.
   - Removes gravity to get linear acceleration in world.
   - Integrates velocity/position with RK4.
   - Simple ZUPT + gyro bias update when "still".

Default output
--------------
By default, iter_imu_samples() only yields:
  - acc_g: (ax, ay, az) in g
  - gyro_dps: (gx, gy, gz) in deg/s

To output all known fields:
  - include_all=True  OR  keys=None

To enable integration:
  - create an RK4DeadReckoner() and pass integrator=...

Rate control
------------
rate_hz:
  - Serial: always reads continuously, but only EMITS at rate_hz (drops extra lines).
  - I2C: sleeps to sample/emit at rate_hz.

Dependencies
------------
- Serial mode: pyserial (pip install pyserial)
- I2C mode: a local imu_i2c_reader.py providing MPU6050Reader

Usage examples
--------------
A) Serial, default fields (acc_g + gyro_dps), 50Hz output
    from imu_stream import iter_imu_samples
    for s in iter_imu_samples(source="serial", port="/dev/ttyUSB0", rate_hz=50):
        print(s)

B) Serial, all fields
    for s in iter_imu_samples(source="serial", port="/dev/ttyUSB0", include_all=True):
        print(s.keys())

C) Serial + integrator (all fields includes integrated states)
    from imu_stream import iter_imu_samples, RK4DeadReckoner
    dr = RK4DeadReckoner(gravity_world=(0.0, 0.0, 9.80665))  # z-up world, stationary acc_world ≈ +g
    for s in iter_imu_samples(source="serial", port="/dev/ttyUSB0", integrator=dr, include_all=True, rate_hz=50):
        print(s["rpy_deg"], s["lin_pos_m"], s["lin_vel_ms"])

D) Only pick some fields
    keys = ("t_ms", "acc_g", "gyro_dps", "rpy_deg", "lin_pos_m")
    for s in iter_imu_samples(source="serial", port="/dev/ttyUSB0", integrator=dr, keys=keys, rate_hz=20):
        print(s)
"""

from __future__ import annotations
import math
import time
from typing import Dict, Iterator, Optional, Sequence, Tuple, Any


# --------------------
# constants
# --------------------
G = 9.80665
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

_SKIP_PREFIXES = (
    "t_ms,", "serial_ok", "Using SDA=", "ping_", "MPU found", "No MPU found",
    "Write PWR", "read_fail", "#"
)


# --------------------
# small vector helpers (no numpy dependency)
# --------------------
def v_add(a, b): return (a[0]+b[0], a[1]+b[1], a[2]+b[2])
def v_sub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def v_mul(s, a): return (s*a[0], s*a[1], s*a[2])
def v_norm(a): return math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])

def s_add(a, b):  # 6D state add
    return (a[0]+b[0], a[1]+b[1], a[2]+b[2], a[3]+b[3], a[4]+b[4], a[5]+b[5])

def s_mul(s, a):  # 6D state scale
    return (s*a[0], s*a[1], s*a[2], s*a[3], s*a[4], s*a[5])


# --------------------
# quaternion (x,y,z,w) helpers (same convention as your rk4 node)
# --------------------
def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    inv = 1.0 / n
    return (x*inv, y*inv, z*inv, w*inv)

def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )

def quat_rotate(q, v):
    # v' = q ⊗ [v,0] ⊗ conj(q)
    x, y, z, w = quat_normalize(q)
    vq = (v[0], v[1], v[2], 0.0)
    q_conj = (-x, -y, -z, w)
    out = quat_mul(quat_mul((x, y, z, w), vq), q_conj)
    return (out[0], out[1], out[2])

def quat_to_rpy(q):
    x, y, z, w = quat_normalize(q)

    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w*y - z*x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi/2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# --------------------
# Integrator (RK4)
# --------------------
class RK4DeadReckoner:
    """
    RK4 dead-reckoning integrator.

    State:
      - q_xyzw: orientation quaternion (x,y,z,w)
      - v: linear velocity in world [m/s]
      - p: linear position in world [m]

    Still detection & bias:
      - stationary if | |acc|-1g | < zupt_acc_g AND |gyro| < zupt_gyro_dps
      - if stationary: gyro_bias <- (1-a)*bias + a*gyro_raw
      - if stationary and enable_zupt: set v = 0
    """

    def __init__(
        self,
        *,
        gravity_world: Tuple[float, float, float] = (0.0, 0.0, 9.80665),
        acc_includes_gravity: bool = True,
        enable_zupt: bool = True,
        zupt_acc_g: float = 0.05,
        zupt_gyro_dps: float = 2.0,
        alpha_gyro_bias: float = 0.01,
        max_dt: float = 0.2,
        use_board_time_if_available: bool = True,
    ):
        self.gw = gravity_world
        self.acc_includes_gravity = acc_includes_gravity
        self.enable_zupt = enable_zupt
        self.zupt_acc_g = zupt_acc_g
        self.zupt_gyro_dps = zupt_gyro_dps
        self.alpha_bias = alpha_gyro_bias
        self.max_dt = max_dt
        self.use_board_time_if_available = use_board_time_if_available

        self.q = (0.0, 0.0, 0.0, 1.0)
        self.v = (0.0, 0.0, 0.0)
        self.p = (0.0, 0.0, 0.0)
        self.gyro_bias_rads = (0.0, 0.0, 0.0)

        self._t_prev: Optional[float] = None
        self._omega_prev_raw: Optional[Tuple[float, float, float]] = None
        self._a_prev_world: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def _q_dot(self, q, omega_rads):
        ox, oy, oz = omega_rads
        omega_q = (ox, oy, oz, 0.0)
        dq = quat_mul(q, omega_q)
        return (0.5*dq[0], 0.5*dq[1], 0.5*dq[2], 0.5*dq[3])

    def _integrate_quat_rk4(self, q0, omega0, omega1, dt):
        def omega_of_tau(tau):
            if dt <= 0:
                return omega1
            k = tau / dt
            return (
                omega0[0] + (omega1[0]-omega0[0])*k,
                omega0[1] + (omega1[1]-omega0[1])*k,
                omega0[2] + (omega1[2]-omega0[2])*k,
            )

        k1 = self._q_dot(q0, omega_of_tau(0.0))
        q1 = (q0[0] + 0.5*dt*k1[0], q0[1] + 0.5*dt*k1[1], q0[2] + 0.5*dt*k1[2], q0[3] + 0.5*dt*k1[3])

        k2 = self._q_dot(q1, omega_of_tau(0.5*dt))
        q2 = (q0[0] + 0.5*dt*k2[0], q0[1] + 0.5*dt*k2[1], q0[2] + 0.5*dt*k2[2], q0[3] + 0.5*dt*k2[3])

        k3 = self._q_dot(q2, omega_of_tau(0.5*dt))
        q3 = (q0[0] + dt*k3[0], q0[1] + dt*k3[1], q0[2] + dt*k3[2], q0[3] + dt*k3[3])

        k4 = self._q_dot(q3, omega_of_tau(dt))

        q_new = (
            q0[0] + (dt/6.0)*(k1[0] + 2*k2[0] + 2*k3[0] + k4[0]),
            q0[1] + (dt/6.0)*(k1[1] + 2*k2[1] + 2*k3[1] + k4[1]),
            q0[2] + (dt/6.0)*(k1[2] + 2*k2[2] + 2*k3[2] + k4[2]),
            q0[3] + (dt/6.0)*(k1[3] + 2*k2[3] + 2*k3[3] + k4[3]),
        )
        return quat_normalize(q_new)

    def _integrate_pv_rk4(self, p0, v0, a0, a1, dt):
        # state s = [p(3), v(3)]
        def a_of_tau(tau):
            if dt <= 0:
                return a1
            k = tau / dt
            return (
                a0[0] + (a1[0]-a0[0])*k,
                a0[1] + (a1[1]-a0[1])*k,
                a0[2] + (a1[2]-a0[2])*k,
            )

        def f(s, tau):
            v = (s[3], s[4], s[5])
            a = a_of_tau(tau)
            return (v[0], v[1], v[2], a[0], a[1], a[2])

        s0 = (p0[0], p0[1], p0[2], v0[0], v0[1], v0[2])
        k1 = f(s0, 0.0)
        k2 = f(s_add(s0, s_mul(0.5*dt, k1)), 0.5*dt)
        k3 = f(s_add(s0, s_mul(0.5*dt, k2)), 0.5*dt)
        k4 = f(s_add(s0, s_mul(dt, k3)), dt)

        s1 = s_add(s0, s_mul(dt/6.0, s_add(s_add(k1, s_mul(2.0, k2)), s_add(s_mul(2.0, k3), k4))))
        p1 = (s1[0], s1[1], s1[2])
        v1 = (s1[3], s1[4], s1[5])
        return p1, v1

    def update(self, sample: Dict[str, Any]) -> Dict[str, Any]:
        """
        Update integrator with one parsed sample dict and return extra fields.
        Requires at least: acc_ms2 or acc_g, gyro_rads or gyro_dps, and time (t_s or host_time_s).
        """
        # pick time
        t = None
        if self.use_board_time_if_available and (sample.get("t_s") is not None):
            t = float(sample["t_s"])
        elif sample.get("host_time_s") is not None:
            t = float(sample["host_time_s"])
        else:
            # last resort: current time
            t = time.time()

        # get acc / gyro in SI
        if sample.get("acc_ms2") is not None:
            acc_ms2 = tuple(sample["acc_ms2"])
        else:
            ax, ay, az = sample["acc_g"]
            acc_ms2 = (ax*G, ay*G, az*G)

        if sample.get("gyro_rads") is not None:
            gyro_rads_raw = tuple(sample["gyro_rads"])
        else:
            gx, gy, gz = sample["gyro_dps"]
            gyro_rads_raw = (gx*DEG2RAD, gy*DEG2RAD, gz*DEG2RAD)

        # still detection in g/dps space
        axg, ayg, azg = (acc_ms2[0]/G, acc_ms2[1]/G, acc_ms2[2]/G)
        gxd, gyd, gzd = (gyro_rads_raw[0]*RAD2DEG, gyro_rads_raw[1]*RAD2DEG, gyro_rads_raw[2]*RAD2DEG)
        a_mag_g = math.sqrt(axg*axg + ayg*ayg + azg*azg)
        gyro_mag_dps = math.sqrt(gxd*gxd + gyd*gyd + gzd*gzd)
        stationary = (abs(a_mag_g - 1.0) < self.zupt_acc_g) and (gyro_mag_dps < self.zupt_gyro_dps)

        # init
        if self._t_prev is None:
            self._t_prev = t
            self._omega_prev_raw = gyro_rads_raw
            self._a_prev_world = (0.0, 0.0, 0.0)
            return {
                "dt_s": None,
                "stationary": stationary,
                "q_xyzw": self.q,
                "rpy_rad": quat_to_rpy(self.q),
                "rpy_deg": tuple(a*RAD2DEG for a in quat_to_rpy(self.q)),
                "lin_vel_ms": self.v,
                "lin_pos_m": self.p,
                "acc_world_ms2": None,
                "acc_lin_world_ms2": None,
                "gyro_bias_rads": self.gyro_bias_rads,
            }

        dt = t - self._t_prev
        self._t_prev = t
        if dt <= 0.0 or dt > self.max_dt:
            # skip update but refresh prev omega
            self._omega_prev_raw = gyro_rads_raw
            return {"dt_s": dt, "stationary": stationary}

        # gyro bias update
        if stationary:
            bx, by, bz = self.gyro_bias_rads
            ox, oy, oz = gyro_rads_raw
            a = self.alpha_bias
            self.gyro_bias_rads = ((1-a)*bx + a*ox, (1-a)*by + a*oy, (1-a)*bz + a*oz)

        # bias-corrected omega
        omega = v_sub(gyro_rads_raw, self.gyro_bias_rads)
        omega0 = v_sub(self._omega_prev_raw, self.gyro_bias_rads) if self._omega_prev_raw is not None else omega
        omega1 = omega

        # attitude RK4
        self.q = self._integrate_quat_rk4(self.q, omega0, omega1, dt)

        # acc -> world
        acc_world = quat_rotate(self.q, acc_ms2)
        if self.acc_includes_gravity:
            a_lin_world = v_sub(acc_world, self.gw)
        else:
            a_lin_world = acc_world

        # p,v RK4
        if stationary and self.enable_zupt:
            self.v = (0.0, 0.0, 0.0)
        else:
            self.p, self.v = self._integrate_pv_rk4(self.p, self.v, self._a_prev_world, a_lin_world, dt)

        self._a_prev_world = a_lin_world
        self._omega_prev_raw = gyro_rads_raw

        rpy = quat_to_rpy(self.q)
        return {
            "dt_s": dt,
            "stationary": stationary,
            "q_xyzw": self.q,
            "rpy_rad": rpy,
            "rpy_deg": (rpy[0]*RAD2DEG, rpy[1]*RAD2DEG, rpy[2]*RAD2DEG),
            "lin_vel_ms": self.v,
            "lin_pos_m": self.p,
            "acc_world_ms2": acc_world,
            "acc_lin_world_ms2": a_lin_world,
            "gyro_bias_rads": self.gyro_bias_rads,
        }


# --------------------
# parsing
# --------------------
def _should_skip(line: str) -> bool:
    s = line.strip()
    if not s:
        return True
    return any(s.startswith(p) for p in _SKIP_PREFIXES)

def parse_arduino_imu_csv(line: str) -> Optional[Dict[str, Any]]:
    """
    Parse one Arduino/ESP32 CSV line:
      t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,temp_C
    temp_C optional. Returns None if header/status/invalid.
    """
    if _should_skip(line):
        return None
    parts = [p.strip() for p in line.strip().split(",")]
    if len(parts) < 9:
        return None
    try:
        t_ms = float(parts[0])
        ax_g, ay_g, az_g = float(parts[1]), float(parts[2]), float(parts[3])
        gx_dps, gy_dps, gz_dps = float(parts[4]), float(parts[5]), float(parts[6])
        roll_deg, pitch_deg = float(parts[7]), float(parts[8])
        temp_C = float(parts[9]) if len(parts) >= 10 else None

        acc_g = (ax_g, ay_g, az_g)
        gyro_dps = (gx_dps, gy_dps, gz_dps)
        acc_ms2 = (ax_g * G, ay_g * G, az_g * G)
        gyro_rads = (gx_dps * DEG2RAD, gy_dps * DEG2RAD, gz_dps * DEG2RAD)

        return {
            "source": "serial_csv",
            "t_ms": t_ms,
            "t_s": t_ms * 1e-3,
            "acc_g": acc_g,
            "gyro_dps": gyro_dps,
            "acc_ms2": acc_ms2,
            "gyro_rads": gyro_rads,
            "roll_deg": roll_deg,
            "pitch_deg": pitch_deg,
            "temp_C": temp_C,
            "acc_norm_g": v_norm(acc_g),
            "gyro_norm_dps": v_norm(gyro_dps),
            "raw": line.strip(),
        }
    except ValueError:
        return None

def _select_keys(full: Dict[str, Any], keys: Optional[Sequence[str]], include_all: bool) -> Dict[str, Any]:
    if include_all or keys is None:
        return full
    out: Dict[str, Any] = {}
    for k in keys:
        if k in full:
            out[k] = full[k]
    return out


# --------------------
# main generator
# --------------------
def iter_imu_samples(
    *,
    source: str = "serial",   # "serial" or "i2c"
    # serial params
    port: str = "/dev/ttyUSB0",
    baud: int = 115200,
    timeout: float = 1.0,
    # i2c params
    i2c_bus: int = 1,
    i2c_addr: int = 0x68,
    # output control
    keys: Optional[Sequence[str]] = ("acc_g", "gyro_dps"),
    include_all: bool = False,
    add_host_time: bool = True,
    # output rate
    rate_hz: Optional[float] = None,
    # integrator
    integrator: Optional[RK4DeadReckoner] = None,
) -> Iterator[Dict[str, Any]]:
    """
    Generator yielding IMU samples as dicts.

    If integrator is provided, integrated fields are merged into the sample dict.
    Use include_all=True (or keys=None) to see all merged fields.
    """
    source = source.lower().strip()
    if source not in ("serial", "i2c"):
        raise ValueError("source must be 'serial' or 'i2c'")

    period: Optional[float] = None
    if rate_hz is not None:
        if rate_hz <= 0:
            raise ValueError("rate_hz must be > 0 or None")
        period = 1.0 / float(rate_hz)

    def should_emit(now_s: float, next_emit_s: Optional[float]) -> Tuple[bool, Optional[float]]:
        if period is None:
            return True, next_emit_s
        if next_emit_s is None:
            return True, now_s + period
        if now_s >= next_emit_s:
            return True, now_s + period
        return False, next_emit_s

    if source == "serial":
        import serial  # pyserial
        ser = serial.Serial(port, baud, timeout=timeout)
        next_emit_s: Optional[float] = None
        try:
            time.sleep(0.5)
            try:
                ser.reset_input_buffer()
            except Exception:
                pass

            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                full = parse_arduino_imu_csv(line)
                if full is None:
                    continue

                now = time.time()
                emit, next_emit_s = should_emit(now, next_emit_s)
                if not emit:
                    continue

                if add_host_time:
                    full["host_time_s"] = now

                if integrator is not None:
                    full.update(integrator.update(full))

                yield _select_keys(full, keys, include_all)
        finally:
            try:
                ser.close()
            except Exception:
                pass

    else:
        from imu_i2c_reader import MPU6050Reader  # needs local file
        reader = MPU6050Reader(bus_id=i2c_bus, addr=i2c_addr)
        next_emit_s: Optional[float] = None
        try:
            while True:
                if period is not None:
                    now = time.time()
                    if next_emit_s is None:
                        next_emit_s = now
                    sleep_s = next_emit_s - now
                    if sleep_s > 0:
                        time.sleep(sleep_s)

                acc_ms2, gyro_rads = reader.read()

                ax, ay, az = acc_ms2
                gx, gy, gz = gyro_rads

                acc_g = (ax / G, ay / G, az / G)
                gyro_dps = (gx * RAD2DEG, gy * RAD2DEG, gz * RAD2DEG)

                now = time.time()
                if period is not None:
                    next_emit_s = now + period

                full: Dict[str, Any] = {
                    "source": "i2c",
                    "t_s": now,          # host time
                    "t_ms": None,
                    "host_time_s": now if add_host_time else None,
                    "acc_ms2": (ax, ay, az),
                    "gyro_rads": (gx, gy, gz),
                    "acc_g": acc_g,
                    "gyro_dps": gyro_dps,
                    "roll_deg": None,
                    "pitch_deg": None,
                    "temp_C": None,
                    "acc_norm_g": v_norm(acc_g),
                    "gyro_norm_dps": v_norm(gyro_dps),
                    "raw": None,
                }

                if integrator is not None:
                    full.update(integrator.update(full))

                yield _select_keys(full, keys, include_all)
        finally:
            try:
                reader.close()
            except Exception:
                pass


# optional quick demo
if __name__ == "__main__":
    PORT = "/dev/ttyUSB0"  # Windows: "COM5"
    dr = RK4DeadReckoner(gravity_world=(0.0, 0.0, 9.80665))
    gen = iter_imu_samples(source="serial", port=PORT, rate_hz=50, integrator=dr, include_all=True)
    for i, s in zip(range(10), gen):
        print(i, s["acc_g"], s["gyro_dps"], s.get("rpy_deg"), s.get("lin_pos_m"))
