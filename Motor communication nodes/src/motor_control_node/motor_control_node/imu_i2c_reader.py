import math
from smbus2 import SMBus


class MPU6050Reader:

    # MPU6050 regs
    WHO_AM_I = 0x75
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43

    ACC_LSB_PER_G = 16384.0   # ±2g
    GYRO_LSB_PER_DPS = 131.0  # ±250 dps
    G = 9.80665

    def __init__(self, bus_id: int = 1, addr: int = 0x68):
        self.bus_id = int(bus_id)
        self.addr = int(addr)
        self.bus = SMBus(self.bus_id)

        # wake up
        self.bus.write_byte_data(self.addr, self.PWR_MGMT_1, 0x00)
        
        try:
            who = self.bus.read_byte_data(self.addr, self.WHO_AM_I)
            self._whoami = who
        except Exception:
            self._whoami = None

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass

    def _read_i16(self, reg: int) -> int:
        hi = self.bus.read_byte_data(self.addr, reg)
        lo = self.bus.read_byte_data(self.addr, reg + 1)
        val = (hi << 8) | lo
        return val - 65536 if (val & 0x8000) else val

    def read(self):
        """
        Returns:
            acc_body: (ax,ay,az) m/s^2
            omega_body: (wx,wy,wz) rad/s
        """
        ax = self._read_i16(self.ACCEL_XOUT_H) / self.ACC_LSB_PER_G * self.G
        ay = self._read_i16(self.ACCEL_XOUT_H + 2) / self.ACC_LSB_PER_G * self.G
        az = self._read_i16(self.ACCEL_XOUT_H + 4) / self.ACC_LSB_PER_G * self.G

        gx = self._read_i16(self.GYRO_XOUT_H) / self.GYRO_LSB_PER_DPS * (math.pi / 180.0)
        gy = self._read_i16(self.GYRO_XOUT_H + 2) / self.GYRO_LSB_PER_DPS * (math.pi / 180.0)
        gz = self._read_i16(self.GYRO_XOUT_H + 4) / self.GYRO_LSB_PER_DPS * (math.pi / 180.0)

        return (ax, ay, az), (gx, gy, gz)
