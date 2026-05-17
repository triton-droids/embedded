import math
import time
import struct
from typing import Optional, Tuple
import socket
#pip install python-can
#import can

# ------------------------- Constants from C++ code -------------------------
Set_mode      = ord('j')  # select control mode
Set_parameter = ord('p')  # set parameter

# control modes
move_control_mode   = 0  # motion control mode
PosPP_control_mode  = 1  # position mode (PP)
Speed_control_mode  = 2  # speed mode
Elect_control_mode  = 3  # current mode
Set_Zero_mode       = 4  # zero mode
PosCSP_control_mode = 5  # position mode (CSP)

SC_MAX   = 23.0
SC_MIN   = 0.0
SV_MAX   = 20.0
SV_MIN   = -20.0
SCIQ_MIN = -23.0

Communication_Type_Get_ID             = 0x00
Communication_Type_MotionControl      = 0x01
Communication_Type_MotorRequest       = 0x02
Communication_Type_MotorEnable        = 0x03
Communication_Type_MotorStop          = 0x04
Communication_Type_SetPosZero         = 0x06
Communication_Type_Can_ID             = 0x07
Communication_Type_GetSingleParameter = 0x11
Communication_Type_SetSingleParameter = 0x12
Communication_Type_ErrorFeedback      = 0x15


CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
#can_id (4) + dlc (1) + padding (3) + data (8)
CAN_FRAME_FMT = "=IB3x8s"

ACTUATOR_OPERATION_MAPPING = {
    0: {"position": 4 * math.pi, "velocity": 50, "torque": 17,  "kp": 500.0,  "kd": 5.0},
    1: {"position": 4 * math.pi, "velocity": 44, "torque": 17,  "kp": 500.0,  "kd": 5.0},
    2: {"position": 4 * math.pi, "velocity": 44, "torque": 17,  "kp": 500.0,  "kd": 5.0},
    3: {"position": 4 * math.pi, "velocity": 50, "torque": 60,  "kp": 5000.0, "kd": 100.0},
    4: {"position": 4 * math.pi, "velocity": 15, "torque": 120, "kp": 5000.0, "kd": 100.0},
    5: {"position": 4 * math.pi, "velocity": 33, "torque": 17,  "kp": 500.0,  "kd": 5.0},
    6: {"position": 4 * math.pi, "velocity": 20, "torque": 60,  "kp": 5000.0, "kd": 100.0},
}

INDEX_LIST = [
    0x7005, 0x7006, 0x700A, 0x700B, 0x7010, 0x7011, 0x7014,
    0x7016, 0x7017, 0x7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D,
]

class DataReadWriteOne:
    """Python equivalent of data_read_write_one (index + data)."""

    def __init__(self, index: int):
        self.index = index
        self.data = 0.0  # can store float or int
        
class DataReadWrite:
    """Python equivalent of data_read_write, with all parameters."""

    def __init__(self, index_list=INDEX_LIST):
        self.run_mode      = DataReadWriteOne(index_list[0])
        self.iq_ref        = DataReadWriteOne(index_list[1])
        self.spd_ref       = DataReadWriteOne(index_list[2])
        self.imit_torque   = DataReadWriteOne(index_list[3])
        self.cur_kp        = DataReadWriteOne(index_list[4])
        self.cur_ki        = DataReadWriteOne(index_list[5])
        self.cur_filt_gain = DataReadWriteOne(index_list[6])
        self.loc_ref       = DataReadWriteOne(index_list[7])
        self.limit_spd     = DataReadWriteOne(index_list[8])
        self.limit_cur     = DataReadWriteOne(index_list[9])
        # read-only fields
        self.mechPos       = DataReadWriteOne(index_list[10])
        self.iqf           = DataReadWriteOne(index_list[11])
        self.mechVel       = DataReadWriteOne(index_list[12])
        self.VBUS          = DataReadWriteOne(index_list[13])
        self.rotation      = DataReadWriteOne(index_list[14])


class MotorSet:
    """Python equivalent of Motor_Set struct."""

    def __init__(self):
        self.set_motor_mode = 0
        self.set_current    = 0.0
        self.set_speed      = 0.0
        self.set_Torque     = 0.0
        self.set_angle      = 0.0
        self.set_limit_cur  = 0.0
        self.set_Kp         = 0.0
        self.set_Ki         = 0.0
        self.set_Kd         = 0.0
        self.set_iq         = 0.0
        self.set_id         = 0.0
        self.set_acc        = 0.0

class RobStrideMotorLinux:
    """
    RobStride motor driver using Linux raw CAN sockets (PF_CAN, CAN_RAW).

    This class is a Python translation of the core logic in motor_cfg.cpp:
    - socket initialization with filter on motor_id (bits 8..15 + extended flag)
    - sending and receiving extended CAN frames
    - decoding motor status (position, velocity, torque, temperature)
    - basic commands: enable/disable, parameter set/get, motion control, velocity command.
    """

    def __init__(self, iface: str, master_id: int, motor_id: int, actuator_type: int):
        """
        :param iface: CAN interface name, e.g. "can0"
        :param master_id: master ID (host ID), e.g. 0xFF
        :param motor_id: motor CAN ID (0x01, 0xFD, etc.)
        :param actuator_type: 0..6, used to look up position/velocity/torque limits.
        """
        self.iface = iface
        self.master_id = master_id & 0xFF
        self.motor_id = motor_id & 0xFF
        self.actuator_type = actuator_type
        self.lims = ACTUATOR_OPERATION_MAPPING[actuator_type]

        self.sock: Optional[socket.socket] = None
        self._init_socket()

        # feedback state
        self.position = 0.0
        self.velocity = 0.0
        self.torque   = 0.0
        self.temperature = 0.0

        # CAN ID flags decoded from status frames
        self.error_code = 0
        self.pattern = 0

        # parameter storage
        self.drw = DataReadWrite()
        self.params = DataReadWriteOne(0x7005)
        self.motor_set_all = MotorSet()

    # ---------------------------------------------------------------------
    # Socket initialization (translation of C++ init_socket)
    # ---------------------------------------------------------------------

    def _init_socket(self):
        """Open PF_CAN, SOCK_RAW, CAN_RAW socket and set filter on motor_id."""
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)

        # Bind to interface by name (Python handles ifindex internally).
        self.sock.bind((self.iface,))

        # Build CAN filter to match frames where:
        # - CAN ID has extended flag
        # - bits 8..15 match motor_id
        #
        # In C++:
        #   rfilter[0].can_id   = (motor_id << 8) | CAN_EFF_FLAG;
        #   rfilter[0].can_mask = (0xFF << 8) | CAN_EFF_FLAG;
        #
        # struct can_filter { canid_t can_id; canid_t can_mask; };
        can_id = (self.motor_id << 8) | CAN_EFF_FLAG
        can_mask = (0xFF << 8) | CAN_EFF_FLAG
        can_filter_struct = struct.pack("=II", can_id, can_mask)

        # On Python, these constants are provided by the socket module
        # as SOL_CAN_RAW and CAN_RAW_FILTER.
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, can_filter_struct)

    # ---------------------------------------------------------------------
    # Low-level helpers: float <-> uint, bytes -> float
    # ---------------------------------------------------------------------

    @staticmethod
    def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
        """Map a float into an unsigned integer range [0, 2^bits - 1]."""
        if x < x_min:
            x = x_min
        if x > x_max:
            x = x_max
        span = x_max - x_min
        offset = x - x_min
        return int((offset * ((1 << bits) - 1)) / span)

    @staticmethod
    def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
        """Map an unsigned integer back into a float range [x_min, x_max]."""
        span = x_max - x_min
        return x_int * span / ((1 << bits) - 1) + x_min

    @staticmethod
    def byte_to_float_from_payload(payload: bytes) -> float:
        """
        Reconstruct float from data[4..7] (little-endian), equivalent to C++ Byte_to_float.

        C++:
            uint32_t data = bytedata[7]<<24 | bytedata[6]<<16 | bytedata[5]<<8 | bytedata[4];
            float data_float = *(float*)(&data);
        """
        if len(payload) < 8:
            return 0.0
        d = list(payload)
        raw = (d[7] << 24) | (d[6] << 16) | (d[5] << 8) | d[4]
        return struct.unpack("<f", struct.pack("<I", raw))[0]

    # ---------------------------------------------------------------------
    # Low-level CAN frame send / receive
    # ---------------------------------------------------------------------

    def _send_frame(self, can_id_no_flag: int, data: bytes):
        """
        Send a single CAN frame.
        :param can_id_no_flag: 29-bit CAN ID, without CAN_EFF_FLAG. This function adds the flag.
        :param data: up to 8 bytes of payload.
        """
        if self.sock is None:
            raise RuntimeError("Socket is not initialized")

        if len(data) > 8:
            raise ValueError("CAN data length must be <= 8 bytes")

        data_padded = data.ljust(8, b"\x00")
        dlc = len(data)

        frame = struct.pack(CAN_FRAME_FMT, can_id_no_flag | CAN_EFF_FLAG, dlc, data_padded)
        self.sock.send(frame)

    def _recv_frame(self, timeout: float = 0.1) -> Optional[Tuple[int, int, bytes]]:
        """
        Receive a single CAN frame, with optional timeout.
        :return: (can_id_no_flag, dlc, data_bytes) or None on timeout.
        """
        if self.sock is None:
            raise RuntimeError("Socket is not initialized")

        self.sock.settimeout(timeout if timeout > 0 else None)
        try:
            frame = self.sock.recv(16)  # sizeof(struct can_frame) is 16 bytes
        except socket.timeout:
            return None

        can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
        if not (can_id & CAN_EFF_FLAG):
            # Only handle extended frames as in C++ code
            return None

        can_id_no_flag = can_id & CAN_EFF_MASK
        return can_id_no_flag, dlc, data[:dlc]

    # ---------------------------------------------------------------------
    # Status frame decoding (translation of receive_status_frame)
    # ---------------------------------------------------------------------

    def receive_status_frame(self, timeout: float = 0.1):
        """
        Receive and decode a single status frame.

        - If communication_type == MotorRequest (0x02):
          decode position, velocity, torque, temperature.
        - If communication_type == 0x11 (GetSingleParameter response):
          decode parameter values into self.drw.
        """
        result = self._recv_frame(timeout)
        if result is None:
            raise RuntimeError("No frame received (timeout or non-extended frame)")

        can_id, dlc, data = result
        if dlc < 8:
            raise RuntimeError("Data size too small")

        communication_type = (can_id >> 24) & 0x1F
        extra_data = (can_id >> 8) & 0xFFFF
        host_id = can_id & 0xFF

        self.error_code = (can_id >> 16) & 0x3F
        self.pattern = (can_id >> 22) & 0x03

        print(f"communication_type: {communication_type}")

        if communication_type == Communication_Type_MotorRequest:
            # decode pvtt
            pos_u16 = (data[0] << 8) | data[1]
            vel_u16 = (data[2] << 8) | data[3]
            tor_u16 = (data[4] << 8) | data[5]
            temp_u16 = (data[6] << 8) | data[7]

            self.position = ((pos_u16 / 32767.0) - 1.0) * self.lims["position"]
            self.velocity = ((vel_u16 / 32767.0) - 1.0) * self.lims["velocity"]
            self.torque   = ((tor_u16 / 32767.0) - 1.0) * self.lims["torque"]
            self.temperature = temp_u16 * 0.1

        elif communication_type == Communication_Type_GetSingleParameter:
            # parameter feedback
            self.params.data = data[4]
            self.params.index = 0x7005

            index_val = (data[1] << 8) | data[0]
            for idx, idx_code in enumerate(INDEX_LIST[:14]):
                if index_val == idx_code:
                    if idx == 0:
                        self.drw.run_mode.data = data[4]
                        print("mode data:", int(data[4]))
                    elif idx == 1:
                        self.drw.iq_ref.data = self.byte_to_float_from_payload(data)
                    elif idx == 2:
                        self.drw.spd_ref.data = self.byte_to_float_from_payload(data)
                    elif idx == 3:
                        self.drw.imit_torque.data = self.byte_to_float_from_payload(data)
                    elif idx == 4:
                        self.drw.cur_kp.data = self.byte_to_float_from_payload(data)
                    elif idx == 5:
                        self.drw.cur_ki.data = self.byte_to_float_from_payload(data)
                    elif idx == 6:
                        self.drw.cur_filt_gain.data = self.byte_to_float_from_payload(data)
                    elif idx == 7:
                        self.drw.loc_ref.data = self.byte_to_float_from_payload(data)
                    elif idx == 8:
                        self.drw.limit_spd.data = self.byte_to_float_from_payload(data)
                    elif idx == 9:
                        self.drw.limit_cur.data = self.byte_to_float_from_payload(data)
                    elif idx == 10:
                        self.drw.mechPos.data = self.byte_to_float_from_payload(data)
                    elif idx == 11:
                        self.drw.iqf.data = self.byte_to_float_from_payload(data)
                    elif idx == 12:
                        self.drw.mechVel.data = self.byte_to_float_from_payload(data)
                    elif idx == 13:
                        self.drw.VBUS.data = self.byte_to_float_from_payload(data)
                    break
        else:
            raise RuntimeError("Invalid communication type")

    # ---------------------------------------------------------------------
    # Parameter set / get (Set_RobStrite_Motor_parameter / Get_...)
    # ---------------------------------------------------------------------

    def set_motor_parameter(self, index: int, value: float, value_mode: str):
        """
        Set motor parameter (single index).

        value_mode:
            'p' -> write float to data[4..7]
            'j' -> write mode (uint8) to data[4], remaining bytes are zero.
        """
        can_id = (Communication_Type_SetSingleParameter << 24) | (self.master_id << 8) | self.motor_id

        data0 = index & 0xFF
        data1 = (index >> 8) & 0xFF
        data2 = 0x00
        data3 = 0x00

        if value_mode == 'p':
            f_bytes = struct.pack("<f", float(value))
            data4, data5, data6, data7 = f_bytes
        elif value_mode == 'j':
            data4 = int(value) & 0xFF
            data5 = data6 = data7 = 0x00
        else:
            raise ValueError("value_mode must be 'p' or 'j'")

        payload = bytes([data0, data1, data2, data3, data4, data5, data6, data7])
        self._send_frame(can_id, payload)
        print("[✓] Motor set parameter command sent.")
        self.receive_status_frame()

    def get_motor_parameter(self, index: int):
        """
        Request a single motor parameter by index.
        Response will be handled in receive_status_frame and stored in self.drw.
        """
        can_id = (Communication_Type_GetSingleParameter << 24) | (self.master_id << 8) | self.motor_id
        payload = bytes([
            index & 0xFF,
            (index >> 8) & 0xFF,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        ])
        self._send_frame(can_id, payload)
        print("execute Get_motor_params")
        self.receive_status_frame()

        # ---------------------------------------------------------------------
    # Mode management helper (new): ensure the motor is in a given control mode
    # ---------------------------------------------------------------------

    def ensure_mode(self, target_mode: int):
        """
        Ensure the motor is in the desired control mode (target_mode).

        This is the Python equivalent of the C++ pattern:
            - Get current run_mode from index 0x7005
            - If run_mode != target_mode AND pattern == 2:
                - disable motor
                - set index 0x7005 to target_mode (mode select)
                - get index 0x7005 again to confirm
                - enable motor

        target_mode should be one of:
            move_control_mode, PosPP_control_mode, Speed_control_mode,
            Elect_control_mode, Set_Zero_mode, PosCSP_control_mode.
        """
        # Read current run_mode (0x7005)
        self.get_motor_parameter(0x7005)

        # drw.run_mode.data is updated in receive_status_frame() for communication_type == 0x11
        current_mode = int(self.drw.run_mode.data) if isinstance(self.drw.run_mode.data, (int, float)) else 0

        # pattern == 2 in your C++ code is used as an additional condition
        if current_mode != target_mode and self.pattern == 2:
            # Disable motor first
            self.disable_motor(clear_error=0)
            time.sleep(0.001)

            # Set new mode (index 0x7005, mode written as uint8)
            self.set_motor_parameter(0x7005, target_mode, 'j')
            time.sleep(0.001)

            # Read back to confirm
            self.get_motor_parameter(0x7005)
            time.sleep(0.001)

            # Enable motor again
            self.enable_motor()
            time.sleep(0.001)

    # ---------------------------------------------------------------------
    # Enable / disable motor
    # ---------------------------------------------------------------------

    def enable_motor(self) -> Tuple[float, float, float, float]:
        """Send motor enable command and read one status frame."""
        can_id = (Communication_Type_MotorEnable << 24) | (self.master_id << 8) | self.motor_id
        self._send_frame(can_id, b"\x00" * 8)
        print("[✓] Motor enable command sent.")
        self.receive_status_frame()
        return self.position, self.velocity, self.torque, self.temperature

    def disable_motor(self, clear_error: int = 0) -> Tuple[float, float, float, float]:
        """Send motor stop command (disable) and read one status frame."""
        can_id = (Communication_Type_MotorStop << 24) | (self.master_id << 8) | self.motor_id
        payload = bytes([clear_error & 0xFF]) + b"\x00" * 7
        self._send_frame(can_id, payload)
        print("[✓] Motor disable command sent.")
        self.receive_status_frame()
        return self.position, self.velocity, self.torque, self.temperature

    # ---------------------------------------------------------------------
    # Motion control (send_motion_command)
    # ---------------------------------------------------------------------

    def send_motion_command(
        self,
        torque: float,
        position_rad: float,
        velocity_rad_s: float,
        kp: float = 0.5,
        kd: float = 0.1,
    ) -> Tuple[float, float, float, float]:
        """
        Send motion control command (torque in CAN ID, pos/vel/Kp/Kd in payload).

        This is a direct translation of C++ send_motion_command:
            can_id = (Type << 24) | (torque_u16 << 8) | motor_id;
            data[0..1] = pos_u16
            data[2..3] = vel_u16
            data[4..5] = kp_u16
            data[6..7] = kd_u16
        """
        tor_u16 = self.float_to_uint(
            torque,
            -self.lims["torque"],
            self.lims["torque"],
            16,
        )
        pos_u16 = self.float_to_uint(
            position_rad,
            -self.lims["position"],
            self.lims["position"],
            16,
        )
        vel_u16 = self.float_to_uint(
            velocity_rad_s,
            -self.lims["velocity"],
            self.lims["velocity"],
            16,
        )
        kp_u = self.float_to_uint(kp, 0.0, self.lims["kp"], 16)
        kd_u = self.float_to_uint(kd, 0.0, self.lims["kd"], 16)

        can_id = (
            (Communication_Type_MotionControl << 24)
            | (tor_u16 << 8)
            | self.motor_id
        )

        payload = bytes([
            (pos_u16 >> 8) & 0xFF, pos_u16 & 0xFF,
            (vel_u16 >> 8) & 0xFF, vel_u16 & 0xFF,
            (kp_u   >> 8) & 0xFF, kp_u   & 0xFF,
            (kd_u   >> 8) & 0xFF, kd_u   & 0xFF,
        ])

        self._send_frame(can_id, payload)
        self.receive_status_frame()
        return self.position, self.velocity, self.torque, self.temperature

    # ---------------------------------------------------------------------
    # Velocity mode (simplified: just write speed command to 0x700A)
    # ---------------------------------------------------------------------

    def send_velocity_mode_command(
        self,
        velocity_rad_s: float,
        acceleration_rad_s2: float = None, # type: ignore
    ) -> Tuple[float, float, float, float]:
        """
        Full Python equivalent of C++:
            std::tuple<float,...> RobStrideMotor::send_velocity_mode_command(float velocity_rad_s)

        C++ logic (simplified):
            if (run_mode != 2 && pattern == 2) {
                Disenable_Motor(0);
                usleep(1000);
                Set_RobStrite_Motor_parameter(0x7005, Speed_control_mode, 'j');
                usleep(1000);
                Get_RobStrite_Motor_parameter(0x7005);
                usleep(1000);
                enable_motor();
                Set_RobStrite_Motor_parameter(0x7018, 27.0f, 'p');       // speed limit
                usleep(1000);
                Set_RobStrite_Motor_parameter(0x7026, set_acc, 'p');     // acceleration
                usleep(1000);
            }
            Set_RobStrite_Motor_parameter(0x700A, velocity_rad_s, 'p');

        Parameters
        ----------
        velocity_rad_s : float
            Target velocity in rad/s.
        acceleration_rad_s2 : float, optional
            Desired acceleration (used for index 0x7026). If None, the previous
            self.motor_set_all.set_acc is used. If that is still zero, no accel
            parameter is written.
        """
        # Update stored acceleration if provided by user
        if acceleration_rad_s2 is not None:
            self.motor_set_all.set_acc = float(acceleration_rad_s2)

        # Ensure we are in speed control mode (mode = 2)
        self.ensure_mode(Speed_control_mode)

        # After ensure_mode(), we are enabled and in speed mode. Now match the C++ behavior.
        # Set speed limit (index 0x7018) and acceleration (index 0x7026) if available.
        # In C++, speed limit is hard-coded to 27.0f.
        try:
            self.set_motor_parameter(0x7018, 27.0, 'p')
            time.sleep(0.001)
        except Exception as e:
            print("[!] Failed to set speed limit (0x7018):", e)

        if self.motor_set_all.set_acc != 0.0:
            try:
                self.set_motor_parameter(0x7026, self.motor_set_all.set_acc, 'p')
                time.sleep(0.001)
            except Exception as e:
                print("[!] Failed to set acceleration (0x7026):", e)

        print("execute vel_mode")
        # Finally, write the actual velocity command to 0x700A
        self.set_motor_parameter(0x700A, velocity_rad_s, 'p')
        print("finish")

        return self.position, self.velocity, self.torque, self.temperature
    
    # ---------------------------------------------------------------------
    # Position mode (PP): RobStrite_Motor_PosPP_control
    # ---------------------------------------------------------------------

    def pos_pp_control(
        self,
        speed_rad_s: float,
        acceleration_rad_s2: float,
        angle_rad: float,
    ) -> Tuple[float, float, float, float]:
        """
        Python equivalent of:
            std::tuple<float,...> RobStride_Motor_PosPP_control(float Speed, float Acceleration, float Angle)

        Logic in C++:
            if (run_mode != 1 && pattern == 2) {
                Disenable_Motor(0);
                usleep(1000);
                Set_RobStrite_Motor_parameter(0x7005, PosPP_control_mode, 'j');
                usleep(1000);
                Get_RobStrite_Motor_parameter(0x7005);
                usleep(1000);
                enable_motor();
                usleep(1000);
            }

            Motor_Set_All.set_speed = Speed;
            Motor_Set_All.set_acc   = Acceleration;
            Motor_Set_All.set_angle = Angle;

            Set_RobStrite_Motor_parameter(0x7025, set_speed, 'p');
            usleep(1000);
            Set_RobStrite_Motor_parameter(0x7026, set_acc,   'p');
            usleep(1000);
            Set_RobStrite_Motor_parameter(0x7016, set_angle, 'p');
            usleep(1000);

        Parameters
        ----------
        speed_rad_s : float
            Target speed limit for PP mode (index 0x7025).
        acceleration_rad_s2 : float
            Acceleration setting (index 0x7026).
        angle_rad : float
            Target position (index 0x7016), in radians.
        """

        # Store values in motor_set_all (for consistency with C++ structure)
        self.motor_set_all.set_speed = float(speed_rad_s)
        self.motor_set_all.set_acc   = float(acceleration_rad_s2)
        self.motor_set_all.set_angle = float(angle_rad)

        # Ensure we are in position PP control mode (mode = 1)
        self.ensure_mode(PosPP_control_mode)

        # Now send the parameters just like in C++
        # 0x7025: speed setting for position mode
        self.set_motor_parameter(0x7025, self.motor_set_all.set_speed, 'p')
        time.sleep(0.001)

        # 0x7026: acceleration setting
        self.set_motor_parameter(0x7026, self.motor_set_all.set_acc, 'p')
        time.sleep(0.001)

        # 0x7016: target position (angle)
        self.set_motor_parameter(0x7016, self.motor_set_all.set_angle, 'p')
        time.sleep(0.001)

        return self.position, self.velocity, self.torque, self.temperature
        
    # ---------------------------------------------------------------------
    # Current mode: RobStrite_Motor_Current_control (Iq / Id)
    # ---------------------------------------------------------------------

    def current_control(
        self,
        iq_command: float,
        id_command: float,
    ) -> Tuple[float, float, float, float]:
        """
        Python equivalent of:
            std::tuple<float,...> RobStrite_Motor_Current_control(float IqCommand, float IdCommand)

        C++ logic (simplified):

            if (drw.run_mode.data != 3) {
                Disenable_Motor(0);
                usleep(1000);
                Set_RobStrite_Motor_parameter(0x7005, Elect_control_mode, 'j');
                usleep(1000);
                Get_RobStrite_Motor_parameter(0x7005);
                usleep(1000);
                enable_motor();
                usleep(1000);
            }

            Motor_Set_All.set_iq = IqCommand;
            Motor_Set_All.set_id = IdCommand;

            Motor_Set_All.set_iq = float_to_uint(set_iq, SCIQ_MIN, SC_MAX, 16);
            Set_RobStrite_Motor_parameter(0x7006, set_iq, 'p');
            usleep(1000);

            Set_RobStrite_Motor_parameter(0x7007, set_id, 'p');
            usleep(1000);

        Note:
            The original C++ code converts Iq to a uint16 (0..65535) using SCIQ_MIN = -23, SC_MAX = +23,
            and then sends it as a float via 'p'. We keep the same behavior here for consistency, although
            it is a bit unusual.
        """

        # Store target currents
        self.motor_set_all.set_iq = float(iq_command)
        self.motor_set_all.set_id = float(id_command)

        # Ensure we are in current control mode (mode = 3)
        self.ensure_mode(Elect_control_mode)

        # Map Iq from [-23, +23] A into [0, 65535] and then send that integral value as float,
        # just like the C++ implementation.
        iq_u16 = self.float_to_uint(
            self.motor_set_all.set_iq,
            SCIQ_MIN,
            SC_MAX,
            16,
        )
        self.motor_set_all.set_iq = float(iq_u16)

        # 0x7006: Iq command
        self.set_motor_parameter(0x7006, self.motor_set_all.set_iq, 'p')
        time.sleep(0.001)

        # 0x7007: Id command (sent directly as float)
        self.set_motor_parameter(0x7007, self.motor_set_all.set_id, 'p')
        time.sleep(0.001)

        return self.position, self.velocity, self.torque, self.temperature
    
    # ---------------------------------------------------------------------
    # Position mode (CSP): RobStrite_Motor_PosCSP_control
    # ---------------------------------------------------------------------

    def pos_csp_control(
        self,
        speed_rad_s: float,
        angle_rad: float,
    ) -> Tuple[float, float, float, float]:
        """
        Python equivalent of:
            std::tuple<float,...> RobStrite_Motor_PosCSP_control(float Speed, float Angle)

        Original C++ logic:

            Motor_Set_All.set_speed = Speed;
            Motor_Set_All.set_angle = Angle;
            if (drw.run_mode.data != 5 && pattern == 2) {
                Disenable_Motor(0);
                usleep(1000);
                Set_RobStrite_Motor_parameter(0x7005, PosCSP_control_mode, 'j');
                usleep(1000);
                Get_RobStrite_Motor_parameter(0x7005);
                usleep(1000);
                enable_motor();
                usleep(1000);
                Motor_Set_All.set_motor_mode = PosCSP_control_mode;
            }
            Motor_Set_All.set_speed = float_to_uint(Speed, -vel_max, vel_max, 16);
            Set_RobStrite_Motor_parameter(0x7017, set_speed, 'p');
            usleep(1000);
            Set_RobStrite_Motor_parameter(0x7016, set_angle, 'p');
            usleep(1000);

        Parameters
        ----------
        speed_rad_s : float
            Target speed used as CSP velocity limit (index 0x7017).
        angle_rad : float
            Target position in radians (index 0x7016).
        """

        # Store values into MotorSet for consistency
        self.motor_set_all.set_speed = float(speed_rad_s)
        self.motor_set_all.set_angle = float(angle_rad)

        # Ensure CSP mode (mode = 5)
        self.ensure_mode(PosCSP_control_mode)

        # Convert speed to 16-bit representation, as in the C++ code
        speed_u16 = self.float_to_uint(
            self.motor_set_all.set_speed,
            -self.lims["velocity"],
            self.lims["velocity"],
            16,
        )
        self.motor_set_all.set_speed = float(speed_u16)

        # 0x7017: CSP speed
        self.set_motor_parameter(0x7017, self.motor_set_all.set_speed, 'p')
        time.sleep(0.001)

        # 0x7016: CSP target position
        self.set_motor_parameter(0x7016, self.motor_set_all.set_angle, 'p')
        time.sleep(0.001)

        return self.position, self.velocity, self.torque, self.temperature

    # ---------------------------------------------------------------------
    # Zero mode (set control mode to "Set_Zero_mode")
    # ---------------------------------------------------------------------

    def set_zero_mode(self):
        """
        Python equivalent of:
            void RobStrite_Motor_Set_Zero_control()
            {
                Set_RobStrite_Motor_parameter(0x7005, Set_Zero_mode, Set_mode);
            }

        This only changes the control mode (run_mode = 4), it does NOT send the
        "SetPosZero" command frame. For actually setting the zero position in
        firmware, use set_zero_position().
        """
        self.set_motor_parameter(0x7005, Set_Zero_mode, 'j')
        time.sleep(0.001)
        
    # ---------------------------------------------------------------------
    # Zero position: SetPosZero command (Set_ZeroPos in C++)
    # ---------------------------------------------------------------------

    def set_zero_position(self):
        """
        Python equivalent of:
            void RobStrideMotor::Set_ZeroPos()

        C++ logic:

            Disenable_Motor(0);
            if (drw.run_mode.data != 4) {
                Set_RobStrite_Motor_parameter(0x7005, Speed_control_mode, 'j');
                usleep(1000);
                Get_RobStrite_Motor_parameter(0x7005);
                usleep(1000);
            }
            frame.can_id = (SetPosZero << 24) | (master_id << 8) | motor_id;
            data[0] = 1; others = 0;
            write(frame);
            enable_motor();

        This sends a special "SetPosZero" command and then re-enables the motor.
        """

        # Disable motor first, as in C++ code
        self.disable_motor(clear_error=0)

        # If current run_mode is not 4, set mode to SPEED (this matches the C++ code exactly)
        self.get_motor_parameter(0x7005)
        current_mode = int(self.drw.run_mode.data) if isinstance(self.drw.run_mode.data, (int, float)) else 0
        if current_mode != Set_Zero_mode:
            self.set_motor_parameter(0x7005, Speed_control_mode, 'j')
            time.sleep(0.001)
            self.get_motor_parameter(0x7005)
            time.sleep(0.001)

        # Build SetPosZero frame
        can_id = (Communication_Type_SetPosZero << 24) | (self.master_id << 8) | self.motor_id
        payload = bytes([1, 0, 0, 0, 0, 0, 0, 0])
        self._send_frame(can_id, payload)
        print("[✓] Motor Set_ZeroPos command sent.")

        # Re-enable motor
        self.enable_motor()
        
    # ---------------------------------------------------------------------
    # Read initial position from MotorRequest frames
    # ---------------------------------------------------------------------

    def read_initial_position(self, timeout_sec: float = 10.0) -> float:
        """
        Python equivalent of:
            float RobStrideMotor::read_initial_position()

        C++ logic (simplified):

            while (within 10 s) {
                read raw frame;
                if (extended) {
                    type = bits 24..31;
                    mid  = bits 8..15;
                    eid  = bits 0..7;
                    print type/mid/eid;
                    if (type == 0x02 && mid == 0x01 && eid == 0xFD) {
                        p_uint = (data[0] << 8) | data[1];
                        pos = uint_to_float(p_uint, -4*pi, 4*pi, 16);
                        print and return pos;
                    }
                }
            }

        Here, we generalize the match condition to:
            type == MotorRequest && mid == self.motor_id
        because our socket filter already ensures we only receive frames for this motor_id.
        """

        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            result = self._recv_frame(timeout=0.1)
            if result is None:
                continue

            can_id, dlc, data = result
            if dlc < 2:
                continue

            type_field = (can_id >> 24) & 0xFF
            mid = (can_id >> 8) & 0xFF
            eid = can_id & 0xFF

            print(f"type: 0x{type_field:02X}")
            print(f"mid:  0x{mid:02X}")
            print(f"eid:  0x{eid:02X}")

            # Match MotorRequest frames for this motor
            if type_field == Communication_Type_MotorRequest and mid == self.motor_id:
                p_uint = (data[0] << 8) | data[1]
                pos = self.uint_to_float(p_uint, -4.0 * math.pi, 4.0 * math.pi, 16)
                print(f"[✓] Initial position read: {pos:.6f} rad")
                return pos

        print("[!] Timeout waiting for motor feedback in read_initial_position().")
        return 0.0
    
    # ---------------------------------------------------------------------
    # Change motor CAN ID: Set_CAN_ID
    # ---------------------------------------------------------------------

    def set_can_id(self, new_can_id: int):
        """
        Python equivalent of:
            void RobStrideMotor::Set_CAN_ID(uint8_t Set_CAN_ID)

        C++ logic:

            Disenable_Motor(0);
            frame.can_id = (Can_ID_type << 24) | (Set_CAN_ID << 16) | (master_id << 8) | motor_id;
            data all zero;
            write(frame);

        IMPORTANT:
            This only sends the command to change the motor's ID in firmware.
            The current Python object still uses the old motor_id and CAN filter.
            To actually communicate with the motor under its new ID, you should
            create a NEW RobStrideMotorLinux instance with motor_id=new_can_id.
        """

        # Disable motor first, as in C++ code
        self.disable_motor(clear_error=0)

        # Build Set_CAN_ID frame
        new_can_id = new_can_id & 0xFF
        can_id = (
            (Communication_Type_Can_ID << 24)
            | (new_can_id << 16)
            | (self.master_id << 8)
            | self.motor_id
        )
        payload = b"\x00" * 8
        self._send_frame(can_id, payload)
        print(f"[✓] Motor Set_CAN_ID command sent (new ID = 0x{new_can_id:02X}).")

        # Note: we do NOT change self.motor_id or reconfigure the socket filter here.
        #       The caller should create a new RobStrideMotorLinux with the new ID.



