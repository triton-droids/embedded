#!/usr/bin/env python3
"""
RobStrideMotorLinux wrapper based on the official robstride_dynamics SDK.

- Each RobStrideMotorLinux instance corresponds to one motor ID.
- Internally it creates a RobstrideBus and a single Motor.
- Uses MIT mode: write_operation_frame / read_operation_frame.
- Keeps the same method names expected by the upper layers (can_node / controller_node):
    - enable_motor()
    - disable_motor()
    - send_velocity_mode_command()
    - pos_pp_control()
    - current_control()
    - send_motion_command()
    - receive_status_frame()
"""

import math
import time
from typing import Tuple

from robstride_dynamics import RobstrideBus, Motor, ParameterType
import robstride_dynamics.table as table


# ---------------- RS-02 parameter table configuration (matching your script) ----------------

RS02_PARAMS = {
    "rs-02": {
        "position": 12.57,  # 4 * PI
        "velocity": 44.0,
        "torque": 17.0,
        "kp": 500.0,
        "kd": 5.0,
    }
}

# Monkey-patch SDK tables for RS-02 using the values above
table.MODEL_MIT_POSITION_TABLE["rs-02"] = RS02_PARAMS["rs-02"]["position"]
table.MODEL_MIT_VELOCITY_TABLE["rs-02"] = RS02_PARAMS["rs-02"]["velocity"]
table.MODEL_MIT_TORQUE_TABLE["rs-02"] = RS02_PARAMS["rs-02"]["torque"]
table.MODEL_MIT_KP_TABLE["rs-02"] = RS02_PARAMS["rs-02"]["kp"]
table.MODEL_MIT_KD_TABLE["rs-02"] = RS02_PARAMS["rs-02"]["kd"]


class RobStrideMotorLinux:
    """
    Thin wrapper around the official SDK.

    - Each motor_id gets its own RobstrideBus (interface + single Motor).
      This avoids dealing with shared-bus complexity for now and keeps it simple/stable.
    """

    def __init__(self, iface: str, master_id: int, motor_id: int, actuator_type: int = 0) -> None:
        self.iface = iface
        self.master_id = master_id
        self.motor_id = int(motor_id)
        self.actuator_type = int(actuator_type)

        # Human-readable name, same style as your example script
        self.motor_name = f"motor_{self.motor_id}"

        # Create Motor and Bus
        motors = {self.motor_name: Motor(id=self.motor_id, model="rs-02")}
        self.bus = RobstrideBus(self.iface, motors, {})

        # Connect to CAN bus with handshake
        self.bus.connect(handshake=True)

        # Cached state (position/velocity/torque/temperature)
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0
        self.temperature = 0.0

        # Default controller gains used for MIT mode commands
        self.default_kp = 40.0
        self.default_kd = 1.5

    # ----------------- Update current state from a status frame -----------------

    def _update_from_response(
        self,
        p_act: float,
        v_act: float,
        t_act: float,
        temp: float
    ) -> Tuple[float, float, float, float]:
        self.position = float(p_act)
        self.velocity = float(v_act)
        self.torque = float(t_act)
        self.temperature = float(temp)
        return self.position, self.velocity, self.torque, self.temperature

    def receive_status_frame(self, timeout: float = 0.0) -> Tuple[float, float, float, float]:
        """
        Read one status frame using read_operation_frame.
        The SDK does not expose a timeout here, so the timeout argument is ignored.
        If reading fails, keep the cached values and print a debug message.
        """
        try:
            p_act, v_act, t_act, temp = self.bus.read_operation_frame(self.motor_name)
            return self._update_from_response(p_act, v_act, t_act, temp)
        except Exception as e:
            print(f"[RobStrideMotorLinux] read_operation_frame failed: {e}")
            return self.position, self.velocity, self.torque, self.temperature

    # ----------------- Public interface used by upper layers -----------------

    def enable_motor(self) -> Tuple[float, float, float, float]:
        """
        1. Enable the motor.
        2. Switch to MIT mode (MODE = 0).
        """
        self.bus.enable(self.motor_name)
        self.bus.write(self.motor_name, ParameterType.MODE, 0)
        time.sleep(0.1)
        return self.receive_status_frame()

    def disable_motor(self) -> Tuple[float, float, float, float]:
        """
        Send a zero-torque MIT command and then disable the motor.
        """
        try:
            self.bus.write_operation_frame(
                self.motor_name,
                0.0,  # position
                0.0,  # kp
                0.0,  # kd
                0.0,  # velocity
                0.0,  # torque
            )
            time.sleep(0.05)
            self.bus.disable(self.motor_name)
        except Exception:
            pass
        return self.receive_status_frame()

    def send_motion_command(
        self,
        torque: float,
        position_rad: float,
        velocity_rad_s: float,
        kp: float,
        kd: float,
    ) -> Tuple[float, float, float, float]:
        """
        Generic MIT-mode command, following the official example argument order:
        write_operation_frame(motor_name, position, kp, kd, velocity, torque)
        """
        self.bus.write_operation_frame(
            self.motor_name,
            position_rad,
            kp,
            kd,
            velocity_rad_s,
            torque,
        )
        p_act, v_act, t_act, temp = self.bus.read_operation_frame(self.motor_name)
        return self._update_from_response(p_act, v_act, t_act, temp)

    def send_velocity_mode_command(
        self,
        velocity_rad_s: float,
        acceleration_rad_s2: float | None = None,
    ) -> Tuple[float, float, float, float]:
        """
        Approximate “velocity control” using MIT mode:
        - Keep the current position as the position target.
        - Set the desired velocity to velocity_rad_s.
        - Use Kp = 0 (no position stiffness), Kd > 0 for some damping.
        """
        self.bus.write_operation_frame(
            self.motor_name,
            self.position,
            0.0,              # Kp
            self.default_kd,   # Kd
            velocity_rad_s,
            0.0,              # torque feed-forward
        )
        p_act, v_act, t_act, temp = self.bus.read_operation_frame(self.motor_name)
        return self._update_from_response(p_act, v_act, t_act, temp)

    def pos_pp_control(
        self,
        speed_rad_s: float,
        acceleration_rad_s2: float,
        angle_rad: float,
    ) -> Tuple[float, float, float, float]:
        """
        Point-to-point position control using MIT mode:
        - Target position = angle_rad
        - Target velocity = speed_rad_s
        - Use default Kp/Kd (tuned for RS-02) for stiffness/damping.
        """
        kp = self.default_kp
        kd = self.default_kd

        self.bus.write_operation_frame(
            self.motor_name,
            angle_rad,
            kp,
            kd,
            speed_rad_s,
            0.0,
        )
        p_act, v_act, t_act, temp = self.bus.read_operation_frame(self.motor_name)
        return self._update_from_response(p_act, v_act, t_act, temp)

    def current_control(
        self,
        iq_command: float,
        id_command: float = 0.0,
    ) -> Tuple[float, float, float, float]:
        """
        The official SDK does not expose a dedicated current-control mode.
        Here we approximate it by using the torque feed-forward field:
        - Treat iq_command as the torque feed-forward.
        - Hold the current position, zero desired velocity.
        """
        self.bus.write_operation_frame(
            self.motor_name,
            self.position,
            0.0,
            0.0,
            0.0,
            iq_command,
        )
        p_act, v_act, t_act, temp = self.bus.read_operation_frame(self.motor_name)
        return self._update_from_response(p_act, v_act, t_act, temp)
