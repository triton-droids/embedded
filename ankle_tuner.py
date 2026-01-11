#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobStride Ankle Tuner + Freudenstein Linkage + Live IMU Feedback
"""

import sys
import os
import time
import math
import struct
import threading
import signal
import numpy as np
from scipy.optimize import fsolve
from dataclasses import dataclass, field
from typing import Optional, Dict, Set, List, Tuple, Any
from collections import deque
import traceback
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# -------------------- RobStride SDK --------------------
# (Adjust path as necessary for your setup)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from robstride_dynamics import RobstrideBus, Motor, ParameterType, CommunicationType
except ImportError:
    try:
        from bus import RobstrideBus, Motor
        from protocol import ParameterType, CommunicationType
    except ImportError:
        print("RobStride SDK not found.")
        sys.exit(1)

# -------------------- IMU Import --------------------
try:
    from imu_read import iter_imu_samples
    IMU_AVAILABLE = True
except ImportError:
    print("WARNING: 'imu_read.py' not found. IMU features disabled.")
    IMU_AVAILABLE = False

# -------------------- Linkage Configuration --------------------
# Link lengths (Tune these based on Plot vs IMU)
L1 = 6.5625  # Ground
L2 = 1.79    # Input (Motor Crank)
L3 = 6.5     # Coupler
L4 = 1.875   # Output (Ankle)

# Pre-calculate Freudenstein Constants
K1 = L1 / L4
K2 = L1 / L2
K3 = (L2**2 - L3**2 + L4**2 + L1**2) / (2 * L2 * L4)

def solve_foot_to_motor(target_foot_deg):
    """
    Given a desired foot angle (theta4), solve for required motor angle (theta2).
    Returns radians for the motor.
    """
    theta4 = np.deg2rad(target_foot_deg)
    
    def linkage_constraint(t2):
        return K1 * np.cos(theta4) - K2 * np.cos(t2) - np.cos(t2 - theta4) + K3

    # Guess: motor roughly tracks foot
    theta2_sol = fsolve(linkage_constraint, theta4)[0]
    return theta2_sol

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

# -------------------- Configuration --------------------
# IDs 5 and 10 are ankles
ANKLE_IDS = [5, 10]
IMU_PORT = "/dev/ttyUSB0"  # CHECK YOUR PORT (e.g., COM3 on Windows)
IMU_BAUD = 115200
# Which IMU field to use? 'pitch_deg' or 'roll_deg' depending on mounting
IMU_TARGET_AXIS = "pitch_deg" 

MOTOR_MODEL_BY_ID = {
    1: "rs-04", 2: "rs-03", 3: "rs-03", 4: "rs-04", 5: "rs-02",
    6: "rs-04", 7: "rs-03", 8: "rs-03", 9: "rs-04", 10: "rs-02",
}
INVERSION_ARRAY = [1, 1, 1, 1, 1, -1, -1, -1, -1, -1]
INVERSION_BY_ID = {i + 1: INVERSION_ARRAY[i] for i in range(len(INVERSION_ARRAY))}
JOINT_LIMITS = {
    5: (-0.6, 0.6),   # Left Ankle
    10: (-0.6, 0.6),  # Right Ankle
}

@dataclass
class Excitation:
    mode: str = "none"
    amp_rad: float = 0.0
    freq_hz: float = 0.0
    t0: float = 0.0
    duration_s: Optional[float] = None
    center_rad: float = 0.0

@dataclass
class MotorState:
    id: int
    name: str
    model: str
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    temperature: float = 0.0
    kp: float = 10.0
    kd: float = 0.2
    direction: int = 1
    limit_lo: float = -10.0
    limit_hi: float = 10.0
    target_rad: float = 0.0          # Logical (Foot) Target
    commanded_target_rad: float = 0.0 # Logical (Foot) Commanded
    hold_center_rad: float = 0.0
    excitation: Excitation = field(default_factory=Excitation)
    temp_state: str = "OK"
    enabled: bool = True
    is_ankle: bool = False  # Enables Freudenstein math

# -------------------- Main Tuner Class --------------------
class GainTunerMIT:
    def __init__(self, motor_ids, channel="can0"):
        self.motor_states = {}
        for mid in motor_ids:
            model = MOTOR_MODEL_BY_ID.get(mid, "rs-03")
            st = MotorState(id=mid, name=f"motor_{mid}", model=model)
            st.direction = INVERSION_BY_ID.get(mid, 1)
            if mid in JOINT_LIMITS:
                st.limit_lo, st.limit_hi = JOINT_LIMITS[mid]
            if mid in ANKLE_IDS:
                st.is_ankle = True
            self.motor_states[mid] = st

        self.selected = set(motor_ids)
        self.bus = None
        self.lock = threading.Lock()
        self.running = True
        self.connected = False
        self.channel = channel
        
        # IMU Data
        self.imu_val_deg = 0.0
        self.imu_offset_deg = 0.0
        self.imu_connected = False

    def connect(self):
        # (Simplified connection logic from your script)
        motors = {st.name: Motor(id=mid, model=st.model) for mid, st in self.motor_states.items()}
        cal = {st.name: {"direction": 1, "homing_offset": 0.0} for st in self.motor_states.values()}
        try:
            self.bus = RobstrideBus(self.channel, motors, cal)
            print(f"Connecting to {self.channel}...")
            self.bus.connect(handshake=True)
            for mid, st in self.motor_states.items():
                self.bus.enable(st.name)
                # Set Zero Mode
                param_id, _, _ = ParameterType.MODE
                data = struct.pack("<HH", param_id, 0x00) + struct.pack("<bBH", 0, 0, 0)
                device_id = self.bus.motors[st.name].id
                self.bus.transmit(CommunicationType.WRITE_PARAMETER, self.bus.host_id, device_id, data)
                time.sleep(0.1)
                
                # Read initial pose
                p, v, t, temp = self.bus.read_operation_frame(st.name)
                st.position, st.velocity, st.torque, st.temperature = p, v, t, temp
                
                # Initialize Logical Targets
                logical_pos = p / float(st.direction) 
                
                # If ankle, we assume motor 0 = foot 0 for startup, 
                # but ideally we should reverse solve Freudenstein here.
                # For safety on startup, we just hold current motor pos.
                if st.is_ankle:
                    # Just sync commanded to 0 to be safe, or estimate based on linear
                    st.target_rad = 0.0 
                    st.commanded_target_rad = 0.0
                else:
                    st.target_rad = logical_pos
                    st.commanded_target_rad = logical_pos

            self.connected = True
            return True
        except Exception as e:
            print(f"Connection Failed: {e}")
            return False

    def control_step(self, dt):
        if not (self.running and self.connected): return
        
        with self.lock:
            now = time.time()
            for st in self.motor_states.values():
                if not st.enabled: continue
                
                # 1. Excitation (Generates Logical/Foot Target)
                ex = st.excitation
                if ex.mode == "sine":
                    st.target_rad = ex.center_rad + ex.amp_rad * math.sin(2*math.pi*ex.freq_hz*(now - ex.t0))
                
                # 2. Clamp Logical Limits (Foot Limits)
                st.target_rad = clamp(st.target_rad, st.limit_lo, st.limit_hi)
                
                # 3. Ramp Generator
                max_step = math.radians(30.0) * dt # 30 deg/s ramp
                delta = st.target_rad - st.commanded_target_rad
                if abs(delta) <= max_step:
                    st.commanded_target_rad = st.target_rad
                else:
                    st.commanded_target_rad += math.copysign(max_step, delta)

                # 4. Kinematic Mapping (The Magic)
                logical_deg = math.degrees(st.commanded_target_rad)
                
                if st.is_ankle:
                    # Convert Foot Deg -> Motor Rads (Non-Linear)
                    motor_rad_target = solve_foot_to_motor(logical_deg)
                    physical_target = motor_rad_target * float(st.direction)
                else:
                    # 1:1 Mapping
                    physical_target = st.commanded_target_rad * float(st.direction)

                # 5. Send & Read
                try:
                    self.bus.write_operation_frame(st.name, physical_target, st.kp, st.kd, 0.0, 0.0)
                    p, v, t, temp = self.bus.read_operation_frame(st.name)
                    st.position, st.velocity, st.torque, st.temperature = p, v, t, temp
                except Exception:
                    pass

    def tare_imu(self):
        """Sets current IMU reading as 0 degrees"""
        with self.lock:
            # We assume the current reading is the offset, so subsequent readings - offset = 0
            # But actually, we want reading - offset = current_target (if we trust the robot is at target)
            # Simplest: Make current IMU reading 0.
            self.imu_offset_deg = self.imu_val_deg + self.imu_offset_deg 
            print(f"IMU Tared. Offset: {self.imu_offset_deg:.2f}")

    def shutdown(self):
        self.running = False
        if self.bus: self.bus.disconnect()

# -------------------- IMU Thread --------------------
def imu_worker(tuner):
    if not IMU_AVAILABLE: return
    print(f"Starting IMU stream on {IMU_PORT}...")
    try:
        # Using serial mode, no integrator needed for simple roll/pitch from Arduino
        for sample in iter_imu_samples(source="serial", port=IMU_PORT, baud=IMU_BAUD):
            if not tuner.running: break
            
            raw_val = sample.get(IMU_TARGET_AXIS)
            if raw_val is not None:
                with tuner.lock:
                    tuner.imu_connected = True
                    # Store relative value
                    tuner.imu_val_deg = raw_val - tuner.imu_offset_deg
    except Exception as e:
        print(f"IMU Error: {e}")

# -------------------- Live Plotter --------------------
class LivePlotter:
    def __init__(self, tuner):
        self.tuner = tuner
        self.fig, self.ax = plt.subplots(2, 1, sharex=True, figsize=(8, 8))
        
        # Plot 1: Position Tracking (Foot Space)
        self.ax[0].set_title("Linkage Tracking: Software Goal vs Hardware Reality")
        self.ax[0].set_ylabel("Ankle Angle (deg)")
        self.l_cmd, = self.ax[0].plot([], [], 'b-', label="Software (Linkage Cmd)")
        self.l_imu, = self.ax[0].plot([], [], 'm--', linewidth=2, label="Hardware (IMU)")
        self.ax[0].legend()
        self.ax[0].grid(True)
        
        # Plot 2: Error
        self.ax[1].set_ylabel("Tracking Error (deg)")
        self.l_err, = self.ax[1].plot([], [], 'r-', label="Error (Cmd - IMU)")
        self.ax[1].legend()
        self.ax[1].grid(True)

        self.t_data = deque(maxlen=200)
        self.cmd_data = deque(maxlen=200)
        self.imu_data = deque(maxlen=200)
        self.err_data = deque(maxlen=200)
        self.start_t = time.time()

    def update(self, frame):
        # Run Control Loop
        self.tuner.control_step(0.02) 
        
        # Get Data
        t = time.time() - self.start_t
        
        # We prefer plotting the FIRST selected motor
        mid = list(self.tuner.selected)[0] if self.tuner.selected else 5
        st = self.tuner.motor_states[mid]
        
        cmd_deg = math.degrees(st.commanded_target_rad) # Where code thinks foot is
        imu_deg = self.tuner.imu_val_deg # Where IMU says foot is
        
        self.t_data.append(t)
        self.cmd_data.append(cmd_deg)
        self.imu_data.append(imu_deg)
        self.err_data.append(cmd_deg - imu_deg)
        
        self.l_cmd.set_data(self.t_data, self.cmd_data)
        self.l_imu.set_data(self.t_data, self.imu_data)
        self.l_err.set_data(self.t_data, self.err_data)
        
        self.ax[0].set_xlim(max(0, t-10), t+0.1)
        self.ax[0].set_ylim(min(min(self.cmd_data), min(self.imu_data))-5, max(max(self.cmd_data), max(self.imu_data))+5)
        self.ax[1].relim()
        self.ax[1].autoscale_view()
        
        return self.l_cmd, self.l_imu, self.l_err

    def show(self):
        ani = FuncAnimation(self.fig, self.update, interval=20)
        plt.show()

# -------------------- CLI --------------------
def command_loop(tuner):
    print("\n--- TUNING CONSOLE ---")
    print(" cmds: tare, select <id>, kp <val>, goto <deg>, sine <amp> <freq>")
    while True:
        try:
            cmd = input(">> ").strip().lower()
            if cmd == "q": 
                tuner.shutdown()
                os._exit(0)
            elif cmd == "tare":
                tuner.tare_imu()
            elif cmd.startswith("select"):
                mid = int(cmd.split()[1])
                tuner.selected = {mid}
                print(f"Selected Motor {mid}")
            elif cmd.startswith("goto"):
                deg = float(cmd.split()[1])
                with tuner.lock:
                    for mid in tuner.selected:
                        tuner.motor_states[mid].excitation = Excitation()
                        tuner.motor_states[mid].target_rad = math.radians(deg)
            elif cmd.startswith("sine"):
                parts = cmd.split()
                amp = float(parts[1])
                freq = float(parts[2])
                with tuner.lock:
                    for mid in tuner.selected:
                        st = tuner.motor_states[mid]
                        st.excitation = Excitation(mode="sine", amp_rad=math.radians(amp), freq_hz=freq, t0=time.time(), center_rad=st.target_rad)
            elif cmd.startswith("kp"):
                kp = float(cmd.split()[1])
                with tuner.lock:
                    for mid in tuner.selected: tuner.motor_states[mid].kp = kp
        except Exception as e:
            print(f"Error: {e}")

# -------------------- Main --------------------
if __name__ == "__main__":
    ids = [5] # Default to left ankle
    tuner = GainTunerMIT(ids)
    
    # Start IMU Thread
    t_imu = threading.Thread(target=imu_worker, args=(tuner,), daemon=True)
    t_imu.start()
    
    if tuner.connect():
        t_cli = threading.Thread(target=command_loop, args=(tuner,), daemon=True)
        t_cli.start()
        
        plotter = LivePlotter(tuner)
        plotter.show()
