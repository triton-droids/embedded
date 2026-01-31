#!/usr/bin/env python3
"""
Python CAN Node (ROS2) with per-motor RX threads + one TX thread. (SAFE RT VERSION)

Architecture:
- RX: one thread per motor reads status frames and updates shared state_buffer
- TX: one thread reads commands from a queue and sends CAN commands
- ROS2: subscribes JointState commands, publishes JointState states

Fix:
- Removed ctypes + pthread_setschedparam (was causing SIGSEGV due to wrong pthread_t usage)
- Use os.sched_setscheduler(0, ...) to set RT scheduling for the CURRENT thread safely.
  If no permission, it fails with PermissionError and we just warn (no crash).
"""

import os
import time
import math
import yaml
import queue
import threading
from dataclasses import dataclass
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from motor_control_hybrid.robstride_motor_linux import RobStrideMotorLinux


# -----------------------------
# Real-time scheduling (Linux) - SAFE
# -----------------------------
SCHED_FIFO = os.SCHED_FIFO
SCHED_OTHER = os.SCHED_OTHER


def set_thread_priority(priority: int, policy: int = SCHED_FIFO) -> bool:
    """
    Set scheduling for CURRENT thread (pid=0 => calling thread).
    Safe: no segfault; returns False on EPERM/EINVAL/etc.
    """
    if priority <= 0:
        return True
    try:
        os.sched_setscheduler(0, policy, os.sched_param(int(priority)))
        return True
    except PermissionError:
        return False
    except OSError:
        return False


# -----------------------------
# Command representation
# -----------------------------
@dataclass
class MotorCommand:
    joint: str
    cmd_type: str               # 'velocity' | 'position' | 'motion' | 'enable' | 'disable'
    position: float = 0.0
    velocity: float = 0.0
    acceleration: float = 0.0
    torque: float = 0.0
    kp: float = 40.0
    kd: float = 1.5


# -----------------------------
# Node
# -----------------------------
class PythonCanNode(Node):
    def __init__(self):
        super().__init__("python_can_node")

        # Parameters
        self.declare_parameter("motor_config_file", "")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("command_queue_size", 200)

        # Compatibility with your original logic:
        # position == 0.0 means "no position command" if True
        self.declare_parameter("use_zero_as_no_position", True)

        # RT options
        self.declare_parameter("rt_enable", True)
        self.declare_parameter("rt_base_priority", 80)   # 80-89 typical
        self.declare_parameter("rt_policy", "fifo")      # "fifo" or "other"

        cfg_path = self.get_parameter("motor_config_file").value
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        qsize = int(self.get_parameter("command_queue_size").value)
        self.use_zero_as_no_position = bool(self.get_parameter("use_zero_as_no_position").value)

        self.rt_enable = bool(self.get_parameter("rt_enable").value)
        self.rt_base_priority = int(self.get_parameter("rt_base_priority").value)
        rt_policy_str = str(self.get_parameter("rt_policy").value).lower().strip()
        self.rt_policy = SCHED_FIFO if rt_policy_str != "other" else SCHED_OTHER

        # Drivers
        self.drivers: Dict[str, RobStrideMotorLinux] = self._load_drivers(cfg_path)

        # Shared state: joint -> (pos, vel, tq, temp, timestamp)
        self.state_buffer: Dict[str, Tuple[float, float, float, float, float]] = {}
        self.state_lock = threading.Lock()

        # Thread-safe command queue
        self.command_queue: "queue.Queue[MotorCommand]" = queue.Queue(maxsize=qsize)

        # Stop flag
        self.stop_event = threading.Event()
        self.stop_event.clear()

        # Throttled logging
        self._last_log_time: Dict[str, float] = {}

        # Threads
        self.rx_threads: Dict[str, threading.Thread] = {}
        self._start_rx_threads(base_priority=self.rt_base_priority)
        self.tx_thread = self._start_thread(self._tx_loop, name="CAN_TX", daemon=True)

        # ROS2 pub/sub
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.cmd_sub = self.create_subscription(JointState, "joint_commands", self._cmd_callback, 10)

        # Timer publish
        period = 1.0 / publish_rate if publish_rate > 0 else 0.02
        self.timer = self.create_timer(period, self._publish_states)

        self.get_logger().info(
            f"Python CAN node started: motors={len(self.drivers)}, "
            f"rx_threads={len(self.rx_threads)}, qsize={qsize}, "
            f"rt_enable={self.rt_enable}, rt_policy={rt_policy_str}, rt_base_prio={self.rt_base_priority}"
        )

    # -------- config / init --------
    def _load_drivers(self, cfg_path: str) -> Dict[str, RobStrideMotorLinux]:
        drivers: Dict[str, RobStrideMotorLinux] = {}

        if not cfg_path or not os.path.exists(cfg_path):
            self.get_logger().warn(f"No motor config file found: {cfg_path}")
            return drivers

        self.get_logger().info(f"Loading motor config from: {cfg_path}")
        with open(cfg_path, "r") as f:
            data = yaml.safe_load(f) or {}

        params = data.get("motor_control_node", {}).get("ros__parameters", data)
        default_iface = params.get("default_can_interface", "can0")
        default_master = int(params.get("default_master_id", 255))
        motors_cfg = params.get("motors", {}) or {}

        for joint_name, cfg in motors_cfg.items():
            iface = cfg.get("can_interface", default_iface)
            master_id = int(cfg.get("master_id", default_master))
            motor_id = int(cfg["motor_id"])
            actuator_type = int(cfg.get("actuator_type", 0))

            self.get_logger().info(
                f"Initializing motor: {joint_name} on {iface}, master={master_id}, motor_id={motor_id}"
            )
            try:
                drivers[joint_name] = RobStrideMotorLinux(
                    iface=iface,
                    master_id=master_id,
                    motor_id=motor_id,
                    actuator_type=actuator_type,
                )
            except Exception as e:
                self.get_logger().error(f"Failed to init {joint_name}: {e}")

        return drivers

    def _start_thread(self, target, name: str, daemon: bool, args: tuple = ()) -> threading.Thread:
        t = threading.Thread(target=target, args=args, daemon=daemon, name=name)
        t.start()
        return t

    def _start_rx_threads(self, base_priority: int = 80):
        # Deterministic order: sort keys so priority assignment is stable across runs
        for idx, joint in enumerate(sorted(self.drivers.keys())):
            motor = self.drivers[joint]
            prio = base_priority + (idx % 10)  # 80-89
            t = self._start_thread(
                target=self._rx_loop_one_motor,
                name=f"CAN_RX_{joint}",
                daemon=True,
                args=(joint, motor, prio),
            )
            self.rx_threads[joint] = t

    # -------- logging helper --------
    def _throttle_warn(self, key: str, msg: str, interval_s: float = 1.0):
        now = time.monotonic()
        last = self._last_log_time.get(key, 0.0)
        if now - last >= interval_s:
            self._last_log_time[key] = now
            self.get_logger().warn(msg)

    # -------- RX / TX loops --------
    def _rx_loop_one_motor(self, joint: str, motor: RobStrideMotorLinux, priority: int):
        # Safe RT scheduling: will never segfault
        if self.rt_enable:
            if set_thread_priority(priority, self.rt_policy):
                self.get_logger().info(f"RX {joint}: RT set (policy={'FIFO' if self.rt_policy==SCHED_FIFO else 'OTHER'} prio={priority})")
            else:
                self._throttle_warn(f"rt_{joint}", f"RX {joint}: failed to set RT (EPERM/EINVAL likely).", 5.0)
        else:
            self.get_logger().info(f"RX {joint}: RT disabled")

        while (not self.stop_event.is_set()) and rclpy.ok():
            try:
                # timeout small; on vcan/no device this likely times out and raises/returns error
                pos, vel, tq, temp = motor.receive_status_frame(timeout=0.01)
                with self.state_lock:
                    self.state_buffer[joint] = (pos, vel, tq, temp, time.time())
            except Exception:
                self._throttle_warn(f"rx_err_{joint}", f"RX {joint}: receive_status_frame failed (no response expected on vcan).", 1.0)

            time.sleep(0.001)

    def _tx_loop(self):
        self.get_logger().info("TX thread started")
        while (not self.stop_event.is_set()) and rclpy.ok():
            try:
                cmd: MotorCommand = self.command_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            motor = self.drivers.get(cmd.joint)
            if motor is None:
                self._throttle_warn("unknown_joint", f"TX: unknown joint {cmd.joint}", 1.0)
                continue

            try:
                if cmd.cmd_type == "velocity":
                    motor.send_velocity_mode_command(velocity_rad_s=cmd.velocity)

                elif cmd.cmd_type == "position":
                    motor.pos_pp_control(
                        speed_rad_s=cmd.velocity,
                        acceleration_rad_s2=cmd.acceleration,
                        angle_rad=cmd.position,
                    )

                elif cmd.cmd_type == "motion":
                    motor.send_motion_command(
                        torque=cmd.torque,
                        position_rad=cmd.position,
                        velocity_rad_s=cmd.velocity,
                        kp=cmd.kp,
                        kd=cmd.kd,
                    )

                elif cmd.cmd_type == "enable":
                    motor.enable_motor()

                elif cmd.cmd_type == "disable":
                    motor.disable_motor()

            except Exception as e:
                self._throttle_warn(f"tx_err_{cmd.joint}", f"TX {cmd.joint}: send failed: {e}", 1.0)

    # -------- ROS callbacks --------
    def _cmd_callback(self, msg: JointState):
        """
        Heuristic:
          - Default cmd_type is velocity using msg.velocity[i]
          - If position is present:
              - if use_zero_as_no_position=True and position==0.0 -> keep velocity mode
              - else -> position mode
        """
        if not msg.name:
            return

        for i, joint in enumerate(msg.name):
            if joint not in self.drivers:
                continue

            cmd_type = "velocity"
            pos = 0.0
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            tq = msg.effort[i] if i < len(msg.effort) else 0.0

            if i < len(msg.position):
                pos = msg.position[i]
                # allow NaN sentinel to mean "no position" if you ever use it
                if not math.isnan(pos):
                    if (not self.use_zero_as_no_position) or (pos != 0.0):
                        cmd_type = "position"

            cmd = MotorCommand(
                joint=joint,
                cmd_type=cmd_type,
                position=pos,
                velocity=vel,
                torque=tq,
            )

            try:
                self.command_queue.put_nowait(cmd)
            except queue.Full:
                self._throttle_warn("q_full", f"Command queue full, drop {joint}", 0.5)

    def _publish_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        with self.state_lock:
            for joint, (pos, vel, tq, temp, ts) in self.state_buffer.items():
                msg.name.append(joint)
                msg.position.append(pos)
                msg.velocity.append(vel)
                msg.effort.append(tq)

        if msg.name:
            self.joint_state_pub.publish(msg)

    # -------- shutdown --------
    def destroy_node(self):
        self.get_logger().info("Shutting down Python CAN node...")
        self.stop_event.set()

        # join RX threads
        for joint, t in self.rx_threads.items():
            t.join(timeout=1.0)
            if t.is_alive():
                self._throttle_warn(f"join_{joint}", f"RX thread {joint} did not exit in time.", 1.0)

        # join TX thread
        if self.tx_thread.is_alive():
            self.tx_thread.join(timeout=1.0)
            if self.tx_thread.is_alive():
                self._throttle_warn("join_tx", "TX thread did not exit in time.", 1.0)

        # disable motors
        for joint, motor in self.drivers.items():
            try:
                motor.disable_motor()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PythonCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
