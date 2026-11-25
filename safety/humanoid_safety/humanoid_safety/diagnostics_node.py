"""Publish ``/safety/status`` from a separate process.

Building and serialising a DiagnosticArray is comparatively expensive, and a
Python publish call holds the GIL for the whole native write.  On the measured
stack a single status publication took 317 ms while the supervisor's IMU
callback gap reached 339 ms, because both shared one process and one
single-threaded executor.  Moving the rendering into its own process keeps the
supervisor's IMU reception independent of it, which an in-process thread cannot
do: a blocking native call freezes every Python thread in that process.

The supervisor publishes one compact JSON snapshot per status period on
``/safety/supervisor_state``; this node renders it.  The tuple (topic names,
QoS, message content) is unchanged for existing consumers such as the Web
monitor, and ``status_publisher:=inline`` restores the in-process path.
"""
from __future__ import annotations

import json
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .status_messages import render


class SafetyDiagnosticsNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_diagnostics")
        self.declare_parameter("state_topic", "/safety/supervisor_state")
        self.declare_parameter("status_topic", "/safety/status")
        self.declare_parameter("status_reliability", "reliable")
        self.declare_parameter("stale_after_s", 3.0)
        self.declare_parameter("status_rate_hz", 1.0)

        reliability = str(self.get_parameter("status_reliability").value)
        if reliability not in ("reliable", "best_effort"):
            raise ValueError("status_reliability must be reliable or best_effort")
        status_qos = (QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
                      if reliability == "best_effort" else QoSProfile(depth=2))
        self._status_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("status_topic").value), status_qos)
        self._stale_after_s = float(self.get_parameter("stale_after_s").value)
        self._stale_period_s = 1.0 / max(1.0, float(self.get_parameter("status_rate_hz").value))
        self._last_publish_mono = 0.0
        self._lock = threading.Lock()
        self._last_state = None
        self._last_state_mono = None
        # Before the first snapshot the clock starts at node start: a supervisor
        # that never appears must still produce a latched diagnostic.
        self._started_mono = time.monotonic()
        self._reported_stale = False
        self._published = 0
        self.create_subscription(
            String, str(self.get_parameter("state_topic").value), self._on_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        # A ROS timer would stop under frozen /clock; staleness is about host time.
        self._stop = threading.Event()
        self._watchdog = threading.Thread(target=self._watch_staleness,
                                          name="safety_status_staleness", daemon=True)
        self._watchdog.start()
        self.get_logger().info(
            f"Safety diagnostics from {self.get_parameter('state_topic').value} "
            f"to {self.get_parameter('status_topic').value} ({reliability})")

    def _on_state(self, msg: String) -> None:
        try:
            snapshot = json.loads(msg.data)
        except (ValueError, TypeError) as error:
            self.get_logger().warn(f"Ignoring malformed supervisor state: {error}",
                                   throttle_duration_sec=5.0)
            return
        if not isinstance(snapshot, dict):
            self.get_logger().warn("Ignoring non-object supervisor state",
                                   throttle_duration_sec=5.0)
            return
        with self._lock:
            self._last_state = snapshot
            self._last_state_mono = time.monotonic()
            if self._reported_stale:
                self._reported_stale = False
                self.get_logger().warn("Supervisor state restored")
            self._publish_locked(snapshot)
            self._last_publish_mono = time.monotonic()

    def _publish_locked(self, snapshot: dict) -> None:
        try:
            self._status_pub.publish(render(snapshot,
                                            stamp=self.get_clock().now().to_msg()))
        except RCLError as error:
            # Never let a diagnostics failure stop the staleness watchdog.
            self.get_logger().error(f'Safety status publication failed: {error}',
                                    throttle_duration_sec=5.0)
            return
        self._published += 1

    def _watch_staleness(self) -> None:
        while not self._stop.wait(0.5) and rclpy.ok():
            with self._lock:
                last = (self._last_state_mono if self._last_state_mono is not None
                        else self._started_mono)
                age = time.monotonic() - last
                if age <= self._stale_after_s:
                    continue
                # Keep publishing while stale, at the normal status rate: a
                # consumer that joins after the supervisor died must still see
                # the latched diagnostic, and DDS drops a one-shot to late
                # subscribers. The log line stays edge-triggered.
                if time.monotonic() - self._last_publish_mono >= self._stale_period_s:
                    self._publish_locked({
                        "latched": True,
                        "reason": f"no supervisor state for {age:.1f} s",
                        "checks": [],
                    })
                    self._last_publish_mono = time.monotonic()
                first = not self._reported_stale
                self._reported_stale = True
            if first:
                self.get_logger().error(
                    "Supervisor state missing; publishing a latched diagnostic")

    def destroy_node(self):
        self._stop.set()
        if self._watchdog.is_alive():
            self._watchdog.join(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyDiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
