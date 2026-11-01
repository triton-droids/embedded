"""Installed rclpy heartbeat test; optional synthetic rcl_publish no-op stub."""
import json
import os
import threading
import time

import rclpy
from std_msgs.msg import String

rclpy.init(args=[], domain_id=225)
node = rclpy.create_node('isolated_publish_gil_research')
publisher = node.create_publisher(String, '/research/gil_probe', 1)
stop = threading.Event()
beats = []
def heartbeat():
    while not stop.is_set():
        beats.append(time.monotonic())
        time.sleep(.001)
thread = threading.Thread(target=heartbeat)
thread.start()
try:
    time.sleep(.05)
    started = time.monotonic()
    publisher.publish(String(data='synthetic mechanism test; no actuator topics'))
    elapsed = time.monotonic()-started
    time.sleep(.05)
finally:
    stop.set()
    thread.join()
    node.destroy_node()
    rclpy.shutdown()
print(json.dumps(dict(synthetic_no_delivery=bool(os.environ.get('LD_PRELOAD')),
    injected_delay_us=os.environ.get('IMU_RESEARCH_DELAY_US','0'),
    publish_ms=elapsed*1000,
    heartbeat_max_gap_ms=max(b-a for a,b in zip(beats,beats[1:]))*1000,
    heartbeat_samples=len(beats))),flush=True)
