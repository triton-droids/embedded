"""Stereo/IMU stamp probe: per topic, count, header-stamp deltas (sim time) and
arrival gaps (wall), over DURATION wall seconds. Usage: python3 stamp_probe.py 30"""
import sys, time, statistics as st
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from rosgraph_msgs.msg import Clock

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
rclpy.init(); n = rclpy.create_node('stamp_probe')
data = {}
clock = []
def cb(topic):
    def f(m):
        s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        data.setdefault(topic, []).append((s, time.time()))
    return f
for t, ty in (('/stereo/left/image_raw', Image), ('/stereo/right/image_raw', Image), ('/head/imu', Imu)):
    n.create_subscription(ty, t, cb(t), qos_profile_sensor_data)
n.create_subscription(Clock, '/clock', lambda m: clock.append((m.clock.sec + m.clock.nanosec * 1e-9, time.time())), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.05)
if len(clock) > 1:
    print(f'clock rtf={(clock[-1][0] - clock[0][0]) / (clock[-1][1] - clock[0][1]):.3f}')
for t, v in data.items():
    if len(v) < 3:
        print(t, 'n', len(v)); continue
    ds = [1e3 * (b[0] - a[0]) for a, b in zip(v, v[1:])]
    dw = [1e3 * (b[1] - a[1]) for a, b in zip(v, v[1:])]
    sim_span = v[-1][0] - v[0][0]; wall = v[-1][1] - v[0][1]
    big = sum(1 for d in ds if d > 100)
    print(f'{t}: n={len(v)} wall_hz={(len(v) - 1) / wall:.2f} sim_hz={(len(v) - 1) / sim_span:.2f} '
          f'stamp_dt_ms med={st.median(ds):.0f} max={max(ds):.0f} >100ms={big} ({100 * big / len(ds):.0f}%) '
          f'arrival_gap_ms med={st.median(dw):.0f} max={max(dw):.0f}')
