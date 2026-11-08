"""Same 100 Hz synthetic IMU stream as imu_bench.cpp, no control topics."""
import argparse
import json
from pathlib import Path
import threading
import time

import rclpy
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile,ReliabilityPolicy

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--duration',type=float,default=12)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
rclpy.init(args=[])
node=rclpy.create_node('python_imu_transport_publisher')
pub=node.create_publisher(Imu,'/research/imu',QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE))
beats=[]
stop=threading.Event()
def heartbeat():
    while not stop.is_set():
        beats.append(time.monotonic())
        time.sleep(.001)
worker=threading.Thread(target=heartbeat)
worker.start()
calls=[];times=[]
try:
    time.sleep(2)
    start=time.monotonic();deadline=start
    while time.monotonic()-start<args.duration:
        times.append(time.monotonic())
        msg=Imu();msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='imu_link'
        msg.orientation.w=1.;msg.angular_velocity.x=.001*len(times);msg.linear_acceleration.z=9.81
        before=time.monotonic();pub.publish(msg);calls.append((time.monotonic()-before)*1000)
        deadline+=.01
        time.sleep(max(0,deadline-time.monotonic()))
        if deadline<time.monotonic()-.01:deadline=time.monotonic()
    elapsed=time.monotonic()-start
finally:
    stop.set();worker.join();node.destroy_node();rclpy.shutdown()
def quantile(values,fraction):
    values=sorted(values)
    return values[int((len(values)-1)*fraction)] if values else 0
gaps=[(b-a)*1000 for a,b in zip(times,times[1:])]
beat_gaps=[(b-a)*1000 for a,b in zip(beats,beats[1:])]
args.output.write_text(json.dumps(dict(language='python',count=len(calls),elapsed_s=elapsed,
    publish_max_ms=max(calls,default=0),publish_p99_ms=quantile(calls,.99),
    source_or_receipt_gap_max_ms=max(gaps,default=0),heartbeat_gap_max_ms=max(beat_gaps,default=0)))+'\n')
