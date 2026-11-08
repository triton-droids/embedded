"""Run with ball_chase.launch.py on a private ROS domain; preserves gantry state.

This checks visual command response and loss stop, not locomotion performance.
"""
import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from std_msgs.msg import Float32, String


def main():
    rclpy.init()
    node=rclpy.create_node('ball_chase_runtime_smoke')
    samples=[]
    heads=[]
    commands=[]
    node.create_subscription(String,'/ball_chase/status',lambda m:samples.append((time.monotonic(),json.loads(m.data))),50)
    node.create_subscription(Float32,'/head/servo_target_rad',lambda m:heads.append(float(m.data)),50)
    node.create_subscription(TwistStamped,'/ball_chase/cmd_vel',lambda m:commands.append((time.monotonic(),m.twist.linear.x,m.twist.angular.z)),50)
    pub=node.create_publisher(PointStamped,'/ball_chase/throw_position',10)
    def spin(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.05)
    def place(x,y):
        msg=PointStamped();msg.header.frame_id='world';msg.point.x=x;msg.point.y=y;msg.point.z=.11
        pub.publish(msg)
    deadline=time.monotonic()+30
    while time.monotonic()<deadline and not any(s['tracking'] for _,s in samples):
        spin(.1)
    assert any(s['tracking'] for _,s in samples),'No rendered RGB-D/TF tracking'
    start=time.monotonic();spin(3)
    left=[s for t,s in samples if t>=start and s['tracking']]
    place(.35,1.8);start=time.monotonic();spin(5)
    right=[s for t,s in samples if t>=start+1 and s['tracking']]
    assert left and right,'Tracking did not recover after ball placement'
    assert any(s['command']['angular_z']>0 for s in left),'No positive turn for left ball'
    assert any(s['command']['angular_z']<0 for s in right),'No negative turn for right ball'
    place(0.,-2.);start=time.monotonic();spin(2)
    lost=[s for t,s in samples if t>=start+1.2]
    assert lost and all(s['lost'] and s['command']=={'linear_x':0.,'angular_z':0.} for s in lost),'Loss did not stop commands'
    zero=[(x,z) for t,x,z in commands if t>=start+1.2]
    assert zero and all(x==0 and z==0 for x,z in zero)
    assert heads and max(heads)-min(heads)>.02,'Head tracker did not respond'
    place(-.25,1.8)
    report={'tracking_frames_left':len(left),'tracking_frames_right':len(right),'lost_zero_status_frames':len(lost),
            'head_target_range_rad':[min(heads),max(heads)],'all_samples':len(samples),
            'verified':'rendered RGB-D -> detection -> image-time moving TF -> bounded pursuit; head response; loss stop',
            'locomotion_verified':False}
    Path('/tmp/ball-chase-runtime-smoke.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
