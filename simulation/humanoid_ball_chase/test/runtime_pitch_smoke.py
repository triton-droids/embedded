"""Real rendered football detection, actual yaw/pitch feedback and loss-stop test."""
import json
from pathlib import Path
import statistics
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState,Image
from std_msgs.msg import String


def main():
    rclpy.init();node=rclpy.create_node('football_pitch_runtime_smoke')
    status=[];feedback=[];images=[]
    node.create_subscription(Image,"/camera/camera/color/image_raw",lambda m:images.append(m) if len(images)<2 else images.__setitem__(-1,m),5)
    node.create_subscription(String,'/ball_chase/status',lambda m:status.append((time.monotonic(),json.loads(m.data))),50)
    node.create_subscription(JointState,'/head/joint_states',lambda m:feedback.append((time.monotonic(),dict(zip(m.name,m.position)))),50)
    pub=node.create_publisher(PointStamped,'/ball_chase/throw_position',10)
    def spin(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
    def place(x,y,z):
        m=PointStamped();m.header.frame_id='world';m.point.x=x;m.point.y=y;m.point.z=z;pub.publish(m)
    def hold(z,seconds=5):
        start=time.monotonic()
        while time.monotonic()-start<seconds:
            place(-.25,1.8,z);spin(.08)
        tracks=[s for t,s in status if t>start+seconds-1 and s['tracking']]
        joints=[j['head_pitch_joint'] for t,j in feedback if t>start+seconds-1]
        if images:cv2.imwrite(f'/tmp/football-height-{z}.png',CvBridge().imgmsg_to_cv2(images[-1],'bgr8'))
        assert tracks and joints,f'No detection/feedback height={z}; joints={joints[-3:]}; status={status[-1] if status else None}'
        assert all(s['detector']=='football_yolo' for s in tracks)
        return statistics.median(joints),len(tracks),max(s['detection_confidence'] for s in tracks)
    deadline=time.monotonic()+30
    while time.monotonic()<deadline and not any(s['tracking'] for _,s in status):spin(.1)
    assert any(s['tracking'] for _,s in status),'YOLO did not acquire rendered football'
    low1,n1,c1=hold(.11)
    high,n2,c2=hold(.65)
    low2,n3,c3=hold(.11)
    assert high<low1-.1,(low1,high,low2)
    assert low2>high+.1,(low1,high,low2)
    place(0.,-2.,.11);start=time.monotonic();spin(2)
    lost=[s for t,s in status if t>start+1.2]
    assert lost and all(s['lost'] and s['command']=={'linear_x':0.,'angular_z':0.} for s in lost)
    place(-.25,1.8,.11);start=time.monotonic();spin(4)
    assert any(s['tracking'] for t,s in status if t>start),'Could not reacquire returned ball'
    result={'pitch_low_high_low_rad':[low1,high,low2],'tracking_samples_last_second':[n1,n2,n3],
            'football_confidence_max':[c1,c2,c3],'lost_zero_frames':len(lost),'reacquired':True,
            'locomotion_verified':False,'detector':'td-perception YOLO11n ONNX class 0'}
    Path('/tmp/football-pitch-smoke.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
