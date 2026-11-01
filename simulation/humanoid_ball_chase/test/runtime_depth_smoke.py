"""Check live RGB/depth/info synchronization and a detected football's depth."""
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from message_filters import Subscriber,TimeSynchronizer
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo


def main():
    rclpy.init();node=rclpy.create_node('football_depth_runtime_smoke');bridge=CvBridge()
    frames={};points={}
    def key(header):return (header.stamp.sec,header.stamp.nanosec)
    def frame(rgb,depth,info):
        assert rgb.header.frame_id==depth.header.frame_id==info.header.frame_id
        frames[key(rgb.header)]=(rgb,depth,info)
        while len(frames)>30:frames.pop(next(iter(frames)))
    subscribers=[Subscriber(node,t,topic,qos_profile=qos_profile_sensor_data) for t,topic in (
        (Image,'/camera/camera/color/image_raw'),
        (Image,'/camera/camera/aligned_depth_to_color/image_raw'),
        (CameraInfo,'/camera/camera/color/camera_info'))]
    sync=TimeSynchronizer(subscribers,10);sync.registerCallback(frame)
    node.create_subscription(PointStamped,'/vision/ball_3d_position',lambda p:points.__setitem__(key(p.header),p),10)
    deadline=time.monotonic()+15
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
        matches=frames.keys()&points.keys()
        if not matches:continue
        stamp=max(matches);rgb,depth,info=frames[stamp];p=points[stamp]
        assert rgb.encoding=='rgb8' and depth.encoding=='16UC1'
        assert rgb.width==depth.width==info.width and rgb.height==depth.height==info.height
        assert p.header.frame_id==depth.header.frame_id
        z=p.point.z;u=round(info.k[0]*p.point.x/z+info.k[2]);v=round(info.k[4]*p.point.y/z+info.k[5])
        image=bridge.imgmsg_to_cv2(depth,'passthrough')
        patch=image[max(v-2,0):v+3,max(u-2,0):u+3];valid=patch[patch>0]
        assert len(valid)>0
        measured=float(np.median(valid))*.001
        assert abs(measured-z)<.15
        result={'encoding':depth.encoding,'units':'millimetres','resolution':[depth.width,depth.height],
                'frame_id':depth.header.frame_id,'matching_rgb_depth_info_point_stamp':list(stamp),
                'nonzero_depth_pixels':int(np.count_nonzero(image)),
                'football_pixel':[u,v],'football_depth_image_m':measured,'football_point_z_m':z}
        Path('/tmp/football-depth-smoke.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2));break
    else:raise AssertionError('No matched live depth/football sample')
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
