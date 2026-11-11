"""Exercise real ROS message decoding, TF rejection and the command watchdog."""
import time

import cv2
import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from humanoid_ball_chase.vision_node import BallChaseVision
from rclpy._rclpy_pybind11 import RCLError
from unittest.mock import Mock
from itertools import chain, repeat


@pytest.fixture
def node():
    rclpy.init(args=['--ros-args','-p','detector:=orange_test','-p','world_model:=false'])
    n=BallChaseVision()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def frame(node):
    rgb=np.zeros((120,160,3),np.uint8)
    cv2.circle(rgb,(80,60),18,(255,100,0),-1)
    bridge=CvBridge()
    color=bridge.cv2_to_imgmsg(rgb,encoding='rgb8')
    depth=bridge.cv2_to_imgmsg(np.full((120,160),2000,np.uint16),encoding='16UC1')
    info=CameraInfo();info.width=160;info.height=120
    info.k=[100.,0.,79.5,0.,100.,59.5,0.,0.,1.]
    for msg in (color,depth,info):
        msg.header.stamp=node.get_clock().now().to_msg() if msg is color else color.header.stamp
        msg.header.frame_id='camera_color_optical_frame'
    t=TransformStamped();t.header.stamp=color.header.stamp;t.header.frame_id='base_link'
    t.child_frame_id=color.header.frame_id
    # optical z -> base x, optical x -> -base y, optical y -> -base z.
    t.transform.rotation.x=-.5;t.transform.rotation.y=.5
    t.transform.rotation.z=-.5;t.transform.rotation.w=.5
    node._tf_buffer.set_transform(t,'test')
    return color,depth,info


def test_actual_messages_track_then_loss_and_stale_stop(node):
    color,depth,info=frame(node)
    node._process_frame(color,depth,info)
    assert node._valid
    assert node._target.vx>0
    node._control_tick()
    # Receipt watchdog uses monotonic time independently of source ROS time.
    from dataclasses import replace
    node._target=replace(node._target,receipt_monotonic=time.monotonic()-1)
    node._control_tick()
    assert not node._valid
    color,depth,info=frame(node)
    node._process_frame(color,depth,info)
    assert node._valid
    color.data=bytes(len(color.data))
    node._process_frame(color,depth,info)
    assert not node._valid


def test_command_publish_failure_latches_zero_even_after_new_detection(node):
    color, depth, info = frame(node)
    node._process_frame(color, depth, info)
    assert node._valid
    node._command_publisher = Mock()
    node._command_publisher.publish.side_effect = chain([RCLError('injected')], repeat(None))
    node._control_tick()
    assert not node._valid
    color, depth, info = frame(node)
    node._process_frame(color, depth, info)
    node._control_tick()
    command = node._command_publisher.publish.call_args.args[0]
    assert command.twist.linear.x == 0
    assert command.twist.angular.z == 0


def test_frame_mismatch_and_missing_tf_cannot_keep_previous_target(node):
    color,depth,info=frame(node)
    node._process_frame(color,depth,info);assert node._valid
    depth.header.frame_id='wrong_frame'
    node._process_frame(color,depth,info);assert not node._valid
    color,depth,info=frame(node)
    for m in (color,depth,info):m.header.frame_id='unknown_camera'
    node._process_frame(color,depth,info);assert not node._valid


def test_future_and_old_source_images_rejected(node):
    for seconds in (-2,2):
        color,depth,info=frame(node)
        color.header.stamp.sec+=seconds
        depth.header.stamp=color.header.stamp;info.header.stamp=color.header.stamp
        node._process_frame(color,depth,info)
        assert not node._valid


def test_inference_discards_intermediate_frames_without_blocking_watchdog(node):
    import threading
    entered=threading.Event();release=threading.Event();done=threading.Event()
    processed=[]
    def slow_frame(color,depth,info,receipt=None):
        processed.append(color)
        if color==1:
            entered.set();release.wait(1)
        if color==3:done.set()
    node._process_frame=slow_frame
    node._on_frame(1,None,None)
    assert entered.wait(1)
    node._on_frame(2,None,None)
    node._on_frame(3,None,None)
    node._control_tick()  # Still runs while inference is blocked.
    release.set()
    assert done.wait(1)
    assert processed==[1,3]


def test_confidence_describes_accepted_frame_while_next_inference_is_running(node):
    from humanoid_ball_chase import core
    class Detector:
        last_confidence=.91
        detect=staticmethod(core.detect_ball)
    node._detector=Detector()
    node._process_frame(*frame(node))
    assert node._valid
    node._detector.last_confidence=0.0  # Next inference has not finished yet.
    status=node._status_locked(node.get_clock().now().nanoseconds*1e-9,time.monotonic(),.2,0.)
    assert status['detection_confidence']==.91


def test_cpu_budget_accepts_bounded_latency_and_still_expires(node):
    from humanoid_ball_chase.vision_node import _FrameRejected
    node.TARGET_TIMEOUT_S=1.0
    now=node.get_clock().now().nanoseconds*1e-9
    node._check_timing(now-.5,time.monotonic())
    with pytest.raises(_FrameRejected):node._check_timing(now-1.1,time.monotonic())
    with pytest.raises(_FrameRejected):node._check_timing(now,time.monotonic()-1.1)


# ----------------------------------------------------------------- world model
@pytest.fixture
def wm_node():
    rclpy.init(args=['--ros-args','-p','detector:=orange_test'])
    n=BallChaseVision()
    t=TransformStamped();t.header.frame_id='odom';t.child_frame_id='base_link'
    t.transform.translation.x=0.3;t.transform.rotation.w=1.0
    n._tf_buffer.set_transform_static(t,'test')
    yield n
    n.destroy_node()
    rclpy.shutdown()


def blank(color):
    color.data=bytes(len(color.data))
    return color


def test_world_model_confirms_then_coasts_through_misses(wm_node):
    n=wm_node
    assert n.world_model
    n._process_frame(*frame(n))
    assert n._tracker.status==0  # tentative: one sighting does not drive
    vx,wz,chase,_=n._world_command(n._tracker.peek(n.get_clock().now().nanoseconds*1e-9),
                                   n.get_clock().now().nanoseconds*1e-9)
    assert chase.mode=='none' and vx==0
    for _ in range(4):  # 6 Hz spacing so the velocity estimate settles
        time.sleep(1/6)
        n._process_frame(*frame(n))
    assert n._tracker.status==1
    out=n._tracker.peek(n._tracker.kf.t)
    # Depth 2.0 m hits the front surface; the center is 2.1 m ahead, plus odom offset 0.3.
    assert out.x[:2]==pytest.approx([2.4,0.0],abs=0.02)
    now=n._tracker.kf.t
    vx,wz,chase,reason=n._world_command(out,now)
    assert reason=='ok' and chase.mode=='tracking' and vx>0 and abs(wz)<0.05

    n._head_publisher=Mock()
    n._command_publisher=Mock()
    color,depth,info=frame(n)
    n._process_frame(blank(color),depth,info)
    assert not n._valid
    n._command_publisher.publish.assert_not_called()  # a miss does not stop at once
    assert n._tracker.status==2
    head=n._head_publisher.publish.call_args.args[0]  # head aims at the prediction
    # Prediction is the ball center one radius above the floor; this test camera sits on the floor.
    assert head.point.x==pytest.approx(80.0,abs=1.0)
    assert head.point.y==pytest.approx(59.5-100*0.1/2.1,abs=1.0)

    t=n._tracker.kf.t+1.0
    vx_c,_,chase,_=n._world_command(n._tracker.peek(t),t)
    assert chase.mode=='coasting' and 0<vx_c<vx
    t=n._tracker.kf.t+5.0
    assert n._tracker.peek(t).status==3
    assert n._world_command(n._tracker.peek(t),t)[:2]==(0.0,0.0)


def test_world_model_status_and_missing_odom_tf(wm_node):
    n=wm_node
    for _ in range(2):
        n._process_frame(*frame(n))
    n._control_tick()
    n.odom_frame='odom_missing'
    n._process_frame(*frame(n))  # no odom TF: rejected, nothing chased
    assert not n._valid
    n._command_publisher=Mock()
    n._control_tick()
    command=n._command_publisher.publish.call_args.args[0]
    assert command.twist.linear.x==0 and command.twist.angular.z==0
    status=n._status_locked(0.0,time.monotonic(),0.,0.)
    assert 'world_model' not in status  # added by the timer only
