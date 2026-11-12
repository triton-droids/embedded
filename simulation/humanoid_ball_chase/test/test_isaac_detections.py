import threading

import numpy as np
import pytest

vision_msgs = pytest.importorskip('vision_msgs.msg')

from humanoid_ball_chase.isaac_detections import IsaacDetectionSource, network_to_image  # noqa: E402

W, H = 424, 240
K = np.array([[300.0, 0.0, W / 2], [0.0, 300.0, H / 2], [0.0, 0.0, 1.0]])


class Stamp:
    def __init__(self, sec, nanosec=0):
        self.sec, self.nanosec = sec, nanosec


def array(stamp, boxes):
    """boxes: (class_id, score, cx, cy, w, h) in network pixels."""
    msg = vision_msgs.Detection2DArray()
    msg.header.stamp.sec, msg.header.stamp.nanosec = stamp.sec, stamp.nanosec
    for cls, score, cx, cy, bw, bh in boxes:
        det = vision_msgs.Detection2D()
        hyp = vision_msgs.ObjectHypothesisWithPose()
        hyp.hypothesis.class_id, hyp.hypothesis.score = str(cls), score
        det.results.append(hyp)
        det.bbox.center.position.x, det.bbox.center.position.y = cx, cy
        det.bbox.size_x, det.bbox.size_y = bw, bh
        msg.detections.append(det)
    return msg


def to_network(u, v):
    scale = min(640 / W, 640 / H)
    return u * scale + (640 - W * scale) / 2, v * scale + (640 - H * scale) / 2


def test_network_to_image_inverts_centered_letterbox():
    cx, cy = to_network(100.0, 50.0)
    u, v, bw, bh = network_to_image(cx, cy, 30.0, 30.0, W, H)
    assert u == pytest.approx(100.0) and v == pytest.approx(50.0)
    assert bw == pytest.approx(30.0 / (640 / W))


def test_picks_the_requested_class_and_reads_depth():
    depth = np.full((H, W), 2.0)
    rgb = np.zeros((H, W, 3), np.uint8)
    stamp = Stamp(10)
    ball, robot = to_network(300.0, 120.0), to_network(100.0, 60.0)
    msg = array(stamp, [(0, 0.9, *ball, 40, 40), (2, 0.8, *robot, 80, 120)])
    source = IsaacDetectionSource(class_id=2, wait_s=0.0)
    source.on_detections(msg)
    u, v, x, y, z = source.detect(rgb, depth, K, stamp)
    assert (u, v) == pytest.approx((100.0, 60.0))
    assert z == pytest.approx(2.0) and x == pytest.approx((100.0 - W / 2) * 2.0 / 300.0)
    assert source.last_confidence == pytest.approx(0.8)
    ball_source = IsaacDetectionSource(class_id=0, wait_s=0.0)
    ball_source.on_detections(msg)
    assert ball_source.detect(rgb, depth, K, stamp)[:2] == pytest.approx((300.0, 120.0))


def test_waits_for_the_frame_and_gives_up():
    depth = np.full((H, W), 2.0)
    rgb = np.zeros((H, W, 3), np.uint8)
    source = IsaacDetectionSource(class_id=0, wait_s=1.0)
    stamp = Stamp(11)
    msg = array(stamp, [(0, 0.7, *to_network(200.0, 100.0), 40, 40)])
    threading.Timer(0.05, source.on_detections, args=(msg,)).start()
    assert source.detect(rgb, depth, K, stamp) is not None  # Arrives after the frame.
    late = IsaacDetectionSource(class_id=0, wait_s=0.05)
    assert late.detect(rgb, depth, K, Stamp(12)) is None
    no_ball = IsaacDetectionSource(class_id=0, wait_s=0.0)
    no_ball.on_detections(array(Stamp(13), [(2, 0.9, 320, 320, 50, 50)]))
    assert no_ball.detect(rgb, depth, K, Stamp(13)) is None


def test_locks_onto_the_instance_nearest_the_previous_target():
    """Two robots: start on the nearer one, keep it while the other scores higher."""
    rgb = np.zeros((H, W, 3), np.uint8)
    depth = np.full((H, W), 3.0)
    depth[:, :W // 2] = 2.0                      # Left robot nearer than the right one.
    left, right = to_network(100.0, 120.0), to_network(320.0, 120.0)
    now = [0.0]
    source = IsaacDetectionSource(class_id=2, wait_s=0.0, reacquire_s=1.0, clock=lambda: now[0])
    seen = []
    for i, (left_score, right_score) in enumerate([(0.5, 0.9), (0.4, 0.95), (0.6, 0.6)]):
        now[0] = 0.2 * i
        stamp = Stamp(20 + i)
        source.on_detections(array(stamp, [(2, left_score, *left, 60, 90), (2, right_score, *right, 60, 90)]))
        seen.append(source.detect(rgb, depth, K, stamp)[0])
    assert seen == pytest.approx([100.0, 100.0, 100.0])   # Never the higher-scoring right robot.
    # The left robot leaves; only the right one remains: follow it (nearest is the only one).
    now[0] = 0.8
    source.on_detections(array(Stamp(30), [(2, 0.9, *right, 60, 90)]))
    assert source.detect(rgb, depth, K, Stamp(30))[0] == pytest.approx(320.0)
    # Locked on the right one now, even when the left one comes back nearer to the camera.
    now[0] = 1.0
    source.on_detections(array(Stamp(31), [(2, 0.5, *left, 60, 90), (2, 0.5, *right, 60, 90)]))
    assert source.detect(rgb, depth, K, Stamp(31))[0] == pytest.approx(320.0)
    # After a gap longer than reacquire_s: the instance nearest the camera again.
    now[0] = 2.5
    source.on_detections(array(Stamp(32), [(2, 0.5, *left, 60, 90), (2, 0.5, *right, 60, 90)]))
    assert source.detect(rgb, depth, K, Stamp(32))[0] == pytest.approx(100.0)
