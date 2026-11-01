"""Straight-approach scenario reusing the full D435/football scene."""
from geometry_msgs.msg import PointStamped
from humanoid_ball_chase.ball_control import BallControl
from humanoid_ball_chase.scene import build_scene as camera_scene


def build_scene(spec, options):
    camera_scene(spec, options)
    spec.body('chase_ball').pos = [0.0, 1.8, 0.11]


class StraightBallControl(BallControl):
    def reset_ball(self, request, response):
        msg = PointStamped()
        msg.header.frame_id = 'world'
        msg.point.y, msg.point.z = 1.8, 0.11
        self.place(msg)
        response.success = True
        response.message = 'Experimental straight-approach ball reset'
        return response


def create_nodes(world):
    return [StraightBallControl(world)]
