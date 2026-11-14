import numpy as np
from humanoid_ball_chase.foot_geometry import FootGeometry

class Robot:
    joint_names=['a','b'];na=2
    def state_from_sim(self,*args):
        self.positions=args[4].copy()
        return self.positions,None
    def forward(self,q):pass
    def sole_center(self,foot):return np.array([-.22,.05,-.65])

def test_joint_order_and_body_to_ros_base_conversion():
    robot=Robot();g=FootGeometry(robot=robot)
    assert g.observe(['b','a'],[2.,1.],10.,20.)
    p=g.point_base(10.01,20.01)
    assert np.allclose(robot.positions,[1.,2.])
    assert np.allclose(p,[.05,.22,-.65])

def test_stale_or_invalid_measurements_cannot_authorize_foot_position():
    g=FootGeometry(robot=Robot());g.observe(['a','b'],[1.,2.],10.,20.)
    assert g.point_base(10.2,20.01) is None
    assert not g.observe(['a','a'],[1.,2.],10.2,20.2)
    assert g.point_base(10.21,20.21) is None
