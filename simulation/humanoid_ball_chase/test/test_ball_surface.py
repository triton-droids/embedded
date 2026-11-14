import numpy as np
from humanoid_ball_chase.ball_surface import point_from_sphere


def fixture_depth():
    k=np.array([[120.,0.,79.5],[0.,120.,59.5],[0.,0.,1.]])
    yy,xx=np.mgrid[:120,:160]
    rays=np.stack(((xx-79.5)/120,(yy-59.5)/120,np.ones_like(xx)),axis=-1)
    center=np.array([.02,.03,.75]);r=.1
    a=(rays*rays).sum(axis=2);b=rays@center
    disc=b*b-a*(center@center-r*r)
    depth=np.full((120,160),1.2)
    hit=disc>=0;depth[hit]=(b[hit]-np.sqrt(disc[hit]))/a[hit]
    box=(82.7,64.3,40.,44.)
    return depth,k,center,box


def test_complete_and_partly_occluded_ball_recover_center():
    for occluded in (False,True):
        depth,k,center,box=fixture_depth()
        if occluded:depth[68:,:]=.60  # Foreground limb covers the lower ball.
        point=point_from_sphere(*box,depth,k,.1)
        assert point is not None
        assert np.linalg.norm(np.array(point[2:])-center)<.01


def test_planar_or_missing_depth_is_not_a_ball():
    depth,k,center,box=fixture_depth()
    for bad in (np.full_like(depth,.6),np.full_like(depth,np.nan)):
        assert point_from_sphere(*box,bad,k,.1) is None
