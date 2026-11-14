"""Read-only kicking-foot geometry from the existing model and measured joints."""
import threading
import time
import numpy as np


class FootGeometry:
    def __init__(self, urdf_path='', forward_axis='positive_y', robot=None):
        if robot is None:
            from pathlib import Path
            from ament_index_python.packages import get_package_share_directory
            from walking_controller.robot import RobotModel
            path=urdf_path or str(Path(get_package_share_directory('humanoid_leg_description'))/'urdf/human_offset_corrected.urdf')
            robot=RobotModel(path)
        from walking_controller.leg_odometry import body_from_base
        self.robot=robot
        self.base_from_body=body_from_base(forward_axis).T
        self._lock=threading.Lock()
        self._sample=None
        self.reason='no_joint_sample'

    def observe(self,names,positions,stamp,receipt=None):
        names=list(names)
        valid=(len(names)==len(set(names)) and len(names)==len(positions)
               and all(n in names for n in self.robot.joint_names)
               and np.isfinite(stamp) and stamp>0)
        if valid:
            index={name:i for i,name in enumerate(names)}
            q=np.array([positions[index[n]] for n in self.robot.joint_names],float)
            valid=bool(np.isfinite(q).all())
        with self._lock:
            self._sample=(q,float(stamp),time.monotonic() if receipt is None else receipt) if valid else None
        return valid

    def point_base(self,now_ros,now_mono=None,max_age=.15):
        with self._lock:sample=self._sample
        if sample is None:
            self.reason='no_valid_joint_sample';return None
        positions,stamp,receipt=sample
        now_mono=time.monotonic() if now_mono is None else now_mono
        if not 0<=now_ros-stamp<=max_age or not 0<=now_mono-receipt<=max_age:
            self.reason='stale_joint_sample';return None
        q,_=self.robot.state_from_sim(np.zeros(3),np.array([0.,0.,0.,1.]),np.zeros(3),np.zeros(3),positions,np.zeros(self.robot.na))
        self.robot.forward(q)
        point=self.base_from_body@self.robot.sole_center('right_foot')
        if not np.isfinite(point).all():
            self.reason='invalid_foot_geometry';return None
        self.reason='ok'
        return point.copy()
