import math
from humanoid_ball_chase.shoot import GoalShot
from humanoid_ball_chase.kick import WalkState

def relative(point,pose):
 x,y,a=pose; dx,dy=point[0]-x,point[1]-y
 return math.cos(a)*dx+math.sin(a)*dy,-math.sin(a)*dx+math.cos(a)*dy

def test_reaches_goal_directed_kick_from_varied_positions():
 for ball,goal,pose in [((.95,.35),(1.8,-.35),(0.,0.,0.)),((.95,-.35),(1.8,.35),(0.,0.,0.)),((.95,.1),(1.8,.35),(0.,0.,.35)),((.1,0.),(1.8,.35),(.8,0.,0.))]:
  shot=GoalShot(); pose=list(pose); requested=False
  for i in range(2400):
   b,g=relative(ball,pose),relative(goal,pose)
   decision=shot.update(i*.05,b,g,0.,.02,WalkState(True,None,0,i*.05))
   assert 0<=decision.vx<=.10 and abs(decision.wz)<=.3
   if decision.kick_foot:
    wanted=math.atan2(goal[1]-ball[1],goal[0]-ball[0])
    assert abs(math.remainder(pose[2]-wanted,2*math.pi))<=shot.max_heading_error
    assert decision.kick_foot=='right_foot'
    requested=True; break
   pose[2]+=decision.wz*.05
   pose[0]+=decision.vx*math.cos(pose[2])*.05;pose[1]+=decision.vx*math.sin(pose[2])*.05
  assert requested,(ball,goal,pose,shot.status())

def test_unusable_ball_cannot_move_or_kick():
 s=GoalShot()
 for ball,goal,std in [(None,(2,0),.02),((1,0),(2,0),float('inf')),((float('nan'),0),(2,0),.02)]:
  d=s.update(1,ball,goal,0,std,WalkState(True,None,0,1))
  assert (d.vx,d.wz,d.kick_foot)==(0,0,None)

def test_misaligned_goal_cannot_request_kick_even_when_ball_is_in_window():
 s=GoalShot(state='approach')
 for i in range(10):
  d=s.update(i*.1,(.23,.16),(1.23,1.16),0,.02,WalkState(True,None,0,i*.1))
  assert d.kick_foot is None


def test_completed_and_timed_out_attempts_hold_zero():
 for s,t in [(GoalShot(state='done'),1),(GoalShot(started_at=0),121)]:
  d=s.update(t,(1,0),(2,0),0,.02,WalkState(True,None,0,t))
  assert (d.vx,d.wz,d.kick_foot)==(0,0,None)


def test_shoot_fixture_matches_known_world_goal():
 from pathlib import Path
 import mujoco
 import numpy as np
 from humanoid_sim import model_builder as mb
 repo=Path(__file__).resolve().parents[3]
 hybrid=repo/'control/motor_control_hybrid'
 import yaml
 from humanoid_sim.motor_model import limits_from_registry
 limits=limits_from_registry(yaml.safe_load((hybrid/'config/motors.yaml').read_text())['motor_control_node']['ros__parameters'])
 model=mb.build_model(mb.WorldOptions(
  robot_xml=str(hybrid/'models/20260914_170827/chrobot_16kg_candidate.xml'),
  mesh_dir=str(repo/'description/humanoid_leg_description/meshes/robot_meshes'),
  joint_torque_limits={name:lim.max_torque for name,lim in limits.items()},
  scene_builder='humanoid_ball_chase.scene:build_scene_shoot'))
 data=mujoco.MjData(model);mujoco.mj_forward(model,data)
 assert np.allclose(data.body('chase_ball').xpos[:2],[-.35,.95])
 posts=[model.geom('shot_post_0').pos,model.geom('shot_post_1').pos]
 assert np.allclose((posts[0]+posts[1])[:2]/2,[.35,1.8])
 assert np.isclose(posts[1][0]-posts[0][0],1.3)
 assert model.joint('chase_ball_free').type[0]==mujoco.mjtJoint.mjJNT_FREE


def test_alignment_waits_for_real_standing_before_approach():
 s=GoalShot(state="align",aligned_since=0)
 d=s.update(1.,(.65,.16),(1.65,.16),0,.02,WalkState(False,None,0,1.))
 assert s.state=="align"
 assert (d.vx,d.wz,d.kick_foot)==(0,0,None)


def test_alignment_rechecks_position_after_turning_displaced_body():
 s=GoalShot(state="align",aligned_since=0)
 d=s.update(1.,(.65,.40),(1.65,.40),0,.02,WalkState(True,None,0,1.))
 assert s.state=="stage"
 assert (d.vx,d.wz,d.kick_foot)==(0,0,None)



def test_pose_approach_handles_recorded_lateral_offsets_without_wrong_heading_kick():
 for ball,goal in [((.634,.276),(1.634,.238)),((.69,.234),(1.69,.214))]:
  shot=GoalShot(state='approach');pose=[0.,0.,0.];requested=False
  for i in range(1200):
   b,g=relative(ball,pose),relative(goal,pose)
   d=shot.update(i*.05,b,g,0.,.02,WalkState(True,None,0,i*.05))
   if d.kick_foot:
    assert abs(shot.heading_error)<=.08
    requested=True;break
   pose[2]+=d.wz*.05
   pose[0]+=d.vx*math.cos(pose[2])*.05;pose[1]+=d.vx*math.sin(pose[2])*.05
  assert requested,shot.status()


def test_final_support_shift_uses_existing_settle_hysteresis():
 s=GoalShot(state='approach')
 d=s.update(1.,(.23,.16),(1.23,.16),0,.02,WalkState(False,None,0,1.))
 assert s.state=='settle' and d.kick_foot is None
 # The gait shifts the body 3cm forward / 4cm sideways before standing.
 d=s.update(1.1,(.26,.20),(1.26,.20),0,.02,WalkState(False,None,0,1.1))
 assert s.state=='settle' and d.kick_foot is None
 s.update(1.2,(.26,.20),(1.26,.20),0,.02,WalkState(True,None,0,1.2))
 d=s.update(1.6,(.26,.20),(1.26,.20),0,.02,WalkState(True,None,0,1.6))
 assert d.kick_foot=='right_foot'


def test_terminal_forward_motion_uses_kick_window_error():
 s=GoalShot(state='approach')
 ball=(.263,.127);heading=-.11
 goal=(ball[0]+math.cos(heading),ball[1]+math.sin(heading))
 d=s.update(1.,ball,goal,0,.02,WalkState(False,None,0,1.))
 from walking_controller.velocity import step_command
 from walking_controller.stepping import StepParams
 params=StepParams()
 step,turn=step_command(d.vx,d.wz,params.t_ds+params.t_ss,.06,.08,.04,.10)
 assert step>.005  # Must remain above the real walking-node rest-step threshold.
 assert d.wz<0  # Both the goal and foot alignment need a right turn.
 assert d.kick_foot is None


def test_kick_completion_does_not_require_seeing_the_departed_ball():
 s=GoalShot(state='kick')
 s.approach.state='kick';s.approach.since=0.;s.approach.kicks_at_request=0
 d=s.update(1.,None,None,0,float('inf'),WalkState(True,None,1,1.))
 assert s.state=='done' and d.kick_foot is None


def test_missing_kick_acknowledgement_is_not_reported_as_success():
 s=GoalShot(state='kick')
 s.approach.state='kick';s.approach.since=0.
 d=s.update(4.,None,None,0,float('inf'),WalkState(True,None,0,4.))
 assert s.state=='fault' and 'acknowledged' in s.fault and d.kick_foot is None


def test_kick_window_follows_observed_foot_not_nominal_body_offset():
 s=GoalShot(state='approach')
 foot=(.05,.22);ball=(.25,.22);goal=(1.25,.22)
 d=s.update(1.,ball,goal,0,.02,WalkState(False,None,0,1.),foot_xy=foot)
 assert s.state=='settle' and d.kick_foot is None
 s.update(1.1,ball,goal,0,.02,WalkState(True,None,0,1.1),foot_xy=foot)
 d=s.update(1.5,ball,goal,0,.02,WalkState(True,None,0,1.5),foot_xy=foot)
 assert d.kick_foot=='right_foot'
 assert s.approach.last_errors==(0.,0.)
