"""Small full-body recovery checks without ROS or installed description packages."""
from pathlib import Path
import sys

import mujoco
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / 'simulation/humanoid_sim'),
                str(ROOT / 'humanoid_control/walking_controller/tools')]

from recovery_eval import description_paths, lying_pose, make_controller
from walking_controller.offline import OfflineSim, load
from walking_controller.recovery import GetUpPlanner, RecoveryParams


@pytest.fixture(scope='module')
def recovery():
    paths, robot, limits = load(description_paths(ROOT / 'humanoid_full_body_description'))
    ctl = make_controller(robot, limits)

    def build(dt):
        return GetUpPlanner(paths['robot_xml'], paths['mesh_dir'], paths['registry'],
                            robot.joint_names, ctl.q_nominal, limits.torque,
                            RecoveryParams(samples=2, threads=1, rollout_dt=dt))

    return paths, robot, limits, build(0.0), build(0.01)


def test_rollout_grid(recovery):
    _, _, _, normal, fast = recovery
    assert fast.nstep * 2 == normal.nstep
    assert normal.rollout_m is normal.m
    assert fast.rollout_m is not fast.m
    assert fast.m.opt.timestep == normal.m.opt.timestep == 0.005
    assert fast.rollout_m.opt.timestep == 0.01
    assert (fast.m.nq, fast.m.nv) == (fast.rollout_m.nq, fast.rollout_m.nv)
    assert fast._targets(fast.nominal[None]).shape[1] == fast.nstep


def test_loop_closure(recovery):
    planner = recovery[3]
    m, d = planner.m, planner.d
    assert len(planner.loops) == 2
    for ankle, crank, rod, grid, table in planner.loops:
        side = m.joint(int(np.nonzero(m.jnt_qposadr == ankle)[0][0])).name.split('_')[0]
        eq = m.eq(f'{side}_ankle_linkage').id
        for k in range(len(grid)):
            d.qpos[ankle], d.qpos[crank], d.qpos[rod] = grid[k], *table[k]
            gap = planner._gap(m.eq_obj1id[eq], m.eq_data[eq, :3],
                               m.eq_obj2id[eq], m.eq_data[eq, 3:6])
            assert np.linalg.norm(gap) < 1e-6


@pytest.mark.parametrize('index', [3, 4])
def test_lying_plan(recovery, index):
    paths, robot, limits = recovery[:3]
    planner = recovery[index]
    sim = OfflineSim(robot, limits, paths)
    lying_pose(sim, planner, 'supine')
    d = sim.data
    ang = np.zeros(3)
    mujoco.mju_rotVecQuat(ang, d.qvel[3:6], d.qpos[3:7])
    planner.reset('rise', sim.time, d.qpos[sim.qpos])
    planner.set_state(d.qpos[:3], d.qpos[3:7], d.qvel[:3], ang,
                      d.qpos[sim.qpos], d.qvel[sim.qvel])
    assert planner.plan(sim.time, sim.foot_forces()) in (None, 'straight')
    for t in np.linspace(0, planner.p.horizon_s, 21):
        target = planner.target(t)
        assert np.isfinite(target).all()
        assert (target >= planner.lo - 1e-12).all()
        assert (target <= planner.hi + 1e-12).all()
