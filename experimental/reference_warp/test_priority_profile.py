"""Exercise actual Linux scheduling readback in disposable child processes."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

import priority_profile as profile


def test_cpu_partition_keeps_smt_siblings_together(tmp_path):
    for cpu, core in enumerate([0,1,2,0,1,2]):
        path = tmp_path/f'cpu{cpu}'/'topology'
        path.mkdir(parents=True)
        (path/'physical_package_id').write_text('0')
        (path/'core_id').write_text(str(core))
    assert profile.cpu_groups(range(6),tmp_path) == dict(imu=[0,3],wbc=[1,4],other=[2,5])
    assert profile.cpu_groups([0,1],tmp_path) == dict(imu=[0,1],wbc=[0,1],other=[0,1])


def test_explicit_fifo_denial_is_not_silent_fallback(monkeypatch):
    monkeypatch.setattr(profile,'fifo_available',lambda:False)
    with pytest.raises(RuntimeError, match='FIFO requested'):
        profile.make_profile('fifo')
    assert profile.make_profile('auto')['mode'] == 'nice'
    assert profile.make_profile('off') is None


@pytest.mark.parametrize('command,role',[
    (['/usr/bin/python3','/install/lib/safety_node','--ros-args'],'safety'),
    (['/install/lib/mujoco_sim'],'bootstrap'),
    (['python','/install/lib/wbc_control_node'],'wbc'),
    (['python','chase_policy.py'],'other')])
def test_command_classification(command,role):
    assert profile.classify(command) == role


def test_real_unprivileged_thread_hierarchy_and_affinity():
    config = profile.make_profile('nice')
    env = dict(os.environ, HUMANOID_PRIORITY_CONFIG=json.dumps(config))
    env['PYTHONPATH'] = str(Path(__file__).resolve().parent)+os.pathsep+env.get('PYTHONPATH','')
    code = '''import priority_profile,json,threading
out=[]
def run(role): out.append(priority_profile.configure_current_thread(role))
threads=[threading.Thread(target=run,args=(role,)) for role in ['imu','imu_source','safety','wbc','other']]
for t in threads:t.start()
for t in threads:t.join()
print(json.dumps(out))
'''
    child = subprocess.run([sys.executable,'-c',code],env=env,capture_output=True,text=True,check=True)
    rows = {v['role']:v for v in json.loads(child.stdout)}
    assert len(rows)==5
    for role in ('imu','imu_source','safety'):
        assert rows[role]['nice']==0
        assert rows[role]['cpus']==config['cpus']['imu']
    assert rows['wbc']['nice']==5
    assert rows['other']['nice']==10
    assert {v['policy'] for v in rows.values()} == {os.SCHED_OTHER}


def test_hook_is_inert_without_explicit_module(monkeypatch):
    from humanoid_sim.thread_setup import configure_thread
    monkeypatch.delenv('HUMANOID_THREAD_SCHEDULER',raising=False)
    assert configure_thread('imu') is None
