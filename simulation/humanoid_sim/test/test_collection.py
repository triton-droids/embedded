"""Exercise real force/contact evolution and future-label censoring."""
from types import SimpleNamespace
import json

import numpy as np
import pytest

pytest.importorskip('mujoco')
from humanoid_sim.collection import EpisodeRecorder
from test_world import make_world, run


def test_force_is_bounded_and_does_not_apply_on_gantry(tmp_path):
    w = make_world()
    r = EpisodeRecorder(w, tmp_path/'episode', seed=1)
    w.recorder = r
    run(w, .1)
    assert r.released_at is None
    assert not np.any(w.data.xfrc_applied)
    w.set_base_held(False)
    w.step(.1)
    assert r.released_at is not None
    r.push_delay = 0.
    r.push_duration = .05
    w.step(.105)
    assert np.linalg.norm(w.data.xfrc_applied[r.base,:3]) > 0
    run(w, .1, start=.11)
    assert not np.any(w.data.xfrc_applied)
    r.finish('interrupted')
    assert not np.any(w.data.xfrc_applied)


def test_real_collapse_records_confirmed_contact_and_both_labels(tmp_path):
    w = make_world()
    r = EpisodeRecorder(w, tmp_path/'episode', seed=3)
    w.recorder = r
    w.set_base_held(False)
    for i in range(800):
        w.step(i*.005)
        if i % 2 == 0:
            sample = w.body_imu()
            r.record_imu(sample, sample.gyro, sample.accel, SimpleNamespace(sec=1,nanosec=i*1000000))
        if r.done:
            break
    assert r.reason == 'fall'
    assert r.fall_time is not None
    m = r.save()
    rows = [json.loads(s) for s in (tmp_path/'episode/samples.jsonl').read_text().splitlines()]
    assert m['label_counts']['1'] > 0
    assert m['label_counts']['0'] > 0
    assert all(row['label'] is None for row in rows if row['sim_time_s'] >= r.fall_time)
    assert all(0 < r.fall_time-row['sim_time_s'] <= .3+1e-9 for row in rows if row['label']==1)


def test_interruption_censors_incomplete_future_and_invalid_history(tmp_path):
    w = make_world()
    r = EpisodeRecorder(w, tmp_path/'episode', seed=3)
    r.released_at = 0.
    for i in range(100):
        r.rows.append(dict(sim_time_s=i*.01,base_held=False,control_fault=None,estop=False,features=[0.]*8))
    r.rows[45]['base_held'] = True
    r.finish('interrupted')
    r.save()
    assert all(row['label'] is None for row in r.rows[-30:])
    assert all(r.rows[i]['label'] is None for i in range(45,65))
    assert any(row['label']==0 for row in r.rows)


def test_labels_include_exact_numerical_boundaries():
    from humanoid_sim.collection_rules import future_label
    assert future_label(1.,1.3000000000000469,2.,.3)==1
    assert future_label(1.,1.6000000000000469,2.,.6)==1
    assert future_label(1.,1.301,2.,.3)==0
    assert future_label(1.,None,1.599,.6) is None
    assert future_label(1.,1.,2.,.6) is None


def test_recovery_scenarios_are_reproducible_and_have_48_families():
    from humanoid_sim.collection_rules import recovery_scenario
    assert recovery_scenario(48001)==recovery_scenario(48001)
    assert len({recovery_scenario(i)['family'] for i in range(48)})==48
    assert {recovery_scenario(i)['friction'] for i in range(48)}=={.65,.85,1.15}
