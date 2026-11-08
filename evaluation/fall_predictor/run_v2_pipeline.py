#!/usr/bin/env python3
"""Bounded unattended recovery collection -> nested-horizon training -> sealed test."""
import argparse
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


def save(path,value):
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(value,indent=2)+'\n');temp.replace(path)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--episodes',type=int,default=384)
    ap.add_argument('--hours',type=float,default=6.5)
    ap.add_argument('--collection-hours',type=float,default=2.5)
    args=ap.parse_args()
    if not 48<=args.episodes<=1000 or not 0<args.collection_hours<args.hours<=12:
        ap.error('invalid bounded budget')
    repo=Path(__file__).resolve().parents[2]
    root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
    status=dict(pid=os.getpid(),state='starting',started_utc=datetime.now(timezone.utc).isoformat(),
                hours=args.hours,planned_episodes=args.episodes,child_pid=None)
    started=time.monotonic();deadline=started+args.hours*3600
    stop=False;child=None
    def signal_stop(signum,frame):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGTERM,signal_stop);signal.signal(signal.SIGINT,signal_stop)
    def update():
        status.update(elapsed_s=time.monotonic()-started,remaining_s=max(0,deadline-time.monotonic()))
        save(root/'status.json',status)
    update()
    # Freeze the training implementation before any new test outcomes exist.
    relative_files=['evaluation/fall_predictor/run_v2_pipeline.py','evaluation/fall_predictor/train_dual.py',
        'evaluation/fall_predictor/train.py','evaluation/fall_predictor/autotrain.py',
        'simulation/humanoid_sim/humanoid_sim/__init__.py',
        'simulation/humanoid_sim/humanoid_sim/collection_rules.py']
    for relative in relative_files:
        target=root/'source/embedded'/relative;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(repo/relative,target)
    # Sealed family allocation is written before starting collection.
    sys.path.insert(0,str(repo/'evaluation/fall_predictor'))
    from train_dual import scenario_folds
    save(root/'sealed_scenario_folds.json',scenario_folds())
    old_source=repo/'data/fall_collection/overnight_20261107'
    old_splits=repo/'data/fall_training/20261107_v1/splits.json'
    save(root/'run.json',dict(status=status,arguments=vars(args)|{'output':str(root)},
        scope='camera off; active simulator stack; collect -> train two horizons -> one new test evaluation',
        quality_gates={'train_recovered':8,'validation_recovered':2,'test_recovered':2}))
    collection=['bash',str(repo/'scripts/run_fall_collection.sh'),'--profile','recovery',
        '--hours',str(args.collection_hours),'--episode-seconds','20','--max-episodes',str(args.episodes),
        '--seed','48048','--domain','88','--output',str(root/'collection')]
    train=[sys.executable,str(root/'source/embedded/evaluation/fall_predictor/train_dual.py'),
        '--legacy-source',str(old_source),'--legacy-splits',str(old_splits),
        '--new-source',str(root/'collection'),'--output',str(root/'training')]
    try:
        for stage,cmd in [('collecting',collection),('training_and_sealed_evaluation',train)]:
            if stop or time.monotonic()>=deadline:break
            if shutil.disk_usage(root).free<3*1024**3:
                status['state']='disk_space_low';break
            status.update(state=stage,command=cmd);update()
            with (root/(stage+'.log')).open('w') as log:
                child=subprocess.Popen(cmd,cwd=repo,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                status['child_pid']=child.pid;update()
                while child.poll() is None and not stop and time.monotonic()<deadline:
                    time.sleep(2);update()
                if child.poll() is None:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=25)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid,signal.SIGKILL);child.wait()
                code=child.returncode;child=None;status['child_pid']=None
                if code!=0:
                    status.update(state='stage_failed',failed_stage=stage,returncode=code);break
                if stop or time.monotonic()>=deadline:break
        if status['state'] in ['collecting','training_and_sealed_evaluation']:
            status['state']='stopped' if stop else ('time_budget_exhausted' if time.monotonic()>=deadline else 'complete')
    except Exception:
        import traceback
        status['state']='error';(root/'error.txt').write_text(traceback.format_exc());raise
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGTERM)
            try:child.wait(timeout=25)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL)
        status.update(ended_utc=datetime.now(timezone.utc).isoformat(),child_pid=None);update()
        print(json.dumps(status),flush=True)


if __name__=='__main__':main()
