#!/usr/bin/env python3
"""Bounded unattended CPU search. Uses fixed train/validation folds; never tests.

Simulation labels remain supervision. Outputs advisory model artifacts only.
"""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import random
import shutil
import signal
import time
import traceback

import numpy as np
import torch
from torch import nn

from train import FEATURES, load_data, ranking_metrics, write_json


class Candidate(nn.Module):
    def __init__(self, mean, std, architecture='base', dropout=0.):
        super().__init__()
        self.register_buffer('mean',torch.as_tensor(mean,dtype=torch.float32))
        self.register_buffer('std',torch.as_tensor(std,dtype=torch.float32))
        self.convolutional = architecture=='cnn'
        if self.convolutional:
            self.net=nn.Sequential(nn.Conv1d(8,16,5,padding=2),nn.ReLU(),
                nn.Conv1d(16,16,3,padding=1),nn.ReLU(),nn.Flatten(),
                nn.Linear(320,32),nn.ReLU(),nn.Dropout(dropout),nn.Linear(32,1))
        else:
            widths={'small':[32,16],'base':[64,32],'wide':[128,64]}[architecture]
            self.net=nn.Sequential(nn.Flatten(),nn.Linear(160,widths[0]),nn.ReLU(),
                nn.Dropout(dropout),nn.Linear(widths[0],widths[1]),nn.ReLU(),
                nn.Linear(widths[1],1))

    def forward(self,x):
        z=(x-self.mean)/self.std
        if self.convolutional: z=z.transpose(1,2)
        return self.net(z).squeeze(-1)


def alarm_mask(scores,groups,times,threshold,confirm):
    """Require k consecutive scored windows, reset at a gap or episode change."""
    raw=scores>=threshold
    alarm=raw.copy()
    edge=(groups[1:]==groups[:-1]) & (np.diff(times)>0) & (np.diff(times)<=.020001)
    breaks=np.r_[0,np.cumsum(~edge)]
    for k in range(1,confirm):
        alarm[k:] &= raw[:-k] & (groups[k:]==groups[:-k]) & (breaks[k:]==breaks[:-k])
        alarm[:k]=False
    return alarm


def policy_metrics(y,scores,groups,times,episodes,threshold,confirm):
    alarm=alarm_mask(scores,groups,times,threshold,confirm)
    eligible=np.unique(groups[y==1])
    detected=np.unique(groups[(y==1)&alarm])
    recall=len(detected)/max(1,len(eligible))
    dt=np.r_[.01,np.diff(times)]
    edge=np.r_[False,(groups[1:]==groups[:-1]) & (np.diff(times)>0) & (np.diff(times)<=.020001)]
    duration=np.where(edge,dt,.01)
    exposure=float(duration[y==0].sum())
    previous=np.r_[False,alarm[:-1]] & edge
    onset=np.flatnonzero(alarm & ~previous & (y==0))
    last={};false_alarms=0
    for i in onset:
        g=int(groups[i]);t=float(times[i])
        if t-last.get(g,-math.inf)>=1.:
            false_alarms+=1;last[g]=t
    advances=[];leads=[]
    for group in detected:
        i=np.flatnonzero((groups==group)&(y==1)&alarm)[0]
        ep=episodes[int(group)]
        leads.append(ep['fall_time']-float(times[i]))
        if ep['first_estop_time'] is not None:
            advances.append(ep['first_estop_time']-float(times[i]))
    before=float(np.median(advances)) if advances else 0.
    tp=int(np.sum(alarm & (y==1)));fp=int(np.sum(alarm & (y==0)))
    return dict(threshold=float(threshold),confirm_frames=int(confirm),
        eligible_fall_episodes=len(eligible),detected_fall_episodes=len(detected),event_recall=recall,
        window_recall=tp/max(1,int(y.sum())),precision=tp/max(1,tp+fp),
        false_positive_windows=fp,false_alarm_onsets=false_alarms,
        negative_exposure_minutes=exposure/60,
        false_alarm_onsets_per_hour=false_alarms/max(exposure/3600,1e-9),
        median_before_safety_estop_ms=before*1000,
        median_contact_lead_ms=float(np.median(leads)*1000) if leads else None,
        qualifies=bool(recall>=.95 and before>=.05))


def policy_key(metrics):
    # Hard validation gates, then minimize false alarms; no test-set feedback.
    return (metrics['qualifies'],
            -metrics['false_alarm_onsets_per_hour'] if metrics['qualifies'] else metrics['event_recall'],
            metrics['event_recall'],metrics['median_before_safety_estop_ms'])


def select_policy(y,scores,groups,times,episodes):
    positive=scores[y==1]
    thresholds=np.unique(np.r_[np.quantile(positive,[0,.01,.025,.05,.075,.1,.15,.2,.3]),
                               [.3,.5,.7,.85,.95,.99]])
    policies=[policy_metrics(y,scores,groups,times,episodes,t,c)
              for t in thresholds for c in (1,3,5)]
    return max(policies,key=policy_key),policies


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--fixed-splits',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--hours',type=float,default=6.5)
    ap.add_argument('--max-trials',type=int,default=96)
    ap.add_argument('--stagnation',type=int,default=24)
    ap.add_argument('--max-epochs',type=int,default=50)
    ap.add_argument('--seed',type=int,default=20261108)
    args=ap.parse_args()
    if not math.isfinite(args.hours) or not 0<args.hours<=24 or min(args.max_trials,args.max_epochs,args.stagnation)<=0:
        ap.error('invalid budget')
    args.output.mkdir(parents=True,exist_ok=True)
    lock=(args.output/'run.lock').open('w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if (args.output/'status.json').exists():
        ap.error('Output already contains a run; choose a fresh directory')
    stop=False
    def on_stop(signum,frame):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGTERM,on_stop);signal.signal(signal.SIGINT,on_stop)
    started=time.monotonic();deadline=started+args.hours*3600
    torch.set_num_threads(3);torch.set_num_interop_threads(1)
    torch.use_deterministic_algorithms(True)
    status=dict(pid=os.getpid(),state='loading',started_utc=datetime.now(timezone.utc).isoformat(),
        hours=args.hours,trials_completed=0,failed_trials=0,best=None,
        stop_rules=dict(max_trials=args.max_trials,unimproved_trials=args.stagnation),
        test_evaluated=False,elapsed_s=0.)
    def save_status():
        status.update(elapsed_s=time.monotonic()-started,remaining_s=max(0.,deadline-time.monotonic()))
        write_json(args.output/'status.json',status)
    save_status()
    shutil.copy2(Path(__file__),args.output/'autotrain_source.py')
    shutil.copy2(Path(__file__).with_name('train.py'),args.output/'train_source.py')
    shutil.copy2(args.fixed_splits,args.output/'splits.json')
    write_json(args.output/'run.json',dict(arguments={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
        torch_version=torch.__version__,manifest_sha256=hashlib.sha256((args.source/'episodes.jsonl').read_bytes()).hexdigest(),
        split_sha256=hashlib.sha256(args.fixed_splits.read_bytes()).hexdigest(),
        intent='validation-only search; no deployment; retain old test without evaluation',
        validation_gates=dict(min_event_recall=.95,min_median_before_safety_ms=50)))
    try:
        x,ends,y,g,t,episodes,audit=load_data(args.source)
        fixed=json.loads(args.fixed_splits.read_text())
        names={ep['id']:ep['index'] for ep in episodes}
        assert len(set().union(*(set(v) for v in fixed.values())))==sum(map(len,fixed.values()))
        assert set().union(*(set(v) for v in fixed.values()))==set(names)
        ids={k:np.flatnonzero(np.isin(g,[names[name] for name in v])) for k,v in fixed.items()}
        train,valid=ids['train'],ids['validation']
        # Test indices are not used for fitting, scoring, policy selection or export checks.
        mean=x[ends[train]].mean(axis=0,dtype=np.float64).astype(np.float32)
        std=np.maximum(x[ends[train]].std(axis=0,dtype=np.float64),1e-4).astype(np.float32)
        positive=train[y[train]==1];negative=train[y[train]==0]
        offsets=np.arange(-19,1)
        def batch(selected): return torch.from_numpy(x[ends[selected,None]+offsets[None,:]])
        def predict(model):
            model.eval();output=[]
            with torch.inference_mode():
                for start in range(0,len(valid),2048):
                    output.append(torch.sigmoid(model(batch(valid[start:start+2048]))).numpy())
            return np.concatenate(output)
        write_json(args.output/'data.json',dict(audit=audit,features=FEATURES,
            training_positive=len(positive),training_negative=len(negative),
            validation_windows=len(valid),validation_positive=int(y[valid].sum()),
            test_windows_reserved=len(ids['test']),mean=mean.tolist(),std=std.tolist()))
        grid=list(itertools.product(['small','base','wide','cnn'],[.0003,.001],[1,2,4],[0.,.1],[.0001,.001]))
        random.Random(args.seed).shuffle(grid)
        grid=[('base',.001,1,0.,.0001)]+[v for v in grid if v!=('base',.001,1,0.,.0001)]
        global_best=None;no_improve=0;failures=0
        status['state']='training';save_status()
        for trial,(arch,lr,ratio,dropout,decay) in enumerate(grid[:args.max_trials]):
            if stop or time.monotonic()>=deadline: break
            if shutil.disk_usage(args.output).free<2*1024**3:
                status['state']='disk_space_low';break
            directory=args.output/f'trial_{trial:03d}';directory.mkdir()
            cfg=dict(architecture=arch,learning_rate=lr,negative_per_positive=ratio,
                     dropout=dropout,weight_decay=decay,seed=args.seed+trial)
            write_json(directory/'config.json',cfg)
            status.update(trial=trial,configuration=cfg,epoch=0);save_status()
            print('TRIAL',trial,json.dumps(cfg),flush=True)
            try:
                torch.manual_seed(args.seed+trial)
                rng=np.random.default_rng(args.seed+trial)
                model=Candidate(mean,std,arch,dropout)
                optimizer=torch.optim.AdamW(model.parameters(),lr=lr,weight_decay=decay)
                best_ap=-1.;patience=0;history=[]
                for epoch in range(1,args.max_epochs+1):
                    if stop or time.monotonic()>=deadline: break
                    selected=np.r_[positive,rng.choice(negative,len(positive)*ratio,replace=False)]
                    rng.shuffle(selected);model.train();losses=[]
                    for start in range(0,len(selected),128):
                        if stop or time.monotonic()>=deadline: break
                        chosen=selected[start:start+128]
                        optimizer.zero_grad(set_to_none=True)
                        loss=nn.functional.binary_cross_entropy_with_logits(model(batch(chosen)),
                            torch.from_numpy(y[chosen].astype(np.float32)))
                        loss.backward();optimizer.step();losses.append(float(loss.detach()))
                    if stop or time.monotonic()>=deadline: break
                    scores=predict(model);ranking=ranking_metrics(y[valid],scores)
                    history.append(dict(epoch=epoch,loss=float(np.mean(losses)),**ranking))
                    write_json(directory/'history.json',history)
                    status.update(epoch=epoch,validation_ap=ranking['average_precision']);save_status()
                    if ranking['average_precision']>best_ap+1e-5:
                        best_ap=ranking['average_precision'];patience=0
                        torch.save(model.state_dict(),directory/'weights.pt')
                    else:
                        patience+=1
                        if patience>=7: break
                if stop or time.monotonic()>=deadline: break
                model.load_state_dict(torch.load(directory/'weights.pt',weights_only=True))
                scores=predict(model)
                policy,policies=select_policy(y[valid],scores,g[valid],t[valid],episodes)
                model.eval()
                traced=torch.jit.trace(model,torch.zeros(1,20,8))
                torch.testing.assert_close(traced(batch(valid[:4])),model(batch(valid[:4])))
                traced.save(str(directory/'model_torchscript.pt'))
                result=dict(trial=trial,config=cfg,validation_ap=best_ap,policy=policy,
                            parameters=sum(p.numel() for p in model.parameters()),epochs=len(history),
                            model_path=str((directory/'model_torchscript.pt').resolve()))
                write_json(directory/'result.json',result)
                write_json(directory/'validation_policies.json',policies)
                with (args.output/'trials.jsonl').open('a') as stream:
                    stream.write(json.dumps(result)+'\n');stream.flush();os.fsync(stream.fileno())
                status['trials_completed']+=1;failures=0
                if global_best is None or (policy_key(policy),best_ap)>(policy_key(global_best['policy']),global_best['validation_ap']):
                    global_best=result;no_improve=0
                    write_json(args.output/'best.json',result)
                    status['best']=result
                    print('NEW_BEST',json.dumps(result),flush=True)
                else: no_improve+=1
                status['trials_without_improvement']=no_improve;save_status()
                if no_improve>=args.stagnation:
                    status['state']='converged';break
            except Exception:
                (directory/'error.txt').write_text(traceback.format_exc())
                print(traceback.format_exc(),flush=True)
                failures+=1;status['failed_trials']+=1;save_status()
                if failures>=3:
                    status['state']='failed_repeatedly';break
        if status['state']=='training':
            status['state']='stopped' if stop else 'complete'
        (args.output/'README.md').write_text('''# Unattended fall-model search

Status: status.json. Best validation candidate: best.json (immutable trial model
path plus threshold and required consecutive frames). Logs: trials.jsonl and
trial_*/history.json. No test evaluation and no deployment are performed.

Three CPU threads; bounded wall budget, trial count, validation stagnation,
free-disk and consecutive-error stops. Per-trial checkpoints preserve progress.
Three model families plus a small CNN, learning rates, class sampling ratios,
dropout and weight decay are searched. Fixed v1 train/validation episode folds
are reused. All candidates use only pre-estop labelled histories. Validation
selection targets >=95% eligible-event recall and >=50 ms median lead over
existing Safety, then minimizes false alarm onsets per negative-exposure hour.
If no candidate meets the gates, best.json is marked qualifies=false; the
runner still never deploys a model.

An alarm requires confirm_frames consecutive scores above threshold, with
resets on episode changes or sample gaps over 20 ms. A 1-second cooldown is
used for counting false alarm onsets. Normalization is in the TorchScript
model; outputs are logits. Apply sigmoid and the saved temporal policy.
Scores are uncalibrated. The old test fold is reserved without reevaluation;
since earlier baseline test results are known and validation is repeatedly
searched, final performance requires newly collected independent scenarios.
Same-simulator and repeated-trajectory limitations still apply. Validation
numbers must not be presented as fresh test performance.
''')
    except Exception:
        status['state']='error';(args.output/'error.txt').write_text(traceback.format_exc())
        raise
    finally:
        status['ended_utc']=datetime.now(timezone.utc).isoformat();save_status();lock.close()
        print(json.dumps(status),flush=True)


if __name__=='__main__': main()
