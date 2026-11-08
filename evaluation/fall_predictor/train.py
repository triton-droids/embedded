#!/usr/bin/env python3
"""Train a small pre-estop fall warning model from episode-labelled IMU logs.

No ROS imports or control outputs. Split trajectories before balancing. Scores
from the balanced training distribution are NOT calibrated probabilities.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import random
import time

import numpy as np
import torch
from torch import nn

FEATURES = ['roll_rad','pitch_rad','gyro_x','gyro_y','gyro_z','accel_x','accel_y','accel_z']
HISTORY = 20


def write_json(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    temp.replace(path)


def stratified_episode_split(episodes, seed):
    """Stratify on mode/outcome/push, never splitting an episode's windows."""
    strata = defaultdict(list)
    for ep in episodes:
        key = (ep['mode'], ep['fall_time'] is not None, ep['actual_push'])
        strata[key].append(ep['index'])
    rng = np.random.default_rng(seed)
    result = {key:[] for key in ('train','validation','test')}
    for ids in strata.values():
        ids = np.asarray(ids)
        rng.shuffle(ids)
        n = len(ids)
        a, b = int(n*.7), int(n*.85)
        # Very small strata stay in train instead of emptying their train group.
        if n < 7:
            a = b = n
        for key, part in zip(result, (ids[:a], ids[a:b], ids[b:])):
            result[key].extend(int(x) for x in part)
    return result


def load_data(source):
    manifest = source/'episodes.jsonl'
    entries = [json.loads(line) for line in manifest.open() if line.strip()]
    episodes, chunks, endpoints, ys, groups, times, stops = [], [], [], [], [], [], []
    audit = Counter()
    offset = 0
    for entry in entries:
        if not entry['valid']:
            audit['invalid_episodes_excluded'] += 1
            continue
        path = source/f"episode_{entry['episode']:06d}"
        metadata = json.loads((path/'metadata.json').read_text())
        if metadata['feature_names'] != FEATURES or metadata['history_frames'] != HISTORY:
            raise ValueError(f'Unexpected feature schema: {path}')
        rows = [json.loads(line) for line in (path/'samples.jsonl').open()]
        x = np.asarray([r['features'] for r in rows], dtype=np.float32)
        ts = np.asarray([r['sim_time_s'] for r in rows], dtype=np.float64)
        if x.shape != (len(rows),8) or not np.isfinite(x).all() or np.any(np.diff(ts)<0):
            raise ValueError(f'Invalid observations: {path}')
        ep_index = len(episodes)
        usable = []
        first_stop = next((r['sim_time_s'] for r in rows if r['estop'] and not r['base_held']),None)
        fall = metadata['fall_time_sim_s']
        for i,r in enumerate(rows):
            if r['label'] is None:
                audit['unknown_windows_excluded'] += 1
                continue
            if i < HISTORY-1:
                raise ValueError('Label without full history')
            window = rows[i-HISTORY+1:i+1]
            gaps = np.diff(ts[i-HISTORY+1:i+1])
            if any(v['estop'] for v in window):
                audit[f"post_estop_label_{r['label']}_excluded"] += 1
                continue
            if (any(v['base_held'] or v['control_fault'] for v in window)
                    or np.any(gaps < .005) or np.any(gaps > .02)):
                raise ValueError(f'Invalid labelled history: {path}:{i}')
            expected = int(fall is not None and 0 < fall-ts[i] <= .3+1e-9)
            legacy = int(fall is not None and ts[i] < fall <= ts[i]+.3)
            if r['label'] not in (expected,legacy) or (r['label']==0 and ts[-1]+1e-9 < ts[i]+.3):
                raise ValueError(f'Invalid future label: {path}:{i}')
            if r['label']!=expected:
                audit['corrected_300ms_boundary_labels']+=1
                r['label']=expected  # In-memory correction; raw logs remain immutable.
            usable.append(i)
        if not usable:
            audit['episodes_without_pre_estop_windows'] += 1
            continue
        # Full raw feature sequence is stored once; windows are gathered per batch.
        chunks.append(x)
        endpoints.extend(offset+i for i in usable)
        ys.extend(rows[i]['label'] for i in usable)
        groups.extend([ep_index]*len(usable))
        times.extend(ts[usable])
        stops.extend([False]*len(usable))
        episodes.append(dict(index=ep_index, id=f"episode_{entry['episode']:06d}",
            mode=entry['mode'], seed=entry['seed'], fall_time=fall, first_estop_time=first_stop,
            actual_push=metadata['push_applied_steps']>0, windows=len(usable),
            positive_windows=sum(rows[i]['label']==1 for i in usable),
            push_force_world_n=metadata['push_force_world_n']))
        offset += len(x)
        if len(episodes)%100==0:
            print(f'Loaded {len(episodes)} episodes, {len(ys)} pre-estop windows',flush=True)
    if not episodes:
        raise ValueError('No usable data')
    return (np.concatenate(chunks),np.asarray(endpoints,dtype=np.int64),
            np.asarray(ys,dtype=np.int64),np.asarray(groups,dtype=np.int32),
            np.asarray(times),episodes,dict(audit))


class FallMLP(nn.Module):
    def __init__(self, mean, std):
        super().__init__()
        self.register_buffer('mean',torch.as_tensor(mean,dtype=torch.float32))
        self.register_buffer('std',torch.as_tensor(std,dtype=torch.float32))
        self.layers = nn.Sequential(nn.Flatten(),nn.Linear(160,64),nn.ReLU(),
                                    nn.Linear(64,32),nn.ReLU(),nn.Linear(32,1))

    def forward(self,x):
        return self.layers((x-self.mean)/self.std).squeeze(-1)


def binary_metrics(y, scores, threshold):
    pred = scores >= threshold
    tp = int(np.sum(pred & (y==1))); fp = int(np.sum(pred & (y==0)))
    fn = int(np.sum(~pred & (y==1))); tn = int(np.sum(~pred & (y==0)))
    return dict(n=len(y),positive=int(y.sum()),threshold=float(threshold),
        tp=tp,fp=fp,fn=fn,tn=tn,recall=tp/max(tp+fn,1),
        precision=tp/max(tp+fp,1),false_positive_rate=fp/max(fp+tn,1),
        accuracy=(tp+tn)/max(len(y),1), f1=2*tp/max(2*tp+fp+fn,1))


def ranking_metrics(y, scores):
    if len(np.unique(y))<2:
        raise ValueError('Both classes required for validation/test')
    order = np.argsort(-scores,kind='stable')
    sy, ss = y[order],scores[order]
    ends = np.r_[np.flatnonzero(np.diff(ss)!=0),len(ss)-1]
    tp = np.cumsum(sy)[ends]
    fp = ends+1-tp
    recall = tp/y.sum(); precision = tp/(ends+1)
    ap = np.sum(np.diff(np.r_[0,recall])*precision)
    auroc = np.trapz(np.r_[0,recall],np.r_[0,fp/(len(y)-y.sum())])
    return dict(average_precision=float(ap),auroc=float(auroc))


def recall_threshold(y,scores,recall=.95):
    positive = np.sort(scores[y==1])
    if not len(positive): raise ValueError('Validation has no positives')
    return float(positive[len(positive)-math.ceil(recall*len(positive))])


def episode_metrics(ids,y,scores,groups,times,episodes,threshold):
    events, leads, before_rule = [], [], []
    alarm_count, exposure = 0, 0.
    for g in np.unique(groups[ids]):
        loc = np.flatnonzero(groups[ids]==g)
        loc = loc[np.argsort(times[ids[loc]])]
        ep = episodes[int(g)]
        ts, labels, score = times[ids[loc]],y[loc],scores[loc]
        pred = score>=threshold
        positive = labels==1
        if ep['fall_time'] is not None:
            eligible = bool(positive.any())
            hits = ts[pred & positive]
            event = dict(episode=ep['id'], eligible=eligible,detected=bool(len(hits)),
                         fall_time_s=ep['fall_time'],first_estop_time_s=ep['first_estop_time'])
            if len(hits):
                lead = ep['fall_time']-float(hits[0]); leads.append(lead)
                event['lead_s'] = lead
                if ep['first_estop_time'] is not None:
                    advantage = ep['first_estop_time']-float(hits[0])
                    before_rule.append(advantage)
                    event['before_safety_estop_s'] = advantage
            events.append(event)
        # Negative-time exposure and alarm onsets, with a one-second cooldown.
        last_alarm, previous, last_t = -math.inf, False, None
        for t,label,alert in zip(ts,labels,pred):
            dt = .01 if last_t is None else float(t-last_t)
            if label==0 and 0 < dt <= .02: exposure += dt
            if last_t is not None and dt>.02: previous=False
            if alert and not previous and label==0 and t-last_alarm>=1.:
                alarm_count+=1;last_alarm=float(t)
            previous=bool(alert);last_t=float(t)
    eligible = [e for e in events if e['eligible']]
    return dict(fall_episodes=len(events),eligible_fall_episodes=len(eligible),
        detected_fall_episodes=sum(e['detected'] for e in eligible),
        missed_eligible_fall_episodes=sum(not e['detected'] for e in eligible),
        fall_episodes_without_pre_estop_positive_window=sum(not e['eligible'] for e in events),
        event_recall=sum(e['detected'] for e in eligible)/max(len(eligible),1),
        median_lead_ms=float(np.median(leads)*1000) if leads else None,
        median_before_safety_estop_ms=float(np.median(before_rule)*1000) if before_rule else None,
        false_alarm_onsets=alarm_count, negative_exposure_minutes=exposure/60,
        false_alarm_onsets_per_hour=alarm_count/(exposure/3600) if exposure else None,
        events=events)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--seed',type=int,default=20261107)
    ap.add_argument('--epochs',type=int,default=50)
    args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    random.seed(args.seed);np.random.seed(args.seed);torch.manual_seed(args.seed)
    torch.set_num_threads(4)
    torch.set_num_interop_threads(1)
    torch.use_deterministic_algorithms(True)
    start=time.perf_counter()
    x,ends,y,groups,times,episodes,audit=load_data(args.source)
    split=stratified_episode_split(episodes,args.seed)
    indices={k:np.flatnonzero(np.isin(groups,v)) for k,v in split.items()}
    for k,ids in indices.items():
        if len(np.unique(y[ids]))!=2: raise ValueError(f'{k} needs both classes')
    assert not (set(split['train']) & set(split['test']) or set(split['train']) & set(split['validation'])
                or set(split['test']) & set(split['validation']))
    train=indices['train']
    # Training-only endpoint feature statistics; normalization ships with model.
    mean=x[ends[train]].mean(axis=0,dtype=np.float64).astype(np.float32)
    std=np.maximum(x[ends[train]].std(axis=0,dtype=np.float64),1e-4).astype(np.float32)
    model=FallMLP(mean,std)
    optimizer=torch.optim.Adam(model.parameters(),lr=.001)
    offsets=np.arange(-19,1,dtype=np.int64)
    def batch(ids):
        return torch.from_numpy(x[ends[ids,None]+offsets[None,:]])
    def predict(ids):
        model.eval();result=[]
        with torch.inference_mode():
            for pos in range(0,len(ids),4096):
                result.append(torch.sigmoid(model(batch(ids[pos:pos+4096]))).numpy())
        return np.concatenate(result)
    config=dict(seed=args.seed,features=FEATURES,history_frames=20,horizon_s=.3,
        network=[160,64,32,1],parameters=sum(p.numel() for p in model.parameters()),
        batch_size=128,learning_rate=.001,max_epochs=args.epochs,early_stopping_patience=5,
        training_balance='all train positives plus equal randomly resampled negatives each epoch',
        excludes='invalid episodes, null labels, and histories containing active estop',
        threshold_rule='highest validation threshold achieving at least 95% positive-window recall',
        source=str(args.source.resolve()),device='cpu',threads=4,torch_version=torch.__version__,
        numpy_version=np.__version__,mean=mean.tolist(),std=std.tolist(),audit=audit,
        manifest_sha256=hashlib.sha256((args.source/'episodes.jsonl').read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        split_summary={k:dict(episodes=len(split[k]),windows=len(ids),positive=int(y[ids].sum()),
                             negative=int((y[ids]==0).sum())) for k,ids in indices.items()})
    write_json(args.output/'config.json',config)
    write_json(args.output/'splits.json',{k:[episodes[i]['id'] for i in v] for k,v in split.items()})
    write_json(args.output/'episodes.json',episodes)
    print(json.dumps(config,indent=2),flush=True)
    positive=train[y[train]==1];negative=train[y[train]==0]
    if len(negative)<len(positive): raise ValueError('Expected majority negative class')
    rng=np.random.default_rng(args.seed)
    best,patience,best_epoch=-1.,0,0
    history=[]
    for epoch in range(1,args.epochs+1):
        model.train()
        selected=np.r_[positive,rng.choice(negative,size=len(positive),replace=False)]
        rng.shuffle(selected)
        losses=[]
        for pos in range(0,len(selected),128):
            ids=selected[pos:pos+128]
            optimizer.zero_grad(set_to_none=True)
            loss=nn.functional.binary_cross_entropy_with_logits(model(batch(ids)),torch.from_numpy(y[ids].astype(np.float32)))
            loss.backward();optimizer.step();losses.append(float(loss.detach()))
        val=predict(indices['validation'])
        rank=ranking_metrics(y[indices['validation']],val)
        record=dict(epoch=epoch,loss=float(np.mean(losses)),balanced_positive=len(positive),
                    balanced_negative=len(positive),validation=rank,elapsed_s=time.perf_counter()-start)
        history.append(record);print(json.dumps(record),flush=True)
        write_json(args.output/'history.json',history)
        if rank['average_precision']>best+1e-5:
            best=rank['average_precision'];best_epoch=epoch;patience=0
            torch.save(model.state_dict(),args.output/'model_state.pt')
        else:
            patience+=1
            if patience>=5: break
    model.load_state_dict(torch.load(args.output/'model_state.pt',weights_only=True))
    val_ids=indices['validation'];val=predict(val_ids)
    threshold=recall_threshold(y[val_ids],val)
    # Test is touched only after checkpoint and threshold selection are final.
    test_ids=indices['test'];scores=predict(test_ids);test_y=y[test_ids]
    result=dict(best_epoch=best_epoch,epochs_run=len(history),threshold=threshold,
        validation={**binary_metrics(y[val_ids],val,threshold),**ranking_metrics(y[val_ids],val)},
        test={**binary_metrics(test_y,scores,threshold),**ranking_metrics(test_y,scores)},
        test_at_05=binary_metrics(test_y,scores,.5),
        test_events=episode_metrics(test_ids,test_y,scores,groups,times,episodes,threshold),
        always_continue=binary_metrics(test_y,np.zeros(len(test_y)),.5),
        always_warn=binary_metrics(test_y,np.ones(len(test_y)),.5))
    # Simple current 40-degree tilt baseline, distinct from full existing Safety.
    tilt=(np.max(np.abs(x[ends[test_ids],:2]),axis=1)>math.radians(40)).astype(np.float32)
    result['tilt40_baseline']=binary_metrics(test_y,tilt,.5)
    prewindow=batch(test_ids[:1])
    for _ in range(30):
        with torch.inference_mode(): model(prewindow)
    latency=[]
    for _ in range(300):
        tick=time.perf_counter()
        with torch.inference_mode(): model(prewindow)
        latency.append((time.perf_counter()-tick)*1000)
    result['cpu_inference_ms']=dict(median=float(np.median(latency)),p95=float(np.quantile(latency,.95)),
        note='batch 1, normalization and forward; excludes ROS, rolling-buffer construction and scheduling')
    traced=torch.jit.trace(model.eval(),torch.zeros(1,20,8))
    traced.save(str(args.output/'model_torchscript.pt'))
    np.savez_compressed(args.output/'test_predictions.npz',episode_index=groups[test_ids],
                        time_s=times[test_ids],label=test_y,score=scores)
    result['elapsed_s']=time.perf_counter()-start
    write_json(args.output/'metrics.json',result)
    report=f'''# First simulated pre-estop fall predictor

12,417-parameter MLP, 20 x 8 IMU history, 300 ms prediction horizon.
Only pre-estop histories are used. Valid episodes are split 70/15/15 within
mode/outcome/push strata before windows are used. Each training epoch contains
all training positives and an equal random subset of negatives. Validation and
test retain natural prevalence. Checkpoint selection uses validation average
precision; threshold uses validation >=95% window recall. Test is evaluated
once after selection. No integration into Safety or robot control.

Best epoch: {best_epoch}; threshold: {threshold:.6f}.

| Test metric | Value |
|---|---:|
| Window recall | {result['test']['recall']:.2%} |
| Window precision | {result['test']['precision']:.2%} |
| False positive rate | {result['test']['false_positive_rate']:.2%} |
| Average precision | {result['test']['average_precision']:.4f} |
| ROC AUC | {result['test']['auroc']:.4f} |
| Missed positive windows | {result['test']['fn']} / {result['test']['positive']} |
| Eligible fall episodes detected | {result['test_events']['detected_fall_episodes']} / {result['test_events']['eligible_fall_episodes']} |
| Median warning lead before contact | {result['test_events']['median_lead_ms']} ms |
| Median lead before existing Safety estop | {result['test_events']['median_before_safety_estop_ms']} ms |
| False alarm onsets per negative-exposure hour | {result['test_events']['false_alarm_onsets_per_hour']} |
| CPU forward median / p95 | {result['cpu_inference_ms']['median']:.3f} / {result['cpu_inference_ms']['p95']:.3f} ms |

Event detection requires at least one warning in an eligible, pre-estop positive
window within 300 ms of confirmed non-foot contact. Events without such input
are listed separately; they are not silently counted as detected. Warning lead
is therefore capped at 300 ms by this metric. Alarm onsets use a 1-second cooldown.
Exposure is only the observed valid negative-window time, not total run time.

Limitations: same simulator, fixed controller and largely repeated reference
motions. Different episodes may share near-identical prefixes and no-push
trajectories; episode splitting prevents overlapping-window leakage but does
not establish scenario/physical-parameter generalization. Independent, more
diverse held-out simulation and real sensor logs are needed before deployment.
Outcomes include the existing Safety motor-disable response. Balanced-training
sigmoid scores are uncalibrated and are not real-world fall probabilities.
CPU timing excludes ROS and scheduling. No policy/Safety behavior was changed.

See config.json, splits.json, history.json and metrics.json for counts, exact
configuration, confusion matrices, baselines and per-event details. The model
accepts float32 [batch,20,8] in feature order from config.json. Normalization is
included and output is an uncalibrated logit; apply sigmoid then the saved
threshold. model_torchscript.pt is the standalone artifact.
'''
    (args.output/'REPORT.md').write_text(report)
    print(json.dumps({k:v for k,v in result.items() if k!='test_events'},indent=2),flush=True)


if __name__=='__main__': main()
