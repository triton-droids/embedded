#!/usr/bin/env python3
"""Train nested 300/600 ms pre-estop risk heads; evaluate a new sealed fold once."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import random
import sys
import time

import numpy as np
import torch
from torch import nn

from train import FEATURES, ranking_metrics, binary_metrics, write_json
from autotrain import select_policy, policy_metrics, policy_key

REPO=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(REPO/'simulation/humanoid_sim'))
from humanoid_sim.collection_rules import future_label, recovery_scenario


def scenario_folds():
    result={}
    for mode in ['walking_reference','standing_reference']:
        families=sorted({recovery_scenario(i)['family'] for i in range(48)
                         if recovery_scenario(i)['mode']==mode})
        random.Random(71007).shuffle(families)
        # The three pilot collection families are development data, never test.
        if mode=='walking_reference':
            pilot=sorted({recovery_scenario(i)['family'] for i in [48000,48001,48002]})
            families=pilot+[v for v in families if v not in pilot]
        for i,family in enumerate(families):
            result[family]='train' if i<16 else ('validation' if i<20 else 'test')
    return result


class DualRisk(nn.Module):
    def __init__(self,mean,std,dropout=.1):
        super().__init__()
        self.register_buffer('mean',torch.tensor(mean,dtype=torch.float32))
        self.register_buffer('std',torch.tensor(std,dtype=torch.float32))
        self.net=nn.Sequential(nn.Conv1d(8,16,5,padding=2),nn.ReLU(),
            nn.Conv1d(16,16,3,padding=1),nn.ReLU(),nn.Flatten(),nn.Linear(320,32),
            nn.ReLU(),nn.Dropout(dropout),nn.Linear(32,2))

    def forward(self,x):
        raw=self.net(((x-self.mean)/self.std).transpose(1,2))
        longer=raw[:,1]
        shorter=longer-nn.functional.softplus(raw[:,0])
        return torch.stack((shorter,longer),dim=1)


def load_sources(old_source,new_source,old_split):
    folds=scenario_folds()
    chunks=[];ends=[];labels=[];groups=[];times=[];splits=[];hard=[];episodes=[]
    offset=0;audit=Counter();quality=Counter()
    old_allowed={name:k for k in ['train','validation'] for name in old_split[k]}
    for source,is_new in [(old_source,False),(new_source,True)]:
        for line in (source/'episodes.jsonl').open():
            e=json.loads(line)
            if not e['valid']:audit['invalid_episode']+=1;continue
            name=f"episode_{e['episode']:06d}"
            if not is_new and name not in old_allowed:continue
            meta=json.loads((source/name/'metadata.json').read_text())
            if meta['feature_names']!=FEATURES:raise ValueError('feature schema mismatch')
            if is_new:
                scenario=meta.get('scenario')
                if not scenario:raise ValueError('New data requires recovery scenario metadata')
                fold=folds[scenario['family']]
                quality[f'{fold}_new_episodes']+=1
                if meta.get('recovered_after_push'):quality[f'{fold}_recovered']+=1
            else:fold=old_allowed[name]
            rows=[json.loads(l) for l in (source/name/'samples.jsonl').open()]
            x=np.asarray([r['features'] for r in rows],dtype=np.float32)
            ts=np.asarray([r['sim_time_s'] for r in rows])
            if x.shape!=(len(rows),8) or not np.isfinite(x).all() or np.any(np.diff(ts)<0):
                raise ValueError(f'Invalid observations: {source/name}')
            fall=meta['fall_time_sim_s'];chosen=[];targets=[]
            for i in range(19,len(rows)):
                window=rows[i-19:i+1]
                gaps=np.diff(ts[i-19:i+1])
                if (any(r['base_held'] or r['control_fault'] or r['estop'] for r in window)
                        or np.any(gaps<.005-1e-9) or np.any(gaps>.02+1e-9)):
                    continue
                target=[future_label(float(ts[i]),fall,float(ts[-1]),h) for h in [.3,.6]]
                if None in target:continue
                assert target[0]<=target[1]
                # Raw outcomes are preserved; v1's saved 300-ms labels are not trusted for retraining.
                if rows[i].get('label') is not None and rows[i]['label']!=target[0]:
                    audit['corrected_300ms_boundary_labels']+=1
                chosen.append(i);targets.append(target)
            if not chosen:audit['no_usable_episode']+=1;continue
            index=len(episodes)
            first_stop=next((r['sim_time_s'] for r in rows if r['estop'] and not r['base_held']),None)
            ep=dict(index=index,id=('new/' if is_new else 'legacy/')+name,
                source=str(source.resolve()),local_id=name,fold=fold,fall_time=fall,
                first_estop_time=first_stop,is_new=is_new,recovered=bool(meta.get('recovered_after_push')),
                family=meta.get('scenario',{}).get('family') if meta.get('scenario') else None)
            episodes.append(ep);chunks.append(x)
            ends.extend(offset+i for i in chosen);labels.extend(targets)
            groups.extend([index]*len(chosen));times.extend(ts[chosen]);splits.extend([fold]*len(chosen))
            # Only new training recovery episodes provide targeted hard negatives.
            hard.extend([bool(is_new and meta.get('recovered_after_push'))]*len(chosen))
            offset+=len(x)
            if len(episodes)%100==0:print('Loaded',len(episodes),'dual episodes',flush=True)
    return (np.concatenate(chunks),np.asarray(ends),np.asarray(labels,dtype=np.int64),
            np.asarray(groups),np.asarray(times),np.asarray(splits),np.asarray(hard),episodes,dict(audit),dict(quality))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--legacy-source',type=Path,required=True)
    ap.add_argument('--new-source',type=Path,required=True)
    ap.add_argument('--legacy-splits',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--epochs',type=int,default=50)
    ap.add_argument('--max-trials',type=int,default=8)
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(3);torch.set_num_interop_threads(1);torch.use_deterministic_algorithms(True)
    started=time.perf_counter();pid=__import__('os').getpid()
    status=dict(pid=pid,state='loading',test_evaluated=False)
    write_json(args.output/'status.json',status)
    old_split=json.loads(args.legacy_splits.read_text())
    x,ends,y,g,t,fold,hard,episodes,audit,quality=load_sources(args.legacy_source,args.new_source,old_split)
    write_json(args.output/'quality.json',dict(quality=quality,audit=audit))
    # Do not silently call a failed data-collection run a recovery experiment.
    for k,minimum in [('train_recovered',8),('validation_recovered',2),('test_recovered',2)]:
        if quality.get(k,0)<minimum:
            status.update(state='insufficient_recovery_data',reason=f'{k} requires {minimum}; got {quality.get(k,0)}')
            write_json(args.output/'status.json',status);raise ValueError(status['reason'])
    ids={k:np.flatnonzero(fold==k) for k in ['train','validation','test']}
    for k,selected in ids.items():
        for h in range(2):
            if len(np.unique(y[selected,h]))<2:raise ValueError(f'{k} lacks classes for head {h}')
    train,valid=ids['train'],ids['validation']
    assert all(episodes[int(v)]['is_new'] for v in np.unique(g[ids['test']]))
    assert not set(g[train])&set(g[valid]) and not set(g[train])&set(g[ids['test']])
    mean=x[ends[train]].mean(0,dtype=np.float64).astype(np.float32)
    std=np.maximum(x[ends[train]].std(0,dtype=np.float64),1e-4).astype(np.float32)
    off=np.arange(-19,1)
    def batch(selected):return torch.from_numpy(x[ends[selected,None]+off[None,:]])
    def predict(model,selected):
        model.eval();out=[]
        with torch.inference_mode():
            for a in range(0,len(selected),2048):out.append(torch.sigmoid(model(batch(selected[a:a+2048]))).numpy())
        return np.concatenate(out)
    strata=[train[y[train,0]==1],train[(y[train,0]==0)&(y[train,1]==1)],train[y[train,1]==0]]
    hard_negative=train[hard[train]&(y[train,1]==0)]
    if not all(len(s) for s in strata) or not len(hard_negative):raise ValueError('Missing sampling strata')
    config=dict(features=FEATURES,horizons_s=[.3,.6],time_tolerance_s=1e-9,
        normalization_mean=mean.tolist(),normalization_std=std.tolist(),
        source_manifest_hashes={str(s):__import__('hashlib').sha256((s/'episodes.jsonl').read_bytes()).hexdigest()
            for s in [args.legacy_source,args.new_source]},
        scenario_family_folds=scenario_folds(),
        hard_negative_windows=len(hard_negative),
        split_summary={k:dict(windows=len(v),episodes=len(np.unique(g[v])),
            positive_300=int(y[v,0].sum()),positive_600=int(y[v,1].sum())) for k,v in ids.items()},
        note='Old test excluded; new test families sealed until one final selected-model evaluation')
    write_json(args.output/'config.json',config);write_json(args.output/'episodes.json',episodes)
    write_json(args.output/'splits.json',{k:[ep['id'] for ep in episodes if ep['fold']==k] for k in ids})
    best=None
    for trial,(lr,ratio,dropout) in enumerate([(lr,r,d) for lr in [.001,.0003] for r in [2,4] for d in [.1,.2]][:args.max_trials]):
        directory=args.output/f'trial_{trial:02d}';directory.mkdir()
        torch.manual_seed(38100+trial);rng=np.random.default_rng(38100+trial)
        model=DualRisk(mean,std,dropout);opt=torch.optim.AdamW(model.parameters(),lr=lr,weight_decay=.0001)
        best_ap=-1.;stale=0;history=[]
        for epoch in range(1,args.epochs+1):
            n=min(len(strata[0]),len(strata[1]))
            safe_count=ratio*2*n
            # Half the safe sampling budget deliberately repeats independent recovery examples.
            selected=np.r_[rng.choice(strata[0],n,replace=False),rng.choice(strata[1],n,replace=False),
                rng.choice(hard_negative,safe_count//2,replace=True),
                rng.choice(strata[2],safe_count-safe_count//2,replace=False)]
            rng.shuffle(selected);model.train();losses=[]
            for a in range(0,len(selected),128):
                chosen=selected[a:a+128];opt.zero_grad(set_to_none=True)
                loss=nn.functional.binary_cross_entropy_with_logits(model(batch(chosen)),torch.from_numpy(y[chosen].astype(np.float32)))
                loss.backward();opt.step();losses.append(float(loss.detach()))
            score=predict(model,valid)
            aps=[ranking_metrics(y[valid,h],score[:,h])['average_precision'] for h in range(2)]
            history.append(dict(epoch=epoch,loss=float(np.mean(losses)),validation_ap_300=aps[0],validation_ap_600=aps[1]))
            write_json(directory/'history.json',history)
            status.update(state='training',trial=trial,epoch=epoch,elapsed_s=time.perf_counter()-started)
            write_json(args.output/'status.json',status)
            if np.mean(aps)>best_ap+1e-5:
                best_ap=float(np.mean(aps));stale=0;torch.save(model.state_dict(),directory/'weights.pt')
            else:
                stale+=1
                if stale>=7:break
        model.load_state_dict(torch.load(directory/'weights.pt',weights_only=True))
        score=predict(model,valid)
        policies=[select_policy(y[valid,h],score[:,h],g[valid],t[valid],episodes)[0] for h in range(2)]
        model.eval();traced=torch.jit.trace(model,torch.zeros(1,20,8))
        torch.testing.assert_close(model(batch(valid[:4])),traced(batch(valid[:4])))
        traced.save(str(directory/'model_torchscript.pt'))
        result=dict(trial=trial,lr=lr,negative_ratio=ratio,dropout=dropout,mean_validation_ap=best_ap,
            validation_policies={'300ms':policies[0],'600ms':policies[1]},
            model_path=str((directory/'model_torchscript.pt').resolve()))
        write_json(directory/'result.json',result);print(json.dumps(result),flush=True)
        if best is None or (policy_key(policies[1]),best_ap)>(policy_key(best['validation_policies']['600ms']),best['mean_validation_ap']):
            best=result;write_json(args.output/'best.json',best)
    # Final model and both policies are frozen BEFORE loading test predictions.
    model=torch.jit.load(best['model_path']).eval()
    test=ids['test'];score=predict(model,test)
    assert np.all(score[:,0]<=score[:,1]+1e-7)
    final={}
    for h,name in enumerate(['300ms','600ms']):
        p=best['validation_policies'][name]
        event=policy_metrics(y[test,h],score[:,h],g[test],t[test],episodes,p['threshold'],p['confirm_frames'])
        event['fall_episodes_total']=sum(ep['fold']=='test' and ep['fall_time'] is not None for ep in episodes)
        event['fall_episodes_without_eligible_positive_windows']=event['fall_episodes_total']-event['eligible_fall_episodes']
        final[name]=dict(events=event,raw_window_metrics=binary_metrics(y[test,h],score[:,h],p['threshold']),
                        ranking=ranking_metrics(y[test,h],score[:,h]))
    recovered_ids=[ep['index'] for ep in episodes if ep['fold']=='test' and ep['recovered']]
    from autotrain import alarm_mask
    for h,name in enumerate(['300ms','600ms']):
        p=best['validation_policies'][name];mask=alarm_mask(score[:,h],g[test],t[test],p['threshold'],p['confirm_frames'])
        final[name]['recovery_episodes']=len(recovered_ids)
        final[name]['recovery_episodes_with_alarm']=len(set(g[test][mask])&set(recovered_ids))
    np.savez_compressed(args.output/'sealed_test_predictions.npz',group=g[test],time=t[test],label=y[test],score=score)
    write_json(args.output/'sealed_test_metrics.json',final)
    status.update(state='complete',test_evaluated=True,test_evaluations=1,best=best,
                  elapsed_s=time.perf_counter()-started)
    write_json(args.output/'status.json',status)
    (args.output/'REPORT.md').write_text('# Dual-horizon training completed\n\n'
        'Outputs: 300 ms and 600 ms logits, with sigmoid(short) <= sigmoid(long).\n'
        'Normalization is embedded. Apply each saved temporal policy separately.\n'
        'Old test excluded. New scenario families were assigned before outcomes were known;\n'
        'only the selected model was evaluated once on the new test fold.\n'
        'Safety remains unchanged. See config.json, quality.json, best.json and sealed_test_metrics.json.\n'
        'Labels include the existing controller and Safety response. Same-simulator limits remain.\n'
        'Metrics from a small independent fold have sampling uncertainty. No deployment was done.\n')
    print(json.dumps(final,indent=2),flush=True)


if __name__=='__main__':main()
