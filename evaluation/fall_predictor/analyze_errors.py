#!/usr/bin/env python3
"""Replay selected validation alarms against original trajectories; no training."""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path

import numpy as np
import torch

from autotrain import alarm_mask, policy_metrics
from train import write_json


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',type=Path,required=True)
    ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--episode-index',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(3)
    best=json.loads((args.run/'best.json').read_text());policy=best['policy']
    split=json.loads((args.run/'splits.json').read_text())
    episodes=json.loads(args.episode_index.read_text())
    validation=set(split['validation'])
    model=torch.jit.load(best['model_path']).eval()
    scores=[];labels=[];groups=[];times=[];raw={};endpoints={};metas={}
    for ep in episodes:
        if ep['id'] not in validation:continue
        directory=args.source/ep['id']
        rows=[json.loads(l) for l in (directory/'samples.jsonl').open()]
        raw[ep['index']]=rows
        metas[ep['index']]=json.loads((directory/'metadata.json').read_text())
        selected=[i for i,r in enumerate(rows) if r['label'] is not None
                  and not any(v['estop'] for v in rows[i-19:i+1])]
        x=np.asarray([r['features'] for r in rows],dtype=np.float32)
        result=[]
        for start in range(0,len(selected),2048):
            ids=np.array(selected[start:start+2048])
            tensor=torch.from_numpy(x[ids[:,None]+np.arange(-19,1)[None,:]])
            with torch.inference_mode():result.extend(torch.sigmoid(model(tensor)).numpy())
        endpoints[ep['index']]=np.array(selected)
        scores.extend(result);labels.extend(rows[i]['label'] for i in selected)
        groups.extend([ep['index']]*len(selected));times.extend(rows[i]['sim_time_s'] for i in selected)
    s=np.array(scores);y=np.array(labels);g=np.array(groups);t=np.array(times)
    reproduced=policy_metrics(y,s,g,t,episodes,policy['threshold'],policy['confirm_frames'])
    for key in ['false_alarm_onsets','false_positive_windows','detected_fall_episodes']:
        assert reproduced[key]==policy[key],(key,reproduced[key],policy[key])
    mask=alarm_mask(s,g,t,policy['threshold'],policy['confirm_frames'])
    edge=np.r_[False,(g[1:]==g[:-1])&(np.diff(t)>0)&(np.diff(t)<=.020001)]
    previous=np.r_[False,mask[:-1]]&edge
    onset=np.flatnonzero(mask&~previous&(y==0))
    events=[];last={}
    for i in onset:
        group=int(g[i]);time=float(t[i])
        if time-last.get(group,-math.inf)<1.:continue
        last[group]=time
        ep=episodes[group];meta=metas[group];rows=raw[group]
        local=int(np.sum(g[:i]==group))
        end=int(endpoints[group][local]);row=rows[end]
        fall=ep['fall_time'];lead=None if fall is None else (fall-time)*1000
        if lead is None:category='no_fall_in_observed_episode'
        elif lead<=400+1e-6:category='fall_301_to_400ms_later'
        elif lead<=500+1e-6:category='fall_401_to_500ms_later'
        else:category='fall_over_500ms_later'
        j=i
        while j+1<len(mask) and mask[j+1] and edge[j+1]:j+=1
        frame_indices=np.flatnonzero((g==group)&mask&(np.arange(len(g))>=i)&(np.arange(len(g))<=j))
        advanced_into_positive=bool(np.any(y[frame_indices]==1))
        start=max(0,end-20);stop=min(len(rows),end+101)
        nearby=np.asarray([r['features'] for r in rows[start:stop]])
        force=np.asarray(meta['push_force_world_n']);push_start=(meta['released_at_sim_s'] or 0)+meta['push_delay_s']
        events.append(dict(number=len(events)+1,episode=ep['id'],mode=ep['mode'],seed=ep['seed'],
            onset_sim_s=time,after_release_ms=(time-meta['released_at_sim_s'])*1000,
            category=category,fall_lead_ms=lead,first_estop_lead_ms=None if ep['first_estop_time'] is None else (ep['first_estop_time']-time)*1000,
            score=float(s[i]),alarm_duration_ms=(t[j]-time+.01)*1000,
            alarm_continues_into_positive=advanced_into_positive,
            onset_roll_deg=math.degrees(row['features'][0]),onset_pitch_deg=math.degrees(row['features'][1]),
            onset_gyro_norm=float(np.linalg.norm(row['features'][2:5])),
            onset_accel_norm=float(np.linalg.norm(row['features'][5:8])),
            nearby_max_abs_tilt_deg=float(np.rad2deg(np.max(np.abs(nearby[:,:2])))),
            actual_push_steps=meta['push_applied_steps'],push_force_n=float(np.linalg.norm(force)),
            time_since_planned_push_ms=(time-push_start)*1000,
            total_observed_after_alarm_ms=(rows[-1]['sim_time_s']-time)*1000,
            alarms_nearby_end_sim_s=float(t[j])))
    assert len(events)==29
    negatives=y==0
    fp_indices=np.flatnonzero(mask&negatives)
    fp_categories=Counter()
    for i in fp_indices:
        fall=episodes[int(g[i])]['fall_time']
        if fall is None:fp_categories['no_fall_in_observed_episode']+=1
        elif (fall-t[i])*1000<=400+1e-6:fp_categories['fall_301_to_400ms_later']+=1
        elif (fall-t[i])*1000<=500+1e-6:fp_categories['fall_401_to_500ms_later']+=1
        else:fp_categories['fall_over_500ms_later']+=1
    summary=dict(reproduced=reproduced,event_categories=dict(Counter(e['category'] for e in events)),
        false_positive_window_categories=dict(fp_categories),
        event_modes=dict(Counter(e['mode'] for e in events)),
        events_without_actual_push=sum(e['actual_push_steps']==0 for e in events),
        events_with_alarm_continuing_into_positive=sum(e['alarm_continues_into_positive'] for e in events),
        events_in_distinct_episodes=len({e['episode'] for e in events}))
    lead_values=np.array([e['fall_lead_ms'] for e in events if e['fall_lead_ms'] is not None])
    summary['onset_fall_lead_ms']=dict(min=float(lead_values.min()),median=float(np.median(lead_values)),max=float(lead_values.max()))
    summary['onsets_at_numeric_300ms_boundary']=int(np.sum(np.abs(lead_values-300)<1e-6))
    fp_leads=np.array([(episodes[int(g[i])]['fall_time']-t[i])*1000
        for i in fp_indices if episodes[int(g[i])]['fall_time'] is not None])
    summary['false_windows_at_numeric_300ms_boundary']=int(np.sum(np.abs(fp_leads-300)<1e-6))
    comparisons=[]
    for confirm in (1,3,5):
        comparisons.append(policy_metrics(y,s,g,t,episodes,policy['threshold'],confirm))
    summary['same_threshold_confirmation_comparison']=comparisons
    write_json(args.output/'summary.json',summary);write_json(args.output/'events.json',events)
    with (args.output/'events.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(events[0]));w.writeheader();w.writerows(events)
    np.savez_compressed(args.output/'validation_replay.npz',score=s,label=y,group=g,time=t,alarm=mask)
    print(json.dumps(summary,indent=2),flush=True)
    # One page per onset, plus an overview of six examples. Plots use physical
    # units, current policy and contact/estop/push markers; no smoothed curves.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    def plot_event(event,axes):
        group=next(ep['index'] for ep in episodes if ep['id']==event['episode'])
        rows=raw[group];ep=episodes[group];meta=metas[group];zero=event['onset_sim_s']
        rt=np.array([r['sim_time_s'] for r in rows]);x=np.array([r['features'] for r in rows])
        keep=(rt>=zero-.7)&(rt<=zero+1.)
        axes[0].plot(rt[keep]-zero,np.rad2deg(x[keep,0]),label='roll (deg)')
        axes[0].plot(rt[keep]-zero,np.rad2deg(x[keep,1]),label='pitch (deg)')
        axes[1].plot(rt[keep]-zero,np.linalg.norm(x[keep,2:5],axis=1),label='gyro norm (rad/s)')
        ix=(g==group)&(t>=zero-.7)&(t<=zero+1.)
        axes[2].plot(t[ix]-zero,s[ix],label='model score')
        axes[2].step(t[ix]-zero,mask[ix].astype(float),where='post',label='3-frame alarm',alpha=.6)
        axes[2].axhline(policy['threshold'],color='grey',ls=':')
        for ax in axes:
            ax.axvline(0,color='orange',label='counted false onset')
            if ep['fall_time'] is not None:ax.axvline(ep['fall_time']-zero,color='red',label='confirmed fall')
            if ep['first_estop_time'] is not None:ax.axvline(ep['first_estop_time']-zero,color='purple',ls='--',label='Safety estop')
            if meta['push_applied_steps']:
                push_start=meta['released_at_sim_s']+meta['push_delay_s']-zero
                ax.axvspan(push_start,push_start+meta['push_duration_s'],color='green',alpha=.12)
            ax.set_xlim(-.7,1.);ax.grid(alpha=.2);ax.legend(fontsize=6,loc='upper left')
        axes[2].set_xlabel('Seconds relative to counted false alarm onset')
        axes[0].set_title(f"#{event['number']} {event['episode']} | {event['category']}",fontsize=9)
    with PdfPages(args.output/'all_29_alarms.pdf') as pdf:
        for event in events:
            fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True)
            plot_event(event,axes);fig.tight_layout();pdf.savefig(fig);plt.close(fig)
    fig,axes=plt.subplots(6,1,figsize=(12,13))
    representative=[]
    for category in sorted({e['category'] for e in events}):
        representative.extend([e for e in events if e['category']==category][:2])
    representative=(representative+[e for e in events if e not in representative])[:6]
    for ax,event in zip(axes,representative):
        group=next(ep['index'] for ep in episodes if ep['id']==event['episode']);zero=event['onset_sim_s'];ep=episodes[group]
        ix=(g==group)&(t>=zero-.5)&(t<=zero+.8)
        ax.plot(t[ix]-zero,s[ix],label='score');ax.axhline(.95,color='grey',ls=':')
        ax.axvline(0,color='orange',label='false onset')
        if ep['fall_time'] is not None:
            ax.axvline(ep['fall_time']-zero,color='red',label='fall')
            ax.axvspan(ep['fall_time']-zero-.3,ep['fall_time']-zero,color='red',alpha=.08,label='positive-label interval')
        ax.set_title(f"#{event['number']} {event['episode']} | {event['category']}",fontsize=9)
        ax.set_xlim(-.5,.8);ax.set_ylim(0,1.05);ax.grid(alpha=.2);ax.legend(fontsize=6,loc='lower left')
    axes[-1].set_xlabel('Seconds relative to false onset');fig.tight_layout();fig.savefig(args.output/'overview.png',dpi=140);plt.close(fig)


if __name__=='__main__':main()
