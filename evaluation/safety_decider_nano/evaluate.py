"""Offline synthetic rule-agreement benchmark; never sends robot commands.

Run with a local, reviewed upstream decider.py and model files:
  python evaluate.py --model-dir /tmp/safety-nano-model --output results.json
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
import time

PACKAGE = Path(__file__).resolve().parents[2] / 'humanoid_control/humanoid_safety'
sys.path.insert(0, str(PACKAGE))
from humanoid_safety.checks import (FallDetectionCheck, ImuFreshnessCheck,
    ImuStuckCheck, ImuObservation, SafetySupervisor)

REVISION = '2c7c01ffe5aba74dd15968cf38b0fdc83d1f3810'
RULES = '''Decide the emergency-stop output of a robot safety supervisor after the history below.
Rules: latch emergency stop if absolute roll or pitch is strictly greater than 40 degrees for 3 accepted consecutive samples, or orientation is invalid. A jump >=20 degrees from the last accepted sample with gyro magnitude <0.5 rad/s is ignored for fall counting, for at most 10 consecutive ignored samples. Invalid samples do not refresh IMU freshness. IMU data age includes source age at receipt plus elapsed time; trip if age strictly exceeds 0.30 seconds. Before the first valid sample, trip only after startup grace of 3 seconds. Trip on 50 bit-identical consecutive IMU samples (orientation, gyro and acceleration). Once latched, emergency stop remains latched even if the robot recovers; no reset is requested. Hardware checks are disabled. All angles in the history are degrees. What emergency-stop output is required now?'''
OPTIONS = {'stop': 'Assert or retain latched emergency stop.',
           'continue': 'Leave emergency stop deasserted; no safety rule has tripped.'}


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def cases():
    rows = []
    for group in ['normal', 'fall', 'pending_fall', 'glitch', 'stale', 'source_stale',
                  'stuck', 'invalid', 'latched_recovered', 'startup_grace', 'startup_expired', 'boundary']:
        for j in range(4):
            sup = SafetySupervisor([FallDetectionCheck(), ImuFreshnessCheck(
                timeout_s=.30, startup_grace_s=3., started_at=0.), ImuStuckCheck()])
            roll, pitch, gyro, valid, source_age = 0., 0., 1., True, 0.
            n, wait = 3, 0.
            if group == 'normal': roll = [0., 12., -25., 39.][j]
            elif group == 'fall': roll = [41., -45., 60., -80.][j]
            elif group == 'pending_fall': roll, n = 45. + j, 1 + j % 2
            elif group == 'glitch': gyro, n = 0., 2 + j
            elif group == 'stale': wait = [.301, .35, .6, 1.][j]
            elif group == 'source_stale': source_age, wait = .20 + j * .01, .15
            elif group == 'stuck': gyro, n = 0., 50 + j
            elif group == 'invalid': valid = False
            elif group == 'latched_recovered': roll = 50. + j
            elif group == 'startup_grace': n, wait = 0, .5 + j * .5
            elif group == 'startup_expired': n, wait = 0, 3.1 + j
            elif group == 'boundary': roll = [40., -40., 39.999, 40.001][j]
            history = []
            def observe(t, r, p, g, good=True, age=0.):
                obs = ImuObservation(received_at=t, roll_rad=math.radians(r),
                    pitch_rad=math.radians(p), valid=good, source_age_s=age,
                    angular_velocity=(g, 0., 0.), linear_acceleration=(0., 0., 9.81))
                sup.observe_imu(obs)
                sup.evaluate(t)
                history.append(dict(time_s=t, roll_deg=r, pitch_deg=p, gyro_rad_s=[g,0.,0.],
                    acceleration_m_s2=[0.,0.,9.81], orientation_valid=good, source_age_s=age))
            if group == 'glitch': observe(.01, 0., 0., 0.)
            for i in range(n):
                r = (60. + j) if group == 'glitch' else roll
                observe(round((len(history)+1)*.01, 4), r, pitch, gyro, valid, source_age)
            if group == 'latched_recovered': observe(.04, 0., 0., 1.)
            now = round((history[-1]['time_s'] if history else 0.) + wait, 4)
            sup.evaluate(now)
            # Lossless run-length compression keeps long frozen histories readable.
            compact = []
            for obs in history:
                values = {k:v for k,v in obs.items() if k != 'time_s'}
                if compact and compact[-1]['sample'] == values:
                    compact[-1]['count'] += 1
                    compact[-1]['last_time_s'] = obs['time_s']
                else:
                    compact.append(dict(first_time_s=obs['time_s'], last_time_s=obs['time_s'],
                                        count=1, sample=values))
            rows.append(dict(id=f'{group}_{j}', category=group, expected='stop' if sup.latched else 'continue',
                state=dict(started_at_s=0., initially_latched=False, reset_requested=False,
                           history_runs=compact, evaluation_time_s=now)))
    return rows


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--model-dir', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--generate-only', action='store_true')
    args=ap.parse_args()
    dataset=cases()
    if args.generate_only:
        args.output.write_text(json.dumps(dataset, indent=2)+'\n')
        print(dict(cases=len(dataset), labels=dict(Counter(r['expected'] for r in dataset))))
        return
    import torch
    import transformers
    torch.set_num_threads(4)
    sys.path.insert(0, str(args.model_dir.resolve()))
    from decider import Decider
    print('Loading model', flush=True)
    model=Decider.from_pretrained(str(args.model_dir.resolve()))
    results=[]
    for row in dataset:
        for order in [list(OPTIONS), list(reversed(OPTIONS))]:
            start=time.perf_counter()
            ans=model.answer(state=row['state'], questions={'q': dict(type='choice',
                instructions=RULES, criteria={k:OPTIONS[k] for k in order})})['answers']['q']
            results.append(dict(**row, order=order, predicted=ans['choice'],
                probabilities=ans['probabilities'], latency_ms=(time.perf_counter()-start)*1000))
        print(row['id'], row['expected'], [r['predicted'] for r in results[-2:]], flush=True)
        # Preserve partial work if inference is interrupted.
        args.output.write_text(json.dumps(dict(status='partial', rows=results), indent=2)+'\n')
    correct=sum(r['expected']==r['predicted'] for r in results)
    stop=[r for r in results if r['expected']=='stop']
    go=[r for r in results if r['expected']=='continue']
    lat=sorted(r['latency_ms'] for r in results)
    summary=dict(n_scenarios=len(dataset), n_requests=len(results), correct=correct,
        accuracy=correct/len(results), unsafe_misses=sum(r['predicted']=='continue' for r in stop),
        stop_recall=sum(r['predicted']=='stop' for r in stop)/len(stop),
        false_stops=sum(r['predicted']=='stop' for r in go),
        order_flips=sum(results[i]['predicted']!=results[i+1]['predicted'] for i in range(0,len(results),2)),
        latency_median_ms=lat[len(lat)//2], latency_p95_ms=lat[math.ceil(.95*len(lat))-1],
        by_category={g:dict(n=len(rs),correct=sum(r['expected']==r['predicted'] for r in rs))
            for g in sorted({r['category'] for r in results})
            for rs in [[r for r in results if r['category']==g]]})
    artifact=dict(status='complete',model='tasksource/tasksource-decider-nano',revision=REVISION,
        benchmark='synthetic agreement with existing IMU checks, not real-world safety accuracy',
        device=str(model.device), torch_version=torch.__version__, transformers_version=transformers.__version__, instructions=RULES,
        weights_sha256=sha256_file(args.model_dir / 'model.safetensors'),
        files_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
            for p in args.model_dir.iterdir() if p.is_file() and p.suffix in ['.py','.json']},
        summary=summary, rows=results)
    args.output.write_text(json.dumps(artifact,indent=2)+'\n')
    print(json.dumps(summary,indent=2))

if __name__=='__main__': main()
