#!/usr/bin/env python3
"""Summarize completed DDS simulation trials, without changing their labels."""
import argparse
from collections import Counter
import json
from pathlib import Path
import re
import statistics


def summarize(root):
    root=Path(root)
    path=root/'episodes.jsonl'
    entries=[json.loads(l) for l in path.read_text().splitlines() if l.strip()] if path.exists() else []
    status=json.loads((root/'status.json').read_text())
    reasons=Counter();crashes=Counter();rmws=Counter();rtfs=[];gaps=[];overruns=[]
    for e in entries:
        m=e.get('metadata',{});reasons[m.get('reason','no_metadata')]+=1
        rmws[m.get('rmw_implementation','unverified')]+=1
        log=root/f"episode_{e['episode']:06d}.log"
        if log.exists():
            crashes.update(re.findall(r'\[ERROR\] \[([^]]+)\]: process has died.*?exit code (-?\d+)',log.read_text(errors='replace')))
        samples=root/f"episode_{e['episode']:06d}"/'samples.jsonl'
        if e['valid'] and samples.exists():
            rows=[json.loads(l) for l in samples.open()]
            if len(rows)>1:
                wall=rows[-1]['wall_monotonic_s']-rows[0]['wall_monotonic_s']
                if wall>0:rtfs.append((rows[-1]['sim_time_s']-rows[0]['sim_time_s'])/wall)
                gaps.extend((b['wall_monotonic_s']-a['wall_monotonic_s'])*1000 for a,b in zip(rows,rows[1:]))
                overruns.append(rows[-1]['physics_overruns'])
    gaps.sort()
    result=dict(state=status['state'],requested_rmw=status.get('requested_rmw'),
        attempts=len(entries),valid=sum(e['valid'] for e in entries),failed=sum(not e['valid'] for e in entries),
        requested_rmw_verified_episodes=sum(e.get('rmw_verified',False) and bool(e.get('metadata')) for e in entries),
        actual_rmw_counts=dict(rmws),termination_reasons=dict(reasons),
        crashed_processes={f'{name}:exit{code}':n for (name,code),n in crashes.items()},
        elapsed_s=status.get('elapsed_s'),
        median_episode_real_time_factor=statistics.median(rtfs) if rtfs else None,
        imu_wall_interval_ms_median=statistics.median(gaps) if gaps else None,
        imu_wall_interval_ms_p95=gaps[max(0,int(.95*len(gaps))-1)] if gaps else None,
        imu_wall_gaps_over_20ms=sum(v>20 for v in gaps),imu_intervals=len(gaps),
        total_physics_overruns=sum(overruns),
        note='Runtime startup/data validity test, not a new model-accuracy benchmark. Timing is measured only on valid episodes.')
    (root/'dds_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    (root/'DDS_REPORT.md').write_text(f'''# DDS full-stack simulation test

Requested RMW: `{result['requested_rmw']}`.
Attempts: {result['attempts']}; valid: {result['valid']}; failed/excluded: {result['failed']}.
Actual RMW verified in simulator metadata: {result['requested_rmw_verified_episodes']} episodes.
Final runner state: `{result['state']}`. Runtime: {result['elapsed_s']} seconds.

The existing policy, C++ control forwarding, motor model, IMUs and Safety are
retained. Camera/depth/viewer are off. Cyclone is a local binary overlay, with
localhost-only discovery and no multicast. This measures repeated startup,
communication and data validity; fall/recovery outcomes also depend on physics,
controller and scheduling and are not pure DDS performance measurements.
`dds_summary.json` includes failure-process counts and observed sensor timing.
`episodes.jsonl` and per-episode logs contain complete evidence. Existing
trained-model control integration is unchanged.
''')
    return result


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('output',type=Path)
    print(json.dumps(summarize(ap.parse_args().output),indent=2))
