#!/usr/bin/env python3
"""Pair two collect_fall_data.py runs by seed (e.g. policy PD vs WBC).

Each episode is classified as invalid, dds_stall, chain_fault, fall, recovered
(pushed, no fall) or no_push, using the first marker in the launch log before
the Safety fall_detection latch:
- dds_stall: sensor/transport stall (IMU freshness trip, policy IMU timeout,
  stale commands). Excluded, as in docs/reports/2026-10-30-wbc-pd-comparison.md.
- chain_fault: the control stack itself stopped commanding (WBC input timeout,
  C++/backend command timeout). Counted as a fall of that configuration.

Usage: compare_runs.py --a NAME=DIR --b NAME=DIR --out DIR
Writes pairs.json, summary.json and prints excluded episode numbers per run,
which shadow_analyze.py accepts via --exclude.
"""
import argparse
import json
import re
from pathlib import Path

import numpy as np

LATCH = re.compile(r'SAFETY ESTOP LATCHED: ([a-z_]+)')
# Command/feedback interruptions that disable or starve the motors by themselves.
TRANSPORT_MARKERS = ('SAFETY ESTOP LATCHED: imu_freshness', '[tracking_policy_node]: IMU timeout',
                     'Rejecting stale')
CHAIN_MARKERS = ('Input source/receipt timeout', 'Command timeout for', 'Motor control lost')
FALLS = ('fall', 'chain_fault')


def first(log, markers):
    return min((i for i in (log.find(s) for s in markers) if i >= 0), default=-1)


def classify(run):
    out = {}
    for line in open(run / 'episodes.jsonl'):
        e = json.loads(line)
        m = e.get('metadata') or {}
        log = (run / f"episode_{e['episode']:06d}.log").read_text(errors='replace')
        latch = LATCH.search(log)
        first_latch = latch.group(1) if latch else None
        force = float(np.linalg.norm(m.get('push_force_world_n', [0, 0, 0])))
        # Launch output is interleaved in arrival order; compare first positions.
        fall_latch = log.find('SAFETY ESTOP LATCHED: fall_detection')
        before = lambda i: i >= 0 and (fall_latch < 0 or i < fall_latch)
        transport, chain = first(log, TRANSPORT_MARKERS), first(log, CHAIN_MARKERS)
        if not e['valid']:
            kind = 'invalid'
        elif m.get('fall_time_sim_s') is not None:
            if before(transport) and (not before(chain) or transport <= chain):
                kind = 'dds_stall'
            elif before(chain):
                kind = 'chain_fault'
            else:
                kind = 'fall'
        elif m.get('push_applied_steps', 0) > 0:
            kind = 'recovered'
        else:
            kind = 'no_push'
        out[e['seed']] = dict(episode=e['episode'], kind=kind, first_latch=first_latch,
                              mode=e['mode'], push_n=force,
                              push_applied=m.get('push_applied_steps', 0) > 0,
                              fall_after_release_s=None if m.get('fall_time_sim_s') is None
                              else m['fall_time_sim_s'] - m['released_at_sim_s'],
                              reason=m.get('reason'))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--a', required=True)
    ap.add_argument('--b', required=True)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    (na, da), (nb, db) = (x.split('=', 1) for x in (args.a, args.b))
    A, B = classify(Path(da)), classify(Path(db))
    args.out.mkdir(parents=True, exist_ok=True)
    summary = {}
    for name, run in ((na, A), (nb, B)):
        kinds = [v['kind'] for v in run.values()]
        usable = [v for v in run.values() if v['kind'] in FALLS + ('recovered', 'no_push')]
        summary[name] = dict(
            episodes=len(run), **{k: kinds.count(k) for k in
                                  ('invalid', 'dds_stall', 'chain_fault', 'fall', 'recovered', 'no_push')},
            fall_rate_usable=sum(v['kind'] in FALLS for v in usable) / max(1, len(usable)),
            falls_before_push=sum(v['kind'] in FALLS and not v['push_applied'] for v in usable),
            by_mode={mode: dict(usable=sum(v['mode'] == mode for v in usable),
                                falls=sum(v['mode'] == mode and v['kind'] in FALLS for v in usable))
                     for mode in ('standing_reference', 'walking_reference')},
            excluded_episodes=sorted(v['episode'] for v in run.values()
                                     if v['kind'] in ('invalid', 'dds_stall')))
    pairs, table = [], {}
    for seed in sorted(set(A) & set(B)):
        a, b = A[seed], B[seed]
        pairs.append(dict(seed=seed, mode=a['mode'], push_n=round(a['push_n'], 1),
                          **{na: a['kind'], nb: b['kind']}))
        if {a['kind'], b['kind']} & {'invalid', 'dds_stall'}:
            continue
        key = f"{na}:{'fall' if a['kind'] in FALLS else 'ok'} / {nb}:{'fall' if b['kind'] in FALLS else 'ok'}"
        table[key] = table.get(key, 0) + 1
    summary['paired_both_usable'] = table
    bins = [(0, 25), (25, 35), (35, 70)]
    summary['falls_by_force_paired'] = {
        f'{lo}-{hi}N': {n: f"{sum(p[n] in FALLS for p in pairs if lo <= p['push_n'] < hi and not {p[na], p[nb]} & {'invalid', 'dds_stall'})}"
                         f"/{sum(1 for p in pairs if lo <= p['push_n'] < hi and not {p[na], p[nb]} & {'invalid', 'dds_stall'})}"
                        for n in (na, nb)} for lo, hi in bins}
    (args.out / 'pairs.json').write_text(json.dumps(pairs, indent=1))
    (args.out / 'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
