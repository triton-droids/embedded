#!/usr/bin/env python3
"""Match a shadow_node.py log with a collect_fall_data.py run and report.

Per-sample truth (gantry, estop, fault, fall time) comes from the collector's
samples.jsonl; online scores are joined by the IMU header stamp, which the
collector records as ros_stamp_ns for exactly the published sample.

Event metrics reuse the training definitions (autotrain.policy_metrics):
eligible = 20-sample window without gantry/estop/fault and 5-20 ms gaps;
labels from collection_rules.future_label; an alarm needs `confirm` consecutive
eligible windows above threshold; false-alarm onsets on label-0 windows with a
1-second cooldown per episode. Also re-scores the windows offline with the same
model to verify the live feature/inference path.

Usage: shadow_analyze.py --collection DIR --shadow shadow.jsonl --best best.json
                         --model model_torchscript.pt --out DIR
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import torch

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'simulation/humanoid_sim'))
from humanoid_sim.collection_rules import future_label  # noqa: E402

HEADS = {'300ms': (0, .3), '600ms': (1, .6)}


def pct(xs, q):
    return float(np.percentile(xs, q)) if len(xs) else None


def load_shadow(path):
    scores, events = {}, []
    for line in open(path):
        r = json.loads(line)
        if r['kind'] == 'score':
            scores[r['stamp_ns']] = r
        else:
            events.append(r)
    return scores, events


def alarms(score, eligible_idx, thr, confirm):
    """Index set of alarm windows: k consecutive eligible windows >= thr."""
    out, run, prev = set(), 0, None
    for i in eligible_idx:
        if score[i] >= thr:  # nan (no live score) compares False
            run = run + 1 if prev == i - 1 else 1
        else:
            run = 0
        if run >= confirm:
            out.add(i)
        prev = i
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--collection', type=Path, required=True)
    ap.add_argument('--shadow', type=Path, required=True)
    ap.add_argument('--best', type=Path, required=True)
    ap.add_argument('--model', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--exclude', default='',
                    help='comma-separated episode numbers to skip (e.g. compare_runs.py dds_stall)')
    a = ap.parse_args()
    excluded = {int(x) for x in a.exclude.split(',') if x.strip()}
    a.out.mkdir(parents=True, exist_ok=True)
    policy = {h: (float(p['threshold']), int(p['confirm_frames']))
              for h, p in json.load(open(a.best))['validation_policies'].items()}
    model = torch.jit.load(str(a.model)).eval()
    scores, events = load_shadow(a.shadow)
    estop_on = sorted(e['recv_mono'] for e in events if e['kind'] == 'estop' and e['value'])
    alarm_events = [e for e in events if e['kind'] == 'alarm']

    per_ep, feat_diff, score_diff = [], [], []
    infer = [s['infer_ms'] for s in scores.values()]
    transport = [(s['recv_wall_ns'] - s['stamp_ns']) / 1e6 for s in scores.values()]
    agg = {h: dict(eligible_falls=0, detected=0, leads=[], before_estop=[], fa_onsets=0,
                   neg_s=0., pos=0, tp=0, fp=0, fall_any_alarm_before=0,
                   recovered_alarm=0, nopush_alarm=0, online_before_estop_ms=[],
                   online_offline_mismatch=0) for h in HEADS}
    n_recovered = n_nopush = n_fall = 0
    for line in open(a.collection / 'episodes.jsonl'):
        e = json.loads(line)
        if e['episode'] in excluded:
            per_ep.append(dict(episode=e['episode'], valid=e['valid'], excluded=True))
            continue
        if not e['valid']:
            per_ep.append(dict(episode=e['episode'], valid=False))
            continue
        d = a.collection / f"episode_{e['episode']:06d}"
        meta = json.loads((d / 'metadata.json').read_text())
        rows = [json.loads(l) for l in (d / 'samples.jsonl').open()]
        ts = np.array([r['sim_time_s'] for r in rows])
        x = np.array([r['features'] for r in rows], dtype=np.float32)
        fall = meta['fall_time_sim_s']
        pushed = meta.get('push_applied_steps', 0) > 0
        first_estop = next((r['sim_time_s'] for r in rows if r['estop'] and not r['base_held']), None)
        eligible, labels = [], {h: {} for h in HEADS}
        online = np.full(len(rows), np.nan, dtype=np.float64), np.full(len(rows), np.nan)
        matched = 0
        for i in range(19, len(rows)):
            w = rows[i - 19:i + 1]
            gaps = np.diff(ts[i - 19:i + 1])
            if (any(r['base_held'] or r['control_fault'] or r['estop'] for r in w)
                    or np.any(gaps < .005 - 1e-9) or np.any(gaps > .02 + 1e-9)):
                continue
            eligible.append(i)
            for h, (_, hz) in HEADS.items():
                labels[h][i] = future_label(float(ts[i]), fall, float(ts[-1]), hz)
            s = scores.get(rows[i]['ros_stamp_ns'])
            if s is not None:
                matched += 1
                online[0][i], online[1][i] = s['p300'], s['p600']
                feat_diff.append(float(np.max(np.abs(np.array(s['f']) - x[i]))))
        if eligible:
            win = torch.from_numpy(np.stack([x[i - 19:i + 1] for i in eligible]))
            with torch.no_grad():
                off = torch.sigmoid(model(win)).numpy()
            offline = np.full((2, len(rows)), np.nan)
            offline[:, eligible] = off.T
            m = ~np.isnan(online[0])
            if m.any():
                score_diff.append(float(np.nanmax(np.abs(np.vstack(online)[:, m] - offline[:, m]))))
        ep = dict(episode=e['episode'], valid=True, mode=meta['scenario']['mode']
                  if meta.get('scenario') else None, fall=fall is not None, pushed=pushed,
                  recovered=bool(meta.get('recovered_after_push')),
                  push_n=float(np.linalg.norm(meta['push_force_world_n'])),
                  eligible=len(eligible), matched_online=matched)
        if fall is not None:
            n_fall += 1
        elif pushed:
            n_recovered += 1
        else:
            n_nopush += 1
        for h, (col, hz) in HEADS.items():
            thr, confirm = policy[h]
            sc = np.vstack(online)[col]  # live scores; missing -> nan -> never >= thr
            al = alarms(sc, eligible, thr, confirm)
            g = agg[h]
            pos = [i for i in eligible if labels[h][i] == 1]
            neg = [i for i in eligible if labels[h][i] == 0]
            g['pos'] += len(pos)
            g['tp'] += sum(i in al for i in pos)
            g['fp'] += sum(i in al for i in neg)
            g['neg_s'] += .01 * len(neg)
            last = -math.inf
            for i in sorted(al):
                if labels[h][i] == 0 and (i - 1) not in al and ts[i] - last >= 1.:
                    g['fa_onsets'] += 1
                    last = ts[i]
            first_alarm = min(al) if al else None
            ep[f'first_alarm_{h}_s'] = None if first_alarm is None else float(ts[first_alarm] - meta['released_at_sim_s'])
            if fall is not None:
                if any(i < len(ts) and ts[i] < fall for i in al):
                    g['fall_any_alarm_before'] += 1
                if pos:
                    g['eligible_falls'] += 1
                    hit = [i for i in pos if i in al]
                    if hit:
                        g['detected'] += 1
                        g['leads'].append(fall - ts[hit[0]])
                        if first_estop is not None:
                            g['before_estop'].append(first_estop - ts[hit[0]])
                        # Live wall-clock: shadow alarm log vs Safety estop receipt.
                        st = rows[hit[0]]['ros_stamp_ns']
                        ae = next((x for x in alarm_events if x['head'] == h and
                                   x['stamp_ns'] <= st and st - x['stamp_ns'] < 10**9), None)
                        ae = ae or next((x for x in alarm_events if x['head'] == h and
                                         0 <= x['stamp_ns'] - st < 10**9), None)
                        if ae:
                            nxt = next((t for t in estop_on if t >= ae['recv_mono']), None)
                            if nxt is not None and nxt - ae['recv_mono'] < 5:
                                g['online_before_estop_ms'].append((nxt - ae['recv_mono']) * 1e3)
                ep[f'detected_{h}'] = bool(pos and any(i in al for i in pos))
            elif pushed:
                g['recovered_alarm'] += bool(al)
            else:
                g['nopush_alarm'] += bool(al)
        per_ep.append(ep)

    summary = dict(excluded=sorted(excluded), episodes_valid=sum(p['valid'] and not p.get('excluded') for p in per_ep),
                   episodes_invalid=sum(not p['valid'] for p in per_ep),
                   falls=n_fall, pushed_recovered=n_recovered, no_push=n_nopush,
                   online_coverage=None, feature_max_abs_diff=max(feat_diff) if feat_diff else None,
                   online_vs_offline_score_max_abs_diff=max(score_diff) if score_diff else None,
                   infer_ms=dict(p50=pct(infer, 50), p99=pct(infer, 99), max=max(infer) if infer else None),
                   imu_transport_ms=dict(p50=pct(transport, 50), p99=pct(transport, 99),
                                         max=max(transport) if transport else None),
                   heads={})
    elig = sum(p.get('eligible', 0) for p in per_ep)
    summary['online_coverage'] = sum(p.get('matched_online', 0) for p in per_ep) / max(1, elig)
    for h, g in agg.items():
        thr, confirm = policy[h]
        summary['heads'][h] = dict(
            threshold=thr, confirm_frames=confirm,
            eligible_fall_episodes=g['eligible_falls'], detected_fall_episodes=g['detected'],
            event_recall=g['detected'] / max(1, g['eligible_falls']),
            falls_with_any_alarm_before_contact=g['fall_any_alarm_before'],
            window_recall=g['tp'] / max(1, g['pos']), precision=g['tp'] / max(1, g['tp'] + g['fp']),
            false_alarm_onsets=g['fa_onsets'], negative_exposure_min=g['neg_s'] / 60,
            false_alarm_onsets_per_hour=g['fa_onsets'] / max(g['neg_s'] / 3600, 1e-9),
            recovered_episodes_with_alarm=g['recovered_alarm'],
            no_push_episodes_with_alarm=g['nopush_alarm'],
            contact_lead_ms=dict(median=pct(g['leads'], 50) and pct(g['leads'], 50) * 1e3,
                                 min=min(g['leads']) * 1e3 if g['leads'] else None),
            before_safety_estop_ms_sim=dict(median=pct(g['before_estop'], 50) and pct(g['before_estop'], 50) * 1e3,
                                            min=min(g['before_estop']) * 1e3 if g['before_estop'] else None,
                                            negative=sum(v < 0 for v in g['before_estop'])),
            before_safety_estop_ms_live=dict(n=len(g['online_before_estop_ms']),
                                             median=pct(g['online_before_estop_ms'], 50),
                                             min=min(g['online_before_estop_ms']) if g['online_before_estop_ms'] else None))
    (a.out / 'summary.json').write_text(json.dumps(summary, indent=2, default=float))
    (a.out / 'episodes.json').write_text(json.dumps(per_ep, indent=1, default=float))
    print(json.dumps(summary, indent=2, default=float))


if __name__ == '__main__':
    main()
