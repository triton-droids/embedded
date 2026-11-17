#!/usr/bin/env python3
"""Live progress of the JetPack 6 runs; run on the WSL host (Humble distro).
  watch -n 15 python3 progress.py         # j6_* runs
  python3 progress.py j6_diag             # tags starting with a prefix
Per run: state, leg reached, seconds since route_driver last logged, RTF,
visual and leg-odometry final error (% of path) once the run is done.
"""
import csv, math, os, re, sys, time

B = os.path.expanduser('~/split_test')
prefix = sys.argv[1] if len(sys.argv) > 1 else 'j6_'
GROUPS = ['base', 'ab_vio', 'ab_vo', 'c_vio', 'c_vo']


def pct(path):
    try:
        r = list(csv.DictReader(open(path)))
    except OSError:
        return None
    if len(r) < 10:
        return None
    p = sum(math.hypot(float(b['true_x']) - float(a['true_x']), float(b['true_y']) - float(a['true_y']))
            for a, b in zip(r, r[1:]))
    return 100 * float(r[-1]['position_error_m']) / p if p > 1 else None


def leg_pct(path):  # leg odometry: odom x = world +y at spawn
    try:
        r = list(csv.DictReader(open(path)))
    except OSError:
        return None
    if len(r) < 10:
        return None
    p = sum(math.hypot(float(b['true_x']) - float(a['true_x']), float(b['true_y']) - float(a['true_y']))
            for a, b in zip(r, r[1:]))
    x = r[-1]
    e = math.hypot(-float(x['y']) - float(x['true_x']), float(x['x']) - float(x['true_y']))
    return 100 * e / p if p > 1 else None


def read(p):
    try:
        return open(p).read()
    except OSError:
        return ''


tags = sorted(t for t in os.listdir(B) if t.startswith(prefix)) if os.path.isdir(B) else []
now = time.time()
print(time.strftime('%H:%M:%S'), f'{len(tags)} runs ({prefix}*)')
print(f"{'tag':22} {'state':9} {'leg':>5} {'idle':>5} {'RTF':>5} {'vis%':>5} {'leg%':>5}")
groups = {}
for t in tags:
    O = f'{B}/{t}'
    route = read(f'{O}/route.log')
    legs = re.findall(r'Leg (\d+)/16', route)
    leg = 16 if 'Route finished' in route else int(legs[-1]) if legs else 0
    end = read(f'{O}/end.txt').strip().replace('END=', '')
    done = os.path.exists(f'{O}/done')
    state = (end or 'ending') if done or end else 'running'
    if state == 'done':
        state = 'OK' if leg == 16 else 'short'
    try:
        idle = f"{now - os.stat(f'{O}/route.log').st_mtime:.0f}" if not done else ''
    except OSError:
        idle = ''
    rtf = re.findall(r'RTF=([\d.]+)', read(f'{O}/rtf.txt'))
    v = pct(f'{O}/loc.csv') if done else None
    lg = leg_pct(f'{O}/leg.csv') if done else None
    print(f"{t:22} {state:9} {leg:>3}/16 {idle:>5} {rtf[0] if rtf else '':>5} "
          f"{'' if v is None else f'{v:.2f}':>5} {'' if lg is None else f'{lg:.2f}':>5}")
    g = next((g for g in sorted(GROUPS, key=len, reverse=True) if t[len(prefix):].startswith(g)
              and t[len(prefix) + len(g):].isdigit()), None)
    if g and done:
        groups.setdefault(g, []).append((state, v))
if groups:
    print('\ngroup   done  OK  fail  vis% median')
    for g in GROUPS:
        rs = groups.get(g, [])
        if rs:
            vs = sorted(v for s, v in rs if v is not None)
            print(f"{g:7} {len(rs):>4} {sum(s == 'OK' for s, _ in rs):>3} {sum(s in ('paused', 'sim_died', 'stuck') for s, _ in rs):>5}"
                  f"  {vs[len(vs) // 2]:.2f}" if vs else '')
    left = 10 * len(GROUPS) - sum(len(v) for v in groups.values())
    print(f'about {left * 6.5 / 60:.1f} h left if the full batch (5 groups x 10) is running')
