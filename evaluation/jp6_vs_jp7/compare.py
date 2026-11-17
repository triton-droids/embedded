#!/usr/bin/env python3
"""JetPack 6 vs JetPack 7 split-run comparison; run on this laptop.

Per group: runs, routes finished, falls/pauses, real-time factor, final error as
% of path for the visual estimate and leg odometry, and Jetson load from
tegrastats (mean CPU %, busiest core %, CPU MHz, GPU %, RAM, VDD_IN).
  python3 compare.py                 # all groups
  python3 compare.py ab_vio ab_vo    # some groups
  python3 compare.py --runs          # also one line per run
JetPack 7 data: WSL Jazzy distro (port 2222) + Jetson 192.168.1.44, tags base01..
JetPack 6 data: WSL Humble distro (port 22) + Jetson 100.71.186.19, tags j6_base01..
"""
import json
import os
import statistics as st
import subprocess
import sys

HOSTS = {
    'JP7': dict(wsl=['-p', '2222', os.environ.get('WSL_HOST', 'rcli@192.168.1.129')], jet='rcli@192.168.1.44', prefix=''),
    'JP6': dict(wsl=[os.environ.get('WSL_HOST', 'rcli@192.168.1.129')], jet='td@100.71.186.19', prefix='j6_'),
}
GROUPS = ['base', 'ab_vio', 'ab_vo', 'c_vio', 'c_vo']
EXCLUDE = {'c_vio09'}  # JetPack 7: robot never moved (chase commands lost), not a localization result.

TEGRA = r'''
import glob, json, os, re, statistics as st
out = {}
for path in sorted(glob.glob(os.path.expanduser('~/split_test/PREFIX*/tegra.log'))):
    tag = path.split('/')[-2]; cpu = []; peak = []; mhz = []; gpu = []; ram = []; pin = []
    lines = open(path).read().splitlines()
    for l in lines[len(lines) // 5:]:  # skip start-up (first fifth)
        m = re.search(r'CPU \[([^\]]+)\]', l)
        if not m: continue
        cores = [re.match(r'(\d+)%@(\d+)', c) for c in m.group(1).split(',')]
        cores = [c for c in cores if c]
        if not cores: continue
        cpu.append(st.mean(int(c.group(1)) for c in cores)); peak.append(max(int(c.group(1)) for c in cores))
        mhz.append(st.mean(int(c.group(2)) for c in cores))
        g = re.search(r'GR3D_FREQ (\d+)%', l); gpu.append(int(g.group(1)) if g else 0)
        r = re.search(r'RAM (\d+)/', l); ram.append(int(r.group(1)) if r else 0)
        p = re.search(r'VDD_IN (\d+)mW', l); pin.append(int(p.group(1)) if p else 0)
    if cpu:
        out[tag] = dict(cpu=st.mean(cpu), peak=st.mean(peak), mhz=st.mean(mhz), gpu=st.mean(gpu), ram=st.mean(ram), vdd=st.mean(pin))
print(json.dumps(out))
'''


def ssh(args, cmd, stdin=None):
    r = subprocess.run(['ssh', '-o', 'ConnectTimeout=10', *args, cmd], input=stdin,
                       capture_output=True, text=True, timeout=300)
    return r.stdout


def collect(name, groups):
    h = HOSTS[name]
    listing = ssh(h['wsl'], 'ls ~/split_test').split()
    tags = [t for t in listing if any(t.startswith(h['prefix'] + g) and t[len(h['prefix'] + g):].isdigit() for g in groups)]
    runs = {}
    if tags:
        for line in ssh(h['wsl'], 'python3 ~/split_summary.py ' + ' '.join(tags)).splitlines():
            d = json.loads(line)
            runs[d['tag']] = d
    try:
        tegra = json.loads(ssh([h['jet']], 'python3 -', TEGRA.replace('PREFIX', h['prefix'])) or '{}')
    except (json.JSONDecodeError, subprocess.TimeoutExpired):
        tegra = {}
    for t, d in runs.items():
        d['tegra'] = tegra.get(t)
    return runs


def med(xs):
    xs = [x for x in xs if x is not None]
    return f'{st.median(xs):.2f}' if xs else '-'


def mean(xs, fmt='{:.0f}'):
    xs = [x for x in xs if x is not None]
    return fmt.format(st.mean(xs)) if xs else '-'


def main():
    show_runs = '--runs' in sys.argv
    groups = [a for a in sys.argv[1:] if not a.startswith('--')] or GROUPS
    data = {n: collect(n, groups) for n in HOSTS}
    hdr = f"{'group':7} {'JP':3} {'runs':>4} {'done':>4} {'fail':>4} {'RTF':>5} {'vis%':>5} {'leg%':>5} {'CPU%':>5} {'peak':>5} {'MHz':>5} {'GPU%':>5} {'RAM':>5} {'mW':>5}"
    print(hdr)
    for g in groups:
        for n in HOSTS:
            p = HOSTS[n]['prefix']
            rs = [d for t, d in sorted(data[n].items())
                  if t.startswith(p + g) and t[len(p + g):].isdigit() and t[len(p):] not in EXCLUDE]
            if not rs:
                print(f'{g:7} {n:3} {0:>4}')
                continue
            done = sum(1 for d in rs if d['finished'])
            fails = sum(1 for d in rs if d['end'] in ('END=paused', 'END=sim_died', 'END=stuck'))
            rtf = [float(d['rtf']) for d in rs if d['rtf'] not in (None, 'nan')]
            te = [d['tegra'] for d in rs if d.get('tegra')]
            print(f"{g:7} {n:3} {len(rs):>4} {done:>4} {fails:>4} {mean(rtf, '{:.3f}'):>5} "
                  f"{med([(d['est'] or {}).get('pct') for d in rs]):>5} {med([(d['leg'] or {}).get('pct') for d in rs]):>5} "
                  f"{mean([t['cpu'] for t in te]):>5} {mean([t['peak'] for t in te]):>5} {mean([t['mhz'] for t in te]):>5} "
                  f"{mean([t['gpu'] for t in te]):>5} {mean([t['ram'] for t in te]):>5} {mean([t['vdd'] for t in te]):>5}")
            if show_runs:
                for d in rs:
                    t = d.get('tegra') or {}
                    print(f"    {d['tag']:12} legs={d['legs']:>2} {d['end']:13} rtf={d['rtf']} "
                          f"vis={(d['est'] or {}).get('pct')} leg={(d['leg'] or {}).get('pct')} cpu={t.get('cpu', 0):.0f}")
    if not data['JP7']:
        import os
        print('\nJP7 hosts unreachable; saved JP7 results (jp7_reference.txt):')
        print(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'jp7_reference.txt')).read())
        return
    print('vis%/leg%: median final error as % of path. fail: physics paused (fall), simulator died or stuck.')
    print('CPU%/peak/MHz/GPU%/RAM(MB)/mW: Jetson tegrastats means after the first fifth of each run.')


if __name__ == '__main__':
    main()
