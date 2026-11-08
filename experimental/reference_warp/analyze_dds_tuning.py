"""Summarize every retained DDS trial including failed control paths."""
import argparse
import json
from pathlib import Path
from publish_probe.analyze import native_rows

HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', required=True)
    args = parser.parse_args()
    output = []
    for path in sorted((HERE/'results').glob(args.label+'-*.imu.json')):
        name = path.name.removesuffix('.imu.json')
        data = json.loads(path.read_text())
        cpu = json.loads(path.with_name(name+'.cpu.json').read_text())
        origin = data['clock']['start_monotonic_s']
        rows = [v for v in native_rows(HERE/'results'/(name+'-trace'))
                if v['layer'] == 'rmw' and origin <= v['begin_ns']/1e9 <= v['end_ns']/1e9 <= origin+32]
        for row in rows:
            row['observer_begin_s'] = row['begin_ns']/1e9-origin
        def last(topic):
            entries = data['status'].get(topic, [])
            return entries[-1]['value'] if entries else None
        imustatus = last('/imu/status') or {}
        safety = last('/safety/status')
        policy = last('/policy/status') or {}
        wbc = last('/wbc/status') or {}
        item = dict(name=name, observer=data['summary'], imu=imustatus, safety=safety,
                    policy=policy, wbc=wbc, library_maps=cpu.get('library_maps'),
                    native_errors=[v for v in rows if v['result'] != 0],
                    longest_native=sorted(rows, key=lambda v:-v['wall_ms'])[:15])
        item['last_imu_t_s'] = data['imu'][-1]['t'] if data['imu'] else None
        item['last_status_t_s'] = {topic:entries[-1]['t'] for topic,entries in data['status'].items() if entries}
        log = (HERE/'results'/(name+'.log')).read_text()
        before_teardown = log.split('user interrupted with ctrl-c (SIGINT)',1)[0]
        item['process_deaths_before_teardown'] = [line for line in before_teardown.splitlines()
                                                 if 'process has died' in line]
        item['sim'] = last('/sim/status')
        item['safety_latched_observations'] = [entry for entry in data['status'].get('/safety/status', [])
            if any(status.get('values', {}).get('latched') == 'True' for status in entry['value'])]
        item['steady_native_over_20ms'] = [v for v in rows if v['observer_begin_s']>=12 and v['wall_ms']>=20]
        steady_imu = [v for v in data['imu'] if v['t']>=12]
        item['steady_observer'] = dict(samples=len(steady_imu),
            max_receipt_gap_s=max((b['t']-a['t'] for a,b in zip(steady_imu,steady_imu[1:])), default=None),
            max_source_age_s=max((v['age'] for v in steady_imu), default=None))
        output.append(item)
        maximu = max((v['wall_ms'] for v in rows if v['topic']=='/imu/data_raw'), default=None)
        maxall = max((v['wall_ms'] for v in rows), default=None)
        print(name, 'imu_gap_ms', round(1000*imustatus.get('max_publish_gap_s',0),3),
              'imu_native_ms', maximu, 'all_native_ms', maxall, 'native_errors',len(item['native_errors']),
              'policy',policy.get('state'),policy.get('fault'), 'wbc',wbc.get('solved_count'))
    destination = HERE/'results'/(args.label+'-summary.json')
    destination.write_text(json.dumps(output,indent=2)+'\n')


if __name__ == '__main__':
    main()
