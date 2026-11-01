"""Summarize retained publish-error handling trials, including all failed attempts."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def main():
    output = []
    for path in sorted((HERE/'results').glob('dds-fix-*.imu.json')):
        name = path.name.removesuffix('.imu.json')
        cpu_path = path.with_name(name+'.cpu.json')
        if not cpu_path.exists():
            continue
        data = json.loads(path.read_text())
        cpu = json.loads(cpu_path.read_text())
        log = path.with_name(name+'.log').read_text().split('user interrupted with ctrl-c (SIGINT)',1)[0]
        def last(topic):
            entries = data['status'].get(topic, [])
            return entries[-1]['value'] if entries else None
        item = dict(name=name, observer=data['summary'],
            last_imu_t=data['imu'][-1]['t'] if data['imu'] else None,
            imu=last('/imu/status'), can_timing=last('/can_timing'),
            safety=last('/safety/status'), policy=last('/policy/status'),
            wbc=last('/wbc/status'), sim=last('/sim/status'),
            cpp=last('/control_core/abstract_command_status'), motor_commands=data.get('motor_commands'),
            injections=[line for line in log.splitlines() if '[dds-fault-injection]' in line],
            process_deaths=[line for line in log.splitlines() if 'process has died' in line],
            faults=[line for line in log.splitlines() if 'LATCHED' in line],
            library_maps=cpu.get('library_maps',{}))
        output.append(item)
        print(name, 'deaths',len(item['process_deaths']), 'injections',len(item['injections']),
              'last_imu',item['last_imu_t'], 'cpp_stage',(item['cpp'] or {}).get('stage'),
              'wbc',(item['wbc'] or {}).get('state'),(item['wbc'] or {}).get('solved_count'))
    (HERE/'results/dds-fix-summary.json').write_text(json.dumps(output,indent=2)+'\n')


if __name__ == '__main__':
    main()
