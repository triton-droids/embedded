"""Serial DDS A/B/C trials on the held normal chase stack; never enables motors."""
import argparse
import os
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', required=True, help='Unique retained result prefix')
    parser.add_argument('--rounds', type=int, default=2)
    args = parser.parse_args()
    if args.rounds < 1:
        parser.error('--rounds must be positive')
    # Each child begins with the original shell environment. Private overlay is sourced only for B/C.
    for round_index in range(args.rounds):
        conditions = ('baseline', 'strict', 'strict5ms') if round_index % 2 == 0 else ('strict5ms', 'strict', 'baseline')
        for condition in conditions:
            name = f'{args.label}-{condition}-{round_index+1}'
            base = HERE/'results'/name
            if base.with_suffix('.log').exists():
                raise FileExistsError(base)
            env = dict(os.environ)
            for key in ('LD_PRELOAD', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                        'RMW_FASTRTPS_USE_QOS_FROM_XML'):
                env.pop(key, None)
            env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
            command = 'source scripts/activate_rosenv.sh\n'
            if condition != 'baseline':
                command += 'source experimental/reference_warp/activate_strict_dds.sh\n'
            if condition == 'strict5ms':
                env['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(HERE/'dds_writer_5ms.xml')
                env['RMW_FASTRTPS_USE_QOS_FROM_XML'] = '0'
            command += ('exec python experimental/reference_warp/stack_cpu_probe.py '
                        '--gui true --domain 232 --name "$TRIAL_NAME" '
                        '--preload experimental/reference_warp/generated/publish_probe/libpublish_wait.so '
                        '--trace-dir "$TRIAL_TRACE"')
            env['TRIAL_NAME'] = name
            env['TRIAL_TRACE'] = str(base)+'-trace'
            print(f'START {name}', flush=True)
            subprocess.run(['bash', '-e', '-c', command], cwd=ROOT, env=env, check=True)
            print(f'FINISH {name}', flush=True)


if __name__ == '__main__':
    main()
