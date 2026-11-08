#!/usr/bin/env python3
"""Run isolated full-stack headless simulation episodes for a bounded night."""
import argparse
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import sys
import signal
import subprocess
import time


def atomic_json(path, value):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value, indent=2)+'\n')
    tmp.replace(path)


def stop_child(child):
    # Also signal descendants when ros2 itself has already exited.
    for sig, timeout in [(signal.SIGINT, 10), (signal.SIGTERM, 3), (signal.SIGKILL, 3)]:
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            break
        try:
            child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
        # Give launch time to reap its children before the next episode.
        break


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--hours', type=float, default=6.5)
    ap.add_argument('--episode-seconds', type=float, default=15.)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--domain', type=int, default=87)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--max-episodes', type=int, default=0, help='0: limited only by --hours')
    ap.add_argument('--profile',choices=['standard','recovery'],default='standard')
    ap.add_argument('--rmw',choices=['rmw_cyclonedds_cpp','rmw_fastrtps_cpp'],default=None)
    ap.add_argument('--max-consecutive-failures',type=int,default=5)
    ap.add_argument('--launch-arg',action='append',default=[],metavar='NAME:=VALUE',
                    help='Extra sim.launch.py argument, repeatable (e.g. control_rate_hz:=200)')
    ap.add_argument('--wbc',action='store_true',
                    help='Insert the TSID WBC between RL and C++ (enable_wbc:=true, policy_impedance)')
    args = ap.parse_args()
    reserved = {'collection_output_dir','collection_seed','collection_duration_s','collection_profile',
                'ros_domain_id','enable_wbc','auto_start','hold_first_frame'}
    for extra in args.launch_arg:
        if ':=' not in extra or extra.split(':=',1)[0] in reserved:
            ap.error(f'invalid or reserved --launch-arg: {extra}')
    if not math.isfinite(args.hours) or not 0 < args.hours <= 24:
        ap.error('--hours must be in (0, 24]')
    if not math.isfinite(args.episode_seconds) or not 1 <= args.episode_seconds <= 120:
        ap.error('--episode-seconds must be in [1, 120]')
    if not 0 <= args.domain <= 101 or args.seed < 0 or args.max_episodes < 0 or args.max_consecutive_failures<1:
        ap.error('invalid domain, seed or maximum episode count')
    if not shutil.which('ros2'):
        ap.error('Source scripts/activate_rosenv.sh first (ROS_DISTRO=humble on this host)')
    if args.rmw:
        os.environ['RMW_IMPLEMENTATION']=args.rmw
        from rclpy.utilities import get_rmw_implementation_identifier
        if get_rmw_implementation_identifier()!=args.rmw:
            ap.error('Requested RMW did not load')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    lock = open(f'/tmp/humanoid-fall-collection-domain-{args.domain}.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        ap.error('another collector already owns this ROS domain')
    stop = False
    def request_stop(signum, frame):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=str(args.domain),
               OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1',
               ROS_LOG_DIR=str(output/'ros_logs'))
    started = time.monotonic()
    deadline = started + args.hours*3600
    status = dict(pid=os.getpid(), started_utc=datetime.now(timezone.utc).isoformat(),
                  hours=args.hours, domain=args.domain, output=str(output),
                  completed=0, failed=0, positive_windows=0, negative_windows=0,
                  state='running')
    status['requested_rmw']=args.rmw or env.get('RMW_IMPLEMENTATION','default')
    repo = Path(__file__).resolve().parents[1]
    commit = subprocess.run(['git', '-C', str(repo), 'rev-parse', 'HEAD'],
                            capture_output=True, text=True).stdout.strip()
    atomic_json(output/'run.json', dict(**status, git_commit=commit,
        middleware_environment={k:env.get(k) for k in ['RMW_IMPLEMENTATION','CYCLONEDDS_URI','ROS_LOCALHOST_ONLY','ROS_DOMAIN_ID']},
        git_diff=subprocess.run(['git','-C',str(repo),'diff'],capture_output=True,text=True).stdout,
        args=vars(args) | {'output':str(output)},
        note='active Safety retained; physics continues after motor disable; no camera/viewer'))
    # Include untracked collector sources as well as the tracked diff.
    snapshot = output/'source'
    for relative in ['scripts/collect_fall_data.py',
                     'simulation/humanoid_sim/humanoid_sim/collection.py',
                     'simulation/humanoid_sim/humanoid_sim/collection_rules.py',
                     'simulation/humanoid_sim/humanoid_sim/main.py',
                     'simulation/humanoid_sim/humanoid_sim/world.py',
                     'simulation/humanoid_sim/humanoid_sim/imu_nodes.py',
                     'simulation/humanoid_sim/launch/sim.launch.py',
                     'simulation/humanoid_sim/config/sim.yaml',
                     'humanoid_control/humanoid_safety/config/safety.yaml',
                     'humanoid_control/motor_control_hybrid/config/motors.yaml']:
        target = snapshot/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(repo/relative, target)
    if args.rmw:
        for relative in ['scripts/run_cyclonedds_test.sh','scripts/summarize_dds_test.py',
                         'simulation/humanoid_sim/config/cyclonedds_local.xml']:
            target=snapshot/relative;target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(repo/relative,target)
    child = None
    consecutive_failures, episode = 0, 0
    try:
        while not stop and time.monotonic() < deadline:
            if args.max_episodes and episode >= args.max_episodes:
                break
            if shutil.disk_usage(output).free < 2*1024**3:
                status['state'] = 'disk_space_low'
                break
            seed = args.seed + episode
            episode_dir = output/f'episode_{episode:06d}'
            mode = 'standing_reference' if seed % 2 else 'walking_reference'
            if args.profile=='recovery':
                from humanoid_sim.collection_rules import recovery_scenario
                mode = recovery_scenario(seed)['mode']
            cmd = ['ros2','launch','humanoid_sim','sim.launch.py',
                   'auto_start:=true','enable_vision:=false','enable_depth:=false',
                   'use_viewer:=false','use_rviz:=false','minimal:=false',
                   'enable_policy:=true','connect_policy:=true',f'enable_wbc:={str(args.wbc).lower()}',
                   'localhost_only:=1',f'ros_domain_id:={args.domain}',
                   f'web_port:={18000+args.domain}',
                   f'hold_first_frame:={str(mode=="standing_reference").lower()}',
                   f'collection_output_dir:={episode_dir}', f'collection_seed:={seed}',
                   f'collection_duration_s:={args.episode_seconds}',
                   f'collection_profile:={args.profile}']
            if args.wbc:
                # WBC needs the rosenv interpreter (Pinocchio + patched TSID).
                cmd.append(f'wbc_python_executable:={sys.executable}')
            cmd += args.launch_arg
            status.update(episode=episode, seed=seed, mode=mode)
            atomic_json(output/'status.json', status)
            print(f'Episode {episode}: seed={seed}, {mode}', flush=True)
            with (output/f'episode_{episode:06d}.log').open('w') as log:
                child = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                                         start_new_session=True)
                episode_deadline = min(deadline, time.monotonic()+args.episode_seconds*4+120)
                last_status = time.monotonic()
                while child.poll() is None and not stop and time.monotonic() < episode_deadline:
                    time.sleep(.5)
                    if time.monotonic()-last_status >= 5:
                        status.update(elapsed_s=time.monotonic()-started,
                                      remaining_s=max(0.,deadline-time.monotonic()),
                                      current_episode_pid=child.pid)
                        atomic_json(output/'status.json', status)
                        last_status = time.monotonic()
                stop_child(child)
                returncode = child.returncode
                child = None
            metadata_path = episode_dir/'metadata.json'
            metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
            counts = metadata.get('label_counts', {})
            log_text = (output/f'episode_{episode:06d}.log').read_text()
            # A node dying after launch began SIGINT teardown (a known rclpy
            # take_message race) does not invalidate the recorded episode.
            teardown = log_text.find("sending signal 'SIGINT'")
            running_part = log_text if teardown < 0 else log_text[:teardown]
            process_crashed = 'process has died' in running_part
            shutdown_errors = log_text.count('process has died') - running_part.count('process has died')
            rmw_verified = not args.rmw or metadata.get('rmw_implementation')==args.rmw
            valid = (metadata.get('reason') in ('completed', 'fall') and
                     counts.get('0',0)+counts.get('1',0) > 0 and not process_crashed
                     and returncode == 0 and rmw_verified)
            status['completed' if valid else 'failed'] += 1
            if valid:
                status['positive_windows'] += counts.get('1',0)
                status['negative_windows'] += counts.get('0',0)
            entry = dict(episode=episode, seed=seed, mode=mode, valid=valid,
                         returncode=returncode, process_crashed=process_crashed,
                         shutdown_errors=shutdown_errors,
                         rmw_verified=rmw_verified,
                         command=cmd, metadata=metadata)
            with (output/'episodes.jsonl').open('a') as stream:
                stream.write(json.dumps(entry)+'\n')
                stream.flush()
                os.fsync(stream.fileno())
            print(json.dumps(entry), flush=True)
            consecutive_failures = 0 if valid else consecutive_failures+1
            episode += 1
            if consecutive_failures >= args.max_consecutive_failures:
                status['state']='failed_repeatedly'
                break
        if status['state']=='running':
            status['state']='stopped' if stop else 'complete'
    finally:
        if child is not None:
            stop_child(child)
        if status['state']=='running':
            status['state']='error'
        status.update(elapsed_s=time.monotonic()-started,
                      current_episode_pid=None, remaining_s=0.,
                      ended_utc=datetime.now(timezone.utc).isoformat())
        atomic_json(output/'status.json', status)
        if args.rmw:
            from summarize_dds_test import summarize
            summarize(output)
        lock.close()
        print(json.dumps(status), flush=True)
    return 0 if status['state']=='complete' else 1


if __name__ == '__main__':
    raise SystemExit(main())
