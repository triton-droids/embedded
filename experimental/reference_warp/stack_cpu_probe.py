"""Bounded normal-launch CPU/IMU probe. Held by default; release must be explicit."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import threading
import urllib.request

import psutil

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=32)
    parser.add_argument('--poll-web', action='store_true', help='Read-only /api/status polling at 5 Hz')
    parser.add_argument('--pause-source-at', type=float,
                        help='Held-only fault test: SIGSTOP simulator for one second at this time')
    parser.add_argument('--gui', choices=['true','false'], required=True)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--name', required=True)
    parser.add_argument('--start-chase', choices=['true','false'], default='false')
    parser.add_argument('--render-camera', choices=['true','false'], default='true')
    parser.add_argument('--publication-mode', choices=['ASYNCHRONOUS','SYNCHRONOUS'], default='ASYNCHRONOUS')
    parser.add_argument('--preload',type=Path,help='Optional real-call timing shim; never a synthetic stub')
    parser.add_argument('--trace-dir',type=Path)
    args = parser.parse_args()
    if args.pause_source_at is not None and args.start_chase != 'false':
        parser.error('Source pause fault injection requires the held, disabled stack')
    base = HERE/'results'/args.name
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='0', MUJOCO_GL='egl')
    observer_env=dict(env)
    if args.preload:
        if args.trace_dir is None:
            parser.error('--preload requires --trace-dir')
        args.trace_dir.mkdir(parents=True,exist_ok=False)
        env['LD_PRELOAD']=str(args.preload.resolve())
        env['HUMANOID_PUBLISH_TRACE_DIR']=str(args.trace_dir.resolve())
    samples, previous, library_maps = [], {}, {}
    paused_source, source_pause = None, None
    web_requests = dict(ok=0, failed=0)
    web_stop = threading.Event()
    def poll_web():
        while not web_stop.is_set():
            try:
                with urllib.request.urlopen(f'http://127.0.0.1:{8000+args.domain}/api/status', timeout=.5) as response:
                    json.loads(response.read())
                web_requests['ok'] += 1
            except (OSError, ValueError):
                web_requests['failed'] += 1
            web_stop.wait(.2)
    if args.poll_web:
        threading.Thread(target=poll_web, daemon=True).start()
    launched = time.monotonic()
    with base.with_suffix('.log').open('w') as log:
        launch = subprocess.Popen(['ros2','launch',str(HERE/'chase.launch.py'),
            f'start_chase:={args.start_chase}',f'use_viewer:={args.gui}',f'ros_domain_id:={args.domain}',
            f'render_camera:={args.render_camera}',
            f'dds_publication_mode:={args.publication_mode}',
            f'web_port:={8000+args.domain}','localhost_only:=0'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        observer = subprocess.Popen(['python',str(HERE/'imu_timing_probe.py'),'--duration',str(args.duration),
            '--output',str(base.with_suffix('.imu.json'))],env=observer_env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            parent = psutil.Process(launch.pid)
            last = time.monotonic()
            while time.monotonic()-launched < args.duration:
                time.sleep(1)
                now = time.monotonic()
                rows = []
                for proc in [parent]+parent.children(recursive=True):
                    try:
                        cpu = sum(proc.cpu_times()[:2])
                        pct = 100*(cpu-previous.get(proc.pid,cpu))/(now-last)
                        rows.append(dict(pid=proc.pid,ppid=proc.ppid(),cpu_pct=pct,cmd=proc.cmdline(),
                                         scheduler=os.sched_getscheduler(proc.pid),affinity=proc.cpu_affinity(),threads=proc.num_threads()))
                        previous[proc.pid] = cpu
                        if proc.pid not in library_maps:
                            try:
                                loaded = [line for line in Path(f'/proc/{proc.pid}/maps').read_text().splitlines()
                                          if 'r-xp' in line and any(name in line for name in (
                                              'libfastrtps.so','librmw_fastrtps','libddsc.so','librmw_cyclonedds'))]
                                if loaded:
                                    library_maps[proc.pid] = dict(cmd=proc.cmdline(), maps=loaded)
                            except (OSError, PermissionError):
                                pass
                    except (psutil.NoSuchProcess,ProcessLookupError):
                        pass
                samples.append(dict(t=now-launched,processes=rows))
                if args.pause_source_at is not None and source_pause is None and now-launched >= args.pause_source_at:
                    for proc in parent.children(recursive=True):
                        if any(Path(part).name == 'mujoco_sim' for part in proc.cmdline()):
                            os.kill(proc.pid, signal.SIGSTOP)
                            paused_source = proc.pid
                            source_pause = dict(pid=proc.pid, stopped_at_s=now-launched)
                            break
                elif paused_source is not None and now-launched >= source_pause['stopped_at_s']+1:
                    os.kill(paused_source, signal.SIGCONT)
                    source_pause['resumed_at_s'] = now-launched
                    paused_source = None
                last = now
                if launch.poll() is not None:
                    break
            observer.wait(timeout=8)
        finally:
            web_stop.set()
            if paused_source is not None:
                try:
                    os.kill(paused_source, signal.SIGCONT)
                except ProcessLookupError:
                    pass
            for child in (observer,launch):
                if child.poll() is None:
                    os.killpg(child.pid,signal.SIGINT)
                    try:
                        child.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid,signal.SIGKILL)
                        child.wait()
    base.with_suffix('.cpu.json').write_text(json.dumps(dict(duration=args.duration,web_requests=web_requests,source_pause=source_pause,gui=args.gui,start_chase=args.start_chase,render_camera=args.render_camera,publication_mode=args.publication_mode,samples=samples,
        library_maps=library_maps, dds_environment={key:env.get(key) for key in (
            'RMW_IMPLEMENTATION','LD_LIBRARY_PATH','AMENT_PREFIX_PATH',
            'FASTRTPS_DEFAULT_PROFILES_FILE','RMW_FASTRTPS_USE_QOS_FROM_XML',
            'RMW_FASTRTPS_PUBLICATION_MODE')}),indent=2)+'\n')
    stable = [sum(p['cpu_pct'] for p in s['processes']) for s in samples if s['t']>=12]
    print(args.name, 'CPU mean after 12s',sum(stable)/len(stable) if stable else None,flush=True)


if __name__ == '__main__':
    main()
