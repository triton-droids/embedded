"""Bounded held-world component benchmark; never enables motors or launches policy.

100 percent CPU means one logical CPU. Startup is excluded. Camera uses the real
SimCameraNode, GUI uses the real ViewerProcess. Unpaced/overload are synthetic
scheduler controls confined to this script, not production changes.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import threading
import time

import numpy as np
import psutil
import yaml

from wbc_payload import camera_model, ROOT
from humanoid_sim.motor_model import MotorBank, limits_from_registry
from humanoid_sim.world import SimWorld, HeadServoConfig
from humanoid_sim.viewer import ViewerProcess


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case', choices=['paced', 'gui', 'camera', 'both', 'unpaced', 'overload'], required=True)
    p.add_argument('--duration', type=float, default=8)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    registry = yaml.safe_load((ROOT/'humanoid_control/motor_control_hybrid/config/motors.yaml').read_text())
    limits = limits_from_registry(registry['motor_control_node']['ros__parameters'])
    w = SimWorld(camera_model(), MotorBank(limits), head=HeadServoConfig(), settle_s=0, base_held=True)
    records = []
    original = w.step

    def step(now):
        wall, cpu = time.monotonic(), time.thread_time()
        original(now)
        if a.case == 'overload':
            # Six milliseconds of actual Python CPU work, not a six-ms sleep.
            until = time.thread_time() + .006
            while time.thread_time() < until:
                pass
        records.append((wall, time.monotonic()-wall, time.thread_time()-cpu))

    w.step = step
    if a.case == 'unpaced':
        def run():
            while not w._stop.is_set():
                w.step(time.monotonic())
        w._run = run
    viewer = camera = None
    context = None
    frames = [0]
    try:
        w.start()
        if a.case in ('gui', 'both'):
            viewer = ViewerProcess(w)
        if a.case in ('camera', 'both'):
            import rclpy
            from humanoid_sim.camera_node import SimCameraNode
            rclpy.init(args=['--ros-args', '-p', 'rate_hz:=6.0', '-p', 'enable_depth:=true'])
            context = rclpy
            # Count actual completed RGB frames without changing renderer work.
            publish = SimCameraNode._publish_color
            def counted(self, pixels, stamp):
                publish(self, pixels, stamp)
                frames[0] += 1
            SimCameraNode._publish_color = counted
            camera = SimCameraNode(w)
        for _ in range(10):
            if viewer:
                viewer.update()
            time.sleep(.2)
        gui_windows = []
        if viewer:
            tree = subprocess.run(['xwininfo','-root','-tree'],capture_output=True,text=True,check=True).stdout
            gui_windows = [line.strip() for line in tree.splitlines() if 'MuJoCo' in line]
        proc = psutil.Process()
        processes = [proc] + proc.children(recursive=True)
        def cpu_snapshot():
            return {v.pid: sum(v.cpu_times()[:2]) for v in processes if v.is_running()}
        c0 = cpu_snapshot()
        threads0 = {t.id: t.user_time+t.system_time for t in proc.threads()}
        sys0 = psutil.cpu_times()
        start = time.monotonic()
        s0, o0, f0 = w.steps, w.overruns, frames[0]
        while time.monotonic()-start < a.duration:
            if viewer:
                if not viewer.is_running():
                    raise RuntimeError('GUI closed during measurement')
                viewer.update()
            time.sleep(.2)
        elapsed = time.monotonic()-start
        end = time.monotonic()
        c1 = cpu_snapshot()
        threads1 = proc.threads()
        sys1 = psutil.cpu_times()
        measured = np.array([v for v in records if start <= v[0] < end])
        gaps = np.diff(measured[:, 0])
        result = dict(case=a.case, elapsed_s=elapsed, mujoco_version=__import__('mujoco').__version__,
            physics_hz=(w.steps-s0)/elapsed, realtime_factor=(w.steps-s0)*w.model.opt.timestep/elapsed,
            overruns=w.overruns-o0, max_step_gap_ms=float(max(gaps)*1000),
            step_wall_p99_ms=float(np.percentile(measured[:,1],99)*1000),
            step_cpu_mean_ms=float(np.mean(measured[:,2])*1000),
            camera_hz=(frames[0]-f0)/elapsed,
            processes=[dict(pid=v.pid, name=v.name(), role='main' if v.pid==os.getpid() else ('gui' if viewer and v.pid==viewer._process.pid else 'helper'),
                            cpu_pct=100*(c1.get(v.pid,c0[v.pid])-c0[v.pid])/elapsed) for v in processes if v.pid in c0],
            threads=[dict(tid=t.id,cpu_pct=100*(t.user_time+t.system_time-threads0.get(t.id,0))/elapsed) for t in threads1],
            system_busy_pct=100*(1-(sys1.idle-sys0.idle)/(sum(sys1)-sum(sys0))),
            physics_thread_tid=w._thread.native_id, control_fault=w.control_fault, gui_windows=gui_windows,
            scope='Held full camera/CAD/football model, no policy/WBC/safety/YOLO; camera RGB-D publishers have no image subscribers')
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)
    finally:
        if camera:
            camera.destroy_node()
        if context:
            context.shutdown()
        if viewer:
            viewer.close()
        w.stop()


if __name__ == '__main__':
    main()
