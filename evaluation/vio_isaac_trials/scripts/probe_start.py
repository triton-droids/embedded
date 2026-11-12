"""Offline: stand at rest, then start walking (like the chase starting); time every controller.step."""
import gc
import time

from walking_controller.offline import load, run
from walking_controller.stepping import StepParams, SteppingController
from walking_controller.wbc import WBCGains

paths, robot, limits = load()
ctl = SteppingController(robot, limits.torque, StepParams(
    dt=0.005, step_length_m=0.0, gains=WBCGains(com_kp_z=100.0, com_weight=10.0, contact_kp=100.0)))
times = []
orig = ctl.step
gc_events = []
gc.callbacks.append(lambda phase, info: gc_events.append((time.monotonic(), phase, info.get('generation'))))


import cProfile, pstats, io
prof_slow = []


def timed(*a, **k):
    pr = cProfile.Profile()
    b = time.monotonic()
    pr.enable()
    r = orig(*a, **k)
    pr.disable()
    if time.monotonic() - b > 0.015:
        out = io.StringIO()
        pstats.Stats(pr, stream=out).sort_stats('cumulative').print_stats(14)
        prof_slow.append((a[0], out.getvalue()))
    times.append((a[0], time.monotonic() - b, ctl.phase if hasattr(ctl, 'phase') else '', getattr(ctl, 'holding', None)))
    return r


ctl.step = timed
state = {'stopped': False, 'resumed': False}


def on_step(t, sim, c):
    if t > 2.0 and not state['stopped']:
        c.request_stop()
        state['stopped'] = True
    if t > 6.0 and not state['resumed']:
        c.p.step_length_m, c.p.turn_rad = 0.05, 0.1
        c.resume()
        state['resumed'] = True
        print(f'resume at t={t:.3f}')
    return False


res = run(ctl, 10.0, paths=paths, robot=robot, limits=limits, on_step=on_step)
print('fault', res['fault'], 'fallen', res['fallen'])
slow = sorted(times, key=lambda x: -x[1])[:8]
for t, d, ph, hold in sorted(slow):
    print(f't={t:6.3f}  {d * 1e3:7.1f} ms  phase={ph} holding={hold}')
print('gen2 GCs:', sum(1 for e in gc_events if e[1] == 'start' and e[2] == 2))

for t, text in prof_slow[:2]:
    print("==== slow cycle at", t)
    print(text[:3500])
