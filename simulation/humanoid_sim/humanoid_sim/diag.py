"""Timing diagnostics for the simulator process (off unless HUMANOID_SIM_DIAG=1).

Collects per-second maxima of named durations and prints one line per second to
stderr (``DIAG {...}`` JSON), for finding where the process stalls:

  cmd_age_ms     motor command age (header stamp to admission in the callback)
  cmd_lock_ms    callback wait for the world lock
  step_ms        one physics step;  step_late_ms  start of a step after its schedule
  cam_scene_ms   camera update_scene (holds the world lock); cam_render_ms  GPU render
  gil_ms         oversleep of a 2 ms sleep in a probe thread: time other threads held the GIL

record() is cheap and lock-free enough for the physics thread (a dict update).
"""
from __future__ import annotations

import json
import os
import sys
import threading
import time

ENABLED = os.environ.get('HUMANOID_SIM_DIAG', '') not in ('', '0')
_max: dict = {}
_count: dict = {}


def record(name: str, value_ms: float) -> None:
    if not ENABLED:
        return
    if value_ms > _max.get(name, 0.0):
        _max[name] = value_ms
    _count[name] = _count.get(name, 0) + 1


def _gil_probe(stop: threading.Event) -> None:
    while not stop.is_set():
        begin = time.monotonic()
        time.sleep(0.002)
        record('gil_ms', (time.monotonic() - begin) * 1e3 - 2.0)


def _report(stop: threading.Event) -> None:
    while not stop.wait(1.0):
        snapshot = {k: round(v, 1) for k, v in _max.items()}
        snapshot['n_cmd'] = _count.get('cmd_age_ms', 0)
        snapshot['t'] = round(time.time(), 1)
        _max.clear()
        _count.clear()
        print('DIAG ' + json.dumps(snapshot, sort_keys=True), file=sys.stderr, flush=True)


_stop = threading.Event()
if ENABLED:
    threading.Thread(target=_gil_probe, args=(_stop,), name='diag_gil', daemon=True).start()
    threading.Thread(target=_report, args=(_stop,), name='diag_report', daemon=True).start()
