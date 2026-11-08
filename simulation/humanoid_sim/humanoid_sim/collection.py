"""One bounded, simulation-only IMU episode. No ROS or hardware commands.

Physics hooks run under world.lock; samples are buffered and written only after
the episode stops. Outcome describes the current controller INCLUDING Safety.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import time

import mujoco
import numpy as np

from . import model_builder as mb
from .collection_rules import future_label, recovery_scenario


class EpisodeRecorder:
    def __init__(self, world, directory, seed=1, duration_s=15.0, profile='standard'):
        if not math.isfinite(duration_s) or not 1 <= duration_s <= 120:
            raise ValueError('collection duration must be in [1, 120] seconds')
        self.world = world
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.duration_s = float(duration_s)
        self.started_wall = time.monotonic()
        self.released_at = None
        self.fall_time = None
        self.contact_since = None
        self.done = False
        self.reason = None
        self.rows = []
        self.seed = int(seed)
        rng = np.random.default_rng(seed)
        # 1/3 no push; remaining trials span small to strong world-horizontal pushes.
        self.push_n = 0.0 if seed % 3 == 0 else float(rng.uniform(5, 100))
        angle = rng.uniform(-math.pi, math.pi)
        self.force = np.array([math.cos(angle), math.sin(angle), 0.]) * self.push_n
        self.push_delay = float(rng.uniform(.25, 2.0))
        self.push_duration = float(rng.uniform(.05, .25))
        self.push_applied_steps = 0
        self.profile = profile
        self.scenario = None
        if profile not in ('standard','recovery'):
            raise ValueError('Unknown collection profile')
        if profile == 'recovery':
            self.scenario = recovery_scenario(seed)
            self.force = np.asarray(self.scenario['force_world_n'])
            self.push_n = float(np.linalg.norm(self.force))
            self.push_delay = self.scenario['delay_s']
            self.push_duration = self.scenario['duration_s']
            world.model.geom('floor').friction[0] = self.scenario['friction']
        self.base = world.model.body(mb.BASE_BODY).id
        self.floor = world.model.geom('floor').id
        self.feet = {world.model.geom(side + '_foot_collision_box').id for side in ('left', 'right')}
        self._wrench = np.zeros(6)

    def before_step(self):
        w = self.world
        t = float(w.data.time)
        held = bool(w.data.eq_active[w._weld])
        if not held and self.released_at is None:
            self.released_at = t
        active = (not self.done and not held and self.released_at is not None
                  and self.push_delay <= t - self.released_at < self.push_delay + self.push_duration)
        w.data.xfrc_applied[self.base, :3] = self.force if active else 0.
        if active and self.push_n:
            self.push_applied_steps += 1

    def after_step(self):
        if self.done or self.released_at is None:
            return
        w, contact_found = self.world, False
        for i in range(w.data.ncon):
            contact = w.data.contact[i]
            pair = {int(contact.geom1), int(contact.geom2)}
            if self.floor in pair and not (pair & self.feet) and contact.dist <= 0:
                mujoco.mj_contactForce(w.model, w.data, i, self._wrench)
                if self._wrench[0] > 1.0:
                    contact_found = True
                    break
        t = float(w.data.time)
        if contact_found:
            if self.contact_since is None:
                self.contact_since = t
            if self.fall_time is None and t - self.contact_since >= .02 - 1e-9:
                self.fall_time = t  # confirmed non-foot contact, not an angle threshold
        else:
            self.contact_since = None
        if self.fall_time is not None and t >= self.fall_time + .35:
            self.finish('fall')
        elif t - self.released_at >= self.duration_s:
            self.finish('completed')

    def record_imu(self, sample, gyro, accel, stamp):
        """Called with world.lock held, using exactly the published sensor values."""
        if self.done:
            return
        w = self.world
        q = sample.quat_wxyz
        qw, qx, qy, qz = q
        roll = math.atan2(2*(qw*qx + qy*qz), 1-2*(qx*qx + qy*qy))
        pitch = math.asin(float(np.clip(2*(qw*qy-qz*qx), -1, 1)))
        pos, _, _, _ = w.base_state()
        joints = w.joint_snapshot()
        self.rows.append(dict(sim_time_s=float(w.data.time), wall_monotonic_s=time.monotonic(),
            ros_stamp_ns=int(stamp.sec)*10**9+int(stamp.nanosec),
            features=[roll, pitch, *map(float, gyro), *map(float, accel)],
            quat_wxyz=q.tolist(), base_height_m=float(pos[2]),
            joint_position=joints.position.tolist(), joint_velocity=joints.velocity.tolist(),
            base_held=bool(w.data.eq_active[w._weld]), estop=bool(w.motors.estop_active),
            control_fault=w.control_fault, physics_overruns=w.overruns))

    def poll(self):
        with self.world.lock:
            if self.world.control_fault is not None:
                self.finish('control_fault')
            elif self.released_at is None and time.monotonic()-self.started_wall > 45:
                self.finish('startup_timeout')
            elif time.monotonic()-self.started_wall > self.duration_s*4+60:
                self.finish('wall_timeout')
        return self.done

    def finish(self, reason):
        if not self.done:
            self.reason, self.done = reason, True
            self.world.data.xfrc_applied[self.base, :3] = 0.

    def save(self):
        """Atomic files: an interrupted/missing future is always labelled unknown."""
        metadata = dict(schema_version=1, seed=self.seed, reason=self.reason,
            released_at_sim_s=self.released_at, fall_time_sim_s=self.fall_time,
            end_sim_s=float(self.world.data.time), rows=len(self.rows),
            duration_s=self.duration_s, push_force_world_n=self.force.tolist(),
            push_delay_s=self.push_delay, push_duration_s=self.push_duration,
            push_applied_steps=self.push_applied_steps,
            feature_names=['roll_rad','pitch_rad','gyro_x','gyro_y','gyro_z','accel_x','accel_y','accel_z'],
            joint_names=self.world.joint_names, mujoco_version=mujoco.__version__,
            physics_timestep_s=float(self.world.model.opt.timestep),
            outcome='under existing controller and active safety supervisor',
            fall_definition='non-foot floor contact >1 N sustained 20 ms',
            horizon_s=.3, horizons_s=[.3,.6], history_frames=20,
            label_time_tolerance_s=1e-9, profile=self.profile, scenario=self.scenario)
        metadata['rmw_implementation'] = getattr(self.world,'rmw_implementation',None)
        # Input: 20 consecutive 100-Hz observations. Gaps, held/faulted states,
        # already-fallen states and incomplete future horizons are excluded.
        times = np.array([r['sim_time_s'] for r in self.rows])
        for i, row in enumerate(self.rows):
            row['label'] = None
            row['label_600ms'] = None
            if i < 19 or self.released_at is None:
                continue
            window = self.rows[i-19:i+1]
            gaps = np.diff(times[i-19:i+1])
            if (any(r['base_held'] or r['control_fault'] for r in window)
                    or np.any(gaps < .005) or np.any(gaps > .02)):
                continue
            t = row['sim_time_s']
            if self.fall_time is not None and t >= self.fall_time:
                continue
            for horizon,key in ((.3,'label'),(.6,'label_600ms')):
                value = future_label(t,self.fall_time,float(times[-1]),horizon)
                future = [r for r in self.rows[i:] if r['sim_time_s'] <= t+horizon+1e-9]
                if not any(r['control_fault'] or r['base_held'] for r in future):
                    row[key] = value
        metadata['label_counts'] = {str(k): sum(r['label']==k for r in self.rows) for k in (0, 1, None)}
        metadata['label_counts_600ms'] = {str(k): sum(r['label_600ms']==k for r in self.rows) for k in (0,1,None)}
        if self.rows and self.released_at is not None:
            released = [r for r in self.rows if not r['base_held']]
            push_end = self.released_at+self.push_delay+self.push_duration
            tail = [r for r in released if r['sim_time_s'] >= times[-1]-1.]
            post = [r for r in released if r['sim_time_s'] >= push_end]
            metadata['recovered_after_push'] = bool(
                self.push_applied_steps and self.fall_time is None and tail and post
                and times[-1] >= push_end+3 and not any(r['estop'] for r in post)
                and max(max(abs(v) for v in r['features'][:2]) for r in tail) < math.radians(25))
            metadata['peak_post_push_tilt_deg'] = (math.degrees(max(max(abs(v) for v in r['features'][:2]) for r in post)) if post else None)
        for name, content in [('samples.jsonl', ''.join(json.dumps(r, allow_nan=False)+'\n' for r in self.rows)),
                              ('metadata.json', json.dumps(metadata, indent=2, allow_nan=False)+'\n')]:
            tmp = self.directory / (name+'.tmp')
            with tmp.open('w') as stream:
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
            tmp.replace(self.directory / name)
        return metadata
