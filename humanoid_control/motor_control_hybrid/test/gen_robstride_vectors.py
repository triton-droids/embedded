#!/usr/bin/env python3
"""Generate test/data/robstride_vectors.json from the Python reference modules.

The C++ codec (include/motor_control_hybrid/robstride_codec.hpp) must reproduce
these frames bit for bit and these conversions to 1e-12. Regenerate after any
change to can_transport.py, motor_configuration.py or ankle_mapping.py:

  PYTHONPATH=humanoid_control/motor_control_hybrid python3 \
    humanoid_control/motor_control_hybrid/test/gen_robstride_vectors.py
"""
import json
import math
import random
import struct
from pathlib import Path
from unittest.mock import patch

import can
import yaml
from robstride_dynamics import Motor

from motor_control_hybrid import motor_configuration
from motor_control_hybrid.can_transport import NonblockingRobstrideBus, PROTOCOL_PROFILES
from motor_control_hybrid.python_can_node import PythonCanNode

HERE = Path(__file__).resolve().parent
rng = random.Random(20261118)
HOST = 255


def bus_for(models):
    bus = NonblockingRobstrideBus.__new__(NonblockingRobstrideBus)
    bus.motors = {f'motor_{i}': Motor(id=i, model=m) for i, m in models.items()}
    bus.calibration = {}
    bus.channel_handler = None
    bus.host_id = HOST
    bus._motor_by_id = {motor.id: name for name, motor in bus.motors.items()}
    sent = []
    bus.transmit = lambda kind, extra, device, data=bytes(8): sent.append(
        ((kind << 24) | (extra << 8) | device, bytes(data).hex()))
    return bus, sent


def value(scale):
    """Mostly in range, sometimes beyond it (clamping) or exactly at the edge."""
    r = rng.random()
    if r < 0.05:
        return scale * rng.choice((-1, 1))
    if r < 0.15:
        return scale * rng.uniform(1.0, 3.0) * rng.choice((-1, 1))
    return rng.uniform(-scale, scale)


models = {1: 'rs-02', 2: 'rs-03', 3: 'rs-04'}
bus, sent = bus_for(models)
operation = []
for _ in range(600):
    device = rng.choice(list(models))
    pmax, vmax, tmax, kpmax, kdmax = PROTOCOL_PROFILES[models[device]]
    args = dict(position=value(pmax), kp=abs(value(kpmax)), kd=abs(value(kdmax)),
                velocity=value(vmax), torque=value(tmax))
    sent.clear()
    bus.write_operation_frame(f'motor_{device}', **args)
    operation.append(dict(model=models[device], device=device, **args,
                          id=sent[0][0], data=sent[0][1]))

lifecycle = []
for device in models:
    for kind in ('enable', 'disable'):
        sent.clear()
        getattr(bus, kind)(f'motor_{device}')
        lifecycle.append(dict(kind=kind, device=device, id=sent[0][0], data=sent[0][1]))

params = []
node = PythonCanNode.__new__(PythonCanNode)
for device in models:
    for mode in (0, 1, 2, -1):
        sent.clear()
        node._set_mode_raw(bus, f'motor_{device}', mode)
        params.append(dict(kind='int8', param=0x7005, value=mode, device=device,
                           id=sent[0][0], data=sent[0][1]))
    from robstride_dynamics import ParameterType
    for ptype, pid in ((ParameterType.CURRENT_LIMIT, 0x7018),
                       (ParameterType.VEL_ACCELERATION_TARGET, 0x7022),
                       (ParameterType.VELOCITY_TARGET, 0x700A)):
        for v in (0.0, 5.0, -2.75, 10.0, rng.uniform(-50, 50), 1e-7):
            sent.clear()
            node._write_parameter(bus, 'j', f'motor_{device}', ptype, v)
            params.append(dict(kind='float', param=pid, value=v, device=device,
                               id=sent[0][0], data=sent[0][1]))

decode = []
for _ in range(400):
    device = rng.choice(list(models) + [9])  # 9 = unknown device
    kind = rng.choice([2] * 8 + [21, 17, 3])
    host = HOST if rng.random() < 0.9 else 254
    extra = (rng.randrange(4) << 14) | (rng.choice([0] * 6 + [1, 0x20, 63]) << 8) | device
    data = bytes(rng.randrange(256) for _ in range(8))
    if rng.random() < 0.03:
        data = b'\x00\xc4\x56' + data[3:]
    frame_id = (kind << 24) | (extra << 8) | host
    msg = can.Message(arbitration_id=frame_id, is_extended_id=True, data=data, timestamp=1.0)
    try:
        sample = bus.decode_feedback(msg)
        result = 'ignored' if sample is None else 'status'
    except RuntimeError:
        result, sample = 'fault', None
    reply = bus.decode_parameter_reply(msg) if result == 'ignored' else None
    if reply is not None:
        result = 'parameter'
    row = dict(id=frame_id, data=data.hex(), result=result)
    if reply is not None:
        row.update(motor_id=int(reply.motor_name.split('_')[1]), index=reply.index,
                   value=int.from_bytes(reply.value, 'little'))
    if sample is not None:
        row.update(motor_id=int(sample.motor_name.split('_')[1]), position=sample.position,
                   velocity=sample.velocity, torque=sample.torque,
                   temperature=sample.temperature, mode=sample.mode,
                   fault_bits=sample.fault_bits)
    decode.append(row)

registry = yaml.safe_load((HERE.parent / 'config' / 'motors.yaml').read_text())
reg_params = registry.get('motor_control_node', {}).get('ros__parameters', registry)
settings_cases = []
for joint, cfg in reg_params['motors'].items():
    cfg = dict(cfg)
    if not cfg.get('ankle_mapping') and rng.random() < 0.5:
        cfg.update(direction=-1, encoder_offset_rad=0.3, max_torque_nm=12.0)
    settings = motor_configuration.MotorSettings.from_config(cfg, reg_params)
    steps = []
    clock = [100.0]
    with patch('time.monotonic', side_effect=lambda: clock[0]):
        for _ in range(40):
            clock[0] += rng.choice((0.02, 0.02, 0.019, 0.00005))
            fb_in = (rng.uniform(-0.9, 0.9), rng.uniform(-3, 3), rng.uniform(-10, 10))
            fb_out = settings.feedback(*fb_in)
            cmd_in = dict(position=rng.uniform(-1.2, 1.2), velocity=rng.uniform(-8, 8),
                          acceleration=rng.uniform(0, 5), torque=rng.uniform(-20, 20),
                          kp=rng.uniform(0, 50), kd=rng.uniform(0, 2))
            cmd_out = settings.command(cmd_in)
            steps.append(dict(now=clock[0], fb_in=fb_in, fb_out=list(fb_out),
                              cmd_in=cmd_in, cmd_out={k: cmd_out[k] for k in cmd_in}))
    settings_cases.append(dict(joint=joint, config=cfg, defaults={
        k: reg_params[k] for k in ('KP', 'KD', 'MAX_VEL_RAD_S') if k in reg_params}, steps=steps))

out = dict(generator='test/gen_robstride_vectors.py', host_id=HOST, operation=operation,
           lifecycle=lifecycle, parameters=params, decode=decode, settings=settings_cases)
(HERE / 'data').mkdir(exist_ok=True)
(HERE / 'data' / 'robstride_vectors.json').write_text(json.dumps(out, separators=(',', ':')) + '\n')
print({k: len(v) for k, v in out.items() if isinstance(v, list)})
