#!/usr/bin/env python3
"""Bounded wire-vector verification on kernel vcan only; never physical CAN."""
import argparse, json, subprocess, time, math
from pathlib import Path
import can
from robstride_dynamics import Motor
from motor_control_hybrid.can_transport import NonblockingRobstrideBus
from motor_control_hybrid.python_can_node import PythonCanNode

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--channel',default='can1')
parser.add_argument('--output',default='/tmp/can-wire-audit.jsonl')
args=parser.parse_args()
channel=args.channel
info=json.loads(subprocess.check_output(['ip','-details','-json','link','show',channel]))
if info[0].get('linkinfo',{}).get('info_kind')!='vcan':
    raise SystemExit('Refusing physical CAN: this verifier is vcan-only')
models={'rs02':(241,'rs-02',44.,17.,500.,5.),'rs03':(242,'rs-03',20.,60.,5000.,100.),'rs04':(243,'rs-04',15.,120.,5000.,100.)}
transport=NonblockingRobstrideBus(channel,{name:Motor(item[0],item[1]) for name,item in models.items()}, {})
transport.host_id=254
transport.connect()
node=PythonCanNode.__new__(PythonCanNode)
records=[]
try:
    with can.Bus(interface='socketcan',channel=channel,can_filters=[{'can_id':i,'can_mask':255,'extended':True} for i in (241,242,243)]) as peer:
        def expect(ident,payload):
            frame=peer.recv(timeout=1)
            assert frame is not None,'No transmitted frame observed'
            assert frame.is_extended_id and not frame.is_fd and frame.dlc==8
            assert frame.arbitration_id==ident and bytes(frame.data)==bytes.fromhex(payload),(frame,hex(ident),payload)
            records.append(dict(direction='tx',can_id=f'{frame.arbitration_id:08X}',data=bytes(frame.data).hex(),expected_id=f'{ident:08X}',expected_data=payload))
        for name,(device,model,vmax,tmax,kpmax,kdmax) in models.items():
            node._set_mode_raw(transport,name,0);expect(0x1200FE00|device,'0570000000000000')
            transport.enable(name);expect(0x0300FE00|device,'0000000000000000')
            transport.write_operation_frame(name,4*math.pi,kpmax,kdmax,vmax,tmax);expect(0x01FFFF00|device,'ffffffffffffffff')
            transport.disable(name);expect(0x0400FE00|device,'0000000000000000')
            feedback=can.Message(arbitration_id=0x028000FE|(device<<8),is_extended_id=True,data=bytes.fromhex('ffffffffffff00fa'))
            peer.send(feedback,timeout=.1)
            end=time.monotonic()+1;sample=None
            while time.monotonic()<end:
                frame=transport.channel_handler.recv(timeout=0.)
                if frame is not None:
                    sample=transport.decode_feedback(frame)
                    if sample:break
                time.sleep(.001)
            assert sample is not None and sample.motor_name==name
            assert sample.position==4*math.pi and sample.velocity==vmax and sample.torque==tmax
            assert sample.temperature==25 and sample.mode==2 and sample.fault_bits==0
            records.append(dict(direction='rx',model=model,can_id=f'{feedback.arbitration_id:08X}',data=bytes(feedback.data).hex(),verified=True))
finally:
    transport.disconnect(disable_torque=False)
Path(args.output).write_text(''.join(json.dumps(record)+'\n' for record in records))
print(f'PASS: {len(records)} SocketCAN vcan TX/RX records matched independent RS02/03/04 vectors; {args.output}')
