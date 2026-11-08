"""Serial real-delivery Python/C++ IMU transport comparisons; all trials retained."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
BUILD=HERE.parent/'generated/publish_probe'


def trial(name,language,readers,duration,domain,preload,output_root):
    output=output_root/name
    output.mkdir(parents=True,exist_ok=False)
    env=dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='0',
             RMW_IMPLEMENTATION='rmw_fastrtps_cpp',RMW_FASTRTPS_PUBLICATION_MODE='ASYNCHRONOUS',
             HUMANOID_THREAD_SCHEDULER='',HUMANOID_PRIORITY_CONFIG='')
    env.pop('LD_PRELOAD',None)
    if preload:
        trace=output/'trace';trace.mkdir()
        env['LD_PRELOAD']=str(BUILD/'libpublish_boundary.so')
        env['HUMANOID_PUBLISH_TRACE_DIR']=str(trace)
    children=[];logs=[]
    try:
        for i,(reliability,stall) in enumerate(readers):
            log=(output/f'reader-{i}.log').open('w');logs.append(log)
            child=subprocess.Popen([str(BUILD/'imu_bench'),'--role','subscriber',
                '--duration',str(duration+3),'--stall-ms',str(stall),'--reliability',reliability,
                '--output',str(output/f'reader-{i}.json')],env=env,stdout=log,stderr=subprocess.STDOUT)
            children.append(child)
        time.sleep(.2)
        command=([sys.executable,str(HERE/'python_publisher.py')] if language=='python'
                 else [str(BUILD/'imu_bench')])
        log=(output/'publisher.log').open('w');logs.append(log)
        publisher=subprocess.Popen(command+['--duration',str(duration),'--output',str(output/'publisher.json')],
                                   env=env,stdout=log,stderr=subprocess.STDOUT)
        children.append(publisher)
        publisher.wait(timeout=duration+12)
        for child in children:
            child.wait(timeout=8)
        codes=[p.returncode for p in children]
        meta=dict(name=name,language=language,readers=readers,duration_s=duration,domain=domain,
                  native_timing=preload,exit_codes=codes)
        (output/'meta.json').write_text(json.dumps(meta,indent=2)+'\n')
        if any(codes):
            raise RuntimeError(f'Trial failed, records retained: {output}, codes={codes}')
        result=json.loads((output/'publisher.json').read_text())
        received=[json.loads(p.read_text())['count'] for p in sorted(output.glob('reader-*.json'))]
        print(name,'publisher',result,'received',received,flush=True)
        if readers and not all(received):
            raise RuntimeError('No real delivery; instrumentation trial invalid')
    finally:
        for child in children:
            if child.poll() is None:
                child.terminate()
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:child.kill();child.wait()
        for log in logs:log.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--duration',type=float,default=8)
    p.add_argument('--smoke',action='store_true')
    p.add_argument('--smoke-name',default='smoke-cpp-reader-v2')
    p.add_argument('--output-root',type=Path,default=HERE.parent/'results/publish-matrix')
    args=p.parse_args()
    if args.smoke:
        trial(args.smoke_name,'python',[('reliable',0)],1,224,True,args.output_root)
        return
    for language in ('python','cpp'):
        trial(f'{language}-none-untraced',language,[],args.duration,225,False,args.output_root)
    configurations=[('none',[]),('mixed',[('reliable',0),('best_effort',0),('best_effort',0)]),
                    ('slow-reliable',[('reliable',100)])]
    for repeat in (1,2):
        cases=configurations if repeat==1 else list(reversed(configurations))
        languages=('python','cpp') if repeat==1 else ('cpp','python')
        for name,readers in cases:
            for language in languages:
                trial(f'{language}-{name}-{repeat}',language,readers,args.duration,225,True,args.output_root)


if __name__=='__main__':main()
