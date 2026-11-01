"""Summarize retained native timing samples; >2 ms calls are always recorded."""
import csv
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
RESULTS=HERE.parent/'results'


def native_rows(path):
    rows=[]
    for file in path.glob('*.csv'):
        for row in csv.reader(file.open()):
            if len(row)!=10:
                raise ValueError(f'Malformed timing row in {file}: {row}')
            layer,pid,tid,begin,end,cpu,result,gil,count,topic=row
            rows.append(dict(layer=layer,pid=int(pid),tid=int(tid),begin_ns=int(begin),end_ns=int(end),
                cpu_ms=int(cpu)/1e6,wall_ms=(int(end)-int(begin))/1e6,result=int(result),
                gil=int(gil),count=int(count),topic=topic))
    return rows


def main():
    output=dict(matrix=[],stack=[])
    for directory in sorted((RESULTS/'publish-matrix').iterdir()):
        if not directory.is_dir():continue
        meta=json.loads((directory/'meta.json').read_text()) if (directory/'meta.json').exists() else {}
        item=dict(name=directory.name,meta=meta)
        if (directory/'publisher.json').exists():
            item['publisher']=json.loads((directory/'publisher.json').read_text())
            item['readers']=[json.loads(p.read_text()) for p in sorted(directory.glob('reader-*.json'))]
        rows=native_rows(directory/'trace')
        item['native_slow_calls']=[v for v in rows if v['wall_ms']>=2]
        output['matrix'].append(item)
    for directory in sorted(RESULTS.glob('boundary-*-trace')):
        name=directory.name.removesuffix('-trace')
        rows=native_rows(directory)
        item=dict(name=name,trace_rows_total=len(rows),by_topic={})
        imu_file=RESULTS/(name+'.imu.json')
        imu=json.loads(imu_file.read_text()) if imu_file.exists() else None
        origin=imu.get('clock',{}).get('start_monotonic_s') if imu else None
        if origin is not None:
            outside=[v for v in rows if not origin<=v['begin_ns']/1e9<=v['end_ns']/1e9<=origin+32]
            rows=[v for v in rows if origin<=v['begin_ns']/1e9<=v['end_ns']/1e9<=origin+32]
            item['outside_observer_window_longest']=sorted(outside,key=lambda v:-v['wall_ms'])[:10]
        item['trace_rows']=len(rows)
        waits=[json.loads(line) for file in directory.glob('*.wait.jsonl') for line in file.read_text().splitlines()]
        if origin is not None:
            waits=[v for v in waits if origin<=v['begin_ns']/1e9<=v['end_ns']/1e9<=origin+32]
        item['wait_events']=waits
        for topic in sorted({v['topic'] for v in rows}):
            item['by_topic'][topic]={}
            for layer in ('rcl','rmw'):
                selected=[v for v in rows if v['topic']==topic and v['layer']==layer]
                if selected:
                    item['by_topic'][topic][layer]=dict(recorded=len(selected),
                        over_2ms=sum(v['wall_ms']>=2 for v in selected),
                        maximum=max(selected,key=lambda v:v['wall_ms']))
        item['longest_calls']=sorted(rows,key=lambda v:-v['wall_ms'])[:30]
        if imu:
            item['observer']=imu['summary']
            item['clock']=imu.get('clock')
            for topic in ('/imu/status','/sim/status','/safety/status','/policy/status','/wbc/status'):
                history=imu['status'].get(topic,[])
                item[topic]=history[-1] if history else None
            if item['clock']:
                origin=item['clock']['start_monotonic_s']
                for row in item['longest_calls']:
                    row['observer_begin_s']=row['begin_ns']/1e9-origin
        output['stack'].append(item)
    destination=RESULTS/'publish-boundary-summary.json'
    destination.write_text(json.dumps(output,indent=2)+'\n')
    for v in output['matrix']:
        p=v.get('publisher',{})
        print(v['name'],v['meta'].get('exit_codes'),p.get('count'),p.get('publish_max_ms'),
              [r['count'] for r in v.get('readers',[])])
    for v in output['stack']:
        print('\n',v['name'],'trace rows',v['trace_rows'])
        print('IMU',v.get('/imu/status'))
        print('Safety',v.get('/safety/status'))
        for row in v['longest_calls'][:6]:print(row)


if __name__=='__main__':main()
