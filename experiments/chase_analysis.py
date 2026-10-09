"""Summarize a recorded matched chase batch without connecting to AirSim."""
import argparse
import json
from pathlib import Path
import sys
if __package__ in (None,''):sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from navigation.paths import PROJECT_ROOT,project_path
from experiments.recorder import atomic_json


def path_error(points,path):
    starts=path[:-1];segments=np.diff(path,axis=0);lengths=np.maximum((segments*segments).sum(axis=1),1e-12)
    delta=points[:,None,:]-starts[None,:,:]
    fraction=np.clip((delta*segments).sum(axis=2)/lengths,0.,1.)
    return np.linalg.norm(delta-fraction[:,:,None]*segments,axis=2).min(axis=1)


def analyze(batch,output):
    batch=project_path(batch);data=json.loads(batch.read_text(encoding='utf-8'));rows=[]
    for result in data['results']:
        row=dict(result)
        row['strategy']=row['chase_options']['strategy']
        run=Path(row['run_dir'])
        if not run.is_dir():
            parts=run.parts
            if 'data' in parts:run=PROJECT_ROOT.joinpath(*parts[parts.index('data'):])
        row['run_relative']=run.relative_to(PROJECT_ROOT).as_posix() if run.is_relative_to(PROJECT_ROOT) else str(run)
        raw=run/'chase_telemetry.json';metadata=run/'run_meta.json'
        if raw.is_file() and metadata.is_file():
            samples=json.loads(raw.read_text(encoding='utf-8'))['samples']
            meta=json.loads(metadata.read_text(encoding='utf-8'))
            if samples:
                target=np.array([s['target'] for s in samples]);speeds=np.array([s['target_speed'] for s in samples])
                errors=path_error(target,np.array(meta['scenario']['target_path']))
                row.update(target_path_error_mean=float(errors.mean()),target_path_error_max=float(errors.max()),
                           target_height_range=float(np.ptp(target[:,2])),target_speed_min=float(speeds.min()),
                           target_speed_max=float(speeds.max()),capture_wall_seconds=samples[-1]['wall_seconds'] if row['captured'] else None)
            telemetry=run/'telemetry.json'
            if telemetry.is_file():
                records=json.loads(telemetry.read_text(encoding='utf-8'))['records']
                ages=[r['map_age'] for r in records if r.get('map_age') is not None]
                row.update(braking_fraction=float(np.mean([r['braking'] for r in records])) if records else None,
                           stale_map_fraction=float(np.mean(np.asarray(ages)>=meta['config']['map_stale_timeout'])) if ages else None)
        rows.append(row)
    groups=[]
    for strategy in sorted({r['strategy'] for r in rows}):
        runs=[r for r in rows if r['strategy']==strategy and r.get('benchmark_valid',False)]
        captured=[r for r in runs if r['captured']]
        groups.append(dict(strategy=strategy,valid_runs=len(runs),captured=len(captured),
                           mean_capture_sim_seconds=float(np.mean([r['capture_sim_seconds'] for r in captured])) if captured else None,
                           pursuer_collisions=sum(bool(r.get('pursuer_collision')) for r in runs)))
    source=batch.relative_to(PROJECT_ROOT).as_posix() if batch.is_relative_to(PROJECT_ROOT) else str(batch)
    report=dict(source_batch=source,arguments=data['arguments'],groups=groups,runs=rows)
    output=project_path(output);atomic_json(output,report)
    history='加载共享历史地图' if data['arguments'].get('load_map') else '关闭历史加载'
    lines=['# 实际追击比较','',f'原始批次：`{report["source_batch"]}`。各策略恢复相同场景起点；{history}，所有观测仍累计到共享地图。',
           '', '| 场景 | 策略 | 结果 | 捕获仿真秒 | 规划次数 | 目标换向 | 目标路径最大误差 m |',
           '| --- | --- | --- | --- | --- | --- | --- |']
    for row in rows:
        seconds=row.get('capture_sim_seconds');error=row.get('target_path_error_max')
        lines.append(f'| {row["preset"]} | {row["strategy"]} | {row["outcome"]} | {seconds:.2f}' if seconds is not None else f'| {row["preset"]} | {row["strategy"]} | {row["outcome"]} | —')
        lines[-1]+=f' | {row.get("plans","—")} | {row.get("target_legs","—")} | '+(f'{error:.2f}' if error is not None else '—')+' |'
    lines+=['','这是有限预设上的实飞结果。捕获要求实际距离连续满足阈值，首次接近不等于捕获；无效目标、初始化故障不算成功。相同起点不意味着 Unreal 动态交通和运行负载完全确定；单轮比较没有统计显著性保证。','']
    output.with_suffix('.md').write_text('\n'.join(lines),encoding='utf-8')
    print(json.dumps(groups,ensure_ascii=False));return report


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('batch')
    parser.add_argument('--output',default='docs/chase_comparison.json');args=parser.parse_args()
    analyze(args.batch,args.output)


if __name__=='__main__':main()
