# -*- coding: utf-8 -*-
import json, os, glob
import numpy as np

for rec in ['122033', '122045', '122336', '122543']:
    p = f'results/20260925_{rec}/recording.json'
    d = json.load(open(p, encoding='utf-8'))
    st = d['states']
    if not st:
        print(f'{rec}: goal={d.get("goal")} EMPTY states')
        continue
    t0 = st[0]['t']
    t1 = st[-1]['t']
    goal = d['goal']
    start = st[0]['p']
    end = st[-1]['p']
    # 末段速度（最后 30 帧）
    v_end = np.mean([np.linalg.norm(s['v']) for s in st[-30:]])
    # 目标距离
    dist_end = np.linalg.norm(np.array(end) - np.array(goal))
    # 停止检测：最后 2 秒是否几乎不动
    last2 = [s for s in st if s['t'] >= t1 - 2.0]
    moved = np.linalg.norm(np.array(last2[-1]['p']) - np.array(last2[0]['p'])) if last2 else 0
    print(f'{rec}: goal={goal} dur={t1-t0:.1f}s')
    print(f'   start={start} end={end}  end_dist_to_goal={dist_end:.1f}m  v_end={v_end:.2f}  moved_last2s={moved:.2f}m')
