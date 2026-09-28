# -*- coding: utf-8 -*-
"""分析最近录制：轨迹终点/时长 + 全程净空 + A* 路径贴墙距离分布。

目的：确认（1）穿墙是否已由 mark_visible_free 修复解决（A* 不再穿 OCCUPIED）；
（2）量化「贴墙过近」——A* 路径与障碍的最近距离分布，为膨胀+距离惩罚调参提供依据。
"""
import json
import os
import numpy as np

ROOT = os.path.dirname(__file__)
results_dir = os.path.join(ROOT, "results")
dirs = sorted([d for d in os.listdir(results_dir)
               if os.path.isdir(os.path.join(results_dir, d))],
              key=lambda d: os.path.getmtime(os.path.join(results_dir, d)))
latest = dirs[-1]
rec_path = os.path.join(results_dir, latest, "recording.json")

with open(rec_path, "r", encoding="utf-8") as f:
    data = json.load(f)

states = data["states"]
plans = data["plans"]
maps = data["maps"]

s0, sN = states[0], states[-1]
print(f"录制 {latest}")
print(f"时长 {(sN['t']-s0['t']):.0f}s  起点 {np.round(s0['p'],2)}  终点 {np.round(sN['p'],2)}")
# 终点离目标距离
goal = np.array(data["goal"])
print(f"终点距目标 {np.linalg.norm(np.array(sN['p'])-goal):.1f}m  goal={goal}")

# 全程净空（clearance 在 telemetry 里，不在 recording states；这里从 state.velocity 与 maps 反推不了）
# 改为：分析所有 plan 的 A* 路径点到最近 occupied 的距离
print("\n=== A* 路径贴墙距离（每个 plan 的路径点 min_dist_to_occ）===")
all_min = []
for pl in plans:
    t = pl["t"]
    path = pl["path"]
    if not path or not maps:
        continue
    best_m = min(maps, key=lambda m: abs(m["t"] - t))
    occ = np.array(best_m["occ"]).reshape(-1, 3) if best_m["occ"] else np.zeros((0, 3))
    if occ.size == 0:
        continue
    for p in path:
        d = np.linalg.norm(occ - np.array(p), axis=1)
        if d.size:
            all_min.append((t, float(d.min()), np.array(p)))

if all_min:
    dm = np.array([a[1] for a in all_min])
    # 下采样 occ 体素中心到体素边的距离约 0.6m（stride=6 × res=0.2 = 1.2m 体素，半宽 0.6m）
    print(f"全部 {len(all_min)} 个路径点，min_dist 分布:")
    print(f"  min={dm.min():.2f}m  p10={np.percentile(dm,10):.2f}  p25={np.percentile(dm,25):.2f} "
          f"median={np.median(dm):.2f}  p75={np.percentile(dm,75):.2f}  max={dm.max():.2f}")
    # 最贴近障碍的 10 个路径点
    idx = np.argsort(dm)[:10]
    print("  最贴墙 10 个路径点（t, min_dist, pos）:")
    for i in idx:
        t, d, p = all_min[i]
        print(f"    t={t:.1f} d={d:.2f}m pos={np.round(p,2)}")
    # 每个 plan 的最小距离
    print("\n=== 每个 plan 的路径最小贴墙距离 ===")
    # 按 t 分组
    per_plan = {}
    for t, d, p in all_min:
        per_plan.setdefault(t, []).append(d)
    for t in sorted(per_plan):
        arr = per_plan[t]
        print(f"  t={t:.1f} 路径点min_dist={min(arr):.2f}m")
