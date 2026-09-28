# -*- coding: utf-8 -*-
"""诊断 run_00：分析两问题（速度突变/向下跑）的真实数据表现。"""
import json
import numpy as np

REC = r"results/20260926_161726/run_00/recording.json"
with open(REC, "r", encoding="utf-8") as f:
    d = json.load(f)

states = d["states"]
plans = d["plans"]
print(f"states={len(states)} plans={len(plans)}")

# 状态 z 随时间
ts = [s["t"] for s in states]
zs = [s["p"][2] for s in states]
vs = [np.linalg.norm(s["v"]) for s in states]
print(f"\n=== 状态 z 范围: {min(zs):.2f} ~ {max(zs):.2f}, 起始 z={zs[0]:.2f}, 终末 z={zs[-1]:.2f} ===")

# 每个 plan：goal z / path 首末 z / path z 范围
print("\n=== 每次重规划 ===")
for i, pl in enumerate(plans):
    goal = pl["goal"]
    path = pl["path"]
    if not path:
        print(f"[{i}] t={pl['t']:.1f} goal_z={goal[2]:.2f} path=空")
        continue
    pz = [p[2] for p in path]
    # 找到与 plan 时刻最接近的 state
    si = min(range(len(states)), key=lambda k: abs(states[k]["t"] - pl["t"]))
    s = states[si]
    print(f"[{i}] t={pl['t']:.1f} state_z={s['p'][2]:.2f} |v|={np.linalg.norm(s['v']):.2f} "
          f"goal=({goal[0]:.1f},{goal[1]:.1f},{goal[2]:.1f}) "
          f"path_z={min(pz):.1f}~{max(pz):.1f} path0_z={pz[0]:.1f} path_end_z={pz[-1]:.1f} n={len(path)}")

# 轨迹 ctrl 的 z（B-spline 控制点 z 范围）
print("\n=== 轨迹 ctrl z 范围 ===")
for i, pl in enumerate(plans):
    ctrl = pl.get("ctrl")
    if ctrl is None or len(ctrl) == 0:
        continue
    cz = [c[2] for c in ctrl]
    print(f"[{i}] t={pl['t']:.1f} ctrl_z={min(cz):.1f}~{max(cz):.1f} n_ctrl={len(ctrl)}")
