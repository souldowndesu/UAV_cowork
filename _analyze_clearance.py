# -*- coding: utf-8 -*-
"""分析最新录制的 A* 路径净空，验证距离惩罚是否生效。"""
import json
import os
import glob as _glob
import numpy as np

base = os.path.dirname(os.path.abspath(__file__))
results = os.path.join(base, "results")
recs = sorted(_glob.glob(os.path.join(results, "*", "recording.json")))
if not recs:
    print("no recordings")
    raise SystemExit
rec = recs[-1]
print("recording:", rec)

with open(rec, "r", encoding="utf-8") as f:
    d = json.load(f)

print("goal:", d.get("goal"))
print("map_size:", d.get("map_size"))
print("n_states:", len(d.get("states", [])),
      "n_plans:", len(d.get("plans", [])),
      "n_maps:", len(d.get("maps", [])))

states = d.get("states", [])
if states:
    p0 = states[0]["p"]; p1 = states[-1]["p"]
    print("start:", p0, "end:", p1, "t:", states[0]["t"], "->", states[-1]["t"])
    print("end dist to goal:", np.linalg.norm(np.array(p1) - np.array(d["goal"])) if d.get("goal") else None)

# 用最后一张地图的 occupied 体素做净空参考（体素中心，下采样 stride=6，res=0.2 -> 1.2m 间距）
maps = d.get("maps", [])
if maps:
    occ = np.array(maps[-1]["occ"], dtype="float64").reshape(-1, 3)
    print("last map t:", maps[-1]["t"], "n_occ:", occ.shape[0])
else:
    occ = np.zeros((0, 3))

plans = d.get("plans", [])
print("last plan t:", plans[-1]["t"] if plans else None)


def min_clearance(pts, occ):
    if occ.shape[0] == 0 or pts.shape[0] == 0:
        return None
    # 对每个点求到最近 occupied 体素的距离（KD-tree 太麻烦，直接分块暴力，点数不多）
    dmin = np.full(pts.shape[0], np.inf)
    for o in occ:
        dd = np.linalg.norm(pts - o, axis=1)
        dmin = np.minimum(dmin, dd)
    return dmin


# 统计每个 plan 的 path 净空
for i, pl in enumerate(plans[-6:]):
    path = np.array(pl["path"], dtype="float64")
    if path.ndim != 2 or path.shape[0] == 0:
        print(f"plan[{i}] empty path")
        continue
    cl = min_clearance(path, occ)
    if cl is None:
        print(f"plan[{i}] no map")
        continue
    print(f"plan[{i}] t={pl['t']:.1f} n={path.shape[0]} "
          f"min_cl={cl.min():.3f} med_cl={np.median(cl):.3f} mean_cl={cl.mean():.3f} "
          f"frac<1.0m={np.mean(cl < 1.0):.2f} frac<0.5m={np.mean(cl < 0.5):.2f}")

# 最终无人机轨迹 vs 最后地图的净空（实际飞行贴墙程度）
if states and occ.shape[0] > 0:
    tr = np.array([s["p"] for s in states], dtype="float64")
    cl = min_clearance(tr[::10], occ)
    if cl is not None:
        print("drone traj: min_cl=%.3f med=%.3f frac<1.0m=%.2f frac<0.5m=%.2f"
              % (cl.min(), np.median(cl), np.mean(cl < 1.0), np.mean(cl < 0.5)))
