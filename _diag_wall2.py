# -*- coding: utf-8 -*-
"""诊断：在最新卡墙位置重建地图，验证 d_min 修复后的前沿投影 + A* 是否可达。"""
import sys
sys.path.insert(0, ".")

import time
import numpy as np
from scipy import ndimage
from config import Config
from nav.airsim_io import AirSimIO
from nav.pipeline import NavigationPipeline
from nav import distance_field
from nav.occupancy import FREE, UNKNOWN, OCCUPIED

cfg = Config()
io = AirSimIO(cfg)

STUCK = np.array([134.0, 20.0, -17.5])
io.teleport(STUCK, yaw_deg=0.0)
time.sleep(0.5)

pipe = NavigationPipeline(io, cfg)
pipe.set_goal(np.array([900.0, 20.0, -20.0]))
pipe.start()

for _ in range(40):
    time.sleep(0.2)
    if pipe._snapshot is not None:
        break

snap = pipe._snapshot
if snap is None:
    print("[diag] no snapshot!")
    pipe.stop()
    sys.exit(1)

print(f"[diag] state_pos={np.round(snap.state_pos,2)} origin={np.round(snap.origin,2)}")
occ = snap.occ
print(f"[diag] occ shape={occ.shape} occ_n={(occ==2).sum()} "
      f"unk_n={(occ==0).sum()} free_n={(occ==1).sum()}")

goal = NavigationPipeline._local_goal(snap.state_pos, pipe.goal, snap.origin,
                                      cfg.res, cfg.nx, cfg.ny, cfg.nz)
coarse = NavigationPipeline._coarsen(snap.occ, snap.origin, cfg.res, cfg.plan_res)
coarse_dist = distance_field.compute(coarse, cfg)
print(f"[diag] local_goal={np.round(goal,2)}")
print(f"[diag] coarse shape={coarse.data.shape} res={coarse.res} "
      f"origin={np.round(coarse.origin,2)}")

d_min = float(cfg.d_min)
d_safe = float(cfg.d_safe)
proj_new = NavigationPipeline._project_goal_free(
    coarse, coarse_dist, snap.state_pos, goal, min_cl=d_safe, d_min=d_min)
proj_old = NavigationPipeline._project_goal_free(
    coarse, coarse_dist, snap.state_pos, goal, min_cl=d_safe, d_min=0.0)
print(f"[diag] projected_goal (NEW d_min={d_min}) ={np.round(proj_new,2)}")
print(f"[diag] projected_goal (OLD d_min=0)    ={np.round(proj_old,2)}")

sv = np.asarray(coarse.world_to_voxel(snap.state_pos)).astype(int)
gv = np.asarray(coarse.world_to_voxel(goal)).astype(int)
pv = np.asarray(coarse.world_to_voxel(proj_new)).astype(int)
print(f"[diag] start_voxel={sv} goal_voxel={gv} proj_voxel={pv}")

data = coarse.data
# 可达分量分析（对齐 A* 语义）
traversable = (data == UNKNOWN) | ((data == FREE) & (coarse_dist >= d_min))
lbl, ncomp = ndimage.label(traversable)
print(f"[diag] traversable components = {ncomp}")
sl = lbl[sv[0], sv[1], sv[2]]
reach = lbl == sl
print(f"[diag] start label={sl} reach_n={int(reach.sum())} "
      f"goal reachable={bool(reach[gv[0], gv[1], gv[2]])}")

# 前沿（NEW 语义）：可达 + FREE + 净空 >= d_safe，邻接 UNKNOWN 或地图边
cand = reach & (data == FREE) & (coarse_dist >= d_safe)
if not cand.any():
    cand = reach & (data == FREE)
nb_unk = ndimage.maximum_filter((data == UNKNOWN).astype("uint8"),
                                size=3, mode="constant", cval=1)
frontier = cand & (nb_unk > 0)
print(f"[diag] cand_n={int(cand.sum())} frontier_n={int(frontier.sum())}")

# 打印 XY 切片（start 的 Z 层）
k = int(sv[2])
slab = data[:, :, k]
print(f"[diag] XY slice at z_voxel={k} (  =unk, . =free, ##=occ):")
ox, oy = coarse.origin[0], coarse.origin[1]
ys = [int((y - oy) / coarse.res) for y in range(0, 51, 5)]
print("       " + " ".join(f"Y{y:>3d}" for y in range(0, 51, 5)))
for xi in range(data.shape[0]):
    x = ox + (xi + 0.5) * coarse.res
    if x < 120 or x > 165:
        continue
    row = []
    for ci in ys:
        if 0 <= ci < data.shape[1]:
            v = slab[xi, ci]
            row.append("##" if v == OCCUPIED else ("." if v == FREE else "  "))
        else:
            row.append("  ")
    print(f"X{x:5.1f} " + " ".join(row))

# A* 能否到达 NEW 投影点
path = pipe.planner.plan(coarse, coarse_dist, snap.state_pos, proj_new)
print(f"[diag] A* -> NEW proj  result len={len(path) if path else None}")
path2 = pipe.planner.plan(coarse, coarse_dist, snap.state_pos, proj_old)
print(f"[diag] A* -> OLD proj  result len={len(path2) if path2 else None}")

pipe.stop()
