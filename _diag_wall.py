# -*- coding: utf-8 -*-
"""诊断：在无人机卡住位置重建体素图并检查 A* 为何无路径。"""
import sys
sys.path.insert(0, ".")

import time
import numpy as np
from config import Config
from nav.airsim_io import AirSimIO
from nav.pipeline import NavigationPipeline
from nav import distance_field

cfg = Config()
io = AirSimIO(cfg)

STUCK = np.array([119.37, 20.17, -21.41])
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

proj = NavigationPipeline._project_goal_free(coarse, coarse_dist, snap.state_pos,
                                             goal, min_cl=float(cfg.d_safe))
print(f"[diag] projected_goal={np.round(proj,2)}")

sv = coarse.world_to_voxel(snap.state_pos)
gv = coarse.world_to_voxel(goal)
pv = coarse.world_to_voxel(proj)
print(f"[diag] start_voxel={sv} goal_voxel={gv} proj_voxel={pv}")
print(f"[diag] start cell={coarse.data[sv[0],sv[1],sv[2]]} "
      f"goal cell={coarse.data[gv[0],gv[1],gv[2]]} "
      f"proj cell={coarse.data[pv[0],pv[1],pv[2]]} "
      f"proj dist={coarse_dist[pv[0],pv[1],pv[2]]:.2f}")

k = int(sv[2])
slab = coarse.data[:, :, k]
print(f"[diag] XY slice at z_voxel={k} (0=unk, .=free, ##=occ):")
oy = coarse.origin[1]
ox = coarse.origin[0]
ys = [y for y in range(-10, 46, 5)]
cols = [int((y - oy) / coarse.res) for y in ys]
print("     " + " ".join(f"{y:4d}" for y in ys))
for xi in range(coarse.data.shape[0]):
    x = ox + (xi + 0.5) * coarse.res
    if x < 108 or x > 153:
        continue
    row = []
    for ci in cols:
        if 0 <= ci < coarse.data.shape[1]:
            v = slab[xi, ci]
            row.append("##" if v == 2 else ("." if v == 1 else "  "))
        else:
            row.append("  ")
    print(f"x={x:5.1f} " + " ".join(row))

path = pipe.planner.plan(coarse, coarse_dist, snap.state_pos, proj)
print(f"[diag] A* (C++) result path_len={len(path) if path else None}")

# Python 回退 A*，大节点上限，看是否是节点上限问题
import nav.astar as astar_mod
saved = astar_mod.USING_FAST
astar_mod.USING_FAST = False
cfg_big = Config()
cfg_big.astar_timeout_nodes = 2000000
p_big = astar_mod.AStarPlanner(cfg_big)
path2 = p_big.plan(coarse, coarse_dist, snap.state_pos, proj)
print(f"[diag] A* (Python, max_nodes=2M) result path_len={len(path2) if path2 else None}")
astar_mod.USING_FAST = saved

pipe.stop()
