# -*- coding: utf-8 -*-
"""诊断：卡墙位 (134,20.6,-18.4) 打印无人机周围 3D 邻域 + A* 路径走向。"""
import sys
sys.path.insert(0, ".")

import time
import numpy as np
from config import Config
from nav.airsim_io import AirSimIO
from nav.pipeline import NavigationPipeline
from nav import distance_field
from nav.occupancy import FREE, UNKNOWN, OCCUPIED

cfg = Config()
io = AirSimIO(cfg)

STUCK = np.array([134.38, 20.62, -18.4])
io.teleport(STUCK, yaw_deg=0.0)
time.sleep(0.5)

pipe = NavigationPipeline(io, cfg)
pipe.set_goal(np.array([900.0, 20.0, -20.0]))
pipe.start()
for _ in range(60):
    time.sleep(0.2)
    if pipe._snapshot is not None:
        break
snap = pipe._snapshot
if snap is None:
    print("no snapshot"); pipe.stop(); sys.exit(1)

coarse = NavigationPipeline._coarsen(snap.occ, snap.origin, cfg.res, cfg.plan_res)
data = coarse.data
sv = np.asarray(coarse.world_to_voxel(snap.state_pos)).astype(int)
print(f"[diag] state_pos={np.round(snap.state_pos,2)} sv={sv} "
      f"origin={np.round(coarse.origin,2)} res={coarse.res}")
print(f"[diag] data[sv]={data[sv[0],sv[1],sv[2]]}")

# 打印 XY 切片，Y 从 35 到 70（覆盖无人机 Y=50）
chars = {2: '#', 0: '.', 1: ' '}
for z in range(sv[2]-2, sv[2]+3):
    if not (0 <= z < data.shape[2]):
        continue
    print(f"\n=== XY slice z_voxel={z} (Z={coarse.origin[2]+(z+0.5)*coarse.res:.1f}) ===")
    x0 = max(0, sv[0]-6); x1 = min(data.shape[0], sv[0]+16)
    y0 = max(0, sv[1]-15); y1 = min(data.shape[1], sv[1]+20)
    for j in range(y1-1, y0-1, -1):
        row = ''.join(chars.get(int(data[i, j, z]), '?') for i in range(x0, x1))
        mark = 'S' if j == sv[1] else ' '
        print(f"    Y={y0+j:3d} |{row}|{mark}")
    print(f"    {' '*6} X:{x0}..{x1-1}  (row=+Y/east, col=+X)")
print(f"\n[diag] legend: #=occ .=unk ' '=free; S=drone Y row; sv=({sv[0]},{sv[1]},{sv[2]})")

# A* 路径走向（含 start 回退）
goal = NavigationPipeline._local_goal(snap.state_pos, pipe.goal, snap.origin,
                                      cfg.res, cfg.nx, cfg.ny, cfg.nz)
coarse_dist = distance_field.compute(coarse, cfg)
proj = NavigationPipeline._project_goal_free(
    coarse, coarse_dist, snap.state_pos, goal, min_cl=float(cfg.d_safe), d_min=float(cfg.d_min))
path = pipe.planner.plan(coarse, coarse_dist, snap.state_pos, proj)
print(f"[diag] proj={np.round(proj,2)}  path len={len(path) if path else None}")
if path:
    for k in range(0, min(8, len(path))):
        print(f"    path[{k}]={np.round(path[k],1)}")

pipe.stop()
