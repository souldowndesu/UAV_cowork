# -*- coding: utf-8 -*-
"""诊断：在 X=134 卡墙位重建地图，查 A* 走向 + 墙后是否被 mark_visible_free 泛洪成 FREE。"""
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

STUCK = np.array([134.38, 20.62, -19.08])
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

d_min = float(cfg.d_min)
d_safe = float(cfg.d_safe)
proj = NavigationPipeline._project_goal_free(
    coarse, coarse_dist, snap.state_pos, goal, min_cl=d_safe, d_min=d_min)
print(f"[diag] projected_goal ={np.round(proj,2)}")

sv = np.asarray(coarse.world_to_voxel(snap.state_pos)).astype(int)
pv = np.asarray(coarse.world_to_voxel(proj)).astype(int)
print(f"[diag] start_voxel={sv} proj_voxel={pv}")
data = coarse.data
print(f"[diag] data[start]={data[sv[0],sv[1],sv[2]]} dist[start]={coarse_dist[sv[0],sv[1],sv[2]]:.2f}")
print(f"[diag] data[proj]={data[pv[0],pv[1],pv[2]]} dist[proj]={coarse_dist[pv[0],pv[1],pv[2]]:.2f}")

# A* (C++)
t0 = time.perf_counter()
path = pipe.planner.plan(coarse, coarse_dist, snap.state_pos, proj)
dt = time.perf_counter() - t0
print(f"[diag] C++ A*(500k) len={len(path) if path else None} t={dt*1000:.1f}ms")
if path:
    print(f"[diag]   path[0]={np.round(path[0],1)}")
    print(f"[diag]   path[2]={np.round(path[2],1)}")
    print(f"[diag]   path[-1]={np.round(path[-1],1)}")

# 墙后是否被泛洪成 FREE：打印 start Z 层的 XY 切片
z = sv[2]
sl_xy = data[:, :, z]
print(f"[diag] XY slice at z_voxel={z} (Z={coarse.origin[2]+(z+0.5)*coarse.res:.1f}):")
x0 = max(0, sv[0]-8); x1 = min(data.shape[0], sv[0]+24)
y0 = max(0, sv[1]-14); y1 = min(data.shape[1], sv[1]+16)
chars = {2: '#', 0: '.', 1: ' '}
for j in range(y1-1, y0-1, -1):
    row = ''.join(chars.get(int(data[i, j, z]), '?') for i in range(x0, x1))
    print(f"    Y={y0+j:3d} |{row}|")
print(f"    {' '*5}  X:{x0}..{x1-1}  (row=+Y/east, col=+X)")
print(f"[diag] legend: #=occ .=unk ' '=free; start=({sv[0]},{sv[1]}) proj=({pv[0]},{pv[1]})")

pipe.stop()
