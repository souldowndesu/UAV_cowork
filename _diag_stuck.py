# -*- coding: utf-8 -*-
"""诊断：在 (145,40.8,-17) 新卡点重建地图，查障碍布局 + 投影 + A* 路径走向。"""
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

STUCK = np.array([145.0, 40.8, -17.0])
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
proj = NavigationPipeline._project_goal_free(
    coarse, coarse_dist, snap.state_pos, goal, min_cl=d_safe, d_min=d_min)
print(f"[diag] projected_goal ={np.round(proj,2)}")

sv = np.asarray(coarse.world_to_voxel(snap.state_pos)).astype(int)
pv = np.asarray(coarse.world_to_voxel(proj)).astype(int)
print(f"[diag] start_voxel={sv} proj_voxel={pv}")

data = coarse.data
traversable = (data == UNKNOWN) | ((data == FREE) & (coarse_dist >= d_min))
lbl, ncomp = ndimage.label(traversable)
sl = lbl[sv[0], sv[1], sv[2]]
reach = lbl == sl
print(f"[diag] ncomp={ncomp} start_label={sl} reach_n={int(reach.sum())} "
      f"proj_reachable={bool(reach[pv[0],pv[1],pv[2]])}")
print(f"[diag] data[proj]={data[pv[0],pv[1],pv[2]]} "
      f"dist[proj]={coarse_dist[pv[0],pv[1],pv[2]]:.2f}")
print(f"[diag] data[start]={data[sv[0],sv[1],sv[2]]} "
      f"dist[start]={coarse_dist[sv[0],sv[1],sv[2]]:.2f}")

# C++ A*
t0 = time.perf_counter()
path_cpp = pipe.planner.plan(coarse, coarse_dist, snap.state_pos, proj)
dt_cpp = time.perf_counter() - t0
print(f"[diag] C++ A*(500k) -> proj  len={len(path_cpp) if path_cpp else None}  t={dt_cpp*1000:.1f}ms")
if path_cpp:
    print(f"[diag]   path[0]={np.round(path_cpp[0],1)}")
    print(f"[diag]   path[3]={np.round(path_cpp[3],1)}")
    print(f"[diag]   path[-1]={np.round(path_cpp[-1],1)}")

# 2D XY 切片（在 start 的 Z 层）
z = sv[2]
sl_xy = data[:, :, z].T  # 转置使 X 横轴 Y 纵轴
print(f"[diag] XY slice at z_voxel={z} (Z={coarse.origin[2]+(z+0.5)*coarse.res:.1f}):")
# 打印 start 附近 25x25 区域（X: sv[0]-10..+15, Y: sv[1]-10..+15）
x0 = max(0, sv[0]-10); x1 = min(data.shape[0], sv[0]+16)
y0 = max(0, sv[1]-10); y1 = min(data.shape[1], sv[1]+16)
sub = data[x0:x1, y0:y1, z]
chars = {2: '#', 0: '.', 1: ' '}
for j in range(y1-1, y0-1, -1):
    row = ''.join(chars.get(int(sub[i, j-y0]), '?') for i in range(x1-x0))
    print(f"    Y={y0+j:3d} |{row}|")
print(f"    {' '*5}  X:{x0}..{x1-1}  (col=+X/north, row=+Y/east)")
print(f"[diag] legend: #=occupied .=unknown ' '=free; start=({sv[0]},{sv[1]}) proj=({pv[0]},{pv[1]})")

pipe.stop()
