# -*- coding: utf-8 -*-
"""诊断：在 X=109.82 首次 A* 失败位置重建地图，查投影点为何 A* 不可达。"""
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

STUCK = np.array([109.82, 19.35, -21.34])
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
      f"proj_reachable={bool(reach[pv[0], pv[1], pv[2]])}")
print(f"[diag] data[proj]={data[pv[0],pv[1],pv[2]]} "
      f"dist[proj]={coarse_dist[pv[0],pv[1],pv[2]]:.2f}")
print(f"[diag] data[start]={data[sv[0],sv[1],sv[2]]} "
      f"dist[start]={coarse_dist[sv[0],sv[1],sv[2]]:.2f}")

# C++ A*（cfg.astar_timeout_nodes 现为 500000）
t0 = time.perf_counter()
path_cpp = pipe.planner.plan(coarse, coarse_dist, snap.state_pos, proj)
dt_cpp = time.perf_counter() - t0
print(f"[diag] C++ A*(500k) -> proj  len={len(path_cpp) if path_cpp else None}  t={dt_cpp*1000:.1f}ms")

# Python 回退（绕过 C++）
import nav.astar as astar_mod
_prev = astar_mod.USING_FAST
astar_mod.USING_FAST = False
path_py = pipe.planner.plan(coarse, coarse_dist, snap.state_pos, proj)
astar_mod.USING_FAST = _prev
print(f"[diag] PY  A* -> proj  len={len(path_py) if path_py else None}")

# Python A* 加大节点上限，判断是否节点上限问题
astar_mod.USING_FAST = False
pipe.planner.max_nodes = 2000000
t0 = time.perf_counter()
path_py2 = pipe.planner.plan(coarse, coarse_dist, snap.state_pos, proj)
dt_py2 = time.perf_counter() - t0
astar_mod.USING_FAST = _prev
pipe.planner.max_nodes = int(cfg.astar_timeout_nodes)
print(f"[diag] PY  A*(2M) -> proj  len={len(path_py2) if path_py2 else None}  t={dt_py2*1000:.1f}ms")

pipe.stop()
