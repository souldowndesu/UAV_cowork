# -*- coding: utf-8 -*-
"""验证 d_min 修复：窄缝（净空<d_min）不应被投影当作可达通道。"""
import numpy as np
from nav.occupancy import OccupancyMap, FREE, UNKNOWN, OCCUPIED
from nav import distance_field
from nav.pipeline import NavigationPipeline
from config import Config

cfg = Config()
res = 0.4
nx, ny, nz = 40, 40, 10
origin = (0.0, 0.0, 0.0)
cm = OccupancyMap(nx, ny, nz, res, origin)
data = np.full((nx, ny, nz), UNKNOWN, dtype=np.uint8)

# 墙前开阔区
data[:20, :, :] = FREE
# 墙：X=20，Y 全范围（全高），仅留一条窄缝 Y=36
data[20, :, :] = OCCUPIED
# 窄缝：Y=36（1 体素 = 净空 0.4 < d_min=0.5）
data[20, 36, :] = FREE
# 墙后区域被 mark_visible_free 泛洪标成 FREE
data[21:, :, :] = FREE

cm.data = data
dist = distance_field.compute(cm, cfg)

# 无人机在墙前 (voxel 17,20,5)；全局目标在墙后 (voxel 30,20,5)
state_pos = np.array([7.0, 8.0, 2.0])   # voxel (17.5, 20, 5)
goal = np.array([12.0, 8.0, 2.0])       # voxel (30, 20, 5)

sv = np.asarray(cm.world_to_voxel(state_pos)).astype(int)
print(f"state voxel={sv} data={data[tuple(sv)]}")
print(f"gap dist @(20,36,5)={dist[20,36,5]:.2f}")

proj_new = NavigationPipeline._project_goal_free(
    cm, dist, state_pos, goal, min_cl=2.0, d_min=float(cfg.d_min))
proj_old = NavigationPipeline._project_goal_free(
    cm, dist, state_pos, goal, min_cl=2.0, d_min=0.0)  # 等价旧版 data!=OCCUPIED

pv = np.asarray(cm.world_to_voxel(proj_new)).astype(int)
print(f"NEW proj world={np.round(proj_new,2)} voxel={pv} (X voxel <20 = 墙前, >=21 = 墙后)")
print(f"OLD proj world={np.round(proj_old,2)} voxel={np.asarray(cm.world_to_voxel(proj_old)).astype(int)}")
assert pv[0] < 20, f"FAIL: 修复后仍投影到墙后 voxel X={pv[0]}"
print("PASS: d_min 修复使投影不再穿过窄缝")
