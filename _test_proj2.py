# -*- coding: utf-8 -*-
"""确定性验证 _project_goal_free 前沿投影：目标被建筑墙挡住时，应投影到
可达分量内的街道前沿点（绕过建筑），且 A* 能找到路径。"""
import sys
import numpy as np
sys.path.insert(0, ".")

from scipy import ndimage
from scipy.ndimage import distance_transform_edt
from nav.occupancy import OccupancyMap, FREE, UNKNOWN, OCCUPIED
from nav.pipeline import NavigationPipeline
from nav.astar import AStarPlanner
from config import Config

cfg = Config()
m = OccupancyMap(80, 80, 20, 0.4, (0.0, 0.0, 0.0))
m.data[...] = UNKNOWN
# 建筑墙：X=40..60, Y=20..60, 全 Z —— 挡住正前方，两侧留街道（Y<20 / Y>60）
m.data[40:61, 20:61, :] = OCCUPIED
m.data[:40, :, :] = FREE          # 建筑前（无人机一侧）已探测
m.data[61:, :20, :] = FREE        # 南街道（绕过建筑后）已探测
m.data[61:, 61:, :] = FREE        # 北街道
sv = (20, 40, 10)
m.data[sv] = FREE
d = distance_transform_edt(m.data != OCCUPIED, sampling=(m.res,) * 3).astype("float32")
d = np.minimum(d, cfg.d_max)

state_pos = m.voxel_to_world(*sv)
goal = m.voxel_to_world(70, 40, 10)   # 建筑正后方，UNKNOWN
proj = NavigationPipeline._project_goal_free(m, d, state_pos, goal, min_cl=float(cfg.d_safe))
pv = m.world_to_voxel(proj)
print(f"[test] start={sv} goal_voxel={m.world_to_voxel(goal)} proj_voxel={pv}")
print(f"[test] proj_world={np.round(proj, 2)}")

lbl, n = ndimage.label(m.data != OCCUPIED)
print(f"[test] start_label={lbl[sv]} goal_label={lbl[m.world_to_voxel(goal)]} "
      f"proj_label={lbl[pv]} (共 {n} 分量)")
assert lbl[pv] == lbl[sv], "投影点必须与起点同连通分量"

path = AStarPlanner(cfg).plan(m, d, state_pos, proj)
print(f"[test] A* -> proj path_len={len(path) if path else None}")
assert path and len(path) >= 2, "A* 应找到到达投影点的路径"
print(f"[test] 起点={np.round(path[0],1)} 终点={np.round(path[-1],1)} "
      f"中途最北Y={min(p[1] for p in path):.1f} 最南Y={max(p[1] for p in path):.1f}")
print("[test] OK：前沿投影把目标绕到了街道，A* 有路径")
