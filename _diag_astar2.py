# -*- coding: utf-8 -*-
"""诊断2：验证“目标投影到 FREE 空间”能否解决 A* 无路径。"""
import sys
sys.path.insert(0, ".")

import time
import numpy as np

from nav.occupancy import OccupancyMap, UNKNOWN, FREE, OCCUPIED
from nav import distance_field, astar
from config import Config


def build_map():
    cfg = Config()
    eff_res = 0.4
    nx = ny = 100
    nz = 25
    origin = (-10.1, -20.1, -13.03)
    m = OccupancyMap(nx, ny, nz, eff_res, origin)
    m.data.fill(FREE)
    cx, cy, cz = 25, 50, 12
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                d = np.sqrt((i - cx) ** 2 + (j - cy) ** 2 + (k - cz) ** 2)
                if d * eff_res > cfg.max_range:
                    m.data[i, j, k] = UNKNOWN
    return m, cfg


def project_goal_free(m, start, goal):
    """沿 goal→start 连线把目标回拉到 FREE 体素（26 连通可达）。"""
    sv = m.world_to_voxel(start)
    gv = m.world_to_voxel(goal)
    n = max(1, int(np.linalg.norm(np.array(gv) - np.array(sv))) * 2)
    for t in np.linspace(0.0, 1.0, n + 1):
        v = (np.array(gv) * (1 - t) + np.array(sv) * t).astype(int)
        if m.in_bounds(*v) and m.data[tuple(v)] == FREE:
            return m.voxel_to_world(*v)
    return start


def main():
    m, cfg = build_map()
    dist = distance_field.compute(m, cfg)
    start = np.array([0.0, 0.0, -7.93])
    goal = np.array([29.7, 19.7, -12.83])
    print(f"occ={int((m.data==2).sum())} unk={int((m.data==0).sum())} free={int((m.data==1).sum())}")

    p = astar.AStarPlanner(cfg)

    for label, g in [("原始(UNKNOWN)", goal), ("投影到FREE", project_goal_free(m, start, goal))]:
        gv = m.world_to_voxel(g)
        gs = int(m.data[gv])
        t0 = time.time()
        path = p.plan(m, dist, start, g)
        dt = time.time() - t0
        print(f"[{label}] goal={np.round(g,2)} state={gs} "
              f"path={'None' if path is None else len(path)} 耗时={dt:.3f}s")


if __name__ == "__main__":
    main()
