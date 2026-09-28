# -*- coding: utf-8 -*-
"""诊断：复现真机 A* 无路径。合成 100x100x25 粗图（94% free / 6% unknown 远角 / 0 occ），
分别跑 C++ 与 Python A*，打印是否找到路径与展开节点数。"""
import sys
sys.path.insert(0, ".")

import time
import numpy as np

from nav.occupancy import OccupancyMap, UNKNOWN, FREE, OCCUPIED
from nav import distance_field, astar
from config import Config


def build_map():
    cfg = Config()
    # 粗网格参数（与 pipeline._coarsen 一致：r=2, eff_res=0.4, 100x100x25）
    eff_res = 0.4
    nx = ny = 100
    nz = 25
    # 无人机在细图 center_offset=(50,100,25)，粗图 = (25,50,12)
    origin = (-10.1, -20.1, -13.03)
    m = OccupancyMap(nx, ny, nz, eff_res, origin)
    # 全部先标 FREE（模拟 mark_visible_free 后的开放空域）
    m.data.fill(FREE)
    # 远角（超出 30m 泛洪半径）标 UNKNOWN
    cx, cy, cz = 25, 50, 12
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                d = np.sqrt((i - cx) ** 2 + (j - cy) ** 2 + (k - cz) ** 2)
                if d * eff_res > cfg.max_range:
                    m.data[i, j, k] = UNKNOWN
    return m, cfg


def main():
    m, cfg = build_map()
    occ = int((m.data == OCCUPIED).sum())
    unk = int((m.data == UNKNOWN).sum())
    free = int((m.data == FREE).sum())
    print(f"map: occ={occ} unk={unk} free={free}  res={m.res} origin={m.origin}")
    dist = distance_field.compute(m, cfg)
    start = np.array([0.0, 0.0, -7.93])
    goal = np.array([29.7, 19.7, -12.83])

    sv = m.world_to_voxel(start)
    gv = m.world_to_voxel(goal)
    print(f"start_v={sv} goal_v={gv} in_bounds={m.in_bounds(*sv)}/{m.in_bounds(*gv)}")
    print(f"start_state={m.data[sv]} goal_state={m.data[gv]} "
          f"start_d={dist[sv]:.2f} goal_d={dist[gv]:.2f}")
    print(f"USING_FAST={astar.USING_FAST}")

    p = astar.AStarPlanner(cfg)
    t0 = time.time()
    path = p.plan(m, dist, start, goal)
    dt = time.time() - t0
    print(f"[C++/默认] path={'None' if path is None else len(path)} 耗时={dt:.3f}s")

    # 强制 Python 回退
    old = astar.USING_FAST
    astar.USING_FAST = False
    t0 = time.time()
    path2 = p.plan(m, dist, start, goal)
    dt2 = time.time() - t0
    astar.USING_FAST = old
    print(f"[Python] path={'None' if path2 is None else len(path2)} 耗时={dt2:.3f}s")


if __name__ == "__main__":
    main()
