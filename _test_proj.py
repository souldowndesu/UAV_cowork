# -*- coding: utf-8 -*-
"""验证 pipeline._project_goal_free 在真实粗图上的行为。"""
import sys
sys.path.insert(0, ".")

import numpy as np
from nav.occupancy import OccupancyMap, UNKNOWN, FREE, OCCUPIED
from nav.pipeline import NavigationPipeline


def build_coarse():
    # 仿 _coarsen 结果：100x100x25, eff_res=0.4, origin=(-10.1,-20.1,-13.03)
    nx = ny = 100
    nz = 25
    res = 0.4
    origin = (-10.1, -20.1, -13.03)
    m = OccupancyMap(nx, ny, nz, res, origin)
    m.data.fill(FREE)
    # 30m 泛洪半径外的体素标 UNKNOWN
    cx, cy, cz = 25, 50, 12
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                if np.sqrt((i-cx)**2 + (j-cy)**2 + (k-cz)**2) * res > 30.0:
                    m.data[i, j, k] = UNKNOWN
    return m


def main():
    m = build_coarse()
    state = np.array([0.0, 0.0, -7.93])
    goal = np.array([29.7, 19.7, -12.83])  # 夹到地图角的局部目标（UNKNOWN）
    p = NavigationPipeline._project_goal_free(m, state, goal)
    pv = m.world_to_voxel(p)
    print(f"state={state} goal_in={goal} state_v={m.world_to_voxel(state)}")
    print(f"投影后 goal={np.round(p,2)} state={int(m.data[pv])} "
          f"(FREE={FREE}, UNKNOWN={UNKNOWN})")
    assert m.data[pv] == FREE, "投影目标必须是 FREE"
    # 目标本身已 FREE 时原样返回
    assert np.allclose(NavigationPipeline._project_goal_free(m, state, state), state)
    print("OK")


if __name__ == "__main__":
    main()
