# -*- coding: utf-8 -*-
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from config import Config
from nav.occupancy import OccupancyMap, OCCUPIED
from nav.astar import AStarPlanner
from nav import distance_field


def _cfg():
    c = Config()
    c.res = 1.0
    c.d_min = 0.0
    c.d_safe = 0.5
    c.d_max = 3.0
    return c


def test_detour_around_wall():
    cfg = _cfg()
    m = OccupancyMap(20, 20, 5, 1.0, (0.0, 0.0, 0.0))
    m.data[10, 5:16, :] = OCCUPIED  # x=10 的墙，y∈[5,15]
    dist = distance_field.compute(m, cfg)
    planner = AStarPlanner(cfg)
    path = planner.plan(m, dist, np.array([2.0, 10.0, 2.0]),
                        np.array([18.0, 10.0, 2.0]))
    assert path is not None, "应能绕墙找到路径"
    ys = [p[1] for p in path]
    assert min(ys) < 5 or max(ys) > 15, "路径应绕过墙（y 越出 [5,15]）"
    for p in path:
        i, j, k = m.world_to_voxel(p)
        assert not m.is_occupied(i, j, k), "路径不应穿过占据体素"


def test_no_path_when_fully_blocked():
    cfg = _cfg()
    m = OccupancyMap(20, 20, 5, 1.0, (0.0, 0.0, 0.0))
    m.data[10, :, :] = OCCUPIED  # 整面墙
    dist = distance_field.compute(m, cfg)
    planner = AStarPlanner(cfg)
    path = planner.plan(m, dist, np.array([2.0, 10.0, 2.0]),
                        np.array([18.0, 10.0, 2.0]))
    assert path is None, "完全被堵时应返回 None"


def test_straight_path_when_clear():
    cfg = _cfg()
    m = OccupancyMap(20, 20, 5, 1.0, (0.0, 0.0, 0.0))
    m.data[:] = 1  # 全自由
    dist = distance_field.compute(m, cfg)
    planner = AStarPlanner(cfg)
    path = planner.plan(m, dist, np.array([2.0, 10.0, 2.0]),
                        np.array([18.0, 10.0, 2.0]))
    assert path is not None


if __name__ == "__main__":
    test_detour_around_wall()
    test_no_path_when_fully_blocked()
    test_straight_path_when_clear()
    print("test_astar: OK")
