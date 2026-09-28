# -*- coding: utf-8 -*-
"""受控实验：验证 h_weight 是否中和了 A* 距离惩罚。

构造一个中间障碍盒，无人机须绕行。对比不同 h_weight 下路径的净空。
"""
import sys
import os
import numpy as np

base = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, base)

from nav.occupancy import OccupancyMap, OCCUPIED, FREE
from nav import distance_field, astar
from config import Config


def make_scene(res):
    nx, ny, nz = 120, 100, 20  # 48 x 40 x 8 m
    m = OccupancyMap(nx, ny, nz, res, (0.0, 0.0, 0.0))
    m.data[:] = FREE
    # 障碍盒：x[20,28] y[14,26] 全高
    i0, i1 = int(20 / res), int(28 / res)
    j0, j1 = int(14 / res), int(26 / res)
    m.data[i0:i1, j0:j1, :] = OCCUPIED
    return m


def run(cfg, label):
    res = cfg.plan_res
    m = make_scene(res)
    dist = distance_field.compute(m, cfg)
    start = np.array([5.0, 20.0, 4.0])
    goal = np.array([43.0, 20.0, 4.0])
    planner = astar.AStarPlanner(cfg)
    path = planner.plan(m, dist, start, goal)
    if path is None:
        print(f"[{label}] no path")
        return None
    pts = np.array(path)
    occ_idx = np.argwhere(m.data == OCCUPIED)
    occ_world = m.origin + (occ_idx.astype("float64") + 0.5) * res
    dmin = np.full(pts.shape[0], np.inf)
    for o in occ_world:
        dmin = np.minimum(dmin, np.linalg.norm(pts - o, axis=1))
    print(f"[{label}] n={pts.shape[0]} min_cl={dmin.min():.3f} "
          f"med_cl={np.median(dmin):.3f} mean_cl={dmin.mean():.3f} "
          f"frac<1m={np.mean(dmin < 1.0):.2f} frac<2m={np.mean(dmin < 2.0):.2f}")
    print(f"[{label}] y range: {pts[:,1].min():.2f} .. {pts[:,1].max():.2f} "
          f"(box y=[14,26])")
    return dmin


if __name__ == "__main__":
    c5 = Config(); c5.h_weight = 5.0; c5.plan_res = 0.4
    c1 = Config(); c1.h_weight = 1.0; c1.plan_res = 0.4
    c12 = Config(); c12.h_weight = 1.2; c12.plan_res = 0.4
    run(c5, "h_weight=5.0 (当前)")
    run(c1, "h_weight=1.0")
    run(c12, "h_weight=1.2")
