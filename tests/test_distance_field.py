# -*- coding: utf-8 -*-
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from config import Config
from nav.occupancy import OccupancyMap
from nav import distance_field


def test_edt_basic():
    cfg = Config()
    m = OccupancyMap(30, 30, 30, 0.2, (0.0, 0.0, 0.0))
    m.mark_occupied(15, 15, 15)
    d = distance_field.compute(m, cfg)
    assert d[15, 15, 15] == 0.0
    # 面邻接体素距离 = res（scipy 精确 / BFS 回退均为此值）
    assert abs(d[16, 15, 15] - 0.2) < 1e-3
    # 距障碍足够远的体素截断到 d_max（角点(0,0,0)距(15,15,15)=5.196m < d_max=6.0，
    # 不截断；改把障碍放到远角(25,25,25)，则角点距其=8.66m > 6.0 会截断）
    m2 = OccupancyMap(30, 30, 30, 0.2, (0.0, 0.0, 0.0))
    m2.mark_occupied(25, 25, 25)
    d2 = distance_field.compute(m2, cfg)
    assert d2[0, 0, 0] == cfg.d_max


def test_unknown_not_obstacle():
    cfg = Config()
    m = OccupancyMap(60, 60, 60, 0.2, (0.0, 0.0, 0.0))
    m.mark_occupied(15, 15, 15)
    d = distance_field.compute(m, cfg)
    # 未观测（Unknown）体素不应成为障碍源：远角(59,59,59)距障碍=sqrt(3*44^2)*0.2≈15.2m > d_max，
    # 应截断到 d_max（若 Unknown 被误当障碍，这里会变成更小的距离）
    assert d[59, 59, 59] == cfg.d_max


if __name__ == "__main__":
    test_edt_basic()
    test_unknown_not_obstacle()
    print("test_distance_field: OK (using scipy=%s)" % distance_field.USING_SCIPY)
