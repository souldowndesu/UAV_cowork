# -*- coding: utf-8 -*-
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from config import Config
from nav.occupancy import OccupancyMap, FREE, OCCUPIED
from nav import raycast


def test_single_ray_marks_free_and_occupied():
    cfg = Config()
    # 居中地图：原点放在载具附近
    m = OccupancyMap(cfg.nx, cfg.ny, cfg.nz, cfg.res,
                     (-15.0, -15.0, -5.0))
    vehicle = np.array([0.0, 0.0, 0.0])
    points = np.array([[2.0, 0.0, 0.0]])  # 前方 2m 命中
    n = raycast.integrate(m, vehicle, points, cfg)
    assert n == 1
    v_hit = m.world_to_voxel([2.0, 0.0, 0.0])
    assert m.is_occupied(*v_hit), "命中点应为占据"
    v_mid = m.world_to_voxel([1.0, 0.0, 0.0])
    assert m.state_at_voxel(*v_mid) == FREE, "中间点应为自由"


def test_empty_points_noop():
    cfg = Config()
    m = OccupancyMap(cfg.nx, cfg.ny, cfg.nz, cfg.res, (-15.0, -15.0, -5.0))
    n = raycast.integrate(m, np.array([0.0, 0.0, 0.0]),
                          np.zeros((0, 3)), cfg)
    assert n == 0
    assert m.known_ratio() == 0.0


if __name__ == "__main__":
    test_single_ray_marks_free_and_occupied()
    test_empty_points_noop()
    print("test_raycast: OK (using fast=%s)" % raycast.USING_FAST)
