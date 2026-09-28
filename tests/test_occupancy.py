# -*- coding: utf-8 -*-
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from nav.occupancy import OccupancyMap, UNKNOWN, FREE, OCCUPIED


def test_world_voxel_roundtrip():
    m = OccupancyMap(50, 50, 50, 0.2, (0.0, 0.0, 0.0))
    p = np.array([1.05, 2.15, 3.25])
    v = m.world_to_voxel(p)
    c = m.voxel_to_world(*v)
    assert np.all(np.abs(c - p) <= 0.1 + 1e-9)


def test_mark_and_query():
    m = OccupancyMap(20, 20, 20, 0.2, (0.0, 0.0, 0.0))
    m.mark_occupied(5, 5, 5)
    assert m.is_occupied(5, 5, 5)
    assert m.state_at_voxel(6, 6, 6) == UNKNOWN
    m.mark_free(6, 6, 6)
    assert m.is_free(6, 6, 6)


def test_recenter_preserves_world_content():
    m = OccupancyMap(30, 30, 30, 0.2, (0.0, 0.0, 0.0))
    wp = m.voxel_to_world(12, 12, 12)  # 记录世界系点
    m.mark_occupied(12, 12, 12)
    m.recenter(np.array([2.0, 1.0, 0.0]))  # 平移 10/5 个体素
    v = m.world_to_voxel(wp)
    assert m.is_occupied(*v), "recenter 后世界内容应保持不变"


def test_out_of_bounds_is_occupied():
    m = OccupancyMap(10, 10, 10, 0.2, (0.0, 0.0, 0.0))
    assert m.state_at_voxel(100, 100, 100) == OCCUPIED


if __name__ == "__main__":
    test_world_voxel_roundtrip()
    test_mark_and_query()
    test_recenter_preserves_world_content()
    test_out_of_bounds_is_occupied()
    print("test_occupancy: OK")
