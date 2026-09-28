# -*- coding: utf-8 -*-
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from config import Config
from nav.occupancy import OccupancyMap
from nav import bspline


def _map_cfg():
    cfg = Config()
    m = OccupancyMap(cfg.nx, cfg.ny, cfg.nz, cfg.res, (-15.0, -15.0, -5.0))
    dist = np.full((cfg.nx, cfg.ny, cfg.nz), cfg.d_max, dtype="float32")
    return cfg, m, dist


def test_encode_initial_state_roundtrip():
    """状态编码数学正确性：Q0/Q1/Q2 应精确恢复 (p,v,a)。"""
    dt = 0.5
    p_s = np.array([1.0, 2.0, -5.0])
    v_s = np.array([3.0, -1.0, 0.5])
    a_s = np.array([0.5, 0.0, -0.2])
    Q0, Q1, Q2 = bspline.encode_initial_state(p_s, v_s, a_s, dt)
    p0 = (Q0 + 4 * Q1 + Q2) / 6.0
    v0 = (Q2 - Q0) / (2 * dt)
    a0 = (Q0 - 2 * Q1 + Q2) / (dt * dt)
    assert np.allclose(p0, p_s, atol=1e-9)
    assert np.allclose(v0, v_s, atol=1e-9)
    assert np.allclose(a0, a_s, atol=1e-9)


def test_initial_velocity_preserved():
    """核心修复：非零初始速度应被新轨迹继承，v(0)=v_s 而非 0。"""
    cfg, m, dist = _map_cfg()
    path = [np.array([0.0, 0.0, -5.0]), np.array([4.0, 0.0, -5.0]),
            np.array([8.0, 0.0, -5.0])]
    p_s = np.array([0.0, 0.0, -5.0])
    v_s = np.array([3.0, 0.0, 0.0])
    a_s = np.array([0.0, 0.0, 0.0])
    goal = np.array([8.0, 0.0, -5.0])
    traj = bspline.fit_and_optimize(path, dist, m, cfg, p_s, v_s, a_s, goal)
    assert traj is not None
    _, v0, _ = traj.eval(0.0)
    assert np.linalg.norm(v0 - v_s) < 0.05, f"v(0)={v0} 应≈v_s={v_s}"


def test_rest_start_is_zero():
    """静止起步时 v(0)=0（首帧/悬停是合理的）。"""
    cfg, m, dist = _map_cfg()
    path = [np.array([0.0, 0.0, -5.0]), np.array([4.0, 0.0, -5.0]),
            np.array([8.0, 0.0, -5.0])]
    traj = bspline.fit_and_optimize(path, dist, m, cfg,
                                    np.array([0.0, 0.0, -5.0]),
                                    np.zeros(3), np.zeros(3),
                                    np.array([8.0, 0.0, -5.0]))
    assert traj is not None
    p0, v0, _ = traj.eval(0.0)
    assert np.linalg.norm(p0 - np.array([0.0, 0.0, -5.0])) < 0.3
    assert np.linalg.norm(v0) < 0.05


def test_endpoint_near_goal():
    cfg, m, dist = _map_cfg()
    path = [np.array([0.0, 0.0, -5.0]), np.array([2.0, 0.0, -5.0]),
            np.array([4.0, 0.0, -5.0]), np.array([6.0, 0.0, -5.0])]
    goal = np.array([6.0, 0.0, -5.0])
    traj = bspline.fit_and_optimize(path, dist, m, cfg,
                                    np.array([0.0, 0.0, -5.0]),
                                    np.zeros(3), np.zeros(3), goal)
    assert traj is not None
    p1, v1, _ = traj.eval(traj.total_time)
    assert np.linalg.norm(p1 - goal) < 1.0
    # 末端钳制：终点速度应≈0（到达自然停下）
    assert np.linalg.norm(v1) < 0.1


if __name__ == "__main__":
    test_encode_initial_state_roundtrip()
    test_initial_velocity_preserved()
    test_rest_start_is_zero()
    test_endpoint_near_goal()
    print("test_bspline: OK")
