# -*- coding: utf-8 -*-
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from config import Config
from nav.coordinator import UnifiedControlCoordinator
from nav.airsim_io import VehicleState
from nav.bspline import BSplineTrajectory


class _MockIO:
    """记录唯一写者下发的指令，验证 Single Writer 不变量。"""
    def __init__(self):
        self.commands = []

    def move_by_velocity_z(self, vx, vy, z, duration):
        self.commands.append((vx, vy, z, duration))
        return None


def _state(x=0.0, v=0.0):
    return VehicleState(position=np.array([x, 0.0, -5.0]),
                        velocity=np.array([v, 0.0, 0.0]),
                        acceleration=np.zeros(3), yaw=0.0)


def test_speed_projection():
    cfg = Config()
    cfg.kp = 2.0
    cfg.kd = 1.0
    cfg.v_max = 3.0
    cfg.a_max = 2.0
    cfg.control_dt = 0.02
    io = _MockIO()
    c = UnifiedControlCoordinator(io, cfg)
    # 直接喂一个静止轨迹（目标在前方远处）
    ctrl = np.tile(np.array([100.0, 0.0, -5.0]), (6, 1))
    c.set_trajectory(BSplineTrajectory(ctrl, 0.5), 0.0)
    c.set_min_clearance(10.0)
    for _ in range(50):
        c.tick(_state(), 0.0)
    vx = [cmd[0] for cmd in io.commands]
    assert max(vx) <= cfg.v_max + 1e-6, "速度不应超过 v_max"


def test_emergency_brake():
    cfg = Config()
    io = _MockIO()
    c = UnifiedControlCoordinator(io, cfg)
    ctrl = np.tile(np.array([100.0, 0.0, -5.0]), (6, 1))
    c.set_trajectory(BSplineTrajectory(ctrl, 0.5), 0.0)
    # 低净空不再触发自动急刹（避障由 A*+B-spline 负责），追踪仍下发非零速度
    c.set_min_clearance(0.1)
    c.tick(_state(), 0.0)
    vx, vy, _, _ = io.commands[-1]
    assert (vx, vy) != (0.0, 0.0), "低净空不应触发自动急刹"
    # 显式 EMERGENCY 标志才零速
    c2 = UnifiedControlCoordinator(io, cfg)
    c2.set_trajectory(BSplineTrajectory(ctrl, 0.5), 0.0)
    c2.set_emergency(True)
    c2.tick(_state(), 0.0)
    vx2, vy2, _, _ = io.commands[-1]
    assert (vx2, vy2) == (0.0, 0.0), "显式 EMERGENCY 应发零速"


def test_accel_continuity():
    cfg = Config()
    cfg.v_max = 5.0
    cfg.a_max = 1.0
    cfg.control_dt = 0.02
    io = _MockIO()
    c = UnifiedControlCoordinator(io, cfg)
    ctrl = np.tile(np.array([100.0, 0.0, -5.0]), (6, 1))
    c.set_trajectory(BSplineTrajectory(ctrl, 0.5), 0.0)
    c.set_min_clearance(10.0)
    prev = np.array([0.0, 0.0])
    for _ in range(30):
        c.tick(_state(), 0.0)
        u = np.array(io.commands[-1][:2])
        assert np.linalg.norm(u - prev) <= cfg.a_max * cfg.control_dt + 1e-6
        prev = u


if __name__ == "__main__":
    test_speed_projection()
    test_emergency_brake()
    test_accel_continuity()
    print("test_coordinator: OK")
