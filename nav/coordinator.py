# -*- coding: utf-8 -*-
"""Unified Control Coordinator —— 唯一控制出口（方案书 §41–§46）。

架构约束（Single Writer，§42）：**全程序只有本类调用 ``move*``**。规划器只产出
``p_d,v_d,a_d``，速度限制/安全约束/紧急制动都只通过本类投影后统一下发，杜绝多写者抖动。

- PD 轨迹跟踪：``u = Kp(p_d−p) + Kd(v_d−v) + v_d``（前馈）。
- 安全投影（§44）：速度上限 ``v_max`` + 加速度连续变化 ``a_max·dt``（§46 无跳变）。
- 紧急制动（§45）：``Emergency Monitor`` 只置标志/净空，本类据此发零速，仍单一出口。
"""
from __future__ import annotations

import time

import numpy as np


class UnifiedControlCoordinator:
    def __init__(self, io, cfg):
        self.io = io
        self.cfg = cfg
        self.traj = None
        self.t0 = 0.0
        self._prev_u = np.zeros(3, dtype="float64")
        self.emergency = False
        self.min_clearance = cfg.d_max
        self._last_tick = None

    # ---- 输入（其它模块只喂数据，不碰执行） ----
    def set_trajectory(self, traj, now: float):
        self.traj = traj
        self.t0 = now

    def set_min_clearance(self, d: float):
        self.min_clearance = d

    def set_emergency(self, flag: bool):
        self.emergency = flag

    # ---- 唯一写出口 ----
    def tick(self, state, now: float) -> dict:
        """每个控制周期调用一次，返回诊断信息，并（唯一地）下发速度指令。"""
        cfg = self.cfg
        z_des = float(state.position[2])  # 默认保持当前高度
        if self.traj is None:
            u = np.zeros(3)
        else:
            t = now - self.t0
            tt = min(max(t, 0.0), self.traj.total_time)
            p_d, v_d, a_d = self.traj.eval(tt)
            z_des = float(p_d[2])
            u = (cfg.kp * (p_d - state.position)
                 + cfg.kd * (v_d - state.velocity)
                 + v_d)

        # 只保留显式 EMERGENCY 零速（紧急监视器置位时）。不再按"净空 < d_emergency"自动急刹：
        # 避障已由 A* 距离代价 + B-spline 软/硬约束负责，自动急刹是与轨迹跟踪对抗的
        # "第二控制器"，会在阈值附近产生 limit-cycle 振荡（速度反复在 0↔巡航之间跳变），
        # 违背 §41 Single Writer。净空仍通过 telemetry 上报供观测。
        braking = bool(self.emergency)
        if braking:
            u = np.zeros(3)

        # 安全投影（§44）
        u = self._project(u)

        # 单一写者下发（fire-and-forget，下一 tick 覆盖）
        # Drone Racing Lab 风格：水平速度 + 锁定高度（AltZ 控制器），ForwardOnly
        self.io.move_by_velocity_z(float(u[0]), float(u[1]), z_des,
                                   float(cfg.velocity_duration))

        return {
            "u": u.tolist(),
            "braking": braking,
            "clearance": float(self.min_clearance),
        }

    def _project(self, u: np.ndarray) -> np.ndarray:
        cfg = self.cfg
        u = np.asarray(u, dtype="float64")
        # 速度上限
        sp = float(np.linalg.norm(u))
        if sp > cfg.v_max:
            u = u * (cfg.v_max / sp)
        # 加速度连续变化（§46）：用**实际 tick 间隔**（而非名义 control_dt），
        # 这样即使控制循环因 RPC 争用变慢，速度指令仍按 a_max 速率平滑爬升。
        now = time.time()
        dt = (now - self._last_tick) if self._last_tick is not None else cfg.control_dt
        self._last_tick = now
        dt = max(dt, 1e-3)
        du = u - self._prev_u
        du_norm = float(np.linalg.norm(du))
        max_du = cfg.a_max * dt
        if du_norm > max_du:
            du = du * (max_du / du_norm)
        u = self._prev_u + du
        self._prev_u = u
        return u
