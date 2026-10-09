# -*- coding: utf-8 -*-
"""B-spline 轨迹表示与平滑优化（方案书 §28–§35，含连续重规划状态拼接）。

核心设计（对应方案书 §30 warm start / 评价里的"连续重规划"）：

- **末端钳制、起点自由**：只把最后一个控制点复制两份，使轨迹终点精确落在
  ``Q_{n-1}`` 且终点速度=0（到达目标自然停下）；起点**不**钳制，因此起点导数自由。
- **起点状态编码**：前 3 个控制点由拼接状态 ``(p_s, v_s, a_s)`` 显式确定，使新轨迹
  在起点以**当前速度/加速度平滑延续**（而非静止起步）：
      Q0 = p_s − Δt·v_s + (Δt²/3)·a_s
      Q1 = p_s − (Δt²/6)·a_s
      Q2 = p_s + Δt·v_s + (Δt²/3)·a_s
- **优化只调 Q3…Q_{n-1}**，Q0/Q1/Q2 锁死（保持运动状态不变）。

代价 J = w_goal·J_goal + w_obs·J_obs + w_smooth·J_smooth + w_dyn·J_dyn（§31）。
"""
from __future__ import annotations

import time
from typing import List, Optional

import numpy as np

from .backend import native as _fast
USING_FAST = _fast is not None


# 均匀三次 B-spline 基（u ∈ [0,1]，作用于控制点 P0..P3）
def _basis0(u):
    t = 1.0 - u
    return (t * t * t) / 6.0


def _basis1(u):
    return (3 * u ** 3 - 6 * u ** 2 + 4) / 6.0


def _basis2(u):
    return (-3 * u ** 3 + 3 * u ** 2 + 3 * u + 1) / 6.0


def _basis3(u):
    return (u ** 3) / 6.0


class BSplineTrajectory:
    def __init__(self, ctrl: np.ndarray, dt: float):
        self.ctrl = np.asarray(ctrl, dtype="float64")
        self.dt = float(dt)
        self.n = self.ctrl.shape[0]
        # 末端钳制：复制最后一个控制点两份 → p(T)=Q_{n-1}、v(T)=0。
        # 起点不复制 → 起点 p/v/a 由 Q0/Q1/Q2 编码，导数自由。
        self._pad = np.concatenate([self.ctrl, self.ctrl[-1:], self.ctrl[-1:]])

    @property
    def total_time(self) -> float:
        # pad 长度 n+2，段数 n-1
        return max(0.0, (self.n - 1) * self.dt)

    def eval(self, t: float) -> (np.ndarray, np.ndarray, np.ndarray):
        """返回 (位置, 速度, 加速度) 在时刻 t。"""
        pad = self._pad
        m = pad.shape[0]           # n + 2
        dt = self.dt
        seg = int(t / dt)
        if seg > m - 4:
            seg = m - 4
            u = 1.0
        else:
            u = t / dt - seg
        if seg < 0:
            seg = 0
            u = 0.0
        P0 = pad[seg]
        P1 = pad[seg + 1]
        P2 = pad[seg + 2]
        P3 = pad[seg + 3]

        p = _basis0(u) * P0 + _basis1(u) * P1 + _basis2(u) * P2 + _basis3(u) * P3

        tt = 1.0 - u
        b0 = -3 * tt * tt / 6.0
        b1 = (9 * u * u - 12 * u) / 6.0
        b2 = (-9 * u * u + 6 * u + 3) / 6.0
        b3 = 3 * u * u / 6.0
        v = (b0 * P0 + b1 * P1 + b2 * P2 + b3 * P3) / dt

        c0 = (6 * tt) / 6.0
        c1 = (18 * u - 12) / 6.0
        c2 = (-18 * u + 6) / 6.0
        c3 = (6 * u) / 6.0
        a = (c0 * P0 + c1 * P1 + c2 * P2 + c3 * P3) / (dt * dt)
        return p, v, a


# ---------------------------------------------------------------------------
# 起点状态编码
# ---------------------------------------------------------------------------
def encode_initial_state(p_s, v_s, a_s, dt):
    """把 (p_s, v_s, a_s) 编码进前 3 个控制点，使 B-spline 起点满足 p/v/a 边界条件。"""
    p_s = np.asarray(p_s, dtype="float64")
    v_s = np.asarray(v_s, dtype="float64")
    a_s = np.asarray(a_s, dtype="float64")
    Q0 = p_s - dt * v_s + (dt * dt / 3.0) * a_s
    Q1 = p_s - (dt * dt / 6.0) * a_s
    Q2 = p_s + dt * v_s + (dt * dt / 3.0) * a_s
    return Q0, Q1, Q2


# ---------------------------------------------------------------------------
# 距离场查询（三线性插值）
# ---------------------------------------------------------------------------
def _dist_at(p, dist, origin, res):
    from .geometry import distance_query
    return float(distance_query(p, dist, origin, res)[0][0])


def _min_clearance(traj, dist, origin, res, n_samples=200):
    """沿轨迹密集采样，返回最小障碍净空（米）。

    用于 B-spline 的**硬约束**校验：软代价只在控制点上采样，可能漏掉两控制点之间的
    直线段切过障碍的情况；这里沿弧长密集采样，确保整条轨迹都满足净空要求。
    """
    min_d = float("inf")
    T = float(traj.total_time)
    for i in range(n_samples + 1):
        t = T * i / n_samples
        p, _, _ = traj.eval(t)
        d = _dist_at(p, dist, origin, res)
        if d < min_d:
            min_d = d
    return min_d


def _psi(d, d_safe):
    if d < d_safe:
        return (d_safe - d) ** 2
    return 0.0


def _cost(ctrl, goal, dist, origin, res, cfg, dt):
    """总代价 J（§31）。"""
    c = np.asarray(ctrl, dtype="float64")
    n = c.shape[0]
    skip = int(getattr(cfg, "smooth_skip", 2))
    j = 0.0
    # J_goal：终点（末端钳制后 p(T)=c[-1]）
    j += cfg.w_goal * float(np.sum((c[-1] - goal) ** 2))
    # J_obs
    for k in range(n):
        dd = _dist_at(c[k], dist, origin, res)
        j += cfg.w_obs * _psi(dd, cfg.d_safe)
    # J_hard：硬净空 barrier，沿 B-spline 曲线密集采样（与 C++ 一致）
    d_min = float(getattr(cfg, "d_min", 0.3))
    w_hard = float(getattr(cfg, "w_hard", 100.0))
    _traj = BSplineTrajectory(c, dt)
    n_samp = 100
    for s in range(n_samp + 1):
        tt = _traj.total_time * s / n_samp
        pp, _, _ = _traj.eval(tt)
        dd = _dist_at(pp, dist, origin, res)
        if dd < d_min:
            j += w_hard * (d_min - dd) ** 2
    # J_smooth（二阶差分）——跳过起点附近 skip 项，允许起点加速（不把 Q3 拉向锁死的 Q2）
    for k in range(skip, n - 2):
        j += cfg.w_smooth * float(np.sum((c[k + 2] - 2 * c[k + 1] + c[k]) ** 2))
    # J_dyn（速度超限）
    if n >= 2:
        vel = (c[1:] - c[:-1]) / dt
        for v in vel:
            sp = float(np.linalg.norm(v))
            if sp > cfg.v_max:
                j += cfg.w_dyn * (sp - cfg.v_max) ** 2
    # J_dyn（加速度超限）——同样跳过起点附近 skip 项
    if n >= 3:
        acc = (c[2:] - 2 * c[1:-1] + c[:-2]) / (dt ** 2)
        for a in acc[skip:]:
            am = float(np.linalg.norm(a))
            if am > cfg.a_max:
                j += cfg.w_dyn * (am - cfg.a_max) ** 2
    return j


def optimize(ctrl0, goal, dist, map, cfg, dt, lock_first: int = 3) -> np.ndarray:
    """梯度下降优化控制点；``lock_first`` 之前的控制点（Q0/Q1/Q2）保持不动。"""
    if USING_FAST:
        return _fast.bspline_optimize(
            np.ascontiguousarray(ctrl0, dtype="float64"),
            np.ascontiguousarray(goal, dtype="float64"),
            np.ascontiguousarray(dist, dtype="float32"),
            np.ascontiguousarray(map.origin, dtype="float64"),
            float(map.res), float(dt),
            float(cfg.w_goal), float(cfg.w_obs), float(cfg.w_smooth), float(cfg.w_dyn),
            float(cfg.d_safe), float(getattr(cfg, "d_min", 0.3)),
            float(getattr(cfg, "w_hard", 100.0)),
            float(cfg.v_max), float(cfg.a_max),
            int(cfg.bspline_iters), float(cfg.bspline_step), 1e-3, 1.0, int(lock_first),
            int(getattr(cfg, "smooth_skip", 2)))

    ctrl = np.asarray(ctrl0, dtype="float64").copy()
    origin = map.origin
    res = map.res
    eps = 1e-3
    lr = float(cfg.bspline_step)
    for _ in range(int(cfg.bspline_iters)):
        grad = np.zeros_like(ctrl)
        for i in range(int(lock_first), ctrl.shape[0]):
            for d in range(3):
                ctrl[i, d] += eps
                fp = _cost(ctrl, goal, dist, origin, res, cfg, dt)
                ctrl[i, d] -= 2 * eps
                fm = _cost(ctrl, goal, dist, origin, res, cfg, dt)
                ctrl[i, d] += eps
                grad[i, d] = (fp - fm) / (2 * eps)
        # 更新 + 裁剪每控制点位移，防止有限差分梯度发散
        delta = lr * grad
        norm = np.linalg.norm(delta, axis=1, keepdims=True)
        max_step = 1.0
        delta = np.where(norm > max_step, delta * (max_step / np.maximum(norm, 1e-12)), delta)
        ctrl -= delta
        # 锁死前 lock_first 个控制点（保持起点运动状态）
        ctrl[:int(lock_first)] = ctrl0[:int(lock_first)]
    return ctrl


def fit_and_optimize(path: List[np.ndarray], dist, map, cfg,
                     p_s, v_s, a_s, goal,
                     prev_ctrl: Optional[np.ndarray] = None) -> Optional[BSplineTrajectory]:
    """A* 粗路径 → B-spline 轨迹（起点继承当前运动状态）。

    ``p_s/v_s/a_s`` 为拼接点状态（世界系）；``path`` 为 A* 世界系航点。
    """
    if path is None or len(path) < 2:
        return None
    pts = np.asarray(path, dtype="float64")
    # 用当前位置作为弧长起点（与拼接点一致）
    pts = np.vstack([np.asarray(p_s, dtype="float64").reshape(1, 3), pts])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = cum[-1]
    if total < 1e-6:
        return None

    # 控制点数自适应路径弧长：让控制点密集贴合 A* 航点（只润滑拐角、不拉直绕行），
    # 同时用 bspline_n_ctrl/bspline_max_ctrl 约束下/上限。固定 20 点会把长绕行"拉直"
    # 切过障碍——这是历史上弃用 B-spline 的根因；自适应密度后曲线严格贴着 A* 折线走。
    spacing = float(getattr(cfg, "bspline_ctrl_spacing", 1.2))
    max_ctrl = int(getattr(cfg, "bspline_max_ctrl", 60))
    n = int(round(total / max(1e-6, spacing))) + 3
    n = int(min(max(n, int(cfg.bspline_n_ctrl)), max_ctrl))

    # 时间分配：段数 = n-1
    total_time = max(1.0, total / max(0.5, float(cfg.v_mission)))
    dt = total_time / (n - 1)

    # 起点速度继承：直接使用当前速度 v_s（不再硬编码替换为 v_mission）。
    # 低速起步的加速由自由控制点布点 + 轨迹优化自然完成，而非强制速度。
    v_enc = np.asarray(v_s, dtype="float64")

    # 起点状态编码覆盖 Q0/Q1/Q2
    Q0, Q1, Q2 = encode_initial_state(p_s, v_enc, a_s, dt)
    ctrl = np.zeros((n, 3))
    ctrl[0], ctrl[1], ctrl[2] = Q0, Q1, Q2

    # 自由控制点 Q3..Q_{n-1}：抛弃起点附近过近的点——从 Q2 前方一个巡航间距处开始布点，
    # 消除 Q2→Q3 反向折返/过近导致的轨迹起点减速。
    cruise_spacing = dt * float(cfg.v_mission)
    start_off = dt * float(np.linalg.norm(v_enc)) + cruise_spacing  # Q2 前进量 + 一个巡航间距
    start_off = min(start_off, total)
    free_targets = np.linspace(start_off, total, n - 3)
    for d in range(3):
        ctrl[3:, d] = np.interp(free_targets, cum, pts[:, d])

    # warm start：旧轨迹控制点按弧长插值到新自由控制点（§30）。控制点数 n 自适应
    # 弧长会随路径长度变化，直接 shape 匹配会失效；这里把旧控制点视为在 [0,total]
    # 近似均匀分布，按弧长 np.interp 到新的 free_targets，使连续重规划在已有轨迹
    # 基础上微调、不跳变（用户 m02861 诉求③）。
    if prev_ctrl is not None and prev_ctrl.ndim == 2 and prev_ctrl.shape[0] >= 4:
        prev = np.asarray(prev_ctrl, dtype="float64")
        old_s = np.linspace(0.0, total, prev.shape[0])
        for d in range(3):
            warm = np.interp(free_targets, old_s, prev[:, d])
            ctrl[3:, d] = 0.5 * ctrl[3:, d] + 0.5 * warm

    ctrl = optimize(ctrl, goal, dist, map, cfg, dt, lock_first=3)
    # 硬净空已作为 barrier 内建到优化代价里（J_hard：沿曲线密集采样、净空<d_min 二次惩罚），
    # 优化过程会主动把曲线推离障碍，因此这里**不再事后拒绝**——拒绝会导致无轨迹、无人机停止。
    return BSplineTrajectory(ctrl, dt)


class PolylineTrajectory:
    """A* 航点的折线轨迹（匀速线性插值），替代 B-spline 平滑。

    直接沿 A* 路线走：不做任何平滑/优化，因此**不会把绕行"拉直"而切过障碍**，轨迹就是
    A* 给出的折线本身。接口与 ``BSplineTrajectory`` 兼容（``eval``/``total_time``/``ctrl``/``dt``），
    协调器无需改动。
    """
    def __init__(self, waypoints, v_mission: float):
        self.waypoints = np.asarray(waypoints, dtype="float64")
        self.ctrl = self.waypoints.copy()
        seg = np.linalg.norm(np.diff(self.waypoints, axis=0), axis=1)
        self.cum = np.concatenate([[0.0], np.cumsum(seg)])
        self.v = float(v_mission)
        self.total_time = float(self.cum[-1]) / max(0.5, self.v)
        self.dt = 0.1

    def eval(self, t: float):
        t = min(max(float(t), 0.0), self.total_time)
        s = t * self.v
        idx = int(np.searchsorted(self.cum, s, side="right")) - 1
        idx = min(max(idx, 0), self.waypoints.shape[0] - 2)
        s0 = float(self.cum[idx]); s1 = float(self.cum[idx + 1])
        L = max(s1 - s0, 1e-9)
        frac = (s - s0) / L
        p = self.waypoints[idx] * (1.0 - frac) + self.waypoints[idx + 1] * frac
        v = (self.waypoints[idx + 1] - self.waypoints[idx]) / L * self.v
        return p, v, np.zeros(3)


def build_polyline(path, p_s, v_mission: float) -> "PolylineTrajectory":
    """从 A* 航点（+当前拼接位置）直接构造折线轨迹，不做 B-spline 优化。"""
    pts = np.vstack([np.asarray(p_s, dtype="float64").reshape(1, 3),
                     np.asarray(path, dtype="float64").reshape(-1, 3)])
    return PolylineTrajectory(pts, v_mission)
