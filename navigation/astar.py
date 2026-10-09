# -*- coding: utf-8 -*-
"""3D 26 连通 A*（方案书 §24–§27）。

- 代价场（§21）：Occupied=∞（不扩展）、Unknown=c_unknown（高但有限）、
  Free=c_free + w_obs·φ(D)，其中 φ(D)=(1-D/d_safe)²（D<d_safe 时，§20）。
- 路径代价按长度累积（§23）：边代价 = C(邻居体素)·边长（欧氏）。
- 启发 h = c_free · 欧氏距离（可采纳，c_free 为每米最小代价）。
- 只在 Local Map 窗口内搜索（§56 边界）；带展开节点上限保护。
"""
from __future__ import annotations

import heapq
import math
from typing import List, Optional

import numpy as np

from .backend import native as _fast
USING_FAST = _fast is not None

_NEIGHBORS = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
              for dz in (-1, 0, 1) if not (dx == 0 and dy == 0 and dz == 0)]
_NEIGHBOR_LEN = [float(np.linalg.norm(o)) for o in _NEIGHBORS]

INF = float("inf")


def _phi(d: float, d_safe: float) -> float:
    # 指数距离惩罚：d_safe 处为 0，随接近障碍指数上升。
    # d_safe=5 时：5m→0、4m→1.72、3m→6.39、2m→19.1、1m→53.6。
    # 比二次 (1-d/d_safe)² 陡得多，让 A* 强烈远离建筑（5m 安全距离）。
    if d >= d_safe:
        return 0.0
    return math.exp(d_safe - d) - 1.0


class AStarPlanner:
    def __init__(self, cfg):
        self.cfg = cfg
        self.c_free = float(cfg.c_free)
        self.c_unknown = float(cfg.c_unknown)
        self.w_obs = float(cfg.w_obs)
        self.d_safe = float(cfg.d_safe)
        self.d_min = float(getattr(cfg, "d_min", 0.3))
        self.res = float(cfg.res)
        self.max_nodes = int(cfg.astar_timeout_nodes)
        # 启发式 h = h_weight * 欧氏距离。h_weight 必须 ≤ c_free 才可采纳/一致，
        # 让障碍距离惩罚（w_obs·φ(D)）真正影响路径选择；>1 是加权 A*（贪心，
        # 会贴墙走最短路径、忽略距离惩罚）。曾设 5.0 导致贴墙，已改回 1.0。
        self.h_weight = float(getattr(cfg, "h_weight", 1.0))

    def plan(self, map, dist, start, goal,
             v_dir=None, w_dir=0.0) -> Optional[List[np.ndarray]]:
        """返回世界系航点列表（体素中心，含终点、不含起点），无路返回 None。

        ``map`` 为 OccupancyMap，``dist`` 为截断距离场（float32，米）。
        边长/启发式按 ``map.res`` 计算（支持粗网格规划）。

        ``v_dir``：当前速度方向（世界系 3-向量，内部归一化）。给定且 ``w_dir>0`` 时，
        起点节点的每个邻居方向与 ``v_dir`` 夹角越大、边代价附加 ``w_dir*(1-cosθ)*res``，
        让 A* 起步沿当前速度方向延伸而非垂直于速度起步（消除大幅摆动/脱离路线）。
        ``v_dir=None`` 或速度接近零时该引导自动失效（退回普通 A*）。
        """
        nx, ny, nz = map.nx, map.ny, map.nz
        res = map.res
        occ = map.data

        # 速度方向引导：归一化；零向量/未提供则禁用
        vd = None
        if v_dir is not None and w_dir > 0.0:
            vd = np.asarray(v_dir, dtype="float64").reshape(3)
            vn = float(np.linalg.norm(vd))
            if vn >= 1e-6:
                vd = vd / vn
            else:
                vd = None

        # 回退 start/goal 到最近净空体素：无人机贴墙时自身体素可能被膨胀占据
        # （或净空 < d_min），以阻塞体素为起点会无路径。goal 原已回退；此处把
        # start 也一并回退，并移到 C++ 快速路径之前，让两条路径都受益。
        start_v = map.world_to_voxel(start)
        goal_v = map.world_to_voxel(goal)
        if not map.in_bounds(*start_v):
            return None
        if self._blocked(map, dist, start_v):
            return None  # Never teleport the search start through an obstacle.
        if self._blocked(map, dist, goal_v):
            goal_v = self._nearest_clear(map, dist, goal_v)
            if goal_v is None:
                return None
            goal = map.voxel_to_world(*goal_v)

        if USING_FAST:
            varr = np.zeros(3, dtype="float64") if vd is None else vd
            r = _fast.astar_plan(
                np.ascontiguousarray(map.data, dtype="int8"),
                np.ascontiguousarray(dist, dtype="float32"),
                np.ascontiguousarray(map.origin, dtype="float64"),
                float(map.res),
                np.ascontiguousarray(start, dtype="float64"),
                np.ascontiguousarray(goal, dtype="float64"),
                float(self.c_free), float(self.c_unknown), float(self.w_obs),
                float(self.d_safe), float(self.d_min), float(self.h_weight),
                int(self.max_nodes),
                float(w_dir),
                np.ascontiguousarray(varr, dtype="float64"))
            if r.shape[0] > 0:
                return [r[i].copy() for i in range(r.shape[0])]
            return None

        start_flat = self._flat(start_v, ny, nz)
        goal_flat = self._flat(goal_v, ny, nz)

        g = np.full(nx * ny * nz, INF, dtype="float64")
        came = np.full(nx * ny * nz, -1, dtype="int32")
        closed = np.zeros(nx * ny * nz, dtype=bool)

        g[start_flat] = 0.0
        h0 = self.h_weight * self._euclid(start_v, goal_v) * res
        heap = [(h0, 0.0, start_flat)]
        expanded = 0

        while heap:
            f, gs, cur = heapq.heappop(heap)
            if closed[cur]:
                continue
            if gs > g[cur]:
                continue
            closed[cur] = True
            expanded += 1
            if expanded > self.max_nodes:
                return None
            if cur == goal_flat:
                return self._reconstruct(map, came, start_flat, goal_flat)

            is_start = (cur == start_flat) and (vd is not None)
            ci, cj, ck = self._unflat(cur, ny, nz)
            for (dx, dy, dz), elen in zip(_NEIGHBORS, _NEIGHBOR_LEN):
                ni, nj, nk = ci + dx, cj + dy, ck + dz
                if not (0 <= ni < nx and 0 <= nj < ny and 0 <= nk < nz):
                    continue
                c = self._cost(occ, dist, ni, nj, nk)
                if c >= INF:
                    continue
                from .geometry import edge_clear
                if not edge_clear(occ, dist, (ci,cj,ck), (dx,dy,dz), self.d_min):
                    continue
                step = c * elen * res
                if is_start:
                    # 方向引导：偏离速度方向的起步额外惩罚（cos 同向=1→0 惩罚，
                    # 垂直=0→w_dir，反向=-1→2·w_dir，均乘以边长 res）
                    cosang = (dx * vd[0] + dy * vd[1] + dz * vd[2]) / elen
                    step += w_dir * (1.0 - cosang) * res
                ng = gs + step
                nflat = self._flat((ni, nj, nk), ny, nz)
                if ng < g[nflat]:
                    g[nflat] = ng
                    came[nflat] = cur
                    h = self.h_weight * self._euclid((ni, nj, nk), goal_v) * res
                    heapq.heappush(heap, (ng + h, ng, nflat))
        return None

    # ------------------------------------------------------------------
    def _cost(self, occ, dist, i, j, k) -> float:
        s = int(occ[i, j, k])
        if s == 2:  # OCCUPIED
            return INF
        # 净空硬截断：FREE 与 UNKNOWN 一视同仁 —— 距膨胀障碍 < d_min 的 UNKNOWN
        # 紧贴墙/墙后不可达区，同样不可通行，防止 A* 穿过未观测的墙。
        d = float(dist[i, j, k])
        if d < self.d_min:
            return INF  # 硬净空（机器人半径膨胀）
        if s == 0:  # UNKNOWN：基础代价更高 + 同样的障碍距离惩罚
            return self.c_unknown + self.w_obs * _phi(d, self.d_safe)
        # FREE：基础代价 + 障碍距离惩罚
        return self.c_free + self.w_obs * _phi(d, self.d_safe)

    def _blocked(self, map, dist, v) -> bool:
        i, j, k = v
        if not map.in_bounds(i, j, k):
            return True
        return self._cost(map.data, dist, i, j, k) >= INF

    def _nearest_clear(self, map, dist, goal_v, radius=6):
        gi, gj, gk = goal_v
        best = None
        best_d2 = INF
        for i in range(gi - radius, gi + radius + 1):
            for j in range(gj - radius, gj + radius + 1):
                for k in range(gk - radius, gk + radius + 1):
                    if not map.in_bounds(i, j, k):
                        continue
                    if self._blocked(map, dist, (i, j, k)):
                        continue
                    d2 = (i - gi) ** 2 + (j - gj) ** 2 + (k - gk) ** 2
                    if d2 < best_d2:
                        best_d2 = d2
                        best = (i, j, k)
        return best

    def _reconstruct(self, map, came, start_flat, goal_flat):
        path = []
        cur = goal_flat
        while cur != start_flat and cur != -1:
            path.append(cur)
            cur = int(came[cur])
        if cur == -1:
            return None
        path.reverse()
        ny, nz = map.ny, map.nz
        waypoints = []
        for f in path:
            i, j, k = self._unflat(f, ny, nz)
            waypoints.append(map.voxel_to_world(i, j, k))
        return waypoints

    @staticmethod
    def _flat(v, ny, nz):
        i, j, k = v
        return (i * ny + j) * nz + k

    @staticmethod
    def _unflat(f, ny, nz):
        k = f % nz
        j = (f // nz) % ny
        i = f // (ny * nz)
        return i, j, k

    @staticmethod
    def _euclid(a, b):
        return float(np.sqrt(sum((x - y) ** 2 for x, y in zip(a, b))))
