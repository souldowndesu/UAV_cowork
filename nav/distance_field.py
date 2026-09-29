# -*- coding: utf-8 -*-
"""截断欧氏距离场（方案书 §18–§20）。

``D(x) = min(d(x, Occupied), D_max)``，仅 Occupied 参与，Unknown 不参与（§19）。

优先用 scipy（C 后端优化专有库）的 ``distance_transform_edt`` 得到精确欧氏距离；
scipy 缺失时退回多源 BFS 的近似 chamfer 距离（26 连通、以体素边长加权、截断到
D_max）。两种路径自动探测，``USING_SCIPY`` 暴露当前路径。
"""
from __future__ import annotations

import numpy as np

try:
    from scipy import ndimage as _ndi
    USING_SCIPY = True
except Exception:
    _ndi = None
    USING_SCIPY = False


def compute(map, cfg) -> np.ndarray:
    """返回与 ``map.data`` 同形的 float32 截断距离场（米）。按 ``map.res`` 采样。"""
    occupied = map.data == 2  # OCCUPIED
    res = map.res
    if not occupied.any():
        # 无任何障碍时 distance_transform_edt(~occupied) 会对"全 foreground"返回到数组
        # 边界的伪距离（把 (0,0,0) 角当唯一 background），导致边界附近 d<d_min 被 A*
        # 误判为不可通行、且距离场在开阔地无意义。此时直接返回全 d_max（处处净空最大）。
        return np.full(map.data.shape, cfg.d_max, dtype="float32")
    if USING_SCIPY:
        # distance_transform_edt：非零体素到最近零体素（=占据）的距离
        d = _ndi.distance_transform_edt(
            ~occupied, sampling=(res, res, res)
        ).astype("float32")
    else:
        d = _bfs_distance(occupied, res, cfg.d_max)
    np.minimum(d, cfg.d_max, out=d)
    return d


def _bfs_distance(occupied: np.ndarray, res: float, d_max: float) -> np.ndarray:
    """多源 BFS 近似截断距离场（chamfer，26 连通，权重=体素边长）。"""
    nx, ny, nz = occupied.shape
    dist = np.full((nx, ny, nz), d_max, dtype="float32")
    # 初始源：所有占据体素（axis0=x, axis1=y, axis2=z）
    ix, iy, iz = np.nonzero(occupied)
    dist[ix, iy, iz] = 0.0
    # 用队列做多源 BFS（26 连通）
    from collections import deque
    q = deque(zip(ix.tolist(), iy.tolist(), iz.tolist()))
    neis = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
            for dz in (-1, 0, 1) if not (dx == 0 and dy == 0 and dz == 0)]
    while q:
        x, y, z = q.popleft()
        base = float(dist[x, y, z])
        if base + res > d_max:
            continue
        for dx, dy, dz in neis:
            nx_, ny_, nz_ = x + dx, y + dy, z + dz
            if not (0 <= nx_ < nx and 0 <= ny_ < ny and 0 <= nz_ < nz):
                continue
            nd = base + res * np.sqrt(dx * dx + dy * dy + dz * dz)
            if nd < float(dist[nx_, ny_, nz_]) - 1e-6:
                dist[nx_, ny_, nz_] = nd
                q.append((nx_, ny_, nz_))
    return dist
