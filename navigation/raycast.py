# -*- coding: utf-8 -*-
"""点云 → 3D 占据图（3D DDA 射线投射，方案书 §13–§14）。

优先调用 C++ 扩展 ``_fast.raycast_batch``；扩展不可用时退回纯 numpy 采样实现
（功能一致、较慢）。两种路径在模块导入时自动探测，``USING_FAST`` 暴露当前路径。
"""
from __future__ import annotations

import numpy as np

from .backend import native as _fast
USING_FAST = _fast is not None


def integrate(map, vehicle_pos, points, cfg) -> int:
    """把一帧 LiDAR 点云融合进 ``map``（原地修改）。

    ``points`` 为世界系（NED）的 (N,3) 点；``vehicle_pos`` 为载具世界系位置。
    返回实际投射的射线数（去重后）。
    """
    pts = np.asarray(points, dtype="float64")
    if pts.ndim != 2 or pts.shape[0] == 0:
        return 0

    # §54：限制最大感知距离 + 最小距离（滤掉全包围 FOV 扫到载具自身的自回波）+ voxel 降采样
    d = pts - np.asarray(vehicle_pos, dtype="float64")
    dist = np.linalg.norm(d, axis=1)
    min_d = float(getattr(cfg, "self_echo_dist", 0.3))
    pts = pts[np.isfinite(pts).all(axis=1) & (dist >= min_d) & (dist <= cfg.max_range)]
    if pts.shape[0] == 0:
        return 0

    v = np.floor((pts - map.origin) / cfg.res).astype("int64")
    _, idx = np.unique(v, axis=0, return_index=True)
    hits = pts[np.sort(idx)]

    n = hits.shape[0]
    origins = np.tile(np.asarray(vehicle_pos, dtype="float64"), (n, 1))
    if USING_FAST:
        _fast.raycast_batch(
            np.ascontiguousarray(origins),
            np.ascontiguousarray(hits),
            map.data,
            np.ascontiguousarray(map.origin.astype("float64")),
            float(map.res),
            float(cfg.max_range),
        )
    else:
        _raycast_numpy(map, origins, hits, map.res)
    return n


def dda_cells(start, end):
    """Exact DDA, including endpoint; tie ordering matches the native kernel."""
    cell = np.floor(start).astype(int)
    last = np.floor(end).astype(int)
    delta = end-start
    step = np.where(delta > 0, 1, -1)
    inv = np.divide(1., np.abs(delta), out=np.full(3, 1e30), where=np.abs(delta)>1e-12)
    crossing = np.where(step > 0, np.floor(start)+1-start, start-np.floor(start))*inv
    crossing[np.abs(delta)<=1e-12] = 1e30
    yield tuple(cell)
    for _ in range(int(np.abs(last-cell).sum())+3):
        if np.array_equal(cell,last): break
        x,y,z=crossing
        a = (0 if x < z else 2) if x < y else (1 if y < z else 2)
        cell[a] += step[a]; crossing[a] += inv[a]
        yield tuple(cell)


def _raycast_numpy(map, origins, hits, res):
    free, occupied = set(), set()
    for o,h in zip(origins,hits):
        cells=list(dda_cells((o-map.origin)/res, (h-map.origin)/res))
        if len(cells)<2: continue
        free.update(c for c in cells[:-1] if map.in_bounds(*c))
        if map.in_bounds(*cells[-1]): occupied.add(cells[-1])
    for c in free: map.data[c]=1
    for c in occupied: map.data[c]=2
