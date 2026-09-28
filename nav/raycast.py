# -*- coding: utf-8 -*-
"""点云 → 3D 占据图（3D DDA 射线投射，方案书 §13–§14）。

优先调用 C++ 扩展 ``_fast.raycast_batch``；扩展不可用时退回纯 numpy 采样实现
（功能一致、较慢）。两种路径在模块导入时自动探测，``USING_FAST`` 暴露当前路径。
"""
from __future__ import annotations

import numpy as np

try:
    from . import _fast  # type: ignore
    USING_FAST = True
except Exception:
    _fast = None
    USING_FAST = False


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
    pts = pts[(dist >= min_d) & (dist <= cfg.max_range)]
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
            float(cfg.res),
            float(cfg.max_range),
        )
    else:
        _raycast_numpy(map, origins, hits, cfg.res)
    return n


def _raycast_numpy(map, origins, hits, res) -> None:
    """纯 numpy 回退：沿每条射线等距采样，标记自由/占据。"""
    for k in range(hits.shape[0]):
        o = origins[k]
        seg = hits[k] - o
        length = float(np.linalg.norm(seg))
        if length < 1e-6:
            continue
        n_steps = max(2, int(np.ceil(length / (res * 0.5))) + 1)
        ts = np.linspace(0.0, 1.0, n_steps)
        pts = o[None, :] + ts[:, None] * seg[None, :]
        v = np.floor((pts - map.origin) / res).astype("int64")

        # 去相邻重复体素
        keep = np.ones(v.shape[0], dtype=bool)
        keep[1:] = np.any(v[1:] != v[:-1], axis=1)
        v = v[keep]

        if v.shape[0] == 0:
            continue
        # 沿途（不含起点）→ 自由；终点 → 占据
        for r in range(1, v.shape[0] - 1):
            map.mark_free(int(v[r, 0]), int(v[r, 1]), int(v[r, 2]))
        if v.shape[0] >= 2:
            map.mark_occupied(int(v[-1, 0]), int(v[-1, 1]), int(v[-1, 2]))
