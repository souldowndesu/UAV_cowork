# -*- coding: utf-8 -*-
"""PersistentMap 离线测试：两方向合并语义、坐标对齐、稀疏落盘。

覆盖用户四个重点：
① 三层存储结构；② Persistent 对感知空间只覆盖 Unknown、不改已感知区域；
③ 正确保存（坐标对齐、unknown 不落盘）；④ 扫描新数据时 unknown 不修改其它内容
   （现有逻辑用 unknown 覆盖 occupied 导致死循环 → 回归测试）。
"""
from __future__ import annotations

import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nav.persistent_map import PersistentMap
from nav.occupancy import UNKNOWN, FREE, OCCUPIED


def _local(n, res, origin=(0.0, 0.0, 0.0), fill=UNKNOWN):
    return np.full((n, n, n), fill, dtype="int8"), np.array(origin, dtype="float64"), res


def test_world_voxel_roundtrip():
    pm = PersistentMap(res=0.6, block=16, origin=(0.0, 0.0, 0.0))
    # 含负坐标（z 向下为负，对应飞行高度 20m）
    for p in [(0.0, 0.0, -20.0), (900.0, 20.0, -20.0), (-5.2, 3.9, 10.4), (0.0, 0.0, 0.0)]:
        i, j, k = pm.world_to_voxel(p)
        w = pm.voxel_to_world(i, j, k)
        assert abs(w[0] - p[0]) <= 0.5 * pm.res
        assert abs(w[1] - p[1]) <= 0.5 * pm.res
        assert abs(w[2] - p[2]) <= 0.5 * pm.res
    # 负索引的块归属：z=-20 → k=floor(-20/0.6)=-34，块 bz=floor(-34/16)=-3
    pm.blocks[(0, 0, -3)] = np.full((16, 16, 16), OCCUPIED, dtype="int8")
    i, j, k = pm.world_to_voxel((0.0, 0.0, -20.0))
    assert pm.query_voxel(i, j, k) == OCCUPIED


def test_res_multiple_validation():
    # 0.6 / 0.2 = 3 整数倍 → 不抛
    PersistentMap(res=0.6).validate_alignment(0.2)
    # 0.5 / 0.2 = 2.5 非整数倍 → 抛错
    try:
        PersistentMap(res=0.5).validate_alignment(0.2)
        raised = False
    except ValueError:
        raised = True
    assert raised, "persistent_res 非 local res 整数倍时必须抛错（否则半格错位）"


def test_merge_skips_unknown():
    """死循环回归：未知（新滚入条带被清零）不能抹掉已建模的 occupied。"""
    pm = PersistentMap(res=0.6, block=4, origin=(0.0, 0.0, 0.0))
    # 第一次：写入一个 OCCUPIED
    loc, org, res = _local(6, 0.2)          # 6*0.2 = 1.2m，覆盖持久体素 (0,0,0)..(1,1,1)
    loc[0, 0, 0] = OCCUPIED
    pm.merge_from_local(loc, org, res)
    assert pm.query_voxel(0, 0, 0) == OCCUPIED
    # 第二次：全 UNKNOWN 的扫描（滚动窗口清零后）合并 → OCCUPIED 必须保留
    loc2, org2, res2 = _local(6, 0.2)
    pm.merge_from_local(loc2, org2, res2)
    assert pm.query_voxel(0, 0, 0) == OCCUPIED, "unknown 不得覆盖已建模的 occupied"


def test_merge_occupied_priority():
    """多个 local 体素（含 OCCUPIED）映射同一持久体素 → OCCUPIED 优先（np.maximum.at）。"""
    pm = PersistentMap(res=0.6, block=4, origin=(0.0, 0.0, 0.0))
    # 先写 FREE
    loc, org, res = _local(6, 0.2, fill=FREE)
    pm.merge_from_local(loc, org, res)
    assert pm.query_voxel(0, 0, 0) == FREE
    # 同一持久体素内再来一个 OCCUPIED（不同 local 体素，但都落在 (0,0,0) 持久体素内）
    loc2, org2, res2 = _local(6, 0.2, fill=FREE)
    loc2[2, 2, 2] = OCCUPIED                 # center=(0.5,0.5,0.5) → 持久 (0,0,0)
    pm.merge_from_local(loc2, org2, res2)
    assert pm.query_voxel(0, 0, 0) == OCCUPIED


def test_seed_fills_only_unknown():
    """先验只填 UNKNOWN，绝不改已感知的 FREE/OCCUPIED。"""
    pm = PersistentMap(res=0.6, block=4, origin=(0.0, 0.0, 0.0))
    # 持久图：voxel(0,0,0)=OCCUPIED，(1,0,0)=FREE
    b0 = np.full((4, 4, 4), UNKNOWN, dtype="int8")
    b0[0, 0, 0] = OCCUPIED
    b0[1, 0, 0] = FREE
    pm.blocks[(0, 0, 0)] = b0
    # local 已有感知：voxel(0,0,0)=FREE（已感知为 free，不应被持久 OCCUPIED 覆盖）
    loc, org, res = _local(12, 0.2)
    loc[0, 0, 0] = FREE                       # 对应持久 voxel(0,0,0)
    loc[5, 0, 0] = UNKNOWN                    # center=(1.1,0.1,0.1) → 持久 voxel(1,0,0)=FREE
    n = pm.seed_local(loc, org, res)
    assert n > 0
    assert loc[0, 0, 0] == FREE, "先验不得覆盖已感知的 FREE"
    assert loc[5, 0, 0] == FREE, "持久 FREE 应填进 UNKNOWN"
    # 持久 OCCUPIED 对应但 local 是 UNKNOWN 的体素：local(3,0,0) center=(0.7,0.1,0.1) → 持久(1,0,0)=FREE
    # local(1,0,0) center=(0.3,0.1,0.1) → 持久(0,0,0)=OCCUPIED → 应被填 OCCUPIED
    assert loc[1, 0, 0] == OCCUPIED, "持久 OCCUPIED 应填进 UNKNOWN"


def test_seed_occlusion():
    """滚动窗口场景：旧地图记得的障碍，在新窗口（该处为 UNKNOWN）里被恢复为不可通行。"""
    pm = PersistentMap(res=0.6, block=4, origin=(0.0, 0.0, 0.0))
    b = np.full((4, 4, 4), UNKNOWN, dtype="int8")
    b[0, 0, 0] = OCCUPIED
    pm.blocks[(0, 0, 0)] = b
    loc, org, res = _local(6, 0.2)            # 全新窗口，全 UNKNOWN
    pm.seed_local(loc, org, res)
    # 世界点 (0.1,0.1,0.1) 落在持久 OCCUPIED 体素内 → 应被标 OCCUPIED
    assert loc[0, 0, 0] == OCCUPIED


def test_save_load_roundtrip():
    pm = PersistentMap(res=0.6, block=4, origin=(0.0, 0.0, 0.0))
    loc, org, res = _local(6, 0.2, fill=FREE)
    loc[0, 0, 0] = OCCUPIED
    pm.merge_from_local(loc, org, res)
    stats_before = pm.stats()
    assert stats_before["n_blocks"] > 0
    assert stats_before["n_occupied"] > 0

    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "pm.npz")
        pm.save(path)
        pm2 = PersistentMap.load(path)
        s2 = pm2.stats()
        assert s2["n_blocks"] == stats_before["n_blocks"]
        assert s2["n_occupied"] == stats_before["n_occupied"]
        assert s2["res"] == 0.6
        assert s2["block"] == 4
        assert pm2.query_voxel(0, 0, 0) == OCCUPIED
        # 空图（文件不存在）加载 → 空图不抛
        pm3 = PersistentMap.load(os.path.join(d, "missing.npz"), res=0.6, block=4)
        assert pm3.stats()["n_blocks"] == 0


def test_rolling_origin_alignment():
    """两方向合并走同一世界锚点 → 滚动窗口 origin 漂移不产生错位。"""
    pm = PersistentMap(res=0.6, block=16, origin=(0.0, 0.0, 0.0))
    # 窗口 A（origin=0）：在 voxel(5,5,5) 放障碍 → 世界 center (1.1,1.1,1.1) → 持久 (1,1,1)
    locA, orgA, resA = _local(10, 0.2, origin=(0.0, 0.0, 0.0))
    locA[5, 5, 5] = OCCUPIED
    pm.merge_from_local(locA, orgA, resA)
    assert pm.query_voxel(1, 1, 1) == OCCUPIED
    # 窗口 B（origin 偏移 0.4m，非持久体素整数倍）→ seed 恢复同一世界障碍
    locB, orgB, resB = _local(10, 0.2, origin=(0.4, 0.4, 0.4))
    pm.seed_local(locB, orgB, resB)
    # 世界点 (1.1,1.1,1.1) 在 B 里是 voxel(3,3,3)（center=0.4+3.5*0.2=1.1）→ OCCUPIED
    assert locB[3, 3, 3] == OCCUPIED
    # 远离障碍处保持 UNKNOWN
    assert locB[0, 0, 0] == UNKNOWN


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"PASS {fn.__name__}")
    print(f"全部 {len(fns)} 个测试通过")


if __name__ == "__main__":
    _run_all()
