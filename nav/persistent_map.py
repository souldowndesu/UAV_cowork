# -*- coding: utf-8 -*-
"""Persistent Map（持久建图，方案书 §4–§7、§16–§17、§59、§61）。

三层存储结构（§4）：
- Persistent World Store（SSD）：本类，稀疏块体素图（§5），``npz`` 落盘。
- Route Cache（RAM）：``self.blocks`` 字典即常驻内存的块集（城市尺度全量加载）。
- Local Working Field（RAM）：``nav/occupancy.py`` 的滚动 ``OccupancyMap``（``r_L``）。

两方向合并语义（核心正确性，§16 Sensor>Historical Map / §17 Local→Persistent）：
- ``seed_local``（Persistent→Local 先验）：只把持久图的 FREE/OCCUPIED 填进 Local 的
  **UNKNOWN**，永不修改 Local 已感知的 FREE/OCCUPIED。
- ``merge_from_local``（Local→Persistent 写回）：只写 Local 的 FREE/OCCUPIED，
  **UNKNOWN 永不写**（unknown 不修改其它内容，消除滚动窗口清零导致的死循环）。

坐标对齐（防止"理解差误"）：
- 单一世界锚点 ``origin``（NED），索引可为负（floor 对负值正确）。
- 两方向都用同一公式：世界点 → 持久体素 = ``floor((center - origin) / res)``，
  按体素中心做点查询（不做块填充），与 Local origin 是否对齐无关。
- ``res`` 必须是 Local ``res`` 的整数倍（调用方校验），保证持久图与精细图跨体素边界
  不产生半格错位。
"""
from __future__ import annotations

import json
import os

import numpy as np

from .occupancy import UNKNOWN, FREE, OCCUPIED


class PersistentMap:
    def __init__(self, res: float, block: int = 16, origin=(0.0, 0.0, 0.0)):
        self.res = float(res)
        self.block = int(block)
        self.origin = np.array(origin, dtype="float64")  # 体素 [0,0,0] 低角（世界系 NED）
        # (bx, by, bz) -> int8[block, block, block]；全 UNKNOWN 的块不创建（§5 稀疏）
        self.blocks = {}

    # ------------------------------------------------------------------
    # 世界 ↔ 体素 ↔ 块
    # ------------------------------------------------------------------
    def world_to_voxel(self, p):
        px = (float(p[0]) - self.origin[0]) / self.res
        py = (float(p[1]) - self.origin[1]) / self.res
        pz = (float(p[2]) - self.origin[2]) / self.res
        return int(np.floor(px)), int(np.floor(py)), int(np.floor(pz))

    def voxel_to_world(self, i, j, k):
        return self.origin + (np.array([i, j, k], dtype="float64") + 0.5) * self.res

    def _block_of(self, i, j, k):
        """全局体素索引 → (块键, 块内偏移)。floor 除法对负索引正确。"""
        bx = int(np.floor(i / self.block))
        by = int(np.floor(j / self.block))
        bz = int(np.floor(k / self.block))
        ox = i - bx * self.block
        oy = j - by * self.block
        oz = k - bz * self.block
        return (bx, by, bz), (ox, oy, oz)

    def validate_alignment(self, local_res: float) -> None:
        """校验 self.res 是 local_res 的整数倍（否则跨体素边界产生半格错位）。"""
        ratio = self.res / float(local_res)
        if abs(ratio - round(ratio)) > 1e-9:
            raise ValueError(
                f"persistent_res({self.res}) 必须是 local res({local_res}) 的整数倍，"
                f"否则持久图与精细图跨体素边界会产生半格错位")

    def query_voxel(self, i, j, k):
        (bx, by, bz), (ox, oy, oz) = self._block_of(i, j, k)
        blk = self.blocks.get((bx, by, bz))
        if blk is None:
            return UNKNOWN
        return int(blk[ox, oy, oz])

    # ------------------------------------------------------------------
    # 两方向合并
    # ------------------------------------------------------------------
    def seed_local(self, local_occ, local_origin, local_res) -> int:
        """load 先验：把持久图的 FREE/OCCUPIED 填进 Local 的 UNKNOWN，返回填充体素数。

        只触碰 ``local_occ == UNKNOWN`` 的位置，绝不覆盖已感知的 FREE/OCCUPIED。
        """
        local_occ = np.asarray(local_occ, dtype="int8")
        local_origin = np.asarray(local_origin, dtype="float64")
        local_res = float(local_res)
        unk = np.argwhere(local_occ == UNKNOWN)
        if unk.shape[0] == 0:
            return 0
        # 每个 Local UNKNOWN 体素的中心 → 持久体素索引（点查询，与 local origin 无关）
        centers = local_origin + (unk.astype("float64") + 0.5) * local_res
        pvi = np.floor((centers - self.origin) / self.res).astype("int64")
        bi = np.floor_divide(pvi, self.block)
        off = np.mod(pvi, self.block)

        states = np.full(unk.shape[0], UNKNOWN, dtype="int8")
        bi_uniq, inv = np.unique(bi, axis=0, return_inverse=True)
        for u in range(bi_uniq.shape[0]):
            blk = self.blocks.get((int(bi_uniq[u, 0]), int(bi_uniq[u, 1]), int(bi_uniq[u, 2])))
            if blk is None:
                continue
            sel = inv == u
            states[sel] = blk[off[sel, 0], off[sel, 1], off[sel, 2]]

        fill = states != UNKNOWN
        if fill.any():
            local_occ[unk[fill, 0], unk[fill, 1], unk[fill, 2]] = states[fill]
        return int(fill.sum())

    def merge_from_local(self, local_occ, local_origin, local_res) -> int:
        """写回：只写 Local 的 FREE/OCCUPIED，UNKNOWN 跳过，OCCUPIED 优先，返回写入体素数。

        多个 Local 体素映射到同一持久体素时用 ``np.maximum.at`` 取最大值（OCCUPIED=2
        优先于 FREE=1），保证细网格里的单个占用点不会被粗网格里的大量空闲点稀释掉。
        读的是 **raw 图**（真实观测），调用方须保证不传碰撞膨胀图。
        """
        local_occ = np.asarray(local_occ, dtype="int8")
        local_origin = np.asarray(local_origin, dtype="float64")
        local_res = float(local_res)
        known = np.argwhere(local_occ != UNKNOWN)
        if known.shape[0] == 0:
            return 0
        centers = local_origin + (known.astype("float64") + 0.5) * local_res
        pvi = np.floor((centers - self.origin) / self.res).astype("int64")
        bi = np.floor_divide(pvi, self.block)
        off = np.mod(pvi, self.block)
        states = local_occ[known[:, 0], known[:, 1], known[:, 2]]

        bi_uniq, inv = np.unique(bi, axis=0, return_inverse=True)
        for u in range(bi_uniq.shape[0]):
            key = (int(bi_uniq[u, 0]), int(bi_uniq[u, 1]), int(bi_uniq[u, 2]))
            blk = self.blocks.get(key)
            if blk is None:
                blk = np.full((self.block, self.block, self.block), UNKNOWN, dtype="int8")
                self.blocks[key] = blk
            sel = inv == u
            batch = np.zeros_like(blk)
            np.maximum.at(batch, (off[sel, 0], off[sel, 1], off[sel, 2]), states[sel])
            touched = batch != UNKNOWN
            blk[touched] = batch[touched]
        return int(known.shape[0])

    # ------------------------------------------------------------------
    # 查询 / 导出
    # ------------------------------------------------------------------
    def _points_of(self, state, stride: int = 1) -> np.ndarray:
        """指定状态体素的世界系中心 (N,3)，stride 下采样控体积。"""
        s = int(max(1, stride))
        pts = []
        # list() 快照：mapping 线程的 merge_from_local 会并发新增块，直接迭代 items()
        # 会抛 "dictionary changed size during iteration"（autosave 周期刷新快照时触发）。
        for (bx, by, bz), blk in list(self.blocks.items()):
            idx = np.argwhere(blk == state)
            if idx.shape[0] == 0:
                continue
            idx = idx[::s]
            g = np.array([bx * self.block, by * self.block, bz * self.block]) + idx
            w = self.origin + (g + 0.5) * self.res
            pts.append(w)
        if not pts:
            return np.zeros((0, 3), dtype="float64")
        return np.concatenate(pts, axis=0)

    def occupied_points(self, stride: int = 1) -> np.ndarray:
        """所有 OCCUPIED 体素的世界系中心 (N,3)，供 replay 显示（stride 下采样控体积）。"""
        return self._points_of(OCCUPIED, stride)

    def unknown_points(self, stride: int = 1) -> np.ndarray:
        """所有 UNKNOWN 体素的世界系中心 (N,3)，供 replay 灰色显示未观测区。"""
        return self._points_of(UNKNOWN, stride)

    def unknown_points_in_bbox(self, bbox, stride: int = 1) -> np.ndarray:
        """只导出世界 bbox=[xmin,xmax,ymin,ymax,zmin,zmax]（NED，含边界）内的 UNKNOWN 体素中心。

        持久图只存 FREE/OCCUPIED（§16/§17），块内未填即 UNKNOWN；但「全量块内空白」没有观察
        意义——只有落在感知区（A* 计算域，由轨迹 bbox 膨胀得到）内的 unknown 才是无人机实际
        扫到却未观测到的区域（遮挡后方 / 未覆盖）。本方法按 bbox 过滤，避免全量导出
        （用户 m03721/m03748：全量 unknown 太卡且无意义）。
        """
        s = int(max(1, stride))
        xmin, xmax, ymin, ymax, zmin, zmax = (float(v) for v in bbox)
        lo = self.world_to_voxel([xmin, ymin, zmin])
        hi = self.world_to_voxel([xmax, ymax, zmax])
        blo = (int(np.floor(lo[0] / self.block)),
               int(np.floor(lo[1] / self.block)),
               int(np.floor(lo[2] / self.block)))
        bhi = (int(np.floor(hi[0] / self.block)),
               int(np.floor(hi[1] / self.block)),
               int(np.floor(hi[2] / self.block)))
        pts = []
        for bx in range(blo[0], bhi[0] + 1):
            for by in range(blo[1], bhi[1] + 1):
                for bz in range(blo[2], bhi[2] + 1):
                    blk = self.blocks.get((bx, by, bz))
                    if blk is None:
                        continue
                    idx = np.argwhere(blk == UNKNOWN)
                    if idx.shape[0] == 0:
                        continue
                    idx = idx[::s]
                    g = np.array([bx * self.block, by * self.block, bz * self.block]) + idx
                    w = self.origin + (g + 0.5) * self.res
                    sel = ((w[:, 0] >= xmin) & (w[:, 0] <= xmax)
                           & (w[:, 1] >= ymin) & (w[:, 1] <= ymax)
                           & (w[:, 2] >= zmin) & (w[:, 2] <= zmax))
                    if sel.any():
                        pts.append(w[sel])
        if not pts:
            return np.zeros((0, 3), dtype="float64")
        return np.concatenate(pts, axis=0)

    # ------------------------------------------------------------------
    # 全向建图（§54）：未裁剪帧融合
    # ------------------------------------------------------------------
    def integrate_frame(self, vehicle_pos, points, max_range, min_d=0.3) -> int:
        """把一帧 LiDAR 点云**未裁剪**地融合进持久图（全向建图，§54）。

        与 ``merge_from_local`` 的关键区别：这里对 max_range 内的所有点做 DDA
        （FREE 沿射线 + OCCUPIED 命中），不受局部滚动窗口尺寸限制。局部图 z 窗只有
        10m，窗外点（尤其高空/低空）会被 ``raycast.integrate`` 丢弃 → 持久图永远缺
        这些数据 → 爬升/下降后 ``seed_local`` 无数据回填 → 出现 unknown（用户 m03549）。

        实现：构造以载具为中心、对齐持久图栅格的稠密缓冲（覆盖 ±max_range），复用
        C++ ``_fast.raycast_batch`` 做 DDA，再用 ``merge_from_local`` 写回稀疏块
        （UNKNOWN 跳过、OCCUPIED 优先，语义与局部写回一致）。返回实际投射射线数。
        """
        pts = np.asarray(points, dtype="float64")
        if pts.ndim != 2 or pts.shape[0] == 0:
            return 0
        d = pts - np.asarray(vehicle_pos, dtype="float64")
        dist = np.linalg.norm(d, axis=1)
        pts = pts[(dist >= min_d) & (dist <= max_range)]
        if pts.shape[0] == 0:
            return 0
        # 在持久图分辨率下去重（相邻命中可能落在同一粗体素）
        v = np.floor((pts - self.origin) / self.res).astype("int64")
        _, idx = np.unique(v, axis=0, return_index=True)
        hits = pts[np.sort(idx)]
        n = hits.shape[0]
        origins = np.tile(np.asarray(vehicle_pos, dtype="float64"), (n, 1))

        half = float(max_range)
        c = np.asarray(vehicle_pos, dtype="float64")
        buf_origin = np.floor((c - half) / self.res) * self.res
        side = int(np.ceil((2.0 * half) / self.res)) + 2
        buf = np.zeros((side, side, side), dtype="int8")

        try:
            from . import _fast  # type: ignore
            if _fast is not None:
                _fast.raycast_batch(
                    np.ascontiguousarray(origins),
                    np.ascontiguousarray(hits),
                    buf,
                    np.ascontiguousarray(buf_origin),
                    float(self.res),
                    float(max_range),
                )
            else:
                self._raycast_into(buf, origins, hits, buf_origin)
        except Exception:
            self._raycast_into(buf, origins, hits, buf_origin)

        self.merge_from_local(buf, buf_origin, self.res)
        return n

    def _raycast_into(self, buf, origins, hits, buf_origin) -> None:
        """纯 numpy 回退：DDA 标记 FREE 沿射线 / OCCUPIED 命中（仅 _fast 不可用时）。"""
        res = self.res
        side = buf.shape[0]
        for k in range(hits.shape[0]):
            o = origins[k]
            seg = hits[k] - o
            length = float(np.linalg.norm(seg))
            if length < 1e-6:
                continue
            n_steps = max(2, int(np.ceil(length / (res * 0.5))) + 1)
            ts = np.linspace(0.0, 1.0, n_steps)
            p = o[None, :] + ts[:, None] * seg[None, :]
            v = np.floor((p - buf_origin) / res).astype("int64")
            keep = np.ones(v.shape[0], dtype=bool)
            keep[1:] = np.any(v[1:] != v[:-1], axis=1)
            v = v[keep]
            inb = ((v[:, 0] >= 0) & (v[:, 0] < side)
                   & (v[:, 1] >= 0) & (v[:, 1] < side)
                   & (v[:, 2] >= 0) & (v[:, 2] < side))
            v = v[inb]
            if v.shape[0] == 0:
                continue
            buf[v[1:-1, 0], v[1:-1, 1], v[1:-1, 2]] = FREE
            buf[v[-1, 0], v[-1, 1], v[-1, 2]] = OCCUPIED

    def stats(self) -> dict:
        n_occ = sum(int((blk == OCCUPIED).sum()) for blk in list(self.blocks.values()))
        return {
            "n_blocks": len(self.blocks),
            "n_occupied": n_occ,
            "res": float(self.res),
            "block": int(self.block),
            "origin": [float(x) for x in self.origin],
        }

    # ------------------------------------------------------------------
    # 持久化（§59：实时线程不等待磁盘 → 调用方在 mapping 线程低频触发）
    # ------------------------------------------------------------------
    def save(self, path) -> None:
        """原子落盘 npz（只存含 FREE/OCCUPIED 的块，unknown 不落盘）+ meta 侧车。

        永不抛异常（失败打印原因），不阻断任务；tmp+replace 保证进程被杀不损坏旧图。
        """
        try:
            d = os.path.dirname(path)
            if d:
                os.makedirs(d, exist_ok=True)
            arrays = {}
            for (bx, by, bz), blk in list(self.blocks.items()):
                arrays[f"b{bx}_{by}_{bz}"] = blk
            tmp = path + ".tmp.npz"  # 以 .npz 结尾，避免 numpy 自动再追加后缀
            np.savez_compressed(tmp, **arrays)
            os.replace(tmp, path)
            mp = path + ".meta.json"
            meta = self.stats()
            with open(mp, "w", encoding="utf-8") as f:
                json.dump(meta, f, ensure_ascii=False)
        except Exception as e:
            print(f"[PersistentMap] 保存失败：{e}")

    @classmethod
    def load(cls, path, res: float = 0.6, block: int = 16,
             origin=(0.0, 0.0, 0.0)) -> "PersistentMap":
        """加载持久图；文件不存在/损坏 → 返回空图（不抛异常）。

        优先用 meta 侧车里的 res/block/origin（保证同一文件自洽往返），缺失时回退到入参。
        """
        meta = {}
        mp = path + ".meta.json"
        if os.path.isfile(mp):
            try:
                with open(mp, "r", encoding="utf-8") as f:
                    meta = json.load(f)
            except Exception:
                meta = {}
        res = float(meta.get("res", res))
        block = int(meta.get("block", block))
        origin = tuple(meta.get("origin", origin))
        pm = cls(res, block, origin)
        if os.path.isfile(path):
            try:
                data = np.load(path)
                for key in data.files:
                    parts = key[1:].split("_")
                    if len(parts) != 3:
                        continue
                    bx, by, bz = int(parts[0]), int(parts[1]), int(parts[2])
                    arr = np.asarray(data[key], dtype="int8")
                    if arr.shape == (block, block, block):
                        pm.blocks[(bx, by, bz)] = arr
                data.close()
            except Exception as e:
                print(f"[PersistentMap] 加载 {path} 失败（{e}），从空图开始")
                pm.blocks.clear()
        return pm
