# -*- coding: utf-8 -*-
"""3D 滚动体素占据图（Free / Unknown / Occupied，方案书 §10–§12）。

- 状态用 ``int8``：``0=Unknown``、``1=Free``、``2=Occupied``（§21 三态）。
- 世界系为 AirSim NED（+X 北 / +Y 东 / +Z 向下）。
- 载具移动时按整格平移体素数组（``np.roll`` + 清零新进条带），避免复制全部体素；
  零拷贝环形缓冲留作后续优化（§12，Phase 1 规模下 ``np.roll`` 已足够）。
- ``origin`` 是体素 ``[0,0,0]`` 低角（min x/y/z）在世界系下的坐标。
"""
from __future__ import annotations

from typing import Tuple

import numpy as np

# 三态（§21）
UNKNOWN = 0
FREE = 1
OCCUPIED = 2


class OccupancyMap:
    def __init__(self, nx: int, ny: int, nz: int, res: float,
                 origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                 center_offset=None):
        self.nx = int(nx)
        self.ny = int(ny)
        self.nz = int(nz)
        self.res = float(res)
        # 形状 (nx, ny, nz)，索引 [i, j, k] 对应世界 x/y/z 轴
        self.data = np.full((self.nx, self.ny, self.nz), UNKNOWN, dtype="int8")
        self.origin = np.array(origin, dtype="float64")  # 体素 [0,0,0] 低角（NED）
        # 无人机在地图中的体素位置（recenter 用它而非几何中心，支持前向偏置）
        if center_offset is None:
            center_offset = (self.nx // 2, self.ny // 2, self.nz // 2)
        self.center_offset = np.array(center_offset, dtype="float64")

    # ------------------------------------------------------------------
    # 世界 ↔ 体素
    # ------------------------------------------------------------------
    def world_to_voxel(self, p) -> Tuple[int, int, int]:
        px = (float(p[0]) - self.origin[0]) / self.res
        py = (float(p[1]) - self.origin[1]) / self.res
        pz = (float(p[2]) - self.origin[2]) / self.res
        return int(np.floor(px)), int(np.floor(py)), int(np.floor(pz))

    def voxel_to_world(self, i: int, j: int, k: int) -> np.ndarray:
        return self.origin + (np.array([i, j, k], dtype="float64") + 0.5) * self.res

    def in_bounds(self, i: int, j: int, k: int) -> bool:
        return 0 <= i < self.nx and 0 <= j < self.ny and 0 <= k < self.nz

    # ------------------------------------------------------------------
    # 写入
    # ------------------------------------------------------------------
    def set_voxel(self, i: int, j: int, k: int, state: int) -> None:
        if self.in_bounds(i, j, k):
            self.data[i, j, k] = state

    def set_world(self, p, state: int) -> None:
        i, j, k = self.world_to_voxel(p)
        self.set_voxel(i, j, k, state)

    def mark_free(self, i: int, j: int, k: int) -> None:
        self.set_voxel(i, j, k, FREE)

    def mark_occupied(self, i: int, j: int, k: int) -> None:
        self.set_voxel(i, j, k, OCCUPIED)

    def mark_visible_free(self, center, max_range: float) -> None:
        """把从 ``center`` 出发、在 ``max_range`` 内、且不被占据体素遮挡的 Unknown 标为 Free。

        全包围 LiDAR 的"无回波方向"即开放空域（如头顶天空），应视为已探明的空闲；
        但被障碍遮挡的后方必须保持 Unknown。实现：在**非占据空间（FREE+UNKNOWN）**上做
        26 连通分量标记，载具体素所在分量即"从载具出发、不被障碍遮挡、可达的已知+未知
        空域"；把其中 ``max_range`` 内的 UNKNOWN 标 FREE。

        OCCUPIED 阻断连通 → 障碍后方保持 UNKNOWN（A* 不会穿墙）。
        滚动窗口清零产生的"UNKNOWN 洞"（即"遗忘障碍"）由 seed_local 在上游用持久图回填
        OCCUPIED 消除，故此处 UNKNOWN 只会是真正的开放空域（天空/未探索前方），标 FREE 不会穿墙。
        """
        try:
            from scipy import ndimage as _ndi
        except Exception:
            return  # 无 scipy 时保守跳过（本工程距离场已依赖 scipy，正常不会走到）
        ci, cj, ck = self.world_to_voxel(center)
        if not self.in_bounds(ci, cj, ck):
            return
        if self.data[ci, cj, ck] == OCCUPIED:
            return  # 载具体素被占据（异常），保守跳过
        # 载具体素必为空闲（无人机自身位置），显式标 FREE 作为泛洪种子。
        self.data[ci, cj, ck] = FREE
        # 可泛洪空间 = FREE + UNKNOWN（OCCUPIED 阻断传播 → 障碍后方保持未知）。
        traversable = self.data != OCCUPIED
        labels, _ = _ndi.label(traversable, structure=np.ones((3, 3, 3), dtype="int8"))
        dl = int(labels[ci, cj, ck])
        if dl == 0:
            return
        in_comp = labels == dl
        r2 = float(max_range) * float(max_range)
        cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
        i = np.arange(self.nx)[:, None]
        j = np.arange(self.ny)[None, :]
        wx = self.origin[0] + (i + 0.5) * self.res
        wy = self.origin[1] + (j + 0.5) * self.res
        dxy2 = (wx - cx) ** 2 + (wy - cy) ** 2
        for k in range(self.nz):
            wz = self.origin[2] + (k + 0.5) * self.res
            d2 = dxy2 + (wz - cz) ** 2
            slab = self.data[:, :, k]
            mask = (d2 <= r2) & (slab == UNKNOWN) & in_comp[:, :, k]
            slab[mask] = FREE

    def inflate_sensor_footprint(self, center, dtheta_h_rad: float, dtheta_v_rad: float) -> None:
        """距离相关的传感器脚印膨胀（与车辆碰撞膨胀分离）。

        每个 occupied hit 距离 ``center`` 为 r 时，其角采样单元在横向于射线方向的尺寸为
        ``Δ_h(r)≈r·tan(Δθ_h)``、``Δ_v(r)≈r·tan(Δθ_v)``。向横向扩展半个单元（``½Δ_h``、
        ``½Δ_v``）接上相邻 beam 空隙——距离越远膨胀越大。垂直是主导项（Δθ_v 大），
        用逐 dz 向量化；水平项小（20m 处仅 0.2m），粗网格 max 已自然填充，故仍按立方近似
        但用向量化水平膨胀实现（nh 通常 1）。
        """
        oi = np.argwhere(self.data == OCCUPIED)
        if oi.shape[0] == 0:
            return
        ow = self.origin + (oi.astype("float64") + 0.5) * self.res
        r = np.linalg.norm(ow - np.asarray(center, dtype="float64"), axis=1)
        n_h = np.ceil(0.5 * r * np.tan(dtheta_h_rad) / self.res).astype("int64")
        n_v = np.ceil(0.5 * r * np.tan(dtheta_v_rad) / self.res).astype("int64")
        # 垂直膨胀（主导，逐 dz 向量化）
        max_nv = int(n_v.max()) if n_v.size else 0
        for dz in range(1, max_nv + 1):
            sel = n_v >= dz
            if not sel.any():
                continue
            ii = oi[sel, 0]; jj = oi[sel, 1]; kk = oi[sel, 2]
            kd = kk + dz
            ok = kd < self.nz
            self.data[ii[ok], jj[ok], kd[ok]] = OCCUPIED
            ku = kk - dz
            ok = ku >= 0
            self.data[ii[ok], jj[ok], ku[ok]] = OCCUPIED
        # 水平膨胀（小项，逐 d 向量化，立方近似）
        max_nh = int(n_h.max()) if n_h.size else 0
        for dy in range(-max_nh, max_nh + 1):
            for dx in range(-max_nh, max_nh + 1):
                if dx == 0 and dy == 0:
                    continue
                sel = (np.abs(dx) <= n_h) & (np.abs(dy) <= n_h)
                if not sel.any():
                    continue
                ii = oi[sel, 0] + dx
                jj = oi[sel, 1] + dy
                kk = oi[sel, 2]
                ok = (ii >= 0) & (ii < self.nx) & (jj >= 0) & (jj < self.ny)
                self.data[ii[ok], jj[ok], kk[ok]] = OCCUPIED

    def inflate_fixed(self, radius: float) -> None:
        """固定半径膨胀（无人机机身余量）：把 occupied 周围 ``radius`` 内的体素标为 Occupied。

        与传感器脚印膨胀（随距离变大）不同，这里按无人机物理尺寸做恒定球形膨胀，
        使"把无人机视作点"的规划对建筑天然保持机身半径的净空。用 scipy 球形结构元素
        一次膨胀完成；scipy 缺失时退化为逐体素曼哈顿近似。
        """
        r = int(np.ceil(float(radius) / self.res))
        if r <= 0:
            return
        try:
            from scipy import ndimage as _ndi
            x = np.arange(-r, r + 1)
            xx, yy, zz = np.meshgrid(x, x, x, indexing="ij")
            struct = (xx * xx + yy * yy + zz * zz) <= r * r
            dilated = _ndi.binary_dilation(self.data == OCCUPIED, structure=struct)
        except Exception:
            # 退化：26 连通重复膨胀 r 次（曼哈顿/切比雪夫近似）
            dilated = (self.data == OCCUPIED).copy()
            for _ in range(r):
                d = dilated.copy()
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for dz in (-1, 0, 1):
                            if dx == 0 and dy == 0 and dz == 0:
                                continue
                            s = tuple(slice(max(0, -o), min(sz, sz - o))
                                      for o, sz in zip((dx, dy, dz), dilated.shape))
                            t = tuple(slice(max(0, o), min(sz, sz + o))
                                      for o, sz in zip((dx, dy, dz), dilated.shape))
                            d[t] |= dilated[s]
                dilated = d
        self.data[dilated] = OCCUPIED

    def derive_collision(self, center, dtheta_h_rad: float, dtheta_v_rad: float,
                         drone_radius: float = 0.0) -> "OccupancyMap":
        """从 raw 占据图派生碰撞图（§两图分离）。

        raw 层只保存真实观测（DDA：射线→Free、命中→Occupied），**永不做原地膨胀**；
        碰撞层每一周期从 raw 复制后依次做：传感器脚印膨胀（随距离）+ 机身半径膨胀（固定），
        膨胀不写回 raw，故不会累积。返回碰撞图仍三态（Free/Unknown/Occupied），供 EDT/A*。
        """
        collision = self.copy()
        collision.inflate_sensor_footprint(center, dtheta_h_rad, dtheta_v_rad)
        if drone_radius > 0:
            collision.inflate_fixed(drone_radius)
        return collision

    # ------------------------------------------------------------------
    # 查询
    # ------------------------------------------------------------------
    def state_at_voxel(self, i: int, j: int, k: int) -> int:
        if not self.in_bounds(i, j, k):
            return OCCUPIED  # 越界按占据处理（保守，§56 边界）
        return int(self.data[i, j, k])

    def is_occupied(self, i: int, j: int, k: int) -> bool:
        return self.state_at_voxel(i, j, k) == OCCUPIED

    def is_free(self, i: int, j: int, k: int) -> bool:
        return self.state_at_voxel(i, j, k) == FREE

    # ------------------------------------------------------------------
    # 滚动（§11–§12）
    # ------------------------------------------------------------------
    def recenter(self, center) -> None:
        """把窗口平移到 ``center``（世界系点），只做整格平移。

        无人机位置落在 ``self.center_offset`` 处（默认几何中心；可设前向偏置，
        例如 X 方向只留 ``map_back_x`` 米在后、其余余量全给前方）。
        """
        des_origin = np.array([float(center[0]), float(center[1]), float(center[2])]) \
            - (self.center_offset + 0.5) * self.res
        delta = np.floor((des_origin - self.origin) / self.res).astype("int64")
        if np.any(delta != 0):
            self._shift(int(delta[0]), int(delta[1]), int(delta[2]))
            self.origin += delta * self.res

    def _shift(self, dx: int, dy: int, dz: int) -> None:
        """按整格平移数据；移入的条带清零（恢复为 Unknown）。"""
        # np.roll 使旧内容沿 (sx,sy,sz) 移动（sx=-dx），再清零新进入窗口的条带
        sx, sy, sz = -dx, -dy, -dz
        arr = np.roll(self.data, shift=(sx, sy, sz), axis=(0, 1, 2))
        self._zero_slice(arr, 0, sx)
        self._zero_slice(arr, 1, sy)
        self._zero_slice(arr, 2, sz)
        self.data = arr

    @staticmethod
    def _zero_slice(arr, axis: int, s: int) -> None:
        if s > 0:
            sl = [slice(None)] * 3
            sl[axis] = slice(0, s)
            arr[tuple(sl)] = UNKNOWN
        elif s < 0:
            sl = [slice(None)] * 3
            sl[axis] = slice(s, None)
            arr[tuple(sl)] = UNKNOWN

    # ------------------------------------------------------------------
    # 统计 / 快照
    # ------------------------------------------------------------------
    def occupied_ratio(self) -> float:
        return float(np.mean(self.data == OCCUPIED))

    def known_ratio(self) -> float:
        return float(np.mean(self.data != UNKNOWN))

    def copy(self) -> "OccupancyMap":
        m = OccupancyMap(self.nx, self.ny, self.nz, self.res, tuple(self.origin),
                         tuple(self.center_offset))
        m.data = self.data.copy()
        return m

    def reset(self) -> None:
        self.data.fill(UNKNOWN)
