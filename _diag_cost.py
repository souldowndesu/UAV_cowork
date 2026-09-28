# -*- coding: utf-8 -*-
"""诊断 A* 代价（真机窗口版）：绕行街道 = UNKNOWN（LiDAR 看不到），墙后 = UNKNOWN。

关键场景差异（vs 上一版）：
- 上一版绕行街道是 FREE（可见），A* 自然绕行——但真机里绕行街道 Y=0 距无人机
  约 20m+，超出 30m 感知半径的南侧斜距（sqrt(30^2+20^2)=36m>30m），是 UNKNOWN。
- 本版绕行街道设为 UNKNOWN，模拟真机「看不到 FREE 走廊」的真实情况。

验证：UNKNOWN 施加 d_min 硬截断 + w_obs·φ(D) 距离惩罚，能否让 A* 选择
「绕行开阔 UNKNOWN 街道」而非「穿贴墙 UNKNOWN 缝隙」。

地面真值（贴近 _scan_city2.py 细扫）：
- 建筑群 X=135..175, Y=5..40（OCCUPIED），目标 Y=20 在建筑区内部。
- 南街道 Y=0..4、北街道 Y=41..44 是贯穿 FREE 通道。
- 无人机卡在 (134,20)，局部窗口 X=124..164（40m）, Y=0..40（40m）。

感知模型（简化 LiDAR）：
- 墙近面（无人机可见的 X=135 面）→ OCCUPIED。
- 墙近面采样缝隙（模拟 LiDAR 采样间隙 + 贴墙 self_echo 过滤）→ UNKNOWN 缝。
- 墙后（被挡）→ UNKNOWN。
- 南街道（斜距 >30m 感知外）→ UNKNOWN。
- 墙前已观测区 → FREE。
"""
import numpy as np
from nav.occupancy import OccupancyMap, UNKNOWN, FREE, OCCUPIED
from nav import distance_field

RES = 0.5
NX, NY, NZ = 80, 80, 20   # 40m x 40m x 10m

NEIS = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
        for dz in (-1, 0, 1) if not (dx == 0 and dy == 0 and dz == 0)]
NEIS_LEN = [np.linalg.norm(o) for o in NEIS]
INF = float("inf")


def _phi(d, d_safe):
    if d >= d_safe:
        return 0.0
    t = 1.0 - d / d_safe
    return t * t


def make_cost_fn(c_free, c_unknown, w_obs, d_safe, d_min, fix_unknown):
    def cost(s, d):
        if s == OCCUPIED:
            return INF
        if d < d_min:
            return INF  # 硬净空（修复后 UNKNOWN 也截断）
        if s == UNKNOWN:
            if fix_unknown:
                return c_unknown + w_obs * _phi(d, d_safe)
            return c_unknown
        return c_free + w_obs * _phi(d, d_safe)
    return cost


def astar(occ, dist, start_v, goal_v, cost_fn, max_nodes=500000):
    import heapq
    nx, ny, nz = occ.shape

    def flat(v):
        return (v[0] * ny + v[1]) * nz + v[2]

    def unflat(f):
        k = f % nz
        j = (f // nz) % ny
        i = f // (ny * nz)
        return i, j, k

    sf, gf = flat(start_v), flat(goal_v)
    g = np.full(nx * ny * nz, INF)
    came = np.full(nx * ny * nz, -1, dtype=np.int64)
    closed = np.zeros(nx * ny * nz, dtype=bool)
    g[sf] = 0.0
    h0 = np.linalg.norm(np.array(start_v) - np.array(goal_v)) * RES
    heap = [(h0, 0.0, sf)]
    expanded = 0
    while heap:
        f, gs, cur = heapq.heappop(heap)
        if closed[cur] or gs > g[cur]:
            continue
        closed[cur] = True
        expanded += 1
        if expanded > max_nodes:
            return None, None
        if cur == gf:
            path = [gf]
            while path[-1] != sf and path[-1] != -1:
                path.append(int(came[path[-1]]))
            if path[-1] == -1:
                return None, None
            path.reverse()
            return [unflat(p) for p in path], gs
        ci, cj, ck = unflat(cur)
        for (dx, dy, dz), elen in zip(NEIS, NEIS_LEN):
            ni, nj, nk = ci + dx, cj + dy, ck + dz
            if not (0 <= ni < nx and 0 <= nj < ny and 0 <= nk < nz):
                continue
            c = cost_fn(int(occ[ni, nj, nk]), float(dist[ni, nj, nk]))
            if c >= INF:
                continue
            ng = gs + c * elen * RES
            nf = flat((ni, nj, nk))
            if ng < g[nf]:
                g[nf] = ng
                came[nf] = cur
                h = np.linalg.norm(np.array([ni, nj, nk]) - np.array(goal_v)) * RES
                heapq.heappush(heap, (ng + h, ng, nf))
    return None, None


def build_map(gap_ij, wall_lo=(22, 10), wall_hi=(79, 78)):
    """无人机 voxel=(20,40,10)（world 134,20,-20）。地图 origin=(124,0,-25)。
    墙 X=voxel 22..79, Y=voxel 10..78 是 OCCUPIED（近面 X=22 起）。
    南街道 Y=voxel 0..9 UNKNOWN；墙后 X>22 UNKNOWN；墙前 X<22 FREE。
    近面缝隙 gap_ij=(i, j_lo, j_hi) 设为 UNKNOWN。"""
    m = OccupancyMap(NX, NY, NZ, RES, (124.0, 0.0, -25.0))
    m.data.fill(UNKNOWN)
    # 墙前已观测区 FREE：X=0..21, Y=0..79
    m.data[:wall_lo[0], :, :] = FREE
    # 南街道 FREE（ground truth 贯穿，但感知外设 UNKNOWN）：Y=0..9 全 UNKNOWN
    # （保持 UNKNOWN，模拟看不到）
    # 墙本体 OCCUPIED：X=22..79, Y=10..78
    m.data[wall_lo[0]:wall_hi[0]+1, wall_lo[1]:wall_hi[1]+1, :] = OCCUPIED
    # 近面缝隙 UNKNOWN：模拟 LiDAR 采样间隙
    gi, gj_lo, gj_hi = gap_ij
    m.data[gi, gj_lo:gj_hi, :] = UNKNOWN
    return m


def run_case(gap_ij, start_v, goal_v, d_min, c_unknown, w_obs, label):
    m = build_map(gap_ij)
    dist = distance_field.compute(m, type("C", (), {"d_max": 4.0})())
    print(f"\n--- {label} ---")
    for fix, name in ((False, "当前"), (True, "修复")):
        fn = make_cost_fn(1.0, c_unknown, w_obs, 2.0, d_min, fix)
        path, tot = astar(m.data, dist, start_v, goal_v, fn)
        if path is None:
            print(f"  [{name}] 无路径")
            continue
        # 判断是否穿墙缝（经过 gap 体素）
        gi, gj_lo, gj_hi = gap_ij
        through_gap = any(p[0] == gi and gj_lo <= p[1] < gj_hi for p in path)
        ys = [p[1] for p in path]
        xs = [p[0] for p in path]
        # 绕行判据：路径是否向南走到 Y<10（南街道）
        go_south = min(ys) < 10
        print(f"  [{name}] len={len(path)} cost={tot:.2f} 穿缝={through_gap} "
              f"绕南街道={go_south} x=[{min(xs)},{max(xs)}] y=[{min(ys)},{max(ys)}]")


if __name__ == "__main__":
    sv = (20, 40, 10)     # (134, 20, -20)
    gv = (79, 40, 10)     # (163.5, 20, -20) 地图东边缘，目标投影方向
    # 近面缝隙：X=22（voxel），Y=39..41（world Y=19.5..20.5，1m 缝，正对无人机）
    gap = (22, 39, 41)

    print("### 基准 d_min=0.5 c_unknown=5 w_obs=6（当前 config）###")
    run_case(gap, sv, gv, 0.5, 5.0, 6.0, "缝1.0m 正对无人机")
    print("\n### d_min=0.8 ###")
    run_case(gap, sv, gv, 0.8, 5.0, 6.0, "缝1.0m")
    print("\n### d_min=1.0 ###")
    run_case(gap, sv, gv, 1.0, 5.0, 6.0, "缝1.0m")
    print("\n### d_min=0.8 c_unknown=10 w_obs=12 ###")
    run_case(gap, sv, gv, 0.8, 10.0, 12.0, "缝1.0m")
    print("\n### d_min=1.0 c_unknown=10 w_obs=12 ###")
    run_case(gap, sv, gv, 1.0, 10.0, 12.0, "缝1.0m")
