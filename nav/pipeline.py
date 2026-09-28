# -*- coding: utf-8 -*-
"""多线程闭环（方案书 §51–§53）。

四线程 + 双缓冲地图快照：
- Sensor（LiDAR 点云，15Hz）
- Mapping（recenter + raycast + 距离场，15Hz）→ 发布地图快照
- Planning（local goal + A* + B-spline，10Hz）→ 更新协调器轨迹
- Control（get_state + 协调器 tick 下发，50Hz，唯一写者）

Mapping 每轮把 (map.data, origin, dist) 拷贝成不可变快照交给 Planning（§53 双缓冲），
避免 Mapping 半写状态被 Planning 读到。
"""
from __future__ import annotations

import threading
import time
import traceback
from dataclasses import dataclass

import numpy as np

from .occupancy import OccupancyMap
from . import raycast, distance_field, astar, bspline
from .coordinator import UnifiedControlCoordinator


@dataclass
class PlanSnapshot:
    occ: np.ndarray        # (nx,ny,nz) int8 占据图拷贝
    origin: np.ndarray     # (3,)
    dist: np.ndarray       # (nx,ny,nz) float32 距离场
    state_pos: np.ndarray  # (3,) 载具位置
    ts: float


class NavigationPipeline:
    def __init__(self, io, cfg):
        self.io = io
        self.cfg = cfg
        self.coordinator = UnifiedControlCoordinator(io, cfg)
        # 前向偏置：无人机在 X 方向距后端 map_back_x 米，余量全给前方（更多前瞻余量）
        back_x_m = float(getattr(cfg, "map_back_x", cfg.map_size[0] / 2.0))
        back_x = int(round(back_x_m / cfg.res))
        self.map = OccupancyMap(cfg.nx, cfg.ny, cfg.nz, cfg.res,
                                center_offset=(back_x, cfg.ny // 2, cfg.nz // 2))
        self.planner = astar.AStarPlanner(cfg)

        self.goal = None
        self.reached = False
        self.collided = False

        self._stop = threading.Event()
        self._points = np.zeros((0, 3), dtype="float32")
        self._points_lock = threading.Lock()
        self._snapshot = None
        self._latest_state = None
        self._telemetry = []
        self._telemetry_lock = threading.Lock()

        self._use_depth = bool(getattr(cfg, "use_depth", False))
        self._lidar_empty = 0

        # 诊断：最近一次重规划的轨迹起点速度（供遥测观察速度连续性）
        self._last_plan_vstart = 0.0
        # 诊断：每次重规划的 A* 路径 / B-spline 控制点 / 目标 / 无人机位置
        self._plan_log = []
        # 诊断：A* 无路径 / 规划成功计数器（节流打印，供真机定位卡住原因）
        self._astar_fail = 0
        self._astar_ok = 0
        self._ctrl_n = 0

        # 录制器（可选）：完整记录状态/点云/重规划，供浏览器实时观看与回放
        self.recorder = None

        self._threads = []

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------
    def set_goal(self, goal):
        self.goal = np.asarray(goal, dtype="float64")

    def start(self):
        self._stop.clear()
        self._threads = [
            threading.Thread(target=self._sensor_loop, daemon=True),
            threading.Thread(target=self._mapping_loop, daemon=True),
            threading.Thread(target=self._planning_loop, daemon=True),
            threading.Thread(target=self._control_loop, daemon=True),
        ]
        for t in self._threads:
            t.start()

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2.0)

    # ------------------------------------------------------------------
    # 线程体
    # ------------------------------------------------------------------
    def _sensor_loop(self):
        cfg = self.cfg
        period = 1.0 / max(1.0, cfg.sensor_hz)
        while not self._stop.is_set():
            t0 = time.time()
            if self._use_depth:
                pts = self.io.get_depth_points()
            else:
                pts = self.io.get_lidar_points()
                if pts.shape[0] == 0:
                    self._lidar_empty += 1
                    if self._lidar_empty > 20:
                        print("[Sensor] LiDAR 持续为空，切换到深度图回退感知")
                        self._use_depth = True
                else:
                    self._lidar_empty = 0
            with self._points_lock:
                self._points = pts
            if self.recorder is not None:
                self.recorder.record_points(time.time(), pts)
            time.sleep(max(0.0, period - (time.time() - t0)))

    def _mapping_loop(self):
        cfg = self.cfg
        period = 1.0 / max(1.0, cfg.mapping_hz)
        while not self._stop.is_set():
            t0 = time.time()
            try:
                state = self.io.get_state()
            except Exception:
                time.sleep(period)
                continue
            with self._points_lock:
                pts = self._points
            self.map.recenter(state.position)
            raycast.integrate(self.map, state.position, pts, cfg)
            # "无回波 = 自由"：把从载具出发、max_range 内、且未被遮挡的 Unknown 标为 Free
            #（保留障碍后方为 Unknown，消除天空/地面的代价不对称导致的向下绕路）。
            self.map.mark_visible_free(state.position, cfg.max_range)
            # 两图分离：raw 只保留真实观测（DDA 的 ray→Free / hit→Occupied + 可见自由），
            # 碰撞图每周期从 raw 派生（传感器脚印膨胀只写碰撞图，不写回 raw，故不累积）。
            collision = self.map.derive_collision(
                state.position,
                np.radians(cfg.lidar_dtheta_h_deg),
                np.radians(cfg.lidar_dtheta_v_deg),
                drone_radius=float(getattr(cfg, "drone_radius", 0.0)))
            dist = distance_field.compute(collision, cfg)
            # §53 双缓冲：发布不可变快照（occ/dist 均为碰撞层）
            self._snapshot = PlanSnapshot(
                occ=collision.data.copy(),
                origin=collision.origin.copy(),
                dist=dist,
                state_pos=state.position.copy(),
                ts=time.time(),
            )
            # 录制：下采样后的碰撞层 occupied（供浏览器显示实际被规划避让的障碍）
            if self.recorder is not None:
                self._record_map_snapshot(state.position, collision)
            time.sleep(max(0.0, period - (time.time() - t0)))

    def _record_map_snapshot(self, center, collision, stride: int = 6):
        """下采样碰撞图并录制 occupied 体素中心（世界系）。

        只录碰撞层 occupied（真实命中 + 传感器脚印膨胀），不录 unknown——规划范围
        （=局部体素图尺寸）由浏览器按 ``recorder.map_size`` 画感知框表示，超出即视为未知。
        """
        from .occupancy import OCCUPIED
        sub = collision.data[::stride, ::stride, ::stride]
        idx = np.argwhere(sub == OCCUPIED)
        if idx.shape[0] == 0:
            return
        world = collision.origin + (idx.astype("float64") * stride + 0.5) * collision.res
        self.recorder.record_map(time.time(), world)

    def _planning_loop(self):
        cfg = self.cfg
        period = 1.0 / max(1.0, cfg.planning_hz)
        prev_ctrl = None
        replan_frac = float(getattr(cfg, "replan_fraction", 1.0 / 3.0))
        while not self._stop.is_set():
            t0 = time.time()
            snap = self._snapshot
            if snap is None or self.goal is None:
                time.sleep(period)
                continue
            # 降低决策频率：当前轨迹未执行满 replan_fraction 时跳过重规划，
            # 让已规划的轨迹真正被执行（避免每周期重置导致慢爬）。
            traj_now = self.coordinator.traj
            if traj_now is not None:
                progress = (time.time() - self.coordinator.t0) / max(traj_now.total_time, 1e-6)
                if progress < replan_frac:
                    time.sleep(period)
                    continue
            # 拼接点状态（连续重规划：继承旧轨迹在 t_s=now+margin 处的位置）。
            # 提前到 A* 之前：让 A* 起点 = 拼接点、速度方向 = 当前速度，路径起步沿
            # 速度方向延伸而非垂直于速度（消除大幅摆动/脱离路线，用户 m02861 诉求①）。
            try:
                p_s, v_s, a_s = self._splice_state(snap, time.time())
            except Exception:
                p_s, v_s, a_s = snap.state_pos, np.zeros(3), np.zeros(3)
            goal = self._local_goal(snap.state_pos, self.goal, snap.origin,
                                    cfg.res, cfg.nx, cfg.ny, cfg.nz)
            # 粗网格 A*（§7/§24 粗路径，plan_res 提速）
            coarse_map = self._coarsen(snap.occ, snap.origin, cfg.res, cfg.plan_res)
            coarse_dist = distance_field.compute(coarse_map, cfg)
            # 局部目标常落在 UNKNOWN（超出 30m 感知半径），会令 A* 展开爆炸→无路径；
            # 回拉到 FREE 且净空 ≥ d_safe 后 A* 只在已感知空间内规划。
            goal = self._project_goal_free(coarse_map, coarse_dist, snap.state_pos,
                                           goal, min_cl=float(cfg.d_safe),
                                           d_min=float(cfg.d_min))
            w_dir = float(getattr(cfg, "w_dir", 0.0))
            path = self.planner.plan(coarse_map, coarse_dist, p_s, goal,
                                     v_dir=v_s, w_dir=w_dir)
            if path is None or len(path) < 2:
                self._astar_fail += 1
                if self._astar_fail % 25 == 1:
                    occ_n = int((coarse_map.data == 2).sum())
                    print(f"[Planning] A* 无路径 (第{self._astar_fail}次) "
                          f"pos={np.round(snap.state_pos,2)} goal={np.round(goal,2)} "
                          f"occ={occ_n} unk={int((coarse_map.data==0).sum())} "
                          f"free={int((coarse_map.data==1).sum())}",
                          flush=True)
                time.sleep(period)
                continue
            self._astar_ok += 1
            if self._astar_ok % 50 == 1:
                print(f"[Planning] A* 成功 (第{self._astar_ok}次) len={len(path)} "
                      f"pos={np.round(snap.state_pos,2)}", flush=True)
            # 拼接点位置已在上面提前获取（A* 需要），此处直接用它拟合 B-spline。
            try:
                # B-spline 拟合 A* 路径：起点继承当前速度/加速度（p_s/v_s/a_s 精确编码进
                # Q0/Q1/Q2），速度连续无突变→消除大幅摆动/脱离路线；控制点数自适应弧长
                # 密集贴合航点，只润滑拐角、不把绕行拉直切障碍（见 fit_and_optimize）。
                traj = bspline.fit_and_optimize(path, coarse_dist, coarse_map, cfg,
                                                p_s, v_s, a_s, goal, prev_ctrl)
                if self._astar_ok % 3 == 1:
                    print(f"[PlanDebug] p_s={np.round(p_s,2)} traj0={np.round(traj.eval(0.0)[0],2)} "
                          f"traj1={np.round(traj.eval(1.0)[0],2)} total_t={traj.total_time:.1f} "
                          f"path0={np.round(path[0],2)} statepos={np.round(snap.state_pos,2)}",
                          flush=True)
                if traj is not None:
                    prev_ctrl = traj.ctrl
                    now = time.time()
                    self.coordinator.set_trajectory(traj, now)
                    self._last_plan_vstart = float(np.linalg.norm(traj.eval(0.0)[1]))
                    self._plan_log.append({
                        "t": now,
                        "pos": np.array(snap.state_pos).copy(),
                        "goal": np.array(goal).copy(),
                        "path": [np.array(p) for p in path] if path else [],
                        "ctrl": np.array(traj.ctrl).copy(),
                        "dt": float(traj.dt),
                    })
                    if self.recorder is not None:
                        # 采样折线轨迹（位置）供可视化
                        n_s = 40
                        tt = np.linspace(0.0, traj.total_time, n_s)
                        traj_pts = np.array([traj.eval(t)[0] for t in tt])
                        self.recorder.record_plan(
                            now, goal,
                            [np.array(p) for p in path] if path else [],
                            traj_pts, traj.ctrl)
            except Exception:
                print("[Planning] 轨迹构建/录制异常：")
                traceback.print_exc()
            # 到达判据
            if np.linalg.norm(snap.state_pos - self.goal) < cfg.goal_tol:
                self.reached = True
            time.sleep(max(0.0, period - (time.time() - t0)))

    def _splice_state(self, snap, now):
        """返回拼接点状态 (p_s, v_s, a_s)。

        优先从旧轨迹在 ``t_splice = 已执行时间 + splice_margin`` 处取值（覆盖规划延迟、
        保证 p/v/a 连续）；旧轨迹耗尽或无旧轨迹时回退到载具当前状态。

        卡墙保护：若无人机实际位置远离旧轨迹拼接点（被障碍挡住、没能沿轨迹前进），
        旧轨迹的未来点会落在无人机"够不到"的地方，新轨迹从那里起步会把无人机反向
        拉向障碍（例如卡墙时反复朝墙里顶）。此时丢弃拼接、改从实际位置起步。
        """
        margin = float(getattr(self.cfg, "splice_margin", 0.1))
        old = self.coordinator.traj
        if old is not None:
            t_old = now - self.coordinator.t0
            t_splice = t_old + margin
            if t_splice < old.total_time:
                p_s, v_s, a_s = old.eval(t_splice)
                state = self._latest_state
                if state is not None and \
                        float(np.linalg.norm(state.position - p_s)) > 2.0:
                    # 卡墙/被挡：拼接点离实际位置过远，回退到实际状态
                    return state.position, state.velocity, state.acceleration
                return p_s, v_s, a_s
        state = self._latest_state
        if state is not None:
            return state.position, state.velocity, state.acceleration
        return snap.state_pos, np.zeros(3), np.zeros(3)

    def _control_loop(self):
        cfg = self.cfg
        period = 1.0 / max(1.0, cfg.control_hz)
        while not self._stop.is_set():
            t0 = time.time()
            try:
                state = self.io.get_state()
            except Exception:
                time.sleep(period)
                continue
            self._latest_state = state
            if self.recorder is not None:
                self.recorder.record_state(time.time(), state.position,
                                           state.velocity, state.acceleration,
                                           state.yaw)
            if state.collision:
                self.collided = True
            # 更新当前净空（从快照距离场查当前位置）
            snap = self._snapshot
            if snap is not None:
                c = bspline._dist_at(state.position, snap.dist, snap.origin, cfg.res)
                self.coordinator.set_min_clearance(c)
            res = self.coordinator.tick(state, time.time())
            self._record(state, res)
            self._ctrl_n += 1
            if self._ctrl_n % 100 == 1:  # ~每 2s 一次心跳
                u = res["u"]
                print(f"[Control] pos={np.round(state.position,2)} "
                      f"|v|={float(np.linalg.norm(state.velocity)):.2f} "
                      f"u=({u[0]:.2f},{u[1]:.2f},{u[2]:.2f}) "
                      f"braking={res['braking']} clr={res['clearance']:.2f}",
                      flush=True)
            time.sleep(max(0.0, period - (time.time() - t0)))

    # ------------------------------------------------------------------
    # 辅助
    # ------------------------------------------------------------------
    @staticmethod
    def _local_goal(current, goal, origin, res, nx, ny, nz):
        # 夹到地图内部（两端各留一个体素余量），避免落到边界外（体素索引 nx 越界）
        lo = origin + res
        hi = origin + (np.array([nx, ny, nz]) - 1) * res
        return np.clip(np.asarray(goal, dtype="float64"), lo, hi)

    @staticmethod
    def _coarsen(occ, origin, fine_res, plan_res):
        """把精细占据图降采样到粗网格（occupied 优先），返回粗 OccupancyMap。"""
        ratio = plan_res / fine_res
        r = int(round(ratio))
        if r < 1:
            r = 1
        eff_res = r * fine_res
        fnx, fny, fnz = occ.shape
        cnx, cny, cnz = fnx // r, fny // r, fnz // r
        cropped = occ[:cnx * r, :cny * r, :cnz * r]
        blocks = cropped.reshape(cnx, r, cny, r, cnz, r)
        coarse = blocks.max(axis=(1, 3, 5))  # OCCUPIED(2) 优先
        cm = OccupancyMap(cnx, cny, cnz, eff_res, tuple(origin))
        cm.data = coarse
        return cm

    @staticmethod
    def _project_goal_free(occ_map, dist_field, state_pos, goal, min_cl=0.0, d_min=0.0):
        """把局部目标投影到"可达 + FREE + 净空足够"的体素（receding-horizon）。

        仅当局部目标本身在无人机可达连通分量内时才直接返回它；否则在**可达
        分量**里挑一个离全局目标最近的**前沿点**（FREE 且邻接未探索 UNKNOWN
        或地图边），让 A* 绕开障碍向目标推进。

        相比沿连线回拉更稳健：目标被建筑墙挡住时，连线回拉会停在墙面前
        （目标=墙前一点，无人机已到 → 原地踏步），前沿投影则让 A* 从墙两侧
        的街道绕过去继续前进。
        ``min_cl`` 要求目标点距障碍 ≥ min_cl（默认 d_safe），避免贴边/翻边。
        """
        from .occupancy import FREE, UNKNOWN, OCCUPIED
        from scipy import ndimage

        sv = occ_map.world_to_voxel(state_pos)
        gv = occ_map.world_to_voxel(goal)
        si, sj, sk = int(sv[0]), int(sv[1]), int(sv[2])
        if not occ_map.in_bounds(si, sj, sk):
            return state_pos

        data = occ_map.data
        if data[si, sj, sk] == OCCUPIED:
            # 无人机太贴墙、自身体素被膨胀占据：找最近的 FREE 体素当"可达种子"
            # （优先净空 ≥ min_cl 的，退而求其次任意 FREE），引导它退回安全区。
            free_idx = np.argwhere(data == FREE)
            if free_idx.size == 0:
                return state_pos
            d2 = np.sum((free_idx - np.array([si, sj, sk])) ** 2, axis=1)
            safe = free_idx[dist_field[free_idx[:, 0], free_idx[:, 1], free_idx[:, 2]] >= min_cl]
            if safe.size:
                n = safe[np.argmin(np.sum((safe - np.array([si, sj, sk])) ** 2, axis=1))]
            else:
                n = free_idx[np.argmin(d2)]
            si, sj, sk = int(n[0]), int(n[1]), int(n[2])

        # 可达分量：严格对齐 A* 的可通行语义 —— OCCUPIED 不可通行；FREE 需净空 ≥ d_min
        # （A* 对 d<d_min 的 FREE 硬截断为 INF）；UNKNOWN 可通行（cost=c_unknown）。
        # 之前用 data!=OCCUPIED 会把「膨胀后净空 <d_min 的窄缝」也当可达，导致投影选到
        # 墙后 A* 实际穿不过的点（本 bug 根因）。
        traversable = (data != OCCUPIED) & (dist_field >= d_min)
        lbl, _ = ndimage.label(traversable)
        reach = lbl == lbl[si, sj, sk]

        # 目标本身可达 + FREE + 净空够 → 直接用
        gi, gj, gk = int(gv[0]), int(gv[1]), int(gv[2])
        if (occ_map.in_bounds(gi, gj, gk) and reach[gi, gj, gk]
                and data[gi, gj, gk] == FREE
                and float(dist_field[gi, gj, gk]) >= min_cl):
            return goal

        # 候选 = 可达 + FREE + 净空够（净空不满足时放宽为可达 + FREE，避免卡死）
        cand = reach & (data == FREE) & (dist_field >= min_cl)
        if not cand.any():
            cand = reach & (data == FREE)
        if not cand.any():
            return state_pos

        # 前沿 = 候选中邻接 UNKNOWN 或地图边的体素（优先向未探索空间推进）
        nb_unk = ndimage.maximum_filter(
            (data == UNKNOWN).astype("uint8"), size=3, mode="constant", cval=1)
        frontier = cand & (nb_unk > 0)
        use = frontier if frontier.any() else cand

        idx = np.argwhere(use)
        world = occ_map.origin + (idx + 0.5) * occ_map.res
        d2 = np.sum((world - np.asarray(goal, dtype="float64")) ** 2, axis=1)
        bi = int(np.argmin(d2))
        return occ_map.voxel_to_world(int(idx[bi, 0]), int(idx[bi, 1]), int(idx[bi, 2]))

    def _record(self, state, res):
        v_actual = float(np.linalg.norm(state.velocity))
        v_cmd = float(np.linalg.norm(res.get("u", [0, 0, 0])))
        a = state.acceleration
        with self._telemetry_lock:
            self._telemetry.append({
                "t": time.time(),
                "x": float(state.position[0]),
                "y": float(state.position[1]),
                "z": float(state.position[2]),
                "vx": float(state.velocity[0]),
                "vy": float(state.velocity[1]),
                "vz": float(state.velocity[2]),
                "v_actual": v_actual,
                "v_cmd": v_cmd,
                "ax": float(a[0]),
                "ay": float(a[1]),
                "az": float(a[2]),
                "a_mag": float(np.linalg.norm(a)),
                "v_plan_start": self._last_plan_vstart,
                "clearance": res["clearance"],
                "braking": res["braking"],
                "goal_dist": (float(np.linalg.norm(state.position - self.goal))
                              if self.goal is not None else None),
                "collision": state.collision,
            })

    def telemetry(self):
        with self._telemetry_lock:
            return list(self._telemetry)

    def plan_log(self):
        return list(self._plan_log)
