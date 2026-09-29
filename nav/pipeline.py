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

import os
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

        # 持久图（第二阶段）：SSD 稀疏块体素图先验（§4–§7/§16–§17/§59/§61）。
        # load 文件不存在/损坏 → 空图（不抛），任务照常从零建图。
        self.persistent = None
        self._persistent_path = None
        self._last_persist_save = 0.0
        self._last_persist_integrate = 0.0
        if bool(getattr(cfg, "use_persistent_map", False)):
            from .persistent_map import PersistentMap
            p_res = float(getattr(cfg, "persistent_res", 0.6))
            p_path = getattr(cfg, "persistent_map_path", "maps/persistent_map.npz")
            if not os.path.isabs(p_path):
                # 相对路径统一锚定到 NavigationPhase1 目录（与运行 CWD 无关）
                p_path = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), p_path)
            self._persistent_path = p_path
            self.persistent = PersistentMap.load(
                p_path,
                res=p_res,
                block=int(getattr(cfg, "persistent_block", 16)),
                origin=tuple(getattr(cfg, "persistent_origin", (0.0, 0.0, 0.0))))
            # 坐标对齐校验：持久图边长必须是局部图边长的整数倍
            self.persistent.validate_alignment(cfg.res)
            print(f"[PersistentMap] 已加载先验：{self.persistent.stats()}", flush=True)

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
        # 终盘：把最后一张 raw 图写回持久图并落盘（保证完整建模，最多丢 5s）
        if self.persistent is not None:
            try:
                self.persistent.merge_from_local(self.map.data, self.map.origin, self.map.res)
                self.persistent.save(self._persistent_path)
                print(f"[PersistentMap] 终盘完成：{self.persistent.stats()}", flush=True)
            except Exception as e:
                print(f"[PersistentMap] 终盘失败：{e}", flush=True)

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
            # 先验加载：把持久图 FREE/OCCUPIED 填进刚滚入的 UNKNOWN 条带（§16/§17）。
            # 只填 UNKNOWN、不改已感知区域；传感器随后覆盖先验（Sensor > Historical）。
            if self.persistent is not None:
                self.persistent.seed_local(self.map.data, self.map.origin, self.map.res)
            raycast.integrate(self.map, state.position, pts, cfg)
            # 全向建图（§54）：把所有 max_range 内的点**未裁剪**地直接写进持久图，
            # 不受局部 40×40×10 滚动窗口限制。否则窗外点（尤其高空/低空）被丢弃 →
            # 爬升/下降后 seed_local 无数据回填 → 出现 unknown（用户 m03549）。
            if self.persistent is not None:
                now = time.time()
                if now - self._last_persist_integrate >= cfg.persistent_integrate_interval:
                    try:
                        self.persistent.integrate_frame(
                            state.position, pts, cfg.max_range,
                            float(getattr(cfg, "self_echo_dist", 0.3)))
                    except Exception as e:
                        print(f"[PersistentMap] 全向建图失败：{e}", flush=True)
                    self._last_persist_integrate = now
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
            # 写回：周期性地把 raw 图的 FREE/OCCUPIED 合并进持久图（UNKNOWN 跳过，
            # OCCUPIED 优先），再原子落盘（§59：实时线程不等待磁盘，仅低频触发）。
            if self.persistent is not None:
                now = time.time()
                if now - self._last_persist_save >= cfg.persistent_save_interval:
                    self.persistent.merge_from_local(self.map.data, self.map.origin, self.map.res)
                    self.persistent.save(self._persistent_path)
                    self._last_persist_save = now
            time.sleep(max(0.0, period - (time.time() - t0)))

    def _record_map_snapshot(self, center, collision, stride: int = 6):
        """下采样碰撞图 occupied + raw 图 unknown，录制体素中心（世界系）。

        occ 录碰撞层（真实命中 + 传感器脚印膨胀，供显示实际被规划避让的障碍）；unk 录
        raw 图里 mark_visible_free 后仍 UNKNOWN 的体素（= 感知区 / A* 计算域内未观测到的
        区域，如遮挡后方）。unknown 逐帧录、降采样存储（计算仍用原密度），回放按帧显示
        （用户 m03748/m03794：全量 unknown 太卡，unknown 只出现在感知区域）。
        """
        from .occupancy import OCCUPIED, UNKNOWN
        sub = collision.data[::stride, ::stride, ::stride]
        idx = np.argwhere(sub == OCCUPIED)
        occ_world = None
        if idx.shape[0]:
            occ_world = collision.origin + (idx.astype("float64") * stride + 0.5) * collision.res
        raw_sub = self.map.data[::stride, ::stride, ::stride]
        uidx = np.argwhere(raw_sub == UNKNOWN)
        unk_world = None
        if uidx.shape[0]:
            unk_world = self.map.origin + (uidx.astype("float64") * stride + 0.5) * self.map.res
        self.recorder.record_map(time.time(), occ_world, unk_world)

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
            # 确保 A* 基于最新采样：mapping 慢（~1.4s/帧）时 planning 会反复读同一个旧
            # snapshot 规划多次，导致图滞后、规划偏移（用户 m03891 问题3）。此处仅当
            # snapshot.ts 变化（mapping 发布了新图）才继续，否则跳过等待新采样。
            if getattr(self, "_last_plan_snap_ts", None) == snap.ts:
                time.sleep(period)
                continue
            self._last_plan_snap_ts = snap.ts
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
            # 粗网格 A*（§7/§24 粗路径，plan_res 提速）
            coarse_map = self._coarsen(snap.occ, snap.origin, cfg.res, cfg.plan_res)
            # 可规划范围 > 感知范围（用户 m04144/m04146）：外圈补一圈 UNKNOWN（代价高
            # 但可达），让 goal 在 occupied 后方时仍能被投影、A* 朝它探索，而非"缺乏
            # 目标"原地徘徊。不扩大建图范围，仅扩大规划范围。
            pad_vox = int(round(float(getattr(cfg, "plan_pad", 0.0)) / coarse_map.res))
            pad_z_vox = int(round(float(getattr(cfg, "plan_pad_z", 0.0)) / coarse_map.res))
            if pad_vox > 0 or pad_z_vox > 0:
                coarse_map = self._pad_unknown(coarse_map, pad_vox, pad_z_vox)
            # 注意：pad 外扩圈保持 UNKNOWN，不回填持久图（用户 m04540 要求）。
            # UNKNOWN 圈是「辅助路线规划手段」——允许 A* 朝感知范围外探索；即使它
            # 覆盖持久图已建好的信息也没关系，无人机朝该方向探索时 raycast 会自然
            # 感知真实障碍并更新规划。仅此处为特例，其余 unknown-不覆盖-occupied 逻辑保留。
            coarse_dist = distance_field.compute(coarse_map, cfg)
            # 局部目标夹到（pad 后的）可规划范围边界，两端留一体素余量
            goal = self._local_goal(snap.state_pos, self.goal, coarse_map.origin,
                                    coarse_map.res, coarse_map.nx, coarse_map.ny, coarse_map.nz)
            # 局部目标允许落在 UNKNOWN（可通行）：goal 在 occupied 后方时投影到可达的
            # UNKNOWN 点，让 A* 有明确目标、朝 goal 方向主动探索（用户 m04146）。
            goal = self._project_goal_free(coarse_map, coarse_dist, snap.state_pos,
                                           goal, min_cl=float(cfg.d_safe),
                                           d_min=float(cfg.d_min))
            w_dir = float(getattr(cfg, "w_dir", 0.0))
            v_dir = v_s
            path = self.planner.plan(coarse_map, coarse_dist, p_s, goal,
                                     v_dir=v_dir, w_dir=w_dir)
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
        if r < 1 or not np.isclose(ratio, r):
            raise ValueError("plan_res must be an integer multiple of res")
        eff_res = r * fine_res
        fnx, fny, fnz = occ.shape
        cnx, cny, cnz = fnx // r, fny // r, fnz // r
        cropped = occ[:cnx * r, :cny * r, :cnz * r]
        blocks = cropped.reshape(cnx, r, cny, r, cnz, r)
        coarse = np.where((blocks == 2).any(axis=(1,3,5)), 2,
                          np.where((blocks == 1).all(axis=(1,3,5)), 1, 0)).astype("int8")  # OCCUPIED(2) 优先
        cm = OccupancyMap(cnx, cny, cnz, eff_res, tuple(origin))
        cm.data = coarse
        return cm

    @staticmethod
    def _pad_unknown(occ_map, pad_xy_vox, pad_z_vox=0):
        """把可规划范围外扩 UNKNOWN（可规划范围 > 感知范围，各向异性）。

        x/y 每边外扩 ``pad_xy_vox``、z 每边外扩 ``pad_z_vox`` 圈 UNKNOWN（cost=c_unknown
        代价高但可达），让 goal 在 occupied 后方时仍能被投影到可达的 UNKNOWN、A* 朝它
        探索（用户 m04144/m04146）。绕楼是水平绕行，z 通常不扩（减少体素/搜索前沿）。
        只扩大规划范围、不扩大建图范围（建图仍只在原始 map_size 内）。
        """
        from .occupancy import UNKNOWN
        d = occ_map.data
        p = int(pad_xy_vox)
        q = int(pad_z_vox)
        padded = np.full((d.shape[0] + 2 * p, d.shape[1] + 2 * p, d.shape[2] + 2 * q),
                         UNKNOWN, dtype=d.dtype)
        padded[p:p + d.shape[0], p:p + d.shape[1], q:q + d.shape[2]] = d
        origin = occ_map.origin - np.array([p, p, q], dtype="float64") * occ_map.res
        cm = OccupancyMap(padded.shape[0], padded.shape[1], padded.shape[2],
                          occ_map.res, tuple(origin))
        cm.data = padded
        return cm

    @staticmethod
    def _project_goal_free(occ_map, dist_field, state_pos, goal, min_cl=0.0, d_min=0.0):
        """把局部目标投影到「可达 + 非 OCCUPIED（FREE 或 UNKNOWN）+ 净空足够」的体素。

        仅当局部目标本身在无人机可达连通分量内（FREE 或 UNKNOWN，A* 可通行）时才直接
        返回它；否则在可达分量里挑一个离全局目标最近的点，让 A* 朝目标方向推进。

        关键（用户 m04146）：goal 可能在 occupied 后方，其位置是 UNKNOWN（LiDAR 被墙
        遮挡感知不到）。旧实现要求目标 ``data == FREE``，把 UNKNOWN 排除在目标/候选
        之外，导致 goal 在墙后时「缺乏目标」——只能选墙前 FREE 前沿点，A* 到墙前
        原地徘徊。现在允许 UNKNOWN 作为目标（代价高但可达），A* 会朝 goal 方向穿过
        UNKNOWN 主动探索、绕过障碍。
        """
        from .occupancy import FREE, OCCUPIED
        from scipy import ndimage

        sv = occ_map.world_to_voxel(state_pos)
        gv = occ_map.world_to_voxel(goal)
        si, sj, sk = int(sv[0]), int(sv[1]), int(sv[2])
        if not occ_map.in_bounds(si, sj, sk):
            return state_pos

        data = occ_map.data
        if data[si, sj, sk] == OCCUPIED or dist_field[si,sj,sk] < d_min:
            return np.asarray(state_pos).copy()

        traversable = (data != OCCUPIED) & (dist_field >= d_min)
        lbl, _ = ndimage.label(traversable)
        reach = (lbl != 0) & (lbl == lbl[si, sj, sk])

        # 目标本身可达（FREE 或 UNKNOWN，非 OCCUPIED；reach 已含净空 ≥ d_min）→ 直接用。
        gi, gj, gk = int(gv[0]), int(gv[1]), int(gv[2])
        if occ_map.in_bounds(gi, gj, gk) and reach[gi, gj, gk]:
            return goal

        # 候选：优先可达 + FREE + 净空 ≥ min_cl（安全），否则退化为所有可达点（含 UNKNOWN）。
        cand = reach & (data == FREE) & (dist_field >= min_cl)
        if not cand.any():
            cand = reach
        if not cand.any():
            return state_pos

        # 选离 goal 最近的可达点（UNKNOWN 本身即可达，无需再要求"邻接前沿"）
        idx = np.argwhere(cand)
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
