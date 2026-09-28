# -*- coding: utf-8 -*-
"""第一阶段集中配置。

所有可调参数集中于此，``scene_test.py`` 可用命令行参数覆盖。默认值取自方案书 §67
的推荐初始参数，其余（代价、B-spline 权重、动力学上限）按 §21/§24/§31–§35 给定。

坐标约定：全程 AirSim NED 世界系（+X 北 / +Y 东 / +Z 向下，向上飞 = z 减小）。
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict


@dataclass
class Config:
    # ---- 局部体素图（§10/§67） ----
    map_size: tuple = (40.0, 40.0, 10.0)      # (x, y, z) 米，NED 轴
    map_back_x: float = 10.0                  # 无人机在 X 方向距地图后端的距离（米）；前向余量 = map_size[0]-map_back_x
    res: float = 0.2                          # 体素边长 r_L，米
    plan_res: float = 0.5                     # 粗路径 A* 的分辨率（§7/§24 粗网格，提速）
    d_max: float = 6.0                        # 截断距离场上限（§18/§67），必须 ≥ d_safe 使 φ(D) 能归零
    d_safe: float = 5.0                       # 障碍安全距离（§20/§67），软代价 ψ(D) 生效阈值（5m 安全距离）
    drone_radius: float = 0.3                 # 无人机物理半径，米（对建筑做固定膨胀，保证机身余量）
    d_min: float = 0.8                        # 硬净空阈值，米（相对已膨胀障碍；A*/B-spline 硬约束）
    max_range: float = 30.0                   # LiDAR 最大感知距离，米（§54），与地图前向余量对齐
    self_echo_dist: float = 0.3               # 射线投射自回波过滤最小距离（滤掉载具自身，米）

    # ---- LiDAR 角分辨率（用于距离相关的传感器脚印膨胀） ----
    lidar_dtheta_h_deg: float = 1.89          # 水平角分辨率（度），=360·通道·RPS/PPS
    lidar_dtheta_v_deg: float = 1.89          # 垂直角分辨率（度），=180/(通道数-1)，96 通道均衡

    # ---- 成本场（§21/§22/§67） ----
    c_free: float = 1.0
    c_unknown: float = 5.0                    # 仅剩"障碍物背后的真·未观测区"才标 Unknown，保持昂贵
    w_obs: float = 1.0                        # 距离场障碍代价权重（φ 已含指数幅值，w_obs=1 使 5m→0/2m→19）
    w_hard: float = 100.0                     # B-spline 硬净空 barrier 权重（净空 < d_min 时二次惩罚，优化内硬约束）

    # ---- A*（§24） ----
    astar_timeout_nodes: int = 500000         # 展开节点上限（粗图 100×100×25=250k 体素，启发式低估~5× 时搜索前沿覆盖整图，须≥整图）
    h_weight: float = 1.0                     # 启发式权重。必须 ≤ c_free(=1.0) 才可采纳/一致；
                                              # 曾设 5.0 使 A* 贪心，中和了障碍距离惩罚（贴墙走最短路径），
                                              # 降到 1.0 让距离惩罚真正生效（避让更安全路径）。
    w_dir: float = 6.0                        # A* 起点速度方向引导权重（§24 补充）：起点节点邻居方向与
                                              # 当前速度夹角越大、边代价附加 w_dir*(1-cosθ)*res。让 A* 起步
                                              # 沿速度方向延伸而非垂直起步（消除大幅摆动/脱离路线）。

    # ---- B-spline（§29–§35） ----
    bspline_order: int = 3                    # 三次
    bspline_n_ctrl: int = 20                  # 控制点下界（短路径用）
    bspline_ctrl_spacing: float = 1.2         # 控制点弧长间距（米），决定贴合 A* 航点的密度：
                                              # 越小越贴合（保留拐角、不拉直绕行），越大越平滑
    bspline_max_ctrl: int = 60                # 控制点数量上限（长路径自适应的封顶，防优化过慢）
    w_goal: float = 1.0
    w_smooth: float = 0.2                     # 平滑权重（降低，只润滑拐角、不抹平折线）
    w_dyn: float = 0.4
    bspline_iters: int = 30                   # 每轮优化迭代
    bspline_step: float = 0.08                # 梯度步长
    splice_margin: float = 0.1                # 拼接点前移时间（秒），覆盖规划延迟
    smooth_skip: int = 2                      # 平滑/加速度代价跳过起点附近几个差分项（允许起点加速）

    # ---- 动力学 / 控制（§35/§44） ----
    v_mission: float = 3.5                    # 任务巡航速度，m/s
    v_max: float = 6.0                        # 载具速度上限，m/s
    a_max: float = 2.0                        # 加速度上限，m/s^2
    kp: float = 1.2                           # 轨迹跟踪位置增益
    kd: float = 1.0                           # 轨迹跟踪速度增益
    control_dt: float = 0.02                  # 控制周期，秒（50Hz）
    velocity_duration: float = 1.0            # moveByVelocityZAsync 指令保持时长（须>控制周期，否则指令过期速度归零）
    goal_tol: float = 1.0                     # 到达判据，米

    # ---- 线程频率（§52） ----
    sensor_hz: float = 5.0                     # LiDAR 点云反序列化慢，降低频率避免饿死控制环
    mapping_hz: float = 10.0
    planning_hz: float = 10.0
    control_hz: float = 50.0
    replan_fraction: float = 0.333            # 当前轨迹执行满该比例才重规划（1/3：运行 1/3 预测轨迹后就更新，
                                              # 使规划及时跟上速度/环境变化，避免轨迹过时导致摆动/撞墙）

    # ---- 仿真连接 ----
    ip: str = "127.0.0.1"
    port: int = 41451
    vehicle_name: str = ""                    # 空串 = 默认载具
    lidar_name: str = "Lidar1"                # 对应 settings.json 里的 Lidar1；未配置时自动回退深度图
    camera_name: str = "0"                    # 深度回退用的前中相机

    # ---- 起飞 / 降落 ----
    takeoff_altitude: float = 3.0             # 起飞后离地高度，米
    flight_altitude: float = 8.0              # 目标巡航高度（NED：发送时取负），米

    # ---- 杂项 ----
    seed: int = 0
    log_dir: str = "results"

    def __post_init__(self):
        self.map_size = tuple(float(v) for v in self.map_size)
        self.res = float(self.res)
        self.nx = int(round(self.map_size[0] / self.res))
        self.ny = int(round(self.map_size[1] / self.res))
        self.nz = int(round(self.map_size[2] / self.res))

    def to_dict(self) -> dict:
        d = asdict(self)
        d.pop("nx", None)
        d.pop("ny", None)
        d.pop("nz", None)
        return d

    @classmethod
    def from_overrides(cls, overrides: dict) -> "Config":
        c = cls()
        for k, v in (overrides or {}).items():
            if not hasattr(c, k):
                raise ValueError(f"未知参数: {k}")
            setattr(c, k, v)
        c.__post_init__()
        return c


def parse_overrides(argv) -> dict:
    """把 ``--key value``（可带 ``--key=value``）解析成 dict，值做类型推断。"""
    overrides = {}
    i = 0
    args = list(argv)
    while i < len(args):
        a = args[i]
        if a.startswith("--"):
            key = a[2:]
            if "=" in key:
                key, val = key.split("=", 1)
                i += 1
            else:
                if i + 1 >= len(args):
                    break
                val = args[i + 1]
                i += 2
            overrides[key] = _coerce(val)
        else:
            i += 1
    return overrides


def _coerce(val: str):
    s = val.strip()
    if s == "":
        return s
    # 元组 / 列表
    if s.startswith(("(", "[")) and s.endswith((")", "]")):
        inner = s[1:-1]
        parts = [p.strip() for p in inner.split(",") if p.strip() != ""]
        return tuple(_coerce(p) for p in parts)
    low = s.lower()
    if low in ("true", "yes"):
        return True
    if low in ("false", "no"):
        return False
    if low in ("none", "null"):
        return None
    try:
        if "." in s or "e" in low or "e-" in low:
            return float(s)
        return int(s)
    except ValueError:
        return s
