# 关键逻辑与回退点（2026-09-26 良好基线）

> 本文件记录当前"已知良好"状态的核心逻辑与设计约束，作为后续修改的护栏与回退基准。
> 任何改动若触碰下述 **⚠️ 不可回退的约束**，请先在此登记动机与验证结果。
> 代码级回退：`git log` / `git checkout <commit>`（本目录已 git init，基线见首次提交）。

## 0. 当前状态快照

- 起点 (0,0,-0.5)、目标 (900,20,-20)，巡航高 10m、目标高 20m。
- 6 个离线测试全通过：`test_occupancy / test_raycast / test_distance_field / test_astar / test_bspline / test_coordinator`。
- C++ 扩展 `_fast`（pybind11）已编译并随源码同步（`cpp/fast_kernels.cpp` + `cpp/fast_planning.cpp`）。
- 三处「速度方向 + 更新频率 + 平滑更新」修复已落地（见 §3）。

## 1. 架构总览（四线程 + 双缓冲）

```
AirSim State + LiDAR  →  Local Occupancy(3D 体素)  →  Truncated Distance Field
        →  A*(3D)  →  B-spline 轨迹  →  Unified Control Coordinator(单一写者)
```

四线程：Sensor(5Hz) / Mapping(10Hz) / Planning(10Hz) / Control(50Hz)。
Mapping 每轮发布不可变 `PlanSnapshot(occ=collision.data, origin, dist, state_pos, ts)`，
Planning 只读快照（§53 双缓冲），避免读到 Mapping 半写状态。

## 2. 核心模块与关键逻辑

### 2.1 三态体素图（`nav/occupancy.py`）
- `UNKNOWN=0 / FREE=1 / OCCUPIED=2`。滚动窗口 `recenter` 用 `np.roll` 整格平移，`center_offset` 前向偏置（X 方向留 `map_back_x=10m` 后端、余量全给前方）。
- ⚠️ **两图分离（关键约束）**：`raw` 层只存真实观测（DDA 射线 ray→Free / hit→Occupied），**永不原地膨胀**；`collision` 层每周期 `derive_collision()` 从 raw 派生（复制 + 传感器脚印膨胀 + 无人机半径），膨胀**不写回 raw**，故不累积。规划/距离场只用 collision 层。

### 2.2 可见自由 `mark_visible_free(center, max_range)`（`nav/occupancy.py`）
- ⚠️ **语义**：在**非占据空间 (FREE+UNKNOWN)** 上 `ndimage.label` 26 连通，取载具所在分量，把分量内 `max_range` 内的 UNKNOWN 一次性标 FREE。OCCUPIED 阻断连通，故障碍后方保持 UNKNOWN（不穿墙）。
- **为什么**：旧实现只在 `free_only` 上 label + 扩 1 层，导致天空(无回波方向)长期 UNKNOWN，A*/前沿投影代价不对称把 goal 拉向低空贴地（run_00 实测 plan goal z 一度=-1.0）。改为开放空域整片标 FREE 后，消除"头顶天空 UNKNOWN→A* 向下钻"的偏置。
- ⚠️ 已删除 `mark_free_within_range`（球形 free）——其语义错（障碍后方也会被标 free）。

### 2.3 距离场（`nav/distance_field.py`）
- `scipy.distance_transform_edt` 截断 `d_max`（当前 6.0）。Unknown 不参与距离场。
- ⚠️ **`_dist_at` 越界回退必须 `1e9`（`nav/bspline.py:131`，C++ 同步 `cpp/fast_planning.cpp`）**：越界=远离障碍=不惩罚，值必须 ≥ `d_safe`(5.0)。旧值 `4.0 < 5.0` 会让 `_psi(4.0,5.0)=(5-4)²=1` 产生虚假障碍惩罚，把贴地图底边界的整条轨迹顶起（曾导致终点 z 偏移 1.355m）。这是 bug，改回 4.0 会复发。

### 2.4 A*（`nav/astar.py` + `cpp/fast_planning.cpp` `astar_plan`）
- 26 连通，代价：OCCUPIED=∞（不扩展）、`d<d_min` 硬截断（FREE 与 UNKNOWN 一视同仁，防穿过未观测的墙）、UNKNOWN=`c_unknown + w_obs·φ(D)`、FREE=`c_free + w_obs·φ(D)`。
- `φ(D) = exp(d_safe - D) - 1`（指数距离惩罚，`d_safe=5`：5m→0 / 4m→1.72 / 3m→6.39 / 2m→19.1 / 1m→53.6）。比二次陡，让 A* 强烈远离建筑。
- ⚠️ **`h_weight` 必须 ≤ `c_free`(=1.0) 才可采纳/一致**。曾设 5.0 使 A* 贪心、中和了障碍距离惩罚（贴墙走最短路径），已改回 1.0。
- **速度方向引导（§3 修复①）**：`plan(..., v_dir, w_dir)` 归一化速度方向，起点节点邻居方向与速度夹角越大、边代价附加 `w_dir·(1-cosθ)·res`。速度近零或未传时自动失效。C++ 与 Python 回退实现**必须同步**。
- `start/goal` 若被膨胀占据或净空不足，`_nearest_clear` 回退到半径 6 内最近净空体素。

### 2.5 B-spline（`nav/bspline.py` + `cpp/fast_planning.cpp` `bspline_optimize`）
- 均匀三次，末端钳制（复制末控制点两份 → `p(T)=Q_{n-1}`、`v(T)=0`），起点**不**钳制。
- 起点状态编码：`encode_initial_state(p_s,v_s,a_s,dt)` 把拼接状态编码进 Q0/Q1/Q2（优化时 `lock_first=3` 锁死），新轨迹起点以当前速度/加速度平滑延续。
- 代价 J = `w_goal·J_goal + w_obs·J_obs + w_hard·J_hard + w_smooth·J_smooth + w_dyn·J_dyn`。
- ⚠️ **控制点数自适应弧长**：`n = clamp(round(total/spacing)+3, n_ctrl, max_ctrl)`，`spacing=1.2`。固定 20 点会把长绕行"拉直"切过障碍——这是历史上弃用 B-spline 的根因；自适应密度后曲线贴合 A* 折线。
- ⚠️ **warm start 按弧长插值**：旧控制点视为 `[0,total]` 均匀分布，`np.interp` 到新 `free_targets`，取 0.5 权重混合。控制点数随弧长变化，不能做 `shape==` 精确匹配（旧实现失效点）。
- ⚠️ **`_psi` 与 `_dist_at` 一致性**：`_psi(d,d_safe)=(d_safe-d)²`，依赖 `_dist_at` 越界返回 1e9 才不会误判。

### 2.6 规划循环（`nav/pipeline.py` `_planning_loop`）
顺序：`_splice_state`(提前到 A* 前) → `_local_goal`(夹到图内) → `_coarsen`(降采样到 plan_res) → `_project_goal_free` → `A* plan(p_s, v_dir=v_s)` → `bspline.fit_and_optimize(p_s,v_s,a_s,prev_ctrl)` → `set_trajectory`。
- ⚠️ **A* 起点 = 拼接点 `p_s`，速度方向 = `v_s`**（不是裸 `state_pos`）。这是"消除垂直起步/大幅摆动"的关键。
- `_splice_state`：旧轨迹 `t_old+margin` 处取 p/v/a；若无人机实际位置离拼接点 > 2.0m（卡墙/被挡），回退到实际状态（防反向拉向障碍）。
- `_project_goal_free`：可达分量（`data!=OCCUPIED && dist>=d_min`，与 A* 可通行语义严格对齐）内挑离全局目标最近的前沿点（FREE 且邻接 UNKNOWN/边）。目标本身可达则直接用。
- ⚠️ **更新频率 `replan_fraction=1/3`**：当前轨迹执行满 1/3 才重规划（`progress = (now-t0)/total_time`）。曾为 0.8，导致轨迹过时、摆动。

### 2.7 控制器（`nav/coordinator.py`）
- Unified Control Coordinator = 全程序唯一调用 `move*` 的对象（Single Writer）。
- PD 跟踪 + 速度/加速度投影 + 紧急制动。`kp=1.2 / kd=1.0`。

## 3. 三处「速度方向 + 更新频率 + 平滑更新」修复（用户 m02861 诉求）

1. **A* 参考速度方向**：`plan()` 加 `v_dir/w_dir` 参数（Python + C++ 同步）。`config.py` 新增 `w_dir=6.0`。实测：对角线目标下无引导首步 `[0.6,0.6]` vs 引导 +X 首步 `[0.6,0.2]`（顺速度）。
2. **更新频率 1/3**：`config.py` `replan_fraction` 0.8 → 0.333。
3. **平滑更新不跳变**：`pipeline.py` 把 `_splice_state` 提前到 A* 前，A* 起点/方向改用 `p_s/v_s`；`bspline.py` warm start 改弧长插值（支持不同控制点数）。

## 4. 关键参数（`config.py` 当前值，改动须登记）

| 参数 | 值 | 约束/说明 |
| --- | --- | --- |
| map_size | (40,40,10) | 局部体素图 (x,y,z) 米 |
| map_back_x | 10.0 | 前向偏置，后端留 10m |
| res / plan_res | 0.2 / 0.5 | 精细 / A* 粗网格 |
| d_max / d_safe / d_min | 6.0 / 5.0 / 0.8 | d_max ≥ d_safe 使 φ 能归零 |
| max_range | 30.0 | LiDAR 感知半径，与图前向余量对齐 |
| c_free / c_unknown / w_obs | 1.0 / 5.0 / 1.0 | Unknown 昂贵、Free 基准 |
| h_weight | 1.0 | ⚠️ 必须 ≤ c_free |
| w_dir | 6.0 | A* 起点速度方向引导权重 |
| w_goal / w_smooth / w_dyn | 1.0 / 0.2 / 0.4 | 平滑权重低=只润滑不抹平 |
| w_hard | 100.0 | 硬净空 barrier |
| bspline_n_ctrl / ctrl_spacing / max_ctrl | 20 / 1.2 / 60 | ⚠️ 控制点自适应弧长 |
| splice_margin | 0.1 | 拼接点前移秒 |
| v_mission / v_max / a_max | 3.5 / 6.0 / 2.0 | 动力学上限 |
| replan_fraction | 0.333 | ⚠️ 执行 1/3 就更新 |
| sensor/mapping/planning/control_hz | 5/10/10/50 | 线程频率 |

## 5. 回退与验证

- **代码回退**：`git log --oneline` 查看提交；`git checkout <commit> -- <path>` 回退单文件；`git checkout <commit>` 整体回退。
- **C++ 回退**：改 `cpp/*.cpp` 后需重跑 `cpp/build_extension.bat` 并确认 `nav/_fast.cp310-win_amd64.pyd` 已更新（注意：先停掉占用 pyd 的 venv python 进程，否则 copy 失败）。
- **回归**：改完跑 6 个离线测试 + `python -c "import nav.pipeline, nav.astar, nav.bspline, config"` 确认可导入。
- **实机验证**：`scene_test.py --start-x 0 --start-y 0 --goal-x 900 --goal-y 20 --goal-z -20 --record --stuck-timeout 20`，看 t≈30s 附近是否还有垂直起步。

## 6. 历史教训（改前必读）

- `_dist_at` 越界回退 4.0 → 1e9（否则贴地轨迹被虚假惩罚顶起）。
- `h_weight` 5.0 → 1.0（否则贪心贴墙）。
- 控制点数固定 20 → 自适应弧长（否则长绕行被拉直切障碍）。
- warm start `shape==` 精确匹配 → 弧长插值（否则点数变化时旧轨迹不继承）。
- 球形 free → 连通泛洪 `mark_visible_free`（否则天空偏置向下钻）。
- 更新频率 0.8 → 1/3（否则轨迹过时摆动）。
- A* 起点用裸 `state_pos` → 拼接点 `p_s` + 速度方向 `v_s`（否则垂直起步）。
