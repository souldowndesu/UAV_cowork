# 工作进度交接文档 — Phase 1 无人机避障导航

> 生成时间：2026-09-28。本文档供另一位 AI / 开发者接手本项目的导航算法迭代工作。
> 核心目标已完成大半，当前正在迭代「绕楼 + 目标到达」的效率问题。

---

## 1. 项目目标与当前迭代焦点

**总目标**：实现方案书《无人机避障方案.md》Phase 1 —— AirSim 3D 自主导航管线：

```
State + LiDAR → 3D Occupancy → 距离场(EDT) → A* 粗路径 → B-spline 轨迹 → 轨迹跟踪控制
```

**最终验收目标**：起点 (0,0) 巡航高 10m，目标 (900,20,-20)（即高度 20m），正确抵达且效率符合预期。

**当前迭代焦点**：用 `goal=200`（沿 +X 方向，途中有 x≈134.4 的高楼）反复跑，解决「绕楼 + 到达」的规划质量问题。

**坐标约定**：全程 AirSim **NED 世界系**（+X 北 / +Y 东 / +Z 向下，向上飞 = z 减小）。LiDAR 返回全局坐标（settings.json 里 `DataFrame="VehicleInertialFrame"` 实测为全局）。

---

## 2. 环境

| 项 | 值 |
|---|---|
| 工作目录 | `D:\Data\VS_code\ITS\CompetitionEnv\NavigationPhase1` |
| 仿真器 | `D:\Data\VS_code\ITS\CompetitionEnv\CityEnviron\WindowsNoEditor\CityEnviron.exe` |
| AirSim 端口 | 41451 |
| Python | `D:\Data\VS_code\ITS\CompetitionEnv\.venv\Scripts\python.exe`（NavigationPhase1 内**无独立 venv**） |
| 依赖 | `requirements.txt`（airsim, numpy, scipy, msgpack-rpc-python 等） |
| C++ 扩展 | `cpp/fast_kernels.cpp` + `cpp/fast_planning.cpp`，MSVC BuildTools + pybind11 编译，产物 `nav/_fast*.pyd` |

**注意**：`_reset.py` 重置无人机到 (0,0,-0.5)；模拟器启动后需等 30~60s 才能连接。所有 pwsh/python 子进程需要 danger-full-access 沙箱（否则 `New-Item`/目录写被 ACL 拒绝）。

---

## 3. 架构总览

### 3.1 模块（`nav/` 目录）

| 模块 | 职责 |
|---|---|
| `occupancy.py` | 三态体素图：`UNKNOWN=0 / FREE=1 / OCCUPIED=2`。`OccupancyMap(nx,ny,nz,res,origin,center_offset)`，方法 `world_to_voxel/voxel_to_world/recenter/_shift`（np.roll 滚动）。`recenter` 用 `center_offset` 放无人机（默认几何中心，可前向偏置） |
| `raycast.py` | 3D DDA（Amanatides-Woo）射线投射。`integrate(map,vehicle_pos,points,cfg)` 滤 `self_echo_dist≤dist≤max_range`，C++ `_fast.raycast_batch` 标记 ray 沿途 free、终点 occupied |
| `distance_field.py` | scipy `distance_transform_edt` 截断距离场（`d_max`）。**有 occ=0 特判**（见 §5.6） |
| `astar.py` | 26 连通 A*，Occupied=∞、Unknown=c_unknown、Free=c_free+w_obs·φ(D)，d<d_min 硬截断。C++ `_fast.astar_plan` 优先 |
| `bspline.py` | `fit_and_optimize(path,dist,map,cfg,p_s,v_s,a_s,goal,prev_ctrl)`：控制点数自适应弧长；`encode_initial_state` 把 p_s/v_s/a_s 编码进 Q0/Q1/Q2（锁死 lock_first=3）；warm start（prev_ctrl 按弧长 np.interp）；梯度下降 J=w_goal·‖c[-1]-goal‖² + w_obs·Σψ(d) + J_hard + w_smooth + w_dyn |
| `coordinator.py` | 起飞/降落/状态机 |
| `persistent_map.py` | 持久图（稀疏块，`persistent_res=0.6`、`persistent_block=16`）。`seed_local/merge_from_local/integrate_frame/occupied_points/unknown_points` |
| `recorder.py` | 回放录制（goal/states/points/plans/maps/persistent/persistent_unknown/persistent_bbox/map_size），落盘到 `results/YYYYMMDD_HHMMSS/recording.json` |
| `pipeline.py` | **核心** `NavigationPipeline`，4 线程主循环 |
| `airsim_io.py` | AirSim 连接/读取 LiDAR/teleport/moveByVelocityZ |

### 3.2 四线程（`pipeline.py`）

| 线程 | 频率 | 职责 |
|---|---|---|
| sensor | 5 Hz | 读 LiDAR 点云（反序列化慢，故意降频） |
| mapping | 10 Hz | 见下方 `_mapping_loop` 顺序 |
| planning | 10 Hz | 见下方 `_planning_loop` 顺序 |
| control | 50 Hz | 轨迹跟踪（PD 控制器） |

**`_mapping_loop` 顺序**（每周期）：
```
recenter → seed_local(局部图从持久图填 UNKNOWN) → raycast.integrate(局部图)
→ integrate_frame(全向建图, 每1s, 未裁剪写持久图) → mark_visible_free
→ derive_collision(sensor footprint + drone_radius 膨胀, 不写回 raw)
→ distance_field.compute(collision) → self._snapshot=PlanSnapshot(原子发布)
→ _record_map_snapshot(录制) → merge_from_local+save(每5s 落盘)
```

**`_planning_loop` 顺序**（每周期，仅当 mapping 发布新 snapshot 才规划）：
```
读 self._snapshot (snapshot.ts 判新) → replan_fraction 检查
→ _splice_state(拼接点) → _coarsen(粗网格 plan_res) → _pad_unknown(外扩 UNKNOWN)
→ (注意：此处已删 seed_local 回填，见 §5.7)
→ distance_field.compute → _local_goal(clip) → _project_goal_free(允许 UNKNOWN)
→ planner.plan(A*) → bspline.fit_and_optimize
```

---

## 4. 当前实现状态（已完成）

- ✅ 三态体素图 + 两图分离（raw 只存真实观测 / collision 派生膨胀，不写回 raw）。
- ✅ 全向建图：所有 LiDAR 点未裁剪写持久图（`integrate_frame`，1s 周期），局部图 40×40×10 只做规划计算。
- ✅ `mark_visible_free`：26 连通泛洪，消除「天空=Unknown 比地面=Free 贵」偏置，保留遮挡后方 UNKNOWN。
- ✅ goal 投影允许 UNKNOWN（`_project_goal_free` 判据 `reach` 而非 `data==FREE`）。
- ✅ 可规划范围 > 感知范围（`plan_pad` 外圈补 UNKNOWN，z 不扩）。
- ✅ 持久图周期刷新到 recorder（autosave 前 `set_persistent`，防 final save 截断丢 persistent）。
- ✅ 规划等最新采样（snapshot.ts 判新，防旧图反复规划）。
- ✅ replay 三态显示（occupied 红 / unknown 灰 / free 不标记），persistent 青色。
- ✅ 8 个离线测试全通过（`tests/test_{occupancy,raycast,distance_field,astar,bspline,coordinator,recorder,persistent_map}.py`）。
- ✅ 绕楼成功过（seed_local 版本，y=53 绕北缺口 y∈[42,54]）。

---

## 5. 关键决策与根因历史（**接手必读，避免重复踩坑**）

### 5.1 两图分离（raw / collision）
raw 层只存真实观测（DDA ray→Free / hit→Occupied，**永不做原地膨胀**，recenter 滚动保留）；collision 层每周期从 raw `derive_collision()` 派生（复制 + 膨胀，膨胀不写回 raw，故不累积）。`mark_free_within_range`（球形 free）已删除——语义错（障碍后方也会被标 free）。

### 5.2 mark_visible_free（替代球形 free）
用 scipy `ndimage.label` 26 连通，从无人机体素泛洪，把 max_range 内、与无人机连通（不被 occupied 阻断）的 UNKNOWN 标 FREE。消除「天空=Unknown 比地面=Free 贵→A* 向下钻」偏置，同时保留遮挡后方 UNKNOWN。耗时 ~23ms。

### 5.3 全向建图（高度变化后高处出现 UNKNOWN 的根因）
- 局部图 map_size=(40,40,10)m，z 窗只有 10m；LiDAR max_range=30m 全向（360°×180°）。
- 旧代码把点转局部图体素后 DDA，窗外点直接丢弃 → 低空扫到的高空建筑被丢，爬升后高处 UNKNOWN。
- **修复**：`persistent_map.integrate_frame` 把所有 max_range 内点**未裁剪** DDA 写持久图（构造对齐持久图栅格的稠密缓冲 + C++ DDA + merge_from_local）。
- 原则（用户 m03548）：所有点都参与建图；只有感知范围内（局部图）的点做规划计算；体素膨胀是「确定点之后」的独立后处理（碰撞图），不进建图。

### 5.4 goal 投影允许 UNKNOWN（用户 m04144/m04146）
goal 在 occupied（高楼）后方时，goal 位置是 UNKNOWN（LiDAR 被墙挡），旧 `_project_goal_free` 要求 `data==FREE` → 找不到目标 → 墙前徘徊。**修复**：goal 判据从 `reach & data==FREE` 改为 `reach`（允许 UNKNOWN，reach 已含非 OCCUPIED + dist≥d_min）；候选同理退化。`plan_pad` 外圈补 UNKNOWN 让可规划范围略大于感知范围，A* 朝 goal 探索。

### 5.5 A* 无路径根因（occ=0 开阔地也报无路径）
1. **scipy EDT 全 True bug**：occ=0 时 `distance_transform_edt(~occupied)` 对全 True 输入返回「到 (0,0,0) 角的伪距离」而非 inf/d_max。**修复**：`distance_field.compute` 开头 `if not occupied.any(): return np.full(shape, d_max)`。
2. **plan_pad 各向同性搜索爆炸**：旧 `_pad_unknown` 各向同性，z 方向 25→85 使粗图到 (160,160,85)=217.6万体素；goal 在 UNKNOWN 外圈，h=c_free=1 低估 c_unknown=5 代价 → 搜索前沿覆盖整图 → 耗尽 `astar_timeout_nodes`。**修复**：`plan_pad_z=0.0`（z 不扩）+ `astar_timeout_nodes=1500000`。
3. **`_coarsen` 精度 bug**：`ratio=plan_res(0.5)/fine_res(0.2)=2.5`，`r=round(2.5)=2`，故 coarse 实际 `eff_res=0.4`（≠plan_res 0.5）。`pad_vox` 必须用 `coarse_map.res`（不是 plan_res）计算。

### 5.6 绕行侧不稳定（楼两侧缺口代价相近）
楼 A（x=134.7 墙，y∈[-4,40]，南翼 y∈[-6,-4]，北翼 y∈[40,42]）两侧缺口 y<-9.6 与 y>48 代价相近，A* 每次独立规划会在「绕南/绕北」间切换，导致 B-spline 轨迹摆动。曾试「趋势继承」（`_last_path_dir` 记录上次 path 方向），用户 m04130 质疑有效性，已回退。

### 5.7 穿墙问题与 pad 区 UNKNOWN（用户 m04540，**当前最终状态**）
- **穿墙根因**：规划范围（pad 后）> 感知范围（局部图 40×40×10）。局部图 recenter 后 y∈[0,40]，楼墙两端（y<0 / y>40）被裁掉，pad 区是纯 UNKNOWN → A* 绕楼时看不到墙两端 → 规划「穿墙」→ B-spline J_hard 拉回 → 楼前徘徊。
- 曾用 `persistent.seed_local(coarse_map.data, ...)` 把持久图回填进 pad 区修复（绕楼成功），但用户 m04540 要求**回退**：
  > 「这部分 unknown 可以持久存在，即使已经建立了持久图，作为一个辅助路线规划的手段，即使 unknown 覆盖了原本的持久图，但是只要往这边走，自然能加载持久图。当然只有这个是特例，原先的其它逻辑依然保留。不需要一定稳定，只需要完成这一轮的修改即可。」
- **最终状态**：`pipeline.py _planning_loop` 里 **pad 外扩圈保持 UNKNOWN 不回填持久图**（代码里留注释说明）。仅此处为特例，其余「unknown 不覆盖 occupied」逻辑（mark_visible_free / raycast 写回 / derive_collision 膨胀）全部保留。

---

## 6. 当前未解决问题

1. **绕楼后 y 偏、不回到 goal y=20**：seed_local 版本绕北（y=53）成功后，无人机 y 持续偏北（56~60），没回到 goal y=20，最终在 x≈217 撞第二栋楼 stuck（终点距 goal 40.9m）。
2. **第二栋楼几何**：persistent 数据显示 x∈[170,260] 有障碍带（如 x∈[190,230] 有 y∈[-10,-5) 和 y∈[55,60) 两条带），goal y=20 在中间开阔带，但无人机绕北后偏北撞上 y∈[55,60) 障碍带。
3. **绕行侧不稳定**（§5.6，未解决）：绕南/绕北切换导致轨迹摆动。

---

## 7. 如何运行与测试

```powershell
# 启动仿真器（后台，等 30~60s）
& 'D:\Data\VS_code\ITS\CompetitionEnv\CityEnviron\WindowsNoEditor\CityEnviron.exe'

# 重置无人机
& 'D:\Data\VS_code\ITS\CompetitionEnv\.venv\Scripts\python.exe' _reset.py

# 重跑（goal=200 绕楼测试）
& 'D:\Data\VS_code\ITS\CompetitionEnv\.venv\Scripts\python.exe' scene_test.py `
  --start-x 0 --start-y 0 --goal-x 200 --goal-y 20 --goal-z -20 `
  --flight_altitude 10 --record --stuck-timeout 20

# 最终验收（goal=900）
# ... scene_test.py --start-x 0 --start-y 0 --goal-x 900 --goal-y 20 --goal-z -20 --flight_altitude 10 --record

# 回放
& 'D:\Data\VS_code\ITS\CompetitionEnv\.venv\Scripts\python.exe' replay.py

# 离线测试（venv 无 pytest，每个文件有 __main__ 自测）
& 'D:\Data\VS_code\ITS\CompetitionEnv\.venv\Scripts\python.exe' tests/test_astar.py
& 'D:\Data\VS_code\ITS\CompetitionEnv\.venv\Scripts\python.exe' tests/test_persistent_map.py
# ... 共 8 个 tests/test_*.py
```

**scene_test.py 参数**：`--start-x/-y/-z(默认0)/-yaw(默认0)/--goal-x/-y/-z(默认 -flight_altitude)/--plot/--record/--visualize/--viz-port(8765)/--interactive/--runs(1)/--stuck-timeout(30)`；`--key value` 可覆盖任意 config 参数（如 `--flight_altitude 10`）。输出目录 `results/YYYYMMDD_HHMMSS/recording.json`。

---

## 8. 代码关键位置索引

| 位置 | 内容 |
|---|---|
| `config.py:17-22` | map_size=(40,40,10), map_back_x=10, res=0.2, plan_res=0.5, d_max=6.0, d_safe=5.0 |
| `config.py:39` | astar_timeout_nodes=1500000 |
| `config.py:46-52` | plan_pad=15.0（x/y）, plan_pad_z=0.0（z 不扩） |
| `config.py:98-104` | 持久图：persistent_res=0.6, persistent_block=16, persistent_integrate_interval=1.0 |
| `config.py:83` | replan_fraction=0.333 |
| `nav/pipeline.py` `_mapping_loop` | recenter→seed_local→raycast.integrate→integrate_frame→mark_visible_free→derive_collision→distance_field→snapshot |
| `nav/pipeline.py` `_planning_loop` | 读 snapshot→_coarsen→_pad_unknown（**不回填持久图**）→distance_field→_project_goal_free→A*→bspline |
| `nav/pipeline.py` `_project_goal_free` | goal 判据允许 UNKNOWN（reach） |
| `nav/pipeline.py` `_pad_unknown` | 各向异性外扩（pad_xy_vox, pad_z_vox），origin 偏移 [p,p,q]*res |
| `nav/pipeline.py` `_coarsen` | blocks.max OCCUPIED 优先，r=round(plan_res/fine_res)，eff_res=r*fine_res |
| `nav/persistent_map.py` `integrate_frame` | 全向建图（未裁剪 DDA 写持久图） |
| `nav/persistent_map.py` `seed_local` | 只填 local==UNKNOWN（体素中心点查询持久图） |
| `nav/persistent_map.py` `merge_from_local` | 只写 FREE/OCCUPIED（OCCUPIED 优先） |
| `nav/occupancy.py` `mark_visible_free` | 26 连通泛洪标 FREE（OCCUPIED 阻断） |
| `nav/distance_field.py` `compute` | occ=0 特判返回全 d_max |
| `nav/bspline.py` `fit_and_optimize` | 自适应控制点 + warm start + 梯度下降 |
| `run_phase1.py` `run_mission` | autosave 前 set_persistent（周期刷新 persistent 到 recorder） |
| `web/viewer.html` | replay 三态渲染（occupied 红/unknown 灰/free 透明，persistent 青） |

---

## 9. 临时诊断脚本（scratch/ 目录，可清理，非核心）

迭代过程中的临时诊断脚本已整理到 `scratch/` 目录（34 个 `_*.py` + `_diag_wall2.out`），包括：
`_analyze_*.py`（楼几何/净空分析）、`_diag_*.py`（各种诊断）、`_repro*.py`、`_scan*.py`、
`_test_*.py`（`_test_integrate.py`/`_test_pad_seed.py`/`_test_goal_unknown.py` 等）、`_verify_unknown_fix.py`、`_monitor.py`。

这些脚本记录了诊断过程，不参与主流程，删除不影响运行。
`_reset.py` 保留在根目录（重置无人机到起点，scene_test 流程会用到）。
