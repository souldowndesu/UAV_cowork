# NavigationPhase1 —— 无人机自主导航 · 第一阶段 AirSim 实现

独立实验项目（无 LLM / 无 Agent / 无持久化地图），实现方案书《无人机避障方案》§60 的
第一阶段实时导航闭环：

```
AirSim State + LiDAR  →  Local Occupancy(3D 体素)  →  Truncated Distance Field
        →  A*(3D)  →  B-spline 轨迹  →  Unified Control Coordinator(单一写者)
```

## 目录结构

```
NavigationPhase1/
├── config.py            # 全部参数集中（§67 默认值，可被 scene_test.py 覆盖）
├── settings.json        # 带 LiDAR 的 AirSim 配置（见下方「仿真器配置」）
├── requirements.txt     # 依赖（scipy/pybind11 为可选加速项）
├── run_phase1.py        # 主入口：起飞→爬升→闭环导航→到达→降落（含 --record/--visualize）
├── scene_test.py        # 固定位置/固定任务实测脚本（调参用，含 --record/--visualize/--goal-z）
├── replay.py            # 回放服务器：自动扫描 results/，页面下拉选择任意一次录制
├── nav/                 # 导航栈核心模块
│   ├── airsim_io.py     #   连接 / 状态 / LiDAR 点云 / 深度回退
│   ├── occupancy.py     #   3D 滚动体素图（Free/Unknown/Occupied）
│   ├── raycast.py       #   3D DDA 射线投射（C++ 或 numpy 回退）
│   ├── distance_field.py#   截断欧氏距离场（scipy 或 BFS 回退）
│   ├── astar.py         #   3D 26 连通 A*（3 态成本场）
│   ├── bspline.py       #   均匀三次 B-spline + 平滑优化 + warm-start
│   ├── coordinator.py   #   Unified Control Coordinator（唯一写者）
│   ├── pipeline.py      #   四线程闭环 + 双缓冲地图快照
│   ├── recorder.py      #   录制器（状态/点云/重规划）+ 录制库扫描
│   └── web_server.py    #   可视化 Web 服务（实时 /api/live + 回放 /api/recordings）
├── web/                 # 浏览器三维四视图页面
│   └── viewer.html      #   点云 3D+俯视、路径 3D+俯视、HUD、拖动进度回放
├── cpp/                 # C++ 加速扩展（pybind11）
│   ├── fast_kernels.cpp #   3D DDA 射线投射内核
│   ├── setup.py
│   └── build_extension.bat
└── tests/               # 纯离线单测（无需 AirSim）
```

## 环境要求

- 复用 `CompetitionEnv/.venv`（Python 3.10，已含 `airsim==1.8.1` / `numpy==1.26.3` /
  `tornado==4.5.3` / `opencv`）。
- 模拟器：`CompetitionEnv/CityEnviron/WindowsNoEditor/CityEnviron.exe`（Multirotor 城市环境）。

### 可选加速依赖（缺失时自动退回纯 Python 实现，功能一致）

| 用途 | 依赖 | 回退 |
| --- | --- | --- |
| 距离场 EDT | `scipy`（C 后端优化库） | 多源 BFS chamfer 距离 |
| 射线投射 | C++ `_fast`（pybind11 编译） | numpy 等距采样 |

```bash
cd CompetitionEnv
.venv\Scripts\pip.exe install scipy pybind11
```

## 仿真器配置（一次性）

本项目 `settings.json` 已为载具 `Drone1` 增加 LiDAR（`SensorType=6`、16 通道、
`DataFrame="VehicleInertialFrame"`）。

> ⚠️ **关键坑**：AirSim 打包成 UE 游戏后，真正被读取的是
> **`%USERPROFILE%\Documents\AirSim\settings.json`**（查找顺序里 `WindowsNoEditor\` 之前的
> exe 深目录/启动目录在双击启动时都落空，最后掉到 `Documents\AirSim`）。若只改
> `CityEnviron\WindowsNoEditor\settings.json` 会**不生效**，载具仍是默认 `SimpleFlight`、无 LiDAR。

正确做法（两条都做，稳妥）：

1. 关闭模拟器。
2. 覆盖 exe 目录那份（备用）：
   `Copy-Item NavigationPhase1\settings.json CityEnviron\WindowsNoEditor\settings.json -Force`
3. 覆盖真正生效的那份（关键）：
   `Copy-Item NavigationPhase1\settings.json "$env:USERPROFILE\Documents\AirSim\settings.json" -Force`
4. 重新启动 `CityEnviron.exe`，确认 `listVehicles()` 返回 `Drone1`。

> 若该构建不支持 LiDAR（`getLidarData` 持续为空），系统会自动切到前中相机
> `DepthPerspective` 反投影点云（近似，忽略俯仰/横滚），无需改仿真器。

## 运行

```bash
cd CompetitionEnv
# 1) 自检 + 简单任务（目标：起点北 20m）
NavigationPhase1\run_phase1.py --goal-x 20 --goal-y 0

# 2) 固定场景实测（调参用）
NavigationPhase1\scene_test.py --start-x 0 --start-y 0 --goal-x 40 --goal-y 5 --plot
```

`scene_test.py` 会瞬移到固定起点、执行固定任务、把遥测（位置/速度/净空/制动/目标距离/
碰撞）写到 `NavigationPhase1/results/<时间戳>/`（JSON + CSV），并打印路径长度、最小净空、
均/峰值速度、制动占比等汇总；`--plot` 画 2D 俯视轨迹图。

## 录制与回放（三维四视图）

导航是**真三维**（3D 体素 + 26 连通 A* + 3D 距离场 + 3D B-spline）。目标默认在巡航高度
`-flight_altitude`，给 `--goal-z` 一个不同高度即触发三维路径（爬升/下降）。

### 1) 录制 / 实时观看（推荐：直接终端交互）

**直接运行 `scene_test.py`（不带参数）即可**，终端会逐项询问，包括是否录制、是否四视图、
目标高度等，回车用默认值（录制/四视图默认开）：

```bash
cd CompetitionEnv
NavigationPhase1\scene_test.py
#   起点 X (北, 米) [默认 0.0]:
#   起点 Y (东, 米) [默认 0.0]:
#   目标 X (北, 米) [默认 40.0]:
#   目标 Y (东, 米) [默认 5.0]:
#   巡航高度 (米) [默认 8.0]:
#   目标高度 (米, 回车=同巡航高度→二维) [默认 8.0]:    ← 给不同高度即三维路径
#   是否录制完整过程 [默认 y]:
#   是否启动三维四视图 [默认 y]:                        ← 运行中开 http://127.0.0.1:8765/
#   是否画轨迹图 [默认 n]:
```

命令行方式仍可用（等价覆盖）：`--record` 录制、`--visualize` 四视图、`--goal-z -12` 三维目标。

```bash
# 三维目标（12m 高度）+ 录制
NavigationPhase1\scene_test.py --goal-x 40 --goal-y 5 --goal-z -12 --record
```

录制结果写到 `results/<时间戳>/recording.json`（外加 `meta.json` 元信息侧车）。

### 3) 回放（自动扫描 + 页面下拉选择 + 拖动进度）

```bash
cd NavigationPhase1
..\.venv\Scripts\python.exe replay.py
```

`replay.py` 会自动扫描 `results/` 下所有录制并列出；浏览器打开
`http://127.0.0.1:8765/`，在页面顶部**下拉框选择某次录制**，下方进度条可拖到任意时间点。
页面含四视图：点云 3D + 俯视、路径 3D + 俯视，HUD 显示位置/速度/加速度。

### 调参（覆盖 config.py 默认值）

```bash
scene_test.py --res 0.25 --dmax 4 --dsafe 1.0 --c-unknown 8 --w-obs 1.5 --v-max 4 ...
```

关键参数（详见方案书 §21/§24/§31–§35/§67）：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `res` | 0.2 | 体素分辨率 r_L（米） |
| `d_max` | 4.0 | 截断距离场上限（米） |
| `d_safe` | 0.8 | 障碍安全距离（米，φ(D) 生效阈值） |
| `c_unknown` | 5.0 | Unknown 路径代价（相对 Free=1） |
| `w_obs` | 1.0 | 障碍距离代价权重 |
| `w_smooth` / `w_dyn` | 0.6 / 0.4 | B-spline 平滑 / 动力学权重 |
| `v_max` / `a_max` | 8.0 / 2.0 | 速度 / 加速度上限 |

## 编译 C++ 加速扩展（可选，需 VS BuildTools + 网络装 pybind11）

```bat
cd NavigationPhase1\cpp
build_extension.bat
```

成功后 `nav/_fast.pyd` 生成，`raycast.py` 自动走 C++ 路径（`test_raycast.py` 会打印
`using fast=True`）。编译失败不影响运行（numpy 回退）。

## 坐标约定（NED）

+**X 北 / +Y 东 / +Z 向下**，向上飞 = z 减小（高度传负值，`z=-8` 表示离地 8m）。

## 离线单测（无需启动模拟器）

```bash
cd NavigationPhase1
..\.venv\Scripts\python.exe tests\test_occupancy.py
..\.venv\Scripts\python.exe tests\test_raycast.py
..\.venv\Scripts\python.exe tests\test_distance_field.py
..\.venv\Scripts\python.exe tests\test_astar.py
..\.venv\Scripts\python.exe tests\test_bspline.py
..\.venv\Scripts\python.exe tests\test_coordinator.py
```

## 架构要点（对应方案书章节）

- **三态体素图** Free/Unknown/Occupied（§21），滚动窗口 + 整格平移（§11–§12）。
- **3D DDA 射线投射**（§14），LiDAR 命中点置 Occupied、沿途置 Free、未观测保持 Unknown。
- **截断距离场** D=min(d,D_max)，仅 Occupied 参与，Unknown 不参与（§18–§19）。
- **3D A***（§24），成本场 Free < Unknown < Occupied，Unknown 可通行但高代价（§22–§23）。
- **B-spline 平滑优化**（§29–§35），目标/障碍/平滑/动力学四代价 + warm-start（§30）。
- **Unified Control Coordinator**（§41–§46），PD 跟踪 + 速度/加速度投影 + 紧急制动，
  **全程序只有该对象调用 `move*`（Single Writer）**。
- **四线程** Sensor/Mapping/Planning/Control（§51–§52），Mapping↔Planning 用双缓冲快照（§53）。

## 未实现（留待后续阶段）

Persistent Map / Route Cache（Phase 2）、Unknown 优先级精调（Phase 3）、Unknown 安全速度
`L_known/d_stop`（Phase 4）、长期地图复用（Phase 5）、移动目标追踪（Phase 6）、SLAM/VIO（Phase 7）。
