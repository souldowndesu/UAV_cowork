# NavigationPhase1

无人机导航、持久建图、场景实验与飞行回放。当前目录于 2026-10-09 起整理，2026-10-10 完成验收；历史开发资料保存在 `archive/`，不参与运行。

## 目录

```text
NavigationPhase1/
  scene_test.py          任意任务实验入口
  chase_test.py          移动目标双机追击与策略比较
  replay.py              点云 / 模拟器 / examples 回放入口
  fly.cmd                选择 1–24 号案例重新飞行
  replay_scene.cmd       启动回放用模拟器
  setup_env.cmd          安装运行环境
  requirements.txt      转到 environment/requirements.txt
  navigation/           导航算法、参数、持久地图实现
    config.py           全部 Config 参数
    profiles.py         safe5 / safe8 / legacy 配置
    paths.py            统一的项目与数据路径
    cpp/                C++ 加速源码和构建入口
  experiments/          实验、重飞、录制、评估、回放实现
    scene_test.py / chase_test.py / mission.py / fly_examples.py
    chase_target.py / chase_analysis.py
    replay.py / simulator_replay.py / web_server.py / web/
    recorder.py / evaluation.py / experiment.py
    start_simulator.cmd / start_simulator.ps1 / verify_examples.py / scenarios/
  data/
    examples/           24 条原始验证飞行、快照、参数和校验值
    records/            新实验录像、遥测、评估；example_runs/ 为重飞记录
    maps/               唯一累计观测地图 navigation_shared.npz
  environment/          依赖、离线 wheels、模拟器 settings、许可、打包工具
  docs/                 当前逻辑、参数、操作、数据格式文档
  archive/              测试、旧文档、历史实验、临时产物与旧 ZIP
```

根目录的 Python / CMD 文件只负责转到对应实现。直接执行 `experiments/` 中的工具或用 `python -m experiments.replay` 也可。所有工具指定的相对数据路径均以本项目根目录为基准；绝对路径不变，不随启动时的工作目录漂移。

## 环境

验证环境：Windows x64、CPython 3.10 x64、AirSim 1.8.1、NumPy 1.26.3、SciPy 1.15.3、OpenCV 4.11.0.86。依赖版本见 `environment/requirements.txt`。双击 `setup_env.cmd`，使用离线 `environment/wheelhouse/` 创建项目内 `.venv` 并核对 C++ 加速模块。已有环境也可运行 `python -m pip install -r requirements.txt`。

下面命令从项目根目录执行。分发包使用 `.venv\Scripts\python.exe`；当前 CompetitionEnv 工作区沿用上一级环境时换成 `..\.venv\Scripts\python.exe`。模拟器不包含在代码包内。普通实飞使用 `environment/settings.json` 配置；回放模拟器入口自动使用 `environment/settings_replay.json`，无需修改 Documents 中的 settings。

## 任意任务：scene_test

先开启模拟器，再运行：

```powershell
# 交互式设置起点、目标、速度、录像与实时显示
.venv\Scripts\python.exe scene_test.py

# 指定目标，空地图开始；默认重置并实时保存录像
.venv\Scripts\python.exe scene_test.py --goal-x 100 --goal-y 70 --goal-altitude 20 --profile safe5 --map-policy cold --max-duration 240

# 明确起点和自定义速度，重复两轮
.venv\Scripts\python.exe scene_test.py --start-x 0 --start-y 0 --start-z -0.5 --start-yaw 0 --goal-x 60 --goal-y 40 --goal-altitude 15 --speed 4 --v-max 5 --runs 2 --map-policy cold

# 使用统一场景文件
.venv\Scripts\python.exe scene_test.py --scenario experiments/scenarios/basic.json

# 实飞同时查看点云、规划和速度：http://127.0.0.1:8767/
.venv\Scripts\python.exe scene_test.py --goal-x 40 --goal-y 5 --visualize --viz-port 8767 --map-policy cold
```

默认配置 `safe5`；参数优先级：profile < 场景 JSON 的 config < 命令行。Config 字段都可以覆盖，例如 `--d-safe 3 --max-range 35 --flight-altitude 10`。坐标为 NED：X 北、Y 东、Z 向下；`--goal-altitude 20` 等同目标 `z=-20`。使用 `--no-record` 关闭录像，`--no-reset` 保留模拟器状态。默认记录在 `data/records/<时间戳>/`，多轮分别保存在 `run_00` 等目录。

三个实飞入口始终收集并合并保存地图；用 `--no-load-map` 或 `--map-policy cold` 控制历史加载。启动爬升使用独立的较长 RPC 等待，读取 RPC 错误并核对实际高度后才启动导航，避免慢速仿真尚未爬升完成就被导航指令取消。

## 移动目标追击

用 `replay_scene.cmd` 启动新版双机场景，再执行：

```powershell
.venv\Scripts\python.exe chase_test.py --list
.venv\Scripts\python.exe chase_test.py --preset reverse --strategy predict --no-load-map --visualize
.venv\Scripts\python.exe chase_test.py --preset all --strategy all --no-load-map
```

不传 `--preset` 会显示案例选择菜单。六种预设覆盖逃逸、直角转弯、停走、折返、高度变化和绕楼；双方均用物理速度控制飞行。默认以双机距离小于 5 米并连续保持 2 秒作为非接触捕获。记录保存在 `data/records/chase_runs/`，同一个 replay 可回放双机和黄色目标轨迹，并选择“双机俯视”。参数、比较口径和验证范围见 [CHASE.md](docs/CHASE.md)。

`scene_test`、追击和 examples 重飞的新观测都合并到 `data/maps/navigation_shared.npz`。`--no-load-map` / `--map-policy cold` 只关闭本轮历史加载，仍保存新观测并保留旧区域。旧 examples 中的地图保持原样，供原始结果回放与校验。

## 回放

```powershell
.venv\Scripts\python.exe replay.py
```

浏览器打开 **http://127.0.0.1:8765/**，选择“运行记录”或“飞行示例 · 24 次验证”，再选择飞行。无需模拟器即可看点云、已飞轨迹、A*、平滑轨迹与持久地图。播放、倍速和进度拖动均在同一网页。服务持续运行到 Ctrl+C，旧参数 `--serve` 保留兼容。

建筑背景回放：双击 `replay_scene.cmd` 启动回放用模拟器，运行同一个 `replay.py`，网页勾选“模拟器场景”。它显示建筑和无人机实际模型，按原录像恢复位姿与当时预测。左键旋转，Shift / 右键平移，滚轮缩放，双击或“重置视角”恢复预设；顶部可切换跟随、俯视、整条路线和双机俯视。手动视角在播放与暂停时保留。

```powershell
# 模拟器安装在其他位置
.\replay_scene.cmd "C:\AirSim\CityEnviron.exe"

# 指定录像目录和网页端口；相对目录锚定项目根目录
.venv\Scripts\python.exe replay.py --results-dir data/records/example_runs --port 8770
```

`--simulator` 可启动即显示场景，`--no-simulator` 仅提供点云。场景必须与录像一致；场景回放会暂停物理，播放已保存的位置和姿态。要重新运行导航使用 `fly.cmd` 或 `scene_test`。启动器会拒绝重复启动正在运行或加载中的模拟器。

## examples 重飞

双击 `fly.cmd`，输入 1–24 选择案例，输入 `q` 退出。新结果保存到 `data/records/example_runs/`，24 条原始示例保持不变。

```powershell
.\fly.cmd --list
.\fly.cmd --case 10 --repeat 3
.venv\Scripts\python.exe experiments/fly_examples.py --case 10 --dry-run
.venv\Scripts\python.exe experiments/verify_examples.py
```

首飞每轮从空图开始；复飞使用本批次开始时的共享地图快照，每次重复恢复相同先验。可用 --load-map / --no-load-map 覆盖；新观测都累计到共享图。旧示例快照只用于原始回放和校验。重飞会重新感知、规划和执行，精确回看原始记录使用 replay。

## 文档与维护

- [现有导航与持久化逻辑](docs/NAVIGATION.md)
- [完整参数表与覆盖方式](docs/PARAMETERS.md)
- [实验、回放和重飞操作](docs/EXPERIMENTS.md)
- [保存数据与路径约定](docs/DATA.md)
- [移动目标操作与参数](docs/CHASE.md)
- [追击实飞比较与验证边界](docs/CHASE_RESULTS.md)

旧 tests 与文档位于 `archive/20261009_cleanup/`，移动清单为其中的 `moves.json`；没有永久删除。日常验证使用 scene_test / chase_test；与当前追击改动相关的校验保存在 archive/20261010_chase/test_chase.py。旧全量测试作为历史存档保留；网页相关校验在 archive/20261009_cleanup/tests/replay_ui.test.cjs。归档诊断脚本和旧报告保留历史路径，不作为当前使用说明。

分发包用 `python environment/package_release.py` 生成到 `environment/releases/`，排除整个 archive、测试临时输出、虚拟环境和缓存。第三方声明见 `environment/THIRD_PARTY_NOTICES.md`。目录迁移验收数据保存在 `docs/validation_20261009.json`；追击、统一地图和原有入口实飞验收见 `docs/validation_20261010_chase.json`。

Git 保存源码、当前文档和小型索引；原始录像、共享地图、离线 wheels、编译后的 `.pyd` 和 ZIP 不进入 Git。需要直接使用 24 条 examples 和离线安装时，使用完整 ZIP；仅克隆源码则自行安装依赖并使用 navigation/cpp 中的构建入口编译加速模块，也可使用 Python 回退实现。
