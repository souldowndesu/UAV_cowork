# 移动目标追击

## 启动与运行

环境与静态导航相同：Windows x64、CPython 3.10、AirSim 1.8.1，依赖见 environment/requirements.txt。先运行 setup_env.cmd，再用 replay_scene.cmd 开启双机模拟器。environment/settings_replay.json 与 settings_chase.json 都包含 Drone1、BlackDrone 和 ReplayCamera；在自己的场景启动器中使用 settings_chase.json 也可。

```powershell
.venv\Scripts\python.exe chase_test.py                       # 菜单选择
.venv\Scripts\python.exe chase_test.py --list
.venv\Scripts\python.exe chase_test.py --preset corner --strategy predict --no-load-map
.venv\Scripts\python.exe chase_test.py --preset all --strategy all --no-load-map --runs 2
.venv\Scripts\python.exe chase_test.py --preset stop_go --delay 0.8 --lead-max 2 --visualize
.venv\Scripts\python.exe replay.py --port 8770
```

当前 CompetitionEnv 工作区的环境在上一级，命令中的 .venv 可改为 ..\.venv。模拟器保持一个实例；飞行与场景回放应分开运行，因为场景回放会暂停物理。

## 场景

预设放在 experiments/scenarios/chase/，JSON 自包含目标路线、起点、速度与高度，不依赖原始 examples 录像文件。固定每个场景的初始条件；不同场景可有不同起点。

| 名称 | 过程 |
| --- | --- |
| escape | 目标沿斜向走廊匀速逃逸 |
| corner | 目标先沿街道移动，再经过约 90 度转弯与高度变化 |
| stop_go | 目标按周期移动、停车、加速和减速 |
| reverse | 短路线折返，形成迎面接近和重新跟随 |
| vertical | 水平逃逸中叠加连续高度变化 |
| detour | 建筑附近转弯、改变高度并沿安全走廊折返 |

轨迹由已有验证飞行的走廊采样生成；目标使用独立物理速度控制器，按实测位置跟随路线、限制加速度并在端点减速换向。只有开始时初始化双机物理状态，飞行过程中不传送。运行前检查双机实际起点误差与碰撞；目标碰撞、初始化失效或 RPC 错误都标记为无效基准，不算算法捕获成功。

## 接近逻辑

控制器拿到的是延迟送达的目标实测位置、速度和仿真时间戳。TargetTracker 拒绝倒序与非法观测，平滑速度并检测转弯；超过有效观测窗口时停止沿旧目标追逐。

- pure：直接使用已收到的位置，不补偿延迟、不增加预测提前量。
- latency：用目标速度补偿观测年龄，估计当前目标位置。
- predict：在延迟补偿上求匀速拦截时间，将提前量限制在配置上限；转弯和近距离时缩短提前量。

三种策略都保留相同的避障和近距离速度前馈，便于单独比较位置预测的影响。目标接近点保留水平跟随间隔和独立垂直间隔，停车时沿用原方向；迎面时优先选择追击机所在一侧的接近点，带切换迟滞，减少擦肩后重复追赶。可用 --stand-off-mode trailing 对照原后方跟随。

远距离复用历史引导、局部 A*、经过碰撞检查的空间曲线与安全速度控制。目标移动足够远或重规划周期到达时更新路线。进入近距离且到跟随点的整条直线满足净空后，使用“目标速度 + 位置误差反馈”；按照双机相对接近速度提前减速，并在整条接近弦通过净空检查后执行。每个控制周期仍经过预测制动、地图过期保护、加速度和 jerk 限制。近距离安全跟随不反复计算几厘米长的 A*。

## 参数与判定

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| --preset | 菜单 | 六种预设，all 顺序运行全部 |
| --strategy | predict | pure / latency / predict；all 对相同场景逐一比较 |
| --profile | safe5 | 追击机基础配置，亦可选 safe8 |
| --delay | 0.35 s | 目标观测传输延迟，按仿真时间模拟 |
| --lead-max | 3 s | 最大匀速预测提前量 |
| --replan-interval | 1.2 s | 墙钟重规划周期；目标明显移动也会触发更新 |
| --stand-off | 3.5 m | 水平跟随间隔 |
| --stand-off-mode | intercept | 按相对位置选接近侧；trailing 始终放在运动方向后方 |
| --vertical-offset | 2 m | 追击机相对目标的上方间隔 |
| --capture-radius | 5 m | 双机三维距离阈值 |
| --capture-hold | 2 s | 距离连续满足阈值的仿真时间 |
| --target-speed | 预设值 | 覆盖目标巡航速度，停走速度同比缩放 |
| --max-duration | 预设值 | 单轮墙钟超时，不是仿真秒数 |
| --load-map / --no-load-map | no-load-map | 是否读取共享历史；两种模式都合并保存观测 |
| --runs | 1 | 每种场景和策略重复次数 |

默认非接触捕获要求：实际三维距离小于 5 米并连续保持 2 秒，目标累计移动超过 15 米，双方均无碰撞。first_close 只是首次接近时间，不能代替最终捕获。仿真运行速度可能慢于墙钟，报告明确分开两者。

捕获判定将目标实际状态序列插值到追击机的采样时间，避免异步 RPC 真值不在同一时刻；仅允许序列端不超过 0.15 仿真秒的速度外推，过大缺口不计入连续捕获。此真值对齐只用于评估，不进入延迟观测或导航预测。

Config 字段也可通过连字符参数覆盖，例如 --v-mission 4 --v-max 5；全部显式参数见 chase_test.py --help。更高速度、延迟、过小捕获半径或超出验证走廊的自定义路线需要另行测试。已记录的成功率只说明对应预设与参数，没有保证任意移动目标都能捕获。

## 记录与回放

新记录在 data/records/chase_runs/<场景>/<策略>/<时间戳>/，包含流式 manifest / chunks、实际双机状态、当时生成的 A* 与平滑路线、地图、chase_telemetry.json、chase_summary.json、run_meta.json。参数、源码哈希、初始误差、碰撞、目标运动距离、换向次数、规划次数及捕获时间均保存。批次比较汇总在 chase_runs/batches/，每完成一轮就更新。

同一 replay.py 自动读取新记录，黄色轨迹是实际目标已飞部分，蓝色是追击机，橙色 / 绿色是当时已有规划。选择“模拟器场景”可看到建筑与双机模型；“双机俯视”以双机中点为中心，按两机间距调整高度。原来的跟随、俯视、整条路线及鼠标旋转 / 平移 / 缩放仍保留。旧静态录像与 24 条 examples 继续可读。

保存使用独立线程，感知和控制不等待磁盘；任务结束等待保存线程并写入最后一帧观测。所有运行的观测只合并写入 data/maps/navigation_shared.npz，不生成每轮 run_map。关闭历史加载仍保留共享图的旧区域；录像中的地图快照仅用于显示。单独移动目标的近身回波会过滤，普通动态交通仍可能留下历史占用，当前持久地图没有完整的动态物体分类与时间衰减。

离线比较：`python -m experiments.chase_analysis data/records/chase_runs/batches/<批次>.json --output docs/chase_comparison.json`。同时生成 JSON 与 Markdown，包含目标执行误差、实际捕获时间、规划次数与碰撞。飞行高度 / 车辆等由预设或入口专用参数决定，Config 通用覆盖用于速度与导航设置。

实际匹配比较、延迟与高速复核，以及回放证据见 [CHASE_RESULTS.md](CHASE_RESULTS.md)。
