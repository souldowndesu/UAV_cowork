# 实验、回放与 examples 重飞

实现集中在 `experiments/`；根目录 `scene_test.py`、`replay.py`、`fly.cmd`、`replay_scene.cmd` 是稳定入口。参数命令在 README 中可直接使用。

## 固定起点实验

`scene_test` 默认重置 AirSim，恢复 API 控制与解锁，放到指定起点，等待状态稳定，起飞并爬升到巡航高度，再执行导航；结束后落地并保存结果。使用 `--start-x/y/z`、`--start-yaw`、`--goal-x/y/z` 或 `--goal-altitude` 指定任务。NED 的 Z 向下。

起飞/爬升使用独立启动 RPC 连接并读取 Future 错误，常规控制连接仍保持短超时；爬升结束核对实际高度，误差超过 0.8 m 则拒绝进入导航。此模拟器的异步返回布尔值不足以判断是否抵达，不能只调用 join 后假定成功。

`--scenario` JSON 示例位于 `experiments/scenarios/`。字段包括 `start`、`goal`、`start_yaw`、`config`、`map_policy`、`max_duration`。`--runs N` 每轮从同一个起点和约定先验开始，随机种子来自 Config。Config 覆盖优先级是 profile、场景 config、命令行。

默认录制器每秒左右追加压缩分块；`--autosave-interval` 控制地图回放快照和强制刷新周期。`--no-record` 关闭录像。`--visualize --viz-port 8767` 开启飞行中的网页显示，多轮时关闭实时网页以避免交互阻塞。

离线评估不会连接模拟器：

```powershell
python scene_test.py --evaluate data/examples/flights/safe5/diagonal/flight_2 --evaluation-dir data/records/evaluations
```

指标、遥测和报告只是诊断；稀疏回放点云不能独立证明没有碰撞。

## 回放界面

`replay.py` 自动扫描 `data/records` 和 `data/examples/flights`。网页有“运行记录”和“飞行示例 · 24 次验证”两个栏目。同目录 manifest 和 chunk 文件必须一起保留，运行中已经落盘的分块也可读取。录制有仿真时间戳时使用仿真时间轴，否则使用墙钟时间；显示测量速度、速度指令、安全限速和巡航参考。

点云模式可看局部图、持久图和路线。历史总览会包含当前时刻以后的信息，勾选时页面明确提示。追击录像还包含黄色目标已飞轨迹，详见 CHASE.md。模拟器模式的蓝线只到当前已飞位置，橙色 A* 与绿色平滑曲线只使用当前时刻已生成的最近一次规划。

## 建筑背景与相机

`replay_scene.cmd` 调用 `experiments/start_simulator.cmd`，通过 `-settings` 载入 `environment/settings_replay.json`。默认找相邻 CityEnviron；可以把 exe 路径作为第一个参数。已知打包环境直接启动内部 Unreal 引擎，避免启动壳等待 Windows 安全弹窗，不修改系统安全设置。RPC 已可用时复用，真实引擎正在加载时避免重复启动。

`replay.py` 默认显示点云，勾选“模拟器场景”才连接 AirSim。场景回放暂停物理，插值录制位置和姿态，更新真实无人机模型与轨迹，外部 ReplayCamera 的图像传到网页。旧录像缺少 pitch / roll 时尝试由配套 telemetry 补齐。世界坐标与载具出生坐标偏移一起应用到模型、标记与相机。

左键旋转、Shift 或右键平移、滚轮缩放；双击或重置按钮恢复当前预设。顶部可选跟随、俯视、整条路线、双机俯视；方向键可旋转，Shift + 方向键平移，`+` / `-` 缩放，`R` 重置。手动调整在播放、暂停和拖动时间轴时保留，换录像或预设时重置。

建筑来自当前模拟器，录制没有保存完整场景，也不恢复当时的动态交通、天气和光照。关闭场景模式或正常结束服务会恢复原始无人机位姿、暂停状态并清理标记；页面断开约 8 秒后释放控制。强制终止进程可能无法清理，此时重启模拟器。

## 重飞

`fly.cmd` 调用 `experiments/fly_examples.py`，菜单选 1–24 或使用 `--case`。`--repeat N` 每轮恢复同一个起始条件；首飞 cold，复飞从本批次开始时的统一共享图创建内存快照；可用 --load-map / --no-load-map 覆盖。旧示例快照只用于原始数据回放与校验。可以指定 `--ip`、`--port`、`--visualize` 和 `--output-dir`，禁止写入原始 examples 目录。

重飞实际调用统一的根目录 `scene_test.py`，其生成的场景输入保存在新记录目录的 `inputs/` 下，不使用系统临时目录。旧录像是固定回放基准，重飞是重新感知与规划，两者结果可以不同。

## 统一地图

新飞行只合并保存到 data/maps/navigation_shared.npz。--load-map 读取共享历史，--no-load-map 空图开始；是否加载与是否保存独立。cold 也不清除已有共享图；snapshot 先验在内存中固定，输出不再生成每轮地图文件。原始 examples 的地图仍只读保留。
