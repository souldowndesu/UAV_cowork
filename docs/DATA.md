# 数据、路径和归档

运行代码不混入保存数据。统一路径由 `navigation/paths.py` 定义，默认数据根是 `NavigationPhase1/data/`；显式绝对路径不改变，相对路径以项目根为基准。

## 三类数据

| 目录 | 内容 | 运行时用途 |
| --- | --- | --- |
| `data/examples/` | catalog、24 条验证飞行、各自地图与场景输入、summary、verification | 基准回放与恢复原先验重飞 |
| `data/records/` | 每次新飞行的独立目录；重飞集中在 example_runs | 查看新结果、诊断、对比与流式保存 |
| `data/maps/` | 长期共享的持久地图及元数据 | 为新导航提供历史先验 |

示例中的地图与记录内 `run_map.npz` 是该案例的一部分，不抽走到 maps，以保持快照关系与原始文件 SHA-256。maps 只管理独立复用的地图。24 个示例的编号、相对目录和飞行文件内容没有改变。

## 单次记录

- `manifest.json`、`chunk_*.json.gz`：v2 流式录像，含 states、points、plans、maps 与事件；完整复制时带整个目录。
- `recording.json`：兼容旧格式；流式记录主要读取 manifest。
- `telemetry.json` / `.csv`：状态、速度、指令、净空、碰撞与目标数据。
- `run_meta.json`：配置、起点、目标、地图先验哈希、源码哈希、启动状态和任务结果。
- `initial_map.npz` / `run_map.npz`：批次先验与独立运行地图。
- `evaluation/`：评估 JSON、Markdown 及可选图表。

历史记录与示例的 provenance 字段可能包含原目录的绝对路径；为了保存原始证据，不对它们做替换。当前工具使用 catalog 相对路径定位录制和快照，在新输出中写入当前目录。移动 records 根目录不改变其中的相对布局，录制库 ID 仍依据相对路径生成。

## 归档

`archive/20261009_cleanup/moves.json` 记录本次每项源路径、目标路径和归档原因。旧测试在 `tests/`，旧说明和诊断在 `docs/history/`，旧场景参数、重复构建文件、临时浏览器资料、旧 ZIP 等保留在各自子目录。原有 `archive/records_20261003/` 保留。整个归档不参与默认回放扫描，也不进入分发 ZIP，没有永久删除。

当前根目录不再使用 nav、results、maps、examples、cpp、web、tools 等旧路径。对应关系：

| 原路径 | 当前路径 |
| --- | --- |
| `nav/*`（算法） | `navigation/*` |
| `config.py` / `cpp/` | `navigation/config.py` / `navigation/cpp/` |
| `nav/recorder.py`、evaluation、experiment、web_server、simulator_replay | `experiments/` 下同名模块 |
| scene_test、replay、run_phase1 中的实现 | `experiments/scene_test.py`、replay.py、mission.py |
| `examples/fly.py` / `examples/verify.py` | `experiments/fly_examples.py` / verify_examples.py |
| `examples/`、`results/`、`maps/` | `data/examples/`、`data/records/`、`data/maps/` |
| settings、requirements、wheelhouse、licenses | `environment/` |
| `web/` | `experiments/web/` |

如果外部代码此前 `from nav...`，需改成 `from navigation...`，录制与评估模块改成 `from experiments...`，配置改成 `from navigation.config import Config`。根目录的用户启动命令继续保留。
