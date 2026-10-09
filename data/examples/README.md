# 24 条原始飞行基准

`catalog.json` 保存编号、目标、速度、首飞 / 复飞、相对目录与 SHA-256。
`flights/` 保留原始录像、遥测、场景输入及地图快照；`summary/` 为原验收结果，`verification/` 为数据完整性报告。

回放使用根目录 `replay.py` 的“飞行示例”栏目。重飞使用根目录 `fly.cmd`，新结果写到 `data/records/example_runs/`。程序实现集中在 `experiments/`，这里仅保存数据。

验证命令：`python experiments/verify_examples.py`。使用说明见根目录 README 和 `docs/EXPERIMENTS.md`。
