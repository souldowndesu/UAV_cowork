# 实际追击比较

原始批次：`data/records/chase_runs/batches/20261010_041422.json`。各策略恢复相同场景起点；关闭历史加载，所有观测仍累计到共享地图。

| 场景 | 策略 | 结果 | 捕获仿真秒 | 规划次数 | 目标换向 | 目标路径最大误差 m |
| --- | --- | --- | --- | --- | --- | --- |
| corner | predict | captured | 34.78 | 28 | 1 | 2.92 |
| detour | predict | captured | 32.06 | 24 | 1 | 1.49 |
| escape | predict | captured | 20.51 | 19 | 0 | 0.16 |
| reverse | predict | captured | 25.08 | 27 | 1 | 1.15 |
| stop_go | predict | captured | 23.98 | 23 | 0 | 0.29 |
| vertical | predict | captured | 20.70 | 18 | 0 | 0.18 |

这是有限预设上的实飞结果。捕获要求实际距离连续满足阈值，首次接近不等于捕获；无效目标、初始化故障不算成功。相同起点不意味着 Unreal 动态交通和运行负载完全确定；单轮比较没有统计显著性保证。
