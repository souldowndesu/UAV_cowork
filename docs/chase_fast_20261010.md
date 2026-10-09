# 实际追击比较

原始批次：`data/records/chase_runs/batches/20261010_034921.json`。各策略恢复相同场景起点；加载共享历史地图，所有观测仍累计到共享地图。

| 场景 | 策略 | 结果 | 捕获仿真秒 | 规划次数 | 目标换向 | 目标路径最大误差 m |
| --- | --- | --- | --- | --- | --- | --- |
| corner | predict | captured | 34.60 | 18 | 1 | 8.26 |
| detour | predict | timed_out | — | 35 | 5 | 2.46 |
| escape | predict | captured | 14.99 | 13 | 0 | 0.31 |
| reverse | predict | captured | 27.81 | 23 | 1 | 1.46 |
| stop_go | predict | captured | 29.27 | 18 | 1 | 1.82 |
| vertical | predict | captured | 15.40 | 13 | 0 | 0.32 |

这是有限预设上的实飞结果。捕获要求实际距离连续满足阈值，首次接近不等于捕获；无效目标、初始化故障不算成功。相同起点不意味着 Unreal 动态交通和运行负载完全确定；单轮比较没有统计显著性保证。
