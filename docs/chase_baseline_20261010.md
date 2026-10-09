# 实际追击比较

原始批次：`data/records/chase_runs/batches/20261010_022052.json`。各策略恢复相同场景起点；关闭历史加载，所有观测仍累计到共享地图。

| 场景 | 策略 | 结果 | 捕获仿真秒 | 规划次数 | 目标换向 | 目标路径最大误差 m |
| --- | --- | --- | --- | --- | --- | --- |
| corner | pure | captured | 53.96 | 46 | 2 | 3.03 |
| corner | latency | captured | 38.65 | 38 | 1 | 2.92 |
| corner | predict | captured | 34.64 | 34 | 1 | 2.93 |
| detour | pure | captured | 36.31 | 28 | 1 | 1.47 |
| detour | latency | captured | 31.91 | 23 | 1 | 1.47 |
| detour | predict | captured | 55.48 | 34 | 2 | 1.48 |
| escape | pure | captured | 34.82 | 20 | 1 | 1.06 |
| escape | latency | captured | 20.64 | 20 | 0 | 0.16 |
| escape | predict | captured | 20.61 | 21 | 0 | 0.16 |
| reverse | pure | timed_out | — | 67 | 2 | 1.49 |
| reverse | latency | captured | 35.11 | 33 | 1 | 1.13 |
| reverse | predict | captured | 34.68 | 31 | 1 | 1.16 |
| stop_go | pure | captured | 32.31 | 36 | 1 | 1.19 |
| stop_go | latency | captured | 24.29 | 25 | 0 | 0.30 |
| stop_go | predict | captured | 24.20 | 24 | 0 | 0.29 |
| vertical | pure | captured | 35.35 | 25 | 1 | 1.00 |
| vertical | latency | captured | 20.89 | 20 | 0 | 0.17 |
| vertical | predict | captured | 20.71 | 20 | 0 | 0.17 |

这是有限预设上的实飞结果。捕获要求实际距离连续满足阈值，首次接近不等于捕获；无效目标、初始化故障不算成功。相同起点不意味着 Unreal 动态交通和运行负载完全确定；单轮比较没有统计显著性保证。
