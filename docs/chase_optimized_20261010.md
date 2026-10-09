# 实际追击比较

原始批次：`data/records/chase_runs/batches/20261010_030438.json`。各策略恢复相同场景起点；关闭历史加载，所有观测仍累计到共享地图。

| 场景 | 策略 | 结果 | 捕获仿真秒 | 规划次数 | 目标换向 | 目标路径最大误差 m |
| --- | --- | --- | --- | --- | --- | --- |
| corner | pure | captured | 29.07 | 25 | 1 | 2.97 |
| corner | latency | captured | 28.36 | 33 | 1 | 2.90 |
| corner | predict | captured | 28.46 | 31 | 1 | 2.88 |
| detour | pure | captured | 28.62 | 22 | 1 | 1.49 |
| detour | latency | captured | 31.59 | 22 | 1 | 1.48 |
| detour | predict | captured | 32.26 | 25 | 1 | 1.50 |
| escape | pure | captured | 32.56 | 18 | 1 | 0.75 |
| escape | latency | captured | 20.72 | 19 | 0 | 0.16 |
| escape | predict | captured | 20.52 | 19 | 0 | 0.16 |
| reverse | pure | timed_out | — | 63 | 2 | 1.47 |
| reverse | latency | captured | 28.27 | 28 | 1 | 1.19 |
| reverse | predict | captured | 27.53 | 28 | 1 | 1.18 |
| stop_go | pure | captured | 32.31 | 32 | 1 | 1.20 |
| stop_go | latency | captured | 24.27 | 23 | 0 | 0.29 |
| stop_go | predict | captured | 23.88 | 23 | 0 | 0.29 |
| vertical | pure | captured | 34.58 | 35 | 1 | 1.05 |
| vertical | latency | captured | 21.02 | 19 | 0 | 0.17 |
| vertical | predict | captured | 21.08 | 19 | 0 | 0.18 |

这是有限预设上的实飞结果。捕获要求实际距离连续满足阈值，首次接近不等于捕获；无效目标、初始化故障不算成功。相同起点不意味着 Unreal 动态交通和运行负载完全确定；单轮比较没有统计显著性保证。
