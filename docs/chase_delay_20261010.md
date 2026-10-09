# 实际追击比较

原始批次：`data/records/chase_runs/batches/20261010_034022.json`。各策略恢复相同场景起点；加载共享历史地图，所有观测仍累计到共享地图。

| 场景 | 策略 | 结果 | 捕获仿真秒 | 规划次数 | 目标换向 | 目标路径最大误差 m |
| --- | --- | --- | --- | --- | --- | --- |
| corner | predict | captured | 36.21 | 20 | 1 | 2.89 |
| detour | predict | captured | 31.34 | 17 | 1 | 1.44 |
| escape | predict | captured | 21.62 | 19 | 0 | 0.16 |
| reverse | predict | captured | 24.99 | 27 | 1 | 1.14 |
| stop_go | predict | captured | 31.11 | 27 | 1 | 1.21 |
| vertical | predict | captured | 22.54 | 18 | 0 | 0.17 |

这是有限预设上的实飞结果。捕获要求实际距离连续满足阈值，首次接近不等于捕获；无效目标、初始化故障不算成功。相同起点不意味着 Unreal 动态交通和运行负载完全确定；单轮比较没有统计显著性保证。
