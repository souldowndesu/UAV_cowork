# 导航参数

参数实现集中在 `navigation/config.py`，档位覆盖在 `navigation/profiles.py`。scene_test 默认 safe5；直接构造 Config 的基础值保留 legacy 分支，不能把基础值当作 scene_test 默认。下表按当前实现自动生成，tuple 已转为 JSON 数组。

## 覆盖

```powershell
python scene_test.py --goal-x 40 --goal-y 5 --profile safe5 --speed 4 --v-max 5 --d-safe 3 --map-size '(60,60,32)' --map-policy cold
```

Config 字段下划线改为连字符，例如 `persistent_map_path` 对应 `--persistent-map-path`；布尔字段传 `true` / `false`。场景文件使用原始字段名。优先级为 profile < 场景 config < 命令行；`--speed` 等同 `v_mission`，不能超过 `v_max`。配置校验会拒绝非有限数字与不一致的安全 / 动力学参数。

目标 / 起点 / 次数 / 超时 / map-policy / 录像等由 scene_test 的显式参数提供，完整列表 `python scene_test.py --help`。`--goal-altitude` 使用正数高度，内部转负 z；`--goal-z` 是原始 NED 值，二者互斥。

确认建筑内部目标时可用 `--goal-in-building-confirmed true --goal-occupied-confirmed true`；安全接近结果保留原目标和当前接近点。历史地图必须属于同一场景和原点。relative paths 统一以项目根目录为基准。

## 当前值

| 字段 | 基础 Config | safe5 | safe8 | 实现备注 |
| --- | --- | --- | --- | --- |
| `navigation_mode` | `"legacy"` | `"safe_route"` | `"safe_route"` | safe_route enables validated spatial spline following |
| `jerk_max` | `8.0` | `8.0` | `8.0` |  |
| `map_stale_timeout` | `1.5` | `1.5` | `1.5` |  |
| `route_replan_distance` | `14.0` | `14.0` | `14.0` |  |
| `safe_shortcuts` | `false` | `true` | `true` |  |
| `fast_history` | `false` | `true` | `true` |  |
| `route_buffer` | `0.0` | `0.8` | `0.8` |  |
| `curve_accel_limit` | `0.0` | `2.0` | `2.0` |  |
| `flight_altitude_min` | `0.0` | `5.0` | `5.0` |  |
| `flight_altitude_max` | `0.0` | `160.0` | `160.0` |  |
| `allow_safe_approach` | `false` | `true` | `true` |  |
| `approach_max_distance` | `0.0` | `35.0` | `35.0` | 0 disables task proximity bound; positive limits accepted approach candidates |
| `goal_occupied_confirmed` | `false` | `false` | `false` |  |
| `goal_in_building_confirmed` | `false` | `false` | `false` |  |
| `goal_brake_reaction` | `0.7` | `0.7` | `0.7` |  |
| `use_world_guide` | `false` | `true` | `true` |  |
| `guide_res` | `1.6` | `1.6` | `1.6` |  |
| `guide_margin` | `35.0` | `35.0` | `35.0` |  |
| `guide_interval` | `2.0` | `2.0` | `2.0` |  |
| `guide_lookahead` | `28.0` | `28.0` | `28.0` |  |
| `guide_clearance` | `0.8` | `1.6` | `1.6` |  |
| `guide_altitude_min` | `5.0` | `5.0` | `5.0` |  |
| `guide_altitude_max` | `50.0` | `160.0` | `160.0` |  |
| `guide_altitude_cost` | `0.0` | `0.0` | `0.0` |  |
| `predictive_braking` | `false` | `true` | `true` |  |
| `brake_reaction` | `0.65` | `0.65` | `0.65` |  |
| `brake_accel` | `2.0` | `2.0` | `2.0` |  |
| `velocity_safety_guard` | `false` | `true` | `true` |  |
| `brake_margin` | `0.6` | `0.6` | `0.6` |  |
| `persistent_full_sensor` | `false` | `true` | `true` |  |
| `progress_exploration` | `false` | `true` | `true` |  |
| `explore_timeout` | `18.0` | `18.0` | `18.0` |  |
| `explore_height_step` | `18.0` | `18.0` | `18.0` |  |
| `map_size` | `[40.0, 40.0, 10.0]` | `[60.0, 60.0, 32.0]` | `[60.0, 60.0, 32.0]` | (x, y, z) 米，NED 轴 |
| `map_back_x` | `10.0` | `30.0` | `30.0` | 无人机在 X 方向距地图后端的距离（米）；前向余量 = map_size[0]-map_back_x |
| `res` | `0.2` | `0.4` | `0.4` | 体素边长 r_L，米 |
| `plan_res` | `0.4` | `0.8` | `0.8` | 粗路径 A* 的分辨率（§7/§24 粗网格，提速） |
| `d_max` | `6.0` | `6.0` | `6.0` | 截断距离场上限（§18/§67），必须 ≥ d_safe 使 φ(D) 能归零 |
| `d_safe` | `5.0` | `3.0` | `3.0` | 障碍安全距离（§20/§67），软代价 ψ(D) 生效阈值（5m 安全距离） |
| `drone_radius` | `0.3` | `0.4` | `0.4` | 无人机物理半径，米（对建筑做固定膨胀，保证机身余量） |
| `d_min` | `0.8` | `1.0` | `1.0` | 硬净空阈值，米（相对已膨胀障碍；A*/B-spline 硬约束） |
| `max_range` | `30.0` | `35.0` | `35.0` | LiDAR 最大感知距离，米（§54），与地图前向余量对齐 |
| `self_echo_dist` | `0.3` | `0.3` | `0.3` | 射线投射自回波过滤最小距离（滤掉载具自身，米） |
| `lidar_dtheta_h_deg` | `1.89` | `1.89` | `1.89` | 水平角分辨率（度），=360·通道·RPS/PPS |
| `lidar_dtheta_v_deg` | `1.89` | `1.89` | `1.89` | 垂直角分辨率（度），=180/(通道数-1)，96 通道均衡 |
| `c_free` | `1.0` | `1.0` | `1.0` |  |
| `c_unknown` | `5.0` | `1.2` | `1.2` | 仅剩"障碍物背后的真·未观测区"才标 Unknown，保持昂贵 |
| `w_obs` | `1.0` | `1.0` | `1.0` | 距离场障碍代价权重（φ 已含指数幅值，w_obs=1 使 5m→0/2m→19） |
| `w_hard` | `100.0` | `100.0` | `100.0` | B-spline 硬净空 barrier 权重（净空 < d_min 时二次惩罚，优化内硬约束） |
| `astar_timeout_nodes` | `1500000` | `1500000` | `1500000` | 展开节点上限（粗图 pad 后 ~176×176×25=774k 体素，启发式低估时搜索前沿覆盖整图，须≥整图） |
| `h_weight` | `1.0` | `1.0` | `1.0` | 启发式权重。必须 ≤ c_free(=1.0) 才可采纳/一致； |
| `w_dir` | `6.0` | `6.0` | `6.0` | A* 起点速度方向引导权重（§24 补充）：起点节点邻居方向与 |
| `plan_pad` | `15.0` | `18.0` | `18.0` | 可规划范围 x/y 每边比感知范围多出的缓冲（米）。绕楼在水平面， |
| `plan_pad_z` | `0.0` | `0.0` | `0.0` | z 方向额外缓冲（米）。绕楼是水平绕行、不需要垂直探索，z 不扩 |
| `bspline_order` | `3` | `3` | `3` | 三次 |
| `bspline_n_ctrl` | `20` | `20` | `20` | 控制点下界（短路径用） |
| `bspline_ctrl_spacing` | `1.2` | `1.2` | `1.2` | 控制点弧长间距（米），决定贴合 A* 航点的密度： |
| `bspline_max_ctrl` | `60` | `60` | `60` | 控制点数量上限（长路径自适应的封顶，防优化过慢） |
| `w_goal` | `1.0` | `1.0` | `1.0` |  |
| `w_smooth` | `0.2` | `0.2` | `0.2` | 平滑权重（降低，只润滑拐角、不抹平折线） |
| `w_dyn` | `0.4` | `0.4` | `0.4` |  |
| `bspline_iters` | `30` | `30` | `30` | 每轮优化迭代 |
| `bspline_step` | `0.08` | `0.08` | `0.08` | 梯度步长 |
| `splice_margin` | `0.1` | `0.1` | `0.1` | 拼接点前移时间（秒），覆盖规划延迟 |
| `smooth_skip` | `2` | `2` | `2` | 平滑/加速度代价跳过起点附近几个差分项（允许起点加速） |
| `v_mission` | `3.5` | `5.0` | `8.0` | 任务巡航速度，m/s |
| `v_max` | `6.0` | `6.0` | `8.0` | 载具速度上限，m/s |
| `a_max` | `2.0` | `3.5` | `3.5` | 加速度上限，m/s^2 |
| `kp` | `1.2` | `1.2` | `1.2` | 轨迹跟踪位置增益 |
| `kd` | `1.0` | `1.0` | `1.0` | 轨迹跟踪速度增益 |
| `control_dt` | `0.02` | `0.03333333333333333` | `0.03333333333333333` | 控制周期，秒（50Hz） |
| `velocity_duration` | `1.0` | `1.0` | `1.0` | moveByVelocityZAsync 指令保持时长（须>控制周期，否则指令过期速度归零） |
| `goal_tol` | `1.0` | `1.0` | `1.0` | 到达判据，米 |
| `sensor_hz` | `5.0` | `5.0` | `5.0` | LiDAR 点云反序列化慢，降低频率避免饿死控制环 |
| `mapping_hz` | `10.0` | `4.0` | `4.0` |  |
| `planning_hz` | `10.0` | `3.0` | `3.0` |  |
| `control_hz` | `50.0` | `30.0` | `30.0` |  |
| `replan_fraction` | `0.333` | `0.333` | `0.333` | 当前轨迹执行满该比例才重规划（1/3：运行 1/3 预测轨迹后就更新， |
| `ip` | `"127.0.0.1"` | `"127.0.0.1"` | `"127.0.0.1"` |  |
| `port` | `41451` | `41451` | `41451` |  |
| `vehicle_name` | `""` | `""` | `""` | 空串 = 默认载具 |
| `lidar_name` | `"Lidar1"` | `"Lidar1"` | `"Lidar1"` | 对应 settings.json 里的 Lidar1；未配置时自动回退深度图 |
| `camera_name` | `"0"` | `"0"` | `"0"` | 深度回退用的前中相机 |
| `takeoff_altitude` | `3.0` | `3.0` | `3.0` | 起飞后离地高度，米 |
| `flight_altitude` | `8.0` | `10.0` | `10.0` | 目标巡航高度（NED：发送时取负），米 |
| `use_persistent_map` | `true` | `true` | `true` | 是否启用持久图先验加载与写回；False = 回到第一阶段行为 |
| `persistent_res` | `0.6` | `0.8` | `0.8` | 持久图体素边长 r_G，米。必须是 res(0.2) 的整数倍（保证坐标对齐） |
| `persistent_block` | `16` | `16` | `16` | 稀疏块边长（体素数），§5 |
| `persistent_origin` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` | 单一世界锚点（NED），索引可为负 |
| `persistent_map_path` | `"data/maps/persistent_map.npz"` | `"data/maps/persistent_safe.npz"` | `"data/maps/persistent_safe.npz"` | 相对 NavigationPhase1 目录 |
| `persistent_save_interval` | `5.0` | `5.0` | `5.0` | 落盘间隔（秒）；任务结束必落盘 |
| `persistent_integrate_interval` | `1.0` | `2.0` | `2.0` | 全向建图周期（秒）：把整帧点云未裁剪地写进持久图（§54） |
| `seed` | `0` | `0` | `0` |  |
| `log_dir` | `"data/records"` | `"data/records"` | `"data/records"` |  |
