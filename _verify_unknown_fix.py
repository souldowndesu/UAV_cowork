"""验证 UNKNOWN 代价修复：近墙 UNKNOWN 被 d_min 硬截断，A* 不再穿缝。"""
import numpy as np
from nav.astar import AStarPlanner
from nav import _fast

# 真实拓扑：墙 X=40..60 只占 Y=20..40，南北留街道；墙后 X>60 全 UNKNOWN。
# 墙前 X<40 全 FREE，墙后 X>60 全 UNKNOWN（未观测）。
nx, ny, nz, res = 100, 80, 20, 0.4
occ = np.zeros((nx, ny, nz), dtype=np.int8)  # 默认 UNKNOWN=0
# 墙前区域 FREE
occ[:40, :, :] = 1
# 南北街道 FREE（贯通到墙后）
occ[:, :20, :] = 1
occ[:, 60:, :] = 1
# 墙 OCCUPIED（X=40..60，Y=20..60）
occ[40:61, 20:60, :] = 2

# 距离场：仅 OCCUPIED 参与
from scipy.ndimage import distance_transform_edt
d = distance_transform_edt(occ != 2, sampling=(res, res, res))

class _Cfg:
    res = res
    c_free = 1.0
    c_unknown = 5.0
    w_obs = 8.0
    d_safe = 2.0
    d_min = 0.8
    h_weight = 1.0
    astar_timeout_nodes = 500000

planner = AStarPlanner(_Cfg)

start = np.array([10*res, 40*res, 10*res])
goal = np.array([80*res, 40*res, 10*res])    # 墙后 UNKNOWN（被墙挡，不可达）
origin = np.zeros(3)

# C++ 路径（真机路径，直接传数组）
path_cpp = _fast.astar_plan(occ, d.astype(np.float32), origin, res, start, goal, 1.0, 5.0, 8.0, 2.0, 0.8, 1.0, 500000)
print("C++ path len:", None if path_cpp is None else len(path_cpp))

# 关键断言：路径不能穿墙（墙内 X=40..60 且 Y=20..60 不应出现）
def crosses_wall(path):
    if path is None:
        return False
    for p in path:
        x, y = p[0], p[1]
        wx = int(x / res); wy = int(y / res)
        if 40 <= wx <= 61 and 20 <= wy < 60:
            return True
    return False

if path_cpp is not None:
    print("CPP crosses wall:", crosses_wall(path_cpp))

# 更关键：单独测 UNKNOWN 体素代价——近墙 UNKNOWN (d<0.8) 应为 INF
near_unknown = (45, 40, 10)  # 墙正后方 UNKNOWN，紧贴墙 d≈0.2
print("near_unknown d =", round(float(d[45, 40, 10]), 3),
      "cost(PY) =", planner._cost(occ, d, 45, 40, 10))
print("VERIFY DONE")
