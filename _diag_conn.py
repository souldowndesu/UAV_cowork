# -*- coding: utf-8 -*-
"""诊断：检查 start 与 goal 是否在同一连通分量（可通行）。"""
import sys
sys.path.insert(0, ".")

import time
import numpy as np
from scipy import ndimage
from config import Config
from nav.airsim_io import AirSimIO
from nav.pipeline import NavigationPipeline
from nav import distance_field

cfg = Config()
io = AirSimIO(cfg)

STUCK = np.array([119.37, 20.17, -21.41])
io.teleport(STUCK, yaw_deg=0.0)
time.sleep(0.5)

pipe = NavigationPipeline(io, cfg)
pipe.set_goal(np.array([900.0, 20.0, -20.0]))
pipe.start()
for _ in range(40):
    time.sleep(0.2)
    if pipe._snapshot is not None:
        break
snap = pipe._snapshot
pipe.stop()

coarse = NavigationPipeline._coarsen(snap.occ, snap.origin, cfg.res, cfg.plan_res)
coarse_dist = distance_field.compute(coarse, cfg)
goal = NavigationPipeline._local_goal(snap.state_pos, pipe.goal, snap.origin,
                                      cfg.res, cfg.nx, cfg.ny, cfg.nz)
proj = NavigationPipeline._project_goal_free(coarse, coarse_dist, snap.state_pos,
                                             goal, min_cl=float(cfg.d_safe))

occ = coarse.data
passable = occ != 2  # FREE(1) 与 UNKNOWN(0) 都可通行
lbl, n = ndimage.label(passable)  # 26 连通
sv = coarse.world_to_voxel(snap.state_pos)
gv = coarse.world_to_voxel(goal)
pv = coarse.world_to_voxel(proj)
print(f"[diag] start={sv} goal={gv} proj={pv}")
print(f"[diag] start_label={lbl[sv[0],sv[1],sv[2]]} "
      f"goal_label={lbl[gv[0],gv[1],gv[2]]} "
      f"proj_label={lbl[pv[0],pv[1],pv[2]]}  (共 {n} 个分量)")
# 各分量体素数（降序）
sizes = np.bincount(lbl.ravel())[1:]
print(f"[diag] top component sizes={np.sort(sizes)[::-1][:5]}")

# 打印 Z 切片（start 层 与 goal 层），正确标注 Y（世界系）
for name, k in [("start_z", int(sv[2])), ("goal_z", int(gv[2]))]:
    slab = occ[:, :, k]
    print(f"\n[diag] === {name} z_voxel={k}  (0=unk .=free ##=occ) ===")
    oy, ox = coarse.origin[1], coarse.origin[0]
    ys = list(range(0, 41, 5))
    print("y=    " + " ".join(f"{y:4d}" for y in ys))
    for xi in range(coarse.nx):
        x = ox + (xi + 0.5) * coarse.res
        if x < 108 or x > 153:
            continue
        row = []
        for y in ys:
            ci = int((y - oy) / coarse.res)
            if 0 <= ci < coarse.ny:
                v = slab[xi, ci]
                row.append("##" if v == 2 else ("." if v == 1 else "  "))
            else:
                row.append("  ")
        print(f"x={x:5.1f} " + " ".join(row))
