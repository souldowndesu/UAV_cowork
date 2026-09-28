# -*- coding: utf-8 -*-
"""诊断：teleport 后无人机的速度/位置演化（拿硬数据，不猜测）。

测三组：
  A. 当前状态（不动），采样 5 次看是否被外部进程拖动
  B. teleport 到 (0,0,-0.5)，采样 pos/vel 演化
  C. teleport 到地面 (0,0,0)（scene_test 用的 start_z），采样 pos/vel 演化
每组采样多时间点，确认：
  1. simSetVehiclePose 是否清零速度
  2. teleport 后无人机是否漂移/坠落
  3. 是否有外部控制权在拖动无人机
"""
import sys
import time

sys.path.insert(0, ".")
from config import Config
from nav.airsim_io import AirSimIO

cfg = Config()
io = AirSimIO(cfg)


def sample(n=6, dt=0.5):
    rows = []
    for i in range(n):
        s = io.get_state()
        v = float((s.velocity ** 2).sum() ** 0.5)
        rows.append((time.time(), s.position, v, s.collision))
        if i < n - 1:
            time.sleep(dt)
    return rows


def dump(title, rows):
    print(f"\n===== {title} =====")
    p0 = rows[0][1]
    for t, p, v, col in rows:
        d = float(((p - p0) ** 2).sum() ** 0.5)
        print(f"  t=+{t-rows[0][0]:4.1f}s pos=({p[0]:7.2f},{p[1]:7.2f},{p[2]:7.2f}) "
              f"|v|={v:5.2f} drift_from_t0={d:5.2f} col={col}")


print("=== A. 当前状态基线（不 teleport，观察是否被外部拖动）===")
dump("A baseline", sample(6, 0.5))

print("\n=== B. teleport 到 (0,0,-0.5) ===")
io.teleport([0.0, 0.0, -0.5], 0.0)
dump("B teleport(-0.5)", sample(8, 0.5))

print("\n=== C. teleport 到地面 (0,0,0) ===")
io.teleport([0.0, 0.0, 0.0], 0.0)
dump("C teleport(0.0)", sample(8, 0.5))

print("\n=== D. 复位到 (0,0,-0.5) 供后续 mission 用 ===")
io.teleport([0.0, 0.0, -0.5], 0.0)
time.sleep(1.0)
s = io.get_state()
print(f"  final pos=({s.position[0]:.2f},{s.position[1]:.2f},{s.position[2]:.2f}) "
      f"|v|={float((s.velocity**2).sum()**0.5):.2f}")
