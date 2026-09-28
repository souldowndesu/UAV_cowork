# -*- coding: utf-8 -*-
"""临时脚本：重置无人机到起点 (0,0,0) 并解除 API 控制/锁定。"""
import sys
sys.path.insert(0, ".")

from config import Config
from nav.airsim_io import AirSimIO

cfg = Config()
io = AirSimIO(cfg)
io.teleport([0.0, 0.0, -0.5], yaw_deg=0.0)
print("[reset] 已重置到 (0,0,-0.5)")
s = io.get_state()
print(f"[reset] 当前 pos=({s.position[0]:.2f},{s.position[1]:.2f},{s.position[2]:.2f}) "
      f"|v|={float((s.velocity**2).sum()**0.5):.2f} collision={s.collision}")
