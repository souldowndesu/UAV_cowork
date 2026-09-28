# -*- coding: utf-8 -*-
"""细网格扫描建筑布局（X=115-175, Y=-10~45, Z=-20），5m 网格。"""
import sys
sys.path.insert(0, ".")

import time
import numpy as np
from config import Config
from nav.airsim_io import AirSimIO

cfg = Config()
io = AirSimIO(cfg)


def probe(x, y, z):
    io.teleport([x, y, z], yaw_deg=0.0)
    time.sleep(0.2)
    pts = io.get_lidar_points()
    if len(pts) == 0:
        return -1.0
    d = np.linalg.norm(pts - np.array([x, y, z]), axis=1)
    return float(d.min())


def main():
    z = -20.0
    xs = list(range(115, 176, 5))
    ys = list(range(-10, 46, 5))
    print("      " + " ".join(f"{y:6d}" for y in ys))
    for x in xs:
        row = []
        for y in ys:
            d = probe(x, y, z)
            row.append(d)
        cells = " ".join(
            ("####" if d < 3.0 else ("  .." if d < 10.0 else "    ")) for d in row)
        print(f"x={x:4d} {cells}")
        print("      " + " ".join(f"{d:5.1f}" for d in row))


if __name__ == "__main__":
    main()
