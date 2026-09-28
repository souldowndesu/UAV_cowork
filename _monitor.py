# -*- coding: utf-8 -*-
"""临时监控脚本：直接连 AirSim 读无人机位置/速度，观察飞行进度（独立于主任务进程）。"""
import sys
import time

sys.path.insert(0, ".")


def main():
    from config import Config
    from nav.airsim_io import AirSimIO

    cfg = Config()
    io = AirSimIO(cfg)
    goal = (900.0, 20.0, -20.0)

    for _ in range(60):
        try:
            s = io.get_state()
            p = s.position
            d = ((p[0] - goal[0]) ** 2 + (p[1] - goal[1]) ** 2 + (p[2] - goal[2]) ** 2) ** 0.5
            print(f"pos=({p[0]:7.2f},{p[1]:7.2f},{p[2]:7.2f}) "
                  f"|v|={float((s.velocity**2).sum()**0.5):5.2f} "
                  f"goal_dist={d:8.2f} collision={s.collision}",
                  flush=True)
        except Exception as e:
            print(f"[monitor] 读取失败：{e}", flush=True)
        time.sleep(2.0)


if __name__ == "__main__":
    main()
