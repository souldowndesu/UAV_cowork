# -*- coding: utf-8 -*-
"""第一阶段主入口：连接 → 起飞 → 爬升 → 闭环导航 → 到达 → 降落。

用法（在 CompetitionEnv 下，先启动 CityEnviron.exe）：
    ..\\..\\.venv\\Scripts\\python.exe run_phase1.py --goal-x 30 --goal-y 0

坐标：NED 世界系（+X 北 / +Y 东 / +Z 向下），高度传负值。
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

from config import Config, parse_overrides
from nav.airsim_io import AirSimIO
from nav.pipeline import NavigationPipeline
from nav.recorder import Recorder
from nav.web_server import VizServer


def run_mission(io, cfg, goal, on_tick=None,
                recorder=None, viz_server=None,
                stuck_timeout=30.0, stuck_dist=2.0,
                autosave_path=None, autosave_interval=5.0):
    """跑一个完整任务，返回 (reached, telemetry, duration, outcome)。

    ``recorder`` 非空时完整录制；``viz_server`` 非空时启动可视化服务（需 recorder）。
    ``outcome`` ∈ {'reached', 'collided', 'stuck'}：卡住 = 连续 ``stuck_timeout`` 秒内
    位置位移 < ``stuck_dist`` 米（无人机顶墙/原地振荡，不再前进）。
    ``autosave_path`` 非空时，每 ``autosave_interval`` 秒把录制落盘一次（防进程被
    硬杀后丢失全部数据）；路径会被覆盖式重写，最终由调用方再保存一次完整版。
    """
    io.arm()
    io.takeoff()
    time.sleep(1.0)
    io.move_to_z(-cfg.flight_altitude, 2.0)

    pipe = NavigationPipeline(io, cfg)
    pipe.set_goal(goal)
    if recorder is not None:
        pipe.recorder = recorder
        recorder.set_goal(goal)
    if viz_server is not None:
        viz_server.recorder = recorder
        viz_server.start()
    pipe.start()

    t0 = time.time()
    outcome = "error"
    last_progress_pos = None
    last_progress_t = time.time()
    last_autosave_t = t0
    try:
        while True:
            time.sleep(0.2)
            if on_tick:
                on_tick(pipe)
            # 周期性自动落盘：保证进程被硬杀/崩溃时磁盘上仍有最近的完整录制
            if recorder is not None and autosave_path and \
                    time.time() - last_autosave_t >= autosave_interval:
                try:
                    d = os.path.dirname(autosave_path)
                    if d:
                        os.makedirs(d, exist_ok=True)
                    recorder.save(autosave_path)
                except Exception as e:
                    print(f"[Mission] 自动落盘失败：{e}")
                last_autosave_t = time.time()
            if pipe.reached:
                outcome = "reached"
                print(f"[Mission] 到达目标 {goal} (耗时 {time.time()-t0:.1f}s)")
                break
            if pipe.collided:
                outcome = "collided"
                print("[Mission] 检测到碰撞，中止")
                break
            # 卡住检测：基于最近遥测位置，长时间无显著前进则中止（避免无限顶墙且保证能保存数据）
            tl = pipe.telemetry()
            if tl:
                last = tl[-1]
                pos = np.array([last["x"], last["y"], last["z"]], dtype="float64")
                if last_progress_pos is None or \
                        np.linalg.norm(pos - last_progress_pos) >= stuck_dist:
                    last_progress_pos = pos
                    last_progress_t = time.time()
                elif time.time() - last_progress_t > stuck_timeout:
                    outcome = "stuck"
                    print(f"[Mission] 卡住（{stuck_timeout:.0f}s 内位移 < {stuck_dist}m），中止")
                    break
    finally:
        pipe.stop()

    telemetry = pipe.telemetry()
    try:
        io.land()
    except Exception:
        pass
    return pipe.reached, telemetry, time.time() - t0, outcome


def main(argv=None):
    parser = argparse.ArgumentParser(description="第一阶段 AirSim 自主导航")
    parser.add_argument("--goal-x", type=float, default=20.0, help="目标 X（NED 北）")
    parser.add_argument("--goal-y", type=float, default=0.0, help="目标 Y（NED 东）")
    parser.add_argument("--goal-z", type=float, default=None,
                        help="目标 Z（NED 向下；默认 -flight_altitude）。给不同高度即触发三维路径")
    parser.add_argument("--record", action="store_true", help="录制完整过程到 results/")
    parser.add_argument("--visualize", action="store_true", help="运行中启动浏览器四视图服务")
    parser.add_argument("--viz-port", type=int, default=8765, help="可视化服务端口")
    args, rest = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    cfg = Config.from_overrides(parse_overrides(rest))
    io = AirSimIO(cfg)

    # 目标 z：默认取巡航高度；显式给 --goal-z 即三维目标（NED 向下）
    goal_z = -cfg.flight_altitude if args.goal_z is None else args.goal_z
    goal = np.array([args.goal_x, args.goal_y, goal_z], dtype="float64")
    print(f"[Main] 目标 = {goal}，配置 = { {k: cfg.to_dict()[k] for k in ('res','d_max','d_safe','v_max')} }")

    recorder = Recorder(map_size=cfg.map_size) if (args.record or args.visualize) else None
    viz = VizServer(recorder=recorder, port=args.viz_port) if args.visualize else None

    reached = False
    telemetry = []
    dur = 0.0
    outcome = "error"
    try:
        reached, telemetry, dur, outcome = run_mission(io, cfg, goal,
                                                       recorder=recorder, viz_server=viz)
    except KeyboardInterrupt:
        print("[Main] 被 Ctrl+C 中断，保存已录制部分后退出")
    except Exception as e:
        print(f"[Main] 运行异常：{e}")
    finally:
        if recorder is not None and args.record:
            try:
                import os
                ts = time.strftime("%Y%m%d_%H%M%S")
                outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), cfg.log_dir, ts)
                os.makedirs(outdir, exist_ok=True)
                rpath = recorder.save(os.path.join(outdir, "recording.json"))
                print(f"[Main] 录制已保存到 {rpath}（回放：python replay.py）")
            except Exception as e:
                print(f"[Main] 保存录制失败：{e}")

    if viz is not None:
        print(f"[Main] 四视图服务地址 {viz.url}（回车退出）")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            pass
        viz.stop()

    if telemetry:
        last = telemetry[-1]
        print(f"[Main] 结束位置=({last['x']:.2f},{last['y']:.2f},{last['z']:.2f}) "
              f"目标距离={last['goal_dist']:.2f}m 碰撞={last['collision']}")
    print(f"[Main] 完成：到达={reached}，耗时={dur:.1f}s")
    return 0 if reached else 1


if __name__ == "__main__":
    sys.exit(main())
