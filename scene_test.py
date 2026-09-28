# -*- coding: utf-8 -*-
"""固定场景实测脚本（固定位置 / 固定任务，用于反复调参对比）。

- 瞬移到固定起点 → 起飞爬升 → 闭环导航到固定目标 → 降落。
- 输出遥测（位置/速度/净空/制动/目标距离/碰撞）到 ``results/<时间戳>/`` 的 JSON + CSV。
- **交互式**：不带目标/起点参数直接运行，会在终端逐项询问（回车用默认值）。
- 高级调参仍可用命令行 ``--res/--dmax/--dsafe/--c-unknown/--v-max/...`` 覆盖。

用法（先启动 CityEnviron.exe）：
    # 交互式（推荐，直接回车用默认）
    ..\\..\\.venv\\Scripts\\python.exe scene_test.py

    # 命令行指定
    ..\\..\\.venv\\Scripts\\python.exe scene_test.py --goal-x 40 --goal-y 5 --plot
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time

import numpy as np

from config import Config, parse_overrides
from nav.airsim_io import AirSimIO
from run_phase1 import run_mission


def _prompt(label, default, cast=float):
    try:
        s = input(f"{label} [默认 {default}]: ").strip()
    except (EOFError, KeyboardInterrupt):
        s = ""
    if s == "":
        return default
    try:
        return cast(s)
    except ValueError:
        print(f"  无法解析 '{s}'，使用默认 {default}")
        return default


def _yn(label, default=True):
    try:
        s = input(f"{label} [默认 {'y' if default else 'n'}]: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        s = ""
    if s == "":
        return default
    return s.startswith("y")


def _interactive():
    print("=== 固定场景实测（交互式）===")
    print("每项直接回车使用方括号内的默认值；Ctrl+C 退出。")
    print("坐标：NED（X=北 / Y=东），高度为离地米数（内部转负 z）。\n")
    d = {}
    d["start_x"] = _prompt("起点 X (北, 米)", 0.0)
    d["start_y"] = _prompt("起点 Y (东, 米)", 0.0)
    d["goal_x"] = _prompt("目标 X (北, 米)", 40.0)
    d["goal_y"] = _prompt("目标 Y (东, 米)", 5.0)
    d["alt"] = _prompt("巡航高度 (米)", 8.0)
    d["goal_alt"] = _prompt("目标高度 (米, 回车=同巡航高度→二维)", d["alt"])
    d["record"] = _yn("是否录制完整过程", True)
    d["visualize"] = _yn("是否启动三维四视图", True)
    d["plot"] = _yn("是否画轨迹图", False)
    return d


def save_telemetry(telemetry, goal, outdir):
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "telemetry.json"), "w", encoding="utf-8") as f:
        json.dump({"goal": list(goal), "records": telemetry}, f, indent=2, ensure_ascii=False)
    with open(os.path.join(outdir, "telemetry.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(telemetry[0].keys()) if telemetry else [])
        w.writeheader()
        w.writerows(telemetry)
    print(f"[SceneTest] 遥测已保存到 {outdir}")


def summarize(telemetry, goal):
    if not telemetry:
        print("[SceneTest] 无遥测数据")
        return
    pos = np.array([[r["x"], r["y"], r["z"]] for r in telemetry])
    spd = np.linalg.norm(np.array([[r["vx"], r["vy"], r["vz"]] for r in telemetry]), axis=1)
    clear = np.array([r["clearance"] for r in telemetry])
    final = pos[-1]
    dist = float(np.linalg.norm(final - np.asarray(goal)))
    print("\n===== 实测汇总 =====")
    print(f"到达目标: {dist < 1.5}")
    print(f"终点到目标距离: {dist:.2f} m")
    print(f"路径长度: {float(np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1))):.2f} m")
    print(f"最小净空: {float(np.min(clear)):.2f} m")
    print(f"平均速度: {float(np.mean(spd)):.2f} m/s  峰值: {float(np.max(spd)):.2f} m/s")
    print(f"制动占比: {float(np.mean([r['braking'] for r in telemetry]))*100:.1f} %")
    print(f"碰撞: {any(r['collision'] for r in telemetry)}")


def plot_trajectory(telemetry, goal, outdir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        print("[SceneTest] matplotlib 不可用，跳过绘图")
        return
    pos = np.array([[r["x"], r["y"]] for r in telemetry])
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(pos[:, 0], pos[:, 1], "-", color="tab:blue", label="轨迹")
    ax.plot(pos[0, 0], pos[0, 1], "go", label="起点")
    ax.plot(goal[0], goal[1], "r*", markersize=14, label="目标")
    ax.set_xlabel("X (北, m)")
    ax.set_ylabel("Y (东, m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend()
    plt.title("第一阶段导航实测轨迹")
    path = os.path.join(outdir, "trajectory.png")
    plt.savefig(path, dpi=120)
    print(f"[SceneTest] 轨迹图已保存到 {path}")


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(description="固定场景实测")
    parser.add_argument("--start-x", type=float, default=None)
    parser.add_argument("--start-y", type=float, default=None)
    parser.add_argument("--start-z", type=float, default=0.0)
    parser.add_argument("--start-yaw", type=float, default=0.0)
    parser.add_argument("--goal-x", type=float, default=None)
    parser.add_argument("--goal-y", type=float, default=None)
    parser.add_argument("--goal-z", type=float, default=None,
                        help="目标 Z（NED 向下；默认 -flight_altitude），给不同高度触发三维路径")
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--record", action="store_true", help="录制完整过程")
    parser.add_argument("--visualize", action="store_true", help="运行中启动浏览器四视图服务")
    parser.add_argument("--viz-port", type=int, default=8765)
    parser.add_argument("--interactive", "-i", action="store_true",
                        help="交互式逐项输入（不带目标/起点参数时默认进入）")
    parser.add_argument("--runs", type=int, default=1,
                        help="批量测试轮数（每轮 teleport 回起点，独立数据目录）")
    parser.add_argument("--stuck-timeout", type=float, default=30.0,
                        help="卡住判定：连续无显著进展秒数（0 关闭卡住检测）")
    args, rest = parser.parse_known_args(argv)

    cfg = Config.from_overrides(parse_overrides(rest))

    # 判定是否交互式：显式 --interactive，或目标/起点没给全
    need_interactive = args.interactive or \
        (args.start_x is None or args.start_y is None or
         args.goal_x is None or args.goal_y is None)

    if need_interactive:
        d = _interactive()
        start_x = args.start_x if args.start_x is not None else d["start_x"]
        start_y = args.start_y if args.start_y is not None else d["start_y"]
        goal_x = args.goal_x if args.goal_x is not None else d["goal_x"]
        goal_y = args.goal_y if args.goal_y is not None else d["goal_y"]
        cfg.flight_altitude = d["alt"]
        cfg.__post_init__()
        goal_z = args.goal_z if args.goal_z is not None else -d["goal_alt"]
        record = bool(args.record or d["record"])
        visualize = bool(args.visualize or d["visualize"])
        args.plot = args.plot or d["plot"]
    else:
        start_x, start_y = args.start_x, args.start_y
        goal_x, goal_y = args.goal_x, args.goal_y
        goal_z = -cfg.flight_altitude if args.goal_z is None else args.goal_z
        record = args.record
        visualize = args.visualize

    io = AirSimIO(cfg)
    start = np.array([start_x, start_y, args.start_z], dtype="float64")
    goal = np.array([goal_x, goal_y, goal_z], dtype="float64")
    print(f"[SceneTest] 起点={start}  目标={goal}  偏航={args.start_yaw}°  轮数={args.runs}")

    from nav.recorder import Recorder
    from nav.web_server import VizServer

    # 批量测试强制关闭可视化（input() 会阻塞循环），单轮可保留
    if args.runs > 1 and visualize:
        print("[SceneTest] 批量测试（--runs>1）自动关闭可视化")
        visualize = False

    ts = time.strftime("%Y%m%d_%H%M%S")
    # 输出目录固定到脚本所在目录（避免相对 CWD 漂移）
    base_outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), cfg.log_dir, ts)

    all_outcomes = []
    for run_i in range(args.runs):
        # 1) 每轮开始：teleport 回起点，并等待稳定
        #    （simSetVehiclePose 只改位置不清速度，自由落体到地面约 2s，需等 |v| 归零）
        io.teleport(start, args.start_yaw)
        time.sleep(2.5)

        outdir = base_outdir if args.runs == 1 else os.path.join(base_outdir, f"run_{run_i:02d}")
        recorder = Recorder(map_size=cfg.map_size) if (record or visualize) else None
        viz = VizServer(recorder=recorder, port=args.viz_port) if visualize else None

        reached = False
        outcome = "error"
        telemetry = []
        dur = 0.0
        try:
            os.makedirs(outdir, exist_ok=True)
            reached, telemetry, dur, outcome = run_mission(
                io, cfg, goal, recorder=recorder, viz_server=viz,
                stuck_timeout=args.stuck_timeout,
                autosave_path=os.path.join(outdir, "recording.json"),
                autosave_interval=5.0)
        except KeyboardInterrupt:
            print("[SceneTest] 被 Ctrl+C 中断，保存已录制部分后退出")
        except Exception as e:
            print(f"[SceneTest] 第 {run_i+1} 轮运行异常：{e}")

        # 2) 每轮结束：无论到达/碰撞/卡住/异常，都保存本轮数据
        if recorder is not None and record:
            try:
                os.makedirs(outdir, exist_ok=True)
                rpath = recorder.save(os.path.join(outdir, "recording.json"))
                print(f"[SceneTest] 第 {run_i+1} 轮录制已保存到 {rpath}（回放：python replay.py）")
            except Exception as e:
                print(f"[SceneTest] 第 {run_i+1} 轮保存录制失败：{e}")

        if viz is not None:
            print(f"[SceneTest] 四视图服务地址 {viz.url}（回车退出）")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                pass
            viz.stop()

        if telemetry:
            save_telemetry(telemetry, goal, outdir)
            summarize(telemetry, goal)
            if args.plot:
                plot_trajectory(telemetry, goal, outdir)

        all_outcomes.append((run_i + 1, outcome, reached, dur))
        print(f"[SceneTest] 第 {run_i+1} 轮结果：outcome={outcome} 到达={reached}")

    if args.runs > 1:
        n_reached = sum(1 for _, _, r, _ in all_outcomes if r)
        print(f"\n[SceneTest] 批量完成：{n_reached}/{args.runs} 轮到达目标")
        for idx, oc, rc, d in all_outcomes:
            print(f"  第 {idx} 轮: outcome={oc} 到达={rc} 耗时={d:.1f}s")
    return 0 if (all_outcomes and all_outcomes[-1][2]) else 1


if __name__ == "__main__":
    sys.exit(main())
