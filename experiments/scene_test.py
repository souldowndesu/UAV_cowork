# -*- coding: utf-8 -*-
"""固定场景实测脚本（固定位置 / 固定任务，用于反复调参对比）。

- 瞬移到固定起点 → 起飞爬升 → 闭环导航到固定目标 → 降落。
- 输出遥测（位置/速度/净空/制动/目标距离/碰撞）到 ``data/records/<时间戳>/`` 的 JSON + CSV。
- **交互式**：不带目标/起点参数直接运行，会在终端逐项询问（回车用默认值）。
- 高级调参仍可用命令行 ``--res/--d-max/--d-safe/--c-unknown/--v-max/...`` 覆盖。

用法（先启动 CityEnviron.exe）：
    # 交互式（推荐，直接回车用默认）
    ..\\.venv\\Scripts\\python.exe scene_test.py

    # 命令行指定
    ..\\.venv\\Scripts\\python.exe scene_test.py --goal-x 40 --goal-y 5 --plot
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from navigation.config import Config, parse_overrides
from navigation.airsim_io import AirSimIO
from experiments.mission import run_mission
from navigation.paths import RECORDS, project_path


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


def _interactive(cfg):
    print("=== 固定场景实测（交互式）===")
    print("每项直接回车使用方括号内的默认值；Ctrl+C 退出。")
    print("坐标：NED（X=北 / Y=东），高度相对世界原点（内部转负 z）。\n")
    d = {}
    d["start_x"] = _prompt("起点 X (北, 米)", 0.0)
    d["start_y"] = _prompt("起点 Y (东, 米)", 0.0)
    d["goal_x"] = _prompt("目标 X (北, 米)", 40.0)
    d["goal_y"] = _prompt("目标 Y (东, 米)", 5.0)
    d["alt"] = _prompt("起飞后导航高度 (米)", cfg.flight_altitude)
    d["speed"] = _prompt("巡航速度 (m/s)", cfg.v_mission)
    d["goal_alt"] = _prompt("目标高度 (米, 回车=同巡航高度→二维)", d["alt"])
    d["record"] = _yn("是否录制完整过程", True)
    d["visualize"] = _yn("是否启动实时网页", True)
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


def summarize(telemetry, goal, outcome=None):
    if not telemetry:
        print("[SceneTest] 无遥测数据")
        return
    pos = np.array([[r["x"], r["y"], r["z"]] for r in telemetry])
    spd = np.linalg.norm(np.array([[r["vx"], r["vy"], r["vz"]] for r in telemetry]), axis=1)
    clear = np.array([r["clearance"] for r in telemetry])
    final = pos[-1]
    dist = float(np.linalg.norm(final - np.asarray(goal)))
    print("\n===== 实测汇总 =====")
    print(f"任务结果: {outcome or ('reached' if dist < 1.5 else 'unfinished')}")
    if outcome == 'safe_approach':
        print(f"抵达目标附近安全位置: {telemetry[-1].get('approach_goal')}")
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
    parser = argparse.ArgumentParser(description="固定场景实测", allow_abbrev=False,
        epilog="Config 参数均可覆盖，例如 --flight-altitude 20 --v-max 6 --d-min 1 --a-max 3.5 --map-size '(60,60,32)'；优先级：profile < 场景 config < 命令行。")
    parser.add_argument("--start-x", type=float, default=0.)
    parser.add_argument("--start-y", type=float, default=0.)
    parser.add_argument("--start-z", type=float, default=-.5)
    parser.add_argument("--start-yaw", type=float, default=0.0)
    parser.add_argument("--goal-x", type=float, default=None)
    parser.add_argument("--goal-y", type=float, default=None)
    altitude_group=parser.add_mutually_exclusive_group()
    altitude_group.add_argument("--goal-z", type=float, default=None,
                        help="目标 Z（NED 向下；默认 -flight_altitude），给不同高度触发三维路径")
    altitude_group.add_argument('--goal-altitude',type=float,help='目标高度（正数米），自动转换为负 NED z')
    parser.add_argument('--profile',choices=('legacy','safe5','safe8'),default='safe5',help='默认 safe5；safe8 为实验档位')
    parser.add_argument('--speed',type=float,help='巡航速度 m/s（等同 v_mission）')
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--record", action=argparse.BooleanOptionalAction, default=True, help="默认实时保存回放数据；--no-record 关闭")
    parser.add_argument('--reset',action=argparse.BooleanOptionalAction,default=True,help='每轮重置仿真物理状态；--no-reset 保留现场')
    parser.add_argument('--autosave-interval',type=float,default=5.,help='持久地图回放快照/强制刷新周期，秒；流式录像约每秒写入')
    parser.add_argument("--visualize", action="store_true", help="运行中启动浏览器实时可视化服务")
    parser.add_argument("--viz-port", type=int, default=8765)
    parser.add_argument("--interactive", "-i", action="store_true",
                        help="交互式逐项输入（不带目标/起点参数时默认进入）")
    parser.add_argument("--runs", type=int, default=1,
                        help="批量测试轮数（每轮 teleport 回起点，独立数据目录）")
    parser.add_argument("--stuck-timeout", type=float, default=30.0,
                        help="卡住判定：连续无显著进展秒数（0 关闭卡住检测）")
    parser.add_argument("--evaluate", nargs="+", metavar="RECORDING",
                        help="离线评估 recording.json / manifest.json / 录制目录，不连接 AirSim")
    parser.add_argument("--evaluation-dir", default=None,
                        help="离线评估输出根目录（每条录制单独输出）")
    parser.add_argument("--scenario", help="固定场景 JSON；显式起点/目标参数及配置覆盖优先")
    parser.add_argument("--map-policy", choices=("cold", "snapshot", "shared"), default=None,
                        help="初始持久图：cold 每轮空图 / snapshot 每轮同一先验 / shared 共享学习（旧行为）")
    parser.add_argument("--max-duration", type=float, default=None,
                        help="导航阶段墙钟时限（秒）；0 不限时；固定场景默认 240 秒，不含起飞")
    args, rest = parser.parse_known_args(argv)

    if args.evaluate:
        from experiments.evaluation import evaluate_file
        output = project_path(args.evaluation_dir) if args.evaluation_dir else RECORDS / "evaluations"
        failed = False
        for source in args.evaluate:
            p = project_path(source)
            name = p.name if p.is_dir() else p.parent.name
            # Include the absolute input-path digest to avoid equal basename collisions.
            import hashlib
            dest = output / (name + "_" + hashlib.sha256(str(p).encode()).hexdigest()[:8])
            try:
                result = evaluate_file(p, dest, plot=args.plot)
                print(f"[Evaluate] {p}: states={result['state_count']} plans={result['plan_count']} outcome={result['mission_outcome']} -> {dest}")
            except Exception as exc:
                failed = True
                print(f"[Evaluate] {p}: 评估失败：{exc}", file=sys.stderr)
        return 1 if failed else 0

    from experiments.experiment import load_scenario, MapBaseline, run_metadata, write_metadata
    scenario = load_scenario(args.scenario) if args.scenario else None
    if scenario:
        for key, value in zip(("start_x", "start_y", "start_z"), scenario["start"]):
            if not any(a == "--" + key.replace("_", "-") or a.startswith("--" + key.replace("_", "-") + "=") for a in argv):
                setattr(args, key, value)
        for key, value in zip(("goal_x", "goal_y", "goal_z"), scenario["goal"]):
            if getattr(args, key) is None:
                setattr(args, key, value)
        if not any(a == "--start-yaw" or a.startswith("--start-yaw=") for a in argv):
            args.start_yaw = scenario.get("start_yaw", 0.)
    from navigation.profiles import profile_overrides
    overrides = profile_overrides(args.profile)
    if scenario:overrides.update(scenario.get('config',{}))
    try:
        overrides.update(parse_overrides(rest))
        if args.speed is not None:overrides['v_mission']=args.speed
        cfg = Config.from_overrides(overrides)
    except ValueError as exc:
        parser.error(str(exc))
    if args.goal_altitude is not None:
        if not np.isfinite(args.goal_altitude) or args.goal_altitude<=0:parser.error('--goal-altitude 必须为正有限数')
        args.goal_z=-args.goal_altitude
    if not np.isfinite(args.autosave_interval) or args.autosave_interval<=0:parser.error('--autosave-interval 必须为正有限数')
    max_duration = args.max_duration if args.max_duration is not None else (scenario.get("max_duration", 240.) if scenario else 0.)
    if not np.isfinite(max_duration) or max_duration < 0:
        parser.error("--max-duration 必须为非负有限数")
    if args.runs < 1:
        parser.error("--runs 必须至少为 1")

    # 判定是否交互式：显式 --interactive，或目标/起点没给全
    need_interactive = args.interactive or args.goal_x is None or args.goal_y is None

    if need_interactive:
        d = _interactive(cfg)
        explicit = lambda name: any(a == '--'+name or a.startswith('--'+name+'=') for a in argv)
        start_x = args.start_x if explicit('start-x') or scenario else d["start_x"]
        start_y = args.start_y if explicit('start-y') or scenario else d["start_y"]
        goal_x = args.goal_x if args.goal_x is not None else d["goal_x"]
        goal_y = args.goal_y if args.goal_y is not None else d["goal_y"]
        cfg.flight_altitude = d["alt"]
        cfg.v_mission = d['speed']
        cfg.__post_init__()
        goal_z = args.goal_z if args.goal_z is not None else -d["goal_alt"]
        record = args.record if explicit('record') or explicit('no-record') else d['record']
        visualize = bool(args.visualize or d["visualize"])
        args.plot = args.plot or d["plot"]
    else:
        start_x, start_y = args.start_x, args.start_y
        goal_x, goal_y = args.goal_x, args.goal_y
        goal_z = -cfg.flight_altitude if args.goal_z is None else args.goal_z
        record = args.record
        visualize = args.visualize

    if not np.isfinite(cfg.v_max) or cfg.v_max<=0 or not np.isfinite(cfg.v_mission) or cfg.v_mission<=0 or cfg.v_mission>cfg.v_max:
        parser.error('巡航速度必须为正且不超过 v_max；高于 6 m/s 可用 --profile safe8 或覆盖 --v-max')
    if not np.isfinite(cfg.flight_altitude) or cfg.flight_altitude<=0:
        parser.error('flight_altitude 必须为正有限数')
    if not np.isfinite([start_x,start_y,args.start_z,args.start_yaw,goal_x,goal_y,goal_z]).all():
        parser.error('起点、目标和偏航必须为有限数')

    io = AirSimIO(cfg)
    start = np.array([start_x, start_y, args.start_z], dtype="float64")
    goal = np.array([goal_x, goal_y, goal_z], dtype="float64")
    print(f"[SceneTest] 起点={start}  目标={goal}  偏航={args.start_yaw}°  轮数={args.runs}")

    from experiments.recorder import Recorder
    from experiments.web_server import VizServer

    # 批量测试强制关闭可视化（input() 会阻塞循环），单轮可保留
    if args.runs > 1 and visualize:
        print("[SceneTest] 批量测试（--runs>1）自动关闭可视化")
        visualize = False

    ts = time.strftime("%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1000000000:09d}"
    # 输出目录固定到脚本所在目录（避免相对 CWD 漂移）
    base_outdir = str(project_path(cfg.log_dir) / ts)
    policy = args.map_policy or (scenario.get("map_policy", "snapshot") if scenario else "shared")
    initial_config = cfg.to_dict()
    baseline = MapBaseline(cfg, policy, base_outdir)

    all_outcomes = []
    for run_i in range(args.runs):
        cfg = Config.from_overrides(initial_config)
        outdir = base_outdir if args.runs == 1 else os.path.join(base_outdir, f"run_{run_i:02d}")
        prior = baseline.prepare(cfg, outdir)
        np.random.seed(cfg.seed)
        metadata = run_metadata(cfg, start.tolist(), goal.tolist(), args.start_yaw, prior, scenario)
        metadata.update(max_duration_wall_s=max_duration, stuck_timeout_wall_s=args.stuck_timeout)
        metadata.update(reset_simulator=args.reset,profile=args.profile,record=record,autosave_interval_s=args.autosave_interval)
        metadata['initial_state_note']=('AirSim reset, API control/arming restored, then teleport and settle; actual preflight state recorded.'
            if args.reset else 'No simulator reset; teleport and settle only. Actual preflight state recorded.')
        os.makedirs(outdir, exist_ok=True)
        write_metadata(os.path.join(outdir, "run_meta.json"), metadata)
        # 1) 每轮开始：teleport 回起点，并等待稳定
        #    （simSetVehiclePose 只改位置不清速度，自由落体到地面约 2s，需等 |v| 归零）
        recorder = Recorder(map_size=cfg.map_size) if (record or visualize) else None
        viz = VizServer(recorder=recorder, port=args.viz_port) if visualize else None

        reached = False
        outcome = "error"
        telemetry = []
        rpath = None
        dur = 0.0
        try:
            if args.reset:io.reset()
            io.teleport(start, args.start_yaw)
            time.sleep(2.5)
            preflight = io.get_state()
            metadata["preflight_state"] = dict(position=preflight.position.tolist(),
                                               velocity=preflight.velocity.tolist(),
                                               timestamp_ns=preflight.timestamp_ns)
            write_metadata(os.path.join(outdir, "run_meta.json"), metadata)
            os.makedirs(outdir, exist_ok=True)
            reached, telemetry, dur, outcome = run_mission(
                io, cfg, goal, recorder=recorder, viz_server=viz,
                stuck_timeout=args.stuck_timeout,
                autosave_path=os.path.join(outdir, "recording.json") if record else None,
                autosave_interval=args.autosave_interval, max_duration=max_duration)
        except KeyboardInterrupt:
            outcome = "interrupted"
            print("[SceneTest] 被 Ctrl+C 中断，保存已录制部分后退出")
            try:io.land()
            except Exception:pass
        except Exception as e:
            print(f"[SceneTest] 第 {run_i+1} 轮运行异常：{e}")
            try:io.land()
            except Exception:pass

        # 2) 每轮结束：无论到达/碰撞/卡住/异常，都保存本轮数据
        if recorder is not None and record:
            try:
                os.makedirs(outdir, exist_ok=True)
                recorder.close()
                rpath = recorder.save(os.path.join(outdir, "recording.json"))
                print(f"[SceneTest] 第 {run_i+1} 轮录制已保存到 {rpath}（回放：python replay.py）")
            except Exception as e:
                print(f"[SceneTest] 第 {run_i+1} 轮保存录制失败：{e}")

        metadata.update(outcome=outcome, reached=bool(reached), duration_wall_s=dur,
                        telemetry_count=len(telemetry),original_goal_reached=outcome=='reached',
                        safe_approach_reached=outcome=='safe_approach',
                        approach_goal=telemetry[-1].get('approach_goal') if telemetry else None)
        write_metadata(os.path.join(outdir, "run_meta.json"), metadata)
        if telemetry:
            try:
                save_telemetry(telemetry, goal, outdir)
            except Exception as exc:
                print(f"[SceneTest] 遥测保存失败（录制和结果元数据已保留）：{exc}")
            summarize(telemetry, goal, outcome)
            if args.plot:
                plot_trajectory(telemetry, goal, outdir)

        from experiments.evaluation import write_evaluation
        evaluation_data = recorder.to_dict() if recorder is not None else {"goal": goal.tolist()}
        if rpath is not None:
            from experiments.recorder import load_recording_dict
            try:
                evaluation_data = load_recording_dict(rpath)
            except Exception as exc:
                print(f"[SceneTest] 完整录制评估读取失败，使用当前缓冲及完整遥测：{exc}")
        try:
            write_evaluation(evaluation_data, Path(outdir) / "evaluation", telemetry, metadata)
        except Exception as exc:
            print(f"[SceneTest] 评估失败（原始记录仍保留）：{exc}")

        all_outcomes.append((run_i + 1, outcome, reached, dur))
        print(f"[SceneTest] 第 {run_i+1} 轮结果：outcome={outcome} 到达={reached}")
        if viz is not None:
            print(f"[SceneTest] 实时网页地址 {viz.url}（数据已保存，回车退出）")
            try:input()
            except (EOFError,KeyboardInterrupt):pass
            viz.stop()
        if outcome == "interrupted":
            break

    if args.runs > 1:
        n_reached = sum(1 for _, _, r, _ in all_outcomes if r)
        print(f"\n[SceneTest] 批量完成：{n_reached}/{args.runs} 轮到达目标")
        for idx, oc, rc, d in all_outcomes:
            print(f"  第 {idx} 轮: outcome={oc} 到达={rc} 耗时={d:.1f}s")
    write_metadata(os.path.join(base_outdir, "batch_summary.json"), {
        "map_policy": policy,
        "runs": [dict(index=i, outcome=oc, reached=rc, duration_wall_s=d) for i, oc, rc, d in all_outcomes],
    })
    return 0 if (all_outcomes and all(r[2] for r in all_outcomes)) else 1


if __name__ == "__main__":
    sys.exit(main())
