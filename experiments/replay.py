# -*- coding: utf-8 -*-
"""回放服务器：自动扫描录制目录，浏览器页面里下拉选择任意一次录制，拖动进度回放。

用法（在 NavigationPhase1 下）：
    ..\\.venv\\Scripts\\python.exe replay.py
然后浏览器打开 http://127.0.0.1:8765/ ，在页面顶部下拉框选择录制。
默认点云回放，可勾选“模拟器场景”；先用 replay_scene.cmd 启动回放用模拟器。
服务持续运行到 Ctrl+C，不依赖 stdin；--serve 保留兼容旧命令。
"""
from __future__ import annotations

import argparse
import os
import sys
import math
import time

from pathlib import Path
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from experiments.recorder import RecordingLibrary
from experiments.web_server import VizServer
from navigation.paths import RECORDS, EXAMPLES, project_path


def main(argv=None):
    parser = argparse.ArgumentParser(description="回放录制（自动扫描 + 页面选择）")
    parser.add_argument("--results-dir", default=None,
                        help="录制目录（默认 data/records，与运行位置无关）")
    parser.add_argument("--port", type=int, default=8765, help="服务端口")
    parser.add_argument('--examples-dir', default=None, help='示例录像目录（默认 data/examples/flights）')
    parser.add_argument('--no-examples', action='store_true', help='只显示运行记录')
    parser.add_argument('--simulator', action=argparse.BooleanOptionalAction, default=None,
                        help='默认提供场景切换；--simulator 启动即显示场景，--no-simulator 仅提供点云')
    parser.add_argument('--sim-ip', default='127.0.0.1')
    parser.add_argument('--sim-port', type=int, default=41451)
    parser.add_argument('--sim-vehicle', default='', help='AirSim 载具名称，空值使用默认载具')
    parser.add_argument('--sim-camera', default='ReplayCamera', help='environment/settings_replay.json 中的外部相机')
    parser.add_argument('--sim-fps', type=float, default=12., help='场景刷新频率上限，1–30')
    parser.add_argument("--serve",action="store_true",help="兼容旧命令；现在默认持续服务，不依赖 stdin")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])
    if not math.isfinite(args.sim_fps) or not 1<=args.sim_fps<=30:parser.error('--sim-fps 必须为 1–30')

    # 默认锚定到脚本所在目录，避免从其它目录运行时 results 解析错
    results_dir = str(RECORDS if args.results_dir is None else project_path(args.results_dir))
    examples_dir = str(EXAMPLES/'flights' if args.examples_dir is None else project_path(args.examples_dir))
    lib = RecordingLibrary(results_dir, collections=None if args.no_examples else
                           {'examples': ('飞行示例 · 24 次验证', examples_dir)})
    metas = lib.scan()
    print(f"[Replay] 找到 {len(metas)} 条录制（运行记录：{os.path.abspath(results_dir)}）：")
    for i, m in enumerate(metas):
        if m.get('error'):
            print(f"  [{i}] {m['name']}  读取失败：{m['error']}")
            continue
        dur = (m.get("t1") - m.get("t0")) if (m.get("t1") is not None and m.get("t0") is not None) else None
        goal = m.get("goal")
        goal_s = ",".join(f"{g:.0f}" for g in goal) if goal else "?"
        dur_s = f"{dur:.1f}s" if dur is not None else "?"
        print(f"  [{i}] [{m['collection_name']}] {m.get('display_name',m['name'])}  目标[{goal_s}]  时长{dur_s}  "
              f"状态{m['n_states']}帧 点云{m['n_points']}帧 重规划{m['n_plans']}次")
    if not metas:
        print("[Replay] 没有录制。先跑一次带 --record 的任务：")
        print("        python scene_test.py --goal-x 10 --goal-y 0 --record")

    simulator=None
    if args.simulator is not False:
        from experiments.simulator_replay import SimulatorReplay
        simulator=SimulatorReplay(lib,args.sim_ip,args.sim_port,args.sim_vehicle,args.sim_camera,args.sim_fps)
        print('[Replay] 可勾选“模拟器场景”；请先用 replay_scene.cmd 启动回放用模拟器。')
    srv = VizServer(library=lib, port=args.port,simulator=simulator,simulator_default=args.simulator is True)
    try:
        srv.start()
        print(f"[Replay] 服务地址 {srv.url}（页面里下拉选择录制；Ctrl+C 退出）")
        while True:time.sleep(.5)
    except KeyboardInterrupt:
        pass
    finally:
        srv.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
