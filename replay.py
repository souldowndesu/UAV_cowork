# -*- coding: utf-8 -*-
"""回放服务器：自动扫描录制目录，浏览器页面里下拉选择任意一次录制，拖动进度回放。

用法（在 NavigationPhase1 下）：
    ..\\..\\.venv\\Scripts\\python.exe replay.py
然后浏览器打开 http://127.0.0.1:8765/ ，在页面顶部下拉框选择录制。
"""
from __future__ import annotations

import argparse
import os
import sys

from nav.recorder import RecordingLibrary
from nav.web_server import VizServer


def main(argv=None):
    parser = argparse.ArgumentParser(description="回放录制（自动扫描 + 页面选择）")
    parser.add_argument("--results-dir", default=None,
                        help="录制目录（默认：脚本所在目录下的 results/，与运行位置无关）")
    parser.add_argument("--port", type=int, default=8765, help="服务端口")
    parser.add_argument("--serve",action="store_true",help="持续服务，不依赖 stdin")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    # 默认锚定到脚本所在目录，避免从其它目录运行时 results 解析错
    if args.results_dir is None:
        results_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
    else:
        results_dir = args.results_dir

    lib = RecordingLibrary(results_dir)
    metas = lib.scan()
    print(f"[Replay] 目录 {os.path.abspath(results_dir)} 下找到 {len(metas)} 条录制：")
    for i, m in enumerate(metas):
        if m.get('error'):
            print(f"  [{i}] {m['name']}  读取失败：{m['error']}")
            continue
        dur = (m.get("t1") - m.get("t0")) if (m.get("t1") is not None and m.get("t0") is not None) else None
        goal = m.get("goal")
        goal_s = ",".join(f"{g:.0f}" for g in goal) if goal else "?"
        dur_s = f"{dur:.1f}s" if dur is not None else "?"
        print(f"  [{i}] {m['name']}  目标[{goal_s}]  时长{dur_s}  "
              f"状态{m['n_states']}帧 点云{m['n_points']}帧 重规划{m['n_plans']}次")
    if not metas:
        print("[Replay] 没有录制。先跑一次带 --record 的任务：")
        print("        python scene_test.py --goal-x 10 --goal-y 0 --record")

    srv = VizServer(library=lib, port=args.port)
    srv.start()
    print(f"[Replay] 服务地址 {srv.url}（页面里下拉选择录制；Ctrl+C 退出）")
    try:
        if args.serve:
            import time
            while True: time.sleep(.5)
        else: input()
    except (EOFError, KeyboardInterrupt):
        pass
    srv.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
