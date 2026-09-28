# -*- coding: utf-8 -*-
import os, sys, json, tempfile, threading, traceback, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from config import Config
from nav.recorder import Recorder
from nav import bspline

try:
    cfg = Config()
    r = Recorder(map_size=cfg.map_size)
    r.set_goal(np.array([900.0, 20.0, -20.0]))

    # 构造一条真实感 A* 折线（多点）
    path = [np.array([x, 0.0, -8.0]) for x in np.arange(0.5, 40.0, 0.5)]
    p_s = np.array([0.0, 0.0, -8.0])
    traj = bspline.build_polyline(path, p_s, cfg.v_mission)

    # 并发录制（模拟 4 线程）
    stop = threading.Event()
    errs = []

    def record_states():
        i = 0
        while not stop.is_set():
            r.record_state(time.time(), np.array([i * 0.01, 0.0, -8.0]),
                           np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 9.8]), 0.0)
            i += 1
            time.sleep(0.001)

    def record_points():
        while not stop.is_set():
            r.record_points(time.time(), np.random.rand(1500, 3).astype("float32"))
            time.sleep(0.05)

    def record_plans():
        while not stop.is_set():
            tt = np.linspace(0.0, traj.total_time, 40)
            traj_pts = np.array([traj.eval(t)[0] for t in tt])
            r.record_plan(time.time(), np.array([40.0, 0.0, -8.0]),
                          [np.array(p) for p in path], traj_pts, traj.ctrl)
            time.sleep(0.2)

    def record_maps():
        while not stop.is_set():
            r.record_map(time.time(), np.random.rand(200, 3).astype("float32"))
            time.sleep(0.1)

    ts = [threading.Thread(target=f, daemon=True)
          for f in (record_states, record_points, record_plans, record_maps)]
    for t in ts:
        t.start()
    time.sleep(2.0)
    stop.set()
    for t in ts:
        t.join(timeout=1.0)

    print('recorded: states=%d points=%d plans=%d maps=%d' % (
        len(r.states), len(r.points), len(r.plans), len(r.maps)))

    # 保存（模拟 Ctrl+C 时的 save）
    tmpdir = tempfile.mkdtemp()
    p = r.save(os.path.join(tmpdir, 'recording.json'))
    sz = os.path.getsize(p)
    print('save OK size=%d' % sz)

    # 重新加载校验
    d = json.load(open(p, encoding='utf-8'))
    print('reload: states=%d points=%d plans=%d maps=%d' % (
        len(d['states']), len(d['points']), len(d['plans']), len(d['maps'])))
    if d['plans']:
        print('plan ctrl len=%d traj len=%d' % (len(d['plans'][0]['ctrl']), len(d['plans'][0]['traj'])))
    print('FULL RECORD VERIFY OK')
except Exception:
    traceback.print_exc()
