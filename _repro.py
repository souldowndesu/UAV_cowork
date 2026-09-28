# -*- coding: utf-8 -*-
import os, sys, json, tempfile, traceback
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from config import Config
from nav.occupancy import OccupancyMap, OCCUPIED
from nav import astar, distance_field, bspline
from nav.recorder import Recorder

try:
    cfg = Config()
    m = OccupancyMap(cfg.nx, cfg.ny, cfg.nz, cfg.res, (-20, -20, -13))  # z 中心 -8
    # 小障碍在 (5,0,-8)，A* 可绕行
    for dx in range(-2, 3):
        for dy in range(-2, 3):
            for dz in range(-2, 3):
                m.set_voxel(125 + dx, 100 + dy, 25 + dz, OCCUPIED)
    dist = distance_field.compute(m, cfg)
    planner = astar.AStarPlanner(cfg)
    start = np.array([0.0, 0.0, -8.0])
    goal = np.array([15.0, 0.0, -8.0])
    path = planner.plan(m, dist, start, goal)
    print('A* path len:', len(path) if path else None, 'first:', path[0] if path else None)
    assert path is not None and len(path) >= 2, 'A* 应找到路径'

    traj = bspline.build_polyline(path, start, cfg.v_mission)
    print('polyline ctrl shape:', np.asarray(traj.ctrl).shape)
    tt = np.linspace(0.0, traj.total_time, 40)
    traj_pts = np.array([traj.eval(t)[0] for t in tt])

    r = Recorder(map_size=cfg.map_size)
    r.record_plan(1.0, goal, [np.array(p) for p in path], traj_pts, traj.ctrl)
    r.record_state(1.0, start, np.zeros(3), np.zeros(3), 0.0)

    d = r.to_dict()
    print('to_dict OK, ctrl_len=', len(d['plans'][0]['ctrl']))
    tmpdir = tempfile.mkdtemp()
    p = r.save(os.path.join(tmpdir, 'recording.json'))
    print('save OK:', os.path.getsize(p), 'bytes')
    json.load(open(p, encoding='utf-8'))
    print('RELOAD OK')
except Exception:
    traceback.print_exc()
