# -*- coding: utf-8 -*-
"""录制器健壮性测试：保证「保存工具不随其它代码改动而失效」。

覆盖三类：
1. 正常 roundtrip：record → save → reload，字段完整、结构正确。
2. 防御式输入：None / 参差列表 / 错误形状 / NaN/Inf，record_* 一律不抛异常。
3. save 永不失败：即使内部数据被破坏，save() 也返回路径并产出合法 JSON（无 NaN 字面量）。
"""
import json
import os
import sys
import tempfile
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from nav.recorder import Recorder


def test_roundtrip_basic():
    rec = Recorder(map_size=(40, 40, 10))
    rec.set_goal(np.array([900.0, 20.0, -20.0]))
    for i in range(50):
        rec.record_state(0.02 * i,
                         np.array([i * 0.1, 0.0, -10.0]),
                         np.array([1.0, 0.0, 0.0]),
                         np.zeros(3), 0.0)
    for i in range(10):
        rec.record_points(0.1 * i, np.random.rand(100, 3).astype("float32"))
        rec.record_map(0.1 * i, np.random.rand(20, 3).astype("float32"))
        path = [np.array([j * 0.5, 0.0, -10.0]) for j in range(6)]
        traj = np.random.rand(40, 3)
        rec.record_plan(0.1 * i, np.array([3.0, 0.0, -10.0]),
                        path, traj, np.array(path))
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "recording.json")
        assert rec.save(p) == p
        data = json.load(open(p, encoding="utf-8"))
        assert data["goal"] == [900.0, 20.0, -20.0]
        assert len(data["states"]) == 50
        assert len(data["points"]) == 10
        assert len(data["plans"]) == 10
        assert len(data["maps"]) == 10
        # 结构：plans 里 path/traj/ctrl 都是 [[x,y,z],...]
        assert isinstance(data["plans"][0]["path"][0], list)
        assert len(data["plans"][0]["path"][0]) == 3
        assert len(data["plans"][0]["ctrl"]) == 6
        assert data["map_size"] == [40.0, 40.0, 10.0]


def test_record_never_raises_adversarial():
    rec = Recorder(map_size=(40, 40, 10))
    # 各种坏输入：None、标量、参差列表、错误形状、字符串、空
    rec.set_goal(None)
    rec.record_state(None, None, None, None, None)
    rec.record_state("t", [1, 2], [1, 2, 3, 4], 5, float("nan"))
    rec.record_points(0.0, None)
    rec.record_points(0.0, [[1, 2, 3], [4, 5]])          # 参差 → 跳过
    rec.record_points(0.0, np.zeros((0, 3), dtype="float32"))
    rec.record_points(0.0, [[1, 2], [3, 4], [5, 6]])      # (3,2) → 展平成 (2,3)
    rec.record_map(0.0, None)
    rec.record_map(0.0, np.array([1.0, 2.0]))             # 非 3 倍数 → 跳过
    rec.record_map(0.0, np.random.rand(9))                # (9,) → (3,3)
    rec.record_plan(None, None, None, None, None)
    rec.record_plan(0.0, [1, 2], "bad", np.zeros((2, 2)), [[1, 2, 3], [4, 5]])
    # 没抛异常即通过
    assert True


def test_nan_cleaned_to_valid_json():
    rec = Recorder()
    rec.record_state(0.0,
                     np.array([np.nan, np.inf, -np.inf]),
                     np.array([0.0, 0.0, 0.0]),
                     np.array([0.0, 0.0, 0.0]))
    rec.record_points(0.0, np.array([[np.nan, 1.0, 2.0], [3.0, np.inf, 5.0]]))
    rec.record_plan(0.0, np.array([np.nan, 0.0, 0.0]),
                    [np.array([np.nan, 0.0, 0.0])],
                    np.array([[0.0, 0.0, np.inf]]),
                    np.array([[1.0, 2.0, 3.0]]))
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "recording.json")
        rec.save(p)
        raw = open(p, encoding="utf-8").read()
        assert "NaN" not in raw and "Infinity" not in raw
        data = json.load(open(p, encoding="utf-8"))
        assert data["states"][0]["p"] == [0.0, 0.0, 0.0]
        assert data["points"][0]["pts"][0] == 0.0


def test_save_never_fails_on_corrupted_data():
    rec = Recorder()
    rec.record_state(0.0, [0, 0, 0], [1, 1, 1], [0, 0, 0])
    # 人为破坏内部数据：塞一个坏 state（p 是 None），完整序列化也应被回退兜住
    with rec._lock:
        rec.states.append({"t": 0.1, "p": None, "v": None, "a": None, "yaw": 0.0})
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "recording.json")
        out = rec.save(p)
        assert out is not None
        data = json.load(open(p, encoding="utf-8"))
        assert "states" in data


def test_concurrent_recording_no_corruption():
    rec = Recorder()
    stop = threading.Event()

    def writer(tag):
        i = 0
        while not stop.is_set():
            rec.record_state(i * 0.01, [i, tag, 0.0], [1, 0, 0], [0, 0, 0])
            rec.record_points(i * 0.01, np.random.rand(50, 3).astype("float32"))
            i += 1

    threads = [threading.Thread(target=writer, args=(k,)) for k in range(4)]
    for t in threads:
        t.start()
    import time
    time.sleep(0.2)
    stop.set()
    for t in threads:
        t.join()
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "recording.json")
        rec.save(p)
        data = json.load(open(p, encoding="utf-8"))
        assert len(data["states"]) > 0


if __name__ == "__main__":
    test_roundtrip_basic()
    test_record_never_raises_adversarial()
    test_nan_cleaned_to_valid_json()
    test_save_never_fails_on_corrupted_data()
    test_concurrent_recording_no_corruption()
    print("test_recorder: OK")
