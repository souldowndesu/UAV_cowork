# -*- coding: utf-8 -*-
"""录制器：把一次任务完整记录下来（载具状态 / LiDAR 点云 / 每次重规划），
供浏览器三维四视图实时观看，或事后拖动进度条回放。线程安全。

设计原则（保证录制/保存永不因其它模块改动而失效，用户以回放作为评判标准）：
- 所有 ``record_*`` 都是防御式：任意形状/类型/None/NaN 的输入都被安全地规整成
  ``(3,)`` 或 ``(N,3)``，绝不抛异常 → 录制线程不会因为数据形状变化而死亡。
- 序列化用向量化清洗 NaN/Inf，产出合法 JSON（不会出现 NaN/Infinity 非法字面量）。
- ``save()`` 永不失败：完整序列化失败 → 回退最小录制 → 回退仅 goal；原子写入失败
  → 回退直接写。每一步失败都打印原因，且保存成功后打印帧数摘要供用户自检。
"""
from __future__ import annotations

import json
import os
import threading

import numpy as np


# ---------------------------------------------------------------------------
# 安全规整辅助（永不抛异常）
# ---------------------------------------------------------------------------
def _vec3(a, dtype="float64"):
    """任意输入 → (3,) float 数组；失败/缺失 → zeros(3)。"""
    try:
        arr = np.asarray(a, dtype=dtype).reshape(-1)
    except Exception:
        return np.zeros(3, dtype=dtype)
    out = np.zeros(3, dtype=dtype)
    n = min(3, arr.shape[0])
    out[:n] = arr[:n]
    return out


def _mat_n3(a, dtype="float64"):
    """任意输入 → (N,3) float 数组；失败/空/非 3 倍数 → (0,3)。"""
    try:
        arr = np.asarray(a, dtype=dtype)
        flat = arr.reshape(-1)
    except Exception:
        return np.zeros((0, 3), dtype=dtype)
    n = flat.shape[0] // 3
    if n == 0:
        return np.zeros((0, 3), dtype=dtype)
    return flat[: n * 3].reshape(n, 3)


def _clean_round(a, nd, dtype="float64"):
    """向量化清洗 NaN/Inf → 0.0 并 round 到 nd 位，返回 1-D Python list。"""
    try:
        arr = np.asarray(a, dtype=dtype).reshape(-1)
    except Exception:
        return []
    arr = np.where(np.isfinite(arr), arr, 0.0)
    return np.round(arr, nd).tolist()


def _l(v, nd=3):
    """(3,) 向量 → [x,y,z]。"""
    return _clean_round(v, nd)


def _l2(a, nd=2):
    """(N,3) 数组 → [[x,y,z],...]。"""
    arr = _mat_n3(a)
    arr = np.where(np.isfinite(arr), arr, 0.0)
    return np.round(arr, nd).tolist()


def _flat(a, nd=2):
    """(N,3) 点云 → 一维 [x1,y1,z1,x2,...]，压缩体积。"""
    return _clean_round(a, nd)


class Recorder:
    def __init__(self, max_points: int = 1500, map_size=None):
        self.max_points = int(max_points)
        # 规划范围 = 局部体素图尺寸 (x,y,z) 米；浏览器据此画感知框（超出即未知、不用于规划）
        self.map_size = (tuple(float(v) for v in map_size)
                         if map_size is not None else None)
        self.goal = None                 # (3,) 世界系
        self.states = []                 # {"t","p","v","a","yaw"}
        self.points = []                 # {"t","pts"(N,3) float32}
        self.plans = []                  # {"t","goal","path","traj","ctrl"}
        self.maps = []                   # {"t","occ"(N,3)} 下采样碰撞图（仅 occupied）
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # 录制（全部防御式，永不抛异常）
    # ------------------------------------------------------------------
    def set_goal(self, goal):
        with self._lock:
            self.goal = _vec3(goal)

    def record_state(self, t, pos, vel, acc, yaw=0.0):
        try:
            tt = float(t) if t is not None else 0.0
            y = float(yaw) if yaw is not None else 0.0
            if not np.isfinite(y):
                y = 0.0
        except Exception:
            return
        with self._lock:
            self.states.append({
                "t": tt,
                "p": _vec3(pos),
                "v": _vec3(vel),
                "a": _vec3(acc),
                "yaw": y,
            })

    def record_points(self, t, pts):
        arr = _mat_n3(pts, dtype="float32")
        if arr.shape[0] == 0:
            return
        try:
            n = arr.shape[0]
            if n > self.max_points:
                step = max(1, n // self.max_points)
                arr = arr[::step][:self.max_points]
            tt = float(t) if t is not None else 0.0
        except Exception:
            return
        with self._lock:
            self.points.append({"t": tt, "pts": arr})

    def record_map(self, t, occ):
        arr = _mat_n3(occ, dtype="float32")
        if arr.shape[0] == 0:
            return
        try:
            tt = float(t) if t is not None else 0.0
        except Exception:
            return
        with self._lock:
            self.maps.append({"t": tt, "occ": arr})

    def record_plan(self, t, goal, path, traj, ctrl):
        try:
            tt = float(t) if t is not None else 0.0
        except Exception:
            tt = 0.0
        with self._lock:
            self.plans.append({
                "t": tt,
                "goal": _vec3(goal),
                "path": _mat_n3(path),
                "traj": _mat_n3(traj),
                "ctrl": _mat_n3(ctrl),
            })

    # ------------------------------------------------------------------
    # 最新快照（实时观看用）
    # ------------------------------------------------------------------
    def latest_state(self):
        with self._lock:
            return self.states[-1] if self.states else None

    def latest_points(self):
        with self._lock:
            return self.points[-1] if self.points else None

    def latest_plan(self):
        with self._lock:
            return self.plans[-1] if self.plans else None

    # ------------------------------------------------------------------
    # 序列化（给浏览器 JSON）
    # ------------------------------------------------------------------
    def to_dict(self):
        with self._lock:
            return {
                "goal": _l(self.goal) if self.goal is not None else None,
                "states": [
                    {"t": s["t"], "p": _l(s["p"]), "v": _l(s["v"]),
                     "a": _l(s["a"]), "yaw": s["yaw"]}
                    for s in self.states
                ],
                "points": [
                    {"t": p["t"], "pts": _flat(p["pts"])} for p in self.points
                ],
                "plans": [
                    {"t": p["t"], "goal": _l(p["goal"]),
                     "path": _l2(p["path"]), "traj": _l2(p["traj"]),
                     "ctrl": _l2(p["ctrl"])}
                    for p in self.plans
                ],
                "maps": [
                    {"t": m["t"], "occ": _flat(m["occ"])}
                    for m in self.maps
                ],
                "map_size": (list(self.map_size) if self.map_size is not None else None),
            }

    def meta(self):
        """轻量元信息（目标/时长/帧数），供回放列表展示，避免每次都加载整段录制。"""
        with self._lock:
            st = self.states
            return {
                "goal": _l(self.goal) if self.goal is not None else None,
                "t0": st[0]["t"] if st else None,
                "t1": st[-1]["t"] if st else None,
                "n_states": len(st),
                "n_points": len(self.points),
                "n_plans": len(self.plans),
                "n_maps": len(self.maps),
            }

    def summary(self):
        """帧数摘要（保存后打印，供用户确认录制完整）。"""
        with self._lock:
            return {
                "n_states": len(self.states),
                "n_points": len(self.points),
                "n_plans": len(self.plans),
                "n_maps": len(self.maps),
            }

    def _minimal_dict(self):
        """最小录制（仅 states + goal），用于完整序列化失败时的回退，保证总能有文件。"""
        with self._lock:
            return {
                "goal": _l(self.goal) if self.goal is not None else None,
                "states": [
                    {"t": s["t"], "p": _l(s["p"]), "v": _l(s["v"]),
                     "a": _l(s["a"]), "yaw": s["yaw"]}
                    for s in self.states
                ],
                "points": [], "plans": [], "maps": [],
                "map_size": (list(self.map_size) if self.map_size is not None else None),
            }

    def save(self, path):
        # 永不失败：完整序列化 → 最小录制 → 仅 goal；原子写入 → 直接写。
        # 每一步失败都打印原因，便于定位；保存成功后打印帧数摘要。
        try:
            data = self.to_dict()
        except Exception as e:
            print(f"[Recorder] 完整序列化失败（{e}），回退到最小录制（仅 states）")
            try:
                data = self._minimal_dict()
            except Exception as e2:
                print(f"[Recorder] 最小录制也失败（{e2}），仅保存 goal")
                data = {"goal": _l(self.goal) if self.goal is not None else None,
                        "states": [], "points": [], "plans": [], "maps": [],
                        "map_size": (list(self.map_size) if self.map_size is not None else None)}
        tmp = path + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False)
            os.replace(tmp, path)
        except Exception as e:
            print(f"[Recorder] 原子写入失败（{e}），尝试直接写入 {path}")
            try:
                with open(path, "w", encoding="utf-8") as f:
                    json.dump(data, f, ensure_ascii=False)
            except Exception as e2:
                print(f"[Recorder] 直接写入也失败（{e2}）")
                return None
        # 侧车 meta.json：回放列表无需加载整段录制即可展示信息（失败不影响主录制）
        mp = os.path.join(os.path.dirname(path), "meta.json")
        try:
            m = self.meta()
            m["name"] = os.path.basename(os.path.dirname(path))
            m["recording"] = os.path.basename(path)
            with open(mp, "w", encoding="utf-8") as f:
                json.dump(m, f, ensure_ascii=False)
        except Exception as e:
            print(f"[Recorder] meta.json 写入失败（{e}），主录制已保存")
        try:
            s = self.summary()
            print(f"[Recorder] 已保存 {path}：states={s['n_states']} points={s['n_points']} "
                  f"plans={s['n_plans']} maps={s['n_maps']}")
        except Exception:
            pass
        return path

    def snapshot_live(self):
        """实时帧（浏览器 /api/live 用），返回 dict 或 None。"""
        st = self.latest_state()
        if st is None:
            return None
        pt = self.latest_points()
        pl = self.latest_plan()
        return {
            "goal": _l(self.goal) if self.goal is not None else None,
            "state": {"t": st["t"], "p": _l(st["p"]), "v": _l(st["v"]),
                      "a": _l(st["a"]), "yaw": st["yaw"]},
            "pts": _flat(pt["pts"]) if pt is not None else [],
            "plan": ({"t": pl["t"], "goal": _l(pl["goal"]),
                      "path": _l2(pl["path"]), "traj": _l2(pl["traj"]),
                      "ctrl": _l2(pl["ctrl"])}
                     if pl is not None else None),
            "map_size": (list(self.map_size) if self.map_size is not None else None),
        }


def load_recording_dict(path):
    """加载已保存的录制文件，返回纯 dict（供回放服务器直接服务）。"""
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# 录制库：自动扫描 / 元信息 / 按 id 加载
# ---------------------------------------------------------------------------
def scan_recordings(root_dir):
    """递归扫描目录树里所有 recording.json，按修改时间降序返回路径列表。

    兼容两层目录结构：
    - 旧格式 ``results/<时间戳>/recording.json``
    - 新批量格式 ``results/<时间戳>/run_NN/recording.json``
    """
    out = []
    root = os.path.abspath(root_dir)
    if not os.path.isdir(root):
        return out
    for dirpath, _dirnames, filenames in os.walk(root):
        if "recording.json" in filenames:
            rp = os.path.join(dirpath, "recording.json")
            try:
                out.append((rp, os.path.getmtime(rp)))
            except OSError:
                continue
    out.sort(key=lambda x: -x[1])
    return [p for p, _ in out]


def read_meta(recording_path, root_dir=None):
    """返回一条录制的元信息。优先读 meta.json 侧车；否则加载录制文件提取。

    ``root_dir`` 非空时，``name`` 用相对该目录的路径（如 ``20260926_161726/run_00``），
    避免批量测试里多个 ``run_00`` 重名；不传则退回目录名。
    """
    def _relname(p):
        if root_dir:
            try:
                rel = os.path.relpath(os.path.dirname(os.path.abspath(p)),
                                      os.path.abspath(root_dir))
                if rel not in (".", os.curdir):
                    return rel
            except ValueError:
                pass
        return os.path.basename(os.path.dirname(p))

    d = os.path.dirname(recording_path)
    mp = os.path.join(d, "meta.json")
    if os.path.isfile(mp):
        try:
            with open(mp, "r", encoding="utf-8") as f:
                meta = json.load(f)
            meta["path"] = recording_path
            meta["bytes"] = os.path.getsize(recording_path)
            meta["name"] = _relname(recording_path)
            return meta
        except Exception:
            pass
    data = load_recording_dict(recording_path)
    st = data.get("states", [])
    return {
        "name": _relname(recording_path),
        "path": recording_path,
        "bytes": os.path.getsize(recording_path),
        "goal": data.get("goal"),
        "t0": st[0]["t"] if st else None,
        "t1": st[-1]["t"] if st else None,
        "n_states": len(st),
        "n_points": len(data.get("points", [])),
        "n_plans": len(data.get("plans", [])),
    }


class RecordingLibrary:
    """一组录制：扫描 / 列表 / 按序号加载（带缓存）。"""

    def __init__(self, root_dir):
        self.root_dir = root_dir
        self._paths = []
        self._cache = {}
        self._lock = threading.Lock()

    def scan(self):
        self._paths = scan_recordings(self.root_dir)
        return self.list()

    def list(self):
        return [read_meta(p, self.root_dir) for p in self._paths]

    def count(self):
        return len(self._paths)

    def get(self, index):
        if not (0 <= index < len(self._paths)):
            return None
        p = self._paths[index]
        with self._lock:
            if p not in self._cache:
                self._cache[p] = load_recording_dict(p)
            return self._cache[p]
