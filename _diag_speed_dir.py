import json, io, numpy as np

rec = r"results\20260926_172157\recording.json"
d = json.load(io.open(rec, encoding="utf-8"))
states = d["states"]
plans = d["plans"]

t0 = states[0]["t"]
st = np.array([[s["t"], s["p"][0], s["p"][1], s["p"][2],
                s["v"][0], s["v"][1], s["v"][2]] for s in states], dtype="float64")
st[:, 0] -= t0

print("=== 每次重规划：速度方向 vs 轨迹起始方向（path[0]->path[1]） ===")
for pl in plans:
    pt = float(pl["t"]) - t0
    path = pl.get("path") or []
    i = int(np.searchsorted(st[:, 0], pt))
    i = min(max(i, 0), len(st) - 1)
    pos = st[i, 1:4]
    vel = st[i, 4:7]
    spd = float(np.linalg.norm(vel))
    ang_path = None
    if len(path) >= 2 and spd > 0.5:
        p0 = np.asarray(path[0], dtype="float64")
        p1 = np.asarray(path[1], dtype="float64")
        pd = p1 - p0
        pdn = float(np.linalg.norm(pd))
        if pdn > 1e-6:
            ang_path = float(np.degrees(np.arccos(
                np.clip(float(np.dot(pd, vel)) / (pdn * spd), -1.0, 1.0))))
    ang_str = "  -  " if ang_path is None else f"{ang_path:5.1f}deg"
    print(f"t={pt:6.1f}s pos=({pos[0]:6.1f},{pos[1]:6.1f},{pos[2]:6.1f}) "
          f"|v|={spd:4.2f} vel=({vel[0]:5.2f},{vel[1]:5.2f},{vel[2]:5.2f}) "
          f"ang_path={ang_str}")
