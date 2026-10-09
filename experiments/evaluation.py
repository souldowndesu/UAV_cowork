"""Offline navigation diagnostics. Sparse replay maps cannot certify collision freedom."""
from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def _vectors(value):
    return np.asarray(value, dtype=float).reshape(-1, 3)


def _stats(value):
    a = np.asarray(value, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return None
    return dict(min=float(a.min()), median=float(np.median(a)),
                p90=float(np.quantile(a, .9)), p99=float(np.quantile(a, .99)),
                max=float(a.max()))


def _length(points):
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def distance_to_polyline(points, path):
    """Exact distance from each query point to the recorded line segments."""
    points, path = _vectors(points), _vectors(path)
    if not len(path):
        return np.full(len(points), np.nan)
    if len(path) == 1:
        return np.linalg.norm(points - path[0], axis=1)
    delta = np.diff(path, axis=0)
    denom = np.sum(delta * delta, axis=1)
    offset = points[:, None, :] - path[:-1]
    f = np.clip(np.sum(offset * delta, axis=2) / np.maximum(denom, 1e-12), 0, 1)
    return np.linalg.norm(offset - f[:, :, None] * delta, axis=2).min(axis=1)


def _prefix_direction(points, metres=10.):
    points = _vectors(points)
    if len(points) < 2:
        return None
    xy = points[:, :2]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    end = np.array([np.interp(min(metres, arc[-1]), arc, xy[:, a]) for a in range(2)])
    delta = end - xy[0]
    norm = np.linalg.norm(delta)
    return delta / norm if norm > .1 else None


def evaluate(data, telemetry=None, run_meta=None):
    """Keep unavailable outcomes/safety evidence unknown, including legacy JSON."""
    run_meta = run_meta or {}
    limits = [
        "Replay occupied samples are sparse: proximity is diagnostic, not a collision test.",
        "Recorded spline samples do not certify continuous curve or swept vehicle clearance.",
        "Wall timestamps do not establish simulation clock rate or physical tracking error.",
    ]
    states = sorted(data.get('states', []), key=lambda r: r['t'])
    if not states and telemetry:
        states = [dict(t=r['t'], p=[r[k] for k in ('x', 'y', 'z')],
                       v=[r[k] for k in ('vx', 'vy', 'vz')],
                       a=[r.get(k, 0.) for k in ('ax', 'ay', 'az')]) for r in telemetry]
    for r in states:
        if not np.isfinite(float(r['t'])) or not np.isfinite(_vectors(r['p'])).all() \
                or not np.isfinite(_vectors(r['v'])).all():
            raise ValueError('Non-finite state timestamp, position or velocity')
    plans = sorted(data.get('plans', []), key=lambda r: r['t'])
    maps = sorted(data.get('maps', []), key=lambda r: r['t'])
    cfg = run_meta.get('config') or data.get('config') or {}
    outcome = run_meta.get('outcome')
    if outcome is None and data.get('outcomes'):
        outcome = data['outcomes'][-1].get('outcome')
    if outcome is None:
        limits.append('No mission outcome in this recording: completion/collision remains unknown.')
    if not cfg:
        limits.append('No historical config: current defaults must not be used as recorded settings.')
    summary = dict(schema_version=1, mission_outcome=outcome or 'unknown',
                   recording_complete=data.get('complete'), state_count=len(states),
                   plan_count=len(plans), map_count=len(maps), limitations=limits)
    t = np.array([r['t'] for r in states])
    p = _vectors([r['p'] for r in states])
    v = _vectors([r['v'] for r in states])
    if len(states):
        summary.update(first_position=p[0].tolist(), last_position=p[-1].tolist(),
                       duration_wall_s=float(t[-1] - t[0]), path_length_m=_length(p),
                       reported_speed_mps=_stats(np.linalg.norm(v, axis=1)))
        goal = data.get('goal')
        if goal is not None:
            summary['goal'] = list(goal)
            summary['final_goal_distance_m'] = float(np.linalg.norm(p[-1] - goal))
            summary['closest_goal_distance_m'] = float(np.linalg.norm(p - goal, axis=1).min())
        if len(states) > 1:
            dt = np.diff(t)
            duration = t[-1] - t[0]
            sp = np.linalg.norm(v, axis=1)
            summary['state_interval_wall_s'] = _stats(dt[dt > 0])
            summary['duplicate_timestamp_count'] = int((dt == 0).sum())
            summary['repeated_position_fraction'] = float((np.linalg.norm(np.diff(p, axis=0), axis=1) == 0).mean())
            if duration > 0:
                summary['effective_state_hz'] = float((len(states) - 1) / duration)
                summary['reported_speed_time_mean_mps'] = float(np.sum(.5 * (sp[1:] + sp[:-1]) * dt) / duration)
                summary['position_speed_wall_mean_mps'] = summary['path_length_m'] / duration
            # Use interpolated start of the final 40 wall seconds.
            begin = max(t[0], t[-1] - 40.)
            start = np.array([np.interp(begin, t, p[:, a]) for a in range(3)])
            tail = np.vstack([start, p[t > begin]])
            summary['tail_window_wall_s'] = float(t[-1] - begin)
            summary['tail_path_length_m'] = _length(tail)
            summary['tail_net_displacement_m'] = float(np.linalg.norm(tail[-1] - tail[0]))
    for name, stream in (('plan', plans), ('map', maps), ('sensor', data.get('points', []))):
        timestamps = sorted(r['t'] for r in stream)
        summary[name + '_interval_wall_s'] = _stats(np.diff(timestamps))
    rows = []
    previous_direction = None
    map_times = np.array([r['t'] for r in maps])
    for i, plan in enumerate(plans):
        path, traj = _vectors(plan.get('path', [])), _vectors(plan.get('traj', []))
        row = dict(index=i, t_wall_s=float(plan['t'] - t[0]) if len(t) else float(plan['t']),
                   astar_length_m=_length(path), sampled_spline_length_m=_length(traj),
                   control_point_count=len(_vectors(plan.get('ctrl', []))))
        dev = distance_to_polyline(traj, path)
        row['spline_to_astar_max_m'] = float(np.nanmax(dev)) if len(dev) and np.isfinite(dev).any() else None
        if len(t) and len(traj):
            current = np.array([np.interp(plan['t'], t, p[:, a]) for a in range(3)])
            row['spline_start_to_vehicle_m'] = float(np.linalg.norm(traj[0] - current))
        direction = _prefix_direction(traj)
        row['prefix_heading_change_deg'] = None
        if direction is not None and previous_direction is not None:
            row['prefix_heading_change_deg'] = float(np.degrees(np.arccos(np.clip(direction @ previous_direction, -1, 1))))
        previous_direction = direction
        mi = np.searchsorted(map_times, plan['t'], side='right') - 1
        if mi >= 0:
            row['latest_recorded_map_age_wall_s'] = float(plan['t'] - map_times[mi])
            occ = _vectors(maps[mi].get('occ', []))
            if len(occ) and len(traj):
                from scipy.spatial import cKDTree
                row['sparse_occupied_sample_distance_m'] = float(cKDTree(occ).query(traj)[0].min())
        rows.append(row)
    summary['prefix_reversals_over_120deg'] = sum((r.get('prefix_heading_change_deg') or 0) > 120 for r in rows)
    summary['spline_to_astar_max_m'] = _stats([r['spline_to_astar_max_m'] for r in rows if r['spline_to_astar_max_m'] is not None])
    summary['spline_start_to_vehicle_m'] = _stats([r['spline_start_to_vehicle_m'] for r in rows if 'spline_start_to_vehicle_m' in r])
    if telemetry:
        if all('collision' in r for r in telemetry):
            summary['telemetry_collision_observed'] = any(r['collision'] for r in telemetry)
        if all('v_cmd' in r and 'v_actual' in r for r in telemetry):
            error = np.array([r['v_cmd'] - r['v_actual'] for r in telemetry])
            summary['diagnostic_command_speed_rmse_mps'] = float(np.sqrt(np.mean(error**2)))
            if cfg.get('navigation_mode')!='safe_route':
                limits.append('v_cmd is the coordinator diagnostic norm; the Z API transmits altitude, not its third velocity component.')
            else:
                limits.append('v_cmd is the transmitted world velocity norm; tracking lag still separates command and actual speed.')
        if all('clearance' in r for r in telemetry):
            summary['telemetry_local_clearance_m'] = _stats([r['clearance'] for r in telemetry])
    return summary, rows


def write_evaluation(data, outdir, telemetry=None, run_meta=None, plot=False):
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    summary, rows = evaluate(data, telemetry, run_meta)
    (outdir / 'evaluation.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with (outdir / 'plan_metrics.csv').open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    lines = ['# Navigation evaluation', '', '| Metric | Value |', '|---|---|']
    lines += [f'| {k} | {json.dumps(v, ensure_ascii=False)} |' for k, v in summary.items() if k != 'limitations']
    lines += ['', 'Evidence limits:', ''] + ['- ' + s for s in summary['limitations']]
    (outdir / 'evaluation.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    if plot and data.get('states'):
        _plot(data, outdir)
    return summary


def evaluate_file(source, outdir, plot=False):
    from .recorder import load_recording_dict
    source = Path(source).resolve()
    if source.is_dir():
        source = source / ('manifest.json' if (source / 'manifest.json').exists() else 'recording.json')
    data = load_recording_dict(source)
    if source.name == 'manifest.json':
        manifest = json.loads(source.read_text(encoding='utf-8'))
        data['config'] = manifest.get('config')
    telemetry_path = source.parent / 'telemetry.json'
    telemetry = json.loads(telemetry_path.read_text(encoding='utf-8')).get('records') if telemetry_path.exists() else None
    meta_path = source.parent / 'run_meta.json'
    meta = json.loads(meta_path.read_text(encoding='utf-8')) if meta_path.exists() else {}
    summary = write_evaluation(data, outdir, telemetry, meta, plot)
    summary.update(source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    (Path(outdir) / 'evaluation.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    return summary


def _plot(data, outdir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    s = data['states']
    t = np.array([r['t'] for r in s]); p = _vectors([r['p'] for r in s]); v = _vectors([r['v'] for r in s])
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    axes[0].plot(p[:, 0], p[:, 1], color='#2563eb')
    axes[0].scatter(*p[0, :2], color='green', label='start')
    axes[0].scatter(*p[-1, :2], color='red', label='last recorded')
    axes[0].set(xlabel='North X (m)', ylabel='East Y (m)', title='Recorded flight'); axes[0].axis('equal'); axes[0].legend()
    axes[1].plot(t - t[0], np.linalg.norm(v, axis=1), label='API speed')
    axes[1].set(xlabel='Wall time (s)', ylabel='Reported speed (m/s)', title='Clock domains need separate logging')
    axes[2].plot(t - t[0], p[:, 1], label='Y position')
    for plan in data.get('plans', []):
        axes[2].axvline(plan['t'] - t[0], color='gray', alpha=.3)
    axes[2].set(xlabel='Wall time (s)', ylabel='East Y (m)', title='Lateral motion and plan updates')
    for ax in axes: ax.grid(alpha=.2)
    fig.tight_layout(); fig.savefig(outdir / 'overview.png', dpi=150); plt.close(fig)
