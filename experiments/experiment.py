"""Reproducible scene inputs and per-run map isolation for scene_test.py."""
from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
from navigation.paths import PROJECT_ROOT, ENVIRONMENT, project_path

ROOT = PROJECT_ROOT


def file_hash(path):
    path = Path(path)
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def load_scenario(path):
    value = json.loads(project_path(path).read_text(encoding='utf-8'))
    for key in ('start', 'goal'):
        a = np.asarray(value[key], dtype=float)
        if a.shape != (3,) or not np.isfinite(a).all():
            raise ValueError(f'scenario {key} must contain three finite NED coordinates')
    if not isinstance(value.get('config', {}), dict):
        raise ValueError('scenario config must be an object')
    if value.get('map_policy', 'snapshot') not in ('cold', 'snapshot', 'shared'):
        raise ValueError('unknown scenario map_policy')
    return value


class MapBaseline:
    """Snapshot once per batch; never let a prior run train the next comparison."""
    def __init__(self, cfg, policy, batch_dir):
        self.enabled = bool(cfg.use_persistent_map)
        self.policy = policy
        self.source = Path(cfg.persistent_map_path)
        if not self.source.is_absolute():
            self.source = ROOT / self.source
        self.source = self.source.resolve()
        self.snapshot = Path(batch_dir) / 'initial_map.npz'
        self.source_hash = file_hash(self.source) if self.enabled and policy != 'cold' else None
        if self.enabled and policy == 'snapshot' and self.source.exists():
            self.snapshot.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(self.source, self.snapshot)
            metadata=Path(str(self.source)+'.meta.json')
            if metadata.exists():shutil.copy2(metadata,Path(str(self.snapshot)+'.meta.json'))
            if file_hash(self.snapshot) != self.source_hash:
                raise RuntimeError('Prior map changed while the initial snapshot was copied')

    def prepare(self, cfg, outdir):
        if not self.enabled:
            return dict(policy=self.policy, enabled=False, initial_sha256=None)
        if self.policy == 'shared':
            return dict(policy=self.policy, enabled=True, path=str(self.source), initial_sha256=file_hash(self.source))
        target = Path(outdir) / 'run_map.npz'
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists():
            raise FileExistsError('Refusing to reuse an existing experimental map')
        if self.policy == 'snapshot' and self.snapshot.exists():
            shutil.copy2(self.snapshot, target)
            metadata=Path(str(self.snapshot)+'.meta.json')
            if metadata.exists():shutil.copy2(metadata,Path(str(target)+'.meta.json'))
        cfg.persistent_map_path = str(target.resolve())
        return dict(policy=self.policy, enabled=True, path=cfg.persistent_map_path,
                    initial_sha256=file_hash(target), source_path=str(self.source),
                    source_sha256=self.source_hash)


def run_metadata(cfg, start, goal, yaw, prior, scenario=None):
    from navigation.backend import BUILD_HASH, native
    files = [ROOT / 'scene_test.py', ROOT / 'replay.py']
    files += sorted((ROOT/'navigation').glob('*.py')) + sorted((ROOT/'experiments').glob('*.py'))
    files += sorted((ROOT/'navigation'/'cpp').glob('*.cpp'))
    return dict(schema_version=1, scenario=scenario, start=list(start), goal=list(goal),
                start_yaw_deg=float(yaw), config=cfg.to_dict(), initial_map=prior,
                native=native is not None, kernel_hash=BUILD_HASH,
                source_hashes={str(p.relative_to(ROOT)): file_hash(p) for p in files},
                settings_sha256=file_hash(ENVIRONMENT / 'settings.json'),
                documents_settings_sha256=file_hash(Path.home() / 'Documents' / 'AirSim' / 'settings.json'),
                outcome='running',
                initial_state_note='Teleport and settling do not reset the simulator. Actual preflight state is recorded separately.')


def write_metadata(path, value):
    from .recorder import atomic_json
    atomic_json(path, value)
