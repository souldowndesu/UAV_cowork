"""Reproducible scene inputs with a shared observation store."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
from navigation.paths import PROJECT_ROOT, ENVIRONMENT, SHARED_MAP, project_path

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
        self.source = Path(cfg.persistent_seed_path or cfg.persistent_map_path)
        if not self.source.is_absolute():
            self.source = ROOT / self.source
        self.source = self.source.resolve()
        self.snapshot = self.source
        self.source_hash = file_hash(self.source) if self.enabled and policy != 'cold' else None
        self.seed=None
        if self.enabled and policy=='snapshot':
            from navigation.persistent_map import PersistentMap
            self.seed=PersistentMap.load(self.source,cfg.persistent_res,cfg.persistent_block,cfg.persistent_origin,strict=True)

    def prepare(self, cfg, outdir):
        if not self.enabled:
            return dict(policy=self.policy, enabled=False, initial_sha256=None)
        cfg.persistent_map_path = str(SHARED_MAP)
        cfg.persistent_accumulate=True
        cfg.load_persistent_map = self.policy == 'shared'
        cfg.persistent_seed_path = str(self.source) if self.policy == 'snapshot' else ''
        cfg._persistent_seed=self.seed
        if self.policy=='shared':
            self.source=SHARED_MAP.resolve();self.source_hash=file_hash(self.source)
        if self.policy == 'snapshot' and self.source!=SHARED_MAP.resolve() and self.source_hash!=file_hash(self.source):
            raise RuntimeError('Initial snapshot changed since the batch started')
        return dict(policy=self.policy, enabled=True, path=cfg.persistent_map_path,
                    initial_sha256=self.source_hash, source_path=str(self.source),
                    source_sha256=self.source_hash,accumulates_observations=True,
                    loads_history=self.policy!='cold',loads_shared_map=cfg.load_persistent_map,seed_path=cfg.persistent_seed_path)


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
