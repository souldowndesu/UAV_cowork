"""Select a preserved example and rerun its navigation through scene_test."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navigation.paths import PROJECT_ROOT, EXAMPLES, RECORDS, project_path
ROOT = PROJECT_ROOT


def read_catalog(root=EXAMPLES):
    catalog = json.loads((Path(root) / 'catalog.json').read_text(encoding='utf-8'))
    return catalog['examples']


def select_case(entries, choice):
    return next((e for e in entries if str(e['number']) == str(choice) or e['id'] == choice), None)


def prepare_scenario(entry, args, root=EXAMPLES):
    from experiments.experiment import file_hash
    folder = Path(root) / entry['directory']
    scenario = json.loads((Path(root) / entry['scenario']).read_text(encoding='utf-8'))
    cfg = scenario['config']
    cfg['log_dir'] = str(Path(args.output_dir).resolve() / entry['id'])
    cfg['persistent_map_path'] = str((folder / 'initial_map.npz').resolve())
    if entry['flight'] == 2 and file_hash(cfg['persistent_map_path']) != entry['initial_map_sha256']:
        raise ValueError('复飞初始地图缺失或校验失败，拒绝使用不同的先验地图')
    if args.ip is not None: cfg['ip'] = args.ip
    if args.port is not None: cfg['port'] = args.port
    scenario['example_id'] = entry['id']
    return scenario


def run_case(entry, args):
    scenario = prepare_scenario(entry, args)
    folder = EXAMPLES / entry['directory']
    meta = json.loads((folder / 'run_meta.json').read_text(encoding='utf-8'))
    print(f"\n{entry['label']}  起点 {scenario['start']} → 目标 {scenario['goal']}", flush=True)
    print(f"地图策略：{scenario['map_policy']}；重复 {args.repeat} 次，每次恢复相同初始地图。", flush=True)
    print(f"新记录目录：{scenario['config']['log_dir']}", flush=True)
    if args.dry_run:
        print(json.dumps(scenario, ensure_ascii=False, indent=2))
        return 0
    # Keep the exact inputs alongside new results. No system temporary directory
    # is needed, including when launched from VS Code with an unwritable TEMP.
    inputs = Path(scenario['config']['log_dir']) / 'inputs'
    inputs.mkdir(parents=True, exist_ok=True)
    path = inputs / f'{uuid.uuid4().hex}.json'
    path.write_text(json.dumps(scenario, ensure_ascii=False, indent=2), encoding='utf-8')
    command = [sys.executable, '-u', str(ROOT / 'scene_test.py'), '--scenario', str(path),
               '--profile', meta.get('profile','safe5'), '--runs', str(args.repeat),
               '--reset', '--record', '--stuck-timeout', str(meta['stuck_timeout_wall_s']),
               '--autosave-interval', str(meta['autosave_interval_s'])]
    if args.visualize: command += ['--visualize', '--viz-port', str(args.viz_port)]
    return subprocess.run(command, cwd=ROOT, env=dict(os.environ, PYTHONUTF8='1')).returncode


def main(argv=None):
    parser = argparse.ArgumentParser(description='选择 24 个验证案例，在已开启的模拟器中重新飞行并实时保存新记录')
    parser.add_argument('--case', help='案例编号 1–24 或完整案例 ID；未指定时显示选择菜单')
    parser.add_argument('--repeat', type=int, default=1, help='同一案例连续重飞次数（每次恢复原始先验）')
    parser.add_argument('--list', action='store_true', help='只列出案例，不连接模拟器')
    parser.add_argument('--dry-run', action='store_true', help='显示实际使用的场景参数，不连接模拟器')
    parser.add_argument('--output-dir', default=str(RECORDS / 'example_runs'))
    parser.add_argument('--visualize', action='store_true', help='单次飞行开启实时网页；多次飞行请另开 replay')
    parser.add_argument('--viz-port', type=int, default=8767)
    parser.add_argument('--ip', help='覆盖模拟器 RPC 地址')
    parser.add_argument('--port', type=int, help='覆盖模拟器 RPC 端口')
    args = parser.parse_args(argv)
    if args.repeat < 1: parser.error('--repeat 必须至少为 1')
    # Example data must remain an immutable baseline even when output is overridden.
    output = project_path(args.output_dir)
    args.output_dir = str(output)
    if output == EXAMPLES or EXAMPLES in output.parents:
        parser.error('新飞行记录不能写入 examples；请选择独立输出目录')
    entries = read_catalog()
    if args.list or args.case is None:
        print('\n24 次验证飞行：高度使用 NED，目标 z=-20 表示高 20 米。')
        for entry in entries: print(f"{entry['label']}  目标 {entry['goal']}  [{entry['id']}]")
    if args.list: return 0
    if args.case is not None:
        entry = select_case(entries, args.case)
        if entry is None: parser.error('未知案例；用 --list 查看编号和 ID')
        return run_case(entry, args)
    print('\n请先开启 CityEnviron 模拟器；飞行入口会重置载具并恢复记录中的起点。')
    while True:
        try: choice = input('\n选择案例编号 1–24（q 退出）：').strip()
        except (EOFError, KeyboardInterrupt): return 0
        if choice.lower() in ('q','quit','exit'): return 0
        entry = select_case(entries, choice)
        if entry is None:
            print('请输入列表中的编号或案例 ID。'); continue
        try:
            code = run_case(entry, args)
            print(f'本次运行结束，返回码 {code}；可以再次选择同一个案例或其他案例。')
        except (OSError, ValueError) as exc:
            print(f'无法运行：{exc}')


if __name__ == '__main__':
    sys.exit(main())
