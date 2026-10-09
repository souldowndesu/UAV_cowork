"""Offline integrity check of the independent 24-flight example collection."""
from pathlib import Path
import json
import sys

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navigation.paths import EXAMPLES
from experiments.experiment import file_hash as sha256
from experiments.recorder import load_recording_dict
ROOT = EXAMPLES


def verify(root=ROOT):
    root = Path(root)
    catalog = json.loads((root/'catalog.json').read_text(encoding='utf-8'))
    checks = []
    for entry in catalog['examples']:
        folder = root/entry['directory']
        mismatches = [name for name, expected in entry['sha256'].items()
                      if not (folder/name).is_file() or sha256(folder/name) != expected]
        data = load_recording_dict(root/entry['recording'])
        prior_ok = entry['flight'] == 1 or sha256(folder/'initial_map.npz') == entry['initial_map_sha256']
        check = dict(id=entry['id'], preserved_files=len(entry['sha256']), mismatches=mismatches,
                     complete=data['complete'], states=len(data['states']), maps=len(data['maps']),
                     plans=len(data['plans']), initial_map_valid=prior_ok,
                     passed=not mismatches and data['complete'] and not data.get('error') and prior_ok
                            and bool(data['states'] and data['maps'] and data['plans']))
        checks.append(check)
        print(f"{entry['id']}: {'PASS' if check['passed'] else 'FAIL'}", flush=True)
    keys = {(e['speed'],e['case'],e['flight']) for e in catalog['examples']}
    return dict(passed=len(checks)==24 and len(keys)==24 and all(c['passed'] for c in checks), examples=checks)


if __name__ == '__main__':
    report = verify()
    output = ROOT/'verification'/'data_integrity.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    sys.exit(0 if report['passed'] else 1)
