"""Build a relocatable distribution without workspace environments or archives."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]
EXCLUDED = {'__pycache__', '.pytest_cache', '.venv', '.git', 'build', 'runtime', 'releases'}


def digest(path):
    result=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):result.update(chunk)
    return result.hexdigest()


def inputs(root):
    files={}
    for name in ('README.md','scene_test.py','replay.py','requirements.txt','setup_env.cmd','fly.cmd','replay_scene.cmd'):
        files[name]=root/name
    for folder in ('navigation','experiments','data','environment','docs'):
        for path in (root/folder).rglob('*'):
            if not path.is_file() or any(p in EXCLUDED for p in path.relative_to(root).parts):continue
            if path.suffix in ('.pyc','.pyo'):continue
            files[path.relative_to(root).as_posix()]=path
    return files


def build(root, output):
    root,output=Path(root).resolve(),Path(output).resolve()
    files=inputs(root)
    if not list((root/'environment'/'wheelhouse').glob('*.whl')):raise ValueError('Offline wheelhouse is empty')
    if not list((root/'navigation').glob('_fast*.pyd')):raise ValueError('Expected Windows / CPython 3.10 native kernel')
    integrity=json.loads((root/'data'/'examples'/'verification'/'data_integrity.json').read_text(encoding='utf-8'))
    if not integrity['passed']:raise ValueError('Examples have not passed integrity verification')
    manifest=dict(schema_version=1,platform='Windows x64',python='CPython 3.10',
                  files={name:dict(bytes=path.stat().st_size,sha256=digest(path)) for name,path in sorted(files.items())})
    output.parent.mkdir(parents=True,exist_ok=True)
    partial=output.with_suffix('.zip.partial')
    with zipfile.ZipFile(partial,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6,allowZip64=True) as archive:
        for name,path in sorted(files.items()):archive.write(path,'NavigationPhase1/'+name)
        archive.writestr('NavigationPhase1/PACKAGE_MANIFEST.json',json.dumps(manifest,indent=2,ensure_ascii=False))
    partial.replace(output)
    with zipfile.ZipFile(output) as archive:
        failure=archive.testzip()
        if failure:raise IOError('ZIP CRC failure: '+failure)
    report=dict(zip=str(output),files=len(files)+1,bytes=output.stat().st_size,sha256=digest(output))
    output.with_suffix('.zip.sha256').write_text(report['sha256']+'  '+output.name+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=ROOT)
    parser.add_argument('--output',type=Path,default=ROOT/'environment'/'releases'/'NavigationPhase1_20261010.zip')
    args=parser.parse_args();build(args.root,args.output)
