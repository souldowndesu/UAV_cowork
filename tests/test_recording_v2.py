import json
import numpy as np
import pytest
from nav.recorder import Recorder,RecordingLibrary,load_recording_dict

def test_v2_map_removals_empty_frames_and_counts(tmp_path):
    rec=Recorder(); rec.set_goal([200,20,-20]); rec.start(tmp_path/'recording.json',{'res':.2})
    rec.record_state(1,[0,0,-10],[0,0,0],[0,0,0])
    rec.record_map(1,[[1,2,3]],[[4,5,6]])
    rec.record_map(2,[],[])
    rec.record_points(1,[[1,2,3]])
    rec.record_event('map_geometry',dict(t=1,origin=[0,0,0],shape=[10,10,10],res=.2,version=1))
    rec.close(); p=tmp_path/'manifest.json'
    meta=json.loads(p.read_text()); data=load_recording_dict(p)
    assert meta['complete'] and meta['counts']['maps']==2
    assert data['maps'][0]['occ']==[1.,2.,3.] and data['maps'][1]['occ']==[]
    assert data['maps'][1]['unk']==[] and data['geometry'][0]['version']==1
    assert rec.snapshot_live()['map']['occ']==[]
    assert len(data['states'])==meta['counts']['states']==1

def test_partial_recording_remains_readable(tmp_path):
    rec=Recorder(); rec.start(tmp_path/'recording.json')
    rec.record_state(1,[0,0,0],[0,0,0],[0,0,0]); rec.flush()
    p=tmp_path/'manifest.json'; assert not json.loads(p.read_text())['complete']
    assert len(load_recording_dict(p)['states'])==1
    rec.close()

def test_library_rescan_stable_id_and_invalidation(tmp_path):
    lib=RecordingLibrary(tmp_path); assert lib.list()==[]
    p=tmp_path/'first'/'recording.json'; rec=Recorder(); rec.record_state(1,[0,0,0],[0,0,0],[0,0,0]); rec.save(str(p))
    id1=lib.list()[0]['id']; assert len(lib.get(id1)['states'])==1
    rec.record_state(2,[1,0,0],[0,0,0],[0,0,0]); rec.save(str(p))
    assert lib.list()[0]['id']==id1 and len(lib.get(id1)['states'])==2

def test_failed_save_never_overwrites_committed_data(tmp_path,monkeypatch):
    import nav.recorder as module
    rec=Recorder(); rec.record_state(1,[0,0,0],[0,0,0],[0,0,0]); path=tmp_path/'recording.json'; rec.save(str(path))
    before=path.read_bytes()
    def fail(*args): raise OSError('injected replace failure')
    monkeypatch.setattr(module.os,'replace',fail)
    rec.record_state(2,[1,0,0],[0,0,0],[0,0,0])
    with pytest.raises(OSError): rec.save(str(path))
    assert path.read_bytes()==before

def test_chunk_corruption_is_reported(tmp_path):
    rec=Recorder(); rec.start(tmp_path/'recording.json'); rec.record_map(0,[],[]); rec.close()
    next(tmp_path.glob('chunk*.gz')).write_bytes(b'corrupt')
    with pytest.raises(ValueError,match='checksum'): load_recording_dict(tmp_path/'manifest.json')


@pytest.mark.parametrize('timestamp', [float('nan'), float('inf'), -float('inf'), None, 'bad'])
def test_invalid_timestamps_do_not_poison_writer(tmp_path, timestamp):
    rec = Recorder(); rec.start(tmp_path/'recording.json')
    rec.record_points(timestamp, [[1, 2, 3]])
    rec.record_map(timestamp, [[1, 2, 3]])
    rec.record_plan(timestamp, [1, 2, 3], [], [], [])
    rec.record_state(1, [0, 0, 0], [0, 0, 0], [0, 0, 0])
    rec.close()
    data = load_recording_dict(tmp_path/'manifest.json')
    assert data['complete'] and len(data['states']) == 1
    assert not data['maps'] and not data['points'] and not data['plans']


def test_event_is_owned_by_recorder(tmp_path):
    rec = Recorder(); rec.start(tmp_path/'recording.json')
    event = dict(t=1, outcome='timeout', detail={'values': [1, 2]})
    rec.record_event('outcome', event)
    event['detail']['values'][0] = 999
    rec.close()
    data = load_recording_dict(tmp_path/'manifest.json')
    assert data['outcomes'][0]['detail']['values'] == [1, 2]


def test_close_is_final_and_flush_after_close_is_safe(tmp_path):
    rec = Recorder(); rec.start(tmp_path/'recording.json'); rec.close()
    rec.flush(); rec.close()
    with pytest.raises(RuntimeError, match='closed'): rec.start(tmp_path/'other'/'recording.json')
    with pytest.raises(RuntimeError, match='closed'): rec.record_map(1, [])


def test_legacy_save_retains_more_than_live_buffer(tmp_path):
    rec = Recorder()
    for i in range(1025): rec.record_state(i, [i, 0, 0], [0, 0, 0], [0, 0, 0])
    path = tmp_path/'recording.json'; rec.save(path)
    assert len(load_recording_dict(path)['states']) == 1025
    with pytest.raises(RuntimeError, match='precede'): rec.start(tmp_path/'other'/'recording.json')


def test_streaming_bounds_live_buffer_without_losing_disk_history(tmp_path):
    rec = Recorder(); rec.start(tmp_path/'recording.json')
    for i in range(1025): rec.record_state(i, [i, 0, 0], [0, 0, 0], [0, 0, 0])
    rec.close()
    assert len(rec.states) == 512
    assert len(load_recording_dict(tmp_path/'manifest.json')['states']) == 1025


def test_writer_commit_failure_keeps_previous_manifest(tmp_path, monkeypatch):
    import time
    import nav.recorder as module
    rec = Recorder(); rec.start(tmp_path/'recording.json')
    rec.record_map(1, [[1, 2, 3]]); rec.flush()
    path = tmp_path/'manifest.json'; before = path.read_bytes()
    replace = module.os.replace
    def fail_manifest(src, dst):
        if str(dst).endswith('manifest.json'): raise OSError('injected manifest failure')
        return replace(src, dst)
    monkeypatch.setattr(module.os, 'replace', fail_manifest)
    rec.record_map(2, [])
    started = time.monotonic()
    with pytest.raises(RuntimeError, match='writer failed'): rec.flush()
    assert time.monotonic() - started < 3
    with pytest.raises(RuntimeError, match='writer failed'): rec.close()
    assert path.read_bytes() == before
    data = load_recording_dict(path)
    assert not data['complete'] and len(data['maps']) == 1
    assert data['maps'][0]['occ'] == [1, 2, 3]


@pytest.mark.parametrize('change, message', [
    ('version', 'unsupported'), ('count', 'event count'),
    ('duplicate', 'duplicate chunk'), ('sequence', 'sequence'),
])
def test_manifest_and_sequence_validation(tmp_path, change, message):
    import gzip
    import hashlib
    rec = Recorder(); rec.start(tmp_path/'recording.json')
    rec.record_map(1, []); rec.record_map(2, []); rec.close()
    path = tmp_path/'manifest.json'; meta = json.loads(path.read_text())
    if change == 'version': meta['schema_version'] = 999
    elif change == 'count': meta['chunks'][0]['events'] += 1
    elif change == 'duplicate': meta['chunks'].append(meta['chunks'][0])
    else:
        chunk = meta['chunks'][0]; chunk_path = tmp_path/chunk['file']
        events = json.loads(gzip.decompress(chunk_path.read_bytes()))
        events[1]['data']['seq'] = events[0]['data']['seq']
        payload = gzip.compress(json.dumps(events).encode())
        chunk_path.write_bytes(payload); chunk['sha256'] = hashlib.sha256(payload).hexdigest()
    path.write_text(json.dumps(meta))
    with pytest.raises(ValueError, match=message): load_recording_dict(path)
