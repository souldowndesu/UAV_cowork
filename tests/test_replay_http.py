import json
from urllib.error import HTTPError
from urllib.request import urlopen

import pytest

from nav.recorder import Recorder, RecordingLibrary
from nav.web_server import VizServer


def request(server, path):
    with urlopen(server.url + path, timeout=3) as response:
        return json.load(response)


def test_bad_manifest_does_not_hide_good_recording(tmp_path):
    bad = tmp_path/'bad'/'manifest.json'; bad.parent.mkdir()
    bad.write_text('{broken')
    rec = Recorder(); rec.record_map(1, []); rec.save(tmp_path/'good'/'recording.json')
    server = VizServer(library=RecordingLibrary(tmp_path), port=0); server.start()
    try:
        entries = request(server, 'api/recordings')
        assert len(entries) == 2
        broken = next(m for m in entries if m['name'] == 'bad')
        good = next(m for m in entries if m['name'] == 'good')
        assert broken['error']
        assert request(server, 'api/recording?id=' + good['id'])['maps'][0]['occ'] == []
        with pytest.raises(HTTPError) as exc:
            request(server, 'api/recording?id=' + broken['id'])
        assert exc.value.code == 422
        assert 'error' in json.load(exc.value)
    finally:
        server.stop()


def test_digit_only_stable_id_is_not_an_index():
    class Library:
        def get(self, key):
            assert key == '1234567890123456'
            return {'states': []}
    server = VizServer(library=Library(), port=0); server.start()
    try:
        assert request(server, 'api/recording?id=1234567890123456') == {'states': []}
    finally:
        server.stop()


def test_web_servers_keep_independent_recordings():
    first = VizServer(recording={'goal': [1, 2, 3]}, port=0)
    second = VizServer(recording={'goal': [4, 5, 6]}, port=0)
    first.start(); second.start()
    try:
        assert request(first, 'api/recording')['goal'] == [1, 2, 3]
        assert request(second, 'api/recording')['goal'] == [4, 5, 6]
    finally:
        first.stop(); second.stop()
