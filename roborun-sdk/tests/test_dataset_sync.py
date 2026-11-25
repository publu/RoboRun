"""The robot's LeRobot dataset (~/roborun/datasets/<robot>/) goes up with the rest of its history, as datasets/<path>."""
from pathlib import Path

from roborun_sdk import robot


def test_the_dataset_folder_is_a_sync_source(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, 'home', classmethod(lambda cls: tmp_path))
    key = tmp_path / 'key'; key.write_text('rr_test_x')
    c = dict(server='https://api.example', key_file=str(key), robot='lab-k1', data=dict(sources={'tasks': str(tmp_path / 'tasks')}))
    sync = robot._syncer(c)
    assert sync.sources['datasets'] == tmp_path / 'roborun' / 'datasets' / 'lab-k1' and 'tasks' in sync.sources
    assert robot.dataset_root('lab-k1') == tmp_path / 'roborun' / 'datasets' / 'lab-k1'


def test_custom_data_folders_are_left_as_given(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, 'home', classmethod(lambda cls: tmp_path))
    key = tmp_path / 'key'; key.write_text('rr_test_x')
    c = dict(server='https://api.example', key_file=str(key), robot='lab-k1', data=dict(sources={'tasks': str(tmp_path)}, custom=True))
    assert 'datasets' not in robot._syncer(c).sources


def test_uninstall_never_takes_the_dataset_with_the_install(tmp_path, monkeypatch):
    """uninstall always removes ~/.roborun and keeps ~/roborun unless asked to purge: the dataset lives in the kept one."""
    monkeypatch.setattr(Path, 'home', classmethod(lambda cls: tmp_path))
    d = robot.dataset_root('lab-k1')
    assert (tmp_path / 'roborun') in d.parents and (tmp_path / '.roborun') not in d.parents


def test_a_new_api_store_sends_the_history_again(tmp_path):
    """When the API names a store this robot has not synced to, what it sent before is not there: it all goes up again, once."""
    from roborun_sdk import datasync
    state = tmp_path / 'sync-state.json'
    health = {}
    sync = datasync.DataSync('https://api.example', 'rr_test_x', 'lab-k1', {}, state_file=state, log=lambda *a: None)
    sync._req = lambda method, path, **kw: (200, dict(health))
    sync.state['files'] = {'datasets/meta/info.json': '10:1'}
    sync._check_store()                                         # an API that names no store: nothing changes
    assert sync.state['files'] and 'store' not in sync.state
    health['store'] = 'aaaa1111bbbb2222'; sync._store_at = -1e9
    sync._check_store()                                         # a store this robot has not synced to: everything again
    assert sync.state['files'] == {} and sync.state['store'] == 'aaaa1111bbbb2222'
    sync.state['files'] = {'datasets/meta/info.json': '10:1'}; sync._store_at = -1e9
    sync._check_store()                                         # the same store: kept
    assert sync.state['files'] == {'datasets/meta/info.json': '10:1'}
    import json
    assert json.loads(state.read_text())['store'] == 'aaaa1111bbbb2222'
    sync._req = lambda method, path, **kw: (_ for _ in ()).throw(RuntimeError('GET /v1/health: 502'))
    sync._store_at = -1e9
    sync._check_store()                                         # an API that cannot answer: the state stays
    assert sync.state['files']
