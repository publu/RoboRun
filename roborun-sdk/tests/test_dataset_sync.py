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
