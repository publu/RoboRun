"""`roborun up` only ever replaces a runtime it installed itself; a source checkout or a shared folder is never written to."""
import json
from pathlib import Path

import pytest

from roborun_sdk import robot


def _runtime(top, extra=()):
    root = top / 'robot' / 'runtime'
    root.mkdir(parents=True)
    (root / 'kin.py').write_text('')
    (root / 'RUNTIME.json').write_text(json.dumps(dict(version='old')))
    for d in ('depth', 'harness', 'supervisor'):
        (top / d).mkdir()
    for name in extra:
        (top / name).mkdir()
    return root


def test_an_sdk_install_is_ours(tmp_path):
    assert robot._ours(_runtime(tmp_path / 'roborun'))


def test_a_source_checkout_is_never_touched(tmp_path, monkeypatch):
    root = _runtime(tmp_path / 'dl', extra=('src', 'packages'))
    monkeypatch.setattr(robot, '_http', lambda *a, **k: pytest.fail('no update check for a folder that is not ours'))
    assert robot._ours(root) is False
    assert robot.ensure_runtime(root, 'https://api.example', 'key', update=True) is False


def test_a_tree_without_runtime_json_is_not_ours(tmp_path):
    root = _runtime(tmp_path / 'roborun')
    (root / 'RUNTIME.json').unlink()
    assert robot._ours(root) is False


def test_a_source_checkout_never_uploads_by_default(tmp_path):
    from types import SimpleNamespace
    a = SimpleNamespace(manifest=str(_runtime(tmp_path / 'dl', extra=('src', 'packages'))), data_root=None, video_motion=None, no_data_sync=False)
    assert robot._data_config(a)['enabled'] is False
    a.manifest = str(_runtime(tmp_path / 'roborun'))
    assert robot._data_config(a)['enabled'] is True


def test_a_runtime_update_keeps_the_robots_dataset(tmp_path, monkeypatch):
    """An install holding ~/roborun/datasets is still ours (updates keep coming), and the update replaces the runtime but
    leaves every episode where it was."""
    import hashlib, subprocess, tarfile
    top = tmp_path / 'roborun'
    root = _runtime(top)
    episode = top / 'datasets' / 'my-robot' / 'meta' / 'info.json'
    episode.parent.mkdir(parents=True); episode.write_text('{"total_episodes": 3}')
    (top / 'depth' / 'old.py').write_text('')
    assert robot._ours(root)

    src = tmp_path / 'bundle'
    new = src / 'manifest' / 'robot' / 'runtime'; new.mkdir(parents=True)
    (new / 'kin.py').write_text(''); (new / 'requirements.txt').write_text('')
    (new / 'RUNTIME.json').write_text(json.dumps(dict(version='new')))
    (src / 'manifest' / 'depth').mkdir()
    bundle = tmp_path / 'roborun-runtime-new.tar.gz'
    with tarfile.open(bundle, 'w:gz') as tar:
        for f in sorted((src / 'manifest').rglob('*')):        # entries named manifest/..., as the real bundle's are
            tar.add(f, arcname=str(f.relative_to(src)), recursive=False)
    meta = dict(version='new', file=bundle.name, bytes=bundle.stat().st_size, sha256=hashlib.sha256(bundle.read_bytes()).hexdigest())
    monkeypatch.setattr(robot, '_http', lambda *a, **k: meta)
    monkeypatch.setattr(robot, 'download', lambda url, key, target, size=None: Path(target).write_bytes(bundle.read_bytes()))
    monkeypatch.setattr(subprocess, 'run', lambda *a, **k: subprocess.CompletedProcess(a, 0, '', ''))

    assert robot.ensure_runtime(root, 'https://api.example', 'key', update=True) is True
    assert robot.installed_runtime(root) == 'new'
    assert not (top / 'depth' / 'old.py').exists()             # the runtime's own folders were replaced
    assert episode.read_text() == '{"total_episodes": 3}'      # the episodes were not


def test_the_history_upload_includes_the_station_telemetry(tmp_path):
    sources = robot._data_sources(str(tmp_path / 'runtime'))
    assert sources['logs'] == str((tmp_path / 'runtime' / 'log' / 'telemetry').absolute())
