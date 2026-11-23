"""A robot gets its motion from the hosted API only: the default server is https://api.roborun.app, the backend's jobs settings say
mode "api" at that server, and no local job transport is ever written. (This guards the settings that
keep every plan coming from api.roborun.app.)"""
import importlib
import inspect
import json
import os

from roborun_sdk import robot


def test_the_default_server_is_the_hosted_api(monkeypatch):
    monkeypatch.delenv('ROBORUN_SERVER', raising=False)
    try:
        assert importlib.reload(robot).SERVER == 'https://api.roborun.app'
    finally:
        importlib.reload(robot)


def test_jobs_settings_point_at_the_server_in_api_mode(tmp_path, monkeypatch):
    monkeypatch.setattr(robot, 'HOME', tmp_path)
    (tmp_path / 'token').write_text('ab-account1.cd-secretvalue\n')
    asked = []

    def keys(method, url, key=None, body=None):
        asked.append((method, url, key))
        return {'keys': [{'kid': 'k1', 'public': 'AAAA'}]}
    monkeypatch.setattr(robot, '_http', keys)
    path = robot._write_jobs(dict(server='https://api.roborun.app', key_file=str(tmp_path / 'token'), robot='bench'))
    jobs = json.loads(open(path).read())
    assert jobs['mode'] == 'api' and jobs['url'] == 'https://api.roborun.app' and jobs['robot'] == 'bench'
    assert 'local_transport' not in jobs
    assert asked == [('GET', 'https://api.roborun.app/v1/keys', 'ab-account1.cd-secretvalue')]       # the API's own signing keys
    assert oct(os.stat(path).st_mode & 0o777) == '0o600'


def test_the_backend_is_started_in_api_mode_without_a_local_transport():
    src = inspect.getsource(robot.cmd_up)
    assert "MANIFEST_JOBS='api'" in src and 'MANIFEST_JOBS_CONFIG=' in src
    assert 'MANIFEST_JOB_TRANSPORT' not in inspect.getsource(robot)
