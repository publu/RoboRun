"""The SDK's calls to the hosted API on a bad network (a scripted HTTP server on loopback; run inside `unshare -rn`): which
calls are retried, what the user is told, `roborun login` through a dropped connection, the runtime download resumed and
never blamed on the arm, the signing keys refreshed on every `roborun up`, the User-Agent naming the release, and a plain http
server on another computer refused."""
import datetime
import hashlib
import json
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace
import urllib.error
from pathlib import Path

import pytest

from roborun_sdk import login, net, robot
from roborun_sdk.body import jobs as body_jobs


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    monkeypatch.setattr(net, 'BACKOFF_S', 0.01)
    monkeypatch.setattr(net, 'BACKOFF_CAP_S', 0.02)
    monkeypatch.setattr(net, 'RETRY_AFTER_CAP_S', 0.05)


class Script:
    def __init__(self):
        self.rules, self.log = {}, []

    def on(self, method, path, *replies):
        self.rules[(method, path)] = list(replies)

    def next(self, method, path):
        for (m, p), replies in self.rules.items():
            if m == method and path.split('?')[0] == p:
                return replies.pop(0) if len(replies) > 1 else replies[0]
        return (404, {'error': {'code': 'not_found'}}, {})


@pytest.fixture
def api():
    script = Script()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a): pass

        def any(self):
            n = int(self.headers.get('Content-Length') or 0)
            body = self.rfile.read(n) if n else b''
            script.log.append(dict(method=self.command, path=self.path, ua=self.headers.get('User-Agent'),
                                   range=self.headers.get('Range'), body=body))
            r = script.next(self.command, self.path)
            if callable(r): r = r(self)
            if r is None: return
            if r == 'reset':
                self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, b'\x01\x00\x00\x00\x00\x00\x00\x00')
                self.connection.close(); self.close_connection = True; return
            status, payload, headers = r
            data = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
            self.send_response(status)
            for k, v in headers.items(): self.send_header(k, v)
            self.send_header('Content-Length', str(len(data))); self.end_headers()
            if self.command != 'HEAD': self.wfile.write(data)

        do_GET = do_POST = do_PUT = do_HEAD = any

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield SimpleNamespace(url=f'http://127.0.0.1:{server.server_address[1]}', script=script)
    server.shutdown(); server.server_close()


def hits(api, method, path):
    return [r for r in api.script.log if r['method'] == method and r['path'].split('?')[0] == path]


# ---- the shared rule
def test_safe_calls_are_retried_and_others_are_not(api):
    api.script.on('GET', '/v1/x', (503, {'error': {'code': 'upstream_error'}}, {'Retry-After': '0'}), 'reset', (200, {'ok': 1}, {}))
    status, _, body = net.request('GET', api.url + '/v1/x')
    assert status == 200 and json.loads(body) == {'ok': 1} and len(hits(api, 'GET', '/v1/x')) == 3
    api.script.on('POST', '/v1/y', (502, b'<html>bad gateway</html>', {}))
    assert net.request('POST', api.url + '/v1/y', body={})[0] == 502 and len(hits(api, 'POST', '/v1/y')) == 1
    assert net.request('POST', api.url + '/v1/y', body={}, retry=True)[0] == 502 and len(hits(api, 'POST', '/v1/y')) == 1 + net.TRIES


def test_problems_are_one_plain_line_never_a_page():
    assert 'roborun login' in net.problem(401, b'{"error":{"code":"invalid_key","message":"unknown or revoked"}}')
    assert 'robot key' in net.problem(403, b'{"error":{"code":"read_only_key","message":"use a robot key"}}')
    assert 'try again in 60 s' in net.problem(429, b'{"error":{"code":"rate_limited","retry_after_s":60}}')
    text = net.problem(502, b'<html>502 Bad Gateway</html>')
    assert '<html>' not in text and 'HTTP 502' in text and 'try again' in text
    assert 'roborun upgrade' in net.problem(426, b'{"error":{"code":"sdk_upgrade_required","message":"update"}}')


def test_offline_is_said_without_an_errno():
    with pytest.raises(net.Offline) as e:
        net.request('GET', 'http://127.0.0.1:9/v1/x', tries=2)
    assert e.value.reason == 'connection refused' and 'Errno' not in net.offline_text(e.value)


def test_the_user_agent_names_the_release_the_runtime_and_python(tmp_path, monkeypatch):
    monkeypatch.setattr(net.sys, 'prefix', str(tmp_path / 'python'))
    (tmp_path / net.BUILD_FILE).write_text(json.dumps({'build': '20261119-172831-b6abbc5e80'}))
    ua = net.user_agent(runtime='2026.11.19+c18915e')
    assert ua.startswith('roborun-sdk/0.1.0 (build 20261119-172831-b6abbc5e80; runtime 2026.11.19+c18915e; python 3.')


def test_plain_http_to_another_computer_is_refused():
    for url in ('https://api.roborun.app', 'http://127.0.0.1:18990', 'http://localhost:8200', 'http://[::1]:1'):
        assert net.check_server(url) == url
    with pytest.raises(SystemExit, match='unencrypted'):
        net.check_server('http://api.example.com')
    assert net.check_server('http://10.0.0.2', insecure=True) == 'http://10.0.0.2'


# ---- roborun status / setup
def test_status_tells_a_read_only_key_from_a_revoked_one_and_trouble_from_offline(api):
    api.script.on('GET', '/v1/account', (403, {'error': {'code': 'read_only_key', 'message': 'use a robot key'}}, {}))
    assert robot.token_state(api.url, 'k')[0] == 'read_only'
    api.script.on('GET', '/v1/account', (401, {'error': {'code': 'invalid_key'}}, {}))
    assert robot.token_state(api.url, 'k')[0] == 'revoked' and robot.check_token(api.url, 'k') is False
    api.script.on('GET', '/v1/account', (502, b'<html>', {}))
    state, words = robot.token_state(api.url, 'k')
    assert state == 'trouble' and 'HTTP 502' in words and robot.check_token(api.url, 'k') is None
    api.script.on('GET', '/v1/account', (426, {'error': {'code': 'sdk_upgrade_required', 'message': 'upgrade'}}, {}))
    state, words = robot.token_state(api.url, 'k')
    assert state == 'upgrade' and 'roborun upgrade' in words and robot.check_token(api.url, 'k') is None
    assert robot.token_state('http://127.0.0.1:9', 'k')[0] == 'offline'
    api.script.on('GET', '/v1/account', (200, {}, {}))
    assert robot.token_state(api.url, 'k') == ('ok', 'token accepted')


def test_a_gateway_page_is_not_printed_raw(api):
    api.script.on('GET', '/v1/runtime', (502, b'<html>502 Bad Gateway</html>', {}))
    with pytest.raises(SystemExit) as e:
        robot._http('GET', api.url + '/v1/runtime', 'k')
    assert '<html>' not in str(e.value) and 'HTTP 502' in str(e.value)
    api.script.on('GET', '/v1/hotel', (200, b'<html>login to the wifi</html>', {}))
    with pytest.raises(SystemExit, match='captive portal'):
        robot._http('GET', api.url + '/v1/hotel', 'k')


# ---- the runtime download
def test_a_dropped_download_resumes_and_is_never_blamed_on_the_arm(api, tmp_path):
    blob = bytes(range(256)) * 400
    half = len(blob) // 2

    def first(h):            # half the file, then the connection drops
        h.send_response(200); h.send_header('Content-Length', str(len(blob))); h.end_headers(); h.wfile.write(blob[:half])
        h.wfile.flush(); h.connection.shutdown(socket.SHUT_RDWR); h.close_connection = True

    def rest(h):
        start = int(h.headers['Range'].split('=')[1].rstrip('-'))
        h.send_response(206); h.send_header('Content-Length', str(len(blob) - start)); h.end_headers(); h.wfile.write(blob[start:])

    api.script.on('GET', '/v1/runtime/roborun-runtime-x.tar.gz', first, rest)
    out = robot.download(api.url + '/v1/runtime/roborun-runtime-x.tar.gz', 'k', tmp_path / 'b.tgz', len(blob), sleep=lambda s: None)
    assert hashlib.sha256(out.read_bytes()).digest() == hashlib.sha256(blob).digest()
    assert hits(api, 'GET', '/v1/runtime/roborun-runtime-x.tar.gz')[1]['range'] == f'bytes={half}-'
    api.script.on('GET', '/v1/runtime/roborun-runtime-y.tar.gz', 'reset')
    with pytest.raises(SystemExit) as e:
        robot.download(api.url + '/v1/runtime/roborun-runtime-y.tar.gz', 'k', tmp_path / 'c.tgz', 10, sleep=lambda s: None)
    text = str(e.value)
    assert text.startswith('could not download the Roborun runtime') and 'arm' not in text and 'CAN' not in text


def test_a_runtime_that_needs_a_newer_sdk_is_not_installed(api, tmp_path, monkeypatch):
    monkeypatch.setattr(net, 'build', lambda: '20261101-000000-aaaa')
    api.script.on('GET', '/v1/runtime', (200, {'version': '9', 'file': 'roborun-runtime-9.tar.gz', 'sha256': 'x', 'bytes': 1,
                                               'min_sdk_build': '20261201-000000-bbbb'}, {}))
    with pytest.raises(SystemExit, match='needs a newer Roborun'):
        robot.ensure_runtime(tmp_path / 'robot' / 'runtime', api.url, 'k')
    assert not hits(api, 'GET', '/v1/runtime/roborun-runtime-9.tar.gz')


# ---- signing keys on every start
def test_roborun_up_refreshes_the_signing_keys(api, tmp_path, monkeypatch):
    (tmp_path / 'key').write_text('rr_test_x\n')
    jobs = tmp_path / 'jobs.json'
    jobs.write_text(json.dumps(dict(mode='api', trusted_keys=[{'kid': 'old', 'pub': 'A', 'not_after': '2099-01-01'}])))
    soon = (datetime.date.today() + datetime.timedelta(days=3)).isoformat()
    api.script.on('GET', '/v1/keys', (200, {'keys': [{'kid': 'new', 'pub': 'B', 'not_after': soon}]}, {}))
    said = []
    c = dict(server=api.url, key_file=str(tmp_path / 'key'), jobs=str(jobs))
    assert robot.refresh_keys(c, say=said.append) == [{'kid': 'new', 'pub': 'B', 'not_after': soon}]
    assert json.loads(jobs.read_text())['trusted_keys'][0]['kid'] == 'new' and oct(jobs.stat().st_mode & 0o777) == '0o600'
    assert said and soon in said[0]
    api.script.on('GET', '/v1/keys', 'reset')                       # offline: the pinned keys stay
    assert robot.refresh_keys(c, say=said.append)[0]['kid'] == 'new'


# ---- roborun login
def test_login_survives_a_dropped_connection_and_slow_down(api, tmp_path, monkeypatch):
    monkeypatch.setattr('roborun_sdk.arms.settings.store_token', lambda f, k: f.write_text(k))
    api.script.on('POST', '/v1/login/device', 'reset', (200, dict(device_code='d', user_code='U', interval=1, expires_in=60,
                                                                  verification_uri_complete='https://app.roborun.app/device?code=U'), {}))
    api.script.on('POST', '/v1/login/device/token', 'reset', (400, {'error': {'code': 'slow_down'}}, {}),
                  (400, {'error': {'code': 'authorization_pending'}}, {}), (200, {'key': 'rr_test_new', 'prefix': 'rr_test_n'}, {}))
    waits, out = [], []
    got = login.login(api.url, tmp_path / 'key', name='t', out=out.append, sleep=waits.append)
    assert got['key'] == 'rr_test_new' and (tmp_path / 'key').read_text() == 'rr_test_new'
    assert waits[2] >= waits[1] + login.SLOW_DOWN_S                 # slow_down adds to the interval
    assert any('cannot be reached' in line for line in out)
    assert len(hits(api, 'POST', '/v1/login/device/token')) == 4     # each poll asks once


def test_login_offline_is_one_line_not_a_traceback(tmp_path):
    with pytest.raises(SystemExit, match='could not be reached'):
        login.login('http://127.0.0.1:9', tmp_path / 'key', name='t', out=lambda s: None, sleep=lambda s: None)


# ---- the humanoid job client
def test_a_body_job_is_polled_through_an_outage(api):
    api.script.on('GET', '/v1/motion/jobs/b1', 'reset', 'reset', 'reset', (200, {'status': 'done'}, {}))
    api.script.on('POST', '/v1/motion/jobs', (503, {'error': {'code': 'upstream_error'}}, {'Retry-After': '0'}), (202, {'status': 'queued'}, {}))
    client = body_jobs.BodyJobs(api.url, 'k', poll_s=0)
    assert client.submit({'id': 'b1'})['status'] == 'queued' and len(hits(api, 'POST', '/v1/motion/jobs')) == 2
    stages = []
    with pytest.raises(body_jobs.BodyJobError, match='not available'):     # done; the result itself is not scripted here
        client.result = lambda request: (_ for _ in ()).throw(body_jobs.BodyJobError('not_ready', 'not available'))
        client.keys = lambda refresh=False: {}
        client.upload = lambda data, content_type='image/jpeg': 'sha'
        client.submit = lambda request: {'status': 'queued'}
        client.run({'id': 'b1'}, b'jpeg', on_status=stages.append, sleep=lambda s: None)
    assert dict(stage='reconnecting', job='b1') in stages


# ---- releases: the API's latest / minimum, the update notice, roborun upgrade
OLD, NEW, MIN = '20261119-172831-b6abbc5e80', '20261201-000000-bbbbbbbbbb', '20261125-000000-aaaaaaaaaa'


@pytest.fixture
def home(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBORUN_HOME', str(tmp_path))
    monkeypatch.setattr(net, 'build', lambda: OLD)
    return tmp_path


def test_every_call_names_its_build_and_the_apis_releases_are_remembered(api, home):
    api.script.on('GET', '/v1/x', (200, {}, {'X-Roborun-SDK-Latest': NEW, 'X-Roborun-SDK-Minimum': '20261101-000000-aaaaaaaaaa'}))
    net.request('GET', api.url + '/v1/x')
    row = hits(api, 'GET', '/v1/x')[0]
    assert f'(build {OLD};' in row['ua']
    assert net.update_status() == dict(available=True, latest=NEW, required=False, current=OLD)
    assert net.update_notice(now=1000.) == f'update available: {NEW}, run: roborun upgrade'
    assert net.update_notice(now=1000. + 3600) is None                     # at most once a day
    assert net.update_notice(now=1000. + 86401) is not None


def test_the_build_header_is_sent(api, home):
    seen = []

    def grab(h):
        seen.append(h.headers.get('X-Roborun-SDK'))
        return (200, {}, {})
    api.script.on('GET', '/v1/y', grab)
    net.request('GET', api.url + '/v1/y')
    assert seen == [OLD]


def test_a_current_or_source_build_is_not_told_to_update(home, monkeypatch):
    net._write_update(dict(latest=OLD, minimum=None))
    assert net.update_status()['available'] is False and net.update_notice() is None
    monkeypatch.setattr(net, 'build', lambda: None)
    net._write_update(dict(latest=NEW, minimum=MIN))
    assert net.update_status() == dict(available=False, latest=NEW, required=False, current=None)


def test_below_the_minimum_every_notice_says_upgrade_and_426_is_one_line(api, home):
    api.script.on('POST', '/v1/motion/jobs', (426, {'error': {'code': 'sdk_upgrade_required', 'message': 'This Roborun version is '
                                                              'no longer supported. Upgrade: curl -fsSL https://roborun.app/install.sh | sh'}},
                                              {'X-Roborun-SDK-Latest': NEW, 'X-Roborun-SDK-Minimum': MIN}))
    with pytest.raises(SystemExit) as e:
        robot._http('POST', api.url + '/v1/motion/jobs', 'k', body={})
    assert 'roborun upgrade' in str(e.value) and 'Traceback' not in str(e.value)
    assert net.update_status()['required'] is True
    assert 'no longer supported' in net.update_notice(now=1.) and 'no longer supported' in net.update_notice(now=2.)   # every time


class _Ran:
    def __init__(self, code=0): self.returncode = code


def _fake_run(ran, installs, code=0, curl_code=0):
    """subprocess.run as cmd_upgrade uses it: curl writes the installer, sh installs (the new build), roborun up restarts."""
    def run(argv, **k):
        ran.append(argv)
        if argv[0] == 'curl':
            if not curl_code:
                Path(argv[-1]).write_text('#!/bin/sh\n')
            return _Ran(curl_code)
        if argv[0] == 'sh' and not code:
            installs.append(k.get('env', {}).get('ROBORUN_UPGRADING'))
        return _Ran(code)
    return run


def test_upgrade_reinstalls_then_restarts_only_when_no_task_runs(home, monkeypatch, capsys):
    ran, installs = [], []
    builds = {'now': OLD}
    monkeypatch.setattr(robot, 'latest_build', lambda: NEW)
    monkeypatch.setattr(robot, 'installed_build', lambda: builds['now'])
    run = _fake_run(ran, installs)
    monkeypatch.setattr('subprocess.run', lambda argv, **k: (run(argv, **k), builds.update(now=NEW) if argv[0] == 'sh' else None)[0])
    monkeypatch.setattr(robot, 'CONFIG', home / 'robot.json')
    monkeypatch.setattr(robot, '_state', lambda unit: 'active')
    monkeypatch.setattr(robot, '_busy', lambda backend: True)
    assert robot.cmd_upgrade(SimpleNamespace()) == 0
    assert [r[0] for r in ran] == ['curl', 'sh'] and robot.INSTALL_URL in ran[0] and installs == ['1']
    assert 'task is running' in capsys.readouterr().out
    monkeypatch.setattr(robot, '_busy', lambda backend: False)
    ran.clear(); builds['now'] = OLD
    assert robot.cmd_upgrade(SimpleNamespace()) == 0
    assert len(ran) == 3 and ran[2][-1] == 'up'
    builds['now'] = OLD
    monkeypatch.setattr('subprocess.run', _fake_run(ran, installs, code=1))
    ran.clear()
    with pytest.raises(SystemExit, match='did not finish'):
        robot.cmd_upgrade(SimpleNamespace())
    assert [r[0] for r in ran] == ['curl', 'sh']                            # nothing restarted after a failed install


def test_upgrade_with_no_network_says_so_and_changes_nothing(home, monkeypatch, capsys):
    ran = []
    monkeypatch.setattr(robot, 'installed_build', lambda: OLD)
    monkeypatch.setattr('subprocess.run', lambda argv, **k: (ran.append(argv), _Ran())[1])

    def offline():
        raise urllib.error.URLError(OSError(101, 'Network is unreachable'))
    monkeypatch.setattr(robot, 'latest_build', offline)
    with pytest.raises(SystemExit) as e:
        robot.cmd_upgrade(SimpleNamespace())
    assert 'could not reach roborun.app' in str(e.value) and f'Still on {OLD}' in str(e.value) and ran == []
    # the list of releases answered, but the installer itself did not download: also a failure, also nothing changed
    monkeypatch.setattr(robot, 'latest_build', lambda: NEW)
    monkeypatch.setattr('subprocess.run', _fake_run(ran, [], curl_code=7))
    with pytest.raises(SystemExit) as e:
        robot.cmd_upgrade(SimpleNamespace())
    assert 'could not download the installer' in str(e.value) and 'curl exit 7' in str(e.value) and [r[0] for r in ran] == ['curl']


def test_upgrade_on_the_newest_build_does_nothing(home, monkeypatch, capsys):
    ran = []
    monkeypatch.setattr(robot, 'installed_build', lambda: NEW)
    monkeypatch.setattr(robot, 'latest_build', lambda: NEW)
    monkeypatch.setattr('subprocess.run', lambda argv, **k: (ran.append(argv), _Ran())[1])
    assert robot.cmd_upgrade(SimpleNamespace(force=False)) == 0
    assert f'already on the newest Roborun ({NEW})' in capsys.readouterr().out and ran == []


def test_the_installer_is_silent_about_next_steps_during_an_upgrade_and_reports_failed_downloads():
    script = (Path(robot.__file__).resolve().parents[2] / 'install.sh').read_text()
    assert '[ -n "${ROBORUN_UPGRADING:-}" ] && return 0' in script
    assert '| env UV_UNMANAGED_INSTALL' not in script and 'could not reach astral.sh' in script


def test_upgrade_is_a_command():
    import argparse
    sub = argparse.ArgumentParser().add_subparsers()
    robot.add_parsers(sub)
    assert sub.choices['upgrade'].get_default('func') is robot.cmd_upgrade


def test_status_names_a_signing_key_past_or_near_its_date(tmp_path):
    import datetime, json
    from roborun_sdk import robot
    jobs = tmp_path / 'jobs.json'
    c = {'jobs': str(jobs)}
    assert robot.key_date_notice(c) is None                                   # no jobs.json yet: nothing to say
    today = datetime.date(2026, 11, 20)
    jobs.write_text(json.dumps({'trusted_keys': [{'kid': 'a', 'not_after': '2026-11-01'}]}))
    assert 'past its date (2026-11-01)' in robot.key_date_notice(c, today)
    jobs.write_text(json.dumps({'trusted_keys': [{'kid': 'a', 'not_after': '2026-11-01'}, {'kid': 'b', 'not_after': '2026-11-25'}]}))
    assert 'ends 2026-11-25' in robot.key_date_notice(c, today)
    jobs.write_text(json.dumps({'trusted_keys': [{'kid': 'b', 'not_after': '2027-11-20'}]}))
    assert robot.key_date_notice(c, today) is None
