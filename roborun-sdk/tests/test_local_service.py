"""The robot on this computer: no link to any server, sign-in against the API, and a status that tells the truth."""
import json
import threading
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace

import pytest

from roborun_sdk import cli, robot
from roborun_sdk.local_proxy import HELLO

TOKEN = 'ab-account1.cd-secretvalue'
APP = 'https://app.roborun.app'


def server(routes):
    seen = []

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass

        def do_GET(self):
            seen.append((self.path, self.headers.get('Authorization')))
            status, body = routes.get(self.path, (404, {}))
            data = json.dumps(body).encode()
            self.send_response(status); self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)
    s = ThreadingHTTPServer(('127.0.0.1', 0), H); s.daemon_threads = True
    threading.Thread(target=s.serve_forever, daemon=True).start()
    s.seen, s.url = seen, f'http://127.0.0.1:{s.server_address[1]}'
    return s


@pytest.fixture()
def home(tmp_path, monkeypatch):
    monkeypatch.setattr(robot, 'HOME', tmp_path); monkeypatch.setattr(robot, 'CONFIG', tmp_path / 'robot.json')
    # never the real home: $HOME and the account's default install are the test's own (see test_install_lifecycle's home)
    (tmp_path / 'user').mkdir()
    monkeypatch.setenv('HOME', str(tmp_path / 'user'))
    monkeypatch.setattr(robot, 'account_home', lambda: tmp_path / 'account' / '.config' / 'roborun')
    (tmp_path / 'token').write_text(TOKEN)
    return tmp_path


def write(home, **kw):
    cfg = dict(server='http://127.0.0.1:9', robot='bench', key_file=str(home / 'token'), backend='http://127.0.0.1:9',
               profile='yam_single', live_port=0, data=dict(enabled=False), hardware=dict(privacy=dict(face_blur=True)))
    robot.CONFIG.write_text(json.dumps(dict(cfg, **kw)))


@pytest.mark.parametrize('module', ['poll', 'agent'])
def test_the_link_to_a_server_is_gone(module):
    with pytest.raises(ImportError):
        __import__(f'roborun_sdk.{module}')


@pytest.mark.parametrize('argv', [['up', '--hub', 'http://x'], ['up', '--gateway', 'ws://x'], ['setup', '--robot', 'r', '--key-file', 'k', '--hub', 'http://x'], ['connect']])
def test_the_old_options_are_refused(argv, capsys):
    with pytest.raises(SystemExit) as e:
        cli.main(argv)
    assert e.value.code != 0


def test_the_token_is_checked_with_the_api():
    ok, bad = server({'/v1/account': (200, dict(ok=True))}), server({'/v1/account': (401, {})})
    assert robot.check_token(ok.url, TOKEN) is True and ok.seen == [('/v1/account', 'Bearer ' + TOKEN)]
    assert robot.check_token(bad.url, TOKEN) is False
    assert robot.check_token('http://127.0.0.1:9', TOKEN) is None           # unreachable says nothing about the token


def test_setup_stops_on_a_refused_token_and_never_shows_it(home, capsys):
    bad = server({'/v1/account': (401, {})})
    a = SimpleNamespace(key_file=str(home / 'token'), server=bad.url, robot='bench', profile='yam_single', rig=None)
    with pytest.raises(SystemExit) as e:
        robot.cmd_setup(a)
    assert 'not accepted' in str(e.value) and TOKEN not in str(e.value) and not robot.CONFIG.exists()


def status(capsys, monkeypatch, running=('roborun-robot', 'roborun-local')):
    monkeypatch.setattr(robot, '_state', lambda u: 'active' if u in running else 'inactive')
    assert robot.cmd_status(None) == 0
    return capsys.readouterr().out


def test_status_says_what_is_true(home, capsys, monkeypatch):
    api = server({'/v1/account': (200, dict(ok=True))})
    backend = server({'/v1/status': (200, dict(profile='yam_single', task=dict(id='abc')))})
    write(home, server=api.url, backend=backend.url, data=dict(enabled=True))
    out = status(capsys, monkeypatch)
    assert 'yam_single' in out and 'abc running' in out and 'token accepted' in out and '(answering)' in out
    assert 'on: every image is blurred' in out
    assert not any(word in out.lower() for word in ('hub', 'manifest', 'gateway'))


def test_status_reports_trouble_as_trouble(home, capsys, monkeypatch):
    api = server({'/v1/account': (401, {})})
    write(home, server=api.url, hardware=dict(privacy=dict(face_blur=False)))
    out = status(capsys, monkeypatch, running=())
    assert 'robot             stopped' in out and 'not answering' in out and 'token not accepted' in out
    assert 'OFF' in out and 'history upload    off' in out


def test_the_local_service_opens_the_page_door_and_calls_nothing(home, monkeypatch):
    api, backend = server({}), server({'/v1/settings': (200, {})})
    with __import__('socket').socket() as s:
        s.bind(('127.0.0.1', 0)); port = s.getsockname()[1]
    write(home, server=api.url, backend=backend.url, live_port=port)
    got = {}

    class Done(Exception): pass

    def pause():
        for _ in range(100):
            try:
                req = urllib.request.Request(f'http://127.0.0.1:{port}{HELLO}', headers={'Origin': APP})
                got.update(json.load(urllib.request.urlopen(req, timeout=1))); break
            except OSError:
                __import__('time').sleep(.05)
        raise Done
    monkeypatch.setattr('signal.pause', pause); monkeypatch.setattr('signal.signal', lambda *a: None)
    with pytest.raises(Done):
        robot.cmd_local(None)
    assert got['robot'] == 'bench' and got['ready'] is True and api.seen == []


def test_the_units_cannot_touch_another_install():
    assert all(u.startswith('roborun-') for u in robot.UNITS)
