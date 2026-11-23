"""`roborun login`: the link to approve this computer shows at once, also when the output goes to a pipe (a log, a coding
agent), and nothing is saved until the sign-in is approved."""
import json
import os
import select
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / 'src'


class Pending(BaseHTTPRequestHandler):
    def do_POST(self):
        self.rfile.read(int(self.headers.get('Content-Length') or 0))
        if self.path == '/v1/login/device':
            code, body = 200, dict(device_code='dev-1', user_code='ABCD-EFGH', interval=1, expires_in=60,
                                   verification_uri_complete='https://app.roborun.app/device?code=ABCD-EFGH')
        else:
            code, body = 400, dict(error=dict(code='authorization_pending', message='waiting'))
        data = json.dumps(body).encode()
        self.send_response(code); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(data)))
        self.end_headers(); self.wfile.write(data)

    def log_message(self, *_):
        pass


def test_the_approve_link_reaches_a_pipe_before_approval(tmp_path):
    server = ThreadingHTTPServer(('127.0.0.1', 0), Pending)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    key = tmp_path / 'token'
    env = dict(os.environ, HOME=str(tmp_path), PYTHONPATH=str(SRC))
    env.pop('PYTHONUNBUFFERED', None)
    proc = subprocess.Popen([sys.executable, '-m', 'roborun_sdk.cli', 'login', '--wait', '--server', f'http://127.0.0.1:{server.server_port}',
                             '--key-file', str(key)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env)
    try:
        seen = b''
        while b'Check that the page shows the code ABCD-EFGH.' not in seen:
            ready, _, _ = select.select([proc.stdout], [], [], 60)
            assert ready, f'nothing printed while waiting for approval: {seen!r}'
            chunk = os.read(proc.stdout.fileno(), 4096)
            assert chunk, f'login exited: {seen!r}'
            seen += chunk
        text = seen.decode()
        assert 'Open https://app.roborun.app/device?code=ABCD-EFGH and approve this computer.' in text
        assert proc.poll() is None and not key.exists()                 # still waiting; no key without approval
    finally:
        proc.kill(); proc.wait(); server.shutdown()


class Approved(Pending):
    def do_POST(self):
        if self.path == '/v1/login/device':
            return super().do_POST()
        self.rfile.read(int(self.headers.get('Content-Length') or 0))
        data = json.dumps(dict(key='rr_test_abcdefghijkl_' + 'x' * 32, prefix='rr_test_abcd')).encode()
        self.send_response(200); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(data)))
        self.end_headers(); self.wfile.write(data)


def test_login_prints_the_link_and_exits_and_up_finishes_it(tmp_path, monkeypatch):
    """`roborun login` prints the link and exits; the next `roborun up` (here: its key step) waits for the approval and saves the key."""
    server = ThreadingHTTPServer(('127.0.0.1', 0), Approved)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = f'http://127.0.0.1:{server.server_port}'
    env = dict(os.environ, HOME=str(tmp_path), PYTHONPATH=str(SRC))
    env.pop('ROBORUN_HOME', None)
    try:
        out = subprocess.run([sys.executable, '-m', 'roborun_sdk.cli', 'login', '--server', url], capture_output=True, text=True,
                             env=env, timeout=60)
        home = tmp_path / '.config' / 'roborun'
        assert out.returncode == 0 and 'Open https://app.roborun.app/device?code=ABCD-EFGH' in out.stdout and 'roborun up' in out.stdout
        assert not (home / 'token').exists() and (home / 'login-pending.json').stat().st_mode & 0o777 == 0o600
        from roborun_sdk import robot
        from roborun_sdk.arms import settings
        monkeypatch.setattr(robot, 'HOME', home)
        monkeypatch.delenv('ROBORUN_API_KEY', raising=False); monkeypatch.delenv('ROBORUN_TOKEN', raising=False)
        monkeypatch.setattr('time.sleep', lambda s: None)
        settings._ask_token(home / 'token', url)
        assert (home / 'token').read_text().startswith('rr_test_abcdefghijkl_') and not (home / 'login-pending.json').exists()
    finally:
        server.shutdown()
