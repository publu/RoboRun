"""The hosted API as far as `roborun up` needs it, on 127.0.0.1: token check, signing keys, profiles and a tiny runtime."""
import hashlib
import io
import json
import sys
import tarfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import roborun_sdk  # noqa: E402

SDK = Path(roborun_sdk.__file__).resolve().parent
TOKEN = 'rr-test-token'
BACKEND = '''
import argparse, json, os, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
p = argparse.ArgumentParser(); p.add_argument('--config'); p.add_argument('--port', type=int); p.add_argument('--host'); a = p.parse_args()
first = {}
def camera():
    # a stand-in for the overhead camera logic: one pose is rough, a clearly different second pose confirms it
    if not os.environ.get('GUIDED_OVERHEAD'):
        return {}
    try:
        with urllib.request.urlopen('http://127.0.0.1:8091/supervisor/pose', timeout=1) as r:
            q = json.load(r)['measured']['left']['positions_rad']
    except Exception:
        return dict(overhead_camera=dict(state='waiting', message='waiting for the arm'))
    first.setdefault('q', q)
    moved = max(abs(x - y) for x, y in zip(q, first['q']))
    return dict(overhead_camera=dict(state='calibrated', message='calibrated from 2 arm poses', arm='left') if moved > .3
                else dict(state='rough', message='one arm pose cannot prove it is the arm', arm='left'))
class H(BaseHTTPRequestHandler):
    def do_GET(self):
        body = json.dumps(dict(status='ok', profile=json.load(open(a.config)).get('rig'), tasks=[], **camera())).encode()
        self.send_response(200); self.send_header('Content-Length', str(len(body))); self.end_headers(); self.wfile.write(body)
    def log_message(self, *_): pass
ThreadingHTTPServer((a.host, a.port), H).serve_forever()
'''


def bundle():
    out = io.BytesIO()
    with tarfile.open(fileobj=out, mode='w:gz') as tar:
        def add(name, text):
            data = text.encode()
            info = tarfile.TarInfo('manifest/robot/runtime/' + name); info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
        add('kin.py', ''); add('requirements.txt', ''); add('server.py', BACKEND)
        add('RUNTIME.json', json.dumps(dict(version='test-1')))
        add('config.json', json.dumps(dict(body='yam', station='http://127.0.0.1:8091', rig='profiles/yam_single/rig.json')))
        for f in sorted((SDK / 'data' / 'profiles').glob('*.json')):
            add(f'profiles/{f.stem}/rig.json', f.read_text())
            add(f'profiles/{f.stem}/config.json', json.dumps(dict(body='yam', station='http://127.0.0.1:8091', rig=f'profiles/{f.stem}/rig.json')))
    return out.getvalue()


BUNDLE = bundle()


SIGN_IN = dict(started=0, approved=False, taken=False)        # `roborun login`'s device sign-in (RFC 8628), one at a time


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, kind='application/json'):
        raw = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code); self.send_header('Content-Type', kind); self.send_header('Content-Length', str(len(raw)))
        self.end_headers(); self.wfile.write(raw)

    def do_POST(self):
        self.rfile.read(int(self.headers.get('Content-Length') or 0))
        if self.path == '/v1/login/device':
            SIGN_IN.update(started=SIGN_IN['started'] + 1, approved=False, taken=False)
            port = self.server.server_address[1]
            return self._send(200, dict(device_code='dev-1', user_code='ABCD-1234', interval=1, expires_in=600,
                                        verification_uri_complete=f'http://127.0.0.1:{port}/approve?code=ABCD-1234'))
        if self.path == '/v1/login/device/token':
            if SIGN_IN['approved'] and not SIGN_IN['taken']:
                SIGN_IN['taken'] = True
                return self._send(200, dict(key=TOKEN, prefix='rr-test'))
            return self._send(400, dict(error=dict(code='authorization_pending', message='waiting for approval')))
        self._send(404, dict(error='nope'))

    def do_GET(self):
        if self.path.startswith('/approve'):                       # the person approving in the browser
            SIGN_IN['approved'] = True
            return self._send(200, dict(approved=True))
        if self.path == '/test/sign-in':
            return self._send(200, SIGN_IN)
        if self.headers.get('Authorization') != f'Bearer {TOKEN}':
            return self._send(401, dict(error='bad token'))
        if self.path == '/v1/account':
            return self._send(200, dict(account='test'))
        if self.path == '/v1/keys':
            return self._send(200, dict(keys=[]))
        if self.path == '/v1/profiles':
            return self._send(200, dict(profiles=[]))
        if self.path == '/v1/runtime':
            return self._send(200, dict(version='test-1', bytes=len(BUNDLE), file='runtime.tgz', sha256=hashlib.sha256(BUNDLE).hexdigest()))
        if self.path == '/v1/runtime/runtime.tgz':
            return self._send(200, BUNDLE, 'application/gzip')
        self._send(404, dict(error='nope'))

    def log_message(self, *_):
        pass


if __name__ == '__main__':
    server = ThreadingHTTPServer(('127.0.0.1', int(sys.argv[1]) if len(sys.argv) > 1 else 0), Handler)
    print(server.server_address[1], flush=True)
    server.serve_forever()
