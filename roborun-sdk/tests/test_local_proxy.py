"""The hosted page's door to this computer's robot: who gets in, and that everything the page needs passes through."""
import http.client
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from fake_policy_bits import own_home

from roborun_sdk.local_proxy import HELLO, LocalProxy, account_of, allowed_origins

APP = 'https://app.roborun.app'
TOKEN = 'ab-account1.cd-secretvalue'


class Backend(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    seen = []
    plugins = False

    def log_message(self, *a): pass

    def _reply(self):
        size = int(self.headers.get('Content-Length') or 0)
        body = self.rfile.read(size) if size else b''
        Backend.seen.append(dict(method=self.command, path=self.path, headers={k.lower(): v for k, v in self.headers.items()}, body=body))
        if self.path.endswith('.mjpg'):
            self.send_response(200); self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=f'); self.send_header('Content-Length', '0')
            self.end_headers(); return
        if self.path == '/v1/plugins/ui.js' and not Backend.plugins:
            self.send_response(404); self.send_header('Content-Length', '0'); self.end_headers(); return
        if self.path.startswith('/v1/stream'):
            self.send_response(200); self.send_header('Content-Type', 'text/event-stream'); self.send_header('Connection', 'close')
            self.end_headers(); self.close_connection = True
            for i in range(3):
                self.wfile.write(b'data: %d\n\n' % i); self.wfile.flush(); time.sleep(.05)
            return
        data = json.dumps(dict(ok=True, path=self.path)).encode()
        self.send_response(206 if self.headers.get('Range') else 200)
        self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(data)))
        self.send_header('Access-Control-Allow-Origin', '*'); self.send_header('Set-Cookie', 'x=1')
        if self.headers.get('Range'): self.send_header('Content-Range', 'bytes 0-1/2')
        self.end_headers()
        if self.command != 'HEAD': self.wfile.write(data)

    do_GET = do_HEAD = do_POST = do_PUT = do_PATCH = do_DELETE = _reply


@pytest.fixture(autouse=True)
def _home(tmp_path, monkeypatch):
    """A client of a robot on 127.0.0.1 also looks for this computer's policy runs (policy.control: robot.json, the runtime's
    task records, closing the stale ones): the test's own home, never this computer's."""
    return own_home(tmp_path, monkeypatch)


@pytest.fixture()
def rig():
    Backend.seen, Backend.plugins = [], False
    backend = ThreadingHTTPServer(('127.0.0.1', 0), Backend); backend.daemon_threads = True
    threading.Thread(target=backend.serve_forever, daemon=True).start()
    proxy = LocalProxy('127.0.0.1', backend.server_address[1], port=0, origins=['http://localhost:5173'], robot='bench', token=TOKEN).bind()
    threading.Thread(target=proxy.server.serve_forever, daemon=True).start()
    yield proxy
    proxy.stop(); backend.shutdown(); backend.server_close()


def call(proxy, method, path, headers=None, body=None, host=None):
    conn = http.client.HTTPConnection('127.0.0.1', proxy.port, timeout=5)
    conn.putrequest(method, path, skip_host=True, skip_accept_encoding=True)
    conn.putheader('Host', host or f'127.0.0.1:{proxy.port}')
    for k, v in (headers or {}).items(): conn.putheader(k, v)
    if body is not None: conn.putheader('Content-Length', str(len(body)))
    conn.endheaders(body)
    r = conn.getresponse(); data = r.read(); conn.close()
    return r, data


def keyed(proxy, path): return f'/k/{proxy.key}{path}'


def no_cors(r): return not [k for k, _ in r.getheaders() if k.lower().startswith('access-control-')]


# ---------------------------------------------------------------- where it listens
def test_binds_loopback_only(rig):
    assert rig.server.server_address[0] == '127.0.0.1'
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect(('10.255.255.255', 9)); ip = s.getsockname()[0]
        except OSError:
            return                                       # no other interface here (isolated test run)
    if ip != '127.0.0.1':
        with pytest.raises(OSError):
            socket.create_connection((ip, rig.port), timeout=.5)


# ---------------------------------------------------------------- handshake
def test_hello_gives_the_allowed_page_the_key_and_account(rig):
    r, data = call(rig, 'GET', HELLO, {'Origin': APP})
    d = json.loads(data)
    assert r.status == 200 and r.getheader('Access-Control-Allow-Origin') == APP and 'Origin' in r.getheader('Vary')
    assert d['roborun'] == 1 and d['key'] == rig.key and d['robot'] == 'bench' and d['ready'] is True
    assert d['account'] == account_of(TOKEN) and 'secretvalue' not in data.decode() and r.getheader('Cache-Control') == 'no-store'


@pytest.mark.parametrize('headers', [{}, {'Origin': 'https://evil.example'}, {'Origin': 'null'}, {'Origin': APP + '.evil.example'},
                                     {'Origin': 'http://app.roborun.app'}, {'Origin': APP + '/'}, {'Origin': 'HTTPS://APP.ROBORUN.APP'}])
def test_hello_refuses_everyone_else(rig, headers):
    r, data = call(rig, 'GET', HELLO, headers)
    assert r.status == 403 and rig.key.encode() not in data and no_cors(r)


def test_hello_never_offers_the_page_code_from_the_robot(rig):
    Backend.plugins = True; rig._backend = (0., False)          # even a backend that serves a script: the page loads none
    assert 'plugins' not in json.loads(call(rig, 'GET', HELLO, {'Origin': APP})[1])


def test_camera_streams_are_not_marked_nosniff_so_an_img_can_show_them(rig):
    r, _ = call(rig, 'GET', keyed(rig, '/internal/cameras/top.mjpg'))
    assert r.status == 200 and r.getheader('X-Content-Type-Options') is None
    assert call(rig, 'GET', keyed(rig, '/v1/settings'))[0].getheader('X-Content-Type-Options') == 'nosniff'


def test_each_run_has_its_own_key(rig):
    assert LocalProxy('127.0.0.1', 1, port=0).key != rig.key


# ---------------------------------------------------------------- Origin
@pytest.mark.parametrize('method', ['GET', 'POST', 'PUT', 'PATCH', 'DELETE'])
@pytest.mark.parametrize('origin', ['https://evil.example', 'null', 'https://sub.app.roborun.app'])
def test_wrong_origin_never_reaches_the_robot_even_with_the_key(rig, method, origin):
    r, _ = call(rig, method, keyed(rig, '/api/stop'), {'Origin': origin, 'X-Robot-Control': '1'}, b'{}' if method != 'GET' else None)
    assert r.status == 403 and no_cors(r) and Backend.seen == []


def test_no_wildcard_and_only_exact_configured_origins():
    assert allowed_origins(['*', 'null', 'http://evil.example', 'https://*.roborun.app']) == {APP}
    assert allowed_origins(['http://localhost:5173']) == {APP, 'http://localhost:5173'}


def test_a_configured_dev_origin_is_allowed_exactly(rig):
    assert call(rig, 'GET', HELLO, {'Origin': 'http://localhost:5173'})[0].status == 200
    assert call(rig, 'GET', HELLO, {'Origin': 'http://localhost:5174'})[0].status == 403


def test_answer_names_the_origin_never_a_wildcard_and_drops_backend_cors_and_cookies(rig):
    r, _ = call(rig, 'GET', keyed(rig, '/v1/settings'), {'Origin': APP})
    assert r.status == 200 and r.getheader('Access-Control-Allow-Origin') == APP
    assert r.getheader('Set-Cookie') is None and r.getheader('Access-Control-Allow-Credentials') is None
    assert r.getheader('X-Content-Type-Options') == 'nosniff' and r.getheader('Content-Security-Policy') == 'sandbox'


# ---------------------------------------------------------------- Host (DNS rebinding)
@pytest.mark.parametrize('host', ['evil.example', 'evil.example:{port}', '127.0.0.1', '127.0.0.1:1', '0.0.0.0:{port}', '[::1]:{port}', 'localhost.evil.example:{port}'])
def test_wrong_host_is_refused(rig, host):
    for path in (HELLO, keyed(rig, '/v1/settings')):
        r, data = call(rig, 'GET', path, {'Origin': APP}, host=host.format(port=rig.port))
        assert r.status == 421 and no_cors(r) and rig.key.encode() not in data
    assert Backend.seen == []


def test_duplicate_host_or_origin_is_refused(rig):
    conn = http.client.HTTPConnection('127.0.0.1', rig.port, timeout=5)
    conn.putrequest('GET', HELLO, skip_host=True, skip_accept_encoding=True)
    conn.putheader('Host', f'127.0.0.1:{rig.port}'); conn.putheader('Host', 'evil.example'); conn.putheader('Origin', APP); conn.endheaders()
    assert conn.getresponse().status == 421
    conn = http.client.HTTPConnection('127.0.0.1', rig.port, timeout=5)
    conn.putrequest('GET', HELLO, skip_host=True, skip_accept_encoding=True)
    conn.putheader('Host', f'127.0.0.1:{rig.port}'); conn.putheader('Origin', APP); conn.putheader('Origin', 'https://evil.example'); conn.endheaders()
    assert conn.getresponse().status == 403


def test_localhost_name_is_accepted_for_the_second_connection_pool(rig):
    assert call(rig, 'GET', keyed(rig, '/v1/settings'), {'Origin': APP}, host=f'localhost:{rig.port}')[0].status == 200


# ---------------------------------------------------------------- preflight
def preflight(rig, origin=APP, path=None, method='POST', headers='content-type,x-robot-control', **extra):
    h = {'Access-Control-Request-Method': method}
    if origin: h['Origin'] = origin
    if headers: h['Access-Control-Request-Headers'] = headers
    h.update({k.replace('_', '-'): v for k, v in extra.items()})
    return call(rig, 'OPTIONS', path or keyed(rig, '/api/stop'), h)[0]


def test_preflight_for_the_page(rig):
    r = preflight(rig)
    assert r.status == 204 and r.getheader('Access-Control-Allow-Origin') == APP
    assert 'POST' in r.getheader('Access-Control-Allow-Methods') and r.getheader('Access-Control-Allow-Headers') == 'content-type, x-robot-control'
    assert r.getheader('Access-Control-Allow-Private-Network') is None and '*' not in (r.getheader('Access-Control-Allow-Headers') or '')
    assert Backend.seen == []


def test_private_and_local_network_are_granted_only_when_asked_for_the_allowed_page(rig):
    assert preflight(rig, Access_Control_Request_Private_Network='true').getheader('Access-Control-Allow-Private-Network') == 'true'
    assert preflight(rig, Access_Control_Request_Local_Network='true').getheader('Access-Control-Allow-Local-Network') == 'true'
    bad = preflight(rig, origin='https://evil.example', Access_Control_Request_Private_Network='true')
    assert bad.status == 403 and no_cors(bad)


@pytest.mark.parametrize('kw', [dict(origin=None), dict(origin='https://evil.example'), dict(origin='null'), dict(method='TRACE'),
                                dict(method='CONNECT'), dict(headers='x-page-only'), dict(headers='x-page-task'),
                                dict(headers='authorization'), dict(path='/v1/settings'), dict(path=HELLO)])
def test_preflight_refusals(rig, kw):
    r = preflight(rig, **kw)
    assert r.status == 403 and no_cors(r)


# ---------------------------------------------------------------- the key
def test_nothing_without_the_key(rig):
    for path in ('/v1/settings', '/k/wrong/v1/settings', '/internal/cameras/top.mjpg', '/k/', f'/k/{rig.key}', f'/k/{rig.key[:-1]}/v1/settings'):
        assert call(rig, 'GET', path)[0].status == 404, path
    assert Backend.seen == []


def test_media_without_origin_needs_the_key_and_can_only_read(rig):
    r, _ = call(rig, 'GET', keyed(rig, '/v1/video-motion/a/input.jpg'))
    assert r.status == 200 and no_cors(r)
    for method in ('POST', 'PUT', 'PATCH', 'DELETE'):
        before = len(Backend.seen)
        assert call(rig, method, keyed(rig, '/api/stop'), {'X-Robot-Control': '1'}, b'{}')[0].status == 403 and len(Backend.seen) == before


@pytest.mark.parametrize('path', ['/', '/ui/index.html', '/docs', '/v1/../secret', '/v1/%2e%2e/secret', '/v1/a%2fb', '/v1/a%5cb', '/internal/..', '/v1/a%00'])
def test_only_the_robot_api_paths_pass(rig, path):
    assert call(rig, 'GET', keyed(rig, path), {'Origin': APP})[0].status == 404 and Backend.seen == []


# ---------------------------------------------------------------- what passes through
def test_post_reaches_the_backend_with_only_the_page_headers(rig):
    r, data = call(rig, 'POST', keyed(rig, '/api/stop?now=1'), {
        'Origin': APP, 'Content-Type': 'application/json', 'X-Robot-Control': '1', 'Cookie': 'manifest_session=x', 'Authorization': 'Bearer y',
        'X-Page-Only': '1', 'X-Page-Task': 'z', 'Referer': APP + '/'}, b'{"a":1}')
    assert r.status == 200 and json.loads(data)['path'] == '/api/stop?now=1'
    seen = Backend.seen[0]
    assert seen['method'] == 'POST' and seen['body'] == b'{"a":1}' and seen['headers']['x-robot-control'] == '1' and seen['headers']['origin'] == APP
    assert not {'cookie', 'authorization', 'x-page-only', 'x-page-task', 'referer'} & set(seen['headers'])
    assert seen['headers']['host'] != f'127.0.0.1:{rig.port}'


def test_range_head_and_keep_alive(rig):
    r, _ = call(rig, 'GET', keyed(rig, '/v1/video-motion/a/file.bin'), {'Range': 'bytes=0-1'})
    assert r.status == 206 and r.getheader('Content-Range') == 'bytes 0-1/2' and Backend.seen[0]['headers']['range'] == 'bytes=0-1'
    assert call(rig, 'HEAD', keyed(rig, '/v1/x'), {'Origin': APP})[0].status == 200
    conn = http.client.HTTPConnection('127.0.0.1', rig.port, timeout=5)
    for _ in range(3):
        conn.request('GET', keyed(rig, '/v1/x'), headers={'Origin': APP}); r = conn.getresponse(); r.read(); assert r.status == 200
    assert conn.sock is not None; conn.close()


def test_streams_arrive_as_they_are_sent(rig):
    conn = http.client.HTTPConnection('127.0.0.1', rig.port, timeout=5)
    conn.request('GET', keyed(rig, '/v1/stream'), headers={'Origin': APP}); r = conn.getresponse()
    assert r.status == 200 and r.getheader('Content-Type') == 'text/event-stream' and r.getheader('Connection') == 'close'
    t0 = time.monotonic(); first = r.read1(100); took = time.monotonic() - t0
    assert first.startswith(b'data: 0') and took < .09; assert b'data: 2' in first + r.read(); conn.close()


def test_body_limits_and_chunked_is_refused(rig):
    r, _ = call(rig, 'POST', keyed(rig, '/api/x'), {'Origin': APP, 'Content-Length': str(1 << 40)})
    assert r.status == 413
    r, _ = call(rig, 'POST', keyed(rig, '/api/x'), {'Origin': APP, 'Transfer-Encoding': 'chunked'})
    assert r.status == 411 and Backend.seen == []


def test_backend_down_is_a_readable_502_and_hello_says_not_ready():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0)); dead = s.getsockname()[1]
    proxy = LocalProxy('127.0.0.1', dead, port=0).bind()
    threading.Thread(target=proxy.server.serve_forever, daemon=True).start()
    r, data = call(proxy, 'GET', keyed(proxy, '/v1/settings'), {'Origin': APP})
    assert r.status == 502 and r.getheader('Access-Control-Allow-Origin') == APP and b'not answering' in data
    assert json.loads(call(proxy, 'GET', HELLO, {'Origin': APP})[1])['ready'] is False
    proxy.stop()


def test_account_is_the_hashed_token_id_and_needs_a_real_token():
    import hashlib
    assert account_of(TOKEN) == hashlib.sha256(b'ab-account1').hexdigest()[:10] and account_of('nonsense') is None and account_of(None) is None


def test_the_door_marks_what_the_page_sends_and_nothing_else(rig):
    call(rig, 'POST', keyed(rig, '/api/stop'), {'Origin': APP, 'X-Roborun-Door': 'forged', 'Sec-Fetch-Site': 'cross-site'}, b'{}')
    sent = Backend.seen[-1]['headers']
    assert sent['x-roborun-door'] == '1' and sent['origin'] == APP and 'sec-fetch-site' not in sent
    call(rig, 'GET', keyed(rig, '/v1/cameras/top.mjpg'), {'X-Roborun-Door': '1'})
    assert 'x-roborun-door' not in Backend.seen[-1]['headers']                      # an <img> carries no Origin and gets no mark


# ---------------------------------------------------------------- this robot's own viewer (no sign-in, this computer only)
def viewed(proxy, path): return f'/v/{proxy.view_key}{path}'


def test_the_viewer_page_is_served_with_a_strict_policy_and_no_cors(rig):
    for path in ('/', '/view', '/view/'):
        r, data = call(rig, 'GET', path)
        assert r.status == 200 and r.getheader('Content-Type').startswith('text/html') and b'Roborun viewer' in data
        csp = r.getheader('Content-Security-Policy')
        assert "script-src 'self'" in csp and "frame-ancestors 'none'" in csp and r.getheader('X-Frame-Options') == 'DENY'
        assert no_cors(r)
    for path, kind in (('/view/viewer.js', 'text/javascript'), ('/view/viewer.css', 'text/css')):
        r, data = call(rig, 'GET', path)
        assert r.status == 200 and r.getheader('Content-Type').startswith(kind) and len(data) > 500
    r, _ = call(rig, 'GET', '/', host='evil.example:8135')
    assert r.status == 421                                    # a DNS-rebinding page gets nothing


def test_only_the_viewer_itself_gets_its_key(rig):
    r, data = call(rig, 'GET', '/view/session', {'Sec-Fetch-Site': 'same-origin'})
    out = json.loads(data)
    assert r.status == 200 and out['key'] == rig.view_key and out['robot'] == 'bench' and no_cors(r)
    assert out['key'] != rig.key                             # not the hosted page's key
    for headers in ({'Origin': 'https://evil.example'}, {'Sec-Fetch-Site': 'cross-site'}, {'Origin': APP},
                    {'Sec-Fetch-Site': 'same-site'}):
        r, data = call(rig, 'GET', '/view/session', headers)
        assert r.status == 403 and rig.view_key.encode() not in data, headers


def test_the_viewer_reads_and_stops_but_changes_nothing(rig):
    own = {'Origin': f'http://127.0.0.1:{rig.port}', 'Sec-Fetch-Site': 'same-origin', 'Content-Type': 'application/json'}
    for path in ('/v1/status', '/v1/tasks?limit=30', '/v1/tasks/abc123', '/v1/tasks/abc123/recording', '/v1/cameras/top.jpg',
                 '/v1/body', '/v1/body/state', '/v1/body/moves/mv-0123456789ab', '/v1/stop/status',
                 '/v1/video-motion/9f3a/recording-9f3a-01/top/12.jpg'):
        r, data = call(rig, 'GET', viewed(rig, path), {'Sec-Fetch-Site': 'same-origin'})
        assert r.status == 200 and json.loads(data)['path'] == path, path
        assert Backend.seen[-1]['path'] == path and 'origin' not in Backend.seen[-1]['headers']
    r, _ = call(rig, 'POST', viewed(rig, '/v1/stop'), own, b'{}')
    assert r.status == 200 and Backend.seen[-1]['method'] == 'POST' and Backend.seen[-1]['path'] == '/v1/stop'
    assert Backend.seen[-1]['headers']['x-roborun-door'] == '1'          # the backend takes a page's Stop only through the door
    before = len(Backend.seen)
    refused = [('POST', '/v1/tasks'), ('POST', '/v1/tasks/abc/stop'), ('POST', '/v1/settings'), ('PUT', '/v1/status'),
               ('POST', '/v1/body/move'), ('GET', '/v1/settings'), ('GET', '/api/state'), ('GET', '/internal/tasks'),
               ('GET', '/v1/video-motion/9f3a/file.bin'), ('GET', '/v1/video-motion/9f3a/history'),
               ('GET', '/v1/robot-profiles'), ('POST', '/v1/robot-profiles/yam/apply'), ('GET', '/v1/tasks/../settings')]
    for method, path in refused:
        r, _ = call(rig, method, viewed(rig, path), own, b'{}' if method != 'GET' else None)
        assert r.status in (403, 404), (method, path, r.status)
    assert len(Backend.seen) == before                       # none of them reached the robot
    r, _ = call(rig, 'GET', '/v/wrong-key/v1/status')
    assert r.status == 404
    r, _ = call(rig, 'POST', viewed(rig, '/v1/stop'), {'Origin': 'https://evil.example', 'Content-Type': 'text/plain'}, b'{}')
    assert r.status == 403 and len(Backend.seen) == before   # another site cannot even press Stop
    r, _ = call(rig, 'GET', viewed(rig, '/v1/status'), {'Origin': APP})
    assert r.status == 403                                   # the hosted page uses its own door, not the viewer's


def test_the_viewer_counts_one_move_as_one_move():
    """The task list said 'Stopped ... · 1 moves' (seen on a fresh install)."""
    import json, shutil, subprocess
    from pathlib import Path
    import roborun_sdk
    node = shutil.which('node')
    if not node:
        pytest.skip('no node to run the viewer\'s JavaScript')
    js = (Path(roborun_sdk.__file__).parent / 'viewer' / 'viewer.js').read_text()
    start = js.index('function describeTask(t)'); end = js.index('\n}\n', start) + 3
    rows = [dict(message='Stopped by you.', moves=1), dict(message='Stopped by you.', moves=3), dict(message='Finished: 9 moves', moves=9)]
    out = subprocess.run([node, '-e', 'const S={humanoid:false};' + js[start:end] +
                          f'console.log(JSON.stringify({json.dumps(rows)}.map(describeTask)))'], capture_output=True, text=True, check=True)
    assert json.loads(out.stdout) == ['Stopped by you. · 1 move', 'Stopped by you. · 3 moves', 'Finished: 9 moves']



def test_the_viewer_shows_why_episodes_were_not_added_only_while_there_are_any():
    import json, shutil, subprocess
    from pathlib import Path
    import roborun_sdk
    node = shutil.which('node')
    if not node:
        pytest.skip('no node to run the viewer\'s JavaScript')
    js = (Path(roborun_sdk.__file__).parent / 'viewer' / 'viewer.js').read_text()
    pick = lambda name: js[js.index(name):js.index('\n}\n', js.index(name)) + 3]
    stub = ('const S={humanoid:false,cams:[]};const rows=[];const dl={replaceChildren(){rows.length=0},append(a,b){rows.push(a.text+": "+b.text)}};'
            'const $=()=>dl;const el=(t,o)=>({text:o&&o.text});function setState(){};let ST;async function api(){return ST}')
    msg = '35 episodes not added to the dataset: the right wrist camera sent no frames (34); no joint readings were recorded (1)'
    run = (stub + pick('function fact(') + pick('function odometry(') + pick('async function refreshStatus(') +
           f'(async()=>{{const out=[];for(const st of [{{ready:true,dataset:{{refused:35,message:{json.dumps(msg)}}}}},{{ready:true}}]){{ST=st;'
           'await refreshStatus();out.push(rows.filter(r=>r.startsWith("Dataset")))}console.log(JSON.stringify(out))})()')
    out = subprocess.run([node, '-e', run], capture_output=True, text=True, check=True)
    assert json.loads(out.stdout) == [['Dataset: ' + msg], []]

def test_the_viewer_page_shows_only_public_routes():
    from pathlib import Path
    import re
    import roborun_sdk
    folder = Path(roborun_sdk.__file__).parent / 'viewer'
    text = ''.join((folder / n).read_text() for n in ('index.html', 'viewer.js', 'viewer.css'))
    assert not re.search(r'/internal/|/api/|\.mp4', text)
    assert '/v1/stop' in text and '/recording' in text and '/v1/cameras/' in text
    assert '[hidden] { display: none !important; }' in (folder / 'viewer.css').read_text()   # a hidden panel stays hidden
