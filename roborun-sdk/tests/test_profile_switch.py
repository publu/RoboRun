"""Switching the robot's profile from the hosted page: the local connector runs the robot's own switch and reports it."""
import http.client
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from roborun_sdk.local_proxy import HELLO, LocalProxy

APP = 'https://app.roborun.app'


def backend(task=None):
    class B(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def do_GET(self):
            body = {'/v1/robot-profiles': dict(active_id='yam_single', profiles=[dict(id='yam_single'), dict(id='yam')], profiles_set_by='sdk'),
                    '/v1/status': dict(ready=True, task=task)}.get(self.path.split('?')[0], dict(ok=True))
            data = json.dumps(body).encode(); self.send_response(200); self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)
        def do_POST(self):
            self.send_response(409); self.send_header('Content-Length', '0'); self.end_headers()
    srv = ThreadingHTTPServer(('127.0.0.1', 0), B); threading.Thread(target=srv.serve_forever, daemon=True).start(); return srv


def door(task=None, command=None, tmp=None):
    b = backend(task); p = LocalProxy('127.0.0.1', b.server_address[1], port=0, robot='r'); p.switch_command = command; p.bind()
    threading.Thread(target=p.server.serve_forever, daemon=True).start(); return p


def call(p, method, path, body=None, origin=APP):
    c = http.client.HTTPConnection('127.0.0.1', p.port, timeout=10)
    h = {'Host': f'127.0.0.1:{p.port}', 'Origin': origin}
    if body is not None: h['Content-Type'] = 'application/json'
    c.request(method, f'/k/{p.key}{path}', body=json.dumps(body) if body is not None else None, headers=h); r = c.getresponse()
    out = r.status, json.loads(r.read() or b'{}'); c.close(); return out


def home(tmp_path, monkeypatch, **config):
    monkeypatch.setenv('ROBORUN_HOME', str(tmp_path)); (tmp_path / 'robot.json').write_text(json.dumps(config))


def test_the_page_switches_the_profile_on_the_robot(tmp_path, monkeypatch):
    home(tmp_path, monkeypatch, sim='cup'); mark = tmp_path / 'ran'
    p = door(command=[sys.executable, '-c', f"open({str(mark)!r}, 'w').write('yam')"])
    code, out = call(p, 'POST', '/v1/robot-profiles/yam/apply', dict(confirm_profile_id='yam'))
    assert code == 202 and out['switch_job']['state'] == 'running' and out['configured_id'] == 'yam'
    for _ in range(50):
        if p.switch_job['state'] != 'running': break
        time.sleep(.1)
    assert p.switch_job['state'] == 'done' and mark.read_text() == 'yam'
    code, out = call(p, 'GET', '/v1/robot-profiles')
    assert code == 200 and out['switch_job']['state'] == 'done' and out['active_id'] == 'yam_single'


@pytest.mark.parametrize('profile,task,want', [('nope', None, 404), ('yam_single', None, 409), ('yam', dict(id='t1'), 409)])
def test_refused_switches_say_why_and_run_nothing(tmp_path, monkeypatch, profile, task, want):
    home(tmp_path, monkeypatch, sim='cup'); mark = tmp_path / 'ran'
    p = door(task=task, command=[sys.executable, '-c', f"open({str(mark)!r}, 'w').write('x')"])
    code, out = call(p, 'POST', f'/v1/robot-profiles/{profile}/apply', {})
    assert code == want and out['error']['message'] and not mark.exists() and p.switch_job is None


def test_only_the_allowed_page_can_switch(tmp_path, monkeypatch):
    home(tmp_path, monkeypatch, sim='cup'); mark = tmp_path / 'ran'
    p = door(command=[sys.executable, '-c', f"open({str(mark)!r}, 'w').write('x')"])
    code, _ = call(p, 'POST', '/v1/robot-profiles/yam/apply', {}, origin='https://evil.example')
    assert code == 403 and not mark.exists()


def test_real_arms_that_are_not_plugged_in_refuse_the_switch_and_no_simulation_starts(tmp_path, monkeypatch):
    home(tmp_path, monkeypatch, hardware=dict(left=dict(arm='yam', bus='can7'))); mark = tmp_path / 'ran'
    p = door(command=[sys.executable, '-c', f"open({str(mark)!r}, 'w').write('x')"])
    code, out = call(p, 'POST', '/v1/robot-profiles/yam/apply', {})
    assert code == 409 and 'No arms connected' in out['error']['message'] and 'can7' in out['error']['message'] and not mark.exists()
    code, out = call(p, 'GET', '/v1/robot-profiles')
    assert out['hardware']['simulated'] is False and out['hardware']['configured']['left']['connected'] is False


def test_nothing_set_up_and_no_simulation_refuses(tmp_path, monkeypatch):
    home(tmp_path, monkeypatch); p = door(command=[sys.executable, '-c', 'pass'])
    code, out = call(p, 'POST', '/v1/robot-profiles/yam/apply', {})
    assert code == 409 and 'roborun setup' in out['error']['message']


def test_plugged_in_arms_are_used(tmp_path, monkeypatch):  # the profile that fits them
    dev = tmp_path / 'ttyACM0'; dev.write_text(''); mark = tmp_path / 'ran'
    home(tmp_path, monkeypatch, hardware=dict(left=dict(arm='so101', bus=str(dev))))
    p = door(command=[sys.executable, '-c', f"open({str(mark)!r}, 'w').write('x')"])
    assert call(p, 'GET', '/v1/robot-profiles')[1]['hardware']['arms_ready'] is True
    code, out = call(p, 'POST', '/v1/robot-profiles/so101/apply', {})
    assert code == 202


def test_use_connected_arms_leaves_the_simulation_only_when_set_up_arms_are_plugged_in(tmp_path, monkeypatch):
    dev = tmp_path / 'ttyACM0'; mark = tmp_path / 'ran'
    home(tmp_path, monkeypatch, sim='cup', hardware=dict(left=dict(arm='so101', bus=str(dev))))
    p = door(command=[sys.executable, '-c', f"open({str(mark)!r}, 'w').write('x')"])
    code, out = call(p, 'POST', '/v1/robot-profiles/use-connected-arms', {})
    assert code == 409 and 'not connected' in out['error']['message']
    dev.write_text(''); p._hw = (0., None)
    code, out = call(p, 'POST', '/v1/robot-profiles/use-connected-arms', {})
    assert code == 202 and 'connected arms' in out['switch_job']['message']


def test_use_connected_arms_with_nothing_plugged_in_says_so(tmp_path, monkeypatch):
    from roborun_sdk.arms import connect
    monkeypatch.setattr(connect, 'adapter_rows', lambda *a, **k: ([], None))    # nothing plugged in, whatever this computer has
    home(tmp_path, monkeypatch, sim='cup'); p = door(command=[sys.executable, '-c', 'pass'])
    code, out = call(p, 'POST', '/v1/robot-profiles/use-connected-arms', {})
    assert code == 409 and out['error']['message'] == 'No arms connected'


def test_defaults_are_matched_sets_and_the_rest_is_custom(tmp_path, monkeypatch):
    (tmp_path / 'inst' / 'profiles' / 'yam_metal').mkdir(parents=True); (tmp_path / 'inst' / 'profiles' / 'yam_metal' / 'rig.json').write_text('{}')
    home(tmp_path, monkeypatch, sim='cup', manifest=str(tmp_path / 'inst')); p = door(command=[sys.executable, '-c', 'pass'])
    cat = call(p, 'GET', '/v1/robot-profiles')[1]['catalog']
    yam = next(d for d in cat['defaults'] if d['arm'] == 'yam')
    assert (yam['one'], yam['two']) == ('yam_solo', 'yam')
    assert next(d for d in cat['defaults'] if d['arm'] == 'so101')['two'] == 'so101_dual'
    assert {c['id'] for c in cat['custom']} == {'yam_metal', 'yam_single'}           # yam_single: what this robot runs now


def test_a_custom_pair_is_put_together_and_switched_to(tmp_path, monkeypatch):
    home(tmp_path, monkeypatch, sim='cup'); mark = tmp_path / 'ran'
    p = door(command=[sys.executable, '-c', f"open({str(mark)!r}, 'w').write('x')"])
    code, out = call(p, 'POST', '/v1/robot-profiles/compose', dict(left='yam', right='maker_arm'))
    assert code == 202 and out['profile_id'] == 'yam+maker_arm'
    assert call(p, 'POST', '/v1/robot-profiles/compose', dict(left='no_such_arm'))[0] == 400


def test_build_a_pair_refusals_are_worded_for_the_page(tmp_path, monkeypatch):
    home(tmp_path, monkeypatch, sim='cup'); p = door(command=[sys.executable, '-c', 'pass'])
    said = lambda body: call(p, 'POST', '/v1/robot-profiles/compose', body)
    for body, words in ((dict(left='', right=None), 'Choose at least one arm.'), (dict(left='  ', right=' '), 'Choose at least one arm.'),
                        (dict(left='no_such_arm'), "There is no arm called 'no_such_arm'. Choose one from the list."),
                        (dict(left='bracketbot'), 'BracketBot has no 3D model yet, so it cannot run here. Choose another arm.'),
                        (dict(left='mine.py:Arm'), 'An arm with its own driver needs a profile made on the robot first; it then appears under Custom.'),
                        ([1, 2], 'Choose a left arm, a right arm, or both.')):
        code, out = said(body)
        assert code == 400 and out['error']['message'] == words
        assert 'roborun' not in words and '--' not in words


def test_real_arms_cannot_be_switched_to_other_arms(tmp_path, monkeypatch):
    dev = tmp_path / 'ttyACM0'; dev.write_text('')
    home(tmp_path, monkeypatch, hardware=dict(left=dict(arm='so101', bus=str(dev))))
    p = door(command=[sys.executable, '-c', 'pass'])
    code, out = call(p, 'POST', '/v1/robot-profiles/yam/apply', {})
    assert code == 409 and out['error']['code'] == 'other_arms'


def test_a_pair_named_by_the_page_with_an_encoded_plus_is_switched_to(tmp_path, monkeypatch):
    # the page sends encodeURIComponent('yam+maker_arm'): the '+' arrives as %2B and must still be this connector's switch, not
    # the backend's (which answers "Robot profile action not found")
    home(tmp_path, monkeypatch, sim='cup'); mark = tmp_path / 'ran'
    p = door(command=[sys.executable, '-c', f"open({str(mark)!r}, 'w').write('x')"])
    code, out = call(p, 'POST', '/v1/robot-profiles/yam%2Bmaker_arm/apply', dict(confirm_profile_id='yam+maker_arm'))
    assert code == 202 and out['configured_id'] == 'yam+maker_arm' and out['switch_job']['profile_id'] == 'yam+maker_arm'
    assert call(p, 'POST', '/v1/robot-profiles/yam%2bmaker_arm/apply', {})[0] == 409     # the switch above is still running


def test_a_switch_that_fails_says_why_on_the_next_read(tmp_path, monkeypatch):
    home(tmp_path, monkeypatch, sim='cup')
    p = door(command=[sys.executable, '-c', "import sys; print('no model for so101 in this scene'); sys.exit(3)"])
    assert call(p, 'POST', '/v1/robot-profiles/so101/apply', {})[0] == 202
    for _ in range(100):
        if p.switch_job['state'] != 'running': break
        time.sleep(.05)
    job = call(p, 'GET', '/v1/robot-profiles')[1]['switch_job']
    assert job['state'] == 'failed' and job['exit_code'] == 3 and job['message'] == 'no model for so101 in this scene'
