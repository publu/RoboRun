"""What a developer building on Roborun touches: the typed client (roborun_sdk.client), `roborun --version | docs | python`, the
errors the SDK raises, the scaffold's scripts, the examples (robot_client.py, llm_tools.py), the humanoid local API's shared core,
and the docs matching the API."""
import base64
import importlib.util
import json
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

from fake_policy_bits import own_home

from roborun_sdk import cli, devkit, scaffold
from roborun_sdk.client import Robot, RobotError, RobotUnavailable, error_from
from roborun_sdk.errors import ArmLoadError, RoborunError

SDK = Path(__file__).resolve().parents[1]
JPEG = b'\xff\xd8\xff\xe0fakejpeg'


@pytest.fixture(autouse=True)
def _home(tmp_path, monkeypatch):
    """A client of a robot on 127.0.0.1 also looks for this computer's policy runs (policy.control: robot.json, the runtime's
    task records, closing the stale ones): the test's own home, never this computer's."""
    return own_home(tmp_path, monkeypatch)


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ------------------------------------------------------------------------------------------------ a stand-in local API
class FakeApi(BaseHTTPRequestHandler):
    """The arm local API as the client uses it (shapes from the runtime's api.py / public_robot.py)."""
    tasks, seen, busy, frames = {}, [], False, {}
    string_errors = False

    def log_message(self, *a):
        pass

    def _json(self, code, obj):
        data = json.dumps(obj).encode()
        self.send_response(code); self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)

    def _error(self, code, err, message, retryable=False):
        self._json(code, {'error': message} if self.string_errors else {'error': {'code': err, 'message': message, 'retryable': retryable}})

    def do_GET(self):
        FakeApi.seen.append(('GET', self.path))
        path = self.path.partition('?')[0]
        if path == '/v1/status':
            return self._json(200, {'ready': True, 'profile': 'yam', 'arms': {'left': {'ok': True}}, 'cameras': {'top': {'fresh': True}}})
        if path == '/v1/robot':
            return self._json(200, {'arms': {'left': {'ok': True, 'joints_rad': [0.0] * 6, 'gripper': 0.95, 'tool_mm': [300, 0, 200], 'x': 1}}})
        if path == '/v1/stop/status':
            return self._json(200, {'state': 'stopped', 'confirmed': True, 'arms': {}, 'task': None, 'message': 'still'})
        m = re.fullmatch(r'/v1/cameras/(\w+)\.jpg', path)
        if m:
            if m.group(1) != 'top':
                return self._error(503, 'unavailable', f'No fresh frame from the {m.group(1)} camera', True)
            data = JPEG + (b'sharp' if 'blur=0' in self.path else b'blurred')
            self.send_response(200); self.send_header('Content-Type', 'image/jpeg'); self.send_header('Content-Length', str(len(data)))
            self.end_headers(); self.wfile.write(data); return
        parts = path.split('/')
        task = self.tasks.get(parts[3]) if len(parts) > 3 else None
        if task is None:
            return self._error(404, 'not_found', 'no such task')
        if path.endswith('/live'):
            self.send_response(200); self.send_header('Content-Type', 'text/event-stream'); self.end_headers()
            for i, status in enumerate(('running', 'running', 'done')):
                view = dict(task, status=status, moves=i, elapsed_s=float(i), message=['Working', 'Moving', 'Finished: 2 moves'][i])
                self.wfile.write(b'data: ' + json.dumps(view).encode() + b'\n\n'); self.wfile.flush()
            task.update(status='done', moves=2, message='Finished: 2 moves', outcome=PICKUP_OUTCOME)
            return
        return self._json(200, task)

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers.get('Content-Length') or 0)) or b'{}')
        FakeApi.seen.append(('POST', self.path, body))
        if self.path == '/v1/tasks':
            if FakeApi.busy:
                return self._error(409, 'busy', 'task x is still running; stop it first', True)
            if not body.get('prompt'):
                return self._error(400, 'invalid', 'prompt is required')
            task = dict(id='t1', status='running', mode=body.get('mode', 'run'), prompt=body['prompt'], moves=0, elapsed_s=0.0,
                        message='Working', error=None, recording_url=None)
            self.tasks['t1'] = task
            return self._json(202, task)
        if self.path == '/v1/stop':
            return self._json(200, {'state': 'stopped', 'confirmed': True, 'stopped_tasks': [], 'arms': {'left': 'holding'}, 'message': 'ok'})
        if self.path.endswith('/stop'):
            task = self.tasks[self.path.split('/')[3]]
            task.update(status='stopped')
            return self._json(200, task)
        self._error(404, 'not_found', 'no such route')


@pytest.fixture
def api():
    FakeApi.tasks, FakeApi.seen, FakeApi.busy, FakeApi.string_errors = {}, [], False, False
    srv = ThreadingHTTPServer(('127.0.0.1', 0), FakeApi)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f'http://127.0.0.1:{srv.server_address[1]}'
    srv.shutdown(); srv.server_close()


def free_port():
    import socket
    s = socket.socket(); s.bind(('127.0.0.1', 0)); port = s.getsockname()[1]; s.close()
    return port


# ------------------------------------------------------------------------------------------------ the typed client
def test_the_client_runs_a_task_to_its_end(api):
    robot = Robot(api)
    seen = []
    task = robot.run('pick up the cup', mode='preview', on_update=lambda v: seen.append(v['status']))
    assert task['status'] == 'done' and task['moves'] == 2 and seen[-1] == 'done' and 'running' in seen
    assert ('POST', '/v1/tasks', {'prompt': 'pick up the cup', 'mode': 'preview'}) in FakeApi.seen
    assert robot.status()['profile'] == 'yam' and robot.state()['arms']['left']['gripper'] == 0.95


def test_the_client_asks_for_a_wrist_camera_only_when_told(api):
    robot = Robot(api)
    robot.run('pick up the box', camera='left')
    assert ('POST', '/v1/tasks', {'prompt': 'pick up the box', 'mode': 'run', 'camera': 'left'}) in FakeApi.seen
    from roborun_sdk import client
    assert client.Task.__annotations__['camera']


def test_the_client_sends_marks_only_when_given(api):
    robot = Robot(api)
    box = {'target': [182, 206, 219, 250], 'place': [408, 200, 522, 310]}
    robot.run('pick up the alphabet soup and place it in the basket', marks=box)
    assert ('POST', '/v1/tasks', {'prompt': 'pick up the alphabet soup and place it in the basket', 'mode': 'run', 'marks': box}) in FakeApi.seen
    robot.run('pick up the cup')
    assert ('POST', '/v1/tasks', {'prompt': 'pick up the cup', 'mode': 'run'}) in FakeApi.seen
    from roborun_sdk import client
    assert client.Task.__annotations__['marks'] and {'target', 'place'} <= set(client.Marks.__annotations__)


def test_the_task_type_names_its_outcome():
    from roborun_sdk import client
    assert client.Task.__annotations__['outcome'] and set(client.Outcome.__annotations__) == {'holding', 'placed', 'verified_by'}
    assert set(client.HeldObject.__annotations__) == set(PICKUP_OUTCOME['holding']['left'])
    assert {'Outcome', 'HeldObject', 'Placed'} <= set(client.__all__)


def test_the_client_stop_frame_and_stop_status(api):
    robot = Robot(api)
    assert robot.stop() == {'state': 'stopped', 'confirmed': True, 'stopped_tasks': [], 'arms': {'left': 'holding'}, 'message': 'ok'}
    assert robot.stop_status()['confirmed'] is True
    assert robot.frame('top') == JPEG + b'blurred' and robot.frame('top', blur=False) == JPEG + b'sharp'
    with pytest.raises(RobotError) as exc:
        robot.frame('left')
    assert (exc.value.status, exc.value.code, exc.value.retryable) == (503, 'unavailable', True)


def test_the_client_errors_are_typed_whatever_their_shape(api):
    robot = Robot(api)
    FakeApi.busy = True
    with pytest.raises(RobotError) as exc:
        robot.start('pick up the cup')
    assert (exc.value.status, exc.value.code, exc.value.retryable) == (409, 'busy', True) and isinstance(exc.value, RoborunError)
    FakeApi.busy, FakeApi.string_errors = False, True
    with pytest.raises(RobotError) as exc:
        robot.start('')
    assert exc.value.status == 400 and exc.value.message == 'prompt is required'
    assert error_from(502, b'<html>Bad Gateway</html>').retryable and error_from(502, b'<html>Bad Gateway</html>').message.startswith('<html>')


def test_nothing_running_is_robot_unavailable_not_a_traceback():
    robot = Robot(f'http://127.0.0.1:{free_port()}', timeout=2)
    with pytest.raises(RobotUnavailable) as exc:
        robot.stop()
    assert exc.value.status == 0 and exc.value.code == 'not_running' and 'roborun up' in exc.value.message
    assert isinstance(exc.value, RobotError)


def test_the_package_exports_the_public_names():
    import roborun_sdk
    assert roborun_sdk.Robot is Robot and roborun_sdk.RobotError is RobotError and roborun_sdk.RoborunError is RoborunError
    assert {'Robot', 'RobotError', 'RobotUnavailable', 'RoborunError'} <= set(roborun_sdk.__all__)
    assert (SDK / 'src' / 'roborun_sdk' / 'py.typed').is_file()
    with pytest.raises(AttributeError):
        roborun_sdk.NoSuchThing


# ------------------------------------------------------------------------------------------------ errors a program can catch
def test_a_driver_that_cannot_load_is_catchable_and_still_a_cli_exit(tmp_path):
    from roborun_sdk.arms import open_arm
    for ref in (f'{tmp_path / "missing.py"}:MyArm', 'no_such_arm'):
        try:
            open_arm(ref, 'left', connect=False)
        except Exception as exc:                                  # a program's own `except Exception` catches it
            assert isinstance(exc, ArmLoadError) and isinstance(exc, RoborunError) and isinstance(exc, SystemExit)
        else:
            raise AssertionError(ref)


# ------------------------------------------------------------------------------------------------ roborun --version | docs | python
def test_version_names_the_sdk_the_build_and_the_runtime(tmp_path, monkeypatch, capsys):
    prefix = tmp_path / 'roborun' / 'python'; prefix.mkdir(parents=True)
    (tmp_path / 'roborun' / 'build.json').write_text(json.dumps({'build': '20261119-172831-b6abbc5e80'}))
    runtime = tmp_path / 'runtime'; runtime.mkdir()
    (runtime / 'RUNTIME.json').write_text(json.dumps({'version': '2026.11.19+c18915e.092756'}))
    monkeypatch.setattr(sys, 'prefix', str(prefix))
    monkeypatch.setenv('MANIFEST_DREAMAPI', str(runtime))
    with pytest.raises(SystemExit) as done:
        cli.main(['--version'])
    out = capsys.readouterr().out
    assert done.value.code == 0
    assert 'roborun-sdk 0.1.0 (build 20261119-172831-b6abbc5e80)' in out and 'robot runtime 2026.11.19+c18915e.092756' in out
    monkeypatch.setenv('MANIFEST_DREAMAPI', str(tmp_path / 'none'))
    assert devkit.version_info()['runtime'] is None and 'not installed yet' in devkit.version_text()


def test_docs_lists_prints_and_copies_the_examples(tmp_path, capsys):
    assert cli.main(['docs']) == 0
    out = capsys.readouterr().out
    assert 'BUILDING.md' in out and 'HOSTED_API.md' in out and '127.0.0.1:8120/v1/docs' in out
    assert cli.main(['docs', 'building']) == 0 and capsys.readouterr().out.startswith('# Building on Roborun')
    assert cli.main(['docs', '--examples', str(tmp_path / 'ex')]) == 0
    copied = {p.name for p in (tmp_path / 'ex').iterdir()}
    assert {'robot_client.py', 'llm_tools.py', 'minimal_arm.py', 'README.md'} <= copied and not any(n.endswith('.json') for n in copied)


def test_roborun_python_runs_roborun_s_own_interpreter_with_the_arguments_as_given(monkeypatch):
    seen = []
    monkeypatch.setattr(devkit.os, 'execv', lambda exe, argv: seen.append((exe, argv)))
    cli.main(['python', '-c', 'print(1)', '--flag'])
    assert seen == [(sys.executable, [sys.executable, '-c', 'print(1)', '--flag'])]


def test_roborun_python_really_imports_the_sdk(tmp_path):
    import subprocess
    script = tmp_path / 'mine.py'
    script.write_text('import roborun_sdk.client, sys; print("ok", sys.argv[1:])')
    env = dict(__import__('os').environ, PYTHONPATH=str(SDK / 'src'))
    out = subprocess.run([sys.executable, '-m', 'roborun_sdk.cli', 'python', str(script), 'a', '-b'], capture_output=True, text=True,
                         env=env, timeout=60)
    assert out.returncode == 0 and out.stdout.strip() == "ok ['a', '-b']", out.stderr


# ------------------------------------------------------------------------------------------------ the scaffold
def test_the_scaffold_says_how_to_run_check_with_roborun_s_python(tmp_path, capsys):
    assert cli.main(['new', 'bot', '--dir', str(tmp_path)]) == 0
    assert 'roborun python check.py' in capsys.readouterr().out
    readme = (tmp_path / 'bot' / 'README.md').read_text()
    assert 'roborun python check.py' in readme and '\npython check.py' not in readme and '/v1/stop' in readme


def test_run_task_says_roborun_is_not_running_instead_of_a_traceback(tmp_path, monkeypatch, capsys):
    project = scaffold.create('bot', tmp_path)
    client = load(project / 'run_task.py', 'generated_run_task_down')
    monkeypatch.setattr('sys.argv', ['run_task.py', 'pick up the cup', '--api', f'http://127.0.0.1:{free_port()}'])
    with pytest.raises(SystemExit) as done:
        client.main()
    assert done.value.code == 3 and 'start it with roborun up' in capsys.readouterr().err


def test_run_task_reads_both_error_shapes(api, tmp_path, monkeypatch, capsys):
    project = scaffold.create('bot', tmp_path)
    client = load(project / 'run_task.py', 'generated_run_task_errors')
    FakeApi.busy = True
    monkeypatch.setattr('sys.argv', ['run_task.py', 'pick up the cup', '--api', api])
    assert client.main() == 2 and 'busy: task x is still running' in capsys.readouterr().err
    FakeApi.busy, FakeApi.string_errors = False, True
    monkeypatch.setattr('sys.argv', ['run_task.py', '', '--api', api])
    assert client.main() == 2 and 'prompt is required' in capsys.readouterr().err


def test_check_reports_what_the_rules_returned_even_on_a_slow_computer(tmp_path, capsys):
    project = scaffold.create('bot', tmp_path)
    check = load(project / 'check.py', 'generated_check_slow')
    check.time = type('T', (), {'sleep': staticmethod(lambda s: time.sleep(max(s, 2.0)))})   # long enough for the hold to settle
    assert check.main() == 0
    assert 'target after your rules: [2.0' in capsys.readouterr().out


# ------------------------------------------------------------------------------------------------ the examples
EXAMPLES = SDK / 'examples'
# GET /v1/tasks/{id}'s outcome after a simulated 'pick up the cup' (yam_single, scene cup): the jaw read 0.947
PICKUP_OUTCOME = {'holding': {'left': {'object': 'cup', 'lifted_mm': 90, 'evidence': 'grip+lift', 'verified': True}, 'right': 'unknown'},
                  'placed': [], 'verified_by': ['grip', 'lift']}


def test_robot_client_example_drives_the_robot(api, capsys):
    ex = load(EXAMPLES / 'robot_client.py', 'example_robot_client_run')
    assert ex.main(['run', 'pick up the cup', '--preview'], robot=Robot(api)) == 0
    out = capsys.readouterr().out
    assert 'started t1 running' in out and '"status": "done"' in out
    assert ex.main(['stop'], robot=Robot(api)) == 0 and '"confirmed": true' in capsys.readouterr().out
    assert ex.main(['status'], robot=Robot(f'http://127.0.0.1:{free_port()}', timeout=2)) == 3


def test_llm_tools_are_valid_specs_and_every_call_answers(api):
    ex = load(EXAMPLES / 'llm_tools.py', 'example_llm_tools')
    names = [t['name'] for t in ex.TOOLS]
    assert names == ['robot_status', 'look', 'arm_state', 'run_task', 'stop'] and len(set(names)) == len(names)
    for tool in ex.TOOLS:
        assert tool['description'] and tool['input_schema']['type'] == 'object'
        json.dumps(tool)
    robot = Robot(api)
    assert ex.call_tool('robot_status', robot=robot)['ready'] is True
    image = ex.call_tool('look', {'camera': 'top'}, robot=robot)
    assert image['type'] == 'image' and image['media_type'] == 'image/jpeg' and base64.b64decode(image['data']) == JPEG + b'blurred'
    assert ex.call_tool('arm_state', robot=robot) == {'arms': {'left': {'ok': True, 'joints_rad': [0.0] * 6, 'gripper': 0.95, 'tool_mm': [300, 0, 200]}}}
    done = ex.call_tool('run_task', {'prompt': 'pick up the cup'}, robot=robot)
    assert done['status'] == 'done' and done['moves'] == 2
    assert done['outcome'] == PICKUP_OUTCOME and done['outcome']['holding']['left']['object'] == 'cup'   # not the gripper value
    assert 'outcome' in next(t for t in ex.TOOLS if t['name'] == 'run_task')['description']
    assert ex.call_tool('stop', robot=robot)['confirmed'] is True
    assert ex.call_tool('look', {'camera': 'left'}, robot=robot) == {'error': {'code': 'unavailable', 'message': 'No fresh frame from the left camera', 'retryable': True}}
    assert ex.call_tool('run_task', {}, robot=robot)['error']['code'] == 'invalid'
    for camera in ('head', 'front', ''):                     # not in the schema: refused before any frame is fetched
        refused = ex.call_tool('look', {'camera': camera}, robot=robot)
        assert refused['error']['code'] == 'invalid' and refused['error']['retryable'] is False, camera
    assert ex.call_tool('fly', robot=robot)['error']['code'] == 'unknown_tool'
    down = ex.call_tool('stop', robot=Robot(f'http://127.0.0.1:{free_port()}', timeout=2))
    assert down['error']['code'] == 'not_running' and down['error']['retryable'] is True


# ------------------------------------------------------------------------------------------------ the humanoid local API's shared core
@pytest.fixture
def body_api():
    from roborun_sdk.body.server import BodyRobot, serve

    class Base:
        def __init__(self): self.v = dict(x=0., y=0., yaw=0., vx=0., vy=0., vyaw=0.); self.stops = 0
        def state(self): return dict(self.v)
        def stop(self): self.stops += 1

    class Head:
        def state(self): return dict(pitch=0., yaw=0.)
        def frame(self): return dict(jpeg=JPEG)

    base = Base()
    robot = BodyRobot('booster_k1', base, Head(), None, robot='t', simulated='room')
    srv = serve(robot, port=0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield Robot(f'http://127.0.0.1:{srv.server_address[1]}'), base, robot
    srv.shutdown(); srv.server_close()


def test_a_humanoid_offers_the_same_core_as_an_arm_robot(body_api):
    client, base, robot = body_api
    assert client.health()['ok'] is True and client.health()['profile'] == 'booster_k1'
    assert client.stop_status()['state'] == 'stopped'
    base.v['vx'] = .3
    assert client.stop_status() == dict(state='stopping', confirmed=False, task=None, base_speed_m_s=.3, base_turn_rad_s=0.,
                                        message='The base is braking')
    stop = client.stop()
    assert stop['stopped'] is True and stop['state'] == 'stopping' and base.stops == 1
    assert client.call('GET', '/api/stop/status')['state'] == 'stopping'
    spec = client.call('GET', '/v1/openapi.json')
    assert {'/body/move', '/stop', '/stop/status', '/health'} <= set(spec['paths'])
    assert client.frame('head') == JPEG and client.state()['base']['vx'] == .3


def test_a_humanoid_s_errors_carry_retryable_and_refuse_unknown_fields(body_api):
    client, base, robot = body_api
    with pytest.raises(RobotError) as exc:
        client.call('POST', '/v1/body/move', {'prompt': 'walk', 'speed': 9})
    assert exc.value.status == 400 and exc.value.code == 'invalid' and 'unknown fields: speed' in exc.value.message
    with pytest.raises(RobotError) as exc:
        client.body_move('walk')                                       # this robot has no motion service key
    assert exc.value.code == 'no_motion_service' and exc.value.retryable is False
    with pytest.raises(RobotError) as exc:
        client.call('DELETE', '/v1/tasks')
    assert exc.value.status == 405 and exc.value.code == 'method_not_allowed'
    with pytest.raises(RobotError) as exc:
        client.call('GET', '/v1/nope')
    assert exc.value.status == 404 and exc.value.retryable is False and 'see /v1/docs' in exc.value.message


# ------------------------------------------------------------------------------------------------ what ships


def test_an_installed_package_finds_its_bundled_docs(tmp_path, monkeypatch):
    fake = tmp_path / 'roborun_sdk'; (fake / '_docs' / 'docs').mkdir(parents=True)
    (fake / '_docs' / 'docs' / 'BUILDING.md').write_text('# Building on Roborun\n')
    monkeypatch.setattr(devkit, '__file__', str(fake / 'devkit.py'))
    assert devkit.docs_root() == (fake / '_docs').resolve()


# ------------------------------------------------------------------------------------------------ the docs match the API
README = (SDK / 'README.md').read_text()
BUILDING = (SDK / 'docs' / 'BUILDING.md').read_text()
HOSTED = (SDK / 'docs' / 'HOSTED_API.md').read_text()
AGENTS = (SDK / 'docs' / 'AGENTS.md').read_text()


def test_the_docs_name_the_model_the_api_accepts():
    assert '`manifest`, the default' not in README and 'manifest-1-beta' in README
    assert '"model": "manifest-1-beta"' in README                   # the sample 202 answer has every field the API sends


def test_the_docs_use_the_routes_every_profile_serves():
    for text in (README, BUILDING):
        assert 'POST /v1/stop' in text and '/v1/stop/status' in text
    for route in ('/v1/cameras/{role}.jpg', '/v1/robot', 'retryable', 'roborun python', 'roborun docs', 'llm_tools.py'):
        assert route in BUILDING, route
    assert 'roborun_sdk.client' in BUILDING and 'roborun login' in README


def test_the_example_links_resolve():
    for name, text in (('README', README), ('BUILDING', BUILDING)):
        base = SDK if name == 'README' else SDK / 'docs'
        for link in re.findall(r'\]\(([^)#:]+)\)', text):
            assert (base / link).exists(), (name, link)


def test_users_get_positions_only():
    from roborun_sdk import client
    assert 'clip_url' not in client.Task.__annotations__
    with pytest.raises(RobotError) as removed:                     # Robot.clip() is only a stub saying it was removed
        Robot('http://127.0.0.1:9').clip('t')
    assert removed.value.code == 'removed' and not re.search(r'/clip\b|clip_url', (SDK / 'src' / 'roborun_sdk' / 'client.py').read_text())
    for name, text in (('README', README), ('BUILDING', BUILDING), ('HOSTED_API', HOSTED), ('examples', (EXAMPLES / 'README.md').read_text()),
                       ('CHANGELOG', (SDK / 'CHANGELOG.md').read_text())):
        assert not re.search(r'/clip\b|clip_url|\.mp4', text), name
    for path in sorted(EXAMPLES.glob('*.py')):
        assert not re.search(r'/clip\b|clip_url|\.clip\(|\.mp4', path.read_text()), path.name
