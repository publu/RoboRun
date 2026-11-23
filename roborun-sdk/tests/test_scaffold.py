"""roborun new: the generated project imports, its driver and rules load through Roborun's own paths, and its task client
works against the local API's task routes (a stand-in server here; the real robot in docs/BUILDING.md)."""
import importlib.util
import json
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest
from pathlib import Path

from roborun_sdk import cli, scaffold
from roborun_sdk.arms import open_arm
from roborun_sdk.arms.base import StreamedArm
from roborun_sdk.embodiment import ArmAdapter


@pytest.fixture
def project(tmp_path):
    return scaffold.create('my-robot', tmp_path)


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_command_writes_the_project(tmp_path, capsys):
    assert cli.main(['new', 'bot', '--dir', str(tmp_path)]) == 0
    assert sorted(str(p.relative_to(tmp_path / 'bot')) for p in (tmp_path / 'bot').rglob('*') if p.is_file()) == sorted(scaffold.FILES)
    out = capsys.readouterr().out
    assert 'created' in out and f'pinned: {scaffold.sdk_requirement()}' in out
    with pytest.raises(SystemExit, match='not empty'):
        cli.main(['new', 'bot', '--dir', str(tmp_path)])
    assert cli.main(['new', 'bot', '--dir', str(tmp_path), '--force']) == 0
    with pytest.raises(SystemExit, match='letters'):
        scaffold.create('../escape', tmp_path)


def test_every_generated_python_file_imports(project):
    for file in ('my_arm.py', 'my_rules.py', 'run_task.py', 'check.py'):
        load(project / file, f'generated_{file[:-3]}')
    assert '{project}' not in (project / 'robot.yaml').read_text() and str(project) in (project / 'robot.yaml').read_text()


def test_the_driver_loads_through_the_own_driver_path_and_moves_its_software_motors(project):
    arm = open_arm(f'{project / "my_arm.py"}:MyArm', 'left', 'serial:///dev/ttyUSB9', rules=str(project / 'my_rules.py'))
    try:
        assert isinstance(arm, StreamedArm) and isinstance(arm, ArmAdapter) and arm.bus == 'serial:///dev/ttyUSB9'
        start = arm.read()['positions_rad']
        target = list(start); target[0] = 3.0; target[1] = start[1] + 0.05
        arm.command(target, 0.5)
        assert arm.held_target()[0] == pytest.approx(2.0)          # my_rules.py: the base keep-out trims the command
        end = time.monotonic() + 3
        while abs(arm.read()['positions_rad'][1] - target[1]) > 1e-3 and time.monotonic() < end:
            time.sleep(0.02)
        assert arm.read()['positions_rad'][1] == pytest.approx(target[1], abs=1e-3)
        assert arm.health()['ok'] and arm.rule_refusal() is None
    finally:
        arm.close()


def test_the_rules_load_and_refuse_below_the_table(project):
    from roborun_sdk.arms.base import _load_rule
    check = _load_rule(str(project / 'my_rules.py'))
    q = [0.0, -0.5, 1.0, 0.0, 0.5, 0.0]
    assert check('left', q, 1.0, dict(tool_position_m=[0.3, 0.0, -0.05])) is False
    assert check('left', [-2.5] + q[1:], 1.0, dict(tool_position_m=None)) == ([-2.0] + q[1:], 1.0)


def test_the_driver_runs_on_the_station(project):
    pytest.importorskip('roborun_station')
    from roborun_sdk.embodiment import run_station
    arm = open_arm(f'{project / "my_arm.py"}:MyArm', 'left')
    station, srv = run_station(arm, port=0)
    try:
        with urllib.request.urlopen(srv.url + '/supervisor/pose', timeout=3) as r:
            assert json.load(r)
        assert station.left.q_cmd == pytest.approx(arm.home)
    finally:
        srv.close(); arm.close()


def test_the_robot_yaml_lines_resolve_to_the_driver_and_the_rules(project, tmp_path, monkeypatch):
    from roborun_sdk import robot
    from roborun_sdk.arms import settings
    monkeypatch.setattr(robot, 'HOME', tmp_path / 'settings')
    (tmp_path / 'settings').mkdir()
    (tmp_path / 'settings' / 'robot.yaml').write_text((project / 'robot.yaml').read_text())
    hw = settings.to_hardware(settings.load())
    assert hw['left']['arm'] == f'{project / "my_arm.py"}:MyArm' and hw['left']['bus'] == 'serial:///dev/ttyUSB0'
    assert hw['left']['options']['rules'] == str(project / 'my_rules.py')
    arm = open_arm(hw['left']['arm'], 'left', hw['left']['bus'], connect=False, **hw['left']['options'])
    assert arm._rule is not None


def test_check_script_passes_and_enables_nothing(project, capsys):
    assert load(project / 'check.py', 'generated_check').main() == 0
    out = capsys.readouterr().out
    assert "'dry_run': True" in out and 'target after your rules: [2.0' in out


class FakeRobot(BaseHTTPRequestHandler):
    """The local API's task routes, as the client uses them: status, create, live stream, one task."""
    tasks = {}

    def log_message(self, *a):
        pass

    def _json(self, code, obj):
        data = json.dumps(obj).encode()
        self.send_response(code); self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)

    def do_GET(self):
        if self.path == '/v1/status':
            return self._json(200, dict(ready=True, profile='yam_single'))
        task_id = self.path.split('/')[3]
        if self.path.endswith('/live'):
            assert 'text/event-stream' in self.headers['Accept']
            self.send_response(200); self.send_header('Content-Type', 'text/event-stream'); self.end_headers()
            for i, status in enumerate(('running', 'running', 'done')):
                view = dict(self.tasks[task_id], status=status, moves=i, elapsed_s=float(i), message=['Working', 'Moving', 'Done'][i])
                self.wfile.write(b'data: ' + json.dumps(view).encode() + b'\n\n'); self.wfile.flush()
            return
        return self._json(200, self.tasks[task_id])

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        if self.path == '/v1/tasks':
            if not body.get('prompt'):
                return self._json(400, dict(error=dict(code='bad_request', message='prompt is required')))
            task = dict(id='t1', status='running', mode=body.get('mode', 'run'), prompt=body['prompt'], moves=0, elapsed_s=0.0,
                        message='Working', error=None, recording_url=None)
            self.tasks['t1'] = task
            return self._json(202, task)
        self._json(404, {})


def test_the_task_client_starts_a_task_and_follows_it_to_the_end(project, monkeypatch, capsys):
    srv = ThreadingHTTPServer(('127.0.0.1', 0), FakeRobot)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        client = load(project / 'run_task.py', 'generated_run_task')
        monkeypatch.setattr('sys.argv', ['run_task.py', 'pick up the cup', '--mode', 'preview',
                                         '--api', f'http://127.0.0.1:{srv.server_address[1]}'])
        assert client.main() == 0
        out = capsys.readouterr().out
        assert 'task t1: running' in out and 'Moving' in out and '"status": "done"' in out
        assert FakeRobot.tasks['t1']['mode'] == 'preview'
        monkeypatch.setattr('sys.argv', ['run_task.py', '', '--api', f'http://127.0.0.1:{srv.server_address[1]}'])
        assert client.main() == 2 and 'prompt is required' in capsys.readouterr().err
    finally:
        srv.shutdown()


# ------------------------------------------------------------------------------------------------ the pin and the tests
def test_the_project_pins_the_installed_roborun(project, monkeypatch):
    try:
        import tomllib                                                      # Python 3.11+
    except ImportError:
        tomllib = pytest.importorskip('tomli')                              # Python 3.10: the test extra installs it
    from packaging.requirements import Requirement
    meta = tomllib.loads((project / 'pyproject.toml').read_text())
    assert meta['project']['name'] == 'my-robot' and meta['tool']['setuptools']['py-modules'] == []
    req = Requirement(meta['project']['dependencies'][0])
    assert req.name == 'roborun-sdk'
    from roborun_sdk import __version__
    assert __version__ in req.specifier                                     # the Roborun that made it satisfies its pin
    assert meta['tool']['uv']['find-links'] == ['https://roborun.app/dl/simple/']
    assert meta['tool']['pytest']['ini_options']['testpaths'] == ['tests']
    assert '{' not in (project / 'pyproject.toml').read_text() + (project / 'README.md').read_text()
    # a release pins exactly; a source checkout (0.1.0 is never published) pins its series
    assert scaffold.sdk_requirement('0.1.20261119172831') == 'roborun-sdk==0.1.20261119172831'
    assert scaffold.sdk_requirement('0.1.0') == 'roborun-sdk>=0.1,<0.2'
    monkeypatch.setattr(scaffold, 'sdk_requirement', lambda version=None: 'roborun-sdk==0.1.20261119172831')
    again = scaffold.create('pinned', project.parent)
    assert '"roborun-sdk==0.1.20261119172831"' in (again / 'pyproject.toml').read_text()
    assert 'roborun-sdk==0.1.20261119172831' in (again / 'README.md').read_text()


def test_the_generated_tests_pass(project):
    import os
    import subprocess
    import sys
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(Path(__file__).resolve().parents[1] / 'src'),
                                                       os.environ.get('PYTHONPATH', '')]))
    done = subprocess.run([sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider'], cwd=project, env=env,
                          capture_output=True, text=True, timeout=120)
    assert done.returncode == 0, done.stdout[-2000:] + done.stderr[-2000:]
    assert '4 passed' in done.stdout
