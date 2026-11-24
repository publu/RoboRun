"""The per-task Stop of a policy run (POST /v1/tasks/{id}/stop through the page's door, Robot.stop_task, policy.control), and
the task routes that must count a policy run as a task (stop status, wait). A real station process logic over HTTP, the
runner, and a fake backend that, like the runtime, does not drive policy runs: it answers 404 for their Stop."""
import http.client
import json
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from fake_policy_bits import Sway, Warming, own_home
from test_policy_runner import Rig, actions

from roborun_sdk import policy
from roborun_sdk.client import Robot, RobotError
from roborun_sdk.local_proxy import LocalProxy
from roborun_sdk.policy import control

APP = 'https://app.roborun.app'


class Backend(BaseHTTPRequestHandler):
    """The runtime as far as these routes go: GET /v1/tasks/{id} reads the record file (as the runtime's list does), the
    per-task Stop knows only its own runners (404 for a policy run), stop status names no policy run, /v1/stop is taken."""
    protocol_version = 'HTTP/1.1'
    runtime = None
    seen = []

    def log_message(self, *a):
        pass

    def _send(self, code, body):
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _any(self):
        size = int(self.headers.get('Content-Length') or 0)
        if size:
            self.rfile.read(size)
        Backend.seen.append((self.command, self.path))
        m = re.fullmatch(r'/v1/tasks/([0-9a-f]{32})(/stop)?', self.path)
        if m and m.group(2):
            return self._send(404, dict(error=dict(code='not_found', message='no such running task', retryable=False)))
        if m:
            try:
                rec = json.loads((Backend.runtime / 'log' / 'tasks' / f'{m.group(1)}.json').read_text())
            except OSError:
                return self._send(404, dict(error=dict(code='not_found', message='no such task')))
            return self._send(200, dict(id=rec['id'], status=rec['status'], message=rec.get('final') or rec.get('phase')))
        if self.path == '/v1/stop/status':
            return self._send(200, dict(state='stopped', confirmed=True, arms={}, task=None, max_speed_rad_s=0.0,
                                        message='Nothing is moving: the arms are holding still'))
        if self.path == '/v1/stop':
            return self._send(200, dict(state='stopped', confirmed=True, stopped_tasks=[], arms={}, message='Stopped'))
        return self._send(404, dict(error=dict(code='not_found', message='no route')))

    do_GET = do_POST = _any


@pytest.fixture
def home(tmp_path, monkeypatch):
    return own_home(tmp_path, monkeypatch)              # never this computer's robot.json, runtime or $HOME


@pytest.fixture
def rig(tmp_path, home, monkeypatch):
    from roborun_sdk import robot
    r = Rig(tmp_path)
    cfg = tmp_path / 'robot.json'
    cfg.write_text(json.dumps(r.config))                  # where policy.control finds the runtime and the station
    monkeypatch.setattr(robot, 'CONFIG', cfg)
    Backend.runtime, Backend.seen = r.runtime, []
    srv = ThreadingHTTPServer(('127.0.0.1', 0), Backend)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    r.api = f'http://127.0.0.1:{srv.server_address[1]}'
    r.backend = srv
    yield r
    srv.shutdown()
    srv.server_close()
    r.close()


def _playing(rig, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if rig.station.lease and rig.station.lease.get('policy', {}).get('chunks'):
            return
        time.sleep(0.02)
    raise AssertionError('the run never played a chunk')


def _record(rig, task_id):
    return json.loads((rig.runtime / 'log' / 'tasks' / f'{task_id}.json').read_text())


def _held_still(rig, seconds=0.5):
    q = list(rig.station.left.q_cmd)
    time.sleep(seconds)
    return max(abs(a - b) for a, b in zip(q, rig.station.left.q_cmd))


# ---------------------------------------------------------------- mid-run
def test_robot_stop_task_stops_a_running_policy_run_like_a_task_s_own_stop(rig):
    run = policy.start(Sway(amplitude=0.2, period=1.0), prompt='p', arms='left', seconds=30, **rig.kw())
    _playing(rig)
    gen = rig.station.hold_generation
    t0 = time.monotonic()
    view = Robot(rig.api).stop_task(run.task_id)
    took = time.monotonic() - t0
    assert rig.station.lease['active'] is False and rig.station.lease['reason'] == 'supervisor_stop'
    assert rig.station.lease['end_note'] == 'per-task Stop'
    assert rig.station.hold_generation == gen              # this task's Stop, not the robot-wide hold
    assert view['id'] == run.task_id and view['status'] in ('stopping', 'stopped')
    assert took < 1.0, took                                # the station's answer, plus at most 0.3 s for the record
    assert _held_still(rig) == 0.0                         # nothing more plays: the arm holds where it is
    result = run.wait(10)
    assert result.status == 'stopped' and result.stop_reason == 'operator_stop'
    rec = _record(rig, run.task_id)
    assert rec['status'] == 'stopped' and rec['stop_reason'] == 'operator_stop'
    assert Robot(rig.api).wait(run.task_id, timeout=5)['status'] == 'stopped'
    assert not any(p.endswith('/stop') and p.startswith('/v1/tasks/') for _, p in Backend.seen)   # never 404'd


def test_stopping_an_ended_policy_run_is_fine_and_other_tasks_go_to_the_backend(rig):
    result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **rig.kw())
    view = Robot(rig.api).stop_task(result.task_id)
    assert view['status'] == 'done'
    other = 'f' * 32
    with pytest.raises(RobotError) as err:
        Robot(rig.api).stop_task(other)
    assert err.value.status == 404 and ('POST', f'/v1/tasks/{other}/stop') in Backend.seen


# ---------------------------------------------------------------- before the lease
def test_a_per_task_stop_while_the_model_warms_ends_the_run_unmoved(rig):
    run = policy.start(Warming(warm=10 ** 6), prompt='p', arms='left', seconds=4, **rig.kw())
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline and not (rig.runtime / 'log' / 'tasks' / f'{run.task_id}.json').exists():
        time.sleep(0.02)
    time.sleep(0.5)
    t0 = time.monotonic()
    view = Robot(rig.api).stop_task(run.task_id)
    assert view['status'] in ('stopping', 'stopped')
    result = run.wait(10)
    assert time.monotonic() - t0 < 1.0                     # seen within one 0.25 s read of the station
    assert result.status == 'stopped' and result.stop_reason == 'operator_stop'
    assert result.message == 'Stopped before anything moved'
    assert not actions(rig) and rig.station.lease is None
    assert _record(rig, run.task_id)['status'] == 'stopped'


def test_a_per_task_stop_between_warm_up_and_the_lease_is_refused_by_the_station(rig, monkeypatch):
    from roborun_sdk.policy import runner as runner_mod
    original = runner_mod.PolicyRunner._warm_up

    def warm_then_stop(self):
        first = original(self)
        control.stop_run(self.id, wait_s=0)
        return first
    monkeypatch.setattr(runner_mod.PolicyRunner, '_warm_up', warm_then_stop)
    monkeypatch.setattr(runner_mod.PolicyRunner, '_check_hold', lambda self, pose=None, force=False: None)
    result = policy.run(Sway(), prompt='p', arms='left', seconds=2, **rig.kw())
    assert result.status == 'failed' and result.stop_reason == 'lease_refused' and 'nothing moved' in result.message
    assert not actions(rig) and rig.station.lease is None


# ---------------------------------------------------------------- the routes that must count it
def test_stop_status_counts_a_running_policy_run_and_wait_follows_it(rig):
    run = policy.start(Sway(), prompt='p', arms='left', seconds=30, **rig.kw())
    _playing(rig)
    robot = Robot(rig.api)
    status = robot.stop_status()
    assert status['state'] == 'running' and status['task'] == dict(id=run.task_id, status='running', kind='policy')
    with pytest.raises(TimeoutError):
        robot.wait(run.task_id, timeout=0.5, poll_s=0.1)
    robot.stop_task(run.task_id)
    assert robot.wait(run.task_id, timeout=5, poll_s=0.1)['status'] == 'stopped'
    assert robot.stop_status()['state'] == 'stopped'


def test_without_the_backend_the_client_still_stops_and_follows_a_policy_run(rig):
    run = policy.start(Sway(), prompt='p', arms='left', seconds=30, **rig.kw())
    _playing(rig)
    robot = Robot('http://127.0.0.1:9')                    # nothing answers there: Roborun's backend is down
    assert robot.task(run.task_id)['status'] == 'running'
    view = robot.stop_task(run.task_id)
    assert view['status'] in ('stopping', 'stopped') and rig.station.lease['reason'] == 'supervisor_stop'
    assert robot.wait(run.task_id, timeout=5, poll_s=0.1)['status'] == 'stopped'
    run.wait(5)


def test_a_remote_client_never_stops_a_policy_run_from_its_own_files(rig):
    run = policy.start(Sway(), prompt='p', arms='left', seconds=30, **rig.kw())
    _playing(rig)
    robot = Robot('http://192.0.2.1:8120', timeout=0.5)    # another computer's robot: this computer's files are not its
    assert robot._policy() is None
    with pytest.raises(RobotError):
        robot.stop_task(run.task_id)
    assert rig.station.lease['active'] is True
    run.stop()
    run.wait(10)


def test_the_station_down_is_said_as_a_stop_not_confirmed(rig):
    run = policy.start(Warming(warm=10 ** 6), prompt='p', arms='left', seconds=4, **rig.kw())
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline and not (rig.runtime / 'log' / 'tasks' / f'{run.task_id}.json').exists():
        time.sleep(0.02)
    with pytest.raises(control.StopNotConfirmed, match="not answering"):
        control.stop_run(run.task_id, station='http://127.0.0.1:9')
    with pytest.raises(LookupError):
        control.stop_run('e' * 32)
    run.stop()
    run.wait(10)


# ---------------------------------------------------------------- the page's door
def _door(rig):
    port = int(rig.api.rsplit(':', 1)[1])
    proxy = LocalProxy('127.0.0.1', port, port=0, robot='bench').bind()
    threading.Thread(target=proxy.server.serve_forever, daemon=True).start()
    return proxy


def _call(proxy, method, path, origin=APP, body=b'{}'):
    conn = http.client.HTTPConnection('127.0.0.1', proxy.port, timeout=10)
    headers = {'Host': f'127.0.0.1:{proxy.port}', 'Content-Type': 'application/json'}
    if origin:
        headers['Origin'] = origin
    conn.request(method, path, body=body if method == 'POST' else None, headers=headers)
    r = conn.getresponse()
    data = json.loads(r.read() or b'{}')
    conn.close()
    return r.status, data


def test_the_page_s_per_task_stop_stops_a_policy_run(rig):
    proxy = _door(rig)
    try:
        run = policy.start(Sway(), prompt='p', arms='left', seconds=30, **rig.kw())
        _playing(rig)
        code, status = _call(proxy, 'GET', f'/k/{proxy.key}/v1/stop/status')
        assert code == 200 and status['task']['id'] == run.task_id and status['state'] == 'running'
        code, view = _call(proxy, 'POST', f'/k/{proxy.key}/v1/tasks/{run.task_id}/stop')
        assert code == 200 and view['status'] in ('stopping', 'stopped')
        assert rig.station.lease['reason'] == 'supervisor_stop'
        assert run.wait(10).status == 'stopped'
        other = 'f' * 32                                   # an ordinary task's Stop still goes to the backend
        code, _ = _call(proxy, 'POST', f'/k/{proxy.key}/v1/tasks/{other}/stop')
        assert code == 404 and ('POST', f'/v1/tasks/{other}/stop') in Backend.seen
        code, _ = _call(proxy, 'POST', f'/k/{proxy.key}/v1/tasks/{run.task_id}/stop', origin=None)
        assert code == 403                                 # a state change always names its page, as before
    finally:
        proxy.stop()


def test_the_viewer_s_stop_status_counts_a_policy_run(rig):
    proxy = _door(rig)
    try:
        run = policy.start(Sway(), prompt='p', arms='left', seconds=30, **rig.kw())
        _playing(rig)
        code, status = _call(proxy, 'GET', f'/v/{proxy.view_key}/v1/stop/status', origin=None)
        assert code == 200 and status['task']['id'] == run.task_id
        run.stop()
        run.wait(10)
    finally:
        proxy.stop()


# ---------------------------------------------------------------- looking for policy runs never stands between a Stop and the backend
ORDINARY = 'f' * 32


def _broken_record(rig, task_id='e' * 32):
    """A policy task record that says running but carries no id (a damaged or hand-edited file)."""
    tasks = rig.runtime / 'log' / 'tasks'
    tasks.mkdir(parents=True, exist_ok=True)
    (tasks / f'{task_id}.json').write_text(json.dumps(dict(kind='policy', status='running', prompt='p')))
    return task_id


def _not_an_object(monkeypatch, tmp_path):
    from roborun_sdk import robot
    own_home(tmp_path, monkeypatch)                       # no `manifest`: the default runtime under $HOME, so $HOME is the test's
    cfg = tmp_path / 'robot-list.json'
    cfg.write_text('[]')                                  # valid JSON, not an object
    monkeypatch.setattr(robot, 'CONFIG', cfg)


def test_robot_stop_sends_the_stop_first_and_a_record_without_an_id_does_not_block_it(rig, monkeypatch):
    _broken_record(rig)
    asked = []
    real = control.running
    monkeypatch.setattr(control, 'running', lambda *a: asked.append(list(Backend.seen)) or real(*a))
    out = Robot(rig.api).stop()
    assert out['state'] == 'stopped' and ('POST', '/v1/stop') in Backend.seen
    assert all(('POST', '/v1/stop') in seen for seen in asked)   # the policy runs were looked for only after the Stop went
    assert control.running() == []                       # a record without a task id is no run


def test_robot_stop_still_reaches_the_backend_when_the_policy_lookup_raises(rig, monkeypatch):
    def broken(*a, **k):
        raise RuntimeError('damaged')
    for name in ('running', 'policy_record', 'merge_stop_status'):
        monkeypatch.setattr(control, name, broken)
    robot = Robot(rig.api)
    assert robot.stop()['state'] == 'stopped' and ('POST', '/v1/stop') in Backend.seen
    assert robot.stop_status()['state'] == 'stopped'
    with pytest.raises(RobotError) as err:
        robot.stop_task(ORDINARY)
    assert err.value.status == 404 and ('POST', f'/v1/tasks/{ORDINARY}/stop') in Backend.seen


def test_a_robot_json_that_is_not_an_object_never_blocks_a_stop(rig, monkeypatch, tmp_path):
    _not_an_object(monkeypatch, tmp_path)
    root, url = control._where()                         # tolerated: the default runtime and station
    assert url.startswith('http://127.0.0.1:')
    assert control.policy_record(ORDINARY) is None and isinstance(control.running(), list)
    robot = Robot(rig.api)
    assert robot.stop()['state'] == 'stopped' and ('POST', '/v1/stop') in Backend.seen
    with pytest.raises(RobotError) as err:
        robot.stop_task(ORDINARY)
    assert err.value.status == 404 and ('POST', f'/v1/tasks/{ORDINARY}/stop') in Backend.seen
    assert robot.stop_status()['state'] == 'stopped'
    proxy = _door(rig)
    try:
        Backend.seen.clear()
        code, _ = _call(proxy, 'POST', f'/k/{proxy.key}/v1/tasks/{ORDINARY}/stop')
        assert code == 404 and ('POST', f'/v1/tasks/{ORDINARY}/stop') in Backend.seen
        code, status = _call(proxy, 'GET', f'/k/{proxy.key}/v1/stop/status')
        assert code == 200 and status['state'] == 'stopped'
        code, out = _call(proxy, 'POST', f'/k/{proxy.key}/v1/stop')
        assert code == 200 and ('POST', '/v1/stop') in Backend.seen
    finally:
        proxy.stop()


def test_the_page_s_per_task_stop_reaches_the_backend_when_the_policy_lookup_raises(rig, monkeypatch):
    task_id = _broken_record(rig)
    def broken(*a, **k):
        raise KeyError('id')
    monkeypatch.setattr(control, 'stop_run', broken)
    proxy = _door(rig)
    try:
        for tid, answer in ((ORDINARY, 404), (task_id, 503)):  # a damaged policy run's Stop is never a quiet 404
            Backend.seen.clear()
            code, out = _call(proxy, 'POST', f'/k/{proxy.key}/v1/tasks/{tid}/stop')
            assert code == answer and ('POST', f'/v1/tasks/{tid}/stop') in Backend.seen
            assert answer == 404 or out['error']['code'] == 'stop_not_confirmed'
        monkeypatch.setattr(control, 'policy_record', broken)
        Backend.seen.clear()
        code, _ = _call(proxy, 'POST', f'/k/{proxy.key}/v1/tasks/{ORDINARY}/stop')
        assert code == 404 and ('POST', f'/v1/tasks/{ORDINARY}/stop') in Backend.seen
    finally:
        proxy.stop()


def test_a_policy_run_whose_stop_fails_oddly_is_sent_to_the_backend_and_never_reads_as_done(rig, monkeypatch):
    task_id = _broken_record(rig)
    def broken(*a, **k):
        raise KeyError('id')
    monkeypatch.setattr(control, 'stop_run', broken)
    with pytest.raises(RobotError) as err:
        Robot(rig.api).stop_task(task_id)
    assert err.value.status == 503 and err.value.code == 'stop_not_confirmed'
    assert ('POST', f'/v1/tasks/{task_id}/stop') in Backend.seen


def test_roborun_stop_with_a_robot_json_that_is_not_an_object_still_stops(monkeypatch, tmp_path):
    import argparse
    from roborun_sdk import cli
    _not_an_object(monkeypatch, tmp_path)
    sent = []
    monkeypatch.setattr(cli, '_post_json', lambda url, *a, **k: sent.append(url) or (200, {}))
    assert cli._stop_this_robot(argparse.Namespace(json=True)) == 0
    assert any(u.endswith('/supervisor/hold') for u in sent) and 'http://127.0.0.1:8120/api/stop' in sent


# ---------------------------------------------------------------- the robot-wide Stop with the backend down, on an older station
class OldStation(BaseHTTPRequestHandler):
    """A station older than the per-run route (404 for /supervisor/policy/stop). Run A holds the lease; run B warms up, so
    B's own Stop is refused ('another task is running'). `hold` is what /supervisor/hold answers (404: not installed)."""
    protocol_version = 'HTTP/1.1'
    seen = []
    lease = {}
    hold = 200

    def log_message(self, *a):
        pass

    def _send(self, code, body):
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _any(self):
        size = int(self.headers.get('Content-Length') or 0)
        body = json.loads(self.rfile.read(size) or b'{}') if size else {}
        OldStation.seen.append((self.command, self.path, body))
        if self.path == '/supervisor/pose':
            return self._send(200, dict(execution_lease=dict(OldStation.lease)))
        if self.path == '/supervisor/stop' and body.get('lease_id') == OldStation.lease.get('id'):
            OldStation.lease['active'] = False
            return self._send(200, dict(lease_ended=True))
        if self.path == '/supervisor/hold' and OldStation.hold == 200:
            OldStation.lease['active'] = False
            return self._send(200, dict(actions=['lease_ended']))
        return self._send(404, dict(error='no route'))

    do_GET = do_POST = _any


def _two_runs(rig, hold):
    """Two running policy records (B newer than A, so it is tried first) and an older station; robot.json points at it."""
    from roborun_sdk import robot
    srv = ThreadingHTTPServer(('127.0.0.1', 0), OldStation)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    a, b = 'a' * 32, 'b' * 32
    OldStation.seen, OldStation.hold = [], hold
    OldStation.lease = dict(id='L1', active=True, kind='policy', run_id=a)
    tasks = rig.runtime / 'log' / 'tasks'
    tasks.mkdir(parents=True, exist_ok=True)
    for i, rid in enumerate((a, b)):
        path = tasks / f'{rid}.json'
        path.write_text(json.dumps(dict(id=rid, kind='policy', status='running', prompt='p')))
        os.utime(path, (time.time() + i, time.time() + i))
    cfg = rig.tmp / 'robot-old.json'
    cfg.write_text(json.dumps(dict(rig.config, station=f'http://127.0.0.1:{srv.server_address[1]}')))
    robot.CONFIG = cfg                                    # the rig fixture's monkeypatch restores it
    return srv, a, b


def test_a_robot_wide_stop_without_the_backend_takes_the_hold_and_tries_every_run(rig):
    srv, a, b = _two_runs(rig, hold=200)
    try:
        out = Robot('http://127.0.0.1:9').stop()          # the backend is down
        paths = [p for _, p, _ in OldStation.seen]
        assert paths.index('/supervisor/hold') < min(i for i, p in enumerate(paths) if p == '/supervisor/policy/stop')
        assert sum(p == '/supervisor/policy/stop' for p in paths) == 2       # B's refusal did not stop A's own Stop
        assert OldStation.lease['active'] is False
        assert out['state'] == 'stopping' and sorted(out['stopped_tasks']) == [a, b]
        assert 'hold' in out['message']
        OldStation.lease['active'], OldStation.seen = True, []
        code, body = LocalProxy('127.0.0.1', 9, port=0, robot='bench').stop_without_backend()   # the page's /v1/stop
        assert code == 200 and sorted(body['stopped_tasks']) == [a, b] and OldStation.lease['active'] is False
        assert [p for _, p, _ in OldStation.seen][0] == '/supervisor/hold'
    finally:
        srv.shutdown(); srv.server_close()


def test_without_the_hold_every_run_is_still_tried_and_every_failure_is_named(rig):
    srv, a, b = _two_runs(rig, hold=404)                  # no hold route either
    try:
        with pytest.raises(RobotError) as err:
            Robot('http://127.0.0.1:9').stop()
        assert err.value.status == 503 and err.value.code == 'stop_not_confirmed'
        assert b in str(err.value) and a not in str(err.value)               # only B failed, and the answer says so
        assert OldStation.lease['active'] is False                           # A was still stopped by its lease id
        OldStation.lease['active'], OldStation.seen = True, []
        code, body = LocalProxy('127.0.0.1', 9, port=0, robot='bench').stop_without_backend()
        assert code == 503 and body['error']['code'] == 'stop_not_confirmed' and body['stopped_tasks'] == [a]
        assert OldStation.lease['active'] is False
    finally:
        srv.shutdown(); srv.server_close()


def test_stopped_tasks_names_a_run_that_closed_while_a_slow_stop_was_answered(rig, monkeypatch):
    run = policy.start(Sway(), prompt='p', arms='left', seconds=30, **rig.kw())
    _playing(rig)
    real = Backend._any

    def slow(self):
        if self.path == '/v1/stop':                       # the backend's Stop ends the run, then answers late
            rig.station.supervisor_stop(dict(lease_id=rig.station.lease['id']))
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and _record(rig, run.task_id)['status'] in ('running', 'stopping'):
                time.sleep(0.02)
            time.sleep(0.5)
        return real(self)
    monkeypatch.setattr(Backend, '_any', slow)
    Backend.do_GET = Backend.do_POST = slow
    try:
        out = Robot(rig.api).stop()
    finally:
        Backend.do_GET = Backend.do_POST = real
    assert _record(rig, run.task_id)['status'] == 'stopped'
    assert run.task_id in out['stopped_tasks']
    run.wait(5)


def test_with_the_station_down_too_every_run_is_tried_and_each_failure_is_named(rig, monkeypatch):
    """Backend down and the station not answering: no hold, and every run's own Stop fails. Each run is still tried (none is
    skipped after the first failure), each failure is named, and the answer is 503 stop_not_confirmed."""
    from roborun_sdk import robot
    srv, a, b = _two_runs(rig, hold=200)
    srv.shutdown(); srv.server_close()                    # the station goes away
    cfg = rig.tmp / 'robot-down.json'
    cfg.write_text(json.dumps(dict(rig.config, station='http://127.0.0.1:9')))
    robot.CONFIG = cfg
    tried = []
    real = control.stop_run
    monkeypatch.setattr(control, 'stop_run', lambda run_id, *a_, **k: tried.append(run_id) or real(run_id, *a_, **k))
    with pytest.raises(RobotError) as err:
        Robot('http://127.0.0.1:9').stop()
    assert err.value.status == 503 and err.value.code == 'stop_not_confirmed'
    assert a in str(err.value) and b in str(err.value)
    assert sorted(tried) == [a, b]
    tried.clear()
    code, body = LocalProxy('127.0.0.1', 9, port=0, robot='bench').stop_without_backend()
    assert code == 503 and body['error']['code'] == 'stop_not_confirmed' and body['stopped_tasks'] == []
    assert a in body['message'] and b in body['message'] and sorted(tried) == [a, b]


def test_an_odd_failure_stopping_one_run_never_keeps_the_next_from_stopping(rig, monkeypatch):
    """The newest run's Stop raises something unexpected (not StopNotConfirmed) and there is no hold: the older, moving run is
    still stopped by its lease id, and the answer names only the run that failed."""
    srv, a, b = _two_runs(rig, hold=404)
    real = control._station_stop

    def odd(url, run_id):
        if run_id == b:
            raise KeyError('damaged answer')
        return real(url, run_id)
    monkeypatch.setattr(control, '_station_stop', odd)
    try:
        with pytest.raises(RobotError) as err:
            Robot('http://127.0.0.1:9').stop()
        assert err.value.status == 503 and b in str(err.value) and a not in str(err.value)
        assert OldStation.lease['active'] is False
        OldStation.lease['active'] = True
        code, body = LocalProxy('127.0.0.1', 9, port=0, robot='bench').stop_without_backend()
        assert code == 503 and body['stopped_tasks'] == [a] and OldStation.lease['active'] is False
    finally:
        srv.shutdown(); srv.server_close()


def test_a_slow_stop_names_the_runs_it_stopped_and_only_those(rig):
    """The backend takes 0.6 s to answer /v1/stop. Runs that closed while it did, because the station ended them, are named;
    a run that ended on its own meanwhile, and one stopped before the Stop was sent, are not."""
    from roborun_sdk.policy.record import utc_now
    tasks = rig.runtime / 'log' / 'tasks'
    tasks.mkdir(parents=True, exist_ok=True)
    early, held, signalled, restarted, done = 'c' * 32, 'd' * 32, 'e' * 32, 'f' * 32, '0' * 32

    def write(rid, **rec):
        (tasks / f'{rid}.json').write_text(json.dumps(dict(id=rid, kind='policy', prompt='p', **rec)))
    write(early, status='stopped', stop_reason='operator_stop', ended='2026-01-01T00:00:00.000+00:00')
    for rid in (held, signalled, restarted, done):
        write(rid, status='running')
    real = Backend._any

    def slow(self):
        if self.path == '/v1/stop':                       # the runs close while the backend's Stop is still on its way
            write(held, status='stopped', stop_reason='operator_stop', ended=utc_now())
            write(signalled, status='stopped', stop_reason='signal', ended=utc_now())
            write(restarted, status='stopped', stop_reason='provider_session_restarted', ended=utc_now())
            write(done, status='done', stop_reason='policy_finished', ended=utc_now())
            time.sleep(0.6)
        return real(self)
    Backend.do_GET = Backend.do_POST = slow
    try:
        t0 = time.monotonic()
        out = Robot(rig.api).stop()
        assert time.monotonic() - t0 >= 0.6
    finally:
        Backend.do_GET = Backend.do_POST = real
    assert sorted(out['stopped_tasks']) == sorted([held, signalled])


def test_a_slow_stop_names_a_warming_run_it_stopped(rig):
    """A run still warming up when the Stop is sent ends unmoved (the hold) before a slow backend answers: it is named."""
    run = policy.start(Warming(warm=10 ** 6), prompt='p', arms='left', seconds=30, **rig.kw())
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline and not (rig.runtime / 'log' / 'tasks' / f'{run.task_id}.json').exists():
        time.sleep(0.02)
    time.sleep(0.5)
    real = Backend._any
    held = []

    def slow(self):
        if self.path == '/v1/stop':                       # the backend's Stop takes the station's hold, then answers late
            held.append(control._hold(rig.srv.url))
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and _record(rig, run.task_id)['status'] in ('running', 'stopping'):
                time.sleep(0.02)
            time.sleep(0.4)
        return real(self)
    Backend.do_GET = Backend.do_POST = slow
    try:
        out = Robot(rig.api).stop()
    finally:
        Backend.do_GET = Backend.do_POST = real
    rec = _record(rig, run.task_id)
    assert held == [True] and rig.station.lease is None
    assert rec['status'] == 'stopped' and rec['stop_reason'] == 'operator_stop'
    assert run.task_id in out['stopped_tasks']
    run.wait(5)


# ---------------------------------------------------------------- a backend that hangs or fails on /v1/stop
def _backend_stop(behaviour):
    """Make the fake backend's POST /v1/stop hang (until released) or answer 500; returns (release, restore)."""
    real = Backend._any
    release = threading.Event()

    def bad(self):
        if self.command == 'POST' and self.path == '/v1/stop':
            size = int(self.headers.get('Content-Length') or 0)
            if size:
                self.rfile.read(size)
            Backend.seen.append((self.command, self.path))
            if behaviour == 'hang':
                release.wait(30)
                return
            if behaviour == 'bad_status_line':                 # a backend that crashes as it answers
                self.close_connection = True
                self.wfile.write(b'garbage\r\n\r\n')
                return
            if behaviour == 'truncated':                       # ... or part way through its body
                self.close_connection = True
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', '400')
                self.end_headers()
                self.wfile.write(b'{"state": "sto')
                return
            if behaviour == 'not_json':
                data = b'ok'
                self.send_response(200)
                self.send_header('Content-Type', 'text/plain')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if behaviour == '200_error':                       # a 200 whose body says the Stop was not confirmed
                return self._send(200, dict(state='stopping', stopped_tasks=[], error=dict(
                    code='stop_not_confirmed', message='an arm did not confirm', retryable=True)))
            if behaviour == '404':
                return self._send(404, dict(error=dict(code='not_found', message='no route', retryable=False)))
            if behaviour == 'late_500':                        # an error that lands just as the client stops waiting
                time.sleep(0.3)
            return self._send(500, dict(error=dict(code='internal', message='boom', retryable=True)))
        return real(self)
    Backend.do_GET = Backend.do_POST = bad

    def restore():
        release.set()
        Backend.do_GET = Backend.do_POST = real
    return restore


def _lease_ends_within(rig, t0, bound):
    deadline = t0 + bound
    while time.monotonic() < deadline:
        if not rig.station.lease['active']:
            return time.monotonic() - t0
        time.sleep(0.01)
    raise AssertionError(f'the policy run still held its lease {bound} s after the Stop')


@pytest.mark.parametrize('behaviour', ['hang', '500'])
def test_robot_stop_stops_a_policy_run_when_the_backend_hangs_or_fails(rig, behaviour):
    run = policy.start(Sway(), prompt='p', arms='left', seconds=30, **rig.kw())
    _playing(rig)
    restore = _backend_stop(behaviour)
    try:
        t0 = time.monotonic()
        out = Robot(rig.api, timeout=10).stop()
        took = time.monotonic() - t0
        assert took < 2.5 and _lease_ends_within(rig, t0, 2.5) < 2.5
        assert out['state'] == 'stopping' and out['stopped_tasks'] == [run.task_id]
        assert ('POST', '/v1/stop') in Backend.seen                         # the backend was still asked first
        assert ('500' in out['message']) if behaviour == '500' else ('within 2 s' in out['message'])
        assert _held_still(rig) == 0.0
        assert run.wait(10).status == 'stopped'
    finally:
        restore()


@pytest.mark.parametrize('behaviour', ['hang', '500'])
def test_the_door_s_stop_stops_a_policy_run_when_the_backend_hangs_or_fails(rig, behaviour):
    proxy = _door(rig)
    run = policy.start(Sway(), prompt='p', arms='left', seconds=30, **rig.kw())
    _playing(rig)
    restore = _backend_stop(behaviour)
    try:
        t0 = time.monotonic()
        code, out = _call(proxy, 'POST', f'/k/{proxy.key}/v1/stop')
        assert time.monotonic() - t0 < 2.5 and _lease_ends_within(rig, t0, 2.5) < 2.5
        assert code == 200 and out['stopped_tasks'] == [run.task_id] and out['state'] == 'stopping'
        assert _held_still(rig) == 0.0
        assert run.wait(10).status == 'stopped'
        first = rig.station.lease['id']
        time.sleep(0.8)                                    # the station starts nothing new within 0.75 s of a Stop
        run = policy.start(Sway(), prompt='p', arms='left', seconds=30, **rig.kw())   # the viewer's Stop: the same
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not (rig.station.lease['id'] != first and rig.station.lease['active']
                                                   and rig.station.lease.get('policy', {}).get('chunks')):
            time.sleep(0.02)
        assert rig.station.lease['id'] != first and rig.station.lease['active'], f'the second run never leased: {run.result}'
        t0 = time.monotonic()
        code, out = _call(proxy, 'POST', f'/v/{proxy.view_key}/v1/stop', origin=None)
        assert code == 200 and _lease_ends_within(rig, t0, 2.5) < 2.5 and out['stopped_tasks'] == [run.task_id]
        assert run.wait(10).status == 'stopped'
    finally:
        restore()
        proxy.stop()


@pytest.mark.parametrize('behaviour', ['hang', '500'])
def test_with_no_policy_run_a_failing_backend_s_stop_is_answered_as_before(rig, behaviour, monkeypatch):
    """No policy run here: nothing is done at the station, and the backend's own answer (its 500) is passed on; a hanging
    backend is waited for, as before (bounded by the client's timeout)."""
    restore = _backend_stop(behaviour)
    called = []
    monkeypatch.setattr(control, '_hold', lambda url: called.append(url) or True)
    try:
        with pytest.raises(RobotError) as err:
            Robot(rig.api, timeout=3).stop()
        assert (err.value.status == 500) if behaviour == '500' else (err.value.code == 'not_running')
        assert called == []
    finally:
        restore()


def test_the_door_s_stop_names_a_run_that_closed_while_a_slow_backend_answered(rig):
    """The backend's Stop ends the run and answers after 0.5 s (inside the bound): the door's answer names the run."""
    proxy = _door(rig)
    run = policy.start(Sway(), prompt='p', arms='left', seconds=30, **rig.kw())
    _playing(rig)
    real = Backend._any

    def slow(self):
        if self.path == '/v1/stop':
            rig.station.supervisor_stop(dict(lease_id=rig.station.lease['id']))
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and _record(rig, run.task_id)['status'] in ('running', 'stopping'):
                time.sleep(0.02)
            time.sleep(0.5)
        return real(self)
    Backend.do_GET = Backend.do_POST = slow
    try:
        code, out = _call(proxy, 'POST', f'/k/{proxy.key}/v1/stop')
    finally:
        Backend.do_GET = Backend.do_POST = real
        proxy.stop()
    assert code == 200 and out['state'] == 'stopped' and run.task_id in out['stopped_tasks']
    run.wait(5)


# ---------------------------------------------------------------- the running task, in the viewer, the page and roborun status
def test_status_names_a_running_policy_run_as_the_task(rig, capsys, monkeypatch):
    real = Backend._any

    def with_status(self):
        if self.path == '/v1/status':
            return self._send(200, dict(profile='yam', task=None, arms={}))
        return real(self)
    Backend.do_GET = Backend.do_POST = with_status
    proxy = _door(rig)
    try:
        code, st = _call(proxy, 'GET', f'/v/{proxy.view_key}/v1/status', origin=None)
        assert code == 200 and st['task'] is None                          # nothing runs yet
        run = policy.start(Sway(), prompt='wave', arms='left', seconds=30, **rig.kw())
        _playing(rig)
        code, st = _call(proxy, 'GET', f'/v/{proxy.view_key}/v1/status', origin=None)
        assert code == 200 and st['task'] == dict(id=run.task_id, status='running', kind='policy', prompt='wave')
        assert st['profile'] == 'yam'
        code, st = _call(proxy, 'GET', f'/k/{proxy.key}/v1/status')
        assert code == 200 and st['task']['id'] == run.task_id
        assert Robot(rig.api).status()['task']['id'] == run.task_id
        from roborun_sdk import robot
        monkeypatch.setattr(robot, '_state', lambda u: 'active' if u == 'roborun-robot' else 'inactive')
        cfg = json.loads(robot.CONFIG.read_text())
        robot.CONFIG.write_text(json.dumps(dict(cfg, backend=rig.api, live_port=0)))
        try:
            robot.cmd_status(None)
        except Exception:                                  # its later lines (sign-in, data sync) need a full install
            pass
        assert f'{run.task_id} running' in capsys.readouterr().out
        run.stop()
        run.wait(10)
    finally:
        Backend.do_GET = Backend.do_POST = real
        proxy.stop()


# ---------------------------------------------------------------- a backend whose Stop answer is broken, not JSON or a 4xx
BROKEN = ['bad_status_line', 'truncated', 'not_json', '404', '200_error']


def test_robot_stop_names_a_200_answer_with_an_error_as_not_confirmed(rig):
    """A 200 whose body carries an error (the shape Roborun's own Stop answers with when it is not confirmed) is not the
    backend taking the Stop: the run is stopped at the station and the message says the backend did not confirm it."""
    run = policy.start(Sway(), prompt='p', arms='left', seconds=30, **rig.kw())
    _playing(rig)
    restore = _backend_stop('200_error')
    try:
        out = Robot(rig.api, timeout=10).stop()
        assert out['confirmed'] is False and out['stopped_tasks'] == [run.task_id]
        assert 'without confirming it (stop_not_confirmed)' in out['message']
        assert run.wait(10).status == 'stopped'
    finally:
        restore()


@pytest.mark.parametrize('behaviour', BROKEN)
def test_robot_stop_stops_a_policy_run_whatever_the_backend_answers(rig, behaviour):
    """A Stop is never refused: only a 2xx JSON answer is the backend taking it. Anything else (a crash mid-answer, a body
    that is not JSON, a 4xx) stops the policy run at the arms' station, as for a 5xx."""
    run = policy.start(Sway(), prompt='p', arms='left', seconds=30, **rig.kw())
    _playing(rig)
    restore = _backend_stop(behaviour)
    try:
        t0 = time.monotonic()
        out = Robot(rig.api, timeout=10).stop()
        assert time.monotonic() - t0 < 2.5 and _lease_ends_within(rig, t0, 2.5) < 2.5
        assert out['state'] == 'stopping' and out['stopped_tasks'] == [run.task_id]
        assert ('POST', '/v1/stop') in Backend.seen
        assert _held_still(rig) == 0.0
        assert run.wait(10).status == 'stopped'
    finally:
        restore()


@pytest.mark.parametrize('behaviour', BROKEN)
def test_the_door_s_stop_stops_a_policy_run_whatever_the_backend_answers(rig, behaviour):
    proxy = _door(rig)
    run = policy.start(Sway(), prompt='p', arms='left', seconds=30, **rig.kw())
    _playing(rig)
    restore = _backend_stop(behaviour)
    try:
        t0 = time.monotonic()
        code, out = _call(proxy, 'POST', f'/k/{proxy.key}/v1/stop')
        assert time.monotonic() - t0 < 2.5 and _lease_ends_within(rig, t0, 2.5) < 2.5
        assert code == 200 and out['state'] == 'stopping' and out['stopped_tasks'] == [run.task_id]
        assert _held_still(rig) == 0.0
        assert run.wait(10).status == 'stopped'
    finally:
        restore()
        proxy.stop()


def test_with_no_policy_run_a_4xx_stop_answer_is_passed_on_as_before(rig, monkeypatch):
    """No policy run here: nothing at the station, the backend's 404 raised by the client and passed on by the door, and
    the door names no run as stopped."""
    restore = _backend_stop('404')
    called = []
    monkeypatch.setattr(control, '_hold', lambda url: called.append(url) or True)
    proxy = _door(rig)
    try:
        with pytest.raises(RobotError) as err:
            Robot(rig.api, timeout=3).stop()
        assert err.value.status == 404
        code, out = _call(proxy, 'POST', f'/k/{proxy.key}/v1/stop')
        assert code == 404 and 'stopped_tasks' not in out
        assert called == []
    finally:
        restore()
        proxy.stop()


class _LateEvent:
    """threading.Event as the Stop's waiter sees it in the worst interleaving: its bounded wait gives up (False) just before
    the send thread finishes, and is_set() then says True."""

    def __init__(self):
        self._e = threading.Event()

    def set(self):
        self._e.set()

    def is_set(self):
        self._e.wait(10)
        return self._e.is_set()

    def wait(self, timeout=None):
        if timeout is not None and timeout < 5:
            return False
        return self._e.wait(timeout)


def test_a_backend_error_that_lands_as_the_wait_ends_still_stops_the_run(rig, monkeypatch):
    """The backend's 500 arrives just after the 2 s wait gave up: the Stop is still taken at the station (never an empty
    answer that stops nothing)."""
    import types
    from roborun_sdk import client, local_proxy
    run = policy.start(Sway(), prompt='p', arms='left', seconds=30, **rig.kw())
    _playing(rig)
    restore = _backend_stop('late_500')
    late = types.SimpleNamespace(Thread=threading.Thread, Event=_LateEvent)
    proxy = _door(rig)
    try:
        monkeypatch.setattr(client, 'threading', late)
        t0 = time.monotonic()
        out = Robot(rig.api, timeout=10).stop()
        assert isinstance(out, dict) and out['stopped_tasks'] == [run.task_id]
        assert _lease_ends_within(rig, t0, 2.5) < 2.5 and _held_still(rig) == 0.0
        assert run.wait(10).status == 'stopped'
        first = rig.station.lease['id']
        time.sleep(0.8)                                    # the station starts nothing new within 0.75 s of a Stop
        run = policy.start(Sway(), prompt='p', arms='left', seconds=30, **rig.kw())
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not (rig.station.lease['id'] != first and rig.station.lease['active']
                                                   and rig.station.lease.get('policy', {}).get('chunks')):
            time.sleep(0.02)
        assert rig.station.lease['id'] != first and rig.station.lease['active'], f'the second run never leased: {run.result}'
        monkeypatch.setattr(local_proxy, 'threading', late)
        t0 = time.monotonic()
        code, out = _call(proxy, 'POST', f'/k/{proxy.key}/v1/stop')
        assert code == 200 and out['stopped_tasks'] == [run.task_id]
        assert _lease_ends_within(rig, t0, 2.5) < 2.5
        assert run.wait(10).status == 'stopped'
    finally:
        restore()
        proxy.stop()


# ---------------------------------------------------------------- roborun up --sim: the simulation's own runtime
def test_under_roborun_up_sim_a_policy_run_lives_in_the_simulation_s_runtime(tmp_path, home, monkeypatch):
    """robot.json as `roborun up --sim cup` leaves it (manifest names the live runtime, the backend runs from the copy in
    <home>/sim): the run's record and recording go to the copy's log/, its cleanup never deletes the live robot's
    recordings, and Robot.stop_task, control.stop_run and `roborun policy runs` find it there."""
    from roborun_sdk import robot, sim_home
    from roborun_sdk.policy.cli import runs
    from roborun_sdk.policy.record import robot_runtime
    from test_policy_runner import _old_recording, _small_mb
    _small_mb(monkeypatch)
    r = Rig(tmp_path)
    live, sim = r.runtime, sim_home.runtime()
    caps = dict(video_record=dict(enabled=True, run_cap_mb=8.0, total_cap_mb=20.0, day_cap_mb=1000.0, min_free_mb=0.0))
    (live / 'config.json').write_text(json.dumps(caps))
    sim.mkdir(parents=True)
    (sim / 'config.json').write_text(json.dumps(caps))
    old = [_old_recording(live / 'log' / 'video-motion', c * 32, f'2026-01-0{i + 1}T00:00:00.000+00:00', 6 * 1024)
           for i, c in enumerate('abc')]
    cfg = tmp_path / 'robot.json'
    cfg.write_text(json.dumps(dict(r.config, sim='cup')))
    monkeypatch.setattr(robot, 'CONFIG', cfg)
    Backend.runtime, Backend.seen = sim, []
    srv = ThreadingHTTPServer(('127.0.0.1', 0), Backend)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    api = f'http://127.0.0.1:{srv.server_address[1]}'
    kw = dict(station_url=r.srv.url, camera_socket=r.sock, dataset_root=r.dataset)   # as the CLI builds it: no runtime=
    try:
        assert robot_runtime(json.loads(cfg.read_text())) == sim
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **kw)
        assert result.status == 'done', result
        assert (sim / 'log' / 'tasks' / f'{result.task_id}.json').exists()
        assert (sim / 'log' / 'video-motion' / result.task_id / f'recording-{result.task_id}-01' / 'recording.json').exists()
        assert not (live / 'log' / 'tasks').exists() and not (live / 'log' / 'video-motion' / result.task_id).exists()
        assert all(p.exists() for p in old)                 # the live robot's recordings are untouched
        assert result.summary['dataset'] == dict(recorded=False, why='simulated robot')
        assert [row['id'] for row in runs()] == [result.task_id]

        run = policy.start(Sway(amplitude=0.2, period=1.0), prompt='p', arms='left', seconds=30, **kw)
        deadline = time.monotonic() + 10                    # the station's lease still shows the first run's chunks
        while time.monotonic() < deadline and (control.policy_record(run.task_id) or {}).get('status') != 'running':
            time.sleep(0.02)
        _playing(r)
        assert control.policy_record(run.task_id)['status'] == 'running'
        view = Robot(api).stop_task(run.task_id)
        assert view['id'] == run.task_id and view['status'] in ('stopping', 'stopped')
        assert run.wait(10).status == 'stopped'
        assert json.loads((sim / 'log' / 'tasks' / f'{run.task_id}.json').read_text())['status'] == 'stopped'
        assert control.stop_run(run.task_id)['status'] == 'stopped'        # stopping an ended run is fine, and found
        assert not any(p.startswith('/v1/tasks/') and p.endswith('/stop') for _, p in Backend.seen)
        assert all(p.exists() for p in old)
    finally:
        srv.shutdown()
        srv.server_close()
        r.close()
