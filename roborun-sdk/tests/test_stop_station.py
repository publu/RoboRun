"""`roborun stop` reaches the arms' station directly as well as Roborun, so it stops a policy run while Roborun is down; and the
station around your ArmAdapter ends a policy lease on either arm's rules refusal and lends the drivers the run's tool speed cap."""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace

import pytest

from fake_policy_bits import Sway, own_home

from roborun_sdk import cli, policy, robot
from roborun_sdk.embodiment import KinematicArm, _station_classes, run_station
from roborun_station import SimClock

HOME_Q = [0.0, 0.5, 0.8, 0.0, 0.3, 0.0]


def free_port():
    import socket
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


@pytest.fixture
def config(tmp_path, monkeypatch):
    """The test's robot.json, in a home of its own: a robot.json without `manifest` falls back to the default runtime under
    $HOME, whose stale policy records `roborun stop` closes (close_orphans), so $HOME and the account home are the test's too."""
    own_home(tmp_path, monkeypatch)
    path = tmp_path / 'robot.json'
    monkeypatch.setattr(robot, 'CONFIG', path)
    return path


def stop(capsys, as_json=False):
    code = cli._stop_this_robot(SimpleNamespace(json=as_json))
    return code, capsys.readouterr().out


def test_with_no_backend_stop_reaches_the_station_and_ends_a_policy_run(tmp_path, config, capsys):
    station, srv = run_station(KinematicArm(home=HOME_Q), port=0)
    try:
        config.write_text(json.dumps(dict(backend=f'http://127.0.0.1:{free_port()}', station=srv.url, robot='t',
                                          manifest=str(tmp_path / 'rt'))))
        run = policy.start(Sway(amplitude=0.05), prompt='p', arms='left', seconds=20, station_url=srv.url,
                           runtime=tmp_path / 'rt', camera_socket='/nonexistent/c.sock', dataset=False,
                           config=json.loads(config.read_text()))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not (station.lease and station.lease.get('policy', {}).get('chunks')):
            time.sleep(0.05)
        code, out = stop(capsys)
        assert code == 0
        assert f"arms' station: holding (ended policy run {run.task_id})" in out and 'Roborun: not running' in out
        assert station.lease['reason'] == 'supervisor_stop'
        result = run.wait(15)
        assert result.status == 'stopped'
        rec = json.loads((tmp_path / 'rt' / 'log' / 'tasks' / f'{result.task_id}.json').read_text())
        assert rec['status'] == 'stopped'
    finally:
        srv.close()


def test_with_neither_running_there_is_nothing_to_stop(config, capsys):
    config.write_text(json.dumps(dict(backend=f'http://127.0.0.1:{free_port()}', station=f'http://127.0.0.1:{free_port()}')))
    code, out = stop(capsys)
    assert code == 0 and 'nothing to stop' in out and "arms' station: not running" in out
    code, out = stop(capsys, as_json=True)
    assert code == 0 and json.loads(out) == dict(station='not running', backend='not running')


def test_a_backend_that_answers_500_is_a_refusal(config, capsys):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_POST(self):
            self.send_response(500); self.send_header('Content-Length', '2'); self.end_headers(); self.wfile.write(b'{}')
    srv = ThreadingHTTPServer(('127.0.0.1', 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        config.write_text(json.dumps(dict(backend=f'http://127.0.0.1:{srv.server_address[1]}',
                                          station=f'http://127.0.0.1:{free_port()}')))
        code, out = stop(capsys)
        assert code == 2 and 'HTTP 500' in out and 'power switch' in out
    finally:
        srv.shutdown()


class Ruled(KinematicArm):
    """A KinematicArm with what a StreamedArm adds: a rules refusal and a tool speed cap (with a tool model)."""
    max_tool_speed_m_s = 0.5
    _tool = object()
    refusal = None

    def rule_refusal(self):
        return self.refusal


def policy_lease(st, arms, **ref):
    oid = st.observe()['observation_id']
    return st.action({'observation_id': oid, 'seconds': 30, 'reference': dict(kind='policy', arms=arms, **ref)})['lease']


def test_a_rules_refusal_on_either_leased_arm_ends_the_policy_lease():
    left, right = Ruled(home=HOME_Q), Ruled(home=HOME_Q)
    st = _station_classes()(left, right, lease_free_hold=True, clock=SimClock())
    policy_lease(st, ['left', 'right'])
    right.refusal = 'rules file refused this motion'
    st.advance(0.1)
    assert st.lease['active'] is False and st.lease['reason'] == 'rules file refused this motion'


def test_the_run_s_tool_speed_cap_is_lent_to_the_drivers_and_given_back():
    left, right = Ruled(home=HOME_Q), Ruled(home=HOME_Q)
    st = _station_classes()(left, right, lease_free_hold=True, clock=SimClock())
    lease = policy_lease(st, ['left', 'right'], max_tool_speed_m_s=0.2)
    assert left.max_tool_speed_m_s == 0.2 and right.max_tool_speed_m_s == 0.2
    assert lease['tool_speed_m_s'] == 0.2 and lease['tool_speed_active'] is True
    st.policy_end({'lease_id': lease['id']})
    assert left.max_tool_speed_m_s == 0.5 and right.max_tool_speed_m_s == 0.5


def test_the_default_cap_comes_from_robot_yaml(tmp_path, monkeypatch):
    monkeypatch.setattr(robot, 'HOME', tmp_path)                 # robot.yaml of this test only
    from roborun_sdk.arms import settings
    settings.path().parent.mkdir(parents=True, exist_ok=True)
    settings.path().write_text('policy:\n  max_tool_speed_m_s: 0.3\n')
    left = Ruled(home=HOME_Q)
    st = _station_classes()(left, None, lease_free_hold=True, clock=SimClock())
    lease = policy_lease(st, ['left'])
    assert lease['tool_speed_m_s'] == 0.3 and left.max_tool_speed_m_s == 0.3
