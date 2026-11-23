"""What protects a real arm around the drivers: settings, the station's view of a sick arm, shutdown, locks, first-run checks."""
import json
import os
import signal
import socket
import subprocess
import sys
import textwrap
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path
from types import SimpleNamespace

import pytest

from roborun_sdk import robot
from roborun_sdk.arms import direction_check, hardware, open_arm, safety, settings
from roborun_sdk.arms.base import ArmHardwareError, StreamedArm
from roborun_sdk.arms.hardware import check_profile, config_from_args
from roborun_sdk.embodiment import ArmAdapter, run_station

ROOT = Path(__file__).resolve().parents[1]


class Bench(StreamedArm):
    limits = [(-3, 3)] * 6
    home = [0.0] * 6
    def __init__(self, joints=6, **kw):
        self.limits = [(-3, 3)] * joints; self.home = [0.0] * joints
        super().__init__(**kw)
        self.q = [0.1] * joints; self.jaw = .5; self.writes = []; self.temps = [30.0] * (joints + 1); self.stuck = set()
        self.fail = False; self.opened = 0; self.off = False; self.reversed = set()
    def open_hardware(self): self.opened += 1; self.fail = False
    def open_dry(self): self.opened += 1
    def read_hardware(self):
        if self.fail: raise OSError('adapter gone')
        return dict(positions_rad=list(self.q), gripper=self.jaw, temperatures_c=list(self.temps), velocities_rad_s=[0.0] * len(self.q))
    def write_hardware(self, q, jaw):
        self.writes.append((list(q), jaw))
        for i, v in enumerate(q):
            if i not in self.stuck:
                self.q[i] = .2 - v if i in self.reversed else v
        self.jaw = jaw
    def limp_hardware(self): pass
    def motors_off_hardware(self): self.off = True


def wait(check, seconds=4.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if check(): return True
        time.sleep(.01)
    return bool(check())


def call(url, body=None):
    data = None if body is None else json.dumps(body).encode()
    with urllib.request.urlopen(urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json', 'X-Manifest-Supervisor': '1'}), timeout=5) as r:
        return json.load(r)


@pytest.fixture
def home(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBORUN_HOME', str(tmp_path))
    monkeypatch.setattr(robot, 'HOME', tmp_path); monkeypatch.setattr(robot, 'CONFIG', tmp_path / 'robot.json')
    return tmp_path


@pytest.fixture
def driver(tmp_path):
    path = tmp_path / 'my_arm.py'
    path.write_text(textwrap.dedent("""
        import os, time
        from pathlib import Path
        from roborun_sdk.arms.base import StreamedArm
        class MyArm(StreamedArm):
            limits = [(-3, 3)] * 6
            home = [0.0] * 6
            EXTRA_SETTINGS = ('declared_knob',)
            def __init__(self, marks=None, **kw):
                super().__init__(**kw); self.marks = Path(marks); self.q = [0.1] * 6
            def mark(self, text):
                with open(self.marks, 'a') as out: out.write(text + '\\n')
            def open_hardware(self): self.mark('open')
            def read_hardware(self): return dict(positions_rad=list(self.q), gripper=.5)
            def write_hardware(self, q, jaw): self.q = list(q)
            def limp_hardware(self): pass
            def motors_off_hardware(self): self.mark('off')
            def close_hardware(self): self.mark('closed at %.2f' % self.q[0])
    """))
    return path


# -- settings ---------------------------------------------------------------------------------------------------------
def test_every_setting_comes_from_the_file_and_an_arm_value_wins(home):
    (home / 'robot.yaml').write_text("""
dry_run: true
motion: {rate_hz: 50, stall_time_s: 2, temp_stop_c: 60, temperature_export: t.jsonl, my_knob: 1}
arms:
  left: {arm: yam, address: 'slcan://A', stall_time_s: 3, payload_kg: .2, rest_pose: [0, 0, 0, 0, 0, 0]}
""")
    o = config_from_args(SimpleNamespace(left=None, right=None, camera=None, arm_options=None), None)['left']['options']
    assert o['dry_run'] is True and o['rate_hz'] == 50 and o['stall_time_s'] == 3 and o['temp_stop_c'] == 60 and o['payload_kg'] == .2
    assert o['temperature_export'] == str(home / 't.jsonl') and o['my_knob'] == 1


def test_nothing_in_the_file_is_needed(home):
    assert settings.load() == {}
    arm = Bench()
    assert arm.stall_guard and arm.jaw_guard and not arm.dry_run and arm.rest_pose is None and arm.max_tool_speed_m_s == .5


def test_unknown_settings_are_named_passed_on_and_never_refused(driver, tmp_path, capsys):
    arm = open_arm(f'{driver}:MyArm', 'left', 'x', connect=False, marks=str(tmp_path / 'm'), my_knob=7, declared_knob=1, jaw_speed=.2)
    assert arm.settings['my_knob'] == 7 and arm.options['my_knob'] == 7 and arm.jaw_speed == .2
    assert len(arm.setting_warnings) == 1 and "'my_knob'" in arm.setting_warnings[0]
    assert 'my_knob' in capsys.readouterr().out


def test_a_plain_adapter_gets_every_setting_too(tmp_path):
    f = tmp_path / 'plain.py'
    f.write_text('from roborun_sdk.embodiment import ArmAdapter\nclass P(ArmAdapter):\n    def __init__(self, bus=None): self.bus = bus\n')
    arm = open_arm(f'{f}:P', 'left', 'b', connect=False, rate_hz=10)
    assert arm.settings == dict(rate_hz=10) and "'rate_hz'" in arm.setting_warnings[0]
    with pytest.raises(SystemExit, match='dry run'):
        open_arm(f'{f}:P', 'left', 'b', connect=False, dry_run=True)


@pytest.mark.parametrize('bad', [dict(stall_time_s=0), dict(gripper_effort=2), dict(rate_hz='fast'), dict(powered_at_connect='maybe'),
                                 dict(ramp_s=-1)])
def test_a_value_that_cannot_work_is_refused_by_name(bad):
    with pytest.raises(ValueError, match=next(iter(bad))):
        Bench(**bad)


@pytest.mark.parametrize('model,role,expected', [('DM4340', 'joint', (70, 80)), ('dm4310', 'gripper', (70, 80)), ('metal', 'joint', (55, 65)),
                                                 ('metal', 'gripper', (45, 55)), ('xl330-m288', 'joint', (55, 65)), ('xl430-w250', 'joint', (57, 67)),
                                                 ('xm430-w350', 'joint', (65, 75)), ('sts3215', 'joint', (55, 65)), ('rs02', 'joint', (70, 80)),
                                                 ('', 'joint', (60, 70))])
def test_temperature_defaults_follow_the_motor_type(model, role, expected):
    assert safety.temperature_limits(model, role) == expected


def test_temperature_limits_can_be_set_per_motor_and_exported(tmp_path):
    out = tmp_path / 'temps.jsonl'
    arm = Bench(joints=7, temp_warn_c=[40] * 8, temp_stop_c=50, temperature_export=str(out)); arm.connect()
    assert wait(lambda: out.exists() and out.read_text())
    assert json.loads(out.read_text().splitlines()[0])['temperatures_c'] == [30.0] * 8
    arm.temps[7] = 45.0
    assert wait(lambda: arm.health()['thermal']['state'] == 'warn') and arm.health()['thermal']['motor'] == 'gripper'
    arm.close()
    with pytest.raises(ValueError, match='temp_warn_c'):
        Bench(temp_warn_c=[1, 2]).connect()


def test_one_bad_temperature_frame_does_not_stop_an_arm():
    guard = safety.ThermalGuard([('a', 'dm4310', 'joint')])
    assert guard.update([200.0], 0.0) is None and guard.update([30.0], .1) is None and guard.state == 'ok'
    assert guard.update([200.0], 1.0) is None and guard.update([200.0], 1.6) == 'stop'


def test_ramp_rate_staleness_and_jaw_step_are_settings():
    arm = Bench(rate_hz=50, command_timeout_s=.05, max_jaw_step=.001, joint_speed_rad_s=.2); arm.connect()
    arm.command([1.0] * 6, 1.0); time.sleep(.3)
    jaws = [j for _, j in arm.writes]
    assert max(abs(b - a) for a, b in zip(jaws, jaws[1:])) <= .001 + 1e-9
    assert arm.q[0] < .5                                   # the command went stale and the arm stopped short
    arm.close()
    assert 10 <= len(arm.writes) <= 25


# -- what a rule can see, and the tool ----------------------------------------------------------------------------------
MODEL = """<mujoco><worldbody><body><joint name="joint1" axis="0 0 1"/><geom size=".01"/>
<body pos="0 0 .1"><joint name="joint2" axis="0 1 0"/><geom size=".01"/><site name="grasp_site" pos="1 0 0"/></body></body></worldbody></mujoco>"""


@pytest.fixture
def two_joint_model(tmp_path):
    pytest.importorskip('mujoco')
    f = tmp_path / 'arm.xml'; f.write_text(MODEL)
    return f


def test_a_rule_sees_joints_velocity_temperature_gripper_and_the_tool(tmp_path, two_joint_model):
    rule = tmp_path / 'r.py'
    rule.write_text('seen = {}\ndef check(side, q, jaw, state):\n    seen.update(state)\n    return state["tool_position_m"][0] < 5 and None\n')
    arm = Bench(joints=2, rules=str(rule), tool_model=dict(xml=str(two_joint_model))); arm.connect()
    arm.command([.1, .1], .5)
    seen = arm._rule.__globals__['seen']
    assert seen['positions_rad'] and seen['velocities_rad_s'] == [0.0, 0.0] and seen['temperatures_c'] == [30.0] * 3
    assert seen['gripper'] == pytest.approx(.5) and seen['thermal']['state'] == 'ok' and seen['side'] == 'left'
    assert len(seen['tool_pose']) == 4 and seen['tool_position_m'][0] > .9
    arm.close()


def test_without_a_model_a_rule_is_told_the_tool_is_unknown(tmp_path):
    rule = tmp_path / 'r.py'; rule.write_text('def check(side, q, jaw, state):\n    assert state["tool_pose"] is None\n')
    arm = Bench(rules=str(rule)); arm.connect(); arm.command([.1] * 6, .5)
    assert arm.health()['tool_speed']['active'] is False and arm.health()['ok']
    arm.close()


def test_tool_speed_is_limited_when_the_model_is_there(two_joint_model):
    arm = Bench(joints=2, tool_model=str(two_joint_model), max_tool_speed_m_s=.1, joint_speed_rad_s=.6); arm.connect()
    assert arm.health()['tool_speed']['active']
    for _ in range(60):
        arm.command([2.0, .1], .5); time.sleep(.01)
    arm.close()
    tool = safety.ToolModel(two_joint_model, 2)
    points = [tool.position(q) for q, _ in arm.writes]
    speeds = [sum((a - b) ** 2 for a, b in zip(p, r)) ** .5 * arm.rate_hz for p, r in zip(points[1:], points)]
    assert max(speeds) <= .1 * 1.02 and max(speeds) > .08


# -- guards through the station -----------------------------------------------------------------------------------------
@pytest.fixture
def station():
    arm = Bench(stall_time_s=.2); arm.connect()
    st, srv = run_station(arm, port=0)
    yield arm, st, srv
    srv.close(); arm.close()


def test_status_and_pose_rows_carry_the_temperature(station):
    arm, st, srv = station
    arm.temps[1] = arm._thermal.warn[1] + 1
    assert wait(lambda: call(srv.url + '/status')['arm_status']['left']['thermal']['state'] == 'warn')
    assert wait(lambda: call(srv.url + '/supervisor/pose')['measured']['left']['thermal']['state'] == 'warn')
    assert call(srv.url + '/supervisor/pose')['measured']['left']['motor_temps_c'][1] == arm.temps[1]
    assert not call(srv.url + '/status')['dead_arms']


def test_a_hot_arm_stops_a_lease_and_is_reported_dead_with_the_reason(station):
    arm, st, srv = station
    arm.temps[0] = 90.0
    assert wait(lambda: call(srv.url + '/status')['dead_arms'] == ['left'])
    status = call(srv.url + '/status')
    assert 'joint1' in status['arm_errors']['left'] and status['last_error']['arm'] == 'left'
    with pytest.raises(urllib.error.HTTPError):
        call(srv.url + '/home', {})
    assert wait(lambda: call(srv.url + '/supervisor/pose')['measured']['left']['fresh'])   # still read (a sample under 50 ms old)


def test_a_blocked_joint_is_reported_and_reset_brings_the_arm_back(station):
    arm, st, srv = station
    arm.stuck.add(2)
    st.left.q_cmd[2] = 1.5
    assert wait(lambda: 'blocked' in (call(srv.url + '/status')['arm_errors'].get('left') or ''))
    assert wait(lambda: abs(st.left.q_cmd[2] - arm.q[2]) < .01)
    arm.stuck.clear()
    call(srv.url + '/reset', {})
    assert wait(lambda: not call(srv.url + '/status')['dead_arms'], 6) and arm.health()['ok']


def test_reset_reopens_only_the_failed_arm_and_says_why_when_it_cannot(station):
    arm, st, srv = station
    other = Bench(); other.connect(); st2, srv2 = run_station(arm2 := other, Bench_connected(), port=0)
    try:
        other.fail = True
        assert wait(lambda: call(srv2.url + '/status')['dead_arms'] == ['left'])
        healthy = st2.right.adapter
        healthy_opened, other_open = healthy.opened, other.open_hardware
        other.open_hardware = lambda: (_ for _ in ()).throw(OSError('still unplugged'))
        call(srv2.url + '/reset', {})
        assert wait(lambda: 'still unplugged' in (call(srv2.url + '/status')['arm_errors'].get('left') or ''), 6)
        assert wait(lambda: call(srv2.url + '/status')['state'] == 'error', 6) and healthy.opened == healthy_opened
        assert healthy.health()['ok']
        other.open_hardware = other_open
        call(srv2.url + '/reset', {})
        assert wait(lambda: not call(srv2.url + '/status')['dead_arms'], 6) and arm2.opened == 2
    finally:
        srv2.close(); other.close(); st2.right.adapter.close()


def Bench_connected():
    arm = Bench(side='right'); arm.connect()
    return arm


def test_a_refused_reset_lets_nothing_go(station):
    arm, st, srv = station
    st.state = 'running'
    limp = []
    arm.limp_hardware = lambda: limp.append(1)
    with pytest.raises(urllib.error.HTTPError):
        call(srv.url + '/reset', {})
    st.state = 'idle'
    assert not limp and not arm._limp


def test_a_plain_adapter_that_keeps_failing_is_reported():
    class Plain(ArmAdapter):
        bad = False
        def read(self):
            if self.bad: raise OSError('cable out')
            return dict(positions_rad=[0.0] * 6, gripper=1.0)
        def command(self, q, jaw): pass
    arm = Plain(); st, srv = run_station(arm, port=0)
    try:
        arm.bad = True
        assert wait(lambda: 'cable out' in (call(srv.url + '/status')['arm_errors'].get('left') or ''))
        arm.bad = False
        assert wait(lambda: not call(srv.url + '/status')['dead_arms'])
    finally:
        srv.close()


# -- parking ------------------------------------------------------------------------------------------------------------
def test_no_rest_pose_means_no_move_at_close():
    arm = Bench(); arm.connect(); time.sleep(.05); arm.close()
    assert arm.close_report['parked'] is False and arm.q == pytest.approx([.1] * 6) and not arm.off


def test_closing_parks_first_and_switches_off_only_when_confirmed():
    arm = Bench(rest_pose=[.2] * 6); arm.connect(); arm.close()
    assert arm.q == pytest.approx([.2] * 6, abs=.002) and arm.off and arm.close_report['motors_off'] in (True, False)


def test_an_arm_that_cannot_reach_its_rest_pose_is_never_switched_off():
    arm = Bench(rest_pose=[.3] * 6, stall_guard=False, park_timeout_s=.4); arm.connect(); arm.stuck.add(1); arm.close()
    assert not arm.off and not arm.close_report['parked'] and 'rest pose' in arm.close_report['why']


def test_a_jaw_holding_something_keeps_the_motors_on():
    arm = Bench(rest_pose=[.1] * 6); arm.connect(); arm._jaw_guard.state = 'gripping'; arm._jaw_guard.held = .5
    arm.close()
    assert arm.close_report['parked'] and not arm.off and 'holding' in arm.close_report['why']


def test_a_hot_arm_with_a_rest_pose_goes_there():
    arm = Bench(rest_pose=[.2] * 6); arm.connect(); arm.temps[0] = 200.0
    assert wait(lambda: arm.q[0] == pytest.approx(.2, abs=.005), 5)
    assert arm.health()['kind'] == 'thermal'
    arm.close()


# -- dry run ------------------------------------------------------------------------------------------------------------
def test_a_dry_run_through_the_station_writes_nothing(capsys):
    arm = Bench(dry_run=True); arm.connect(); st, srv = run_station(arm, port=0)
    try:
        st.left.q_cmd = [.5] * 6
        assert wait(lambda: arm.dry_log and arm.dry_log[-1]['positions_rad'][0] > .15)
        assert not arm.writes and arm.q == [.1] * 6 and call(srv.url + '/status')['arm_status']['left']['dry_run']
    finally:
        srv.close(); arm.close()
    assert 'dry_run_would_send' in capsys.readouterr().out


def test_a_driver_without_a_read_only_open_refuses_a_dry_run():
    class NoDry(Bench):
        open_dry = StreamedArm.open_dry
    with pytest.raises(ArmHardwareError, match='dry run'):
        NoDry(dry_run=True).connect()


# -- the profile, the lock, the process ---------------------------------------------------------------------------------
RIG = dict(arms=dict(left=dict(model='m.xml'), right=dict(model='m.xml', parked=dict(q=[0] * 6))))


def test_arms_must_match_the_profile():
    arm = SimpleNamespace(home=[0.0] * 6)
    assert not check_profile(dict(left=arm), RIG)
    assert 'parked' in check_profile(dict(right=arm), RIG)[0]
    assert 'no left arm' in check_profile(dict(left=arm), dict(arms={}))[0]
    assert '7 joints' in check_profile(dict(left=arm), dict(arms=dict(left=dict(joints=7))))[0]
    assert not check_profile(dict(left=SimpleNamespace(home=[0.0] * 7)), dict(arms=dict(left=dict(joints=7))))


def test_the_lock_names_who_holds_the_arms(home):
    with safety.ArmsLock('the robot station', ['can0']):
        with pytest.raises(safety.ArmsBusy, match=f'the robot station \\(process {os.getpid()}\\)'):
            safety.ArmsLock('arm-check').acquire()
    safety.ArmsLock('arm-check').acquire().release()


def test_two_homes_cannot_share_an_address(home, tmp_path, monkeypatch):
    address = f'virtual://{uuid.uuid4().hex}'
    first = safety.ArmsLock('one', [address]).acquire()
    monkeypatch.setenv('ROBORUN_HOME', str(tmp_path / 'other'))
    with pytest.raises(safety.ArmsBusy):
        safety.ArmsLock('two', [address]).acquire()
    safety.ArmsLock('two', ['elsewhere' + address]).acquire().release()
    first.release()


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def start_station(tmp_path, hw, name='hw.json'):
    file = tmp_path / name; file.write_text(json.dumps(hw))
    port = free_port()
    env = dict(os.environ, ROBORUN_HOME=str(tmp_path), PYTHONPATH=os.pathsep.join([str(ROOT / 'src'), str(ROOT.parent / 'roborun-station/src')]))
    proc = subprocess.Popen([sys.executable, '-m', 'roborun_sdk.cli', 'hardware-station', '--hardware', str(file), '--port', str(port)],
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    return proc, f'http://127.0.0.1:{port}'


def up(url, proc):
    def ready():
        assert proc.poll() is None, proc.stdout.read()
        try:
            return call(url + '/status')
        except OSError:
            return None
    assert wait(ready, 30)


def test_sigterm_parks_and_closes_the_arm_and_a_second_station_is_refused(tmp_path, driver):
    marks = tmp_path / 'marks'
    hw = dict(left=dict(arm=f'{driver}:MyArm', bus='addr-1', options=dict(marks=str(marks), rest_pose=[.2] * 6)))
    proc, url = start_station(tmp_path, hw)
    try:
        up(url, proc)
        second, _ = start_station(tmp_path, hw, 'hw2.json')
        output = second.communicate(timeout=30)[0]
        assert second.returncode not in (0, None) and 'in use by the robot station' in output
        assert marks.read_text().count('open') == 1            # the refused one never touched the arm
        proc.send_signal(signal.SIGTERM)
        assert proc.wait(timeout=20) == 0
    finally:
        proc.kill()
    lines = marks.read_text().split('\n')
    assert lines[1] == 'off' and lines[2] == 'closed at 0.20'


def test_arms_that_do_not_fit_the_profile_are_never_opened(tmp_path, driver):
    marks = tmp_path / 'marks'; rig = tmp_path / 'rig.json'; rig.write_text(json.dumps(dict(arms=dict(left=dict(joints=7)))))
    proc, _ = start_station(tmp_path, dict(profile_rig=str(rig), left=dict(arm=f'{driver}:MyArm', bus='addr-2', options=dict(marks=str(marks)))))
    output = proc.communicate(timeout=30)[0]
    assert proc.returncode != 0 and '7 joints' in output and not marks.exists()
    assert '7 joints' in (tmp_path / 'robot' / 'station-failure.txt').read_text()


def test_the_loaded_profile_gives_the_station_its_rig_and_models(tmp_path):
    (tmp_path / 'config.json').write_text(json.dumps(dict(rig='profiles/x/rig.json')))
    (tmp_path / 'profiles/x').mkdir(parents=True); (tmp_path / 'profiles/x/rig.json').write_text('{}')
    assert hardware.loaded_rig(tmp_path) == dict(profile_rig=str(tmp_path / 'profiles/x/rig.json'), models_dir=str(tmp_path / 'models'))
    assert hardware.loaded_rig(tmp_path / 'nothing') == {}


# -- the powered direction check ----------------------------------------------------------------------------------------
def answers(*rows, default='y'):
    rows = list(rows); asked = []
    def ask(prompt):
        asked.append(prompt)
        return rows.pop(0) if rows else default
    ask.asked = asked
    return ask


def checked(arm):
    arm.survey = lambda: dict(ok=True)
    return arm


def test_nothing_is_powered_without_a_yes():
    arm = checked(Bench())
    report = direction_check.run(arm, ask=answers('n'), say=None)
    assert not report['ok'] and report['stopped'] == 'not confirmed' and arm.opened == 0


def test_a_failed_read_only_check_stops_before_power():
    arm = Bench(); arm.survey = lambda: dict(ok=False, problems=['joint 2 missing'])
    report = direction_check.run(arm, ask=answers(), say=None)
    assert 'joint 2 missing' in report['stopped'] and arm.opened == 0


def test_every_joint_is_asked_for_moves_a_little_and_comes_back():
    arm = checked(Bench())
    ask = answers()
    report = direction_check.run(arm, ask=ask, say=None)
    assert report['ok'] and [r['joint'] for r in report['joints']] == [1, 2, 3, 4, 5, 6, 'gripper']
    assert all(r['result'] == 'ok' for r in report['joints']) and len(ask.asked) == 8
    assert arm.q == pytest.approx([.1] * 6, abs=.01)
    assert max(abs(v - .1) for q, _ in arm.writes for v in q) <= direction_check.STEP_RAD * 1.1
    moving = {i for q, _ in arm.writes for i, v in enumerate(q) if abs(v - .1) > 1e-3}
    assert moving == set(range(6))
    assert all(sum(abs(v - .1) > 1e-3 for v in q) <= 1 for q, _ in arm.writes)      # one joint at a time


def test_a_reversed_joint_stops_the_check_there():
    arm = checked(Bench()); arm.reversed.add(1)
    report = direction_check.run(arm, ask=answers(), say=None)
    assert not report['ok'] and report['joints'][-1]['joint'] == 2 and report['joints'][-1]['result'] == 'reversed'
    assert 'joint 2' in report['stopped'] and len(report['joints']) == 2


def test_a_joint_that_does_not_move_is_reported_and_skip_and_stop_work():
    arm = checked(Bench(stall_guard=False)); arm.stuck.add(0)
    assert direction_check.run(arm, ask=answers(), say=None)['joints'][0]['result'] == 'did not move'
    arm = checked(Bench())
    report = direction_check.run(arm, ask=answers('y', 's', 'q'), say=None)
    assert report['joints'] == [dict(joint=1, result='skipped by you')] and 'before joint 2' in report['stopped']
    assert all(q[0] == pytest.approx(.1, abs=1e-3) for q, _ in arm.writes)


def test_the_prompt_says_where_the_tool_should_go_when_the_model_is_there(two_joint_model):
    arm = checked(Bench(joints=2, tool_model=str(two_joint_model)))
    ask = answers()
    assert direction_check.run(arm, ask=ask, say=None)['ok']
    assert 'toward' in ask.asked[1] and 'mm' in ask.asked[1]


def test_the_command_needs_a_terminal_and_a_free_robot(home, monkeypatch):
    monkeypatch.setattr('sys.stdin', SimpleNamespace(isatty=lambda: False))
    with pytest.raises(SystemExit, match='terminal'):
        direction_check.run(checked(Bench()))
    with safety.ArmsLock('the robot station'):
        with pytest.raises(SystemExit, match='in use'):
            hardware.cmd_check(SimpleNamespace(arm='metal_arm@virtual://x', side='left', options=None, move=True))


# -- the YAM joint limit ------------------------------------------------------------------------------------------------
def test_the_simulated_station_and_the_catalog_agree_on_yam_joint_3():
    from roborun_sdk.arms import catalog
    from roborun_station.core import yam_left_spec, yam_right_spec
    assert catalog()['yam']['limits'][2][1] == 3.14159
    assert yam_left_spec().limits[2][1] == yam_right_spec().limits[2][1] == 3.14159


def test_a_jaw_that_sat_idle_is_not_taken_for_a_blocked_one_when_it_starts_to_close():
    from roborun_sdk.arms.safety import JawGuard
    guard = JawGuard(.03, .3, .9)
    for i in range(100):                                          # a minute at rest, nothing asked of it
        assert guard.update(1.0, 1.0, 1.0, 0.0, i * .6, .02) == 1.0
    now = 60.0; shaped = 1.0; measured = 1.0
    for _ in range(30):                                           # closing: the motor trails the command by more than the margin
        now += .01; shaped -= .006; measured = max(measured - .002, shaped + .04)
        assert guard.update(.4, shaped, measured, 0.0, now, .02) == pytest.approx(shaped) and guard.state == 'free'


def test_a_jaw_held_against_something_is_still_caught():
    from roborun_sdk.arms.safety import JawGuard
    guard = JawGuard(.03, .3, .9); now = 0.0
    while now < .5 and guard.state == 'free':
        now += .01; guard.update(.4, .5, .8, 0.0, now, .02)
    assert guard.state == 'gripping' and .29 <= now <= .35


def test_home_and_reset_on_the_station_need_roborun_s_own_client(home):
    """Every arm, not only the Metal: a program on this computer without the supervisor header cannot Home (opens the jaws) or Reset."""
    from roborun_sdk.embodiment import run_station
    arm = Bench(); arm.connect()
    station, srv = run_station(arm, port=0)
    try:
        for route in ('/home', '/reset'):
            req = urllib.request.Request(srv.url + route, data=b'{}', headers={'Content-Type': 'application/json'})
            with pytest.raises(urllib.error.HTTPError) as refused:
                urllib.request.urlopen(req, timeout=5)
            assert refused.value.code == 403
    finally:
        srv.close(); arm.close()


class LaggyJaw(Bench):
    """A jaw motor that trails its command, as a real one does: each write covers a fifth of the remaining distance."""
    def write_hardware(self, q, jaw):
        super().write_hardware(q, self.jaw)
        self.jaw += (jaw - self.jaw) * 0.2


def test_a_stop_while_closing_holds_the_jaw_where_it_is_not_where_its_command_got_to(home):
    arm = LaggyJaw(jaw_speed=2.0); arm.connect()
    try:
        arm.command([0.1] * 6, 0.0)                       # close (nothing in the jaw)
        time.sleep(0.08)
        at_stop = arm.jaw
        arm.hold()
        time.sleep(0.4)
        assert abs(arm.jaw - at_stop) < 0.02, (at_stop, arm.jaw)   # stopped where it was, not run on to the command
    finally:
        arm.close()
