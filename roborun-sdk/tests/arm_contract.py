"""What every motor-bus arm must do, run against its simulated bus. A family's test file supplies the `rig` fixture."""
import argparse
import json
import time
import urllib.request

import pytest

from roborun_sdk.arms import calibrate as stored, catalog, open_arm
from roborun_sdk.arms.base import ArmHardwareError
from roborun_sdk.embodiment import run_station


def wait(check, seconds=4.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(.01)
    return bool(check())


class Rig:
    """An arm from the catalog, a simulated bus holding it away from Home, and its stored calibration."""

    def __init__(self, arm_id, tmp_path, monkeypatch, make_bus, jaw=(0.4, 1.4), zero=lambda i: .2 + .05 * i, address=None):
        monkeypatch.setenv('ROBORUN_HOME', str(tmp_path))
        self.arm_id, self.row, self.make_bus, self.address = arm_id, catalog()[arm_id], make_bus, address
        self.table = open_arm(arm_id, 'left', 'unset', connect=False)
        self.start, self.zero = {}, {}
        needs_file = self.table.calibration in ('required', 'gripper')
        full = self.table.calibration == 'required'
        for i, motor in enumerate(self.table.motors):
            self.zero[motor.name] = zero(i) if full else motor.zero
        if self.table.gripper.closed is None or needs_file:
            self.jaw = jaw
        else:
            self.jaw = (self.table.gripper.closed, self.table.gripper.open)
        for i, motor in enumerate(self.table.joints):
            if motor is not None:
                q = max(motor.lo + .02, min(motor.hi - .02, self.table.home[i] + .12))
                self.start[motor.name] = self.zero[motor.name] + motor.sign * q
        for motor in self.table.held:
            self.start[motor.name] = self.zero[motor.name] + .05
        self.start[self.table.gripper.name] = (self.jaw[0] + self.jaw[1]) / 2
        self.bus = make_bus(self)
        self.arms = []
        if needs_file:
            self.save_calibration()

    def save_calibration(self):
        data = dict(version=1, arm=self.arm_id, motors={m.name: m.id for m in self.table.motors}, facts=self.facts(),
                    gripper=dict(closed=self.jaw[0], open=self.jaw[1]))
        if self.table.calibration == 'required':
            data['joints'] = {m.name: dict(zero=self.zero[m.name], lo=-10.0, hi=10.0)
                              for m in self.table.motors if m.role != 'gripper'}
        stored.save(self.arm_id, self.address, data)

    def facts(self): return {}

    def arm(self, connect=False, **options):
        arm = open_arm(self.arm_id, 'left', self.address, connect=connect, **options)
        self.arms.append(arm)
        return arm

    def joint_motors(self): return [m for m in self.table.joints if m is not None]

    def moved(self):
        return max(abs(self.bus.pos(name) - start) for name, start in self.start.items())

    def close(self):
        for arm in self.arms:
            try: arm.close()
            except Exception: pass
        self.bus.close()


def first_joint(arm):
    return next((i, m) for i, m in enumerate(arm.joints) if m is not None)


def settle(arm, seconds=None):
    time.sleep(arm.ramp_s + .25 if seconds is None else seconds)


def test_nothing_moves_at_connect_and_every_motor_is_held(rig):
    arm = rig.arm(connect=True)
    settle(arm)
    assert rig.moved() < .02
    assert set(rig.bus.enabled()) >= {m.name for m in rig.table.motors}
    sample = arm.read()
    for motor, q in zip(arm.joints, sample['positions_rad']):
        if motor is not None:
            assert q == pytest.approx(motor.to_joint(rig.start[motor.name]), abs=.02)
    assert sample['gripper'] == pytest.approx(.5, abs=.03)


def test_samples_carry_velocity_temperature_and_gripper_effort(rig):
    arm = rig.arm(connect=True)
    sample = arm.read()
    n = len(arm.home)
    assert len(sample['positions_rad']) == n and len(sample['velocities_rad_s']) == n
    assert len(sample['temperatures_c']) == n + 1 and min(sample['temperatures_c']) > 0
    assert 0 <= sample['gripper_effort'] <= 1


def test_it_follows_commands_through_the_stream(rig):
    arm = rig.arm(connect=True)
    settle(arm)
    index, motor = first_joint(arm)
    target = list(arm.read()['positions_rad'])
    target[index] += -.1 if target[index] + .1 > motor.hi else .1
    def reached():
        arm.command(target, .9)
        now = arm.read()
        return abs(now['positions_rad'][index] - target[index]) < .015 and abs(now['gripper'] - .9) < .03
    assert wait(reached, 6)
    assert rig.bus.pos(motor.name) == pytest.approx(motor.to_motor(target[index]), abs=.02)


def test_commands_past_a_limit_are_clamped(rig):
    arm = rig.arm(connect=True)
    arm.command([50.0] * len(arm.home), 5.0)
    assert arm._target == [hi for _, hi in arm.limits] and arm._jaw_target == 1.0
    with arm._io:
        arm.write_hardware([50.0] * len(arm.home), 5.0)
        for motor in arm.joints:
            if motor is not None:
                assert motor_target_ok(rig, arm, motor)


def motor_target_ok(rig, arm, motor):
    ceiling = motor.to_motor(motor.hi + arm.limit_grace)
    start = rig.start[motor.name]
    def there():                                           # a servo's encoder range may end before the joint limit
        sent = rig.bus.target(motor.name)
        return motor.sign * (sent - start) > 0 and motor.sign * (sent - ceiling) < .01
    assert wait(there, 1)
    return True


def test_a_motor_fault_latches_and_refuses_motion(rig):
    arm = rig.arm(connect=True)
    settle(arm, .1)
    rig.bus.fault(rig.joint_motors()[1].name)
    assert wait(lambda: arm._error is not None)
    with pytest.raises(ArmHardwareError): arm.read()
    with pytest.raises(ArmHardwareError): arm.command([0] * len(arm.home), 0)


def test_a_motor_that_goes_silent_latches(rig):
    arm = rig.arm(connect=True)
    rig.bus.silence(rig.table.gripper.name)
    assert wait(lambda: arm._error is not None)
    assert rig.table.gripper.name in arm._error


def test_a_wrong_motor_is_refused_and_nothing_is_enabled(rig):
    rig.bus.wrong_motor(rig.joint_motors()[0].name)
    with pytest.raises(ArmHardwareError) as caught:
        rig.arm(connect=True)
    assert rig.joint_motors()[0].name in str(caught.value) or 'unexpected motor' in str(caught.value)
    assert not rig.bus.enabled() and not rig.bus.enable_frames()


def test_a_missing_motor_is_refused_and_nothing_is_enabled(rig):
    rig.bus.remove(rig.joint_motors()[-1].name)
    with pytest.raises(ArmHardwareError, match=rig.joint_motors()[-1].name):
        rig.arm(connect=True)
    assert not rig.bus.enabled() and not rig.bus.enable_frames()


def test_survey_reads_only(rig):
    report = rig.arm().survey()
    json.dumps(report, default=str)
    assert report['ok'], report['problems']
    assert set(report['motors']) >= {m.name for m in rig.table.motors}
    assert not rig.bus.enabled() and not rig.bus.enable_frames() and rig.moved() < 1e-3
    rig.bus.wrong_motor(rig.joint_motors()[0].name)
    report = rig.arm().survey()
    assert not report['ok'] and report['problems'] and not rig.bus.enable_frames()


def test_release_keeps_the_arm_up_and_a_new_command_takes_it_back(rig):
    arm = rig.arm(connect=True)
    settle(arm)
    arm.release()
    time.sleep(.6)
    assert rig.moved() < .08
    assert set(rig.bus.enabled()) >= {m.name for m in rig.table.motors}
    now = arm.read()
    index = first_joint(arm)[0]
    def holds():
        arm.command(now['positions_rad'], now['gripper'])
        return abs(arm.read()['positions_rad'][index] - now['positions_rad'][index]) < .02
    assert wait(holds, 2)
    settle(arm)
    assert arm._error is None and not arm._soft


def test_closing_does_not_drop_the_arm(rig):
    arm = rig.arm(connect=True)
    settle(arm)
    arm.close()
    time.sleep(.4)
    assert rig.moved() < .08 and set(rig.bus.enabled()) >= {m.name for m in rig.table.motors}


def _roomy_target(arm, sample, distance=.4):
    """(joint index, motor, a target `distance` away inside the limits)."""
    index, motor = max(((i, m) for i, m in enumerate(arm.joints) if m is not None),
                       key=lambda row: max(row[1].hi - sample['positions_rad'][row[0]], sample['positions_rad'][row[0]] - row[1].lo))
    target = list(sample['positions_rad'])
    target[index] += distance if motor.hi - target[index] > target[index] - motor.lo else -distance
    return index, motor, target


def test_an_arm_left_powered_keeps_holding_and_is_never_sent_zero_gains(rig):
    for motor in rig.table.motors:
        rig.bus.power_on(motor.name)
    arm = rig.arm(connect=True)
    time.sleep(arm.ramp_s + .3)
    assert rig.moved() < .03 and arm._error is None
    assert set(rig.bus.enabled()) >= {m.name for m in rig.table.motors}
    if rig.table.motors and getattr(rig.bus, 'parameters', True):
        assert set(arm.health()['adopted']) == {m.name for m in rig.table.motors}


def test_an_arm_left_powered_can_be_refused_and_stays_as_it_was(rig):
    if not getattr(rig.bus, 'parameters', True):
        pytest.skip('old firmware cannot say a motor is on')
    for motor in rig.table.motors:
        rig.bus.power_on(motor.name)
    with pytest.raises(ArmHardwareError, match='still powered'):
        rig.arm(connect=True, powered_at_connect='refuse')
    assert set(rig.bus.enabled()) >= {m.name for m in rig.table.motors} and rig.moved() < .03


def test_a_motor_that_is_already_too_hot_is_not_enabled(rig):
    rig.bus.set_temp(rig.joint_motors()[0].name, 120)
    with pytest.raises(ArmHardwareError, match='cooled'):
        rig.arm(connect=True)
    assert not rig.bus.enabled() and not rig.bus.enable_frames()


def test_temperature_warns_then_stops_and_the_arm_keeps_holding(rig):
    arm = rig.arm(connect=True)
    settle(arm)
    index = next(i for i, m in enumerate(arm.joints) if m is not None)
    motor = arm.joints[index]
    rig.bus.set_temp(motor.name, arm._thermal.warn[index] + 1)
    assert wait(lambda: arm.health()['thermal']['state'] == 'warn')
    assert arm.health()['ok'] and arm.health()['thermal']['motor'] == motor.name
    arm.command(arm.read()['positions_rad'], arm.read()['gripper'])
    rig.bus.set_temp(motor.name, arm._thermal.stop[index] + 1)
    assert wait(lambda: arm.health()['kind'] == 'thermal')
    assert motor.name in arm.health()['error']
    with pytest.raises(ArmHardwareError, match='cool'):
        arm.command(arm.read()['positions_rad'], .5)
    time.sleep(.3)
    assert arm.read() and rig.moved() < .05 and set(rig.bus.enabled()) >= {m.name for m in rig.table.motors}
    with pytest.raises(ArmHardwareError, match='too hot'):
        arm.reopen()
    rig.bus.set_temp(motor.name, 30)
    assert wait(lambda: arm._thermal.cooled())
    arm.reopen()
    assert arm.health()['ok'] and arm.command(arm.read()['positions_rad'], .5) is None


def test_a_blocked_joint_stops_pushing_and_refuses_more_motion(rig):
    arm = rig.arm(connect=True)
    settle(arm)
    index, motor, target = _roomy_target(arm, arm.read())
    rig.bus.block(motor.name)
    def tripped():
        try:
            arm.command(target, .5)
        except ArmHardwareError:
            pass
        return arm.health()['kind'] == 'stall'
    assert wait(tripped, 5), arm.health()
    assert f'joint {index + 1}' in arm.health()['error']
    assert wait(lambda: abs(rig.bus.target(motor.name) - rig.bus.pos(motor.name)) < .03, 1)
    with pytest.raises(ArmHardwareError, match='blocked'):
        arm.command(target, .5)
    assert arm.read()


def test_a_blocked_jaw_stops_pushing_and_opens_again(rig):
    arm = rig.arm(connect=True, gripper_hold_effort=.5)
    settle(arm)
    jaw = rig.table.gripper.name
    q = arm.read()['positions_rad']
    rig.bus.block(jaw)
    def gripping():
        arm.command(q, 0.0)
        return arm.health()['jaw'] == 'gripping'
    assert wait(gripping, 4), arm.health()
    stroke = abs(arm.gripper.open - arm.gripper.closed)
    time.sleep(.1)
    arm.command(q, 0.0)
    assert abs(rig.bus.target(jaw) - rig.bus.pos(jaw)) < .05 * stroke
    assert arm.health()['ok']
    def released():
        arm.command(q, 1.0)
        return arm.health()['jaw'] == 'free'
    assert wait(released, 2)


def test_a_dry_run_reads_the_arm_and_writes_nothing(rig):
    arm = rig.arm(connect=True, dry_run=True)
    sample = arm.read()
    index, motor, target = _roomy_target(arm, sample, .1)
    def logged():
        arm.command(target, .9)
        return len(arm.dry_log) > 20 and arm.dry_log[-1]['positions_rad'][index] != round(sample['positions_rad'][index], 4)
    assert wait(logged, 3)
    assert arm.health()['dry_run'] and arm.read()['positions_rad'] == pytest.approx(sample['positions_rad'], abs=.01)
    arm.release()
    arm.close()
    assert not rig.bus.enabled() and not rig.bus.enable_frames() and rig.moved() < 1e-3
    assert rig.bus.target(motor.name) == pytest.approx(rig.start[motor.name], abs=1e-2)


def test_a_rest_pose_is_reached_confirmed_and_only_then_the_motors_go_off(rig):
    rig.bus.set_load(False)
    probe = rig.arm(connect=True)
    rest = [m_q for m_q in probe.read()['positions_rad']]
    probe.close()
    index = next(i for i, m in enumerate(probe.joints) if m is not None)
    rest[index] += -.05 if rest[index] + .05 > probe.joints[index].hi else .05
    arm = rig.arm(connect=True, rest_pose=rest, park_speed_rad_s=.5)
    settle(arm)
    arm.close()
    assert arm.close_report == dict(parked=True, motors_off=True, why=None)
    assert not rig.bus.enabled()
    assert arm.joints[index].to_joint(rig.bus.pos(arm.joints[index].name)) == pytest.approx(rest[index], abs=.03)


def test_a_rest_pose_that_does_not_carry_the_arm_keeps_the_motors_on(rig):
    arm = rig.arm(connect=True, home_before_motors_off=True, rest_pose=None)
    settle(arm)
    arm.home = list(arm.read()['positions_rad'])            # already there: only the fade-out decides
    rig.bus.make_heavy()                                    # held firmly it stays up; with the stiffness gone it sinks
    arm.close()
    assert arm.close_report['parked'] and not arm.close_report['motors_off'] and 'carry' in arm.close_report['why']
    assert set(rig.bus.enabled()) >= {m.name for m in rig.table.motors}


def test_reset_reopens_a_failed_adapter_and_status_says_why_until_then(rig):
    arm = rig.arm(connect=True)
    station, server = run_station(arm, port=0)
    try:
        settle(arm)
        jaw = rig.table.gripper.name
        rig.bus.silence(jaw)
        assert wait(lambda: 'left' in _call(server.url + '/status')['dead_arms'])
        status = _call(server.url + '/status')
        assert jaw in status['arm_errors']['left'] and jaw in status['last_error']['message']
        rig.bus.restore(jaw)
        _call(server.url + '/reset', {})
        assert wait(lambda: not _call(server.url + '/status')['dead_arms'], 8), _call(server.url + '/status')
        assert wait(lambda: _call(server.url + '/status')['state'] in ('idle', 'homing'), 8)
        assert arm.read() and arm._error is None
    finally:
        server.close()


def _call(url, body=None):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json', 'X-Manifest-Supervisor': '1'})
    with urllib.request.urlopen(request, timeout=5) as reply:
        return json.load(reply)


def test_a_full_station_pass_home_stop_reset(rig):
    arm = rig.arm(connect=True)
    station, server = run_station(arm, port=0)
    try:
        pose = _call(server.url + '/supervisor/pose')
        assert pose['arms']['left'].get('state') != 'unavailable', pose['arms']['left']
        assert rig.moved() < .02
        _call(server.url + '/home', {})
        def home():
            s = arm.read()
            return max(abs(a - b) for a, b in zip(s['positions_rad'], arm.home)) < .03 and s['gripper'] > .95
        assert wait(home, 25), (arm.read(), arm.home, _call(server.url + '/status'))
        assert wait(lambda: _call(server.url + '/status')['state'] == 'idle', 10)
        _call(server.url + '/stop', {})
        at_home = arm.read()['positions_rad']
        _call(server.url + '/reset', {})
        assert wait(lambda: _call(server.url + '/status')['state'] == 'idle', 8) and arm._error is None
        assert not arm._soft                               # a healthy arm keeps holding through a Reset
        time.sleep(.3)
        assert max(abs(a - b) for a, b in zip(arm.read()['positions_rad'], at_home)) < .08
    finally:
        server.close()


def hand_operator(rig):
    """Answers arm-calibrate's two prompts the way a person would: pose, then sweep and open the gripper."""
    full, jaw, steps = rig.table.calibration == 'required', rig.table.gripper.name, []
    def operator(prompt=''):
        steps.append(prompt)
        if len(steps) == 1:                                # at the zero pose, gripper closed
            rig.bus.move_by_hand(jaw, rig.jaw[0])
            time.sleep(.06)
            return ''
        closed = rig.bus.pos(jaw)                          # calibration may have re-based the encoder at the zero pose
        for k in range(1, 11):
            rig.bus.move_by_hand(jaw, closed + (rig.jaw[1] - rig.jaw[0]) * k / 10)
            if full:
                for motor in rig.joint_motors() + rig.table.held:
                    rig.bus.move_by_hand(motor.name, rig.bus.pos(motor.name) + (.05 if k <= 5 else -.1))
            time.sleep(.05)
        if full:                                           # back to the zero pose before connecting
            for motor in rig.joint_motors() + rig.table.held:
                rig.bus.move_by_hand(motor.name, rig.bus.pos(motor.name) + .25)
            time.sleep(.05)
        return ''
    return operator, steps


def test_the_calibrate_command_records_a_file(rig, monkeypatch):
    if rig.table.calibration == 'none':
        pytest.skip('nothing to calibrate')
    parser = argparse.ArgumentParser()
    stored.add_parser(parser.add_subparsers())
    args = parser.parse_args(['arm-calibrate', f'{rig.arm_id}@{rig.address}'])
    stored.path(rig.arm_id, rig.address).unlink(missing_ok=True)
    operator, _ = hand_operator(rig)
    monkeypatch.setattr('builtins.input', operator)
    assert args.func(args) == 0
    assert stored.load(rig.arm_id, rig.address)['arm'] == rig.arm_id and not rig.bus.enable_frames()


def test_calibration_is_required_and_recorded_by_hand_with_torque_off(rig):
    if rig.table.calibration not in ('required', 'gripper'):
        pytest.skip('this arm runs on the maker\'s zeros')
    stored.path(rig.arm_id, rig.address).unlink()
    with pytest.raises(ArmHardwareError, match='hw calibrate'):
        rig.arm(connect=True)
    assert not rig.bus.enable_frames()
    full = rig.table.calibration == 'required'
    operator, steps = hand_operator(rig)
    stored.record(rig.arm(), ask=operator, say=lambda _: None)
    assert len(steps) == 2 and not rig.bus.enable_frames() and not rig.bus.enabled()
    data = stored.load(rig.arm_id, rig.address)
    assert data['gripper']['open'] - data['gripper']['closed'] == pytest.approx(rig.jaw[1] - rig.jaw[0], abs=.03)
    arm = rig.arm(connect=True)
    assert arm.read()['gripper'] == pytest.approx(1.0, abs=.03)
    if full:
        motor = next(m for m in arm.joints if m is not None)
        assert motor.hi - motor.lo == pytest.approx(.25 + .25 + .04, abs=.12)      # limits come from the sweep
