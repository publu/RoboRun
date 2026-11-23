"""Connection parity: Damiao control-mode repair and the position-force jaw, CAN bus load, and `roborun motor`.

Everything runs on the simulated buses; nothing here reaches hardware.
"""
import argparse
import os
import signal
import uuid

import pytest

pytest.importorskip('can')

from arm_contract import Rig, settle, wait  # noqa: E402
from fake_damiao import FakeDamiaoBus  # noqa: E402
from roborun_sdk.arms import canbus, motor_tools  # noqa: E402
from roborun_sdk.arms.base import ArmHardwareError  # noqa: E402
from roborun_sdk.arms.safety import ArmsLock  # noqa: E402


@pytest.fixture
def metal(tmp_path, monkeypatch):
    def make_bus(r):
        rows = [dict(m.spec, name=m.name, id=m.id) for m in r.table.motors]
        return FakeDamiaoBus(r.address[len('virtual://'):], rows, r.start)
    r = Rig('metal_arm', tmp_path, monkeypatch, make_bus, address=f'virtual://{uuid.uuid4().hex}')
    yield r
    r.close()


def joint(rig, index=1):
    return rig.bus.by_name[rig.joint_motors()[index].name]


def answer(value):
    asked = []
    def yes(question, default):
        asked.append(question)
        return value
    yes.asked = asked
    return yes


# -- B1: control mode ------------------------------------------------------------------------------------------------
def test_a_wrong_mode_is_found_read_only_and_offered(metal):
    joint(metal).mode = 2
    report = metal.arm().survey()
    assert not report['ok'] and report['repairs'][0]['now'] == 2 and report['repairs'][0]['to'] == 1
    assert any('--repair' in p for p in report['problems'])
    assert not metal.bus.writes() and not metal.bus.enable_frames()


def test_saying_no_writes_nothing(metal):
    joint(metal).mode = 3
    said = []
    assert motor_tools.offer_repairs(metal.arm(), answer(False), say=said.append) is False
    assert 'Nothing was written.' in said and not metal.bus.writes() and joint(metal).mode == 3


def test_the_repair_writes_reads_back_saves_and_the_arm_then_connects(metal):
    motor = joint(metal)
    motor.mode = motor.flash['mode'] = 2
    said = []
    assert motor_tools.offer_repairs(metal.arm(), answer(True), say=said.append)
    assert motor.mode == 1 and motor.flash['mode'] == 1 and len(metal.bus.saves()) == 1
    assert not metal.bus.enable_frames() and not metal.bus.enabled()       # nothing had torque during the write
    assert any('saved' in s for s in said) and any('flash' in s for s in said)
    metal.bus.power_cycle()
    assert motor.mode == 1
    metal.arm(connect=True)
    assert set(metal.bus.enabled()) >= {m.name for m in metal.table.motors}


def test_without_saving_it_is_gone_after_a_power_cycle(metal):
    motor = joint(metal)
    motor.mode = motor.flash['mode'] = 2
    assert motor_tools.offer_repairs(metal.arm(), answer(True), say=lambda _: None, save=False)
    assert motor.mode == 1 and not metal.bus.saves()
    metal.bus.power_cycle()
    assert motor.mode == 2


def test_an_enabled_motor_stops_the_repair_before_anything_is_written(metal):
    joint(metal).mode = 2
    metal.bus.power_on(metal.joint_motors()[0].name)
    with pytest.raises(ArmHardwareError, match='enabled'):
        motor_tools.offer_repairs(metal.arm(), answer(True), say=lambda _: None)
    assert not metal.bus.writes()


def test_a_value_that_does_not_read_back_is_not_saved(metal):
    motor = joint(metal)
    motor.mode = motor.flash['mode'] = 2
    motor.refuse_writes = True
    with pytest.raises(ArmHardwareError, match='reads back as 2'):
        motor_tools.offer_repairs(metal.arm(), answer(True), say=lambda _: None)
    assert not metal.bus.saves() and motor.flash['mode'] == 2 and not metal.bus.enable_frames()


def test_no_repair_is_offered_while_another_motor_is_in_doubt(metal):
    joint(metal).mode = 2
    metal.bus.silence(metal.joint_motors()[3].name)
    said = []
    assert not motor_tools.offer_repairs(metal.arm(), answer(True), say=said.append)
    assert any('other problems' in s for s in said) and not metal.bus.writes()


def test_ctrl_c_during_the_write_waits_for_the_read_back(metal):
    motor = joint(metal)
    motor.mode = 2
    arm = metal.arm()
    real = arm._exchange
    def interrupted(m, command, *args, **kw):
        if command == 0x55:
            os.kill(os.getpid(), signal.SIGINT)                 # Ctrl-C lands right as the write goes out
        return real(m, command, *args, **kw)
    arm._exchange = interrupted
    with pytest.raises(KeyboardInterrupt):
        motor_tools.offer_repairs(arm, answer(True), say=lambda _: None)
    assert motor.mode == 1 and motor.flash['mode'] == 1          # finished, read back and saved before stopping
    assert not metal.bus.enable_frames()


def test_ctrl_c_at_the_question_writes_nothing(metal):
    joint(metal).mode = 2
    def interrupted(question, default):
        raise KeyboardInterrupt
    with pytest.raises(KeyboardInterrupt):
        motor_tools.offer_repairs(metal.arm(), interrupted, say=lambda _: None)
    assert not metal.bus.writes()


def test_arm_check_repair_asks_in_the_terminal_and_refuses_without_one(metal, monkeypatch):
    from roborun_sdk.arms import hardware
    joint(metal).mode = 2
    args = argparse.Namespace(arm=f'metal_arm@{metal.address}', side='left', options=None, move=False, repair=True,
                              yes=False, no_save=False)
    monkeypatch.setattr('sys.stdin.isatty', lambda: False)
    with pytest.raises(SystemExit, match='--yes'):
        hardware.cmd_check(args)
    assert not metal.bus.writes()
    monkeypatch.setattr('sys.stdin.isatty', lambda: True)
    monkeypatch.setattr('builtins.input', lambda text: 'no')
    assert hardware.cmd_check(args) == 1 and not metal.bus.writes()
    monkeypatch.setattr('builtins.input', lambda text: 'yes')
    assert hardware.cmd_check(args) == 0 and joint(metal).mode == 1


def test_a_joint_in_position_force_mode_is_refused_not_driven(metal):
    joint(metal, 0).mode = 4
    with pytest.raises(ArmHardwareError, match='position-force'):
        metal.arm(connect=True)
    assert not metal.bus.enabled() and not metal.bus.writes()


def test_a_jaw_in_position_force_mode_is_driven_in_that_mode(metal):
    jaw = metal.bus.by_name[metal.table.gripper.name]
    jaw.mode = 4
    arm = metal.arm(connect=True)
    settle(arm)
    assert metal.moved() < .02 and jaw.enabled
    def reached():
        arm.command(arm.read()['positions_rad'], .85)
        return abs(arm.read()['gripper'] - .85) < .03
    assert wait(reached, 6)
    frames = metal.bus.pos_force_frames()
    assert frames and all(f[0] == 0x300 + jaw.id for f in frames)
    assert 0 < jaw.current <= 1 and not metal.bus.writes()      # its mode was never changed


def test_the_survey_says_how_a_position_force_jaw_is_driven(metal):
    metal.bus.by_name[metal.table.gripper.name].mode = 4
    report = metal.arm().survey()
    assert report['ok'] and 'position-force' in report['motors'][metal.table.gripper.name]['drive']


# -- CAN bus load ----------------------------------------------------------------------------------------------------
def test_bus_load_arithmetic():
    assert canbus.load_warning('can0', 2 * 7 * 250) is None                 # a YAM at i2rt's 250 Hz: ~46 %
    text = canbus.load_warning('can0', 2 * 8 * 500)                         # 8 motors at 500 Hz: over 100 %
    assert text and 'rate_hz' in text and '1000 kbit/s' in text


def test_a_stream_that_saturates_the_bus_is_said_at_connect(metal, capsys):
    arm = metal.arm(connect=True, rate_hz=600)
    assert any('CAN bus' in w for w in arm.setting_warnings)
    assert 'can_bus_load' in capsys.readouterr().out
    assert metal.arm(rate_hz=600).survey()['warning'].startswith(metal.address)


def test_two_arms_on_one_adapter_are_counted_together(metal, capsys):
    from roborun_sdk.arms import connect
    one = dict(arm='metal_arm', bus=metal.address, options=dict(rate_hz=300))
    assert connect.bus_load_warnings(dict(left=one)) == []
    warnings = connect.bus_load_warnings(dict(left=one, right=one))
    assert len(warnings) == 1 and 'left arm' in warnings[0] and 'right arm' in warnings[0]


# -- roborun motor ---------------------------------------------------------------------------------------------------
def motor_args(action, bus, **kw):
    base = dict(action=action, bus=bus, baud=None, ids=None, json=False, yes=False, no_save=False)
    return argparse.Namespace(**dict(base, **kw))


def run_motor(args, replies=('yes',)):
    said, replies = [], list(replies)
    code = motor_tools.cmd_motor(args, ask=lambda text: said.append(text) or replies.pop(0), say=said.append)
    return code, said


def test_damiao_list_is_read_only(metal):
    code, said = run_motor(motor_args('list', f'damiao@{metal.address}', ids='1-8'))
    assert code == 0 and '7 damiao motor(s)' in said[0]
    assert not metal.bus.writes() and not metal.bus.enable_frames()


def test_damiao_set_id_moves_the_motor_and_its_feedback_id_and_saves(metal):
    motor = metal.bus.by_name[metal.table.gripper.name]
    code, said = run_motor(motor_args('set-id', f'damiao@{metal.address}', id=7, new_id=9))
    assert code == 0 and motor.id == 9 and motor.recv == 0x19 and motor.flash['id'] == 9
    assert 'flash' in said[0] and not metal.bus.enable_frames()


def test_damiao_set_id_refuses_a_taken_id_and_a_powered_motor(metal):
    with pytest.raises(SystemExit, match='already answers'):
        run_motor(motor_args('set-id', f'damiao@{metal.address}', id=7, new_id=6))
    metal.bus.power_on(metal.table.gripper.name)
    with pytest.raises(SystemExit, match='enabled'):
        run_motor(motor_args('set-id', f'damiao@{metal.address}', id=7, new_id=9))
    assert not metal.bus.writes()


def test_damiao_set_zero_reads_back_zero(metal):
    code, _ = run_motor(motor_args('set-zero', f'metal_arm@{metal.address}', id=3))
    assert code == 0 and not metal.bus.enable_frames()
    code, said = run_motor(motor_args('list', f'damiao@{metal.address}', ids='3', json=True))
    import json
    assert abs(json.loads(said[0])['motors'][0]['position_rad']) < .001


def test_saying_no_or_ctrl_c_at_the_question_writes_nothing(metal):
    code, said = run_motor(motor_args('set-zero', f'damiao@{metal.address}', id=3), replies=('no',))
    assert code == 1 and said[-1] == 'Nothing was written.'
    def interrupted(text):
        raise KeyboardInterrupt
    assert motor_tools.cmd_motor(motor_args('set-id', f'damiao@{metal.address}', id=7, new_id=9), ask=interrupted,
                                 say=lambda _: None) == 130
    assert not [f for f in metal.bus.frames if f[1][-1] == 0xfe] and not metal.bus.writes()


def test_a_bus_held_by_a_station_is_refused(metal):
    with ArmsLock('the robot station', [metal.address]):
        with pytest.raises(SystemExit, match='in use by the robot station'):
            run_motor(motor_args('list', f'damiao@{metal.address}'))


def test_a_write_that_does_not_read_back_says_so(metal, monkeypatch):
    monkeypatch.setattr(motor_tools.DamiaoTool, 'zero_tolerance_rad', -1.0)    # every read-back now disagrees
    with pytest.raises(ArmHardwareError, match='not 0'):
        run_motor(motor_args('set-zero', f'damiao@{metal.address}', id=3))


@pytest.fixture
def maker(tmp_path, monkeypatch):
    from fake_robstride import FakeRobStrideBus
    def make_bus(r):
        rows = [dict(m.spec, name=m.name, id=m.id) for m in r.table.motors]
        return FakeRobStrideBus(r.address[len('virtual://'):], rows, r.start)
    r = Rig('maker_arm', tmp_path, monkeypatch, make_bus, address=f'virtual://{uuid.uuid4().hex}')
    yield r
    r.close()


def test_robstride_list_zero_and_no_id_change(maker):
    code, said = run_motor(motor_args('list', f'robstride@{maker.address}', ids='1-8'))
    assert code == 0 and f'{len(maker.table.motors)} robstride motor(s)' in said[0]
    name = maker.joint_motors()[1].name
    code, _ = run_motor(motor_args('set-zero', f'maker_arm@{maker.address}', id=maker.bus.by_name[name].id))
    assert code == 0 and maker.bus.pos(name) == 0.0 and not maker.bus.enable_frames()
    with pytest.raises(SystemExit, match='no documented id change'):
        run_motor(motor_args('set-id', f'robstride@{maker.address}', id=1, new_id=9))


def test_robstride_zero_is_refused_on_a_powered_motor(maker):
    name = maker.joint_motors()[1].name
    maker.bus.power_on(name)
    before = maker.bus.pos(name)
    with pytest.raises(SystemExit, match='enabled'):
        run_motor(motor_args('set-zero', f'robstride@{maker.address}', id=maker.bus.by_name[name].id))
    assert maker.bus.pos(name) == pytest.approx(before, abs=.05)


@pytest.fixture
def servos(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBORUN_HOME', str(tmp_path))      # the arms lock is per Roborun home: never the user's, never shared
    from fake_feetech import FakeFeetechBus
    from fake_dynamixel import FakeDynamixelBus
    buses = dict(feetech=FakeFeetechBus({1: 1000, 2: 3000}, load=False).start(),
                 dynamixel=FakeDynamixelBus({1: (1000, 1060, None), 2: (3000, 1060, None)}, load=False).start())
    yield buses
    for bus in buses.values():
        bus.close()


@pytest.mark.parametrize('family', ['feetech', 'dynamixel'])
def test_servo_list_set_id_and_zero(servos, family):
    fake = servos[family]
    spec = f'{family}@{fake.address}'
    code, said = run_motor(motor_args('list', spec, ids='0-4'))
    assert code == 0 and '2 ' in said[0]
    code, _ = run_motor(motor_args('set-id', spec, id=2, new_id=5))
    assert code == 0 and set(fake.servos) == {1, 5}
    code, said = run_motor(motor_args('set-zero', spec, id=5))
    assert code == 0 and '"position": 2047' in said[-1]
    assert not fake.torque_writes_on()


@pytest.mark.parametrize('family', ['feetech', 'dynamixel'])
def test_servo_writes_are_refused_with_torque_on(servos, family):
    fake = servos[family]
    fake.servos[1].mem[40 if family == 'feetech' else 64] = 1
    with pytest.raises(SystemExit, match='torque on'):
        run_motor(motor_args('set-zero', f'{family}@{fake.address}', id=1))
    with pytest.raises(SystemExit, match='already answers'):
        run_motor(motor_args('set-id', f'{family}@{fake.address}', id=2, new_id=1))


def test_the_motor_command_runs_from_the_cli(metal, capsys):
    from roborun_sdk import cli
    assert cli.main(['motor', 'list', f'damiao@{metal.address}', '--ids', '1-3']) in (0, None)
    assert '3 damiao motor(s)' in capsys.readouterr().out
    with pytest.raises(SystemExit, match='terminal'):
        cli.main(['motor', 'set-zero', f'damiao@{metal.address}', '3'])     # no terminal under pytest, no --yes
    assert not [f for f in metal.bus.frames if f[1][-1] == 0xfe]
