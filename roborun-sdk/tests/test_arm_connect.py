"""Finding and reaching arm adapters: every address form, every USB ID we claim, and failures in plain words.

The buses are the simulated ones (a virtual CAN wire behind fake USB adapters, a pseudo-terminal, real vcan links in this
test's own network namespace). No adapter is attached.
"""
import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import can
import pytest
import usb.core
from serial.tools import list_ports

from fake_yam_chain import FakeYamChain, SimGsUsbDevice, SimSlcanAdapter
from roborun_sdk import cli
from roborun_sdk.arms import canbus, connect, serial_servo
from roborun_sdk.arms.base import ArmHardwareError
from roborun_sdk.arms.yam import YamArm

RAISED = (0.4, 1.3, 1.1, 0.2, -0.3, 0.2)


@pytest.fixture(autouse=True)
def home(tmp_path, monkeypatch):
    from roborun_sdk import robot
    monkeypatch.setattr(robot, '_state', lambda name: 'stopped')
    monkeypatch.setenv('ROBORUN_HOME', str(tmp_path))
    monkeypatch.setattr(robot, 'HOME', tmp_path)
    monkeypatch.setattr(robot, 'CONFIG', tmp_path / 'robot.json')
    monkeypatch.setattr(canbus.tempfile, 'gettempdir', lambda: str(tmp_path))
    return tmp_path


def attach(monkeypatch, usb_devices=(), ports=()):
    monkeypatch.setattr(usb.core, 'find', lambda find_all=False, **kw: iter(list(usb_devices)))
    monkeypatch.setattr(list_ports, 'comports', lambda: list(ports))


def port(vid=0xad50, pid=0x60c4, serial='S1', device='/dev/ttyACM0', product='CANable', location='1-4:1.0'):
    return SimpleNamespace(vid=vid, pid=pid, serial_number=serial, device=device, product=product, location=location)


def dongle(vid=0x1d50, pid=0x606f, serial='G1', ports=(2, 3), product='candleLight'):
    return SimpleNamespace(idVendor=vid, idProduct=pid, serial_number=serial, product=product, bus=1, port_numbers=ports)


@pytest.fixture
def chain():
    fake = FakeYamChain('connect-test', q=RAISED)
    yield fake
    fake.close()


# -- recognising adapters ---------------------------------------------------------------------------------------------
@pytest.mark.parametrize('ids', sorted(canbus.GS_USB_IDS))
def test_every_gs_usb_id_in_the_kernel_table_is_listed(monkeypatch, ids):
    attach(monkeypatch, [dongle(*ids)])
    assert [r['address'] for r in canbus.list_adapters()] == ['gs_usb://G1']


@pytest.mark.parametrize('ids', sorted(canbus.SLCAN_IDS))
def test_every_canable_slcan_id_is_listed(monkeypatch, ids):
    attach(monkeypatch, ports=[port(*ids)])
    assert [r['address'] for r in canbus.list_adapters()] == ['slcan://S1']


def test_the_stock_st_serial_id_counts_only_when_it_says_it_is_can(monkeypatch):
    attach(monkeypatch, ports=[port(0x0483, 0x5740, 'A', product='CANable clone'),
                               port(0x0483, 0x5740, 'B', device='/dev/ttyACM1', product='STM32 Virtual ComPort')])
    assert [r['serial'] for r in canbus.list_adapters()] == ['A']


def test_an_unrelated_usb_device_is_not_an_adapter(monkeypatch):
    attach(monkeypatch, [dongle(0x1234, 0x5678)], [port(0x1234, 0x5678)])
    assert canbus.list_adapters() == []


# -- addressing: no serial, shared serial, device path ----------------------------------------------------------------
def test_adapters_without_a_serial_are_addressed_by_usb_port(monkeypatch):
    attach(monkeypatch, [dongle(serial=None)], [port(serial=None)])
    rows = {r['address']: r for r in canbus.list_adapters()}
    assert set(rows) == {'gs_usb://port/1-2.3', 'slcan://port/1-4'}
    assert all('no serial' in r['note'] for r in rows.values())


def test_a_shared_serial_is_refused_and_each_adapter_gets_its_port_address(monkeypatch):
    attach(monkeypatch, [dongle(ports=(2, 1)), dongle(ports=(2, 2))])
    assert [r['address'] for r in canbus.list_adapters()] == ['gs_usb://port/1-2.1', 'gs_usb://port/1-2.2']
    with pytest.raises(canbus.AdapterError) as err:
        canbus.open_bus('gs_usb://G1')
    assert 'share this serial' in str(err.value) and 'gs_usb://port/1-2.1' in str(err.value)


def test_two_gs_usb_arms_with_one_serial_each_run_by_port(monkeypatch):
    a, b = FakeYamChain('port-a', q=RAISED), FakeYamChain('port-b', q=RAISED)
    one, two = SimGsUsbDevice(a.name, 'SAME', port_numbers=(1,)), SimGsUsbDevice(b.name, 'SAME', port_numbers=(2,))
    attach(monkeypatch, [one, two])
    try:
        reports = [YamArm(bus=f'gs_usb://port/1-{n}', gripper_limits=x.jaw_limits()).survey() for n, x in ((1, a), (2, b))]
        assert all(r['ok'] for r in reports), reports
    finally:
        for thing in (one, two, a, b):
            thing.close()


def test_slcan_opens_by_device_path_and_shares_its_lock_with_the_serial_address(monkeypatch, chain):
    adapter = SimSlcanAdapter(chain.name)
    attach(monkeypatch, ports=[adapter.port_row()])
    try:
        report = YamArm(bus=f'slcan://{adapter.port}', gripper_limits=chain.jaw_limits()).survey()
        assert report['ok'], report
        bus = canbus.open_bus(f'slcan://{adapter.port}')
        with pytest.raises(canbus.AdapterError, match='in use by process'):
            canbus.open_bus(f'slcan://{adapter.serial}')
        bus.shutdown()
    finally:
        adapter.close()


def test_a_missing_device_path_is_explained():
    with pytest.raises(canbus.AdapterError, match='/dev/ttyNOPE does not exist.*roborun hw detect'):
        canbus.open_bus('slcan:///dev/ttyNOPE')


def test_a_serial_servo_adapter_without_a_serial_is_found_by_port(monkeypatch):
    attach(monkeypatch, ports=[port(0x1a86, 0x7523, None, '/dev/ttyUSB0', 'CH340', '3-1:1.0')])
    assert serial_servo.list_serial_ports()[0]['address'] == 'serial://port/3-1'
    assert serial_servo.resolve_port('serial://port/3-1') == '/dev/ttyUSB0'
    with pytest.raises(ArmHardwareError, match=r'not connected.*serial://port/3-1.*roborun hw detect'):
        serial_servo.resolve_port('serial://GONE')


def test_a_serial_port_has_one_owner():
    master, slave = os.openpty()
    device = os.ttyname(slave)
    held = serial_servo.open_port(f'serial://{device}', 115200)
    try:
        with pytest.raises(ArmHardwareError, match='in use by another program'):
            serial_servo.open_port(f'serial://{device}', 115200)
    finally:
        held.close(); os.close(master); os.close(slave)


# -- permissions --------------------------------------------------------------------------------------------------------
class Locked(SimpleNamespace):
    @property
    def serial_number(self):
        raise ValueError('The device has no langid (permission issue, no string descriptors supported or device error)')
    product = serial_number


def locked():
    return Locked(idVendor=0x1d50, idProduct=0x606f, bus=1, port_numbers=(2, 3))


def test_a_usb_permission_problem_says_so_with_the_udev_rule(monkeypatch):
    attach(monkeypatch, [locked()])
    with pytest.raises(canbus.AdapterError) as err:
        canbus.open_bus('gs_usb://ANYSERIAL')
    text = str(err.value)
    assert 'may not open' in text and 'found 0' not in text and canbus.UDEV_FILE in text and 'udevadm trigger' in text
    assert 'idVendor}=="1d50"' in text and 'idVendor}=="1209"' in text
    row = canbus.list_adapters()[0]
    assert row['address'] == 'gs_usb://port/1-2.3' and row['problem'] == 'permission' and 'udevadm' in row['note']


def test_a_usb_access_error_at_open_is_also_explained(monkeypatch, chain):
    device = SimGsUsbDevice(chain.name)
    device.get_active_configuration = lambda: (_ for _ in ()).throw(usb.core.USBError('Access denied', errno=13))
    device.set_configuration = device.get_active_configuration
    attach(monkeypatch, [device])
    try:
        with pytest.raises(canbus.AdapterError, match='may not open the adapter'):
            canbus.open_bus('gs_usb://SIMGSUSB1')
    finally:
        device.close()


# -- socketcan -------------------------------------------------------------------------------------------------------
@pytest.fixture
def vcan():
    name = 'rrtest0'
    made = subprocess.run(['ip', 'link', 'add', 'dev', name, 'type', 'vcan'], capture_output=True)
    if made.returncode:
        pytest.skip(f'no vcan here: {made.stderr.decode().strip()}')
    yield name
    subprocess.run(['ip', 'link', 'del', name], capture_output=True)


def test_a_real_socketcan_link_that_is_down_is_brought_up_and_carries_frames(vcan):
    assert canbus._link_info(vcan)['state'] == 'down'
    bus = canbus.open_bus(vcan)
    other = can.Bus(interface='socketcan', channel=vcan)
    try:
        assert canbus._link_info(vcan)['state'] == 'up'
        bus.send(can.Message(arbitration_id=0x123, data=b'\x01\x02', is_extended_id=False))
        got = other.recv(1.0)
        assert got is not None and got.arbitration_id == 0x123
        with pytest.raises(canbus.AdapterError, match='in use by process'):
            canbus.open_bus(vcan)
    finally:
        bus.shutdown(); other.shutdown()
    canbus.open_bus(vcan).shutdown()


def test_a_missing_link_names_the_links_that_exist(vcan):
    with pytest.raises(canbus.AdapterError, match=rf'no CAN link.*{vcan}.*roborun hw detect'):
        canbus.open_bus('can9')


def fake_sys(tmp_path, monkeypatch):
    monkeypatch.setattr(canbus.shutil, 'which', lambda _: None)


def ip_answer(flags, state=None, bitrate=None):
    data = dict(state=state, bittiming=dict(bitrate=bitrate))
    return json.dumps([dict(flags=flags, linkinfo=dict(info_kind='can', info_data=data))])


def test_without_the_ip_tool_sysfs_still_answers(tmp_path, monkeypatch):
    link = tmp_path / 'net' / 'can0'; link.mkdir(parents=True)
    (link / 'type').write_text('280\n'); (link / 'flags').write_text('0x1\n')
    monkeypatch.setattr(canbus, 'SYS_NET', tmp_path / 'net')
    def no_ip(*a, **kw):
        raise FileNotFoundError('ip')
    assert canbus.can_links(no_ip)[0]['name'] == 'can0' and canbus.can_links(no_ip)[0]['state'] == 'up'


def test_a_down_link_this_user_cannot_raise_gets_the_exact_command(tmp_path, monkeypatch):
    fake_sys(tmp_path, monkeypatch)
    def run(cmd, **kw):
        return SimpleNamespace(returncode=2 if 'set' in cmd else 0, stdout=ip_answer(['NOARP']))
    with pytest.raises(canbus.AdapterError) as err:
        canbus.ensure_link('can0', run=run, platform='linux')
    assert 'link is down' in str(err.value)
    assert 'sudo ip link set can0 up type can bitrate 1000000' in str(err.value)


@pytest.mark.parametrize('flags,state,rate,why', [(['UP'], 'BUS-OFF', 1000000, 'bus-off'), (['UP'], 'ERROR-ACTIVE', 500000, '500000 bit/s')])
def test_bus_off_or_a_wrong_bitrate_restarts_the_link(tmp_path, monkeypatch, flags, state, rate, why):
    fake_sys(tmp_path, monkeypatch)
    def run(cmd, **kw):
        return SimpleNamespace(returncode=1 if 'set' in cmd else 0, stdout=ip_answer(flags, state, rate))
    with pytest.raises(canbus.AdapterError) as err:
        canbus.ensure_link('can0', run=run, platform='linux')
    assert why in str(err.value)
    assert 'sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 1000000' in str(err.value)


def test_passwordless_sudo_is_used_and_a_password_is_never_asked_for(tmp_path, monkeypatch):
    fake_sys(tmp_path, monkeypatch)
    monkeypatch.setattr(canbus.shutil, 'which', lambda _: '/usr/bin/sudo')
    calls, up = [], []
    def run(cmd, **kw):
        calls.append(cmd)
        if cmd[0] == 'sudo':
            up.append(1)
            return SimpleNamespace(returncode=0, stdout='')
        if 'set' in cmd:
            return SimpleNamespace(returncode=2, stdout='')
        return SimpleNamespace(returncode=0, stdout=ip_answer(['UP'] if up else [], 'ERROR-ACTIVE', 1000000))
    assert canbus.ensure_link('can0', run=run, platform='linux')['state'] == 'up'
    assert all(c[:2] == ['sudo', '-n'] for c in calls if c[0] == 'sudo')


def test_socketcan_on_macos_points_at_the_usb_addresses():
    with pytest.raises(canbus.AdapterError, match='only on Linux.*roborun hw detect'):
        canbus.ensure_link('can0', platform='darwin')


def test_an_adapter_held_by_slcand_names_the_link_to_use(tmp_path, monkeypatch):
    target = tmp_path / 'ttyACM0'; target.write_text('')
    proc = tmp_path / 'proc' / '4242' / 'fd'
    proc.mkdir(parents=True)
    (proc / '3').symlink_to(target)
    (proc.parent / 'cmdline').write_bytes(b'slcand\0-o\0-s8\0' + str(target).encode() + b'\0slcan0\0')
    monkeypatch.setattr(canbus, 'PROC', tmp_path / 'proc')
    with pytest.raises(canbus.AdapterError, match=r'slcand \(process 4242\).*slcan0.*use the address slcan0'):
        canbus.open_bus(f'slcan://{target}')


# -- roborun adapters ------------------------------------------------------------------------------------------------
def test_the_command_prints_a_paste_ready_address_for_everything(monkeypatch, capsys, tmp_path):
    attach(monkeypatch, [dongle(), dongle(0x1209, 0x2323, None, (5,))],
           [port(), port(0x0403, 0x6014, 'FT1', '/dev/ttyUSB0', 'FT232H', '3-1:1.0'), SimpleNamespace(
               vid=None, pid=None, serial_number=None, device='/dev/ttyS0', product=None, location=None)])
    monkeypatch.setattr(canbus, '_link_names', lambda run=None: ['can0'])
    monkeypatch.setattr(canbus, '_link_info', lambda name, run=None: dict(state='down', kind='can', bitrate=None))
    monkeypatch.setattr(sys, 'platform', 'linux')
    assert cli.main(['adapters']) == 0
    out = capsys.readouterr().out
    for address in ('can0', 'gs_usb://G1', 'gs_usb://port/1-5', 'slcan://S1', 'serial://FT1'):
        assert address in out
    assert 'ttyS0' not in out and 'the link is down' in out and 'ARM@ADDRESS' in out
    assert cli.main(['arms', '--detect', '--json']) == 0
    assert {r['kind'] for r in json.loads(capsys.readouterr().out)['adapters']} == {'socketcan', 'gs_usb', 'slcan', 'serial'}


def test_the_command_works_on_macos_without_socketcan(monkeypatch, capsys):
    attach(monkeypatch, [dongle()], [port(device='/dev/tty.usbmodem1', location='20-1')])
    monkeypatch.setattr(sys, 'platform', 'darwin')
    monkeypatch.setattr(canbus, 'can_links', lambda: pytest.fail('macOS has no CAN links'))
    assert cli.main(['adapters', '--json']) == 0
    assert {r['address'] for r in json.loads(capsys.readouterr().out)['adapters']} == {'gs_usb://G1', 'slcan://S1'}


def test_an_empty_computer_says_so(monkeypatch, capsys, tmp_path):
    attach(monkeypatch)
    monkeypatch.setattr(canbus, '_link_names', lambda run=None: [])
    assert cli.main(['adapters']) == 0
    assert 'no CAN or serial adapters found' in capsys.readouterr().out


# -- plain failures from the commands ---------------------------------------------------------------------------------
def message(argv):
    with pytest.raises(SystemExit) as out:
        cli.main(argv)
    assert isinstance(out.value.code, str), out.value.code
    assert 'Traceback' not in out.value.code
    return out.value.code


def test_arm_check_on_an_absent_adapter_is_one_plain_message(monkeypatch):
    attach(monkeypatch)
    text = message(['arm-check', 'yam@gs_usb://NOPE'])
    assert 'gs_usb://NOPE' in text and 'not connected' in text and 'roborun hw detect' in text


def test_arm_calibrate_on_an_absent_adapter_is_one_plain_message(monkeypatch):
    attach(monkeypatch)
    text = message(['arm-calibrate', 'so101@serial://NOPE'])
    assert 'serial://NOPE' in text and 'not connected' in text and 'roborun hw detect' in text


def test_bad_arm_settings_are_named_not_thrown(monkeypatch, tmp_path):
    options = tmp_path / 'o.json'; options.write_text('{"gripper_type": "anvil"}')
    attach(monkeypatch)
    text = message(['arm-check', 'yam@can0', '--options', str(options)])
    assert 'yam@can0' in text and 'anvil' in text and 'linear_4310' in text and 'robot.yaml' in text


@pytest.mark.parametrize('error,words', [
    (ImportError('x', name='piper_sdk'), 'piper_sdk'), (PermissionError(13, 'denied', '/dev/x'), 'permission denied'),
    (ArmHardwareError('motor 3 has not answered'), 'powered'), (RuntimeError('boom'), 'roborun hw detect')])
def test_every_failure_names_the_arm_and_a_next_step(error, words):
    text = connect.explain(error, 'yam@can0')
    assert 'yam@can0' in text and words in text and 'Next:' in text


def up_args(**kw):
    base = dict(left=None, right=None, camera=None, arm_options=None, sim=None)
    return SimpleNamespace(**dict(base, **kw))


def test_up_stops_before_starting_anything_when_an_adapter_is_missing(monkeypatch):
    attach(monkeypatch)
    started = []
    with pytest.raises(SystemExit) as out:
        connect._wrap_up(started.append)(up_args(left='yam@gs_usb://GONE', right='so101@serial://ALSOGONE'))
    assert not started and 'left arm' in out.value.code and 'right arm' in out.value.code and 'GONE' in out.value.code


def test_up_on_a_running_robot_does_not_trip_over_its_own_adapters(monkeypatch):
    from roborun_sdk import robot
    attach(monkeypatch)
    monkeypatch.setattr(robot, '_state', lambda name: 'running')
    started = []
    connect._wrap_up(started.append)(up_args(left='yam@gs_usb://HELD_BY_THE_STATION'))
    assert len(started) == 1


def test_up_shows_why_the_station_failed_and_never_a_traceback(monkeypatch, home):
    attach(monkeypatch, [dongle()])
    def station_died(_a):
        (home / 'robot').mkdir(exist_ok=True)
        (home / 'robot' / connect.FAILURE_FILE).write_text('gs_usb://G1: it fell over. Next: replug it\n')
        raise SystemExit('the arms did not come up: see the log')
    with pytest.raises(SystemExit) as out:
        connect._wrap_up(station_died)(up_args(left='yam@gs_usb://G1'))
    assert out.value.code == 'gs_usb://G1: it fell over. Next: replug it'
    def bug(_a):
        raise KeyError('nothing')
    with pytest.raises(SystemExit, match='roborun up failed'):
        connect._wrap_up(bug)(up_args(left='yam@gs_usb://G1'))


def test_the_station_records_its_failure_for_up(monkeypatch, home, tmp_path):
    attach(monkeypatch)
    hardware = tmp_path / 'hw.json'
    hardware.write_text(json.dumps(dict(left=dict(arm='yam', bus='gs_usb://GONE'))))
    with pytest.raises(SystemExit):
        connect._wrap_station(lambda a: 0)(SimpleNamespace(hardware=str(hardware)))
    assert 'gs_usb://GONE' in (home / 'robot' / connect.FAILURE_FILE).read_text()


# -- YAM grippers -----------------------------------------------------------------------------------------------------
@pytest.mark.parametrize('gripper', ['crank_4310', 'linear_3507', 'linear_4310', 'flexible_4310', 'no_gripper'])
def test_every_upstream_motorized_gripper_connects_on_the_simulated_chain(gripper):
    fake = FakeYamChain(f'grip-{gripper}', q=RAISED, gripper=gripper)
    arm = YamArm(bus=fake.address, gripper_type=gripper, gripper_limits=fake.jaw_limits() if gripper != 'no_gripper' else None)
    try:
        assert arm.survey()['ok']
        arm.connect()
        assert arm.read()['positions_rad'] == pytest.approx(list(RAISED), abs=0.01)
        arm.close()
    finally:
        fake.close()


@pytest.mark.parametrize('handle', ['yam_teaching_handle_v1', 'yam_teaching_handle_v2'])
def test_a_teaching_handle_is_an_end_with_no_gripper(handle):
    arm = YamArm(bus='virtual://x', gripper_type=handle)
    assert arm.handle and not arm.has_jaw


def test_ethercat_is_refused_in_plain_words():
    with pytest.raises(ArmHardwareError, match='EtherCAT.*roborun hw detect'):
        YamArm(bus='ecat:eth0')
