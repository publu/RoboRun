"""YamArm on the real vendored i2rt stack, against a simulated Damiao chain (tests/fake_yam_chain.py)."""
import json
import sys
import time
import urllib.request
import uuid
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('can')
pytest.importorskip('mujoco')
pytest.importorskip('usb')

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fake_yam_chain import JAW_CLOSED, JAW_STROKE, FakeYamChain, SimGsUsbDevice, SimSlcanAdapter  # noqa: E402
from roborun_sdk.arms import canbus, catalog, open_arm  # noqa: E402
from roborun_sdk.arms.base import ArmHardwareError  # noqa: E402
from roborun_sdk.arms.yam import YamArm  # noqa: E402
from roborun_sdk.embodiment import run_station  # noqa: E402

RAISED = (0.4, 1.3, 1.1, 0.2, -0.3, 0.2)
ENABLE, DISABLE = 0xFC, 0xFD


def wait(check, seconds=5.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if check():
            return True
        time.sleep(0.01)
    return bool(check())


def excursion(fake, since):
    rows = np.array([row for t, row in list(fake.trace) if t >= since])
    return np.abs(rows - rows[0]).max(axis=0)


def commands_of(fake, motor, since):
    return [row for row in list(fake.mit) if row[1] == motor and row[0] >= since]


@pytest.fixture
def bench(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBORUN_HOME', str(tmp_path))
    interval = sys.getswitchinterval()
    sys.setswitchinterval(1e-4)
    made = []

    class Bench:
        def chain(self, **kw):
            fake = FakeYamChain(f'yam-{uuid.uuid4().hex[:8]}', **kw)
            made.append(fake)
            return fake

        def arm(self, fake, connect=True, cls=YamArm, **kw):
            kw.setdefault('gripper_limits', fake.jaw_limits())
            arm = cls(bus=kw.pop('bus', fake.address), **kw)
            made.append(arm)
            if connect:
                arm.connect()
            return arm

        def keep(self, thing):
            made.append(thing)
            return thing

    yield Bench()
    for thing in reversed(made):
        try:
            thing.close()
        except Exception:
            pass
    sys.setswitchinterval(interval)


# -- the plant itself must be able to catch a bad driver ---------------------------------------------------------
def test_the_simulated_arm_falls_when_nothing_holds_it(bench):
    import can
    fake = bench.chain(q=RAISED)
    bus = bench.keep(can.Bus(interface='virtual', channel=fake.name, receive_own_messages=False))
    for motor in range(1, 8):
        bus.send(can.Message(arbitration_id=motor, data=bytes([0xFF] * 7 + [ENABLE]), is_extended_id=False))
    time.sleep(0.05)
    start = fake.positions()
    time.sleep(0.2)
    assert abs(fake.positions()[1] - start[1]) > 0.02


# -- connect ------------------------------------------------------------------------------------------------------
def test_nothing_moves_at_connect_and_read_reports_the_arm(bench):
    fake = bench.chain(q=RAISED, jaw=-1.0)
    start = time.monotonic()
    before = fake.positions()
    arm = bench.arm(fake)
    time.sleep(1.0)
    moved = excursion(fake, start)
    assert moved[:6].max() < 0.004, moved
    assert moved[6] < 0.01, moved
    sample = arm.read()
    assert sample['positions_rad'] == pytest.approx(before, abs=0.004)
    assert sample['gripper'] == pytest.approx((JAW_CLOSED - (-1.0)) / JAW_STROKE, abs=0.01)
    assert len(sample['velocities_rad_s']) == 6 and max(map(abs, sample['velocities_rad_s'])) < 0.2
    assert sample['temperatures_c'] == [35.0] * 7
    assert 0.0 <= sample['gripper_effort'] <= 1.0
    assert len(fake.special_frames(ENABLE)) == 7 and not fake.special_frames(DISABLE)


def test_without_the_gravity_hold_the_stock_startup_lets_the_arm_sag(bench):
    """Why the vendored driver has gravity_hold_from_first_cycle: this is the same start without it."""
    from roborun_sdk.arms.yam import get_yam_robot
    fake = bench.chain(q=RAISED)
    start = time.monotonic()
    try:
        bench.keep(get_yam_robot(channel=fake.address, gripper_limits_override=np.array(fake.jaw_limits())))
        time.sleep(0.3)
    except RuntimeError as exc:                                               # it fell far enough to trip the joint limits
        assert 'limit' in str(exc)
    assert excursion(fake, start)[:6].max() > 0.02


def test_a_motor_that_ignores_the_status_request_still_starts_held(bench):
    fake = bench.chain(q=RAISED)
    fake.answers_status_request = False
    arm = bench.arm(fake)
    time.sleep(0.3)
    assert arm.read()['positions_rad'] == pytest.approx(fake.positions(), abs=0.01)
    assert commands_of(fake, 2, 0.0)[0][6] != pytest.approx(0.0, abs=0.5)     # the first command already carries gravity


def test_the_startup_checks_ran_before_any_motor_was_enabled(bench):
    fake = bench.chain()
    bench.arm(fake)
    enabled_at = min(f[0] for f in fake.special_frames(ENABLE))
    reads = [f for f in fake.heard() if f[1] == 0x7FF and f[2][2] == 0x33]
    assert {f[2][3] for f in reads} >= {20, 10, 21, 22, 23}                   # Gr, CTRL_MODE, PMAX, VMAX, TMAX
    assert {f[2][0] for f in reads} == set(range(1, 8))
    assert all(f[0] < enabled_at for f in reads)


def test_a_wrong_motor_type_is_refused_before_anything_is_enabled(bench):
    fake = bench.chain()
    fake.motors[2].reported_kind = 'DM4310'
    with pytest.raises(ArmHardwareError, match='motor type check failed'):
        bench.arm(fake)
    assert not fake.special_frames(ENABLE) and not list(fake.mit)
    assert 0x55 not in fake.register_commands() and 0xAA not in fake.register_commands()


def test_a_wrong_control_mode_is_repaired_at_connect_as_the_driver_does(bench):
    fake = bench.chain(q=RAISED)
    fake.motors[3].registers['CTRL_MODE'] = 3
    bench.arm(fake)
    assert fake.motors[3].registers['CTRL_MODE'] == 1 and 0xAA in fake.register_commands()


def test_mis_scaled_motor_registers_are_refused(bench):
    fake = bench.chain()
    original = fake._read
    fake._read = lambda motor, name: 30.0 if motor.id == 3 and name == 'VMAX' else original(motor, name)
    with pytest.raises(ArmHardwareError, match='mis-scaled'):
        bench.arm(fake)
    assert not fake.special_frames(ENABLE)


# -- motion -------------------------------------------------------------------------------------------------------
def test_a_far_command_is_followed_smoothly_and_arrives(bench):
    fake = bench.chain(q=RAISED)
    arm = bench.arm(fake)
    goal = list(RAISED)
    goal[1] += 0.3
    goal[3] -= 0.2
    start = time.monotonic()
    deadline = start + 2.2
    while time.monotonic() < deadline:
        arm.command(goal, 0.5)
        time.sleep(1 / 30)
    assert wait(lambda: abs(fake.positions()[1] - goal[1]) < 0.01, 1.0), fake.positions()
    assert fake.positions() == pytest.approx(goal, abs=0.02)
    rows = [r for r in commands_of(fake, 2, start) if r[4] > 0]               # from where the position loop is on
    times, targets = np.array([r[0] for r in rows]), np.array([r[2] for r in rows])
    changes = np.flatnonzero(np.diff(targets)) + 1
    late = times[changes[1:]][np.diff(times[changes]) > 3 / arm.rate_hz]      # this computer held the stream back: the trajectory waited, then went on
    grid = np.arange(times[0], times[-1], 0.05)
    sampled = targets[np.searchsorted(times, grid, side='right') - 1]
    speed = np.diff(sampled) / 0.05
    accel = np.diff(speed) / 0.05
    clean = np.array([not len(late) or abs(late - t).min() > 0.12 for t in grid[1:-1]])
    assert clean.mean() > .5, late
    assert abs(speed).max() <= arm.joint_speed_rad_s * 1.25, abs(speed).max()
    assert abs(accel[clean]).max() <= arm.joint_accel_rad_s2 * 4, abs(accel[clean]).max()   # test_arm_base pins the exact limits
    assert fake.jaw_position() == pytest.approx(JAW_CLOSED - 0.5 * JAW_STROKE, abs=0.25)


def test_stop_holds_near_where_the_arm_was_and_stays(bench):
    fake = bench.chain(q=RAISED)
    arm = bench.arm(fake)
    goal = list(RAISED)
    goal[1] += 0.6
    end = time.monotonic() + 0.5
    while time.monotonic() < end:
        arm.command(goal, 0.0)
        time.sleep(1 / 30)
    arm.hold()
    at_stop = fake.positions()[1]
    time.sleep(1.0)
    settled = time.monotonic()
    time.sleep(0.5)
    assert excursion(fake, settled)[:6].max() < 0.003
    assert abs(fake.positions()[1] - at_stop) < 0.18                          # braking distance plus tracking lag
    assert fake.positions()[1] < goal[1] - 0.1


def test_commands_that_stop_arriving_end_in_a_hold(bench):
    fake = bench.chain(q=RAISED)
    arm = bench.arm(fake)
    goal = list(RAISED)
    goal[2] -= 0.6
    arm.command(goal, 0.0)
    time.sleep(arm.command_timeout_s + 1.0)
    settled = time.monotonic()
    time.sleep(0.3)
    assert excursion(fake, settled)[:6].max() < 0.003 and fake.positions()[2] > goal[2] + 0.1


def test_release_is_gravity_compensation_not_a_drop_and_reengages_without_a_jump(bench):
    fake = bench.chain(q=RAISED)
    arm = bench.arm(fake)
    time.sleep(0.2)
    arm.release()
    time.sleep(0.1)
    released = time.monotonic()
    time.sleep(0.6)
    assert excursion(fake, released)[:6].max() < 0.01
    frames = commands_of(fake, 2, released)
    assert frames and all(r[4] == 0 for r in frames)                          # no stiffness
    assert min(abs(r[6]) for r in frames) > 0.5                               # the gravity torque is still sent
    assert not fake.special_frames(DISABLE)
    here = fake.positions()
    resumed = time.monotonic()
    arm.command(here, arm.read()['gripper'])
    time.sleep(0.4)
    assert excursion(fake, resumed)[:6].max() < 0.004
    assert commands_of(fake, 2, resumed)[-1][4] > 0


# -- faults -------------------------------------------------------------------------------------------------------
def test_a_motor_dropping_out_latches_a_fault(bench):
    fake = bench.chain(q=RAISED)
    arm = bench.arm(fake)
    time.sleep(0.2)
    fake.drop(4)

    assert wait(lambda: arm._error is not None, 2.0)
    with pytest.raises(ArmHardwareError):
        arm.read()
    with pytest.raises(ArmHardwareError):
        arm.command(list(RAISED), 0.0)
    time.sleep(0.2)
    fake.motors[4].silent = False                                             # coming back does not clear it
    time.sleep(0.2)
    with pytest.raises(ArmHardwareError):
        arm.read()


def test_a_motor_reporting_a_fault_latches_a_fault(bench):
    fake = bench.chain(q=RAISED)
    arm = bench.arm(fake)
    time.sleep(0.2)
    fake.fault(5)

    def failed():
        try:
            arm.read()
        except ArmHardwareError as exc:
            return str(exc)
        return False
    assert wait(failed, 2.0)


# -- read-only survey ---------------------------------------------------------------------------------------------
def test_survey_reads_the_chain_and_enables_nothing(bench):
    fake = bench.chain(q=RAISED)
    arm = bench.arm(fake, connect=False)
    report = arm.survey()
    assert report['ok'], report
    assert [m['id'] for m in report['motors']] == list(range(1, 8))
    assert report['motors'][1]['type'] == 'DM4340' and report['motors'][1]['registers']['Gr'] == 40
    assert report['motors'][1]['position_rad'] == pytest.approx(RAISED[1], abs=1e-3)
    assert not list(fake.mit)
    assert all(f[1] == 0x7FF and f[2][2] == 0x33 for f in fake.heard())       # register reads and nothing else


def test_survey_reports_a_wrong_control_mode_without_repairing_it(bench):
    fake = bench.chain()
    fake.motors[3].registers['CTRL_MODE'] = 3
    report = bench.arm(fake, connect=False).survey()
    assert not report['ok'] and 'CTRL_MODE' in report['error']
    assert fake.motors[3].registers['CTRL_MODE'] == 3
    assert set(fake.register_commands()) == {0x33} and not fake.special_frames(ENABLE)


def test_survey_names_a_wrong_motor(bench):
    fake = bench.chain()
    fake.motors[6].reported_kind = 'DM4340'
    report = bench.arm(fake, connect=False).survey()
    assert not report['ok'] and 'motor 6' in report['error'] and not list(fake.mit)


# -- close --------------------------------------------------------------------------------------------------------
def test_close_leaves_the_arm_in_gravity_compensation_without_a_drop(bench):
    fake = bench.chain(q=RAISED)
    arm = bench.arm(fake)
    time.sleep(0.2)
    start = time.monotonic()
    arm.close()
    last = commands_of(fake, 2, start)[-1]
    assert last[4] == 0 and abs(last[6]) > 0.5                                # no stiffness, gravity torque kept
    time.sleep(0.2)
    assert excursion(fake, start)[:6].max() < 0.01
    assert not fake.special_frames(DISABLE)


def test_the_arm_can_be_opened_again_after_a_close(bench):
    fake = bench.chain(q=RAISED)
    for motor in fake.motors.values():
        motor.registers['TIMEOUT'] = 0                                        # the last frame stays in force
    bench.arm(fake).close()
    time.sleep(0.05)
    start = time.monotonic()
    second = bench.arm(fake)
    time.sleep(0.3)
    assert excursion(fake, start)[:6].max() < 0.01
    assert second.read()['positions_rad'] == pytest.approx(fake.positions(), abs=0.004)


# -- the jaw ------------------------------------------------------------------------------------------------------
def test_an_uncalibrated_gripper_is_refused_before_anything_moves(bench):
    fake = bench.chain()
    arm = YamArm(bus=fake.address)
    with pytest.raises(ArmHardwareError, match='hw calibrate --gripper'):
        arm.connect()
    assert not fake.heard()


def test_gripper_calibration_finds_the_stops_and_keeps_the_arm_still(bench):
    fake = bench.chain(q=RAISED, jaw=0.5)
    for motor in fake.motors.values():
        motor.registers['TIMEOUT'] = 0                                        # the arm stays up between the two opens
    start = time.monotonic()
    result = YamArm(bus=fake.address).calibrate_gripper()
    assert result['closed'] == pytest.approx(JAW_CLOSED, abs=0.05)
    assert result['open'] == pytest.approx(JAW_CLOSED - JAW_STROKE, abs=0.05)
    assert excursion(fake, start)[:6].max() < 0.01                            # only the jaw moved
    assert fake.jaw_position() == pytest.approx(JAW_CLOSED - JAW_STROKE, abs=0.1)   # it ends open
    arm = bench.keep(YamArm(bus=fake.address))
    arm.connect()
    assert arm.read()['gripper'] == pytest.approx(1.0, abs=0.02)


def test_the_generic_calibration_command_points_at_the_gripper_calibration():
    from roborun_sdk.arms.calibrate import record
    with pytest.raises(SystemExit, match='hw calibrate --gripper'):
        record(YamArm(bus='virtual://unused'))


def test_a_blocked_jaw_does_not_become_a_calibration(bench):
    fake = bench.chain()
    fake.jaw.friction = 5.0                                                   # it cannot move
    arm = YamArm(bus=fake.address)
    with pytest.raises(ArmHardwareError, match='only travelled'):
        arm.calibrate_gripper()
    assert not arm._jaw_file.exists()


@pytest.mark.parametrize('true_position, expected', [(JAW_CLOSED - 0.1, 0.1 / JAW_STROKE),
                                                     (JAW_CLOSED - JAW_STROKE + 0.1, 1 - 0.1 / JAW_STROKE),
                                                     (0.0, JAW_CLOSED / JAW_STROKE)])
def test_after_a_power_cycle_the_jaw_is_found_without_moving_it(bench, true_position, expected):
    """The stroke is longer than one motor turn, so near either end the raw reading alone is ambiguous."""
    fake = bench.chain(q=RAISED, jaw=true_position)
    arm = YamArm(bus=fake.address)
    arm._save_jaw(closed=JAW_CLOSED, open=JAW_CLOSED - JAW_STROKE, last=true_position + 0.02)
    start = time.monotonic()
    bench.keep(arm).connect()
    assert arm._jaw_target == pytest.approx(expected, abs=0.01)               # the very first sample was already right
    time.sleep(0.3)
    assert arm.read()['gripper'] == pytest.approx(expected, abs=0.01)
    assert excursion(fake, start)[6] < 0.02


def test_the_jaw_opens_and_closes_on_command_across_the_motor_turn(bench):
    fake = bench.chain(q=RAISED, jaw=0.0)
    arm = bench.arm(fake)
    arm.joint_speed_rad_s = 0.6
    for goal in (0.05, 0.95):
        end = time.monotonic() + 2.2
        while time.monotonic() < end:
            arm.command(list(RAISED), goal)
            time.sleep(1 / 30)
        assert arm.read()['gripper'] == pytest.approx(goal, abs=0.05)
        assert fake.jaw_position() == pytest.approx(JAW_CLOSED - goal * JAW_STROKE, abs=0.35)


def test_a_squeezing_jaw_reports_effort(bench):
    fake = bench.chain(q=RAISED, jaw=0.0)
    arm = bench.arm(fake)
    time.sleep(0.1)
    free = arm.read()['gripper_effort']
    fake.jaw.friction = 50.0                                                  # something is in the way
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        arm.command(list(RAISED), 0.0)
        time.sleep(1 / 30)
    assert arm.read()['gripper_effort'] > max(0.3, free)


# -- catalog ------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize('name', ['yam', 'yam_pro', 'yam_ultra', 'big_yam'])
def test_every_catalog_arm_opens_through_the_catalog_without_moving(bench, name):
    row = catalog()[name]
    assert row['status'] == 'beta' and row['transport'] == 'can' and row['install'] == 'yam'
    assert all(lo < hi for lo, hi in row['limits']) and len(row['limits']) == 6
    fake = bench.chain(q=(0.2, 0.9, 0.8, 0.1, 0.1, 0.1), arm=name)
    start = time.monotonic()
    arm = bench.keep(open_arm(name, 'left', fake.address, gripper_limits=fake.jaw_limits()))
    time.sleep(0.3)
    assert excursion(fake, start)[:6].max() < 0.006
    assert arm.read()['positions_rad'] == pytest.approx([0.2, 0.9, 0.8, 0.1, 0.1, 0.1], abs=0.006)


def test_the_yam_entry_uses_the_rig_home_of_each_side():
    rig = json.loads((Path(__file__).resolve().parents[1] / 'src/roborun_sdk/data/profiles/yam.json').read_text())
    assert open_arm('yam', 'left', 'virtual://unused', connect=False).home == rig['arms']['left']['home']
    assert open_arm('yam', 'right', 'virtual://unused', connect=False).home == rig['arms']['right']['home']


def test_joint_signs_and_zero_match_the_rig_kinematics():
    """The runtime's YAM chain (the signed pack) and the vendored model agree on every joint's sign and zero."""
    import mujoco
    from i2rt.robots.utils import ArmType, GripperType, combine_arm_and_gripper_xml
    pack = json.loads((Path(__file__).resolve().parents[1] / 'src/roborun_sdk/data/sim-yam-metal.pack.json').read_text())
    chain = pack['arms']['left']['chain']

    def rot(axis, angle):
        axis = np.asarray(axis, dtype=float)
        k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k

    def rig_rotation(q):
        out = np.eye(3)
        for joint, angle in zip(chain, q):
            r, p, y = joint['rpy']
            out = out @ rot([0, 0, 1], y) @ rot([0, 1, 0], p) @ rot([1, 0, 0], r) @ rot(joint['axis'], angle)
        return out

    model = mujoco.MjModel.from_xml_path(combine_arm_and_gripper_xml(ArmType.YAM, GripperType.LINEAR_4310))
    data = mujoco.MjData(model)
    tool = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper')

    def model_rotation(q):
        data.qpos[:] = 0
        data.qpos[:6] = q
        mujoco.mj_kinematics(model, data)
        return data.xmat[tool].reshape(3, 3).copy()

    fixed = model_rotation(np.zeros(6)).T @ rig_rotation(np.zeros(6))
    rng = np.random.default_rng(1)
    for _ in range(30):
        q = np.array([rng.uniform(j['lower'], j['upper']) for j in chain])
        delta = (model_rotation(q) @ fixed).T @ rig_rotation(q)
        assert np.degrees(np.arccos(np.clip((np.trace(delta) - 1) / 2, -1, 1))) < 0.05


def test_arm_check_prints_clean_json_and_calibration_runs_from_the_command_line(bench, capsys):
    from roborun_sdk.arms import hardware
    from roborun_sdk.arms.yam import main
    fake = bench.chain(q=RAISED, jaw=0.5)
    assert main(['calibrate-gripper', f'yam@{fake.address}']) == 0
    assert json.loads(capsys.readouterr().out)['stroke_rad'] == pytest.approx(JAW_STROKE, abs=0.1)
    since = time.monotonic()
    hardware.cmd_check(type('Args', (), dict(arm=f'yam@{fake.address}', side='left', options=None))())
    report = json.loads(capsys.readouterr().out)
    assert report['ok'] and report['gripper_calibrated'] and not fake.special_frames(ENABLE, since)


def test_no_gripper_arm_has_no_jaw_to_calibrate(bench):
    fake = bench.chain(q=RAISED, gripper='no_gripper')
    arm = bench.keep(YamArm(bus=fake.address, gripper_type='no_gripper'))
    arm.connect()
    arm.command(list(RAISED), 0.0)
    time.sleep(0.2)
    assert arm.read()['gripper'] == 1.0 and 'gripper_effort' not in arm.read()
    assert len(fake.special_frames(ENABLE)) == 6


# -- the USB adapters ---------------------------------------------------------------------------------------------
@pytest.fixture
def slcan_adapter(bench, monkeypatch):
    from serial.tools import list_ports
    fake = bench.chain(q=RAISED)
    adapter = bench.keep(SimSlcanAdapter(fake.name))
    monkeypatch.setattr(list_ports, 'comports', lambda: [adapter.port_row()])
    return fake, adapter, f'slcan://{adapter.serial}'


@pytest.fixture
def gs_usb_adapter(bench, monkeypatch):
    import usb.core
    fake = bench.chain(q=RAISED)
    device = bench.keep(SimGsUsbDevice(fake.name))
    monkeypatch.setattr(usb.core, 'find', lambda find_all=False, **kw: iter([device]))
    return fake, device, f'gs_usb://{device.serial_number}'


def test_every_motor_gets_its_hold_right_behind_its_enable(bench):
    fake = bench.chain(q=RAISED)
    start = time.monotonic()
    bench.arm(fake)
    frames = fake.heard(start)
    for motor in fake.motors:
        at = next(i for i, f in enumerate(frames) if f[1] == motor and f[2] == bytes([0xFF] * 7 + [ENABLE]))
        assert frames[at + 1][1] == motor and frames[at + 1][2] != bytes([0xFF] * 7 + [ENABLE]), motor


@pytest.mark.parametrize('adapter', ['slcan_adapter', 'gs_usb_adapter'])
def test_the_real_startup_runs_over_usb_adapters(bench, request, adapter):
    fake, _device, address = request.getfixturevalue(adapter)
    report = bench.arm(fake, connect=False, bus=address).survey()
    assert report['ok'], report
    assert not fake.special_frames(ENABLE)
    start = time.monotonic()
    arm = bench.arm(fake, bus=address)
    time.sleep(0.4)
    assert excursion(fake, start)[:6].max() < 0.006
    assert arm.read()['positions_rad'] == pytest.approx(list(RAISED), abs=0.006)
    assert {f[2][3] for f in fake.heard() if f[1] == 0x7FF} >= {20, 10, 21, 22, 23}
    goal = list(RAISED)
    goal[0] += 0.05
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        arm.command(goal, arm.read()['gripper'])
        time.sleep(1 / 30)
    assert fake.positions()[0] == pytest.approx(goal[0], abs=0.015)
    arm.close()
    assert not fake.special_frames(DISABLE)


def test_slcan_adapter_is_put_in_normal_mode_at_1_mbit(slcan_adapter):
    _fake, adapter, address = slcan_adapter
    bus = canbus.open_bus(address)
    bus.shutdown()
    assert 'S8' in adapter.commands and 'M0' in adapter.commands and adapter.commands.index('M0') < len(adapter.commands)


def test_gs_usb_is_pinned_by_serial_timed_for_1_mbit_and_never_usb_reset(gs_usb_adapter):
    _fake, device, address = gs_usb_adapter
    assert [r['address'] for r in canbus.list_adapters()] == [address]
    bus = canbus.open_bus(address)
    assert device.started and device.claimed and device.rate() == 1_000_000
    with pytest.raises(RuntimeError, match='in use'):
        canbus.open_bus(address)
    bus.shutdown()
    assert not device.claimed and device.resets == 0
    with pytest.raises(RuntimeError, match='not connected'):
        canbus.open_bus('gs_usb://SOMEOTHERSERIAL')
    canbus.open_bus(address).shutdown()                                       # the claim was released


def test_gs_usb_echoes_are_not_delivered_as_received_frames(gs_usb_adapter):
    import can
    _fake, _device, address = gs_usb_adapter
    bus = canbus.open_bus(address)
    try:
        bus.send(can.Message(arbitration_id=0x123, data=b'\x01\x02', is_extended_id=False))
        assert bus.recv(timeout=0.1) is None
    finally:
        bus.shutdown()


@pytest.mark.parametrize('fclk, brp', [(48_000_000, 3), (80_000_000, 5), (170_000_000, 10)])
def test_gs_usb_bit_timing_for_common_adapter_clocks(fclk, brp):
    prop, seg1, seg2, sjw, got_brp = canbus.gs_usb_timing((0, fclk, 1, 16, 1, 8, 4, 1, 1024, 1))
    assert got_brp == brp and fclk / (brp * (1 + prop + seg1 + seg2)) == 1_000_000
    assert (1 + prop + seg1) / (1 + prop + seg1 + seg2) == pytest.approx(0.875, abs=0.05) and sjw >= 1


# -- a whole station ----------------------------------------------------------------------------------------------
def post(srv, route, body=None):
    request = urllib.request.Request(srv.url + route, data=json.dumps(body or {}).encode(),
                                     headers={'Content-Type': 'application/json', 'X-Manifest-Supervisor': '1'})
    with urllib.request.urlopen(request, timeout=3) as reply:
        return json.load(reply)


def get(srv, route):
    with urllib.request.urlopen(srv.url + route, timeout=3) as reply:
        return json.load(reply)


@pytest.fixture
def station(bench):
    fake = bench.chain(q=RAISED)
    arm = bench.arm(fake)
    st, srv = run_station(arm, port=0)
    bench.keep(srv)
    goal = list(RAISED)
    goal[1] += 0.25

    def predict(body):
        retained = body['commanded']['q_l_chunk']
        target = [a + max(-0.02, min(0.02, g - a)) for a, g in zip(retained[:6], goal)] + [retained[6]]
        return dict(target=target, acquired_at=st.clock())
    st.predict = predict
    return st, srv, arm, fake, goal


def start_lease(st):
    session = st.register_session(seconds=6)
    observation = st.observe()['observation_id']
    return st.action(dict(observation_id=observation, seconds=6, instruction='x',
                          reference=dict(kind='video_motion', session_id=session)))['lease']['id']


def test_station_connects_without_motion_and_serves_the_measured_pose(station):
    st, srv, _arm, fake, _goal = station
    start = time.monotonic()
    time.sleep(0.5)
    # a row is fresh when its sample is under 50 ms old: on a loaded machine (pytest -n auto) one read can miss that
    assert wait(lambda: get(srv, '/supervisor/pose')['measured']['left']['fresh'], 2.0)
    row = get(srv, '/supervisor/pose')['measured']['left']
    assert row['positions_rad'] == pytest.approx(fake.positions(), abs=0.005)
    assert excursion(fake, start)[:6].max() < 0.003


def test_station_lease_moves_the_arm_and_stop_holds_it(station):
    st, srv, _arm, fake, goal = station
    start = time.monotonic()
    lease = start_lease(st)
    assert wait(lambda: fake.positions()[1] > RAISED[1] + 0.06, 3.0), fake.positions()
    st.heartbeat(dict(lease_id=lease))
    answer = post(srv, '/supervisor/hold')
    assert answer['holding'] or answer['arms']['left']['state'] in ('moving', 'holding')
    assert wait(lambda: not st.lease['active'], 1.0)
    time.sleep(1.2)
    settled = time.monotonic()
    time.sleep(0.4)
    assert excursion(fake, settled)[:6].max() < 0.003
    assert fake.positions()[1] < goal[1] - 0.02
    assert post(srv, '/supervisor/hold')['holding']
    rows = [r for r in commands_of(fake, 2, start) if r[4] > 0]               # from where the position loop is on
    assert np.abs(np.diff([r[2] for r in rows])).max() < 0.02                 # no jump in what the motor was told


def test_station_refuses_to_claim_a_hold_after_a_motor_drops_out(station):
    st, srv, arm, fake, _goal = station
    time.sleep(0.2)
    fake.drop(3)
    assert wait(lambda: arm._error is not None, 3.0)
    time.sleep(0.4)                                                           # the newest sample is now too old
    assert get(srv, '/supervisor/pose')['arms']['left']['state'] == 'stale'
    answer = post(srv, '/supervisor/hold')
    assert not answer['holding'] and answer['arms']['left']['state'] == 'fault'
    assert '3' in answer['arms']['left']['cause'] and get(srv, '/status')['dead_arms'] == ['left']


# -- safety on the real YAM stack ---------------------------------------------------------------------------------
def power_on(fake):
    """A YAM an earlier session left enabled, holding where it is."""
    for motor in fake.motors.values():
        motor.registers['TIMEOUT'] = 0                  # its last command stays in force until someone talks to it
        motor.state, motor.was_enabled, motor.heard_at = 1, True, time.monotonic()
        motor.cmd = (motor.p, 0.0, 400.0, 5.0, 0.0)     # stiff enough that its own sag is small


def test_an_arm_left_powered_stays_up_and_never_gets_a_zero_gain_frame(bench):
    fake = bench.chain(q=RAISED)
    power_on(fake)
    time.sleep(0.4)
    start = time.monotonic()
    arm = bench.arm(fake)
    time.sleep(0.5)
    assert excursion(fake, start)[:6].max() < 0.02
    assert len(arm.health()['adopted']) == 7
    for motor in range(1, 7):
        assert not [row for row in commands_of(fake, motor, start) if row[4] == 0 and row[5] == 0 and row[6] == 0]


def test_an_arm_left_powered_can_be_refused(bench):
    fake = bench.chain(q=RAISED)
    power_on(fake)
    with pytest.raises(ArmHardwareError, match='still powered'):
        bench.arm(fake, powered_at_connect='refuse')


def test_payload_and_gravity_factor_reach_the_gravity_torque(bench):
    def gravity(**options):
        fake = bench.chain(q=RAISED)
        arm = bench.arm(fake, **options)
        robot = arm.robot
        arm.close()                 # the model's data is shared with the control thread: read it once that has stopped
        torque = robot.kdl.compute_inverse_dynamics(np.array(RAISED), np.zeros(6), np.zeros(6)).copy()
        factor = np.array(robot.gravity_comp_factor, dtype=float).copy()
        return torque, factor
    plain, factor = gravity()
    loaded, _ = gravity(payload_kg=0.5)
    assert abs(loaded[1]) > abs(plain[1]) + 0.5                                # the shoulder carries the payload too
    _, changed = gravity(gravity_comp_factor=[1.0, 0.55, 1.1, 1.2, 1.0, 1.0])
    assert changed[1] == pytest.approx(0.55) and factor[1] != pytest.approx(0.55)
    with pytest.raises(ValueError, match='gravity_comp_factor'):
        YamArm(bus='x', gravity_comp_factor=[1.0])


def test_a_dry_run_reads_the_yam_and_never_enables_or_commands_it(bench):
    fake = bench.chain(q=RAISED, jaw=-1.0)
    arm = bench.arm(fake, dry_run=True)
    sample = arm.read()
    assert sample['positions_rad'] == pytest.approx(RAISED, abs=0.003) and sample['gripper_available']
    assert sample['gripper'] == pytest.approx((JAW_CLOSED + 1.0) / JAW_STROKE, abs=0.01)
    goal = list(RAISED); goal[1] += 0.1
    assert wait(lambda: (arm.command(goal, 0.2), len(arm.dry_log) > 10 and arm.dry_log[-1]['positions_rad'][1] > RAISED[1])[1], 3)
    arm.close()
    assert not fake.special_frames(ENABLE) and not list(fake.mit) and 0x55 not in fake.register_commands()


def test_a_hot_motor_stops_the_yam_which_keeps_holding(bench):
    fake = bench.chain(q=RAISED)
    arm = bench.arm(fake)
    time.sleep(0.2)
    fake.motors[2].temp = 90
    assert wait(lambda: arm.health()['kind'] == 'thermal', 3)
    with pytest.raises(ArmHardwareError, match='joint2'):
        arm.command(list(RAISED), 0.5)
    start = time.monotonic()
    time.sleep(0.3)
    assert excursion(fake, start)[:6].max() < 0.01


def test_a_blocked_jaw_keeps_a_chosen_squeeze_not_full_force(bench):
    fake = bench.chain(q=RAISED, jaw=0.0)
    arm = bench.arm(fake, gripper_hold_effort=0.4)
    fake.jaw.friction = 50.0
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        arm.command(list(RAISED), 0.0)
        time.sleep(1 / 30)
    assert arm.health()['jaw'] == 'gripping'
    assert 0.1 < arm.read()['gripper_effort'] < 0.7


def test_grip_force_settings_reach_the_limiter(bench):
    fake = bench.chain(q=RAISED)
    arm = bench.arm(fake, gripper_force_n=20.0, gripper_effort=0.5)
    assert arm.robot._limit_gripper_force == pytest.approx(10.0)


def test_a_yam_at_rest_is_switched_off_only_when_its_pose_carries_it(bench):
    fake = bench.chain(q=RAISED)
    for motor in fake.arm_motors:
        motor.friction = 50.0                           # the rest pose carries the arm
    arm = bench.arm(fake, rest_pose=list(RAISED))
    time.sleep(0.2)
    arm.close()
    assert arm.close_report == dict(parked=True, motors_off=True, why=None)
    assert len(fake.special_frames(DISABLE)) == 7
    fake = bench.chain(q=RAISED)
    arm = bench.arm(fake, rest_pose=list(RAISED))
    time.sleep(0.2)
    arm.close()
    assert arm.close_report['parked'] and not arm.close_report['motors_off'] and 'carry' in arm.close_report['why']
    assert not fake.special_frames(DISABLE)


def test_the_driver_keeps_joint_3_inside_the_maker_limit(bench):
    fake = bench.chain(q=RAISED)
    arm = bench.arm(fake)
    assert arm.limits[2][1] == pytest.approx(3.14159)
    arm.command([1.0, 1.0, 3.6, 0.0, 0.0, 0.0], 0.5)
    assert arm._target[2] == pytest.approx(3.14159)


def test_the_tool_model_comes_with_the_arm(bench):
    fake = bench.chain(q=RAISED)
    arm = bench.arm(fake)
    assert arm.health()['tool_speed']['active']
    assert arm._rule_state()['tool_position_m'] is not None


# -- the teaching handle: an end with no gripper, read-only ------------------------------------------------------------
HANDLES = ['yam_teaching_handle_v1', 'yam_teaching_handle_v2']


@pytest.mark.parametrize('handle', HANDLES)
def test_a_teaching_handle_runs_as_an_arm_without_a_gripper_and_reads_the_handle(bench, handle):
    fake = bench.chain(q=RAISED, gripper=handle)
    fake.handle.update(counts=round(0.35 / (2 * np.pi) * 4096), buttons=2)
    start, before = time.monotonic(), fake.positions()
    arm = bench.arm(fake, gripper_type=handle, gripper_limits=None)
    time.sleep(0.6)
    assert excursion(fake, start)[:6].max() < 0.004
    sample = arm.read()
    assert sample['positions_rad'] == pytest.approx(before, abs=0.004) and sample['gripper_available'] is False
    assert sample['handle']['available'] and sample['handle']['trigger'] == pytest.approx(0.5, abs=0.01)
    assert sample['handle']['buttons'] == [False, True]
    assert len(fake.special_frames(ENABLE)) == 6                             # six joints, no gripper motor
    assert all(r == b'\xff\x02' for r in fake.handle['requests'])            # the report request and nothing else
    fake.handle['counts'] = 0
    assert wait(lambda: arm.read()['handle']['trigger'] == 0.0, 2)


def test_a_silent_handle_never_stops_the_arm(bench):
    fake = bench.chain(q=RAISED, gripper=HANDLES[0])
    fake.handle['silent'] = True
    arm = bench.arm(fake, gripper_type=HANDLES[0], gripper_limits=None)
    time.sleep(0.3)
    asked = len(fake.handle['requests'])
    time.sleep(0.3)
    assert arm.read()['handle'] == dict(available=False)
    assert len(fake.handle['requests']) == asked                             # not asked again once known silent
    fake.handle['silent'] = False


def test_a_handle_that_goes_quiet_while_running_is_reported_not_fatal(bench):
    fake = bench.chain(q=RAISED, gripper=HANDLES[1])
    arm = bench.arm(fake, gripper_type=HANDLES[1], gripper_limits=None)
    assert wait(lambda: arm.read()['handle']['available'], 2)
    fake.handle['silent'] = True
    assert wait(lambda: arm.read()['handle'] == dict(available=False), 2)
    assert arm.read()['positions_rad'] == pytest.approx(list(RAISED), abs=0.01)


def test_the_survey_and_a_dry_run_read_the_handle_and_enable_nothing(bench):
    fake = bench.chain(q=RAISED, gripper=HANDLES[0])
    report = bench.arm(fake, connect=False, gripper_type=HANDLES[0], gripper_limits=None).survey()
    assert report['ok'] and report['handle']['available'] and len(report['motors']) == 6
    arm = bench.arm(fake, gripper_type=HANDLES[0], gripper_limits=None, dry_run=True)
    assert wait(lambda: arm.read().get('handle', {}).get('available'), 2)
    assert not fake.special_frames(ENABLE) and not list(fake.mit)


def test_a_handle_needs_no_gripper_calibration(bench):
    fake = bench.chain(gripper=HANDLES[0])
    arm = bench.arm(fake, connect=False, gripper_type=HANDLES[0], gripper_limits=None)
    assert not arm._jaw_needs_stops() and [row[2] for row in arm.thermal_motors()] == ['joint'] * 6
    with pytest.raises(ArmHardwareError, match='needs no calibration'):
        arm.calibrate_gripper()
