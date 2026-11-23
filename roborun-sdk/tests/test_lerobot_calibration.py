"""Importing a LeRobot calibration file (only its format) for Feetech and Dynamixel arms, on simulated servo buses."""
import json
import math

import pytest

pytest.importorskip('serial')

from fake_dynamixel import FakeDynamixelBus  # noqa: E402
from fake_feetech import FakeFeetechBus  # noqa: E402
from roborun_sdk.arms import calibrate as stored, open_arm  # noqa: E402
from roborun_sdk.arms.base import ArmHardwareError  # noqa: E402

NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']


@pytest.fixture
def home(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBORUN_HOME', str(tmp_path / 'roborun'))
    monkeypatch.setenv('HF_LEROBOT_CALIBRATION', str(tmp_path / 'lerobot'))
    return tmp_path


def lerobot_file(home, homing=None, ranges=None, kind='robots/so101_follower', name='arm1', **extra):
    rows = {n: dict(id=i + 1, drive_mode=0, homing_offset=(homing or {}).get(n, -500 + 100 * i),
                    range_min=(ranges or {}).get(n, (900, 3100))[0], range_max=(ranges or {}).get(n, (900, 3100))[1])
            for i, n in enumerate(NAMES)}
    for n, row in extra.items():
        rows[n].update(row)
    path = home / 'lerobot' / kind / f'{name}.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(rows))
    return path


@pytest.fixture
def so101(home):
    """Servos already homed by LeRobot: homing offsets set, every joint near the middle (reads ~2047)."""
    fake = FakeFeetechBus({i + 1: 2047 + (-500 + 100 * i) for i in range(6)}, load=False).start()
    for i in range(6):
        fake.servos[i + 1].mem[31:33] = (abs(-500 + 100 * i) | (1 << 11 if -500 + 100 * i < 0 else 0)).to_bytes(2, 'little')
    fake.servos[6].actual = 1000 + fake.servos[6].homing()                  # the gripper sits closed, at range_min
    yield fake
    fake.close()


def snapshot(fake):
    return {i: bytes(s.mem[:56]) for i, s in fake.servos.items()}


def test_the_file_is_found_and_imported_without_writing_a_servo(home, so101):
    path = lerobot_file(home)
    arm = open_arm('so101', 'left', so101.address, connect=False)
    assert [p for p, _ in stored.lerobot_files(arm)] == [path]
    before, writes = snapshot(so101), len([p for p in so101.packets if p[4] in (3, 0x83)])
    stored.import_lerobot(arm, path, ask=lambda text: '', say=lambda _: None)
    assert snapshot(so101) == before and len([p for p in so101.packets if p[4] in (3, 0x83)]) == writes
    data = stored.load('so101', so101.address)
    to_rad = 2 * math.pi / 4096
    assert data['facts']['homing']['shoulder_pan'] == -500 and data['facts']['source'] == 'lerobot'
    assert data['joints']['shoulder_pan']['zero'] == pytest.approx(2047 * to_rad)
    assert data['joints']['elbow_flex']['lo'] == pytest.approx((900 - 2047) * to_rad - 0.02)
    assert data['gripper'] == dict(closed=pytest.approx(900 * to_rad), open=pytest.approx(3100 * to_rad))


def test_the_imported_arm_connects_and_reads_its_zero_pose_as_zero(home, so101):
    stored.import_lerobot(open_arm('so101', 'left', so101.address, connect=False), lerobot_file(home), say=lambda _: None)
    arm = open_arm('so101', 'left', so101.address, connect=True)
    try:
        sample = arm.read()
        assert max(abs(q) for q in sample['positions_rad']) < 0.01        # every joint sits where LeRobot homed it
    finally:
        arm.close()


def test_a_servo_with_another_homing_offset_refuses_the_file(home, so101):
    path = lerobot_file(home, homing={'elbow_flex': 77})
    with pytest.raises(SystemExit, match='elbow_flex: the servo holds homing offset -300, the file says 77'):
        stored.import_lerobot(open_arm('so101', 'left', so101.address, connect=False), path, say=lambda _: None)
    assert stored.load('so101', so101.address) is None


def test_a_joint_outside_the_recorded_range_refuses_the_file(home, so101):
    path = lerobot_file(home, ranges={'wrist_flex': (100, 600)})
    with pytest.raises(SystemExit, match='wrist_flex reads'):
        stored.import_lerobot(open_arm('so101', 'left', so101.address, connect=False), path, say=lambda _: None)


def test_files_for_other_arms_are_not_offered(home, so101):
    lerobot_file(home, name='koch', kind='robots/koch_follower', gripper=dict(id=9))
    arm = open_arm('so101', 'left', so101.address, connect=False)
    assert stored.lerobot_files(arm) == []
    path = lerobot_file(home, name='flipped', wrist_roll=dict(drive_mode=1))
    with pytest.raises(SystemExit, match='drive_mode 1'):
        stored.import_lerobot(arm, path, say=lambda _: None)


def test_a_servo_rehomed_after_the_import_is_caught_at_connect(home, so101):
    stored.import_lerobot(open_arm('so101', 'left', so101.address, connect=False), lerobot_file(home), say=lambda _: None)
    so101.servos[2].mem[31:33] = (5).to_bytes(2, 'little')
    with pytest.raises(ArmHardwareError, match='homing offset is not the calibrated one'):
        open_arm('so101', 'left', so101.address, connect=True)
    assert not so101.torque_writes_on()


def test_a_gripper_left_half_open_is_not_guessed(home, so101):
    so101.servos[6].actual = 2000 + so101.servos[6].homing()
    with pytest.raises(SystemExit, match='not near either end'):
        stored.import_lerobot(open_arm('so101', 'left', so101.address, connect=False), lerobot_file(home),
                              ask=lambda text: '', say=lambda _: None)


def test_dynamixel_koch_imports_and_checks_its_offsets(home):
    from roborun_sdk.arms import catalog
    rows = (catalog()['koch_v1_1']['options'])
    ids = [r['id'] for r in rows['joints']] + [rows['gripper']['id']]
    names = [r['name'] for r in rows['joints']] + [rows['gripper']['name']]
    model = {'xl330-m077': 1190, 'xl330-m288': 1200, 'xl430-w250': 1060, 'xm430-w350': 1020, 'xm540-w270': 1120}
    models = [model[r['model']] for r in rows['joints']] + [model[rows['gripper']['model']]]
    fake = FakeDynamixelBus({i: (2047 - 300, m, None) for i, m in zip(ids, models)}, load=False).start()
    try:
        for i in ids:
            fake.servos[i].mem[20:24] = (300).to_bytes(4, 'little', signed=True)   # LeRobot's offset: present = actual + 300
        path = home / 'lerobot' / 'robots' / 'koch_follower' / 'k.json'
        path.parent.mkdir(parents=True)
        path.write_text(json.dumps({n: dict(id=i, drive_mode=0, homing_offset=300, range_min=1000, range_max=3000)
                                    for n, i in zip(names, ids)}))
        arm = open_arm('koch_v1_1', 'left', fake.address, connect=False)
        assert [p for p, _ in stored.lerobot_files(arm)] == [path]
        stored.import_lerobot(arm, path, say=lambda _: None)
        fake.servos[ids[0]].mem[20:24] = (0).to_bytes(4, 'little', signed=True)
        with pytest.raises(ArmHardwareError, match='homing offset'):
            open_arm('koch_v1_1', 'left', fake.address, connect=True)
        assert not fake.torque_writes_on()
    finally:
        fake.close()


def test_the_command_line_import(home, so101, monkeypatch, capsys):
    import argparse
    lerobot_file(home)
    monkeypatch.setattr('sys.stdin.isatty', lambda: False)
    assert stored.cmd_calibrate(argparse.Namespace(arm=f'so101@{so101.address}', side='left', from_lerobot='auto')) == 0
    assert 'range_min is taken as closed' in capsys.readouterr().out
    assert stored.load('so101', so101.address)['facts']['source'] == 'lerobot'
