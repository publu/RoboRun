"""SO-100, SO-101 and XLeRobot against simulated STS3215 servos on a fake serial port."""
import math

import pytest

pytest.importorskip('serial')

from arm_contract import *  # noqa: F401,F403,E402  the shared checks are collected from here
from arm_contract import Rig, hand_operator, settle, wait  # noqa: E402
from roborun_sdk.arms import calibrate as stored  # noqa: E402
from fake_feetech import COUNTS, FakeFeetechBus  # noqa: E402
from roborun_sdk.arms.base import ArmHardwareError  # noqa: E402
from roborun_sdk.arms.feetech import MID  # noqa: E402

TO_COUNTS = COUNTS / (2 * math.pi)


class View:
    """The contract's view of a servo bus, by motor name."""
    def __init__(self, fake, ids): self.fake, self.ids = fake, ids
    def servo(self, name): return self.fake.servos[self.ids[name]]
    def pos(self, name): return self.servo(name).present() / TO_COUNTS
    def move_by_hand(self, name, pos): s = self.servo(name); s.actual = pos * TO_COUNTS + s.homing()
    def enabled(self): return [n for n in self.ids if self.servo(n).mem[40]]
    def enable_frames(self): return self.fake.torque_writes_on()
    def fault(self, name): self.servo(name).error = 32
    def silence(self, name): self.servo(name).mute = True
    def remove(self, name): self.silence(name)
    def wrong_motor(self, name): self.servo(name).mem[3:5] = (1020).to_bytes(2, 'little')
    def target(self, name): return self.servo(name).u16(42) / TO_COUNTS
    def set_temp(self, name, value): self.servo(name).temp = int(value)
    def block(self, name): self.servo(name).blocked = True
    def restore(self, name): self.servo(name).mute = False
    def make_heavy(self): self.fake.sag = 400.0; self.fake.hold_needs = 400
    def set_load(self, value): self.fake.load = value
    def power_on(self, name): s = self.servo(name); s.sync_goal(); s.mem[40] = 1; s.armed = True
    def close(self): self.fake.close()


@pytest.fixture(params=['so101', 'so100', 'xlerobot'])
def rig(request, tmp_path, monkeypatch):
    def make_bus(r):
        fake = FakeFeetechBus({m.id: r.start[m.name] * TO_COUNTS for m in r.table.motors}).start()
        r.address = fake.address
        return View(fake, {m.name: m.id for m in r.table.motors})
    r = Rig(request.param, tmp_path, monkeypatch, make_bus, jaw=(1.0, 2.2), zero=lambda i: MID / TO_COUNTS)
    yield r
    r.close()


def test_the_servos_get_no_goal_away_from_where_they_are(rig):
    arm = rig.arm(connect=True)
    settle(arm)
    for motor in rig.table.motors:
        assert rig.bus.servo(motor.name).u16(42) == pytest.approx(round(rig.start[motor.name] * TO_COUNTS), abs=8)


def test_release_drops_the_torque_limit_but_not_the_torque(rig):
    arm = rig.arm(connect=True)
    settle(arm)
    arm.release()
    assert wait(lambda: rig.bus.servo('shoulder_lift').u16(48) == 350, 1)
    assert rig.bus.servo('shoulder_lift').mem[40] == 1


def test_the_gripper_torque_is_capped(rig):
    arm = rig.arm(connect=True)
    settle(arm)
    assert rig.bus.servo('gripper').u16(48) == 500


def test_calibration_puts_the_zero_pose_mid_encoder(rig):
    stored.path(rig.arm_id, rig.address).unlink()
    operator, _ = hand_operator(rig)
    stored.record(rig.arm(), ask=operator, say=lambda _: None)
    data = stored.load(rig.arm_id, rig.address)
    assert all(abs(row['zero'] * TO_COUNTS - MID) < 3 for row in data['joints'].values())
    assert set(data['facts']['homing']) == {m.name for m in rig.table.motors}
    rig.bus.servo(rig.joint_motors()[0].name).mem[31:33] = (5).to_bytes(2, 'little')      # someone re-homed it since
    with pytest.raises(ArmHardwareError, match='homing offset'):
        rig.arm(connect=True)


def test_a_port_is_pinned_by_its_usb_serial_number(monkeypatch):
    from types import SimpleNamespace as Port
    from roborun_sdk.arms import serial_servo
    ports = [Port(device='/dev/ttyACM3', serial_number='AB12', product='x'), Port(device='/dev/ttyACM4', serial_number='CD34', product='y')]
    monkeypatch.setattr(serial_servo.list_ports, 'comports', lambda: ports)
    assert serial_servo.resolve_port('serial://CD34') == '/dev/ttyACM4'
    assert serial_servo.resolve_port('serial:///dev/ttyUSB9') == '/dev/ttyUSB9'
    with pytest.raises(ArmHardwareError, match='ZZ'):
        serial_servo.resolve_port('serial://ZZ')
    with pytest.raises(ArmHardwareError, match='not a serial address'):
        serial_servo.resolve_port('/dev/ttyUSB0')
    monkeypatch.setattr(serial_servo.sys, 'platform', 'darwin')
    ports[0].device = '/dev/tty.usbmodem1'
    assert serial_servo.resolve_port('serial://AB12') == '/dev/cu.usbmodem1'
