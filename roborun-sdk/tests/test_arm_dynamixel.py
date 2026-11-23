"""Koch v1.1, OMX, WidowX 250 6DOF and ViperX 300 6DOF against simulated X-series servos on a fake serial port."""
import math

import pytest

pytest.importorskip('serial')

from arm_contract import *  # noqa: F401,F403,E402  the shared checks are collected from here
from arm_contract import Rig, settle, wait  # noqa: E402
from fake_dynamixel import COUNTS, FakeDynamixelBus  # noqa: E402
from roborun_sdk.arms.base import ArmHardwareError  # noqa: E402
from roborun_sdk.arms.dynamixel import MODELS, crc16, stuff, unstuff  # noqa: E402

TO_COUNTS = COUNTS / (2 * math.pi)


class View:
    def __init__(self, fake, ids): self.fake, self.ids = fake, ids
    def servo(self, name): return self.fake.servos[self.ids[name]]
    def pos(self, name): return self.servo(name).actual / TO_COUNTS
    def move_by_hand(self, name, pos): self.servo(name).actual = pos * TO_COUNTS
    def enabled(self): return [n for n in self.ids if self.servo(n).mem[64]]
    def enable_frames(self): return self.fake.torque_writes_on()
    def fault(self, name): s = self.servo(name); s.alert = 1; s.mem[70] = 0x20
    def silence(self, name): self.servo(name).mute = True
    def remove(self, name): self.silence(name)
    def wrong_motor(self, name): self.servo(name).mem[0:2] = (777).to_bytes(2, 'little')
    def target(self, name): return self.servo(name).goal() / TO_COUNTS
    def set_temp(self, name, value): self.servo(name).temp = int(value)
    def block(self, name): self.servo(name).blocked = True
    def restore(self, name): self.servo(name).mute = False
    def make_heavy(self): self.fake.sag = 400.0; self.fake.hold_needs = 320
    def set_load(self, value): self.fake.load = value
    def power_on(self, name):
        s, spec = self.servo(name), self.specs[name]
        s.mem[11] = int(spec.get('mode', 3))
        for address, size, value in spec.get('registers', ()):
            s.mem[address:address + size] = int(value).to_bytes(size, 'little', signed=value < 0)
        s.mem[116:120] = round(s.actual).to_bytes(4, 'little', signed=True); s.mem[64] = 1; s.armed = True
    def close(self): self.fake.close()


@pytest.fixture(params=['koch_v1_1', 'omx', 'widowx_250s', 'viperx_300s'])
def rig(request, tmp_path, monkeypatch):
    def make_bus(r):
        rows = {m.id: (r.start[m.name] * TO_COUNTS, MODELS[m.model], None) for m in r.table.motors}
        for shadow in r.table.shadows:
            master = next(m for m in r.table.motors if m.id == shadow['master'])
            rows[shadow['id']] = (r.start[master.name] * TO_COUNTS, MODELS[shadow['model']], shadow['master'])
        fake = FakeDynamixelBus(rows).start()
        r.address = fake.address
        view = View(fake, {m.name: m.id for m in r.table.motors}); view.specs = {m.name: m.spec for m in r.table.motors}; return view
    r = Rig(request.param, tmp_path, monkeypatch, make_bus, jaw=(1.0, 2.2), zero=lambda i: 3.2 + .1 * i)
    yield r
    r.close()


def test_packets_are_stuffed_and_checked_like_the_robotis_sdk():
    assert crc16(bytes.fromhex('FFFFFD0001030001')) == 0x4E19   # the PING example in the protocol 2.0 manual
    raw = b'\x01\xff\xff\xfd\x02'
    assert stuff(raw) == b'\x01\xff\xff\xfd\xfd\x02' and unstuff(stuff(raw)) == raw


def test_release_lowers_pwm_and_keeps_torque(rig):
    arm = rig.arm(connect=True)
    settle(arm)
    name = rig.joint_motors()[0].name
    arm.release()
    assert wait(lambda: int.from_bytes(rig.bus.servo(name).mem[100:102], 'little') == 300, 1)
    assert rig.bus.servo(name).mem[64] == 1


def test_operating_modes_come_from_the_table(rig):
    arm = rig.arm(connect=True)
    for motor in rig.table.motors:
        assert rig.bus.servo(motor.name).mem[11] == motor.spec['mode']
    if rig.table.gripper.spec['mode'] == 5:
        assert int.from_bytes(rig.bus.servo('gripper').mem[102:104], 'little') == 300
    assert arm.read()


def test_a_shadow_that_does_not_follow_its_master_is_refused(rig):
    if not rig.table.shadows:
        pytest.skip('no doubled joints')
    rig.bus.fake.servos[3].mem[12] = 255
    with pytest.raises(ArmHardwareError, match='shoulder_shadow'):
        rig.arm(connect=True)
    assert not rig.bus.enabled()


def test_shadows_follow_and_are_never_sent_their_own_goal(rig):
    if not rig.table.shadows:
        pytest.skip('no doubled joints')
    arm = rig.arm(connect=True)
    settle(arm)
    assert rig.bus.fake.servos[3].mem[64] == 1
    for packet in rig.bus.fake.packets:
        params = unstuff(packet[8:-2])
        if packet[7] == 0x83 and int.from_bytes(params[:2], 'little') == 116:
            ids = params[4::5]
            assert 3 not in ids and 5 not in ids
