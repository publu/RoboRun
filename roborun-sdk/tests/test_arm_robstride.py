"""The MakerMods Maker Arm against a simulated RobStride CAN bus."""
import uuid

import pytest

pytest.importorskip('can')

from arm_contract import *  # noqa: F401,F403  the shared checks are collected from here
from arm_contract import Rig, settle, wait
from fake_robstride import FakeRobStrideBus
from roborun_sdk.arms.base import ArmHardwareError



@pytest.fixture(params=[True, False], ids=['current-firmware', 'firmware-without-parameters'])
def rig(request, tmp_path, monkeypatch):
    def make_bus(r):
        rows = [dict(m.spec, name=m.name, id=m.id) for m in r.table.motors]
        return FakeRobStrideBus(r.address[len('virtual://'):], rows, r.start, parameters=request.param)
    r = Rig('maker_arm', tmp_path, monkeypatch, make_bus, address=f'virtual://{uuid.uuid4().hex}')
    yield r
    r.close()


def test_a_motor_outside_mit_mode_is_refused(rig):
    if not rig.bus.parameters:
        pytest.skip('old firmware cannot report its run mode')
    rig.bus.by_name['elbow_flex'].run_mode = 1
    with pytest.raises(ArmHardwareError, match='run mode'):
        rig.arm(connect=True)
    assert not rig.bus.enabled()


def test_connect_never_clears_a_motor_fault(rig):
    rig.bus.fault('wrist_flex')
    with pytest.raises(ArmHardwareError, match='stall'):
        rig.arm(connect=True)
    assert rig.bus.by_name['wrist_flex'].fault_word and not rig.bus.enabled()


def test_a_fault_is_named_while_streaming(rig):
    arm = rig.arm(connect=True)
    settle(arm, .2)
    rig.bus.fault('shoulder_lift')
    assert wait(lambda: arm._error is not None) and 'shoulder_lift' in arm._error


def test_a_whole_turn_after_a_power_cycle_is_corrected(rig):
    import math
    rig.bus.move_by_hand('shoulder_pan', rig.start['shoulder_pan'] + 2 * math.pi)
    arm = rig.arm(connect=True)
    assert arm.read()['positions_rad'][0] == pytest.approx(arm.joints[0].to_joint(rig.start['shoulder_pan'] + 2 * math.pi), abs=.02)
    assert abs(arm.joints[0].turn) == pytest.approx(2 * math.pi)
