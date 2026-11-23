"""Metal Arm, reBot B601-DM and OpenArm 1.0 / v2 against a simulated Damiao CAN bus."""
import uuid

import pytest

pytest.importorskip('can')

from arm_contract import *  # noqa: F401,F403  the shared checks are collected from here
from arm_contract import Rig
from fake_damiao import FakeDamiaoBus



@pytest.fixture(params=['metal_arm', 'rebot_b601_dm', 'openarm_v2', 'openarm_v1'])
def rig(request, tmp_path, monkeypatch):
    def make_bus(r):
        rows = [dict(m.spec, name=m.name, id=m.id) for m in r.table.motors]
        return FakeDamiaoBus(r.address[len('virtual://'):], rows, r.start)
    r = Rig(request.param, tmp_path, monkeypatch, make_bus, address=f'virtual://{uuid.uuid4().hex}')
    yield r
    r.close()


def test_a_motor_with_other_scales_is_named_with_what_it_reports(rig):
    name = rig.joint_motors()[0].name
    rig.bus.by_name[name].tmax = 99.0
    report = rig.arm().survey()
    assert any(name in p and '99' in p for p in report['problems'])


def test_a_motor_not_in_mit_mode_is_refused(rig):
    rig.bus.by_name[rig.joint_motors()[0].name].mode = 2
    with pytest.raises(Exception, match='MIT'):
        rig.arm(connect=True)
    assert not rig.bus.enabled()


def test_a_motor_that_switches_itself_off_latches(rig):
    from arm_contract import settle, wait
    arm = rig.arm(connect=True)
    settle(arm, .2)
    rig.bus.by_name[rig.joint_motors()[0].name].enabled = False
    assert wait(lambda: arm._error is not None) and 'switched itself off' in arm._error


def test_openarm_drives_all_seven_joints(rig):
    if not rig.arm_id.startswith('openarm'):
        pytest.skip('OpenArm only')
    from arm_contract import settle, wait
    arm = rig.arm(connect=True)
    settle(arm)
    assert len(arm.home) == 7 and [m.name for m in arm.joints][2] == 'joint_3' and not arm.held
    target = list(arm.read()['positions_rad']); target[2] += .08
    def reached():
        arm.command(target, .5)
        return abs(arm.read()['positions_rad'][2] - target[2]) < .015
    assert wait(reached, 6)


def test_right_openarm_uses_its_own_shoulder_limit(rig):
    if not rig.arm_id.startswith('openarm'):
        pytest.skip('OpenArm only')
    from roborun_sdk.arms import open_arm
    assert open_arm(rig.arm_id, 'right', rig.address, connect=False).limits[1][1] == pytest.approx(1.5708, abs=.001)


def test_a_can_fd_link_gets_fd_frames(rig, monkeypatch):
    """A link brought up with `fd on` (OpenArm's -fd setup) carries FD frames with bit-rate switch, like enactic/openarm_can."""
    if rig.arm_id != 'openarm_v1':
        pytest.skip('one arm is enough')
    from roborun_sdk.arms import mit_can
    from arm_contract import settle
    sent, real_open = [], mit_can.open_bus
    def fd_bus(*a, **kw):
        bus = real_open(*a, **kw)
        bus.roborun_fd = True
        send = bus.send
        bus.send = lambda msg, *x, **y: (sent.append(msg), send(msg, *x, **y))[1]
        return bus
    monkeypatch.setattr(mit_can, 'open_bus', fd_bus)
    arm = rig.arm(connect=True)
    settle(arm, .2)
    assert sent and all(m.is_fd and m.bitrate_switch for m in sent)


def test_classic_link_with_silent_motors_names_the_fd_setup(rig):
    if rig.arm_id != 'openarm_v1':
        pytest.skip('one arm is enough')
    from roborun_sdk.arms.damiao import DamiaoArm
    arm = rig.arm()
    arm.bus = 'can0'                                   # a socketcan name: the hint is only for links
    arm._register = lambda motor, rid: None
    arm._drain = lambda: None
    _, got = DamiaoArm._probe(arm)
    assert any('fd on' in p and 'dbitrate 5000000' in p for p in got)


def test_link_mode_is_read_from_ip():
    from roborun_sdk.arms.canbus import _parse_link
    link = lambda mode: dict(flags=['UP'], linkinfo=dict(info_kind='can', info_data=dict(ctrlmode=mode, bittiming=dict(bitrate=1000000))))
    assert _parse_link(link(['FD']))['fd'] and _parse_link(link({'FD': True}))['fd']
    assert not _parse_link(link(None))['fd'] and not _parse_link(link(['LOOPBACK']))['fd']
