"""roborun profile create and the station for arms of any joint count, on a small generated arm model (no motion data involved)."""
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

mujoco = pytest.importorskip('mujoco')
ROOT = Path(__file__).resolve().parents[2]
for p in (ROOT/'roborun-station/src', ROOT/'roborun-sdk/src'):
    if str(p) not in sys.path: sys.path.insert(0, str(p))

from roborun_sdk import profile_create as pc                  # noqa: E402
from roborun_sdk.embodiment import KinematicArm, _station_classes      # noqa: E402


def links(n):
    body, close = '', ''
    for i in range(1, n+1):
        axis = '0 0 1' if i % 2 else '0 1 0'
        body += f'<body name="l{i}" pos="0 0 .12"><joint name="a{i}" axis="{axis}" range="-2.5 2.5"/><geom type="capsule" fromto="0 0 0 0 0 .12" size=".03"/>'
        close += '</body>'
    return body, close


def model(tmp, n, jaw):
    body, close = links(n)
    fingers = ('<body name="fl" pos="0 .01 .05"><joint name="f1" type="slide" axis="0 1 0" range="0 .03"/><geom type="box" size=".01 .005 .02"/></body>'
               '<body name="fr" pos="0 -.01 .05"><joint name="f2" type="slide" axis="0 -1 0" range="0 .03"/><geom type="box" size=".01 .005 .02"/></body>') if jaw == 'slide' else (
               '<body name="d" pos="0 .01 .05"><joint name="drive" axis="1 0 0" range="0 .8"/><geom type="box" size=".01 .005 .02"/></body>'
               '<body name="e" pos="0 -.01 .05"><joint name="follow" axis="1 0 0" range="-.8 0"/><geom type="box" size=".01 .005 .02"/></body>')
    eq = '<equality><joint joint1="follow" joint2="drive" polycoef="0 -1 0 0 0"/></equality>' if jaw != 'slide' else ''
    path = tmp/'arm.xml'
    path.write_text(f'<mujoco><compiler angle="radian"/><worldbody><body name="b0"><geom type="sphere" size=".05"/>{body}<body name="hand"><geom type="box" size=".02 .04 .02"/>{fingers}</body>{close}</body></worldbody>{eq}</mujoco>')
    return path


def args(tmp, n, fingers, **kw):
    return SimpleNamespace(name='demo', urdf=str(model(tmp, n, 'slide' if ',' in fingers else 'rev')), joints=','.join(f'a{i+1}' for i in range(n)), fingers=fingers,
                           tcp_link='hand', tcp_offset='0,0,.08', tcp_quat=None, base_link='b0', first_link='l1', home=','.join(['.3']*n), arms=1, place=None,
                           base_profile='yam', manifest=None, jaw_closed_rad=kw.get('closed'), jaw_open_rad=kw.get('opened'), jaw_gap_m=None)


@pytest.fixture
def home(tmp_path, monkeypatch):
    from roborun_sdk import robot
    monkeypatch.setattr(robot, 'HOME', tmp_path/'home')
    return tmp_path


@pytest.mark.parametrize('n', (6, 7, 8))
def test_two_finger_arm_of_n_joints(home, n, capsys):
    assert pc.create(args(home, n, 'f1,f2')) == 0
    d = home/'home/profiles/demo'; rig = json.loads((d/'rig.json').read_text()); arm = rig['arms']['left']
    assert arm.get('joints', 6) == n and len(arm['home']) == n and ('joints' in arm) == (n != 6)
    m = mujoco.MjModel.from_xml_path(str(home/'home/models/custom/demo/demo.xml'))
    names = [m.joint(i).name for i in range(m.njnt)]
    assert names == [f'joint{i+1}' for i in range(n+2)]
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'gripper') >= 0 and mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'tip_right') >= 0


def test_one_revolute_jaw_driver(home, capsys):
    assert pc.create(args(home, 7, 'drive', closed=0., opened=.8)) == 0
    arm = json.loads((home/'home/profiles/demo/rig.json').read_text())['arms']['left']
    assert arm['jaw'] == dict(type='revolute', joint='joint8', closed_rad=0., open_rad=.8)


def test_refusals(home):
    with pytest.raises(SystemExit, match='needs --jaw-closed-rad'): pc.create(args(home, 7, 'drive'))
    bad = args(home, 7, 'f1,f2'); bad.home = '0,0'
    with pytest.raises(SystemExit, match='7 joint angles'): pc.create(bad)


def station(n):
    limits = [(-3., 3.)]*n
    answers = []
    def predict(body):
        answers.append(body); return answers[-1]['_reply'] if '_reply' in body else reply[0]
    reply = [None]
    cls = _station_classes()
    st = cls(KinematicArm(home=[0.]*n, limits=limits), predict=predict, lease_free_hold=True)
    return st, reply, answers


@pytest.mark.parametrize('n', (6, 7))
def test_station_takes_answers_of_joints_plus_jaw(n):
    st, reply, _ = station(n)
    sid = st.register_session(arm='left'); st.action({'observation_id': st.observe()['observation_id'], 'seconds': 6, 'reference': {'kind': 'video_motion', 'session_id': sid}})
    reply[0] = dict(target=[.01]*n+[.99], acquired_at=st.clock())
    st._refill(st.clock())
    assert st.lease['active'] and st.lease['answers'] == 1 and st.left.q_cmd == pytest.approx([.01]*n)
    assert len(st.pose()['measured']['left']['positions_rad']) == n
    reply[0] = dict(target=[.01]*(n+1), acquired_at=st.clock())         # one value short of joints + jaw: refused
    st._refill(st.clock())
    assert not st.lease['active'] and st.lease['reason'] == 'prediction_error'


def test_adapter_needs_a_limit_per_joint():
    class Arm(KinematicArm):
        limits = [(-1., 1.)]*3
    with pytest.raises(ValueError, match='3 limits for 7 joints'):
        _station_classes()(Arm(home=[0.]*7, limits=[(-1., 1.)]*3))
    assert math.isclose(KinematicArm(home=[0.]*7).limits[6][1], math.pi)
