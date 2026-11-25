"""Camera cards (roborun.camera/1): robot.yaml ids, aliases and card sections, plain-word validation, strict defaults per arm
variant, the worked examples of the card schema, the outbound subset, `roborun cameras describe`, the broker serving a camera
under its alias, and the simulation drawing each declared camera from its card."""
import json
import math
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from roborun_sdk import camera_card as cc
from roborun_sdk import cameras
from roborun_sdk.arms import settings
from camera_test_helpers import Scripted, frame, run_dir, serve, until  # noqa: F401

YAM_TOP = [[1, 0, 0, -0.125], [0, -1, 0, 0.75], [0, 0, -1, 1.04], [0, 0, 0, 1]]
CAD = [[0., 0., 1., -0.0704], [0., 1., 0., 0.], [-1., 0., 0., -0.0577], [0., 0., 0., 1.]]   # a stand-in rigid design mount
FACTORY = dict(width=640, height=480, fx=430.3, fy=430.3, ppx=319.6, ppy=241.2, distortion_model='inverse_brown_conrady',
               coeffs=[0., 0., 0., 0., 0.])
D405 = dict(serial='x', intrinsics=FACTORY, depth_scale_m=0.0001, depth_available=True, depth_aligned_to='color')
USB = dict(serial='y', intrinsics=dict(width=640, height=480, fx=None, fy=None, ppx=None, ppy=None, distortion_model=None,
                                      coeffs=None), depth_scale_m=None, depth_available=False, depth_aligned_to=None)


def rig(variant='yam', **cams):
    arm = lambda side: dict(variant=variant, model=f'{variant}.xml', T_table_base=np.eye(4).tolist())
    return dict(frame='table', arms=dict(left=arm('left'), right=arm('right')),
                cameras={**dict(top=dict(role='overhead', T_table_camera=YAM_TOP, fovy_deg=64.9), left={}, right={}), **cams})


def card_of(cid, entry, rig_spec=None, model=None, cad=None):
    cfgs, _ = cc.check_config({cid: entry}, rig_spec)
    settings_, _ = cameras.camera_settings(cid, entry)
    return cc.validate(cc.derive(cid, settings_, cfgs[cid], rig=rig_spec, model=model, cad=cad))


# ---------------------------------------------------------------------------------------------------- validation
@pytest.mark.parametrize('cams,words', [
    (dict(Top='usb:1'), 'camera "Top": a camera name is lower-case letters, digits and _ (up to 32), starting with a letter'),
    (dict(overhead='usb:1'), 'camera "overhead": that name is taken: call it top, or pick another name'),
    (dict(head='usb:1'), 'camera "head": that name is taken'),
    (dict(head_cam=dict(source='usb:1', card=dict(alias='head'))), 'camera head_cam: alias must be top, left or right'),
    (dict(left=dict(source='usb:1', card=dict(alias='right'))), 'camera left: it is already the left camera; remove alias or rename the camera'),
    (dict(top='usb:2', stereo=dict(source='usb:1', card=dict(alias='top'))), 'cameras top and stereo both answer as top: give one of them another alias or none'),
    (dict(left=dict(source='usb:1', card=dict(mount='left.link4'))), 'camera left: mount must be world, left.tool or right.tool, not left.link4'),
    (dict(right=dict(source='usb:1', card=dict(mount='left.tool'))), 'camera right: a camera on the left gripper can only be the left camera (a wrist camera plans its own arm)'),
    (dict(top=dict(source='usb:1', card=dict(mount='right.tool'))), 'camera top: the top camera is fixed to the table (mount world)'),
    (dict(side=dict(source='usb:1', card=dict(alias='left', mount='world'))), 'camera side: a camera fixed to the table cannot be the left camera'),
    (dict(head=dict(source='usb:1', card=dict(mount='body.head'))), 'camera "head": that name is taken'),
    (dict(face=dict(source='usb:1', card=dict(mount='body.head'))), 'camera face: body.head is for body profiles; on an arm robot use world, left.tool or right.tool'),
    (dict(top=dict(source='usb:1', card=dict(pose=[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 2, 0], [0, 0, 0, 1]]))), 'camera top: pose must be a 4x4 rigid transform (rotation and metres, last row 0 0 0 1)'),
    (dict(top=dict(source='usb:1', card=dict(pose=[[1, 0, 0]]))), 'camera top: pose must be a 4x4 rigid transform'),
    (dict(left=dict(source='usb:1', card=dict(pose=[[1, 0, 0, .6], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))), 'camera left: pose puts the camera more than 0.5 m from the gripper'),
    (dict(left=dict(source='usb:1', card=dict(depth='no'))), 'camera left: depth must be true or false'),
    (dict(left=dict(source='usb:1', card=dict(rectified=1))), 'camera left: rectified must be true or false'),
    (dict(top=dict(source='usb:1', card=dict(sees=['Table']))), 'camera top: sees is a list of up to 8 short names such as table or left.gripper'),
    (dict(top=dict(source='usb:1', card=dict(sees=[f's{i}' for i in range(9)]))), 'camera top: sees is a list'),
    (dict(top=dict(source='usb:1', card=dict(lens=dict(fx=1)))), 'camera top: card does not take "lens"; it takes alias, label, mount, pose, depth, rectified, sees, device, description, fingers'),
    (dict(top=dict(source='usb:1', card='world')), 'camera top: card must be a section (alias, mount, ...)'),
    (dict(aux=dict(source='usb:1', card=dict(pose=np.eye(4).tolist()))), 'camera aux: a pose needs a mount'),
])
def test_every_mistake_is_refused_in_plain_words_before_anything_starts(cams, words):
    with pytest.raises(ValueError, match=re.escape(words)):
        cc.check_config(cams)
    with pytest.raises(ValueError):
        cameras.camera_configs(dict(cameras=cams))                   # the broker refuses the same before it starts


def test_an_arm_the_robot_does_not_have_is_an_error_when_named_and_a_warning_by_default():
    one = rig('so101'); one['arms']['right']['parked'] = dict(positions_rad=[0] * 6)
    with pytest.raises(ValueError, match='camera aux: this robot has no right arm'):
        cc.check_config(dict(aux=dict(source='usb:1', card=dict(mount='right.tool'))), one)
    _, warnings = cc.check_config(dict(right='usb:1'), one)            # an old file with a right camera on a one-arm robot
    assert warnings == ['camera right: this robot has no right arm, so this camera has no gripper to ride']


def test_a_body_card_may_be_called_head():
    cfgs, _ = cc.check_config(dict(head='usb:1'), body=True)
    assert cfgs['head']['alias'] == 'head' and cfgs['head']['mount'] == 'body.head'


# ---------------------------------------------------------------------------------------------------- ids and aliases
def test_ids_top_left_right_are_their_own_alias_and_a_free_id_may_take_one():
    cfgs, _ = cc.check_config(dict(stereo=dict(source='usb:1', card=dict(alias='top', label='Stereo head')), left='usb:2',
                                   aux='usb:3', side=dict(source='usb:4', card=dict(mount='right.tool'))))
    assert [cfgs[c]['alias'] for c in ('stereo', 'left', 'aux', 'side')] == ['top', 'left', None, None]
    assert [cfgs[c]['mount'] for c in ('stereo', 'left', 'aux', 'side')] == ['world', 'left.tool', None, 'right.tool']
    assert cfgs['stereo']['label'] == 'Stereo head' and cfgs['left']['label'] == 'Left wrist' and cfgs['aux']['label'] == 'aux'


def test_the_broker_serves_a_camera_under_its_alias_and_a_free_one_under_its_name(serve):
    configs = cameras.camera_configs(dict(cameras=dict(stereo=dict(source='usb:1', card=dict(alias='top')), aux='usb:3')))
    assert set(configs) == {'top', 'aux'} and configs['top'][2]['id'] == 'stereo'
    top, aux = Scripted('a'), Scripted('b', depth=False)
    top.settings, aux.settings = configs['top'][2], configs['aux'][2]
    broker, path = serve(dict(top=top, aux=aux))
    got = until(lambda: frame(path), what='a top frame')
    assert got['camera_id'] == 'stereo' and got['role'] == 'top' and got['rotate'] == 0
    assert until(lambda: frame(path, 'aux'), what='the free camera')['camera_id'] == 'aux'
    status = until(lambda: broker.status() if broker.status()['state'] == 'streaming' else None, what='streaming')
    assert status['available_roles'] == ['top'] and status['extra_roles'] == ['aux']
    assert status['unavailable_roles'] == ['left', 'right'] and status['roles_without_depth'] == []    # the runtime's lists: task cameras only
    assert status['complete_observation'] is False


def test_a_class_of_your_own_gets_the_alias_as_its_role_and_never_the_card(tmp_path):
    path = tmp_path / 'my_cam.py'
    path.write_text('import numpy as np\nclass Cam:\n    def __init__(self, role, port=None): self.role, self.port = role, port\n'
                    '    def open(self): return dict(intrinsics=dict(fx=100., fy=100., ppx=20., ppy=10., distortion_model="none"), rectified=True)\n'
                    '    def read(self): return np.zeros((20, 40, 3), np.uint8)\n')
    hardware = dict(cameras=dict(stereo=dict(source=f'{path}:Cam', port='/dev/x', card=dict(alias='top', label='Stereo'))))
    sources = cameras.build_sources(hardware, str(tmp_path / 'c.sock'))
    cam = sources['top'].camera
    assert (cam.role, cam.port) == ('top', '/dev/x')                  # card is withheld; every other key still reaches it
    assert cameras.setting_warnings(cameras.camera_configs(hardware)) == []
    model = sources['top'].open()
    assert model['rectified'] is True and model['intrinsics']['distortion_model'] == 'none'
    card = card_of('stereo', hardware['cameras']['stereo'], rig(), model=dict(model, intrinsics=model['intrinsics']))
    assert card['optics']['rectified'] is True and card['optics']['pinhole']['fx'] == 100. and card['source']['kind'] == 'custom'


def test_an_old_custom_camera_section_is_unchanged(tmp_path):
    """The form robot.yaml files already use: top: {source: file.py:Class, port: ..., width, height}."""
    path = tmp_path / 'stereo.py'
    path.write_text('import numpy as np\nclass Head:\n    def __init__(self, role, width, height, port): self.args = (role, width, height, port)\n'
                    '    def read(self): return np.zeros((480, 640, 3), np.uint8)\n')
    hardware = dict(cameras=dict(top=dict(source=f'{path}:Head', port='/dev/ttyACM0', width=640, height=480),
                                 right=dict(source='usb:R'), left=dict(source='usb:L', rotate=180)))
    configs = cameras.camera_configs(hardware)
    assert configs['top'] == ('custom', f'{path}:Head', dict(source=f'{path}:Head', width=640, height=480, rotate=0), dict(port='/dev/ttyACM0'))
    assert configs['left'][2] == dict(source='usb:L', rotate=180) and configs['right'][2] == dict(source='usb:R', rotate=0)
    assert cameras.build_sources(hardware, str(tmp_path / 'c.sock'))['top'].camera.args == ('top', 640, 480, '/dev/ttyACM0')


def test_old_robot_yaml_forms_load_exactly_as_before():
    old = dict(top='realsense:123', left='usb:cam', right=dict(source='rtsp://me:pw@h/x', fps=15, exposure=dict(managed=True)))
    configs = cameras.camera_configs(dict(cameras=old))
    assert list(configs) == ['top', 'left', 'right']
    assert configs['top'][2] == dict(source='realsense:123', rotate=0) and configs['left'][2] == dict(source='usb:cam', rotate=0)
    assert configs['right'][2] == dict(source='rtsp://me:pw@h/x', fps=15, rotate=0, exposure=dict(managed=True))
    cfgs, warnings = cc.check_config(old, rig())
    assert warnings == [] and [c['mount'] for c in cfgs.values()] == ['world', 'left.tool', 'right.tool']


# ---------------------------------------------------------------------------------------------------- strict defaults
@pytest.mark.parametrize('variant,cad,source,warning', [
    ('yam', CAD, 'cad', None),
    ('yam', None, 'none', cc.ASK_ROBOT),                             # the design mount needs the runtime at hand
    ('openarm_v1', CAD, 'none', 'The left wrist camera has no measured mount on this robot (openarm_v1)'),
    ('openarm_v2', CAD, 'none', 'no measured mount on this robot (openarm_v2)'),
    ('metal_arm', CAD, 'none', 'no measured mount on this robot (metal_arm)'),
    ('big_yam', CAD, 'none', 'no measured mount on this robot (big_yam)'),   # never the YAM's mount for another arm
])
def test_a_wrist_pose_with_no_rig_mount_follows_the_arm_variant_strictly(variant, cad, source, warning):
    card = card_of('left', 'realsense:123', rig(variant), cad=cad)
    pose = card['pose']
    assert pose['source'] == source and pose['preview'] is True and pose['nominal'] is True and pose['parent'] == 'left.tool'
    assert (pose['T'] is not None) == (source == 'cad')
    assert (warning is None and pose['warning'] is None) or warning in pose['warning']
    assert card['calibration'] is None and card['planning'] is None    # the robot reports those, never the SDK


def test_a_rig_without_a_variant_leaves_the_mount_to_the_robot():
    spec = rig(); del spec['arms']['left']['variant']; spec['arms']['left']['model'] = 'yam_pro.xml'
    assert card_of('left', 'usb:1', spec, cad=CAD)['pose']['warning'] == cc.ASK_ROBOT


def test_the_rig_mount_answers_first_and_a_rotated_camera_gets_no_design_or_rig_pose():
    T = [[1, 0, 0, 0], [0, 1, 0, -.05], [0, 0, 1, -.06], [0, 0, 0, 1]]
    spec = rig('openarm_v1', left=dict(T_tool_camera=T))
    assert card_of('left', 'usb:1', spec)['pose']['source'] == 'rig_mount'
    turned = card_of('left', dict(source='usb:1', rotate=180), spec, cad=CAD)
    assert turned['pose']['T'] is None and turned['pose']['source'] == 'none' and turned['pose']['warning'] == cc.ROTATED_POSE
    assert cc.card_warnings(turned) == ['camera left: a design or rig mount is for the camera unrotated; calibrate it as delivered']
    top = card_of('top', dict(source='usb:1', rotate=90), rig())
    assert top['pose']['T'] is None and top['pose']['source'] == 'none'


def test_a_configured_pose_is_reported_only_and_never_planned_from():
    P = [[1, 0, 0, .01], [0, 1, 0, 0], [0, 0, 1, .02], [0, 0, 0, 1]]
    card = card_of('left', dict(source='usb:1', card=dict(pose=P)), rig('openarm_v1'))
    assert card['pose']['configured'] == P and card['pose']['T'] is None and card['pose']['source'] == 'none'


def test_depth_comes_from_the_camera_and_a_declaration_against_it_is_a_warning():
    card = card_of('left', dict(source='usb:1', card=dict(depth=True)), rig(), model=USB)
    assert card['optics']['depth'] == dict(available=False, scale_m=None, aligned_to=None, declared=True)
    assert 'camera left: card says depth but the camera gives none: depth comes from the camera' in cc.card_warnings(card)


@pytest.mark.parametrize('lens,declared,want', [
    (dict(FACTORY), None, True), (dict(FACTORY, coeffs=[.1, 0, 0, 0, 0]), None, False), (dict(FACTORY, coeffs=[.1, 0, 0, 0, 0]), True, True),
    (USB['intrinsics'], None, None), (USB['intrinsics'], True, True), (USB['intrinsics'], False, False),
    (dict(FACTORY, distortion_model='none', coeffs=None), None, True), (dict(FACTORY, distortion_model=None, coeffs=None), None, False)])
def test_rectified_follows_the_lens_or_a_declaration(lens, declared, want):
    assert cc.rectified(cc.normalise_intrinsics(lens), declared) is want


# ---------------------------------------------------------------------------------------------------- worked examples
def test_example_yam_dual_with_d405_wrists():
    yam = cc.rig_for('yam_dual')
    assert yam['frame'] == 'right_base' and yam['arms']['left']['variant'] == 'yam'
    left = card_of('left', 'realsense:SERIAL_B', yam, model=D405, cad=CAD)
    assert left['role'] == 'wrist' and left['label'] == 'Left wrist' and left['source'] == dict(kind='realsense', ref='realsense:SERIAL_B')
    assert left['pose'] == dict(parent='left.tool', T=CAD, source='cad', preview=True, nominal=True, warning=None)
    o = left['optics']
    assert o['delivered'] == dict(width=640, height=480, rotate=0, fps=None) and o['lens']['source'] == 'factory'
    assert o['rectified'] is True and o['pinhole'] == dict(FACTORY, distortion_model='none')
    assert (o['hfov_deg'], o['vfov_deg']) == (73.3, 58.3) and o['depth'] == dict(available=True, scale_m=.0001, aligned_to='color', declared=None)
    top = card_of('top', 'realsense:SERIAL_A', yam)                   # not opened yet: the nominal from the rig's fovy
    assert top['pose']['T'] == YAM_TOP and top['pose']['source'] == 'nominal' and top['role'] == 'overhead'
    p = top['optics']['pinhole']
    assert top['optics']['pinhole_nominal'] is True and (p['ppx'], p['ppy']) == (319.5, 239.5) and p['fx'] == p['fy'] == pytest.approx(240 / math.tan(math.radians(64.9 / 2)), abs=1e-3)   # ~377.5
    T_tool = np.asarray(CAD); T_table = np.eye(4); T_table[:3, 3] = [.1, .4, .3]
    out = cc.outbound(dict(left, pose=dict(left['pose'], preview=False)), intrinsics=dict(FACTORY, model='none', coeffs=[0] * 5),
                      T_table_camera=T_table.tolist(), T_tool_camera=T_tool.tolist(), table_z_m=0.)
    assert out == dict(name='left', role='wrist', intrinsics=dict(width=640, height=480, fx=430.3, fy=430.3, ppx=319.6, ppy=241.2,
                                                                  distortion_model='none', coeffs=[0.] * 5),
                       T_table_camera=T_table.tolist(), T_tool_camera=CAD, table_z_m=0.)
    assert set(out) <= {'name', 'intrinsics', 'T_table_camera', 'role', 'T_tool_camera', 'table_z_m'}   # the hosted CAMERA keys
    with pytest.raises(ValueError, match="outbound\\(\\) is for wrist cameras"):
        cc.outbound(top, intrinsics=FACTORY, T_table_camera=YAM_TOP, T_tool_camera=None)


def test_example_openarm_stereo_head_as_top_and_usb_wrists():
    spec = cc.rig_for('openarm_v1_dual')
    assert spec['arms']['left']['variant'] == 'openarm_v1'
    head = dict(source='~/.config/roborun/cameras/stereo_head.py:StereoHead', port='/dev/ttyACM0', width=640, height=480,
                card=dict(label='Stereo head (oblique)', mount='world'))
    opened = dict(serial='stereo', intrinsics=dict(width=640, height=480, fx=500., fy=500., ppx=322., ppy=236., distortion_model='none',
                                                     coeffs=[0.] * 5), depth_scale_m=.001, depth_available=True, depth_aligned_to='color')
    top = card_of('top', head, spec, model=opened)
    assert top['role'] == 'overhead' and top['label'] == 'Stereo head (oblique)' and top['pose']['source'] == 'nominal'
    assert top['optics']['rectified'] is True and top['optics']['pinhole']['ppx'] == 322. and top['optics']['depth']['scale_m'] == .001
    left = card_of('left', dict(source='usb:ID_L', rotate=180, card=dict(depth=False)), spec, model=USB, cad=CAD)
    assert left['pose'] == dict(parent='left.tool', T=None, source='none', preview=True, nominal=True,
                                warning='The left wrist camera has no measured mount on this robot (openarm_v1): its position on the '
                                        'gripper is not known yet, so motion cannot be planned from it')
    o = left['optics']
    assert o['delivered'] == dict(width=640, height=480, rotate=180, fps=None) and o['rectified'] is None and o['pinhole'] is None
    assert o['lens'] == dict(USB['intrinsics'], source='none')
    assert o['depth'] == dict(available=False, scale_m=None, aligned_to=None, declared=False)
    with pytest.raises(ValueError, match='the left camera has no pinhole intrinsics yet'):
        cc.outbound(left, intrinsics=FACTORY, T_table_camera=YAM_TOP, T_tool_camera=CAD)


def test_example_booster_k1_head_camera():
    doc, warnings = cc.describe({}, profile='booster_k1')
    card = doc['cameras'][0]
    assert warnings == [] and doc['table_frame'] is None and card['id'] == card['alias'] == 'head' and card['role'] == 'egocentric'
    assert card['source'] == dict(kind='body', ref='booster_k1') and card['mount'] == 'body.head'
    assert card['pose'] == dict(parent='body.head', T=None, source='profile', preview=False, nominal=True, warning=None,
                                body=dict(eye_height_m=0.888, pitch_rad=0.3, head_dof=2))
    o = card['optics']
    assert o['pinhole'] == dict(width=544, height=488, fx=208.7129, fy=208.7129, ppx=271.5, ppy=243.5, distortion_model='none',
                                coeffs=[0.] * 5)
    assert o['pinhole_nominal'] is True and (o['hfov_deg'], o['vfov_deg']) == (105.0, 98.9) and o['lens'] is None and o['rectified'] is None
    from roborun_sdk.body import profiles
    assert {k: o['pinhole'][k] for k in ('fx', 'ppx', 'ppy')} == {k: profiles.intrinsics(544, 488, 105)[k] for k in ('fx', 'ppx', 'ppy')}
    with pytest.raises(ValueError, match='outbound'):
        cc.outbound(card, intrinsics=o['pinhole'], T_table_camera=YAM_TOP, T_tool_camera=CAD)


# ---------------------------------------------------------------------------------------------------- outbound
def _wrist():
    return dict(card_of('left', 'realsense:1', rig(), model=D405, cad=CAD))


@pytest.mark.parametrize('change,words', [
    (dict(intrinsics=dict(FACTORY, fx=431.)), "the left frame's intrinsics are not the camera's pinhole"),
    (dict(intrinsics=dict(FACTORY, width=320)), "the left frame's intrinsics are not the camera's pinhole"),
    (dict(intrinsics=dict(FACTORY, fx=None)), "the left frame's intrinsics"),
    (dict(T_table_camera=None), "the left wrist camera's pose at the exposure is needed"),
    (dict(T_tool_camera=[[1, 0]]), 'the left wrist camera has no known mount')])
def test_outbound_refuses_what_the_hosted_api_would_misread(change, words):
    kw = dict(intrinsics=FACTORY, T_table_camera=YAM_TOP, T_tool_camera=CAD)
    kw.update(change)
    with pytest.raises(ValueError, match=words):
        cc.outbound(_wrist(), **kw)


def test_outbound_sends_camera_facts_only():
    card = _wrist()
    card['sees'] = ['table']; card['device'] = 'wrist camera'
    out = cc.outbound(card, intrinsics=FACTORY, T_table_camera=YAM_TOP, T_tool_camera=CAD)
    text = json.dumps(out)
    assert 'realsense' not in text and 'sees' not in out and 'model' not in out['intrinsics'] and 'table_z_m' not in out
    described = dict(card, description='Left wrist wide-angle view', fingers='Two white rubber pads at the bottom')
    out2 = cc.outbound(described, intrinsics=FACTORY, T_table_camera=YAM_TOP, T_tool_camera=CAD)
    assert out2['description'] == 'Left wrist wide-angle view' and out2['fingers'] == 'Two white rubber pads at the bottom'
    nomount = dict(card, pose=dict(card['pose'], T=None, source='none'))
    with pytest.raises(ValueError, match='the left wrist camera has no known mount: motion cannot be planned from it'):
        cc.outbound(nomount, intrinsics=FACTORY, T_table_camera=YAM_TOP, T_tool_camera=CAD)


def test_outbound_refuses_a_frame_that_was_never_rectified_even_when_its_k_is_the_pinhole():
    """A merged card's pinhole is often exactly the raw lens K (the runtime rectifies to the same K): an unrectified frame
    then matches the pinhole numerically, but its pixels still carry the lens distortion and must not go out as a pinhole."""
    raw = dict(width=640, height=480, fx=400., fy=401., ppx=320., ppy=240., distortion_model='brown_conrady',
               coeffs=[.2, -.3, 0., 0., .1])
    card = _wrist()
    card['optics'] = dict(card['optics'], lens=dict(raw, source='calibration'), rectified=True, pinhole_nominal=False,
                          pinhole=dict(raw, distortion_model='none', coeffs=[0.] * 5))
    with pytest.raises(ValueError, match='the left frame is not rectified'):
        cc.outbound(card, intrinsics=raw, T_table_camera=YAM_TOP, T_tool_camera=CAD)
    with pytest.raises(ValueError, match='the left frame is not rectified'):
        cc.outbound(card, intrinsics={k: v for k, v in raw.items() if k not in ('distortion_model', 'coeffs')},
                    T_table_camera=YAM_TOP, T_tool_camera=CAD)
    for served in (dict(raw, distortion_model='none', coeffs=[0.] * 5), dict(raw, coeffs=[0.] * 5),
                   dict({k: v for k, v in raw.items() if k != 'distortion_model'}, model='pinhole', coeffs=None)):
        out = cc.outbound(card, intrinsics=served, T_table_camera=YAM_TOP, T_tool_camera=CAD)
        assert out['intrinsics'] == dict(width=640, height=480, fx=400., fy=401., ppx=320., ppy=240., distortion_model='none',
                                         coeffs=[0.] * 5)


def test_a_calibration_record_normalises_to_the_card_vocabulary():
    rec = dict(model='brown', K=[[400, 0, 320], [0, 401, 240], [0, 0, 1]], D=[.1, -.2, 0, 0, 0], size=[640, 480])
    assert cc.normalise_intrinsics(rec) == dict(width=640, height=480, fx=400., fy=401., ppx=320., ppy=240.,
                                                distortion_model='brown_conrady', coeffs=[.1, -.2, 0., 0., 0.])
    assert cc.normalise_intrinsics(dict(width=640, height=480, fx=1., fy=1., ppx=1., ppy=1., model='none', coeffs=[0.] * 5))['distortion_model'] == 'none'


# ---------------------------------------------------------------------------------------------------- back-compat rewrites
def test_a_single_right_arm_takes_its_cards_to_the_left_slot():
    hw = dict(right=dict(arm='so101'), cameras=dict(right=dict(source='usb:1', card=dict(label='mine')),
                                                    aux=dict(source='usb:2', card=dict(mount='right.tool')),
                                                    stereo=dict(source='usb:3', card=dict(alias='right'))))
    out = settings.one_arm_on_the_left(hw)
    with pytest.raises(ValueError, match='both answer as left'):        # moved right + a free id aliased right: now both left
        cc.check_config(out['cameras'])
    del hw['cameras']['stereo']
    out = settings.one_arm_on_the_left(hw)
    assert set(out) == {'left', 'cameras'} and out['cameras']['aux']['card']['mount'] == 'left.tool'
    assert out['cameras']['left']['card'] == dict(label='mine') and 'right' not in out['cameras']
    cc.check_config(out['cameras'], dict(arms=dict(left=dict(variant='so101'))))


def test_camera_flag_takes_the_role_from_a_free_camera_for_this_run():
    said = []
    cams = cc.overlay_cameras(dict(stereo=dict(source='usb:1', card=dict(alias='top', label='S'))), 'top', 'usb:9', say=said.append)
    assert cams == dict(stereo=dict(source='usb:1', card=dict(label='S')), top='usb:9')
    assert said == ['camera stereo: --camera top replaces it as the top camera for this run']
    cc.check_config(cams)


def test_setup_keeps_cards_and_free_names_and_gives_a_held_role_to_its_holder():
    from roborun_sdk.guided import _setup_cameras
    existing = dict(top=dict(source='usb:old', card=dict(label='mine')), left='usb:L', aux='usb:A',
                    stereo=dict(source='usb:S', card=dict(alias='right')))
    out = _setup_cameras(dict(right='realsense:1'), existing)
    assert out == dict(stereo='realsense:1', left=None)                # top has a card, aux is yours: both stay
    with pytest.raises(SystemExit, match='Nothing was written'):
        _setup_cameras(dict(top='usb:1'), dict(Top='usb:2'))


# ---------------------------------------------------------------------------------------------------- roborun cameras describe
def _describe(tmp_path, text, capsys, *extra, json_out=True):
    path = tmp_path / 'robot.yaml'
    path.write_text(text)
    from roborun_sdk import cli
    code = cli.main(['cameras', 'describe', *(['--json'] if json_out else []), '--config', str(path),
                     '--socket', str(tmp_path / 'none.sock'), *extra])
    out = capsys.readouterr()
    return code, out.out, out.err


def test_describe_prints_every_card_and_opens_nothing(tmp_path, capsys):
    code, out, err = _describe(tmp_path, 'profile: yam_dual\ncameras:\n  top: realsense:123\n  aux:\n    source: usb:3\n'
                                         '    card: {label: Door camera, sees: [table]}\n', capsys)
    doc = json.loads(out)
    assert code == 0 and doc['schema'] == 'roborun.cameras/1' and doc['profile'] == 'yam_dual' and doc['table_frame'] == 'right_base'
    assert [c['id'] for c in doc['cameras']] == ['top', 'aux'] and doc['cameras'][1]['mount'] is None
    assert doc['cameras'][1]['sees'] == ['table'] and doc['cameras'][1]['role'] is None and doc['cameras'][1]['pose'] is None
    for card in doc['cameras']:
        cc.validate(card)
    assert sorted(p.name for p in tmp_path.iterdir()) == ['robot.yaml']                  # nothing written
    code, out, _ = _describe(tmp_path, 'profile: yam_dual\ncameras:\n  top: realsense:123\n', capsys, json_out=False)
    assert code == 0 and out.startswith('Profile yam_dual, table frame right_base.')
    assert 'top (Overhead): a table camera fixed to the table, RealSense' in out and 'ask the robot (GET /v1/cameras)' in out


def test_describe_exits_1_with_the_plain_error(tmp_path, capsys):
    code, out, err = _describe(tmp_path, 'cameras:\n  overhead: usb:1\n', capsys)
    assert code == 1 and out == '' and 'camera "overhead": that name is taken' in err


def test_describe_reads_the_opened_models_from_the_camera_service(tmp_path, capsys, serve):
    top = Scripted('a', intrinsics=dict(FACTORY, width=64, height=48))
    top.settings = dict(source='usb:1', rotate=0)
    broker, path = serve(dict(top=top))
    until(lambda: broker.status()['models'].get('top'), what='an opened camera')
    yaml = tmp_path / 'robot.yaml'
    yaml.write_text('profile: yam_dual\ncameras:\n  top: usb:1\n')
    from roborun_sdk import cli
    assert cli.main(['cameras', 'describe', '--json', '--config', str(yaml), '--socket', path]) == 0
    out = capsys.readouterr().out                                    # the broker's own log lines come first
    card = json.loads(out[out.index('{\n "schema"'):])['cameras'][0]
    assert card['optics']['delivered']['width'] == 64 and card['optics']['depth']['available'] is True
    assert card['optics']['rectified'] is True and 'pinhole_nominal' not in card['optics']


# ---------------------------------------------------------------------------------------------------- the simulation
def test_the_simulation_keeps_every_wrist_camera_and_takes_only_carded_entries():
    from roborun_sdk.mujoco_sim import SIZE, sim_cameras
    spec = rig()
    del spec['cameras']['right']                                     # as yam_metal: the rig lists top and left only
    plain = sim_cameras(spec)
    assert set(plain) == {'top', 'left', 'right'}
    assert plain['right']['size'] == SIZE and plain['right']['depth'] is True and plain['right']['fovy'] is None
    one = dict(spec, arms=dict(left=spec['arms']['left']))
    assert set(sim_cameras(one)) == {'top', 'left'}                  # no right arm, no right wrist camera
    real = dict(top=dict(source='usb:1', width=1280, height=720), left=dict(source='usb:2', rotate=90), Back='usb:3')
    assert sim_cameras(spec, real) == plain                          # no card: sizes and bad entries change nothing, as before
    carded = sim_cameras(spec, dict(real, top=dict(real['top'], card={}), right=dict(source='usb:4', card=dict(depth=False))))
    assert carded['top']['size'] == (1280, 720) and carded['right']['depth'] is False and carded['left'] == plain['left']
    with pytest.raises(ValueError):
        sim_cameras(spec, dict(top=dict(source='usb:1', card=dict(mount='nowhere'))))


SIM_SCRIPT = Path(__file__).with_name('sim_camera_cards.py')


@pytest.mark.parametrize('scenario', ['wrist_pose', 'rgb_only', 'as_before', 'unchanged_setup'])
def test_the_simulation_draws_each_declared_camera_from_its_card(scenario, tmp_path):
    """Rendered in a fresh process per scenario (one GL context per process, EGL else OSMesa), as the simulation runs:
    wrist_pose  a wrist card at a known pose: the image is where the card's pose and intrinsics say, and moves with the gripper
    rgb_only    an RGB-only role serves no depth and says so (frames, status, cards)
    as_before   with no robot.yaml cameras, every role renders as it always has
    unchanged_setup  an arm the rig lists no camera for keeps its wrist camera; robot.yaml entries without a card change nothing"""
    pytest.importorskip('mujoco')
    runtime = Path(os.environ.get('ROBORUN_TEST_RUNTIME') or Path.home() / 'roborun' / 'robot' / 'runtime')
    kin = runtime / 'kin.py'                       # the robot runtime roborun up downloads (ROBORUN_TEST_RUNTIME: another copy)
    if not kin.exists():
        pytest.skip('no robot runtime here (roborun up downloads it; ROBORUN_TEST_RUNTIME names another copy)')
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(Path(__file__).resolve().parents[1] / 'src'), str(Path(__file__).parent),
                                                        os.environ.get('PYTHONPATH', '')]))
    env.setdefault('MUJOCO_GL', 'egl')
    try:
        run = subprocess.run([sys.executable, str(SIM_SCRIPT), scenario, str(kin), str(tmp_path)], env=env, capture_output=True,
                             text=True, timeout=300)
    except subprocess.TimeoutExpired:
        pytest.skip('the simulation could not render here within 300 s')
    if 'CANNOT RENDER' in run.stdout:
        pytest.skip(f'the simulation cannot render here: {run.stdout.strip()[-300:]}')
    assert run.returncode == 0 and 'PASSED' in run.stdout, run.stdout[-3000:] + run.stderr[-3000:]


def test_the_card_functions_are_on_the_stable_surface_and_an_older_tree_still_compares(tmp_path):
    import importlib.util
    sdk = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location('stable_surface_cards', sdk / 'tools' / 'stable_surface.py')
    tool = importlib.util.module_from_spec(spec); spec.loader.exec_module(tool)
    now = tool.surface(sdk)
    assert now['roborun_sdk.camera_card:outbound'] == '(card, *, intrinsics, T_table_camera, T_tool_camera, table_z_m=None)'
    old = tmp_path / 'old' / 'roborun_sdk'
    for rel in ('client.py', 'errors.py', 'arms/__init__.py', 'arms/base.py', 'cameras.py'):    # a release before the cards
        (old / rel).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(sdk / 'src' / 'roborun_sdk' / rel, old / rel)
    removed, changed, added = tool.compare(tool.surface(tmp_path / 'old'), now)
    assert not removed and not changed and 'roborun_sdk.camera_card:check_config' in added
