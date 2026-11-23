"""The simulated demo cup: a real open cup, standing where the arm reaches and the overhead camera sees it, sized to the gripper."""
import math

import numpy as np
import pytest

from roborun_sdk import arm_profiles
from roborun_sdk.mujoco_sim import CUP, _hollow, cup_mesh


def _volume(v, f):
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    return float(np.einsum('ij,ij->i', a, np.cross(b, c)).sum() / 6)


def _frustum(r0, r1, h):
    return math.pi * h * (r0 * r0 + r0 * r1 + r1 * r1) / 3


def test_the_cup_is_a_closed_solid_with_its_faces_outward():
    v, f = _hollow(.045, .04, -.0575, .0575, .003, .005)
    want = _frustum(.04, .045, .115) - _frustum(.037, .042, .11)       # outer cup less its inside
    assert _volume(v, f) == pytest.approx(want, rel=.01)               # positive: every face points out of the solid


def test_the_cup_stands_upright_open_at_the_top_inside_its_box():
    (v, _), (lining, _) = cup_mesh(.045, .115)
    assert v[:, 2].min() == pytest.approx(-.0575) and v[:, 2].max() == pytest.approx(.0575)
    assert np.hypot(v[:, 0], v[:, 1]).max() == pytest.approx(.045, abs=1e-6)
    top = v[np.isclose(v[:, 2], .0575)]
    assert np.hypot(top[:, 0], top[:, 1]).min() > .03                 # the rim is a ring: nothing closes the top
    assert lining[:, 2].max() < .0575 and np.hypot(lining[:, 0], lining[:, 1]).max() < .045 - .002


@pytest.mark.parametrize('profile', ['yam_solo', 'maker_arm_dual', 'yam+maker_arm', 'so101', 'koch_v1_1', 'widowx_250s', 'ur5e'])
def test_a_catalog_profile_puts_a_cup_its_gripper_closes_on_where_the_camera_sees_it(profile):
    rig = arm_profiles.rig(profile)
    left = arm_profiles.info(arm_profiles.parse(profile)[0])
    cup = rig['sim_object']
    d, _, h = cup['size_mm']
    assert cup['shape'] == 'cup' and cup['fill'] > 0
    assert d <= 90 and d <= .85 * left['sides']['left']['jaw_gap_m'] * 1000 + 1 and h >= round(1.25 * d)    # koch_v1_1: taller, by hand
    assert cup['position_mm'][2] == round(h / 2)                       # standing on the table
    cam = np.asarray(rig['cameras']['top']['T_table_camera'])[:2, 3]
    xy = np.asarray(cup['position_mm'][:2]) / 1000
    assert .25 <= np.hypot(*(xy - cam)) <= .7                          # off the camera's axis (its side shows), still in view
    T = np.asarray(rig['arms']['left']['T_table_base']); base = T[:3, 3]
    c, s = T[0, 0], T[1, 0]; sx, sy, _ = left['sides']['left']['shoulder']
    shoulder = base[:2] + [c * sx - s * sy, s * sx + c * sy]
    assert np.hypot(*(xy - shoulder)) == pytest.approx(left['object_r_m'], abs=.002)    # where this arm grasps best


@pytest.mark.parametrize('model', ['so101', 'so100', 'koch_v1_1'])
def test_a_small_arm_starts_tucked_so_it_does_not_stand_over_its_cup(model):
    a = arm_profiles.info(model)['sides']['left']
    assert a['home'] != [0.0] * len(a['home']) and a['rest'] == [0.0] * len(a['rest'])     # home tucked; rest stays the model zero
    assert all(lo <= v <= hi for v, (lo, hi) in zip(a['home'], a['limits']))


def test_the_default_scene_is_the_same_cup():
    assert CUP['shape'] == 'cup' and CUP['size_mm'] == [90, 90, 115] and CUP['position_mm'][2] == pytest.approx(115 / 2)


@pytest.mark.parametrize('model', ['trossen_wxai', 'unitree_r1_a7'])
def test_a_tucked_home_keeps_the_fingers_level_and_inside_the_limits(model):
    a = arm_profiles.info(model)['sides']['left']
    assert a['home'] != a['rest'] and all(lo <= v <= hi for v, (lo, hi) in zip(a['home'], a['limits']))


@pytest.mark.parametrize('profile', ['koch_v1_1', 'piper', 'widowx_250s', 'so101'])
def test_the_overhead_camera_comes_down_for_a_small_cup_seeing_it_from_the_same_angle(profile):
    yam = np.asarray(arm_profiles.rig('yam_solo')['cameras']['top']['T_table_camera'])
    rig = arm_profiles.rig(profile); T = np.asarray(rig['cameras']['top']['T_table_camera'])
    xy = np.asarray(rig['sim_object']['position_mm'][:2]) / 1000
    assert T[:3, :3].tolist() == yam[:3, :3].tolist()                                            # still straight down
    assert arm_profiles.CAMERA_MIN_Z - 1e-6 <= T[2, 3] < yam[2, 3]
    s = T[2, 3] / yam[2, 3]
    assert T[:2, 3] - xy == pytest.approx(s * (yam[:2, 3] - np.asarray(arm_profiles.OBJECT_XY)), abs=1e-3)   # same angle to the cup
    assert yam[2, 3] == 1.04 and arm_profiles.rig('yam_solo')['cameras'] == arm_profiles.rig('maker_arm')['cameras']   # YAM-size cups: unchanged
