"""A robot is the arms it has: one or two, any mix, either side. Two YAMs and YAM + Metal keep their tuned profiles."""
import pytest

from roborun_sdk.arm_profiles import parse, rig
from roborun_sdk.arms.settings import _profile_for, one_arm_on_the_left


def hw(**sides):
    return {s: dict(arm=a, bus='can0') for s, a in sides.items()}


@pytest.mark.parametrize('sides,want', [
    (dict(left='yam', right='yam'), 'yam_dual'),
    (dict(left='yam', right='metal_arm'), 'yam_metal'),
    (dict(left='yam'), 'yam'),
    (dict(left='yam', right='maker_arm'), 'yam+maker_arm'),
    (dict(left='maker_arm', right='yam'), 'maker_arm+yam'),
    (dict(left='maker_arm', right='maker_arm'), 'maker_arm_dual'),
    (dict(left='so101', right='yam'), 'so101+yam'),
])
def test_any_mix_has_a_profile(sides, want):
    assert _profile_for(hw(**sides)) == want


def test_profile_names_map_to_the_runtime_ids_at_the_edge():
    from roborun_sdk.arm_profiles import runtime_profile, shown_profile, user_profile
    said = []
    assert (runtime_profile('yam'), runtime_profile('yam_dual'), runtime_profile('so101_dual')) == ('yam_solo', 'yam', 'so101_dual')
    assert (shown_profile('yam'), shown_profile('yam_solo'), shown_profile('yam_metal')) == ('yam_dual', 'yam', 'yam_metal')
    assert user_profile('yam_single', said.append) == 'yam' and user_profile('makerarm', said.append) == 'maker_arm_dual'
    assert said == ['profile yam_single was renamed to yam', 'profile makerarm was renamed to maker_arm_dual']
    assert user_profile('yam', said.append) == 'yam' and len(said) == 2


def test_a_single_arm_on_the_right_runs_like_one_on_the_left():
    h = one_arm_on_the_left(dict(right=dict(arm='maker_arm', bus='/dev/ttyACM0'), cameras=dict(right='/dev/video2', top='/dev/video0')))
    assert h['left']['arm'] == 'maker_arm' and 'right' not in h and h['cameras'] == dict(left='/dev/video2', top='/dev/video0')
    assert _profile_for(h) == 'maker_arm'


def test_tuned_profiles_are_never_rebuilt_from_the_catalog():
    for name in ('yam', 'yam_single', 'yam_metal', 'makerarm'):
        assert parse(name) is None
    assert parse('yam_solo') == ('yam', None)


def test_a_mixed_pair_composes_a_rig_with_both_arms():
    r = rig('yam+maker_arm')
    assert r['arms']['left']['variant'] == 'yam' and r['arms']['right']['variant'] == 'maker_arm' and 'parked' not in r['arms']['right']


def test_a_config_that_kept_the_name_but_lost_its_rig_is_written_again(tmp_path):
    import json
    from roborun_sdk.robot import profile_config_matches
    root = tmp_path; (root / 'profiles' / 'yam').mkdir(parents=True); (root / 'workspaces' / 'yam_single').mkdir(parents=True)
    (root / 'profiles' / 'yam' / 'config.json').write_text(json.dumps(dict(body='yam', rig='profiles/yam/rig.json')))
    (root / 'workspaces' / 'yam_single' / 'profile.json').write_text(json.dumps(dict(kind='arm_pair', id='yam_single', body='yam',
                                                                                   rig='profiles/yam_single/rig.json')))
    (root / 'config.json').write_text(json.dumps(dict(body='yam', profile_id='yam_single', rig='profiles/yam/rig.json')))
    assert not profile_config_matches(root, 'yam_single') and profile_config_matches(root, 'yam')
    (root / 'config.json').write_text(json.dumps(dict(body='yam', profile_id='yam_single', rig='profiles/yam_single/rig.json')))
    assert profile_config_matches(root, 'yam_single')
