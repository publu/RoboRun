"""Existing robots keep their arm count across the rename (`yam` is now ONE YAM arm, `yam_dual` two): a robot that ran two YAMs
under the old SDK (robot.json "profile": "yam", the runtime's two-arm id) still runs two after the upgrade, whether it is started
with plain `roborun up`, with an old robot.yaml saying `profile: yam`, or with the old docs' `roborun up --profile yam --sim cup`.
Every change of meaning is said in one line; nothing changes the arm count silently."""
import json
from types import SimpleNamespace

import pytest

from roborun_sdk import robot
from roborun_sdk.arm_profiles import shown_profile
from roborun_sdk.arms import settings

OLD_YAML = """# Your robot's local settings. `roborun setup` fills these in; edit anything, then run: roborun up
robot: lab-pair
profile: yam                 # written by the old guided setup: two YAM arms then
arms:
  left:
    arm: yam
    address: slcan://A
  right:
    arm: yam
    address: slcan://B
"""


@pytest.fixture
def home(tmp_path, monkeypatch):
    monkeypatch.setattr(robot, 'HOME', tmp_path); monkeypatch.setattr(robot, 'CONFIG', tmp_path / 'robot.json')
    for name in ('ROBORUN_TOKEN', 'ROBORUN_API_KEY'):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setattr(robot, 'check_token', lambda server, key: None)
    (tmp_path / 'token').write_text('rr_test_x\n')
    return tmp_path


def old_robot_json(home, profile='yam', **extra):
    """robot.json as the old SDK saved it: no profile_names marker, the runtime's profile id."""
    (home / 'robot.json').write_text(json.dumps(dict(server='https://api.example', robot='lab-pair', key_file=str(home / 'token'),
                                                     profile=profile, backend='http://127.0.0.1:8120', **extra)))


def up_args(**kw):
    return SimpleNamespace(**dict(dict(left=None, right=None, camera=None, arm_options=None, profile=None, profile_id=None, rig=None,
                                       robot=None, key_file=None, sim=None, server=None), **kw))


def resolve(home, capsys, **kw):
    """What `roborun up` decides before it starts anything: the runtime profile it switches to (None: keeps the running one)."""
    a = up_args(**kw)
    robot.resolve_profile(a)
    return robot._rid(a), capsys.readouterr().out


def test_a_plain_up_on_a_robot_that_ran_two_yams_keeps_them(home, capsys):
    old_robot_json(home)
    rid, said = resolve(home, capsys)
    assert rid is None                                            # no profile asked for: the running two-arm `yam` stays
    assert 'renamed' not in said and 'one YAM arm' not in said
    assert json.loads((home / 'robot.json').read_text())['profile_names'] == robot.PROFILE_NAMES
    assert shown_profile('yam') == 'yam_dual'                     # and status / up / profiles call it yam_dual


def test_an_old_robot_yaml_saying_yam_becomes_yam_dual_once_and_says_so(home, capsys):
    old_robot_json(home)
    (home / 'robot.yaml').write_text(OLD_YAML)
    rid, said = resolve(home, capsys)
    assert rid == 'yam'                                           # two YAMs, the runtime's id, unchanged
    text = (home / 'robot.yaml').read_text()
    assert 'profile: yam_dual' in text and 'written by the old guided setup' in text and 'arm: yam' in text   # comments kept
    assert said.count('changed it to yam_dual') == 1
    rid, said = resolve(home, capsys)                             # once: the next start says nothing
    assert rid == 'yam' and said == ''


def test_an_old_robot_yaml_saying_yam_with_two_arms_named_migrates_before_any_robot_json(home, capsys):
    (home / 'robot.yaml').write_text(OLD_YAML)                    # robot.json gone (or never kept): the two arms named decide
    rid, said = resolve(home, capsys)
    assert rid == 'yam' and 'profile: yam_dual' in (home / 'robot.yaml').read_text() and 'changed it to yam_dual' in said


def test_robot_yaml_yam_on_a_one_arm_robot_stays_one_arm(home, capsys):
    old_robot_json(home, profile='yam_solo')                      # the old SDK's single YAM
    (home / 'robot.yaml').write_text('profile: yam\narms: {left: {arm: yam, address: "slcan://A"}}\n')
    rid, said = resolve(home, capsys)
    assert rid == 'yam_solo' and 'profile: yam\n' in (home / 'robot.yaml').read_text() and said == ''


def test_the_old_docs_profile_yam_sim_cup_keeps_two_arms_and_says_so(home, capsys):
    old_robot_json(home)
    rid, said = resolve(home, capsys, profile='yam', sim='cup')
    assert rid == 'yam'                                           # two arms, as the old instruction meant
    assert 'profile yam is now one YAM arm; your robot ran two' in said and 'yam_dual' in said


def test_robot_yaml_yam_written_after_the_upgrade_still_never_drops_an_arm_silently(home, capsys):
    old_robot_json(home, profile_names=robot.PROFILE_NAMES, **{robot.OLD_TWO_YAMS: True})   # migrated; then `profile: yam` by hand
    (home / 'robot.yaml').write_text('profile: yam\n')
    rid, said = resolve(home, capsys)
    assert rid == 'yam' and 'profile yam is now one YAM arm' in said


def test_one_arm_on_purpose_and_the_page_switch_are_obeyed(home, capsys):
    old_robot_json(home)
    rid, said = resolve(home, capsys, profile='yam_solo')
    assert rid == 'yam_solo' and 'one YAM arm' not in said
    rid, said = resolve(home, capsys, profile_id='yam_solo')      # the page's switch names the runtime id as is
    assert rid == 'yam_solo'


def test_a_new_robot_follows_the_new_names(home, capsys):
    rid, _ = resolve(home, capsys, profile='yam', sim='cup')      # nothing ran here before: yam is one YAM arm
    assert rid == 'yam_solo'
    rid, _ = resolve(home, capsys, sim='cup')                     # and the simulator's default is two
    assert rid == 'yam'


def test_a_robot_set_up_by_this_sdk_as_two_yams_takes_profile_yam_as_one_arm(home, capsys):
    old_robot_json(home, profile_names=robot.PROFILE_NAMES)      # this SDK ran `roborun up --sim cup` (two arms) here
    rid, said = resolve(home, capsys, profile='yam', sim='cup')
    assert rid == 'yam_solo' and 'your robot ran two' not in said  # the new docs' `yam` is one arm


def test_the_old_two_yam_mark_is_set_once_and_kept(home, capsys):
    old_robot_json(home)
    resolve(home, capsys)
    assert json.loads((home / 'robot.json').read_text())[robot.OLD_TWO_YAMS] is True
    rid, said = resolve(home, capsys, profile='yam')
    assert rid == 'yam' and 'your robot ran two' in said
