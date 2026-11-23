"""One local file drives the robot: plain `roborun up` reads it, and people can change anything in it."""
import time
from types import SimpleNamespace

import pytest

from roborun_sdk import robot
from roborun_sdk.arms import settings
from roborun_sdk.arms.base import ArmHardwareError, StreamedArm
from roborun_sdk.arms.hardware import config_from_args


class Bench(StreamedArm):
    limits = [(-3, 3)] * 6
    def __init__(self, **kw):
        super().__init__(**kw); self.q = [0.] * 6; self.jaw = .5
    def open_hardware(self): pass
    def read_hardware(self): return dict(positions_rad=list(self.q), gripper=self.jaw)
    def write_hardware(self, q, jaw): self.q = list(q); self.jaw = jaw
    def limp_hardware(self): pass


@pytest.fixture
def home(tmp_path, monkeypatch):
    monkeypatch.setattr(robot, 'HOME', tmp_path); monkeypatch.setattr(robot, 'CONFIG', tmp_path / 'robot.json')
    for name in ('ROBORUN_TOKEN', 'ROBORUN_API_KEY'):                         # a key in the shell running the suite plays no part
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setattr(robot, 'check_token', lambda server, key: None)       # no API: a token check decides nothing, at once
    return tmp_path


def args(**kw):
    return SimpleNamespace(**dict(dict(left=None, right=None, camera=None, arm_options=None, profile=None, rig=None, robot=None,
                                       key_file=None, sim=None), **kw))


def write(home, text):
    (home / 'robot.yaml').write_text(text)


def test_the_template_is_valid_and_turns_nothing_on(home):
    settings.ensure_file()
    assert settings.load() == {} and settings.effective('stored') == 'stored'


def test_the_file_becomes_the_hardware_with_motion_defaults_and_arm_overrides(home):
    write(home, """
arms:
  left: {arm: yam, address: 'slcan://A', joint_speed_rad_s: 0.2, my_own_knob: 7}
  right: {arm: my_arm.py:MyArm}
cameras: {top: 'realsense:1'}
motion: {joint_speed_rad_s: 0.5, jaw_speed: 0.3}
privacy: {face_blur: false}
rules: my_rules.py
""")
    hw = config_from_args(args(), None)
    assert hw['left']['bus'] == 'slcan://A'
    assert hw['left']['options'] == dict(joint_speed_rad_s=.2, jaw_speed=.3, my_own_knob=7, rules=str(home / 'my_rules.py'))
    assert hw['right']['arm'] == f'{home}/my_arm.py:MyArm' and hw['cameras'] == dict(top='realsense:1')
    assert hw['privacy'] == dict(face_blur=False)


def test_a_flag_still_overrides_one_arm_for_this_run(home):
    write(home, 'arms: {left: {arm: yam, address: "slcan://A"}, right: {arm: yam, address: "slcan://B"}}')
    hw = config_from_args(args(left='yam@slcan://C'), None)
    assert hw['left']['bus'] == 'slcan://C' and hw['right']['bus'] == 'slcan://B'


def test_a_broken_file_and_an_unknown_section_are_named(home):
    write(home, 'arms: [')
    with pytest.raises(SystemExit, match='robot.yaml is not valid'): settings.load()
    write(home, 'armz: {}'); said = []
    settings.load(said.append); assert "'armz'" in said[0]


def test_a_simulated_arm_starts_only_when_asked_for(home, monkeypatch):
    monkeypatch.setenv('ROBORUN_TOKEN', 'secret-token')
    a = args(sim='cup'); settings.first_run(a)
    assert (a.profile, a.sim) == ('yam_dual', 'cup') and a.robot            # the simulator's default: two YAMs
    token = home / 'token'
    assert token.read_text() == 'secret-token\n' and token.stat().st_mode & 0o777 == 0o600 and a.key_file == str(token)


def test_sim_in_robot_yaml_asks_for_the_simulator(home, monkeypatch):
    monkeypatch.setenv('ROBORUN_TOKEN', 't')
    write(home, 'sim: pickup-bench\n')
    a = args(); settings.first_run(a)
    assert (a.profile, a.sim) == ('yam_dual', 'pickup-bench')


def test_no_arms_and_no_terminal_starts_nothing_and_says_what_to_do(home, monkeypatch):
    monkeypatch.setattr('sys.stdin', SimpleNamespace(isatty=lambda: False))
    with pytest.raises(SystemExit) as stopped: settings.first_run(args())
    text = str(stopped.value)
    assert 'roborun setup' in text and 'roborun up --sim cup' in text and 'roborun hw check' in text


def test_a_first_up_without_a_token_says_where_it_goes(home, monkeypatch):
    monkeypatch.setattr('sys.stdin', SimpleNamespace(isatty=lambda: False))
    with pytest.raises(SystemExit, match='ROBORUN_TOKEN'): settings.first_run(args())


def test_a_first_up_with_arms_uses_their_profile(home):
    (home / 'token').write_text('t\n')
    write(home, 'arms: {left: {arm: yam, address: "slcan://A"}}')
    a = args(); settings.first_run(a); assert a.profile == 'yam' and a.sim is None          # one YAM: `yam` (the runtime's yam_solo)
    write(home, 'arms: {left: {arm: so101, address: "serial:///dev/x"}}')
    a = args(); settings.first_run(a); assert a.profile == 'so101'
    write(home, 'arms: {left: {arm: so101, address: "serial:///dev/x"}, right: {arm: koch_v1_1, address: "serial:///dev/y"}}')
    a = args(); settings.first_run(a); assert a.profile == 'so101+koch_v1_1'
    write(home, 'arms: {left: {arm: robo_t1, address: "x"}}')                 # no public model: the user names a profile
    with pytest.raises(SystemExit, match='no model for robo_t1.*set "profile:" in .*robot.yaml'): settings.first_run(args())
    write(home, 'profile: yam\narms: {left: {arm: robo_t1, address: "x"}}')   # ... and then that profile runs
    a = args(); settings.first_run(a); assert a.profile == 'yam'


def test_motion_settings_reach_the_arm():
    arm = Bench(joint_speed_rad_s=.1, joint_accel_rad_s2=.5, jaw_speed=.2)
    assert (arm.joint_speed_rad_s, arm.joint_accel_rad_s2, arm.jaw_speed) == (.1, .5, .2)
    with pytest.raises(ValueError, match='joint_speed_rad_s'): Bench(joint_speed_rad_s=0)


def rules(tmp_path, body):
    f = tmp_path / 'rules.py'; f.write_text('def check(side, positions_rad, gripper, state):\n' + body)
    return str(f)


def test_a_rule_can_change_or_refuse_a_command(tmp_path):
    arm = Bench(rules=rules(tmp_path, '    if positions_rad[0] > 1: return False\n    return [min(v, .5) for v in positions_rad], gripper\n'))
    arm.connect(); arm.command([.9] * 6, .5); assert arm._target == [.5] * 6
    arm.command([2.] * 6, .5); assert arm._target != [2.] * 6
    time.sleep(.05); arm.close()


def test_a_rule_that_breaks_stops_the_arm(tmp_path):
    arm = Bench(rules=rules(tmp_path, '    raise RuntimeError("oops")\n')); arm.connect()
    with pytest.raises(ArmHardwareError, match='oops'): arm.command([.1] * 6, .5)
    with pytest.raises(ArmHardwareError): arm.read()
    arm.close()


def test_the_maker_arm_gripper_uses_the_makers_temperature_limit():
    from roborun_sdk.arms.safety import JAW_TEMPS, MOTOR_TEMPS
    assert JAW_TEMPS['rs00'] == (45.0, 55.0) and MOTOR_TEMPS['rs00'] == (70.0, 80.0)


def test_both_arms_on_one_adapter_is_said_as_a_settings_mistake(home):
    write(home, 'arms: {left: {arm: yam, address: "slcan://A"}, right: {arm: yam, address: "slcan://A"}}')
    with pytest.raises(SystemExit, match='both arms are set to the same adapter, slcan://A'): config_from_args(args(), None)
    write(home, 'arms: {left: {arm: so101, address: "serial:///dev/x"}, right: {arm: koch_v1_1, address: "serial:///dev/y"}}')
    assert config_from_args(args(), None)['right']['bus'] == 'serial:///dev/y'
