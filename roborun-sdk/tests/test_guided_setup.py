"""The first run a new person has: the real `roborun` CLI on a terminal, simulated hardware in another process.

CAN arms sit on virtual CAN links or behind a simulated slcan adapter, serial arms on pseudo-terminals, and the terminal
is a pty. Run inside `unshare -rn` (the suite's rule); the virtual CAN parts need that namespace.
"""
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent / 'guided_env'))
from drive import Rig, TOKEN  # noqa: E402

pytestmark = pytest.mark.skipif(os.geteuid() != 0 or subprocess.run(['ip', 'link', 'add', 'gtest0', 'type', 'vcan'], capture_output=True).returncode != 0
                                or subprocess.run(['ip', 'link', 'del', 'gtest0'], capture_output=True).returncode != 0,
                                reason='needs unshare -rn with the vcan module')


@pytest.fixture
def rigs(tmp_path):
    made = []

    def make(*arms, cameras=None):
        made.append(Rig(tmp_path / f'rig{len(made)}', [dict(a) for a in arms], cameras))
        return made[-1]
    yield make
    for rig in made:
        rig.close()


SAFE = r'Is the arm supported and the area around it clear\?'
NO_DEPTH = 'Tasks need a depth camera (e.g. RealSense). Your webcam will show and record.'
HOST = __import__('roborun_sdk.arms.settings', fromlist=['x']).default_robot_name()     # the robot's name: never asked
SO101 = dict(name='A', arm='so101', via='serial')
KOCH = dict(name='B', arm='koch_v1_1', via='serial', serial='SIMB')
YAM = dict(name='A', arm='yam', via='vcan')


def to_arms(s, rig):
    """Signing in the way `roborun login` does: open the link, approve, and setup carries on by itself."""
    s.expect(r'Open http://127\.0\.0\.1:\d+/approve\?code=ABCD-1234 and approve this computer', 60)
    s.expect('Waiting for your approval', 30)
    rig.approve()
    s.expect('Signed in', 30)


def no_cameras(s):
    s.expect('No cameras plugged in, so none are set up', 60)          # never a question: setup carries on


def pick(s, rig, last=lambda: None):
    """Says which arm this really is, the way the person would: from the suggestions. `last` runs before the final answer."""
    at = s.expect(r'The (?:(?:left|right) )?arm, on (\S+):', 60)[1]
    arm = rig.arm_at[at]
    m = s.expect(r'Is that your arm|Which arm is (?:on the (?:left|right)|this)|SO-100 instead\? \[n\]', 30)
    if m[0].startswith('SO-100'):
        assert arm in ('so100', 'so101'), arm
        last()
        return s.send('y' if arm == 'so100' else '')
    if m[0].startswith('Is that'):
        assert f'({arm})' in s.seen[s.seen.rindex('This looks like'):], s.seen[-600:]
        last()
        return s.send('y')
    number = __import__('re').findall(rf'(\d+)\. [^\n]*\({arm}\)  ', s.seen[s.seen.rindex('The ', 0, m.start()):])[-1]
    s.send(number)
    s.expect('Use this arm')
    last()
    s.send('y')


def calibrate_by_hand(s, rig, name='A'):
    s.expect(r'A picture of each step: https://roborun\.app/docs/calibration/\w+\.png', 90)
    s.answer(SAFE, 'y', 30)
    s.answer('zero pose.*press Enter', '')
    s.expect('Move every joint slowly', 30)
    rig.cmd(f'sweep {name}')
    s.send()
    s.expect('saved', 30)


def movement_check(s, joints):
    """One yes before power; every joint then moves in turn with no question; one question after."""
    s.answer(SAFE, 'y', 90)
    s.expect('Ctrl-C if anything moves wrong', 60)
    for n in range(1, joints + 1):
        s.expect(rf'joint {n}: ok', 90)
    s.expect('gripper: ok', 60)
    s.answer(r'Every joint moved as shown\? \[y\]', '', 60)
    s.expect('passed the movement check', 90)


def skip_movement_check(s):
    s.answer(SAFE, 'n', 90)
    s.expect('Movement check skipped, and recorded', 30)


def move_alone(rig, arm, camera, samples=12, timeout=60):
    """A person moving one arm by hand, the other still, until the setup has looked at the cameras `samples` times (the
    wrist check needs 8 such looks per arm): waits on what the setup has seen, not on the clock, so a slow machine passes."""
    import time
    start = rig.cmd('frames')['frames'].get(camera, 0)
    rig.cmd(f'wiggle {arm} {camera} 0')
    end = time.monotonic() + timeout
    try:
        while rig.cmd('frames')['frames'].get(camera, 0) < start + samples:
            assert time.monotonic() < end, f'the setup looked at {camera} only {rig.cmd("frames")["frames"].get(camera, 0) - start} times'
            time.sleep(.05)
    finally:
        rig.cmd(f'still {arm}')


def finish_without_starting(s):
    s.expect(f'This robot is called {HOST}', 60)
    s.answer('Start Roborun now', 'n', 60)
    assert s.wait(30) == 0


def serial_arm_setup(rig, joints, *start, name='A', motion=True):
    s = rig.roborun(*(start or ('setup',)))
    to_arms(s, rig)
    pick(s, rig)
    if rig.has_cameras:
        s.expect('it is the top camera', 60)                                # one camera: nothing to ask
    else:
        no_cameras(s)
    calibrate_by_hand(s, rig, name)
    if motion:
        movement_check(s, joints)
    else:
        skip_movement_check(s)
    return s


def checks(rig):
    folder = rig.config / 'checks'
    return {f.stem: json.loads(f.read_text())['result'] for f in folder.glob('*.json')} if folder.is_dir() else {}


# -- full first runs, one per motor family --------------------------------------------------------------------------------
def test_yam_on_virtual_can_all_the_way_to_a_running_robot(rigs):
    rig = rigs(YAM, cameras=dict(camA=120, camB=90))
    s = rig.roborun('up')
    to_arms(s, rig)
    s.answer('Which arm is this', '1')
    s.answer('Use this arm', 'y')
    assert rig.enables()['A'] == 0 and rig.direct_frames()['A'] == 0        # identifying asked registers only
    s.answer('TOP camera', 'h'); s.answer('Keep the cameras uncovered', '')
    s.expect('Now cover'); rig.cmd('cover camA'); s.send(); s.answer('uncover it', ''); rig.cmd('uncover camA')
    s.answer('Is it the top camera', 'y')
    s.answer('WRIST camera', 'n')
    s.expect('OK: all 7 motors', 60)
    assert rig.enables()['A'] == 0                                          # the read-only check enabled nothing
    s.expect('https://roborun.app/docs/calibration/yam.png', 60)
    s.expect(SAFE, 30)
    assert rig.enables()['A'] == 0 and not rig.enabled()                    # asked before anything was powered
    s.send('y')
    s.expect('Done: the jaw travels', 90)
    movement_check(s, 6)
    s.expect(f'This robot is called {HOST}', 60)
    s.expect('ready: open https://app.roborun.app', 240)
    s.expect('Next: open https://app.roborun.app', 30)
    assert s.wait(60) == 0
    # a webcam and no depth camera: setup finishes, and says so in exactly one line, at its end
    assert s.seen.count(NO_DEPTH) == 1 and s.seen.rstrip().endswith(NO_DEPTH) and 'RGB only, without depth' not in s.seen
    y = rig.yaml()
    assert y['arms']['left'] == dict(arm='yam', address='vcan0') and y['cameras'] == dict(top='usb:camA') and y['profile'] == 'yam'
    assert y['robot'] == HOST and 'sim' not in y
    assert checks(rig) == {'yam_vcan0': 'passed'}
    stored = json.loads((rig.config / 'robot.json').read_text())
    assert 'sim' not in stored and stored['hardware']['left']['arm'] == 'yam'
    assert TOKEN not in s.seen and TOKEN not in (rig.config / 'setup-state.json').read_text()
    assert (rig.config / 'token').stat().st_mode & 0o777 == 0o600
    assert 'movement check' not in s.seen.split('ready: open')[1]           # it passed, so `up` has nothing to warn about


def test_feetech_arm_by_hand_calibration_and_movement_check(rigs):
    rig = rigs(SO101)
    s = serial_arm_setup(rig, 5)
    finish_without_starting(s)
    assert not rig.enabled()
    y = rig.yaml()
    assert y['arms']['left'] == dict(arm='so101', address='serial://SIMA') and y['profile'] == 'so101'
    assert (rig.config / 'calibration' / 'so101_serial_SIMA.json').is_file() and checks(rig) == {'so101_serial_SIMA': 'passed'}


def test_dynamixel_arm(rigs):
    rig = rigs(dict(KOCH, name='A', serial='SIMA'))
    s = serial_arm_setup(rig, 5)
    finish_without_starting(s)
    assert not rig.enabled() and rig.yaml()['arms']['left']['arm'] == 'koch_v1_1'


def test_damiao_arm_behind_a_usb_slcan_adapter(rigs):
    rig = rigs(dict(name='A', arm='metal_arm', via='slcan'))
    s = rig.roborun('setup')
    to_arms(s, rig)
    pick(s, rig)
    assert rig.direct_frames()['A'] == 0
    no_cameras(s)
    s.answer("gripper range of the arm by hand", 'n', 60)
    skip_movement_check(s)
    finish_without_starting(s)
    assert checks(rig) == {'metal_arm_slcan_SIMA': 'skipped'} and rig.enables()['A'] == 0
    assert rig.yaml()['arms']['left'] == dict(arm='metal_arm', address='slcan://SIMA')


def test_robstride_arm_on_virtual_can_is_found_without_touching_a_motor(rigs):
    rig = rigs(dict(name='A', arm='maker_arm', via='vcan'))
    s = rig.roborun('setup')
    to_arms(s, rig)
    s.expect('7 RobStride motors answered', 60)
    pick(s, rig)
    assert rig.direct_frames()['A'] == 0 and rig.enables()['A'] == 0
    no_cameras(s)
    s.answer('gripper range of the arm by hand', 'n', 60)
    movement_check(s, 6)
    finish_without_starting(s)
    assert not rig.enabled() and checks(rig) == {'maker_arm_vcan0': 'passed'}


def test_two_different_arms_told_apart_by_unplugging(rigs):
    rig = rigs(SO101, KOCH, cameras=dict(top=100, wristL=110, wristR=120))
    s = rig.roborun('setup')
    to_arms(s, rig)
    s.answer('How many arms', '2')
    s.answer('Which adapter goes to the LEFT arm', 'u')
    s.expect("Unplug the USB plug of the LEFT")
    rig.cmd('unplug B')                                                     # the koch is on the left
    s.send()
    s.expect('That is serial://SIMB')
    s.expect('Plug it back in'); rig.cmd('replug B'); s.send()
    pick(s, rig)                                                  # koch
    pick(s, rig)                                                  # so101: the remaining adapter
    s.answer('TOP camera', '1')
    s.answer('LEFT camera', '1')
    s.answer('RIGHT camera', '1')
    calibrate_by_hand(s, rig, 'B')
    calibrate_by_hand(s, rig, 'A')
    s.answer('Check that the wrist cameras', 'y', 30)
    s.expect('move the LEFT arm by hand')
    s.send()
    move_alone(rig, 'B', 'wristL')                                          # wristL really rides on the koch: correct so far
    move_alone(rig, 'A', 'wristR')
    s.expect('Wrist cameras: ok', 40)
    skip_movement_check(s); skip_movement_check(s)
    finish_without_starting(s)
    y = rig.yaml()
    assert y['arms'] == dict(left=dict(arm='koch_v1_1', address='serial://SIMB'), right=dict(arm='so101', address='serial://SIMA'))
    assert y['profile'] == 'koch_v1_1+so101' and y['cameras'] == dict(top='usb:top', left='usb:wristL', right='usb:wristR')


def test_swapped_wrist_cameras_are_caught_and_swapped(rigs):
    rig = rigs(SO101, KOCH, cameras=dict(wristL=110, wristR=120))
    s = rig.roborun('setup')
    to_arms(s, rig)
    s.answer('How many arms', '2')
    s.answer('Which adapter goes to the LEFT arm', '1')
    pick(s, rig); pick(s, rig)
    s.answer('TOP camera', 'n')
    s.answer('LEFT camera', '2')                                            # wristR, on the wrong arm
    s.answer('RIGHT camera', '1')
    calibrate_by_hand(s, rig, 'A'); calibrate_by_hand(s, rig, 'B')
    s.answer('Check that the wrist cameras', 'y', 30)
    s.expect('move the LEFT arm by hand')
    s.send()
    move_alone(rig, 'A', 'wristL')                                          # so101 is left; its real camera is wristL
    move_alone(rig, 'B', 'wristR')
    s.expect('Wrist cameras: swapped', 40)
    s.answer('Swap the left and right', 'y')
    skip_movement_check(s); skip_movement_check(s)
    finish_without_starting(s)
    assert rig.yaml()['cameras'] == dict(left='usb:wristL', right='usb:wristR')


def overhead_rig(rigs):
    rig = rigs(SO101, cameras=dict(camA=100))
    rig.env['GUIDED_OVERHEAD'] = '1'
    s = serial_arm_setup(rig, 5, 'up')
    s.expect(f'This robot is called {HOST}', 60)
    s.expect('Overhead camera: found, not confirmed', 240)
    return rig, s


def test_a_second_pose_confirms_the_overhead_camera_after_an_explicit_yes(rigs):
    rig, s = overhead_rig(rigs)
    m = s.expect(r'one slow move[^\n]*jaws empty\?', 60)
    assert 'cm across the table' in m[0] and rig.cmd('state')
    s.send('y')
    s.expect('overhead camera: calibrated', 90); s.expect('tasks can run', 10)
    assert s.wait(60) == 0


def test_saying_no_to_both_ways_moves_nothing_and_says_what_that_means(rigs):
    rig, s = overhead_rig(rigs)
    s.answer(r'jaws empty\?', 'n', 60)
    s.answer('by hand instead', 'n', 30)
    s.expect('will not run', 30); s.expect('not confirmed yet', 30)
    assert s.wait(60) == 0


def test_ctrl_c_during_the_second_pose_leaves_the_arm_held_and_roborun_running(rigs):
    rig, s = overhead_rig(rigs)
    s.answer(r'jaws empty\?', 'y', 60)
    import time; time.sleep(1.5)
    s.ctrl_c()
    s.expect('holding where it is', 30)
    assert s.wait(30) == 130 and rig.enabled()


# -- when things are not as hoped --------------------------------------------------------------------------------------------
def test_nothing_connected_says_what_to_check_and_rescans(rigs):
    rig = rigs(dict(SO101, plugged=False))
    s = rig.roborun('up')
    to_arms(s, rig)
    s.expect('dialout'); s.expect('udev rule')
    s.expect('What now')
    s.send('r'); s.expect('What now')
    rig.cmd('replug A')
    s.send('r')
    s.expect('adapter serial://SIMA', 30)
    s.expect(r'SO-100 instead\? \[n\]')
    s.ctrl_c(); assert s.wait(20) == 130


def test_enter_never_traps_a_person_with_no_arm(rigs):
    rig = rigs()
    s = rig.roborun('setup')
    to_arms(s, rig)
    s.expect('udev rule'); s.answer(r'What now\? \[r\]', '')                  # the first look: Enter looks again
    m = s.expect(r'What now\? \[s\] \(Enter: simulated arms now; plug your arm in and run roborun setup any time\)')
    assert 'Still no arm adapter found.' in s.seen and s.seen.count('udev rule') == 1   # the checklist once, not on every look
    s.send('')                                                                          # Enter moves forward: simulated arms
    finish_without_starting(s)
    assert rig.yaml()['sim'] == 'cup' and rig.yaml()['profile'] == 'yam_dual'


def test_the_simulator_is_only_an_explicit_choice(rigs):
    rig = rigs()
    s = rig.roborun('setup')
    to_arms(s, rig)
    s.answer('What now', 's')
    finish_without_starting(s)
    assert 'Two simulated YAM arms will run' in s.seen
    assert rig.yaml()['sim'] == 'cup' and rig.yaml()['profile'] == 'yam_dual' and not rig.yaml().get('arms')


def test_an_adapter_unplugged_mid_setup_is_explained_and_recovered(rigs):
    rig = rigs(SO101)
    s = rig.roborun('setup')
    to_arms(s, rig)
    pick(s, rig, last=lambda: rig.cmd('unplug A'))                         # unplugged before setup goes on to read it
    s.expect('this adapter is not connected', 60)
    s.answer('What now', 'r')
    s.expect('this adapter is not connected', 30)
    rig.cmd('replug A')
    s.answer('What now', 'r')
    s.expect('OK: all 6 motors', 60)
    s.ctrl_c(); s.expect('Setup paused', 30)


def test_a_wrong_arm_is_caught_by_the_read_only_check(rigs):
    rig = rigs(YAM)
    s = rig.roborun('setup')
    to_arms(s, rig)
    s.answer('Which arm is this', 'l')
    s.answer('Number or name', 'metal_arm')
    s.expect('not yet run on real hardware by us')
    s.answer('Use this arm', 'y')
    no_cameras(s)
    s.expect('does not look like a metal_arm', 60)
    assert rig.enables()['A'] == 0
    s.answer('What now', 'a')
    s.answer('Which arm is this', '1', 60)
    m = s.answer('Use this arm', 'y')
    assert 'YAM (I2RT) (yam)' in s.seen[:m.start()][-300:] and 'not yet run on real hardware' in s.seen[:m.start()][-300:]   # beta: said
    s.expect('OK: all 7 motors', 90)
    s.ctrl_c(); assert s.wait(20) == 130


def test_arms_that_cannot_grip_or_have_no_driver_say_so_when_chosen(rigs):
    rig = rigs(dict(KOCH, name='A', serial='SIMA'))
    s = rig.roborun('setup')
    to_arms(s, rig)
    if s.expect(r'Is that your arm|Which arm is this', 60)[0].startswith('Is that'):
        s.send('n'); s.expect('Which arm is this')
    s.send('l')
    s.answer('Number or name', 'robo_t1')
    s.expect('bring your own'); s.expect('cannot drive this arm by itself')
    s.answer('Which arm is this', 'l')
    s.answer('Number or name', 'ur5e')
    s.expect('it cannot grip yet')
    s.answer('Use this arm', 'n')
    s.answer('Which arm is this', 'l')
    s.answer('Number or name', 'koch_v1_1')
    s.expect('not yet run on real hardware by us'); s.ctrl_c(); assert s.wait(20) == 130


def test_a_powered_off_arm_is_not_guessed(rigs):
    rig = rigs(SO101)
    rig.cmd('silence A')
    s = rig.roborun('setup')
    to_arms(s, rig)
    s.expect('no motor answered')
    s.expect('did not identify this arm')
    s.answer('Which arm is this', 'l')
    s.ctrl_c(); assert s.wait(20) == 130


# -- Ctrl-C ----------------------------------------------------------------------------------------------------------------
STAGES = ['token', 'adapter_menu', 'arm_choice', 'camera', 'calibration', 'before_power', 'between_joints', 'while_moving']


@pytest.mark.parametrize('stage', STAGES)
def test_ctrl_c_at_every_stage_leaves_no_motor_on_and_resumes(rigs, stage):
    rig = rigs(SO101, cameras=dict(camA=100, camB=120))
    s = rig.roborun('setup')
    def stop_here():
        s.ctrl_c()
        s.expect('Setup paused', 30)
        assert s.wait(30) == 130
        assert not rig.enabled(), 'a motor was left on'
    if stage == 'token':
        s.expect('Waiting for your approval'); return stop_here()
    to_arms(s, rig)
    if stage == 'adapter_menu':
        s.expect('serial://SIMA: 6 Feetech'); return stop_here()
    if stage == 'arm_choice':
        s.expect('SO-100 instead'); return stop_here()
    pick(s, rig)
    if stage == 'camera':
        s.expect('TOP camera'); stop_here()
        s = rig.roborun('setup'); s.expect('Picking up where you left off'); s.answer('TOP camera', '1'); s.answer('WRIST camera', 'n')
        s.expect(SAFE, 60)
        s.ctrl_c(); s.expect('Setup paused', 30); return
    s.answer('TOP camera', '1'); s.answer('WRIST camera', 'n')
    if stage == 'calibration':
        s.answer(SAFE, 'y', 60)
        s.expect('zero pose'); return stop_here()
    calibrate_by_hand(s, rig)
    if stage == 'before_power':
        s.expect(SAFE, 60); return stop_here()
    s.answer(SAFE, 'y', 60)
    s.expect('Ctrl-C if anything moves wrong', 60)
    if stage == 'between_joints':
        s.expect('joint 1: ok', 60)
        assert rig.enabled(), 'the check should be powered here'
        stop_here()
        assert checks(rig)['so101_serial_SIMA'] == 'stopped'
        s = rig.roborun('setup')                                            # and it carries on, with the arm's check to do again
        s.expect('Picking up where you left off')
        skip_movement_check(s)
        finish_without_starting(s)
        return
    s.expect('Joint 2 of 5', 60)
    import time; time.sleep(1.0)                                            # joint 2 is moving when Ctrl-C lands
    stop_here()
    assert checks(rig)['so101_serial_SIMA'] == 'stopped'


def test_a_refused_movement_answer_stops_the_check_at_once(rigs):
    rig = rigs(SO101)
    s = serial_arm_setup(rig, 0, motion=False)
    finish_without_starting(s)
    assert checks(rig)['so101_serial_SIMA'] == 'skipped' and rig.enables()['A'] == 0     # no yes, no power
    s = rig.roborun('setup')
    s.answer('What would you like to do', '5', 30)
    s.answer(SAFE, 'y', 60)
    s.expect('gripper: ok', 90)
    s.answer(r'Every joint moved as shown\? \[y\]', 'n', 60)             # the person saw something wrong: refused
    s.expect('you saw a joint move other than as shown', 30)
    s.answer('What now', 'q', 30); assert s.wait(30) == 130
    assert checks(rig)['so101_serial_SIMA'] == 'failed' and not rig.enabled()


# -- again, and without a terminal ------------------------------------------------------------------------------------------
def test_a_second_run_keeps_what_the_person_wrote(rigs):
    rig = rigs(SO101, cameras=dict(camA=100, camB=120))
    s = rig.roborun('setup')
    to_arms(s, rig); pick(s, rig)
    s.answer('TOP camera', '1'); s.answer('WRIST camera', 'n')
    calibrate_by_hand(s, rig)
    skip_movement_check(s)
    finish_without_starting(s)
    file = rig.config / 'robot.yaml'
    text = file.read_text()
    text = text.replace('    address: serial://SIMA', '    address: serial://SIMA   # the arm by the window\n    joint_speed_rad_s: 0.3')
    text += '\n# my own note\nmotion:\n  temp_stop_c: 60 # cooler, please\nprivacy:\n  face_blur: false\n'
    file.write_text(text)
    s = rig.roborun('setup')
    s.answer('What would you like to do', '3', 30)                          # change the cameras
    s.answer('TOP camera', '2', 30); s.answer('WRIST camera', 'n', 30)
    s.answer('Start Roborun now', 'n', 60); assert s.wait(30) == 0
    after = file.read_text()
    y = rig.yaml()
    assert '# the arm by the window' in after and '# my own note' in after and '# cooler, please' in after
    assert y['arms']['left']['joint_speed_rad_s'] == .3 and y['motion'] == dict(temp_stop_c=60) and y['privacy'] == dict(face_blur=False)
    assert y['cameras'] == dict(top='usb:camB') and y['arms']['left']['address'] == 'serial://SIMA'
    assert '# Your own rules, in Python' in after                           # the template's own help stays
    assert (rig.config / 'robot.yaml.before-setup').read_text() == text


def test_a_file_written_in_flow_style_is_still_updated_correctly(rigs):
    rig = rigs(SO101, cameras=dict(camA=100))
    rig.config.mkdir(parents=True, exist_ok=True)
    (rig.config / 'token').write_text(TOKEN + '\n')
    (rig.config / 'robot.yaml').write_text('# mine\nmotion: {joint_speed_rad_s: 0.2}\ncameras: {top: {source: "usb:old", fps: 15}}\n')
    s = rig.roborun('setup')
    pick(s, rig)
    s.expect('it is the top camera', 60)
    calibrate_by_hand(s, rig)
    skip_movement_check(s)
    finish_without_starting(s)
    y = rig.yaml()
    assert y['motion'] == dict(joint_speed_rad_s=.2) and y['cameras']['top'] == dict(source='usb:camA', fps=15) and y['arms']['left']['arm'] == 'so101'
    assert (rig.config / 'robot.yaml').read_text().startswith('# mine')


def test_without_a_terminal_up_starts_nothing_and_prints_the_way_forward(rigs):
    rig = rigs(SO101)
    done = rig.plain('up')
    assert done.returncode != 0
    for text in ('there is no terminal', 'roborun hw detect', 'roborun hw arms', 'hw check ARM@ADDRESS --move', 'roborun up --sim cup', 'ROBORUN_TOKEN'):
        assert text in done.stdout, text
    assert not (rig.config / 'robot.json').exists() and rig.enables()['A'] == 0
    assert 'simulation' not in done.stdout.lower().replace('the simulator', '')


def test_detect_works_without_a_terminal_and_touches_nothing(rigs):
    rig = rigs(YAM, cameras=dict(camA=100))
    done = rig.plain('setup', '--detect', '--json')
    out = json.loads(done.stdout[done.stdout.index('{'):])
    assert out['adapters'][0]['could_be'][0]['arm'] == 'yam' and out['cameras'][0]['source'] == 'usb:camA'
    assert rig.enables()['A'] == 0 and rig.direct_frames()['A'] == 0


def test_sim_in_robot_yaml_runs_without_a_terminal(rigs):
    rig = rigs()
    rig.config.mkdir(parents=True, exist_ok=True)
    (rig.config / 'token').write_text(TOKEN + '\n')
    (rig.config / 'robot.yaml').write_text('sim: cup\n')
    done = rig.plain('up', timeout=240, env=dict(MUJOCO_GL='osmesa'))
    assert 'there is no terminal' not in done.stdout
    assert json.loads((rig.config / 'robot.json').read_text()).get('sim') == 'cup'


# -- parity: a Damiao motor in another control mode, and a LeRobot calibration ----------------------------------------
def test_a_damiao_motor_in_another_mode_is_repaired_only_after_a_yes(rigs):
    rig = rigs(dict(name='A', arm='metal_arm', via='slcan', modes={'J2': 2}))
    s = rig.roborun('setup')
    to_arms(s, rig)
    pick(s, rig)
    no_cameras(s)
    s.expect(r'J2 \(id 0x2\): CTRL_MODE 2 \(position-speed\) -> 1 \(MIT\)', 90)
    assert rig.cmd('modes')['A']['J2'] == [2, 2]
    s.answer('Write it', 'y', 30)
    s.expect('read back, saved', 30)
    s.expect('OK: all 7 motors', 60)
    assert rig.cmd('modes')['A']['J2'] == [1, 1] and rig.enables()['A'] == 0
    s.answer("gripper range of the arm by hand", 'n', 60)
    skip_movement_check(s)
    finish_without_starting(s)


def test_saying_no_to_the_mode_repair_writes_nothing(rigs):
    rig = rigs(dict(name='A', arm='metal_arm', via='slcan', modes={'J2': 3}))
    s = rig.roborun('setup')
    to_arms(s, rig)
    pick(s, rig)
    no_cameras(s)
    s.answer('Write it', 'n', 90)
    s.expect('Nothing was written', 30)
    s.expect('does not look like a metal_arm', 30)
    assert rig.cmd('modes')['A']['J2'] == [3, 3] and rig.enables()['A'] == 0


def test_a_lerobot_calibration_is_offered_and_used_without_writing_the_servos(rigs):
    rig = rigs(SO101)
    names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
    folder = rig.home / '.cache' / 'huggingface' / 'lerobot' / 'calibration' / 'robots' / 'so101_follower'
    folder.mkdir(parents=True)
    rows = {n: dict(id=i + 1, drive_mode=0, homing_offset=0, range_min=1000, range_max=3000) for i, n in enumerate(names)}
    rows['gripper'].update(range_min=2040, range_max=3400)            # the simulated gripper rests closed, at 2048
    (folder / 'my_arm.json').write_text(json.dumps(rows))
    s = rig.roborun('setup')
    to_arms(s, rig)
    pick(s, rig)
    no_cameras(s)
    s.answer('Use that calibration', 'y', 90)
    s.answer('Close the gripper fully', '', 30)
    s.expect('the servos were only read', 30)
    skip_movement_check(s)
    finish_without_starting(s)
    saved = json.loads((rig.config / 'calibration' / 'so101_serial_SIMA.json').read_text())
    assert saved['facts']['source'] == 'lerobot' and not rig.enabled() and rig.enables()['A'] == 0


def test_setup_over_a_running_robot_switches_the_motors_off_through_roborun(rigs):
    """Setup stops a running Roborun the way `roborun down` does: the station switches the motors off (killed, they would stay
    enabled until their CAN watchdog faulted them, and the next start would ask for a power cycle)."""
    rig = rigs(YAM)
    s = rig.roborun('up')
    to_arms(s, rig)
    s.answer('Which arm is this', '1'); s.answer('Use this arm', 'y')
    s.expect('OK: all 7 motors', 60)
    s.answer(SAFE, 'y', 60)
    s.expect('Done: the jaw travels', 90); movement_check(s, 6)
    s.expect('ready: open https://app.roborun.app', 240); assert s.wait(60) == 0
    assert rig.enabled()                                                    # the running robot holds the arm
    again = rig.roborun('setup')
    again.answer('Stop it for the setup', 'y', 60)
    again.answer('Move them slowly to their rest pose', 'n', 60)
    again.expect('Support them by hand', 30); assert again.wait(30) == 1
    assert rig.enabled()                                                    # raised: still held by the running station, never dropped
    rig.plain('down', '--release', timeout=60)                              # the person supports it and lets go
    assert not rig.enabled(), rig.enabled()                                 # switched off by the station, not left enabled
    again = rig.roborun('setup')
    again.expect('Which arm is this|robot.yaml already names', 90)    # nothing running: setup carries on
    assert 'Stop it for the setup' not in again.seen and not rig.enabled()
    again.ctrl_c(); again.wait(30)


# -- the short path: one arm, one camera ---------------------------------------------------------------------------------
def test_one_so101_and_one_camera_is_seven_questions(rigs):
    """Sign-in by link, SO-101 by default, the camera is the top one, the name is the computer's: what is left to ask is
    the SO-100 line, the two safety yeses, the two calibration steps, the one look at the movement, and whether to start."""
    rig = rigs(SO101, cameras=dict(camA=100))
    s = serial_arm_setup(rig, 5)
    finish_without_starting(s)
    assert s.sends == 7, s.sends
    assert 'left slot' not in s.seen and 'left arm' not in s.seen.lower() and 'LEFT' not in s.seen
    assert 'beta' not in s.seen and s.seen.count('not yet run on real hardware by us') == 1
    assert 'Paste your Roborun key' not in s.seen and 'A name for this robot' not in s.seen
    assert len(__import__('re').findall(SAFE, s.seen)) == 2                 # one before calibration, one before power
    y = rig.yaml()
    assert y['arms']['left'] == dict(arm='so101', address='serial://SIMA') and y['cameras'] == dict(top='usb:camA')
    assert y['robot'] == HOST and checks(rig) == {'so101_serial_SIMA': 'passed'} and not rig.enabled()
    assert (rig.config / 'token').read_text().strip() == TOKEN and rig.sign_ins()['started'] == 1


def test_one_yam_and_one_camera_is_six_questions(rigs):
    rig = rigs(YAM, cameras=dict(camA=100))
    s = rig.roborun('setup')
    to_arms(s, rig)
    s.answer('Which arm is this', '1'); s.answer('Use this arm', 'y')
    s.expect('it is the top camera', 60)
    s.answer(SAFE, 'y', 60); s.expect('Done: the jaw travels', 90)
    movement_check(s, 6)
    finish_without_starting(s)
    assert s.sends == 6, s.sends
    assert s.seen.count('real hardware by us') == 1 and 'LEFT' not in s.seen          # the YAM is beta: said once, plainly
    assert rig.yaml()['cameras'] == dict(top='usb:camA') and checks(rig) == {'yam_vcan0': 'passed'}


def test_so100_is_one_yes_away(rigs):
    rig = rigs(SO101)
    s = rig.roborun('setup')
    to_arms(s, rig)
    s.answer(r'SO-100 instead\? \[n\]', 'y', 60)
    s.expect('OK: all 6 motors', 60)
    s.ctrl_c(); assert s.wait(20) == 130
    assert json.loads((rig.config / 'setup-state.json').read_text())['arms']['left']['arm'] == 'so100'


def test_a_key_pasted_while_the_link_waits_is_used_and_never_shown(rigs):
    rig = rigs(SO101)
    s = rig.roborun('setup')
    s.expect('Waiting for your approval', 60)
    s.send('rr-not-a-key')
    s.expect('That key was not accepted', 30)
    s.expect('Waiting for your approval', 30)
    s.send(TOKEN)
    s.expect('Saved to', 30)
    s.expect(r'SO-100 instead', 60)
    s.ctrl_c(); assert s.wait(20) == 130
    assert TOKEN not in s.seen and 'rr-not-a-key' not in s.seen                        # not echoed
    assert (rig.config / 'token').read_text().strip() == TOKEN


def test_a_key_in_the_environment_needs_no_sign_in(rigs):
    rig = rigs(SO101)
    s = rig.roborun('setup', env=dict(ROBORUN_TOKEN=TOKEN))
    s.expect('Saved the key from the environment', 60)
    s.expect(r'SO-100 instead', 60)
    s.ctrl_c(); assert s.wait(20) == 130
    assert rig.sign_ins()['started'] == 0 and (rig.config / 'token').read_text().strip() == TOKEN


def test_up_again_never_kills_a_station_holding_a_raised_arm(rigs):
    """`roborun up` over a running robot lets the old station switch its motors off first, as down does: with the arm raised it
    refuses and keeps holding (killed, the arm would fall and its motors fault on their CAN watchdog)."""
    rig = rigs(YAM)
    s = rig.roborun('up')
    to_arms(s, rig)
    s.answer('Which arm is this', '1'); s.answer('Use this arm', 'y')
    s.answer(SAFE, 'y', 60); s.expect('Done: the jaw travels', 90)
    movement_check(s, 6)
    s.expect('ready: open https://app.roborun.app', 300); assert s.wait(60) == 0
    assert rig.enabled()
    again = rig.plain('up', timeout=120)
    assert again.returncode != 0 and 'keeps holding the raised arms' in again.stdout, again.stdout[-800:]
    assert rig.enabled()                                                    # still held by the running station, not dropped
    rig.plain('down', '--release', timeout=60)
    assert not rig.enabled()
