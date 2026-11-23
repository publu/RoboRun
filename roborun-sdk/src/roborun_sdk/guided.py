"""The guided setup: from a fresh install with arms plugged in to a working robot.yaml, one question at a time.

`roborun up` comes here when robot.yaml names no arms and a terminal is attached; `roborun setup` comes here any time.
Every answer is saved as soon as it is given (<roborun home>/setup-state.json, never the token), so Ctrl-C and running it
again continue where it stopped. Nothing moves or is powered without a yes for that step, and every step that powers a
motor ends with that motor switched off, however the step ended.
"""
from __future__ import annotations

import contextlib
import json
import os
import signal
import sys
import time
from pathlib import Path

from . import robot
from .arm_profiles import MODELS, info as model_info, parse as parse_profile
from .arms import catalog, identify, is_own_driver, open_arm, settings
from .arms import calibrate as stored
from .arms import motor_tools
from .arms import canbus, connect, direction_check
from .arms.safety import ArmsBusy, ArmsLock

APP = 'https://app.roborun.app'
STEPS = ('token', 'hardware', 'arms', 'cameras', 'check', 'calibrate', 'wrist', 'motion', 'write')
TITLES = dict(token='Signing in', hardware='Looking for your arms', arms='Which arms are these?',
              cameras='Cameras', check='Reading the arms (nothing moves)', calibrate='Calibration',
              wrist='Are the wrist cameras on the right arms?', motion='Movement check', write='Saving your settings')
ROLE_TEXT = dict(top='the camera looking down at the table', left="the camera on the left arm's wrist",
                 right="the camera on the right arm's wrist")
STATUS_TEXT = {'supported': '',                       # run on real hardware: nothing to warn about
               'beta': 'not yet run on real hardware by us (tested against a simulated bus or the maker\'s simulator)',
               'bring-your-own': 'bring your own: Roborun has no driver for it'}
SAFE = 'Is the arm supported and the area around it clear?'  # the one yes before calibration, and the one before power
PICTURES = 'https://roborun.app/docs/calibration'           # <arm>.png: how the arm should look while calibrating
SO_FAMILY = ('so100', 'so101', 'xlerobot')                  # the same motors: SO-101 unless the person says SO-100
DARK_SHARE = 0.55                # a covered lens gives at most this share of its uncovered brightness
GRACE_S = 5.0


class Again(Exception):
    def __init__(self, step):
        super().__init__(step)
        self.step = step


def line(text=''):
    print(text, flush=True)


def state_file():
    return Path(robot.HOME) / 'setup-state.json'


def preview_dir():
    return Path(robot.HOME) / 'setup-previews'


def _interrupted(*_):
    raise KeyboardInterrupt


class _NoInterrupt:
    """Ctrl-C waits while motors are being switched off; it only cuts the grace period short."""

    def __enter__(self):
        self.hit = False
        self.old = signal.signal(signal.SIGINT, self._note)
        return self

    def _note(self, *_):
        self.hit = True

    def wait(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not self.hit:
            time.sleep(0.05)

    def __exit__(self, *_):
        signal.signal(signal.SIGINT, self.old)


# -- what to say when something is missing ---------------------------------------------------------------------------------
def nothing_found_help():
    out = ['Nothing that looks like an arm adapter is connected. Please check:',
           '  - the arm has power (its supply is on, any emergency stop is released)',
           '  - the USB cable goes from the adapter to this computer; try another port or cable, and avoid unpowered hubs',
           '  - the CAN or serial wires from the adapter reach the arm']
    if sys.platform == 'darwin':
        out += ['On this Mac:',
                '  - Apple menu > About This Mac > More Info > System Report > USB should list the adapter',
                '  - serial adapters appear as /dev/cu.usbmodem* or /dev/cu.usbserial*; no driver is needed for CANable or candleLight',
                '  - some USB-serial chips (CH340, for example) need the maker\'s driver on older macOS']
    else:
        rule = canbus.usb_permission_fix()
        dialout = ('you are in the dialout group' if canbus.in_dialout()
                   else 'sudo usermod -aG dialout "$USER", then log out and in again')
        out += ['On this Linux computer:',
                '  - lsusb should list the adapter, and sudo dmesg | tail shows what happened when you plugged it in',
                f'  - permission for serial adapters: {dialout}',
                f'  - permission for USB CAN adapters (the udev rule): {rule}',
                '  - a CAN adapter already set up as a network link shows in: ip -details link show type can']
    return out


def no_terminal_text(file):
    token = settings.token_path(argparse_free())
    have_token = token.is_file() or settings.given_token()
    lines = ['No arms are set up and there is no terminal here to ask in, so nothing was started.', '',
             'In a terminal, run `roborun setup` for the guided setup. Or do it by hand:', '']
    n = 1
    if not have_token:
        lines += [f'  {n}. Sign in: roborun login (or put your key in {token}, one line, chmod 600; or set ROBORUN_API_KEY or ROBORUN_TOKEN)']
        n += 1
    lines += [f'  {n}. See what is connected, and what it could be (asks nothing, moves nothing):  roborun hw detect',
              f'  {n + 1}. Name your arms and cameras in {file}:',
              '         arms:', '           left:', '             arm: ARM            # from: roborun hw arms',
              '             address: ADDRESS    # from: roborun hw detect',
              '         cameras:', '           top: SOURCE           # from: roborun hw detect',
              f'  {n + 2}. Read the arm, with nothing enabled:   roborun hw check ARM@ADDRESS',
              f'  {n + 3}. If that asks for calibration:         roborun hw calibrate ARM@ADDRESS',
              f'  {n + 4}. Before the first motion, in a terminal: roborun hw check ARM@ADDRESS --move',
              f'  {n + 5}. Start:                                roborun up', '',
              'To try Roborun without arms, ask for the simulator: roborun up --sim cup   (or `sim: cup` in robot.yaml)']
    return '\n'.join(lines)


def argparse_free():
    return type('Args', (), dict(key_file=None))()


def enter(a):
    """From `roborun up`: the guided setup on a terminal. Without one: instructions, and nothing is started."""
    if not sys.stdin.isatty():
        raise SystemExit(_token_verdict(a) + no_terminal_text(settings.path()))
    Wizard(a).run()


def _token_verdict(a):
    """Without a terminal the token can still be right or wrong: a good one is kept, a wrong one is said and never saved."""
    file, given = settings.token_path(a), settings.given_token()
    token = given or (file.read_text().strip() if file.is_file() else '')
    if not token:
        return ''
    verdict = robot.check_token((getattr(a, 'server', None) or robot.SERVER).rstrip('/'), token)
    if verdict is False:
        return 'That access token was not accepted: check it and try again.\n\n'
    if given:
        settings.store_token(file, given)
    return ''


def main(a) -> int:
    if getattr(a, 'detect', False):
        return detect(getattr(a, 'json', False))
    if not sys.stdin.isatty():
        raise SystemExit(no_terminal_text(settings.ensure_file()))
    if getattr(a, 'restart', False):
        state_file().unlink(missing_ok=True)
    settings.ensure_file()
    wizard = Wizard(a, from_setup=True)
    if wizard.run() == 'start':
        from .cli import main as cli_main
        code = cli_main(['up'])
        if code == 0:
            wizard.overhead_step()
        for text in wizard.after:
            line(text)
        return code
    return 0


# -- what is connected ------------------------------------------------------------------------------------------------
def adapters():
    rows, notes = connect.adapter_rows()
    return [r for r in rows if r.get('address')], notes


def _kept_camera(entry):
    """robot.yaml's camera entry when setup does not manage it (your own class, a stream address): it stays untouched; None for
    a camera setup finds itself (realsense:, usb:) and did not assign this time: that role is removed."""
    from . import cameras
    source = entry.get('source') if isinstance(entry, dict) else entry
    try:
        kind = cameras.parse_source(source)[0]
    except (ValueError, TypeError):
        return entry                      # not something setup understands: left alone, `roborun up` names any problem
    return None if kind in ('realsense', 'usb') else entry


def camera_rows():
    """([(source, description)], [notes]). Cameras are optional: anything that goes wrong here is only a note."""
    from .arms.packages import ensure
    notes = []
    try:
        ensure('cameras', 'cameras')
    except (Exception, SystemExit):                        # plain USB cameras still work; RealSense support can follow
        notes.append('RealSense support could not be installed (is this computer online?). Other cameras are still listed.')
    try:
        from . import cameras
        if sys.platform == 'darwin':
            try:
                if cameras.mac_camera_access(ask=True) != 'authorized':
                    notes.append(cameras.MAC_CAMERA_DENIED)
            except cameras.CameraError as exc:
                notes.append(str(exc))
        realsense, more = cameras.list_realsense(robot.camera_socket())
        usb, others = cameras.list_usb()
        return realsense + usb, notes + more + others
    except (Exception, SystemExit) as exc:
        return [], [f'cameras could not be listed: {exc}']


def detect(as_json=False) -> int:
    rows, notes = adapters()
    found = []
    for row in rows:
        probe = identify.probe(row)
        found.append(dict(address=row['address'], bus=row['bus'], device=row['detail'], note=row.get('note'), read=probe.summary(),
                          family=probe.family, motors=probe.motors,
                          could_be=[dict(arm=m.arm, fit=m.level, why=m.why) for m in identify.rank(probe)]))
    cams, cam_notes = camera_rows()
    if as_json:
        line(json.dumps(dict(adapters=found, cameras=[dict(source=s, description=d) for s, d in cams], notes=notes + cam_notes), indent=1))
        return 0
    if not found:
        line('No CAN or serial adapters found.')
    for row in found:
        line(f"{row['address']}  ({row['bus']}, {row['device']})")
        line(f"  {row['read']}")
        for m in row['could_be']:
            line(f"  could be {m['arm']}: {m['why']}")
    for source, text in cams:
        line(f'camera {source}  ({text})')
    if not cams:
        line('No cameras found.')
    for note in notes + cam_notes:
        line(f'note: {note}')
    return 0


def describe_arm(row):
    """The honest lines about an arm, shown when it is chosen."""
    out = [f"{row['label']} ({row['id']})"]
    status = STATUS_TEXT.get(row.get('status', 'beta'), row.get('status'))
    if status:
        out.append(f'  {status}')
    if not can_grip(row):
        out.append('  it cannot grip yet: Roborun does not drive a gripper on this arm, so it moves but will not pick things up')
    if row.get('profile') is None:
        out.append('  it has no built-in model: Manifest cannot plan for it until you add one (roborun profiles create --urdf ...)')
    if row.get('fits_profile_shape') is False:
        out.append('  its shape does not fit a Manifest arm profile yet')
    if row.get('platforms'):
        out.append(f"  runs on: {row['platforms']}")
    if row.get('note'):
        out.append(f"  note: {row['note']}")
    if row.get('unverified'):
        out.append(f"  {len(row['unverified'])} things about this arm are still unverified (roborun hw arms --json shows them)")
    return out


def can_grip(row):
    module, _, name = str(row.get('driver') or '').partition(':')
    if not module:
        return False
    try:
        import importlib
        return getattr(getattr(importlib.import_module(module), name), 'has_gripper', True) is not False
    except Exception:
        return 'no gripper is driven' not in str(row.get('note', '')).lower()


def tool_model(arm, side):
    """The arm's model, so the movement check can say where the tool should go."""
    try:
        spec = model_info(arm)['sides']
        xml = MODELS / arm / (spec.get(side) or spec['left'])['model']
        return dict(tool_model=dict(xml=str(xml), site='grasp_site')) if xml.is_file() else {}
    except (OSError, KeyError, ValueError):
        return {}


class _KeyTaken(Exception):
    pass


class _PasteWatch:
    """While the sign-in link waits for approval: a key pasted on the terminal (not echoed) is tried instead. Ctrl-C still
    stops; the terminal is put back however the wait ends."""

    def __init__(self, take):
        self.take, self.saved, self.fd, self.said, self.open_line = take, None, None, False, False

    def __enter__(self):
        try:
            import termios
            self.fd = sys.stdin.fileno()
            self.saved = termios.tcgetattr(self.fd)
            quiet = termios.tcgetattr(self.fd)
            quiet[3] &= ~termios.ECHO
            termios.tcsetattr(self.fd, termios.TCSADRAIN, quiet)
        except (ImportError, OSError, ValueError, AttributeError):
            self.saved = None
        return self

    def sleep(self, seconds):
        import select
        if not self.said:                                   # after the link and its code are on the screen
            sys.stdout.write('Waiting for your approval. Or paste a Roborun key here and press Enter (it will not show). Key: ')
            sys.stdout.flush()
            self.said = self.open_line = True
        end = time.monotonic() + seconds
        while (left := end - time.monotonic()) > 0:
            try:
                ready = select.select([sys.stdin], [], [], left)[0]
            except (OSError, ValueError):
                time.sleep(left)
                return
            if not ready:
                return
            text = sys.stdin.readline()
            if not text:
                raise EOFError
            self.say()                                      # the Enter was not echoed
            if text.strip() and self.take(text.strip()):
                raise _KeyTaken
            self.said = False                               # not taken: ask again at the next wait
            return

    def say(self, text=''):
        """A line of its own, also after the waiting prompt."""
        if self.open_line:
            line()
            self.open_line = False
        if text:
            line(text)

    def __exit__(self, *_):
        if self.saved is not None:
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)


class Wizard:
    def __init__(self, a, from_setup=False):
        self.a, self.from_setup = a, from_setup
        self.after, self.redo_motion = [], False
        self.rows, self.notes = [], []
        self.probes = {}
        self.camera_list, self.camera_notes = [], []
        self.count = None                     # arms being chosen right now; afterwards, the arms in the saved answers
        self.st = self.load()

    # -- saved answers -------------------------------------------------------------------------------------------
    def load(self):
        try:
            data = json.loads(state_file().read_text())
            if data.get('version') == 1:
                return data
        except (OSError, ValueError):
            pass
        return dict(version=1, done=[], arms={}, cameras={}, plan={}, blocked=[])

    def save(self):
        target = state_file()
        target.parent.mkdir(parents=True, exist_ok=True)
        temp = target.with_suffix('.tmp')
        temp.write_text(json.dumps(self.st, indent=1))
        temp.replace(target)

    def mark(self, step):
        if step not in self.st['done']:
            self.st['done'].append(step)
        self.save()

    def the(self, side, first=False):
        """'the arm' with one arm (no left or right to think about), 'the left arm' with two."""
        one = (self.count or len(self.st['arms']) or 1) == 1
        text = 'the arm' if one else f'the {side} arm'
        return text[0].upper() + text[1:] if first else text

    def reopen_from(self, step):
        keep = set(STEPS[:STEPS.index(step)])
        if step in ('hardware', 'arms') and 'cameras' in self.st['done']:      # choosing the arm again does not undo the cameras
            keep.add('cameras')
        self.st['done'] = [s for s in self.st['done'] if s in keep]
        self.save()

    # -- talking to the person -----------------------------------------------------------------------------------
    def ask(self, text, default='', hint=''):
        answer = input(f'{text} ' + (f'[{default}] ' if default else '') + (f'({hint}) ' if hint else '')).strip()
        return answer or default

    def raw(self, text):
        return input(text)

    def yes(self, text, default=True):
        answer = self.ask(f"{text} ({'Y/n' if default else 'y/N'})").lower()
        return default if not answer else answer in ('y', 'yes')

    def enter(self, text):
        input(f'{text} Press Enter when ready. ')

    def choose(self, text, options, default=None, hint=''):
        """options: [(key, label)]. Returns the key. hint: said after the default, e.g. what Enter does."""
        for key, label in options:
            line(f'  {key}. {label}')
        keys = {str(k).lower(): k for k, _ in options}
        while True:
            answer = self.ask(text, str(default) if default is not None else '', hint).lower()
            if answer in keys:
                return keys[answer]
            line(f"Please answer with one of: {', '.join(str(k) for k, _ in options)}")

    # -- the run --------------------------------------------------------------------------------------------------
    def run(self):
        before = {number: signal.signal(number, _interrupted) for number in (signal.SIGTERM, signal.SIGHUP)}
        try:
            return self._steps()
        except (KeyboardInterrupt, EOFError):
            line()
            line('Setup paused: nothing is moving and no motor was left on by this setup.')
            line('Your answers are saved. To continue: roborun setup')
            raise SystemExit(130) from None
        finally:
            for number, handler in before.items():
                signal.signal(number, handler)

    def _steps(self):
        line('Roborun setup. Ctrl-C stops at any point; run `roborun setup` to carry on from there.')
        self.free_the_arms()
        self.pick_up_existing()
        index = 0
        while index < len(STEPS):
            step = STEPS[index]
            if step in self.st['done']:
                index += 1
                continue
            shown = [s for s in STEPS if s != 'wrist']
            if step == 'wrist' and not self.wrist_applies():
                self.mark(step)
                index += 1
                continue
            line()
            line(f"--- {TITLES[step]}" + (f' (step {shown.index(step) + 1} of {len(shown)})' if step in shown else ''))
            try:
                getattr(self, f'step_{step}')()
            except Again as again:
                self.reopen_from(again.step)
                index = STEPS.index(again.step)
                continue
            self.mark(step)
            index += 1
        return self.finish()

    def free_the_arms(self):
        running = [u for u in ('roborun-station', 'roborun-cameras', 'roborun-sim') if robot._state(u) in ('active', 'running')]
        if not running:
            return
        config = json.loads(robot.CONFIG.read_text()) if robot.CONFIG.exists() else {}
        if robot._busy(config.get('backend') or 'http://127.0.0.1:8120'):
            raise SystemExit('A task is running on this robot. Stop it first, then run roborun setup again.')
        if not self.yes('Roborun is running and holds the arms and cameras. Stop it for the setup?'):
            raise SystemExit('Nothing was changed. Run roborun down, then roborun setup.')
        # through the station, as `roborun down` does: it switches the motors off itself (killed, they would sit enabled until
        # their CAN watchdog faults them, and the next start asks for a power cycle) and never lets a raised arm fall
        held = None
        if 'roborun-station' in running:
            from .arms.hardware import ask_station, faulted

            def stop_on_fault(held):
                bad = faulted(held)
                if bad:
                    raise SystemExit('Support the ' + ' and '.join(bad) + ' arm now: ' + '; '.join(f'{s} arm: {held[s]}' for s in bad)
                                     + '. Then check its power and cable (roborun hw check) and run roborun setup again.')

            held = ask_station('down')
            if held is None:                                   # no answer: whether an arm is still powered is not known
                line('The robot station did not answer. If an arm is raised, support it now: its motors may switch off.')
            stop_on_fault(held)
            if held and self.yes('These arms are raised and still holding: ' + '; '.join(f'{s} arm: {w}' for s, w in held.items())
                                 + '. Move them slowly to their rest pose and switch them off?'):
                held = ask_station('park')
                if held is None:
                    raise SystemExit('The robot station did not answer while parking. If an arm is raised, support it now: its '
                                     'motors may switch off. Then run roborun status, and roborun setup again.')
                stop_on_fault(held)
            if held:
                raise SystemExit('Roborun keeps holding the raised arms, so nothing falls (' + '; '.join(f'{s} arm: {w}' for s, w in held.items())
                                 + '). Support them by hand and run '
                                 'roborun down --release, or run roborun down --park; then roborun setup.')
        for unit in running:
            robot._stop(unit)

    def pick_up_existing(self):
        file_settings = settings.load(warn=lambda _: None)
        arms = file_settings.get('arms') or {}
        named = {side: dict(arm=str(row['arm']), address=str(row.get('address') or ''))
                 for side, row in arms.items() if isinstance(row, dict) and row.get('arm')}
        if named and ('write' in self.st['done'] or not self.st['arms']):
            self.st['arms'] = named
            self.st['cameras'] = {role: (row.get('source') if isinstance(row, dict) else row)
                                  for role, row in (file_settings.get('cameras') or {}).items()}
            self.st.pop('mode', None)
            try:
                self.st['profile'] = self.profile_for(named)
            except _NoProfile:
                self.st['profile'] = None
            line('robot.yaml already names these:')
            for side, row in named.items():
                line(f"  {side} arm: {row['arm']} at {row['address']}")
            for role, source in self.st['cameras'].items():
                line(f'  {role} camera: {source}')
            what = self.choose('What would you like to do?', [
                (1, 'keep this and start Roborun'), (2, 'change the arms (then repeat what follows)'), (3, 'change the cameras'),
                (4, 'repeat the read-only check and calibration'), (5, 'run the movement check again'), (6, 'start over')], 1)
            self.st['done'] = {1: list(STEPS), 2: ['token', 'hardware'], 3: ['token', 'hardware', 'arms', 'check', 'calibrate', 'motion'],
                               4: ['token', 'hardware', 'arms', 'cameras'],
                               5: ['token', 'hardware', 'arms', 'cameras', 'check', 'calibrate', 'wrist'], 6: []}[what]
            if what == 1:
                self.st['done'].remove('write')
            elif what == 5:
                self.st['plan'] = {side: True for side in named}
                self.redo_motion = True
            if what == 6:
                self.st.update(arms={}, cameras={}, plan={}, blocked=[])
            self.save()
        elif 'write' in self.st['done']:                                       # finished once, but robot.yaml no longer says so
            self.st.update(done=[s for s in self.st['done'] if s == 'token'], arms={}, cameras={}, plan={}, blocked=[])
            self.st.pop('mode', None)
            self.save()
        elif self.st['done']:
            line('Picking up where you left off: ' + ', '.join(TITLES[s].lower() for s in self.st['done'] if s in TITLES) + ' already done.')

    # -- 1. token -------------------------------------------------------------------------------------------------
    def step_token(self):
        file = settings.token_path(self.a)
        server = (getattr(self.a, 'server', None) or robot.SERVER).rstrip('/')
        if file.is_file() and file.read_text().strip():
            verdict = robot.check_token(server, file.read_text().strip())
            if verdict is not False:
                line(f'Using the access token in {file}' + ('' if verdict else ' (the service could not be reached, so it was not checked)'))
                return
            line(f'The token in {file} was not accepted.')
        given = settings.given_token()                      # ROBORUN_API_KEY / ROBORUN_TOKEN: headless and CI
        if given:
            verdict = robot.check_token(server, given)
            if verdict is not False:
                settings.store_token(file, given)
                line(f'Saved the key from the environment to {file}' + ('' if verdict else ' (the service could not be reached, so it will be checked when Roborun starts)'))
                return
            line('The key in ROBORUN_API_KEY / ROBORUN_TOKEN was not accepted.')
        if self.sign_in(server, file):
            return
        self.paste_key(server, file)

    def sign_in(self, server, file):
        """`roborun login`'s approve link, right here: open it anywhere, approve, and setup carries on. A key pasted while
        it waits is used instead. False when the sign-in cannot run (no API, expired): then a key is asked for."""
        from .login import login
        watch = _PasteWatch(lambda key: self.try_key(server, file, key))
        try:
            with watch:
                login(server, file, out=watch.say, sleep=watch.sleep)
                return True
        except _KeyTaken:
            return True
        except SystemExit as exc:
            line(f'Sign-in by link did not work: {exc.code}')
            return False

    def try_key(self, server, file, token):
        verdict = robot.check_token(server, token)
        if verdict is False:
            line('That key was not accepted. Paste it again, or approve the link above.')
            return False
        settings.store_token(file, token)
        line(f'Saved to {file}' + ('' if verdict else ' (the service could not be reached, so it will be checked when Roborun starts)'))
        return True

    def paste_key(self, server, file):
        import getpass
        for _ in range(5):
            token = getpass.getpass('Paste your Roborun key (it starts with rr_live_; it will not show as you type). Roborun is invite-only: '
                                    "if you don't have a key, email support@roborun.dev. Key: ").strip()
            if token and self.try_key(server, file, token):
                return
        raise SystemExit('No valid key, so setup stopped. Roborun is invite-only: to get a key, email support@roborun.dev, '
                         'then run roborun setup again.')

    # -- 2. what is connected --------------------------------------------------------------------------------------
    def scan(self):
        self.rows, self.notes = adapters()
        self.camera_list, self.camera_notes = camera_rows()
        self.probes = {}

    def show_scan(self):
        for row in self.rows:
            line(f"  adapter {row['address']}  ({row['bus']}, {row['detail'] or row['kind']})")
            if row.get('note'):
                line(f"    note: {row['note']}")
        for source, text in self.camera_list:
            line(f'  camera  {source}  ({text})')
        for note in self.notes + self.camera_notes:
            line(f'  note: {note}')

    def step_hardware(self):
        looked = 0
        while True:
            self.scan()
            self.show_scan()
            if self.rows:
                return
            looked += 1
            if looked == 1:                                 # the checklist once; after that Enter moves on, never loops
                for text in nothing_found_help():
                    line(text)
            else:
                line('Still no arm adapter found.')
            what = self.choose('What now?', [('r', 'look again (after checking the above)'),
                                             ('n', 'my arm is reached over the network or through its maker\'s software'),
                                             ('s', 'use simulated arms instead (two YAMs; no real arm will move)'),
                                             ('q', 'stop here')], 'r' if looked == 1 else 's',
                                 '' if looked == 1 else 'Enter: simulated arms now; plug your arm in and run roborun setup any time')
            if what == 'q':
                raise KeyboardInterrupt
            if what == 'n':
                return
            if what == 's':
                self.st['mode'] = 'sim'
                self.st['done'] = [s for s in STEPS if s != 'write']
                self.save()
                raise Again('write')

    def ensure_scanned(self):
        if not self.rows:
            self.scan()

    # -- 3. which arms -----------------------------------------------------------------------------------------------
    def probe(self, row):
        if row['address'] not in self.probes:
            self.probes[row['address']] = identify.probe(row)
        return self.probes[row['address']]

    def step_arms(self):
        self.ensure_scanned()
        for row in self.rows:
            line(f"  {row['address']}: {self.probe(row).summary()}")
            for note in self.probe(row).notes:
                line(f'    {note}')
        count = 1
        if len(self.rows) != 1:
            count = self.choose('How many arms are you setting up?', [(1, 'one arm'), (2, 'two arms')], min(len(self.rows), 2) or 1)
        else:
            line('One adapter is connected, so this sets up one arm. For a second arm, plug in its adapter and run roborun setup again.')
        chosen = {}
        sides = ['left', 'right'][:count]
        taken = set()
        self.count = count
        try:
            for side in sides:
                row = self.pick_adapter(side, taken)
                if row is not None:
                    taken.add(row['address'])
                chosen[side] = self.pick_arm(side, row)
        finally:
            self.count = None
        self.check_pair(chosen)
        self.st['arms'] = chosen
        self.st['plan'] = {}
        self.st['cameras'] = {role: source for role, source in self.st['cameras'].items() if role == 'top' or role in chosen}
        self.save()

    def pick_adapter(self, side, taken):
        free = [r for r in self.rows if r['address'] not in taken]
        if not free:
            line(f'No adapter is left for the {side} arm, so it will need an address typed by hand.')
            return None
        if len(free) == 1:
            if len(self.rows) > 1:
                line(f"The {side} arm is on the remaining adapter, {free[0]['address']}.")
            return free[0]
        options = [(i + 1, f"{r['address']}: {self.probe(r).summary()}") for i, r in enumerate(free)]
        options += [('u', "I can't tell: find it by unplugging it"), ('n', 'this arm has no adapter here (network or maker software)')]
        what = self.choose('Which adapter is the arm on?' if self.count == 1 else f'Which adapter goes to the {side.upper()} arm?', options)
        if what == 'n':
            return None
        return self.find_by_unplug(side, free) if what == 'u' else free[what - 1]

    def find_by_unplug(self, side, free):
        while True:
            self.enter(f"Unplug the USB plug of {'the arm' if self.count == 1 else 'the ' + side.upper() + ' arm'}'s adapter from this computer, leave the others in.")
            now = {r['address'] for r in adapters()[0]}
            gone = [r for r in free if r['address'] not in now]
            if len(gone) == 1:
                line(f"That is {gone[0]['address']}.")
                while gone[0]['address'] not in {r['address'] for r in adapters()[0]}:
                    self.enter('Plug it back in.')
                self.probes.pop(gone[0]['address'], None)
                return gone[0]
            line('No adapter disappeared.' if not gone else f'{len(gone)} adapters disappeared. Please unplug exactly one.')

    def pick_arm(self, side, row):
        rows = catalog()
        found = self.probe(row) if row else None
        matches = identify.rank(found) if found else []
        best = [m for m in matches if m.level in ('exact', 'extra')]
        close = [m for m in matches if m.level == 'partial']
        others = [m for m in matches if m.level == 'family']
        line()
        if row is not None:
            line(f"{self.the(side, True)}, on {row['address']}: {found.summary()}")
        if set(SO_FAMILY) <= {m.arm for m in best} and not close:
            # SO-100, SO-101 and an XLeRobot arm have the same motors: the SO-101 is by far the most common
            line(f"This looks like: {rows['so101']['label']} ({next(m.why for m in best if m.arm == 'so101')}).")
            for text in describe_arm(rows['so101'])[1:]:
                line('  ' + text)
            arm = 'so100' if self.ask('SO-100 instead?', 'n').lower() in ('y', 'yes') else 'so101'
            return self.finish_choice(side, arm, row)
        if best and len(best) == 1 and not close:
            line(f"This looks like: {rows[best[0].arm]['label']} ({best[0].why}).")
            for text in describe_arm(rows[best[0].arm]):
                line('  ' + text)
            if self.yes('Is that your arm?'):
                return self.finish_choice(side, best[0].arm, row)
        options, by_key = [], {}
        def add(group, items):
            for m in items:
                key = len(options) + 1
                options.append((key, f"{rows[m.arm]['label']} ({m.arm})  {group}: {m.why}"))
                by_key[key] = m.arm
        add('matches', best); add('close', close)
        if not best and not close:
            add('same motor family only', others)
        if not options:
            line('The motors did not identify this arm, so please choose it yourself.')
        options += [('l', 'show every arm Roborun knows (roborun hw arms)'), ('o', 'I wrote my own driver for it')]
        while True:
            what = self.choose('Which arm is this?' if self.count == 1 else f'Which arm is on the {side}?', options)
            arm = self.from_list(row) if what == 'l' else self.own_driver() if what == 'o' else by_key[what]
            if arm is None:
                continue
            if is_own_driver(arm):
                return self.finish_choice(side, arm, row)
            for text in describe_arm(rows[arm]):
                line(text)
            if not rows[arm].get('driver'):
                line('Roborun cannot drive this arm by itself. Write an ArmAdapter for it (see the README: "Your own arm") and choose "I wrote my own driver".')
                continue
            if self.yes('Use this arm?'):
                return self.finish_choice(side, arm, row)

    def from_list(self, row):
        rows, keys = catalog(), []
        for i, (arm, data) in enumerate(rows.items(), 1):
            keys.append(arm)
            transport = data.get('transport')
            fit = '' if row is None or transport == ('serial' if row['bus'] == 'serial' else 'can') else f'  (connects by {transport}, not this adapter)'
            status = {'supported': '', 'beta': 'not yet run on real hardware'}.get(data.get('status', 'beta'), data.get('status'))
            line(f"  {i:>2}. {arm:<18} {data['label']:<32} {status}{fit}".rstrip())
        while True:
            answer = self.ask('Number or name of your arm (Enter to go back):')
            if not answer:
                return None
            if answer in rows:
                return answer
            if answer.isdigit() and 1 <= int(answer) <= len(keys):
                return keys[int(answer) - 1]
            line('That is not in the list.')

    def own_driver(self):
        ref = self.ask('Your driver, as my_arm.py:MyArm (Enter to go back):')
        if ref and not is_own_driver(ref):
            line('Write it as FILE.py:ClassName or package.module:ClassName.')
            return None
        return ref or None

    def finish_choice(self, side, arm, row):
        address = row['address'] if row else ''
        if not address:
            address = self.ask(f'The address of {self.the(side)} (what its driver connects to, e.g. an IP address):')
        if not is_own_driver(arm) and row is not None:
            wanted = catalog()[arm].get('transport')
            have = 'serial' if row['bus'] == 'serial' else 'can'
            if wanted in ('can', 'serial') and wanted != have:
                line(f'Note: {arm} connects by {wanted}, and this adapter is {have}. The read-only check will tell.')
        return dict(arm=arm, address=address)

    def profile_for(self, chosen):
        hardware = {side: dict(arm=row['arm'], bus=row['address']) for side, row in chosen.items()}
        existing = settings.load(warn=lambda _: None).get('profile')
        try:
            name = settings._profile_for(hardware)
        except SystemExit as exc:
            if existing:
                return existing
            raise _NoProfile(str(exc)) from None
        if parse_profile(name) is None and name not in robot.bundled():
            raise _NoProfile('these two arms have no combined profile yet (their planning models cannot be paired)')
        return name

    def check_pair(self, chosen):
        try:
            self.st['profile'] = self.profile_for(chosen)
        except _NoProfile as exc:
            line(f'{exc}.')
            line('Choose arms with a built-in model, or make a profile first: roborun profiles create --urdf ...')
            if not self.yes('Choose the arms again?'):
                raise KeyboardInterrupt
            raise Again('arms') from None

    # -- 4. cameras ---------------------------------------------------------------------------------------------------
    def grab(self, source):
        from . import cameras
        made = cameras.build_sources(dict(cameras=dict(top=source)), robot.camera_socket())['top']
        made.open()
        try:
            for _ in range(30):
                frame = made.read()
                if frame is not None:
                    return frame['rgb']
                time.sleep(.05)
            raise RuntimeError('it delivered no image')
        finally:
            made.close()

    def save_preview(self, index, rgb):
        import cv2
        import numpy as np
        preview_dir().mkdir(parents=True, exist_ok=True)
        path = preview_dir() / f'camera-{index}.jpg'
        cv2.imwrite(str(path), cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2BGR))
        return path

    def step_cameras(self):
        self.ensure_scanned()
        if not self.camera_list:                            # setup never stops for cameras: they are added later
            line('No cameras plugged in, so none are set up. To add one later: plug it in and run roborun setup.')
            self.st['cameras'] = {}
            return
        if len(self.camera_list) == 1:                      # one camera: it is the top camera, nothing to ask
            source, text = self.camera_list[0]
            self.st['cameras'] = dict(top=source)
            line(f'One camera, {source} ({text}): it is the top camera, {ROLE_TEXT["top"]}.')
            line('To change that or add cameras later: plug them in and run roborun setup.')
            self.save()
            return
        previews = {}
        for i, (source, text) in enumerate(self.camera_list, 1):
            try:
                previews[source] = self.save_preview(i, self.grab(source))
                line(f'  {i}. {source}  ({text})  preview: {previews[source]}')
            except Exception as exc:
                line(f'  {i}. {source}  ({text})  could not take a picture: {exc}')
        line('The pictures stay on this computer. Open them to see which camera is which, or use the hand trick below.')
        roles = ['top'] + [side for side in ('left', 'right') if side in self.st['arms']]
        assigned, left = {}, [s for s, _ in self.camera_list]
        for role in roles:
            if not left:
                break
            assigned[role] = self.assign_role(role, left)
            if assigned[role]:
                left.remove(assigned[role])
            else:
                del assigned[role]
        self.st['cameras'] = assigned
        self.save()

    def role_words(self, role):
        """(name, what it is): with one arm its wrist camera is just the wrist camera, no left or right."""
        if role != 'top' and len(self.st['arms']) == 1:
            return 'wrist', "the camera on the arm's wrist"
        return role, ROLE_TEXT[role]

    def assign_role(self, role, left):
        names = {s: t for s, t in self.camera_list}
        for i, source in enumerate(left, 1):
            line(f'  {i}. {source}  ({names[source]})')
        name, what = self.role_words(role)
        while True:
            answer = self.ask(f'{name.upper()} camera, {what}: its number, h to find it by covering the lens with your hand, or n for none:', 'h').lower()
            if answer == 'n':
                return None
            if answer.isdigit() and 1 <= int(answer) <= len(left):
                return left[int(answer) - 1]
            if answer == 'h':
                found = self.find_by_covering(role, left)
                if found and self.yes(f'That looks like {found} ({names[found]}). Is it the {name} camera?'):
                    return found

    def find_by_covering(self, role, left):
        self.enter('Keep the cameras uncovered.')
        before = {s: self.bright_of(s) for s in left}
        self.enter(f'Now cover the lens of the {self.role_words(role)[0].upper()} camera with your hand and keep it there.')
        after = {s: self.bright_of(s) for s in left}
        self.enter('Thanks, you can uncover it.')
        ratio = {s: after[s] / before[s] for s in left if before[s] and after[s] is not None and before[s] > 8}
        dark = [s for s, value in ratio.items() if value < DARK_SHARE]
        if len(dark) == 1:
            return dark[0]
        line('No single camera went dark. Choose it by number from the pictures instead.' if not dark
             else 'More than one camera went dark. Choose it by number from the pictures instead.')
        return None

    def bright_of(self, source):
        try:
            return float(self.grab(source).mean())
        except Exception:
            return None

    # -- 5. read-only check and calibration ----------------------------------------------------------------------------
    def spec(self, side):
        row = self.st['arms'][side]
        return f"{row['arm']}@{row['address']}"

    def build(self, side, dry=False):
        row = self.st['arms'][side]
        options = {k: v for k, v in ((settings.load(warn=lambda _: None).get('arms') or {}).get(side) or {}).items()
                   if k not in ('arm', 'address')}
        if not is_own_driver(row['arm']):
            options = dict(tool_model(row['arm'], side), **options)
        return open_arm(row['arm'], side, row['address'] or None, connect=False, **options)

    def locked(self, side):
        return ArmsLock('roborun setup', [self.st['arms'][side]['address']])

    def on_hardware_trouble(self, side, exc):
        """A step failed on the hardware: try again, go back, or stop. Never guess."""
        text = connect.explain(exc, self.spec(side)) if not isinstance(exc, SystemExit) else str(exc.code)
        line(f'Problem with {self.the(side)}: {text}')
        what = self.choose('What now?', [('r', 'try again (after fixing it)'), ('a', 'choose the arm or adapter again'), ('q', 'stop here')], 'r')
        if what == 'q':
            raise KeyboardInterrupt
        if what == 'a':
            self.rows = []
            raise Again('hardware')

    def step_check(self):
        for side in self.st['arms']:
            while not self.check_one(side):
                pass

    def check_one(self, side):
        line(f'Reading {self.the(side)} ({self.spec(side)}). Nothing is enabled or moved.')
        try:
            with self.locked(side), self.vendor_chatter():
                arm = self.build(side)
                report = arm.survey()
            if report.get('repairs'):                       # motors in another control mode: shown, asked, then written
                with self.locked(side):
                    if motor_tools.offer_repairs(arm, self.yes, say=line):
                        return False                        # read it again
        except NotImplementedError:
            line('  This arm has no read-only check. Its own software will refuse a wrong connection when Roborun starts.')
            return True
        except (Exception, SystemExit) as exc:
            if isinstance(exc, ArmsBusy):
                exc = SystemExit(str(exc))
            self.on_hardware_trouble(side, exc)
            return False
        problems = [str(p)[:400] for p in (report.get('problems') or ([report['error']] if report.get('error') else []))]
        if report.get('ok') and not problems:
            count = len(report.get('motors') or ())
            line(f'  OK: all {count} motors answered as this arm expects.')
            on = [name for name, row in (report.get('motors') or {}).items() if isinstance(row, dict) and row.get('torque_on')] \
                if isinstance(report.get('motors'), dict) else []
            if on:
                line(f"  Note: {', '.join(on)} already have power from an earlier session.")
            if report.get('warning'):
                line(f"  Note: {report['warning']}")
            return True
        line(f"  This does not look like a {self.st['arms'][side]['arm']}, or part of it is not answering:")
        for text in problems[:8]:
            line(f'    - {text}')
        what = self.choose('What now?', [('r', 'read it again (after fixing power or wiring)'), ('a', 'choose a different arm'), ('q', 'stop here')], 'a')
        if what == 'q':
            raise KeyboardInterrupt
        if what == 'a':
            raise Again('arms')
        return False

    def step_calibrate(self):
        blocked = []
        for side in self.st['arms']:
            while True:
                try:
                    with self.locked(side):
                        needed = self.calibrate_one(side)
                    if needed:
                        blocked.append(side)
                    break
                except (KeyboardInterrupt, EOFError):
                    raise
                except (Exception, SystemExit) as exc:
                    if isinstance(exc, ArmsBusy):
                        exc = SystemExit(str(exc))
                    self.on_hardware_trouble(side, exc)
        self.st['blocked'] = blocked

    def calibrate_one(self, side):
        arm = self.build(side)
        name = self.st['arms'][side]['arm']
        the = self.the(side, True)
        if hasattr(arm, 'calibrate_gripper'):
            return self.calibrate_yam(side, arm)
        kind = getattr(arm, 'calibration', None)
        if kind in (None, 'none'):
            line(f'{the} needs no calibration.')
            return False
        have = stored.load(arm.arm_id, arm.bus) is not None
        if have and not self.yes(f'{the} is already calibrated. Do it again?', False):
            return False
        if kind == 'optional' and not have and not self.yes(f'Record the gripper range of {self.the(side)} by hand? (optional: the arm also runs on its table values)', False):
            return False
        if stored.offer_lerobot(arm, self.yes, ask=self.raw, say=line):     # LeRobot's file, the servos only read
            return False
        line(f'Calibration teaches Roborun where the joints of {self.the(side)} are. Its motors stay off, so it goes limp:')
        line('hold it, or rest it on the table. Keep the jaws empty. You will:')
        if kind == 'required':
            zero = (getattr(arm, 'options', None) or {}).get('zero_pose')
            line(f"  1. put the arm in its zero pose{f' ({zero})' if zero else ''}, close the gripper, and press Enter")
            line('  2. slowly move every joint as far as it goes each way, open the gripper fully, and press Enter')
        else:
            line('  1. close the gripper fully, and press Enter')
            line('  2. open the gripper fully, and press Enter')
        self.picture(name)
        if not self.yes(SAFE, False):
            return self.blocked(side, kind == 'required' or kind == 'gripper', name)
        stored.record(arm, ask=self.raw, say=line)
        return False

    def picture(self, name):
        if not is_own_driver(name):
            line(f'  A picture of each step: {PICTURES}/{name}.png')

    def blocked(self, side, required, name):
        if required:
            line(f'{self.the(side, True)} cannot start until it is calibrated: roborun hw calibrate {self.spec(side)}')
            return True
        return False

    def calibrate_yam(self, side, arm):
        the = self.the(side, True)
        if not arm._jaw_needs_stops():
            line(f'{the} needs no calibration.')
            return False
        if arm._jaw_memory() is not None and not self.yes(f'The gripper of {self.the(side)} is already calibrated. Do it again?', False):
            return False
        line(f'The gripper of {self.the(side)} has to find its two ends once. Only the jaw motor gets power: the jaw closes and')
        line('opens fully, then its motor switches off. The rest of the arm is not touched. Keep the jaws empty.')
        self.picture(self.st['arms'][side]['arm'])
        if not self.yes(SAFE, False):
            line(f'Not done. {the} cannot start until it is: roborun hw calibrate {self.spec(side)}')
            return True
        with self.vendor_chatter():
            result = arm.calibrate_gripper()
        line(f"  Done: the jaw travels {result['stroke_rad']:.2f} rad.")
        return False

    def vendor_chatter(self):
        """What driver libraries print goes to a log, so the instructions stay readable."""
        logs = Path(robot.HOME) / 'logs'
        logs.mkdir(parents=True, exist_ok=True)
        return contextlib.redirect_stderr(open(logs / 'setup-arms.log', 'a'))

    def switch_off(self, arm):
        if not getattr(arm, 'can_power_off', False):
            return
        with _NoInterrupt():
            try:
                still_on = arm.switch_off()
            except Exception as exc:
                line(f'  Could not confirm the motors are off ({exc}). If the arm is still stiff, turn off its power.')
                return
        if still_on:
            line(f"  {', '.join(still_on)} did not switch off. Turn off the arm's power.")

    # -- wrist cameras ----------------------------------------------------------------------------------------------
    def wrist_applies(self):
        return {'left', 'right'} <= set(self.st['cameras']) and {'left', 'right'} <= set(self.st['arms'])

    def step_wrist(self):
        cams, arms = self.st['cameras'], self.st['arms']
        if not self.yes('Check that the wrist cameras are on the right arms? You move each arm by hand; nothing is powered.'):
            return
        from . import camera_roles, cameras
        arms_open, sources = {}, {}
        try:
            with ArmsLock('roborun setup', [arms[s]['address'] for s in arms]):
                for side in ('left', 'right'):
                    arm = self.build(side)
                    arm.open_dry()
                    arms_open[side] = arm
                    sources[side] = cameras.build_sources(dict(cameras={side: cams[side]}), robot.camera_socket())[side]
                    sources[side].open()
                def frame(role):
                    for _ in range(20):
                        item = sources[role].read()
                        if item is not None:
                            return item['rgb']
                    raise RuntimeError('no image')
                def positions():
                    import numpy as np
                    return {side: np.asarray(arm.read_hardware()['positions_rad'], float) for side, arm in arms_open.items()}
                self.enter('For the next 20 seconds: move the LEFT arm by hand for about 8 seconds, keeping the right one still, then '
                           'the RIGHT arm with the left one still.')
                answer = camera_roles.watch(None, 20.0, read_frame=frame, read_arms=positions)
        except (KeyboardInterrupt, EOFError):
            raise
        except (Exception, SystemExit) as exc:
            line(f'The check could not run ({exc}). After setup you can run: roborun cameras --check-roles')
            return
        finally:
            for arm in arms_open.values():
                try: arm.close_hardware()
                except Exception: pass
            for source in sources.values():
                try: source.close()
                except Exception: pass
        line(f"Wrist cameras: {answer['verdict']}. {answer['reason']}")
        if answer['verdict'] == 'swapped' and self.yes('Swap the left and right wrist cameras?'):
            cams['left'], cams['right'] = cams['right'], cams['left']
            self.save()

    # -- 6. the movement check ------------------------------------------------------------------------------------
    def step_motion(self):
        for side in self.st['arms']:
            while not self.motion_one(side):
                pass

    def motion_one(self, side):
        row = self.st['arms'][side]
        the = self.the(side)
        done = direction_check.load_result(row['arm'], row['address'])
        if done and done.get('result') == 'passed' and not self.redo_motion:
            when = time.strftime('%Y-%m-%d', time.localtime(done['at']))
            if not self.yes(f'{the[0].upper() + the[1:]} passed the movement check on {when}. Run it again?', False):
                return True
        line(f'Movement check on {the} ({self.spec(side)}): the motors get power, and each joint moves about 3 degrees and')
        line('back, slowly, one after the other, then the gripper. It is the only check that catches a joint turning the wrong way.')
        line('Keep a hand on the power switch and the jaws empty. The arm must rest on something or be held: when the check')
        line('ends, however it ends, the motors switch off and the arm goes limp. If you say no, Roborun still runs and reminds you.')
        if not self.yes(SAFE, False):
            direction_check.save_result(row['arm'], row['address'], 'skipped', how='guided setup: not ready')
            line(f'Movement check skipped, and recorded. roborun up will remind you; to run it later: roborun hw check {self.spec(side)} --move')
            return True
        try:
            with self.locked(side):
                arm = self.build(side)
                try:
                    with self.vendor_chatter():
                        report = direction_check.run(arm, say=line, confirmed=True, finish=lambda: self.end_powered(arm))
                finally:
                    self.switch_off(arm)
            if report['ok'] and self.ask('Every joint moved as shown?', 'y').lower() not in ('y', 'yes'):
                report.update(ok=False, stopped='you saw a joint move other than as shown')
                for joint in report['joints']:
                    if joint['result'] == 'ok':
                        joint['result'] = 'not confirmed by you'
        except (KeyboardInterrupt, EOFError):
            direction_check.save_result(row['arm'], row['address'], 'stopped', how='guided setup: interrupted')
            raise
        except (Exception, SystemExit) as exc:
            self.on_hardware_trouble(side, exc)
            return False
        result = direction_check.result_of(report)
        direction_check.save_result(row['arm'], row['address'], result, stopped=report['stopped'], how='guided setup')
        if result == 'passed':
            line(f'{the[0].upper() + the[1:]} passed the movement check, and that is recorded.')
            return True
        if result == 'partial':
            line('Some joints had no room to move, so this is recorded as not fully checked.')
            return True
        line(f"The movement check stopped: {report['stopped']}")
        what = self.choose('What now?', [('r', 'run it again'), ('c', 'continue anyway (this is recorded, and roborun up will say so)'), ('q', 'stop here')], 'q')
        if what == 'q':
            raise KeyboardInterrupt
        return what == 'c'

    def end_powered(self, arm):
        """After a powered step: the stream stops, holding fades, the motors go off. Ctrl-C only shortens the wait."""
        if not getattr(arm, 'can_power_off', False):
            arm.close()
            return
        with _NoInterrupt() as guard:
            try:
                arm.power_down(GRACE_S, say=line, wait=guard.wait)
            except Exception as exc:
                line(f'  While switching off: {exc}')

    # -- 7. robot.yaml ------------------------------------------------------------------------------------------------
    def step_write(self):
        now = settings.load(warn=lambda _: None)
        name = getattr(self.a, 'robot', None) or now.get('robot') or settings.default_robot_name()
        if not now.get('robot') and not getattr(self.a, 'robot', None):     # never asked: robot.yaml keeps it editable
            line(f"This robot is called {name} on the website (this computer's name; to change it, edit robot: in robot.yaml).")
        self.st['robot'] = name
        if self.st.get('mode') == 'sim':
            changes = dict(robot=name, profile='yam_dual', sim='cup')
            line('Two simulated YAM arms will run. To use real arms later: roborun setup')
        else:
            arms = dict(self.st['arms'])
            for side in ('left', 'right'):
                if side not in arms and (now.get('arms') or {}).get(side):
                    arms[side] = None
            # a role setup did not assign is removed only when it is a camera setup itself finds (realsense:, usb:) and so
            # could have offered; your own camera class or stream address stays as you wrote it
            cameras = dict(self.st['cameras'])
            for role, entry in (now.get('cameras') or {}).items():
                if role not in cameras and _kept_camera(entry) is None:
                    cameras[role] = None
            changes = dict(robot=name, arms=arms, cameras=cameras, sim=None)
            profile = self.profile_to_write(now)
            if profile:
                changes['profile'] = profile
        file = settings.update_file(changes)
        line(f'Saved {file}. Your own edits in it were kept; the previous version is {file.name}.before-setup.')

    def profile_to_write(self, now):
        wanted = self.st.get('profile')
        have = now.get('profile')
        if have and wanted and have != wanted:
            if not self.yes(f'robot.yaml says profile {have}, and these arms suggest {wanted}. Change it to {wanted}?', False):
                return have
        return wanted or have

    # -- after the robot is up: the overhead camera ---------------------------------------------------------------------
    def overhead_step(self):
        """Run needs a confirmed camera pose, and one arm in one pose cannot give it: offer the second pose now."""
        if self.st.get('mode') == 'sim' or not robot.CONFIG.exists():
            return
        from . import overhead
        from .arms.hardware import loaded_rig
        config = json.loads(robot.CONFIG.read_text())
        backend = config.get('backend') or 'http://127.0.0.1:8120'
        before = {number: signal.signal(number, _interrupted) for number in (signal.SIGTERM, signal.SIGHUP)}
        try:
            state = overhead.check(backend)
            if not state or state.get('code') == 'no_depth':       # a webcam: setup says so once, at its end
                return
            line()
            line('--- Overhead camera')
            line('Overhead camera: ' + overhead.describe(state))
            if not overhead.second_pose_helps(state):
                return
            if state.get('state') == 'rough':
                line('One arm pose is not enough to prove what the camera found is your arm. Until it has seen a second pose,')
            else:
                line('Seeing the arm in a second pose tells which place in the picture is the arm. Until then,')
            line('tasks can preview but will not run.')
            rig = loaded_rig(config['manifest'])
            checks = [direction_check.load_result(r['arm'], r['address']) or {} for r in self.st['arms'].values()]
            trusted = all(row.get('result') == 'passed' for row in checks)
            if not trusted:
                line('The movement check has not passed, so Roborun will not move the arm for this; you can move it by hand.')
            final = overhead.improve(backend, lambda text: self.yes(text, False), rig_file=rig.get('profile_rig'), models_dir=rig.get('models_dir'),
                                     ask=self.raw, moves_allowed=trusted, say=line)
            if final and final.get('state') == 'calibrated':
                line('The overhead camera is confirmed: tasks can run.')
            else:
                line('The overhead camera is not confirmed yet: tasks can preview, not run. To try again: roborun setup')
        except (KeyboardInterrupt, EOFError):
            line()
            line('Stopped. The arm is holding where it is and Roborun is still running. To try again: roborun setup')
            raise SystemExit(130) from None
        finally:
            for number, handler in before.items():
                signal.signal(number, handler)

    def finish(self):
        self.st['finished_at'] = time.time()
        self.save()
        cams = len(self.st.get('cameras') or {})
        after = [f'Next: open {APP} on this computer. Your robot appears there and you can tell it what to do.',
                 'Other useful commands: roborun status, roborun down, roborun setup (to change anything).']
        if self.st.get('mode') != 'sim' and not cams:
            after.insert(1, 'No cameras are set up: Manifest needs a camera view to plan. Add them with roborun setup.')
        from . import cameras
        no_depth = self.st.get('mode') != 'sim' and cameras.webcams_only((self.st.get('cameras') or {}).values())
        if no_depth:
            cameras.SETUP_SAYS_NO_DEPTH = True             # said once, here, not again by `roborun up`
            after.append(cameras.NO_DEPTH)
        self.after = after
        if self.a is not None:
            self.a.after_up = after
            self.a.after_start = [self.overhead_step]
        line()
        if self.st.get('blocked'):
            line('Setup is saved, but Roborun cannot start yet: ' + ', '.join(f'the {s} arm still needs its calibration' for s in self.st['blocked']))
            line('Run roborun setup again when the arm is ready.')
            raise SystemExit(1)
        if self.from_setup:
            if self.yes('All set. Start Roborun now?'):
                return 'start'
            line('When you are ready: roborun up')
            if no_depth:
                line(cameras.NO_DEPTH)
            return 'done'
        line('All set. Starting Roborun...')
        return 'done'


class _NoProfile(Exception):
    pass
