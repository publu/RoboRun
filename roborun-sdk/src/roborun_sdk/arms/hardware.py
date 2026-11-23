"""`roborun up --left yam@ADDRESS` -> the real station; `roborun hw arms` and `roborun hw check`."""
import json
import os
import signal
import sys
import threading
import time
import uuid
from pathlib import Path

from . import catalog, is_own_driver, open_arm, parse_spec


def config_from_args(a, old=None):
    """The hardware section of the robot's config, or `old` when no hardware flag was given."""
    from .settings import effective
    old = effective(old)
    arm_options = getattr(a, 'arm_options', None)
    if not (a.left or a.right or a.camera or arm_options):
        return _one_adapter_per_arm(old)
    out = dict(old or {})
    for side in ('left', 'right'):
        spec = getattr(a, side)
        if spec:
            arm, bus = parse_spec(spec)
            if is_own_driver(arm):
                where, _, cls = arm.partition(':')
                if where.endswith('.py'):                # the station runs from another folder
                    arm = f'{Path(where).expanduser().resolve()}:{cls}'
            elif arm not in catalog():
                raise SystemExit(f'unknown arm {arm!r}: `roborun hw arms` lists them, or pass your own driver as my_arm.py:MyArm')
            out[side] = dict(arm=arm, bus=bus)
    for item in arm_options or ():
        side, sep, path = item.partition('=')
        if not sep or side not in ('left', 'right') or not out.get(side):
            raise SystemExit(f'--arm-options {item!r}: use left=FILE or right=FILE, for an arm that is configured')
        try:
            options = json.loads(Path(path).expanduser().read_text())
        except (OSError, ValueError) as exc:
            raise SystemExit(f'--arm-options {path}: {exc}')
        if not isinstance(options, dict):
            raise SystemExit(f'--arm-options {path}: a JSON object, e.g. {{"limits": [[-3, 3], ...], "home": [...]}}')
        out[side] = dict(out[side], options=options)
    if a.camera:
        from ..camera_card import overlay_cameras
        cams = dict(out.get('cameras') or {})
        for item in a.camera:
            role, sep, source = item.partition('=')
            if not sep or role not in ('top', 'left', 'right') or not source:
                raise SystemExit(f'--camera {item!r}: use top=SOURCE, left=SOURCE or right=SOURCE')
            cams = overlay_cameras(cams, role, source, say=lambda text: print(f'warning: {text}'))   # as before for id ROLE
        out['cameras'] = cams
    if not (out.get('left') or out.get('right')):
        raise SystemExit('name at least one arm: --left ARM@ADDRESS or --right ARM@ADDRESS')
    from .settings import one_arm_on_the_left
    return _one_adapter_per_arm(one_arm_on_the_left(out))


SHARED_BUS = ('can', 'vcan', 'slcan:', 'serial:', '/dev/')     # one arm per CAN link, CAN adapter or serial port


def _one_adapter_per_arm(hw):
    """Both arms on the same CAN or serial adapter is a settings mistake: said as one, before either arm is opened (opened, the
    second arm only finds the adapter busy, held by Roborun's own station)."""
    left, right = ((hw or {}).get(side) or {} for side in ('left', 'right'))
    bus = str(left.get('bus') or '')
    if bus and bus == str(right.get('bus') or '') and bus.startswith(SHARED_BUS):
        raise SystemExit(f'both arms are set to the same adapter, {bus}: each arm needs its own. Plug in the other arm\'s '
                         'adapter and set its address in robot.yaml (`roborun hw detect` lists them), or run roborun setup')
    return hw


def loaded_rig(root) -> dict:
    """What the station needs to know about the profile the backend has loaded: its rig and where the arm models are."""
    root = Path(root)
    try:
        rig = root / json.loads((root / 'config.json').read_text()).get('rig', 'rig.json')
    except (OSError, ValueError):
        return {}
    return dict(profile_rig=str(rig), models_dir=str(root / 'models')) if rig.is_file() else {}


def check_profile(arms, rig, name='the loaded profile'):
    """[problems]: every configured arm needs an active slot of its side, with its joint count."""
    problems, slots = [], rig.get('arms') or {}
    for side, arm in arms.items():
        slot = slots.get(side)
        if not isinstance(slot, dict):
            problems.append(f'{side} arm: {name} has no {side} arm')
        elif isinstance(slot.get('parked'), dict):
            problems.append(f'{side} arm: {name} keeps its {side} arm parked')
        else:
            wanted, have = int(slot.get('joints', 6)), len(arm.home)
            if wanted != have:
                problems.append(f'{side} arm: {name} plans for {wanted} joints, this arm has {have}')
    return problems


def _tool_model(hw, side):
    """The planner's model of this arm, so the tool speed limit and the rules' tool pose work out of the box."""
    if not (hw.get('profile_rig') and hw.get('models_dir')):
        return None
    try:
        slot = json.loads(Path(hw['profile_rig']).read_text())['arms'][side]
        model = Path(hw['models_dir']) / slot['model']
    except (OSError, ValueError, KeyError, TypeError):
        return None
    return dict(xml=str(model), site='grasp_site') if model.is_file() else None


def _open_arms(hw):
    """Built, checked against the profile, and only then connected. Nothing is enabled before every check passed."""
    arms = {}
    for side in ('left', 'right'):
        if hw.get(side):
            options = dict(hw[side].get('options') or {})
            model = _tool_model(hw, side)
            if model and 'tool_model' not in options:
                options['tool_model'] = model
            arms[side] = open_arm(hw[side]['arm'], side, hw[side].get('bus'), connect=False, **options)
    if hw.get('profile_rig'):
        problems = check_profile(arms, json.loads(Path(hw['profile_rig']).read_text()))
        if problems:
            raise SystemExit('these arms do not match the loaded profile: ' + '; '.join(problems)
                             + '. Change arms in robot.yaml, or the profile (roborun profiles)')
    opened = []
    try:
        for arm in arms.values():
            arm.connect()
            opened.append(arm)
    except BaseException:
        for arm in opened:
            try: arm.close()
            except Exception: pass
        raise
    return arms


def _control(name):
    from .. import robot
    return Path(robot.HOME) / 'robot' / f'station-{name}.json'


def ask_station(what, timeout=None):
    """`roborun down` asks the running station to let go (down), park first (park) or switch off a supported arm (release).
    {side: why it keeps holding}, empty when every motor is off; None when no station here understands the question."""
    try:
        pid = json.loads(_control('ready').read_text())['pid']
        os.kill(pid, 0)
    except (OSError, ValueError, KeyError):
        return None
    ident = uuid.uuid4().hex
    _control('request').write_text(json.dumps(dict(id=ident, do=what)))
    os.kill(pid, signal.SIGUSR1)
    end = time.monotonic() + (timeout or (120 if what == 'park' else 30))
    while time.monotonic() < end:
        try:
            answer = json.loads(_control('answer').read_text())
            if answer.get('id') == ident:
                return answer['held']
        except (OSError, ValueError, KeyError):
            pass
        time.sleep(.1)
    return None


FAULTED = 'its motors faulted and may be off'     # an arm `down` could not keep holding: never reported as held


def let_go_of_running_station(say=print, then='roborun up'):
    """Before a running station is replaced: it switches its motors off itself, as `roborun down` does. Killed instead, the
    motors would stay enabled until their CAN watchdog faulted them (the next start then asks for a power cycle), and a
    raised arm would fall. Raises SystemExit with what to do when an arm is still held or has faulted; returns otherwise."""
    held = ask_station('down')
    if held is None:
        say('The robot station did not answer. If an arm is raised, support it now: its motors may switch off.')
        return
    bad = faulted(held)
    if bad:
        raise SystemExit('Support the ' + ' and '.join(bad) + ' arm now: ' + '; '.join(f'{s} arm: {held[s]}' for s in bad)
                         + f'. Then check its power and cable (roborun arm-check) and run {then} again.')
    if held:
        raise SystemExit('Roborun keeps holding the raised arms, so nothing falls (' + '; '.join(f'{s} arm: {w}' for s, w in held.items())
                         + f'). Run roborun down --park, or support them by hand and run roborun down --release; then {then}.')


def faulted(held):
    """The sides of a `down` answer whose arm faulted instead of holding."""
    return [side for side, why in (held or {}).items() if str(why).startswith(FAULTED)]


def _answer_station(arms, off):
    request = json.loads(_control('request').read_text())
    held = {}
    for side, arm in arms.items():
        if side in off:
            continue
        try:
            if not hasattr(arm, 'let_go'):
                arm.close()
                why = None
            else:
                why = None
                if request['do'] == 'park':
                    result = arm.park()
                    if result.get('state') != 'confirmed':
                        why = f"it did not reach its rest pose ({result.get('why') or result.get('state')})"
                why = why or arm.let_go(force=request['do'] == 'release')
        except Exception as exc:
            why = f'{type(exc).__name__}: {exc}'[:200]
        thread = getattr(arm, '_thread', None)
        if why and thread is not None and not thread.is_alive():
            # its control loop has stopped: nothing is commanding the motors any more. (A refused or broken rules file sets an
            # error too, but the loop keeps holding the arm: that arm is held, and says why.)
            why = f'{FAULTED} ({why})'
        if why:
            held[side] = why
        else:
            off.add(side)
    _control('answer').write_text(json.dumps(dict(id=request['id'], held=held)))
    return not held


def cmd_station(a) -> int:
    from ..embodiment import run_station
    from .safety import ArmsBusy, ArmsLock
    hw = json.loads(Path(a.hardware).read_text())
    lock = ArmsLock('the robot station', [hw[s].get('bus') for s in ('left', 'right') if hw.get(s)])
    arms, srv, off = {}, None, set()
    stop, asked = threading.Event(), threading.Event()
    try:
        try:
            lock.acquire()
        except ArmsBusy as exc:
            raise SystemExit(str(exc))
        arms = _open_arms(hw)
        slots = arms if 'left' in arms else dict(left=arms['right'])    # the station's first slot is the one that moves
        station, srv = run_station(slots['left'], slots.get('right'), port=a.port, sessions_dir=a.sessions, predict=a.predict)
        for number in (signal.SIGTERM, signal.SIGINT):
            signal.signal(number, lambda *_: stop.set())
        signal.signal(signal.SIGUSR1, lambda *_: asked.set())
        _control('ready').parent.mkdir(parents=True, exist_ok=True)
        _control('ready').write_text(json.dumps(dict(pid=os.getpid())))
        while not stop.wait(.2):
            if asked.is_set():
                asked.clear()
                if srv is not None:
                    srv.close()                               # from here nothing commands the arms; they only hold
                    srv = None
                if _answer_station(arms, off):
                    break
    finally:
        _control('ready').unlink(missing_ok=True)
        if srv is not None:
            srv.close()                                       # nothing commands the arms any more, then they close safely
        for side, arm in arms.items():
            if side in off:
                continue
            try:
                arm.close()
            except Exception as exc:
                print(json.dumps(dict(event='arm_close_failed', error=f'{type(exc).__name__}: {exc}'[:200])), flush=True)
        lock.release()
    return 0


def cmd_arms(a) -> int:
    rows = catalog()
    if a.json:
        print(json.dumps(rows, indent=1))
        return 0
    for arm, row in rows.items():
        print(f"{arm:<18} {row['label']:<34} {row.get('status', 'beta'):<10} {row.get('motors', '')}")
    return 0


def cmd_check(a) -> int:
    from .safety import ArmsBusy, ArmsLock
    arm, bus = parse_spec(a.arm)
    options = json.loads(Path(a.options).expanduser().read_text()) if a.options else {}
    try:
        with ArmsLock('arm-check', [bus]):
            obj = open_arm(arm, a.side, bus, connect=False, **options)
            if getattr(a, 'repair', False):
                from .motor_tools import confirm, offer_repairs
                return 0 if offer_repairs(obj, lambda question, _: confirm(question, a.yes), save=not a.no_save) else 1
            if getattr(a, 'move', False):
                from .direction_check import result_of, run, save_result
                report = run(obj)
                save_result(arm, bus, result_of(report), stopped=report['stopped'], how='arm-check --move')
                print(json.dumps(report, indent=1, default=str))
                return 0 if report['ok'] else 1
            try:
                report = obj.survey()
            except NotImplementedError as exc:
                raise SystemExit(str(exc))
            except Exception as exc:
                raise SystemExit(f'could not read {a.arm}: {exc}. Check that it is powered and connected.')
    except ArmsBusy as exc:
        raise SystemExit(str(exc))
    print(json.dumps(report, indent=1, default=str))
    if report.get('ok') is False:
        problems = report.get('problems') or [report.get('error') or 'the check failed']
        print(f"{a.arm}: not ready: {'; '.join(str(p) for p in problems)[:600]}. Next: check that it has power and that it is this arm",
              file=sys.stderr)
        return 1
    return 0


def add_parsers(sub):
    s = sub.add_parser('hardware-station', help=argparse_hidden())
    s.add_argument('--hardware', required=True); s.add_argument('--port', type=int, default=8091)
    s.add_argument('--sessions'); s.add_argument('--predict', default='http://127.0.0.1:8120/internal/video-motion/predict')
    s.set_defaults(func=cmd_station)
    from .settings import add_parser as add_settings
    add_settings(sub)
    l = sub.add_parser('arms', help='the arms Roborun can drive'); l.add_argument('--json', action='store_true')
    l.set_defaults(func=cmd_arms)
    c = sub.add_parser('arm-check', help='read-only check of an arm: nothing is enabled or moved (unless --move)')
    c.add_argument('arm', help='ARM@ADDRESS, e.g. yam@slcan://SERIAL'); c.add_argument('--side', default='left')
    c.add_argument('--options', help='a JSON file of driver options: limits, home, ...')
    c.add_argument('--move', action='store_true', help='powered direction check: tiny moves, one joint at a time, each confirmed by you')
    c.add_argument('--repair', action='store_true', help='Damiao: show motors in the wrong control mode, ask, then write and read it back (motors stay disabled)')
    c.add_argument('--yes', action='store_true', help='with --repair: do not ask'); c.add_argument('--no-save', action='store_true', help="with --repair: do not save to the motor's flash (lasts until power-off)")
    c.set_defaults(func=cmd_check)
    from .motor_tools import add_parser as add_motor
    add_motor(sub)


def argparse_hidden():
    return 'run the real arms named in a hardware file (started by roborun up)'
