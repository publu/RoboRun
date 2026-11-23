"""Connecting to arms in plain words: `roborun hw detect`, the checks `roborun up` makes first, and readable failures.

Every failure says which adapter, what is wrong and what to do next. `ROBORUN_DEBUG=1` shows the traceback instead.
"""
import contextlib
import errno
import functools
import io
import json
import os
import sys
from pathlib import Path

from . import canbus, catalog, is_own_driver, parse_spec
from .canbus import AdapterError

FAILURE_FILE = 'station-failure.txt'


def _home():
    from .. import robot
    return Path(robot.HOME)


def _chain(exc):
    seen = []
    while exc is not None and exc not in seen:
        seen.append(exc)
        exc = exc.__cause__ or exc.__context__
    return seen


def _serial_text(exc, name):
    code, text = getattr(exc, 'errno', None), str(exc)
    if code in (errno.EACCES, errno.EPERM) or 'permission' in text.lower():
        device = name.rpartition('@')[2]
        return f'{name}: this user may not open the serial port. Next: {canbus.serial_permission_fix(device)}'
    if code == errno.EBUSY or 'busy' in text.lower() or 'lock' in text.lower():
        return f'{name}: the serial port is in use by another program. Next: close it (`roborun down` if it is this robot)'
    if code == errno.ENOENT or 'no such file' in text.lower():
        return f'{name}: the serial port does not exist. Next: check the USB cable, or run `roborun hw detect`'
    return f'{name}: the serial port failed ({text}). Next: replug the adapter and run `roborun hw detect`'


def explain(exc, spec=None) -> str:
    """One line for any failure on the way to an arm: adapter, cause, next step."""
    chain = _chain(exc)
    for item in chain:
        if isinstance(item, AdapterError):
            return str(item)
    name = spec or 'the arm'
    text = str(exc).replace('AdapterError: ', '').strip().rstrip('.') or type(exc).__name__
    try:
        import serial
        for item in chain:
            if isinstance(item, serial.SerialException):
                return _serial_text(item, name)
    except ImportError:
        pass
    try:
        import usb.core
        if any(isinstance(item, usb.core.NoBackendError) for item in chain):
            return f'{name}: libusb is missing. Next: reinstall with curl -fsSL https://roborun.app/install.sh | sh'
    except ImportError:
        pass
    if isinstance(exc, ImportError):
        return f'{name}: the Python package {exc.name or text} is missing. Next: reinstall with curl -fsSL https://roborun.app/install.sh | sh'
    if isinstance(exc, PermissionError):
        device = exc.filename or 'the device'
        return f'{name}: permission denied ({text}). Next: {canbus.serial_permission_fix(device)}'
    if str(exc).startswith('rules:'):
        return str(exc)
    if isinstance(exc, (ValueError, TypeError, KeyError)):
        return f"{name}: its settings were refused ({text}). Next: fix this arm's settings in robot.yaml"
    return (f'{name}: {text}. Next: check that the arm is powered and its CAN or serial cable is seated, '
            f'then run `roborun hw detect`' + (f' and `roborun hw check {spec}`' if spec else ''))


def plain(function, spec_of=lambda a: None):
    """`function`, with any failure turned into its one-line explanation."""
    @functools.wraps(function)
    def run(a):
        try:
            return function(a)
        except Exception as exc:
            if os.environ.get('ROBORUN_DEBUG'):
                raise
            raise SystemExit(explain(exc, spec_of(a))) from None
    return run


# -- roborun hw detect ------------------------------------------------------------------------------------------------
def adapter_rows():
    from .serial_servo import list_serial_ports
    rows, notes = [], []
    usb = canbus.list_adapters()
    links = canbus.can_links() if sys.platform == 'linux' else []
    by_port = {link['usb_port']: link['name'] for link in links if link['usb_port']}
    for link in links:
        note = None if link['state'] == 'up' else f"the link is {link['state']}: Roborun brings it up, or names the command"
        rate = f", {link['bitrate']} bit/s" if link['bitrate'] else ''
        rows.append(dict(address=link['name'], bus='CAN', kind='socketcan', detail=f"{link['driver'] or link['kind'] or 'CAN link'}{rate}", note=note))
    for row in usb:
        note = row.get('note')
        twin = by_port.get(row['usb_port'])
        if twin and row['kind'] == 'gs_usb':
            note = ' '.join(filter(None, [note, f'also the kernel link {twin}: use one of the two addresses']))
        rows.append(dict(address=row['address'], bus='CAN', kind=row['kind'], detail=row['product'] or '', note=note))
    can_ports = {os.path.realpath(r['port']) for r in usb if r.get('port')}
    for row in list_serial_ports():
        if row['usb'] and os.path.realpath(row['device']) not in can_ports:
            rows.append(dict(address=row['address'], bus='serial', kind='serial', detail=row['product'] or row['device'], note=row.get('note')))
    dfu = canbus.dfu_adapters()
    if dfu:
        notes.append(f'{dfu} STM32 device(s) in firmware-update (DFU) mode: remove the boot jumper or run dfu-util -a 0 -s :leave')
    return rows, notes


def cmd_adapters(a) -> int:
    rows, notes = adapter_rows()
    if getattr(a, 'json', False):
        print(json.dumps(dict(adapters=rows, notes=notes), indent=1))
        return 0
    if rows:
        width = max(len(r['address'] or '') for r in rows)
        print(f"{'ADDRESS':<{width}}  {'BUS':<6}  {'TYPE':<9}  DEVICE")
        for r in rows:
            print(f"{r['address'] or '(none)':<{width}}  {r['bus']:<6}  {r['kind']:<9}  {r['detail']}")
            if r['note']:
                print(f"{'':<{width}}  note: {r['note']}")
        print('\nUse it as ARM@ADDRESS, e.g. yam@' + rows[0]['address'] if rows[0]['address'] else '')
    else:
        print('no CAN or serial adapters found: check the USB cable and power')
    for note in notes:
        print(f'note: {note}')
    return 0


# -- what `roborun up` checks in the foreground --------------------------------------------------------------------
def check_hardware(hardware):
    """Every arm's adapter is there and usable, or SystemExit naming all that are not. Nothing is opened or moved."""
    from .serial_servo import resolve_port
    problems, table = [], catalog()
    for side in ('left', 'right'):
        row = (hardware or {}).get(side)
        if not row or is_own_driver(row['arm']):
            continue
        info = table.get(row['arm']) or {}
        bus, transport = row.get('bus'), info.get('transport')
        if transport not in ('can', 'serial'):
            continue
        if not bus:
            problems.append(f"{side} arm ({row['arm']}): no address given. Next: set one; `roborun hw detect` prints them")
            continue
        try:
            if transport == 'can':
                canbus.check(bus)
            else:
                resolve_port(bus)
        except Exception as exc:
            spec = '{}@{}'.format(row['arm'], bus)
            problems.append(f'{side} arm: {explain(exc, spec)}')
    if problems:
        raise SystemExit('\n'.join(problems))
    for text in bus_load_warnings(hardware):
        print(f'warning: {text}')


def bus_load_warnings(hardware):
    """Arms whose streams would saturate their CAN bus, two arms on one adapter counted together. Opens nothing."""
    from . import open_arm
    per_bus = {}
    for side in ('left', 'right'):
        row = (hardware or {}).get(side)
        if not row or is_own_driver(row['arm']) or (catalog().get(row['arm']) or {}).get('transport') != 'can' or not row.get('bus'):
            continue
        try:
            with contextlib.redirect_stdout(io.StringIO()):          # its setting notes are said when it really opens
                arm = open_arm(row['arm'], side, row['bus'], connect=False, **(row.get('options') or {}))
            frames = arm.can_frames_per_s()
        except Exception:
            continue                                  # the arm's own checks say what is wrong with it
        entry = per_bus.setdefault(row['bus'], [0.0, [], getattr(arm, 'bitrate', canbus.BITRATE)])
        entry[0] += frames
        entry[1].append(f"the {side} arm ({row['arm']})")
    out = []
    for bus, (frames, who, bitrate) in per_bus.items():
        text = canbus.load_warning(bus, frames, bitrate, ' and '.join(who))
        if text:
            out.append(text)
    return out


def _failure_path():
    return _home() / 'robot' / FAILURE_FILE


def _wrap_station(inner):
    @functools.wraps(inner)
    def run(a):
        path = _failure_path()
        path.unlink(missing_ok=True)
        try:
            check_hardware(json.loads(Path(a.hardware).read_text()))
            return inner(a)
        except KeyboardInterrupt:
            raise
        except SystemExit as exc:
            if exc.code in (None, 0):
                raise
            message = str(exc.code)
        except Exception as exc:
            if os.environ.get('ROBORUN_DEBUG'):
                raise
            message = explain(exc)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(message + '\n')
        raise SystemExit(message)
    return run


def say_movement_checks(hardware):
    """A first real run does not refuse an arm whose movement check was skipped, and does not hide it either."""
    from .direction_check import load_result, save_result
    import time
    for side in ('left', 'right'):
        row = (hardware or {}).get(side)
        if not row or is_own_driver(row['arm']):
            continue
        done = load_result(row['arm'], row.get('bus'))
        if done and done.get('result') == 'passed':
            continue
        spec = '{}@{}'.format(row['arm'], row.get('bus'))
        when = time.strftime('%Y-%m-%d', time.localtime(done['at'])) if done else None
        said = {'skipped': f'was skipped on {when}', 'failed': f'FAILED on {when}', 'stopped': f'was stopped before the end on {when}',
                'partial': f'left joints out on {when}'}.get((done or {}).get('result'), 'has not been run')
        print(f'note: {side} arm: the movement check {said}. To run it: roborun hw check {spec} --move')
        save_result(row['arm'], row.get('bus'), (done or {}).get('result', 'not run'), started_without_check_at=time.time())


def say_cameras(hardware, wait_s=8):
    """What `up` knows about the cameras, said out loud: none configured, one that cannot load, one that does not stream."""
    if not hardware:
        return
    if not hardware.get('cameras'):
        print('note: no cameras are set up, so tasks cannot plan. Add them with: roborun setup')
        return
    import socket
    import time
    from .. import cameras, robot
    for role, (kind, ident, _, _) in cameras.camera_configs(hardware).items():
        if kind == 'custom':
            try:
                cameras.load_camera_class(ident)
            except ValueError as exc:
                print(f'warning: {role} camera: {exc}')
    status, end = {}, time.monotonic() + wait_s
    while time.monotonic() < end:
        try:
            with socket.socket(socket.AF_UNIX) as conn:
                conn.settimeout(2); conn.connect(robot.camera_socket())
                cameras.send(conn, json.dumps(dict(operation='status')).encode())
                status = json.loads(cameras.receive(conn, cameras.MAX_PACKET)).get('cameras') or {}
        except (OSError, ValueError):
            status = {}
        if status and all(row.get('state') != 'connecting' for row in status.values()):
            break
        time.sleep(.25)
    for role, row in status.items():
        if row.get('state') != 'streaming':
            print(f"warning: {role} camera ({cameras.redact(row.get('source') or '')}): {row.get('error') or 'no picture yet'}")


def _wrap_up(inner):
    @functools.wraps(inner)
    def run(a):
        from .. import robot
        from .hardware import config_from_args
        from .settings import setup_if_needed
        path = _failure_path()
        path.unlink(missing_ok=True)
        try:
            setup_if_needed(a)
            stored = json.loads(robot.CONFIG.read_text()).get('hardware') if robot.CONFIG.exists() else None
            hardware = None if getattr(a, 'sim', None) else config_from_args(a, stored)
            if robot._state('roborun-station') not in ('active', 'running'):   # a running station holds its own adapters
                check_hardware(hardware)
            say_movement_checks(hardware)
            result = inner(a)
            say_cameras(hardware)
            for step in getattr(a, 'after_start', ()):
                step()
            for text in getattr(a, 'after_up', ()):
                print(text)
        except SystemExit as exc:
            if isinstance(exc.code, str) and 'did not come up' in exc.code and path.exists():
                raise SystemExit(path.read_text().strip()) from None
            raise
        except Exception as exc:
            if os.environ.get('ROBORUN_DEBUG'):
                raise
            raise SystemExit(f'roborun up failed: {explain(exc)}') from None
        return result
    return run


def add_parsers(sub):
    p = sub.add_parser('adapters', help='the CAN and serial adapters connected here, with the ADDRESS to use for each')
    p.add_argument('--json', action='store_true')
    p.set_defaults(func=plain(cmd_adapters))
    arms = sub.choices.get('arms')
    if arms is not None:
        arms.add_argument('--detect', action='store_true', help='list connected adapters instead (same as roborun hw detect)')
        listing = arms.get_default('func')
        arms.set_defaults(func=lambda a: cmd_adapters(a) if a.detect else listing(a))
    motor = sub.choices.get('motor')
    if motor is not None:
        for act in motor._subparsers._group_actions[0].choices.values():
            act.set_defaults(func=plain(act.get_default('func') or motor.get_default('func'), lambda a: a.bus))
    for name in ('arm-check', 'arm-calibrate'):
        parser = sub.choices.get(name)
        if parser is not None:
            parser.set_defaults(func=plain(parser.get_default('func'), lambda a: a.arm))
    station = sub.choices.get('hardware-station')
    if station is not None:
        station.set_defaults(func=_wrap_station(station.get_default('func')))
    up = sub.choices.get('up')
    if up is not None:
        up.set_defaults(func=_wrap_up(up.get_default('func')))
