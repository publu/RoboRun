"""Per-arm calibration, recorded by hand with torque off: `roborun hw calibrate ARM@ADDRESS`.

Stored as <ROBORUN_HOME>/calibration/<arm>@<address>.json. What is recorded depends on the arm's table:
    required   the zero pose, every joint's range (swept by hand) and the gripper's two ends
    gripper    only the gripper's two ends: the joints' zeros are the maker's
    optional   the same as gripper, and the arm also runs on the table's values when no file exists
"""
import json
import os
import re
import sys
import threading
import time
from pathlib import Path

from .. import net

MIN_SWEEP_RAD = 0.3
MIN_JAW_RAD = 0.2
MARGIN_RAD = 0.02


def path(arm, address):
    home = net.roborun_home()
    return home / 'calibration' / (re.sub(r'[^A-Za-z0-9._-]+', '_', f'{arm}@{address}').strip('_') + '.json')


def load(arm, address):
    try:
        return json.loads(path(arm, address).read_text())
    except FileNotFoundError:
        return None


def save(arm, address, data):
    target = path(arm, address)
    target.parent.mkdir(parents=True, exist_ok=True)
    temp = target.with_suffix('.tmp')
    temp.write_text(json.dumps(data, indent=1))
    temp.replace(target)
    return target


class _Sampler(threading.Thread):
    """Follows every motor while the operator moves the arm; counts whole turns across the encoder seam."""

    def __init__(self, arm, start):
        super().__init__(daemon=True)
        self.arm, self.now = arm, dict(start)
        self.lo, self.hi, self.turns, self.last = dict(start), dict(start), dict.fromkeys(start, 0.0), dict(start)
        self.done = threading.Event()
        self.error = None

    def run(self):
        while not self.done.is_set():
            try:
                self.take(self.arm.calibration_read())
            except Exception as exc:
                self.error = exc
                return
            time.sleep(.02)

    def take(self, raw):
        for name, value in raw.items():
            step = value - self.last[name]
            if abs(step) > 3.5:                            # one reading cannot move this far: the encoder wrapped
                self.turns[name] -= round(step / 6.283185307179586) * 6.283185307179586
            self.last[name] = value
            value += self.turns[name]
            self.now[name] = value
            self.lo[name], self.hi[name] = min(self.lo[name], value), max(self.hi[name], value)


def record(arm, ask=None, say=print):
    """Walks the operator through it; nothing here ever enables a motor."""
    ask = ask or input
    if arm.calibration == 'none':
        raise SystemExit(f'{arm.arm_id} needs no calibration')
    full = arm.calibration == 'required'
    states = arm.calibration_open()
    try:
        if any(s.enabled for s in states.values()):
            ask(f'{arm.arm_id}: torque is on. SUPPORT THE ARM: it will go limp. Press Enter to turn torque off. ')
            arm._torque_off()
        pose = 'Put the arm in its zero pose' + (f' ({arm.options["zero_pose"]})' if arm.options.get('zero_pose') else '')
        ask(f'{pose if full else "Keep the arm supported"} and close the gripper fully, then press Enter. ')
        raw = arm.calibration_read()
        facts = {}
        if full:
            facts, raw = arm._on_zero_pose(raw)
        closed = raw[arm.gripper.name]
        sampler = _Sampler(arm, raw)
        sampler.start()
        what = 'Move every joint slowly through its full safe range, open' if full else 'Open'
        ask(f'{what} the gripper fully, then press Enter. ')
        sampler.done.set()
        sampler.join(1)
        if sampler.error is not None:
            raise sampler.error
        sampler.take(arm.calibration_read())
    finally:
        arm.calibration_close()
    name = arm.gripper.name
    opened = sampler.lo[name] if closed - sampler.lo[name] > sampler.hi[name] - closed else sampler.hi[name]
    if abs(opened - closed) < MIN_JAW_RAD:
        raise SystemExit('the gripper barely moved: nothing saved, run it again and open the gripper fully')
    data = dict(version=1, arm=arm.arm_id, address=arm.bus, recorded_at=time.time(), mode=arm.calibration,
                motors={m.name: m.id for m in arm.motors}, gripper=dict(closed=closed, open=opened), facts=facts)
    if full:
        joints, short = {}, []
        for motor in [m for m in arm.joints if m is not None] + arm.held:
            lo, hi = sampler.lo[motor.name] - raw[motor.name], sampler.hi[motor.name] - raw[motor.name]
            if motor.role == 'joint' and hi - lo < MIN_SWEEP_RAD:
                short.append(motor.name)
            lo, hi = sorted((motor.sign * lo, motor.sign * hi))
            joints[motor.name] = dict(zero=raw[motor.name], lo=lo - MARGIN_RAD, hi=hi + MARGIN_RAD)
        if short:
            raise SystemExit(f'these joints barely moved: {", ".join(short)}. Nothing saved: their sweep sets their limits')
        data['joints'] = joints
    target = save(arm.arm_id, arm.bus, data)
    say(f'saved {target}')
    return target


# -- a calibration LeRobot already made ------------------------------------------------------------------------------
# LeRobot (Apache-2.0) keeps one JSON file per arm: {motor name: {id, drive_mode, homing_offset, range_min, range_max}},
# under $HF_LEROBOT_CALIBRATION, else $HF_LEROBOT_HOME/calibration, else ~/.cache/huggingface/lerobot/calibration, in
# robots/<type>/ and teleoperators/<type>/. Only that file format is used here. Its calibration wrote each servo's
# homing offset so the pose "every joint at the middle of its range" reads the middle of the encoder (2047), which is
# the zero pose these arms' tables name: so the import keeps the servos exactly as they are, takes 2047 as each zero
# and LeRobot's recorded range as each joint's range, and checks every offset against the servo before saving.
LEROBOT_MID = 2047
LEROBOT_SLACK = 50                  # counts a joint may sit outside the recorded range and still be this arm


def lerobot_dirs():
    if os.environ.get('HF_LEROBOT_CALIBRATION'):
        return [Path(os.environ['HF_LEROBOT_CALIBRATION'])]
    if os.environ.get('HF_LEROBOT_HOME'):
        return [Path(os.environ['HF_LEROBOT_HOME']) / 'calibration']
    hf = Path(os.environ.get('HF_HOME') or Path.home() / '.cache' / 'huggingface')
    return [hf / 'lerobot' / 'calibration']


def _lerobot_rows(data, arm):
    """{our motor name: LeRobot's row} when the file covers every motor of this arm with the same ids, else None."""
    if not isinstance(data, dict):
        return None
    out = {}
    for motor in arm.motors:
        row = data.get(motor.name)
        if not isinstance(row, dict) or int(row.get('id', -1)) != motor.id:
            return None
        out[motor.name] = row
    return out


def lerobot_files(arm):
    """[(path, rows)] of the LeRobot calibration files that fit this arm's motors."""
    found = []
    for folder in lerobot_dirs():
        for path in sorted(folder.glob('*/*/*.json')) if folder.is_dir() else ():
            try:
                rows = _lerobot_rows(json.loads(path.read_text()), arm)
            except (OSError, ValueError, TypeError):
                continue
            if rows:
                found.append((path, rows))
    return found


def import_lerobot(arm, path, ask=None, say=print):
    """Reads the servos (never writes them) and saves Roborun's calibration from LeRobot's file. `ask`: a terminal
    question used to confirm which end of the gripper is closed; without one, LeRobot's range_min is taken as closed."""
    if arm.calibration != 'required' or not hasattr(arm, '_homing'):
        raise SystemExit(f'{arm.arm_id} does not take a LeRobot calibration: run roborun hw calibrate {arm.arm_id}@{arm.bus}')
    path = Path(path).expanduser()
    try:
        rows = _lerobot_rows(json.loads(path.read_text()), arm)
    except (OSError, ValueError) as exc:
        raise SystemExit(f'{path}: cannot read it ({exc})')
    if rows is None:
        raise SystemExit(f'{path} is not for this arm: it needs ' + ', '.join(f'{m.name} (id {m.id})' for m in arm.motors))
    for name, row in rows.items():
        if int(row.get('drive_mode', 0)) != 0:
            raise SystemExit(f'{path}: {name} has drive_mode {row["drive_mode"]}: only drive_mode 0 files can be imported')
        if not int(row['range_min']) < int(row['range_max']):
            raise SystemExit(f'{path}: {name} has an empty range')
    rad = arm._to_radians
    arm.calibration_open()
    try:
        problems, now = [], {}
        for motor in arm.motors:
            row = rows[motor.name]
            homing = arm._homing(motor)
            if homing != int(row['homing_offset']):
                problems.append(f'{motor.name}: the servo holds homing offset {homing}, the file says {row["homing_offset"]}')
        counts = {name: round(value / (2 * 3.141592653589793) * arm.counts_per_turn) % arm.counts_per_turn
                  for name, value in arm.calibration_read().items()}
        for motor in arm.motors:
            row = rows[motor.name]
            if not int(row['range_min']) - LEROBOT_SLACK <= counts[motor.name] <= int(row['range_max']) + LEROBOT_SLACK:
                problems.append(f'{motor.name} reads {counts[motor.name]}, outside the file\'s range '
                                f'[{row["range_min"]}, {row["range_max"]}]')
        if problems:
            raise SystemExit(f'{path} does not match this arm, nothing was saved: ' + '; '.join(problems))
        jaw = rows[arm.gripper.name]
        ends = (int(jaw['range_min']), int(jaw['range_max']))
        closed_end = ends[0]
        if ask is not None:
            ask('Close the gripper fully (by hand, or leave it closed), then press Enter. ')
            reading = round(arm.calibration_read()[arm.gripper.name] / (2 * 3.141592653589793) * arm.counts_per_turn) \
                % arm.counts_per_turn
            span = ends[1] - ends[0]
            if min(abs(reading - e) for e in ends) > 0.3 * span:
                raise SystemExit(f'the gripper reads {reading}, not near either end of [{ends[0]}, {ends[1]}]: nothing '
                                 'was saved; close it fully and run it again')
            closed_end = min(ends, key=lambda e: abs(reading - e))
    finally:
        arm.calibration_close()
    open_end = ends[1] if closed_end == ends[0] else ends[0]
    joints = {}
    for motor in [m for m in arm.joints if m is not None] + arm.held:
        row = rows[motor.name]
        lo, hi = sorted((motor.sign * rad(int(row['range_min']) - LEROBOT_MID), motor.sign * rad(int(row['range_max']) - LEROBOT_MID)))
        joints[motor.name] = dict(zero=rad(LEROBOT_MID), lo=lo - MARGIN_RAD, hi=hi + MARGIN_RAD)
    data = dict(version=1, arm=arm.arm_id, address=arm.bus, recorded_at=time.time(), mode=arm.calibration,
                motors={m.name: m.id for m in arm.motors}, gripper=dict(closed=rad(closed_end), open=rad(open_end)),
                facts=dict(homing={m.name: int(rows[m.name]['homing_offset']) for m in arm.motors}, source='lerobot',
                           lerobot_file=str(path)), joints=joints)
    target = save(arm.arm_id, arm.bus, data)
    say(f'saved {target} from {path}; the servos were only read' + ('' if ask is not None else
        ' (the gripper\'s range_min is taken as closed)'))
    return target


def offer_lerobot(arm, yes, ask=None, say=print):
    """The guided setup: when LeRobot calibration files fit this arm, offer to use one. True once imported."""
    if getattr(arm, 'calibration', None) != 'required' or not hasattr(arm, '_homing'):
        return False
    for path, _ in lerobot_files(arm):
        if not yes(f'LeRobot already calibrated an arm with these motors ({path}). Use that calibration? The servos are '
                   'only read, never changed', True):
            continue
        try:
            import_lerobot(arm, path, ask=ask, say=say)
            return True
        except SystemExit as exc:
            say(str(exc.code))
    return False


def cmd_import_lerobot(arm, choice):
    files = [(Path(choice), None)] if choice not in (None, 'auto') else lerobot_files(arm)
    if not files:
        raise SystemExit('no LeRobot calibration file fits this arm in ' + ', '.join(map(str, lerobot_dirs()))
                         + ': name it, roborun hw calibrate ARM@ADDRESS --from-lerobot FILE')
    terminal = sys.stdin.isatty()
    if len(files) > 1:
        if not terminal:
            raise SystemExit('several LeRobot calibrations fit this arm; name one with --from-lerobot FILE: '
                             + ', '.join(str(p) for p, _ in files))
        for n, (p, _) in enumerate(files, 1):
            print(f'  {n}. {p}')
        files = [files[int(input('Which one? ')) - 1]]
    return import_lerobot(arm, files[0][0], ask=input if terminal else None)


def _sweep_jaw(arm):
    if sys.stdin.isatty():
        input('Only the jaw motor gets power: the jaw closes and opens fully, the rest of the arm is not touched. '
              'Keep the jaws empty, then press Enter. ')
    print(json.dumps(arm.calibrate_gripper(), indent=1))


def cmd_gripper(a) -> int:
    from . import open_arm, parse_spec
    from .safety import ArmsBusy, ArmsLock
    name, address = parse_spec(a.arm)
    if not address:
        raise SystemExit('name the address too: roborun hw calibrate --gripper ARM@ADDRESS')
    try:
        with ArmsLock('calibrate-gripper', [address]):
            arm = open_arm(name, a.side, address, connect=False)
            if not hasattr(arm, 'calibrate_gripper'):
                raise SystemExit(f'{name} has no motor-driven gripper sweep: use roborun hw calibrate {a.arm}')
            _sweep_jaw(arm)
    except ArmsBusy as exc:
        raise SystemExit(str(exc))
    return 0


def cmd_calibrate(a) -> int:
    from . import open_arm, parse_spec
    name, address = parse_spec(a.arm)
    if not address:
        raise SystemExit('name the address too: roborun hw calibrate ARM@ADDRESS')
    from .safety import ArmsBusy, ArmsLock
    try:
        with ArmsLock('arm-calibrate', [address]):
            arm = open_arm(name, a.side, address, connect=False)
            if getattr(a, 'from_lerobot', None):
                cmd_import_lerobot(arm, a.from_lerobot)
            elif hasattr(arm, 'calibrate_gripper'):           # a YAM: its joints need nothing, its jaw sweeps its own stroke
                _sweep_jaw(arm)
            else:
                record(arm)
    except ArmsBusy as exc:
        raise SystemExit(str(exc))
    return 0


def add_parser(sub):
    c = sub.add_parser('arm-calibrate', help='record an arm\'s zero, ranges and gripper by hand, with torque off')
    c.add_argument('arm', help='ARM@ADDRESS, e.g. so101@serial://SERIAL')
    c.add_argument('--side', default='left')
    c.add_argument('--from-lerobot', nargs='?', const='auto', metavar='FILE',
                   help="use LeRobot's calibration of this arm (found, or this file): the servos are only read")
    c.add_argument('--gripper', action='store_true', help='YAM arms: only the jaw moves, to find its two ends, once (jaws empty)')
    c.set_defaults(func=lambda a: cmd_gripper(a) if a.gripper else cmd_calibrate(a))
    g = sub.add_parser('calibrate-gripper', help='YAM arms: the jaw finds its two ends, once (jaws empty; only the jaw moves)')
    g.add_argument('arm', help='ARM@ADDRESS, e.g. yam@slcan://SERIAL')
    g.add_argument('--side', default='left')
    g.set_defaults(func=cmd_gripper)
