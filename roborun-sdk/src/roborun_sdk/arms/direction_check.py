"""`roborun hw check ARM@ADDRESS --move`: does each joint move the way its command says?

For a first run on real hardware. One joint at a time, after the operator confirms that joint: a tiny move away from the
nearer limit at low speed, then back. Measured and commanded directions are compared. The first wrong answer ends the
check, because the next joint's move could not be trusted either. Anything unexpected stops it too: the arm is left holding
where it is, and is closed the normal way.

Commands and readings go through the same sign in the arm's table, so a wrong sign there moves both together and the
numbers still agree. With the arm's model the prompt therefore says where the tool should go: the operator's eyes are
the check on that.
"""
import json
import math
import os
import re
import sys
import time
from pathlib import Path

STEP_RAD = 0.05                     # about 3 degrees
JAW_STEP = 0.08                     # of the stroke
SPEED_RAD_S = 0.1
JAW_SPEED = 0.1
ACCEL_RAD_S2 = 0.5
JERK_RAD_S3 = 5.0
MOVE_TIMEOUT_S = 6.0
COMMAND_HZ = 30.0
MIN_SHARE = 0.5                     # of the commanded step, or the joint did not really follow
OTHER_JOINT_RAD = 0.03              # any other joint moving this much stops the check
PAUSE_S = 1.0                       # in one confirmed run: each joint is announced this long before it moves


def _result_file(arm_id, address):
    home = Path(os.environ.get('ROBORUN_HOME', Path.home() / '.config' / 'roborun'))
    return home / 'checks' / (re.sub(r'[^A-Za-z0-9._-]+', '_', f'{arm_id}@{address}').strip('_') + '.json')


def load_result(arm_id, address):
    """What is recorded about this arm's movement check, or None when it has never been run or skipped."""
    try:
        return json.loads(_result_file(arm_id, address).read_text())
    except (OSError, ValueError):
        return None


def save_result(arm_id, address, result, **details):
    """result: passed, partial (joints skipped), failed, stopped or skipped."""
    target = _result_file(arm_id, address)
    target.parent.mkdir(parents=True, exist_ok=True)
    row = dict(load_result(arm_id, address) or {}, arm=arm_id, address=address, result=result, at=time.time(), **details)
    temp = target.with_suffix('.tmp')
    temp.write_text(json.dumps(row, indent=1))
    temp.replace(target)
    return row


def result_of(report):
    """The word recorded for a finished run()."""
    if not report['ok']:
        return 'failed' if any(r['result'] != 'ok' and not r['result'].startswith('skipped') for r in report['joints']) else 'stopped'
    return 'partial' if any(r['result'].startswith('skipped') for r in report['joints']) or not report['joints'] else 'passed'


def _slow_down(arm):
    for name, value in (('joint_speed_rad_s', SPEED_RAD_S), ('joint_accel_rad_s2', ACCEL_RAD_S2),
                        ('joint_jerk_rad_s3', JERK_RAD_S3), ('jaw_speed', JAW_SPEED)):
        setattr(arm, name, min(getattr(arm, name, value), value))


def _drive(arm, q, jaw, arrived):
    """Stream one target until `arrived(sample)` or the timeout; returns the last sample."""
    end = time.monotonic() + MOVE_TIMEOUT_S
    while True:
        arm.command(q, jaw)
        sample = arm.read()
        if arrived(sample) or time.monotonic() > end:
            return sample
        time.sleep(1 / COMMAND_HZ)


def _settle(arm, q, jaw, seconds=0.4):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        arm.command(q, jaw)
        time.sleep(1 / COMMAND_HZ)
    return arm.read()


def _verdict(moved, step):
    if abs(moved) < abs(step) * MIN_SHARE:
        return 'did not move' if abs(moved) < abs(step) * 0.1 else 'moved too little'
    return 'ok' if moved * step > 0 else 'reversed'


def _step_for(arm, index, start):
    lo, hi = arm.limits[index]
    return STEP_RAD if hi - start[index] >= start[index] - lo else -STEP_RAD


def _tool_hint(arm, index, start):
    """Where the model says the tool goes, in words; empty without a model."""
    tool = getattr(arm, '_tool', None)
    if tool is None:
        return ''
    target = list(start)
    target[index] += _step_for(arm, index, start)
    delta = [(a - b) * 1000 for a, b in zip(tool.position(target), tool.position(start))]
    axis = max(range(3), key=lambda i: abs(delta[i]))
    if abs(delta[axis]) < 1:
        return ' The tool should barely move (it turns in place).'
    return f" The tool should go about {abs(delta[axis]):.0f} mm toward {'+' if delta[axis] > 0 else '-'}{'xyz'[axis]} of the arm's base."


def _check_joint(arm, index, start, jaw):
    lo, hi = arm.limits[index]
    step = _step_for(arm, index, start)
    row = dict(joint=index + 1, start_rad=round(start[index], 4), commanded_rad=step)
    if not (lo <= start[index] + step <= hi):
        return dict(row, result='skipped: no room inside its limits')
    target = list(start)
    target[index] += step
    sample = _drive(arm, target, jaw, lambda s: abs(s['positions_rad'][index] - start[index]) >= abs(step) * 0.9)
    sample = _settle(arm, target, jaw)
    q = sample['positions_rad']
    moved = q[index] - start[index]
    others = max((abs(a - b) for i, (a, b) in enumerate(zip(q, start)) if i != index), default=0.0)
    _drive(arm, start, jaw, lambda s: abs(s['positions_rad'][index] - start[index]) < 0.01)
    _settle(arm, start, jaw)
    row.update(measured_rad=round(moved, 4), result=_verdict(moved, step))
    if others > OTHER_JOINT_RAD:
        row['result'] = f'another joint moved {others:.3f} rad'
    return row


def _check_jaw(arm, q, jaw):
    step = JAW_STEP if jaw <= 0.5 else -JAW_STEP
    row = dict(joint='gripper', start=round(jaw, 3), commanded=step)
    sample = _drive(arm, q, jaw + step, lambda s: abs(s['gripper'] - jaw) >= abs(step) * 0.9)
    moved = _settle(arm, q, jaw + step)['gripper'] - jaw
    _drive(arm, q, jaw, lambda s: abs(s['gripper'] - jaw) < 0.01)
    return dict(row, measured=round(moved, 3), result=_verdict(moved, step))


def _ask(ask, text):
    answer = ask(text).strip().lower()
    return 'yes' if answer in ('y', 'yes') else 'skip' if answer in ('s', 'skip') else 'quit'


def run(arm, ask=None, say=print, confirm=None, finish=None, confirmed=False, pause_s=PAUSE_S) -> dict:
    """Interactive unless `ask` is given. The joints are checked in order; nothing moves without a yes for that joint.

    `confirm`, when given, is asked after each joint that measured correctly: the operator's eyes are the only check on a
    wrong sign, so anything but a yes ends the run. `finish` replaces arm.close() at the end, however the run ended.

    `confirmed`: the caller already had the operator's yes to power the arm and move it (the guided setup asks once, with
    the arm supported and the area clear). Then every joint and the gripper run in sequence, each announced `pause_s`
    before it moves, with no question in between; Ctrl-C stops it (KeyboardInterrupt ends the run through `finish`, as
    at any question), and a joint measured wrong still ends it at once. The operator's eyes are asked once, afterwards, by
    the caller."""
    if confirmed:
        ask = confirm = None
    elif ask is None:
        if not sys.stdin.isatty():
            raise SystemExit('arm-check --move asks before every joint, so it needs a terminal')
        ask = input
    out = dict(ok=False, joints=[], stopped=None)
    say = (lambda *_: None) if say is None else say
    try:
        survey = arm.survey()
    except NotImplementedError:
        survey = None
    if survey is not None and not survey.get('ok', True):
        out['stopped'] = 'the read-only check found problems: ' + '; '.join(map(str, survey.get('problems') or [survey.get('error')]))
        return out
    if confirmed:
        say('Powering the arm. Each joint moves about 3 degrees, slowly, and back, one after the other, then the gripper.')
        say('Ctrl-C if anything moves wrong.')
    else:
        say('This powers the arm and moves one joint at a time by about 3 degrees, slowly, and back.\n'
            'Keep a hand on the power switch, keep people and objects clear, and support the arm if it could fall.')
    if not confirmed and _ask(ask, 'Power the arm now? Type y to continue: ') != 'yes':
        out['stopped'] = 'not confirmed'
        return out
    _slow_down(arm)
    arm.connect()
    try:
        first = arm.read()
        start = [float(v) for v in first['positions_rad']]
        jaw = float(first['gripper'])
        arm.command(start, jaw)
        time.sleep(max(0.3, getattr(arm, 'ramp_s', 0.0) + 0.2))    # stiffness has come in before anything moves
        for index in range(len(start)):
            if confirmed:
                say(f'Joint {index + 1} of {len(start)}: about {math.degrees(STEP_RAD):.0f} degrees and back.{_tool_hint(arm, index, start)}')
                time.sleep(pause_s)
                answer = 'yes'
            else:
                answer = _ask(ask, f'Joint {index + 1} of {len(start)}: it will move about {math.degrees(STEP_RAD):.0f} degrees and back.'
                                   f'{_tool_hint(arm, index, start)} Clear? [y]es, [s]kip, anything else stops: ')
            if answer == 'quit':
                out['stopped'] = f'stopped by you before joint {index + 1}'
                return out
            if answer == 'skip':
                out['joints'].append(dict(joint=index + 1, result='skipped by you'))
                continue
            row = _check_joint(arm, index, start, jaw)
            out['joints'].append(row)
            say(f"joint {index + 1}: {row['result']}" + (f" (commanded {row['commanded_rad']:+.3f}, measured {row['measured_rad']:+.3f} rad)" if 'measured_rad' in row else ''))
            if row['result'] != 'ok' and not row['result'].startswith('skipped'):
                out['stopped'] = f"joint {index + 1}: {row['result']}. Check its sign and zero in the arm's table before going on"
                return out
            if confirm is not None and row['result'] == 'ok' and _ask(confirm, f'Did joint {index + 1} move, and only joint {index + 1}, '
                                                                               'as described, and come back? Type y if you are sure: ') != 'yes':
                row['result'] = 'not confirmed by you'
                out['stopped'] = f'joint {index + 1}: you were not sure it moved correctly'
                return out
        if confirmed and first.get('gripper_available') is not False:
            say('Gripper: it moves a little and back.')
            time.sleep(pause_s)
        if first.get('gripper_available') is not False and (confirmed or _ask(ask, 'Gripper: it will move a little and back. Jaws clear? [y]es, [s]kip: ') == 'yes'):
            row = _check_jaw(arm, start, jaw)
            out['joints'].append(row)
            say(f"gripper: {row['result']}")
            if row['result'] != 'ok':
                out['stopped'] = f"gripper: {row['result']}"
                return out
        out['ok'] = all(r['result'] == 'ok' or r['result'].startswith('skipped') for r in out['joints'])
        return out
    except Exception as exc:
        out['stopped'] = f'{type(exc).__name__}: {exc}'[:300]
        try:
            arm.hold()
        except Exception:
            pass
        return out
    finally:
        (finish or arm.close)()
