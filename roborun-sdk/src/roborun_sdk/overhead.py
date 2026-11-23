"""The overhead camera's pose, as a person sees it: `roborun up`, `roborun status` and the guided setup read it from the robot.

The robot finds the pose by itself after connecting, from its own arm and table seen by the overhead depth camera
(no targets, nothing to print). Everything here only reads, except improve(), which moves the arm only after a yes.
"""
from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

STATION = 'http://127.0.0.1:8091'
MIN_TOOL_MOVE_M = .20           # the runtime confirms a pose when the gripper's spread is 8 cm: half this, with a margin
CLEARANCE_M = .12               # from the other arm, measured between link origins
SETTLED = ('calibrated', 'rough', 'needs_attention')
LABEL = dict(calibrated='calibrated', rough='found, not confirmed', needs_attention='NEEDS ATTENTION', calibrating='calibrating', waiting='waiting', starting='starting')


def read(backend, timeout=3):
    """The robot's overhead-camera state, or None when this robot does not report one (the simulator, a lab tree)."""
    with urllib.request.urlopen(backend.rstrip('/') + '/v1/status', timeout=timeout) as r:
        return json.load(r).get('overhead_camera')


def describe(state):
    """One line for `roborun status`."""
    if not state:
        return 'not reported'
    return f"{LABEL.get(state.get('state'), state.get('state'))}: {state.get('message')}"


def check(backend, wait_s=45, clock=time.monotonic, sleep=time.sleep):
    """Waits until the robot has a verdict on its overhead camera; returns the state (or the last one seen, None if there is none).

    The hook for `roborun up` and the guided setup: nothing moves."""
    end = clock() + wait_s; state = None
    while True:
        try:
            state = read(backend)
        except Exception:
            pass
        else:
            if state is None or state.get('state') in SETTLED:
                return state
        if clock() >= end:
            return state
        sleep(1)


# -- a second arm pose --------------------------------------------------------------------------------------------------
class _Model:
    """The arm's model on the table: where its tool and bodies are for given joints."""

    def __init__(self, xml, base, joints):
        import mujoco
        import numpy as np
        self.mj, self.np, self.joints = mujoco, np, joints
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data = mujoco.MjData(self.model)
        self.site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'grasp_site')
        self.base = np.array(base, float)

    def at(self, q):
        """(tool xyz, body xyz rows, touching pairs of links that are not neighbours), all on the table."""
        self.data.qpos[:self.joints] = q
        self.mj.mj_forward(self.model, self.data)
        R, t = self.base[:3, :3], self.base[:3, 3]
        bodies = self.data.xpos[1:] @ R.T + t
        parent, owner, touching = self.model.body_parentid, self.model.geom_bodyid, set()
        for contact in self.data.contact[:self.data.ncon]:
            a, b = sorted((int(owner[contact.geom1]), int(owner[contact.geom2])))
            if a and parent[b] != a and parent[a] != b and parent[a] != parent[b]:
                touching.add((a, b))
        return R @ self.data.site_xpos[self.site] + t, bodies, touching


def needed_travel(slot):
    """How far the tool has to go between two poses: the runtime wants a spread (half the distance, for two poses) of 8 cm or
    15 % of the arm's reach, whichever is less. A margin on top."""
    try:
        from .arm_profiles import info
        return min(MIN_TOOL_MOVE_M, 2.4 * .15 * float(info(slot['variant'])['reach_m']))
    except (OSError, KeyError, ValueError, TypeError):
        return MIN_TOOL_MOVE_M


def choose_pose(q, limits, model, need_m=MIN_TOOL_MOVE_M, avoid=None):
    """The nearest pose from `q` (where the arm is now, Home or not) that takes the tool `need_m` across the table: inside the
    joint limits, higher than the table all the way, with no new contact and clear of the other arm. None when there is none."""
    np = model.np
    q = np.array(q, float)
    margin = .08
    tool0, bodies0, contacts0 = model.at(q)
    floor = np.minimum(bodies0[:, 2], .04)
    candidates = []
    for yaw in np.arange(.2, 1.11, .1):
        for sign in (1, -1):
            for lift in (0., -.15, .15, -.3, .3):
                for reach in (0., -.15, .15, -.3, .3):
                    d = np.zeros(len(q)); d[0] = sign * yaw
                    if len(q) > 2:
                        d[1], d[2] = lift, reach
                    candidates.append((float(np.abs(d).max() + .3 * (abs(lift) + abs(reach))), d))
    for _, d in sorted(candidates, key=lambda row: row[0]):
        target = q + d
        if any(step and not lo + margin <= v <= hi - margin for step, v, (lo, hi) in zip(d, target, limits)):
            continue                                       # a joint that stays where it is may sit at its limit
        tool, _, _ = model.at(target)
        if float(np.hypot(*(tool - tool0)[:2])) < need_m or tool[2] < min(tool0[2], .08):
            continue
        fine = True
        for u in np.linspace(.125, 1, 8):
            _, bodies, contacts = model.at(q + u * d)
            if contacts - contacts0 or (bodies[:, 2] < floor - 1e-3).any():
                fine = False
                break
            if avoid is not None and len(avoid) and np.linalg.norm(bodies[:, None, :] - avoid[None, :, :], axis=2).min() < CLEARANCE_M:
                fine = False
                break
        if fine:
            return [float(v) for v in target], float(np.hypot(*(tool - tool0)[:2]))
    return None


class Station:
    """The robot's arm station on this computer, for the two supervised moves below."""

    def __init__(self, url=STATION):
        self.url = url.rstrip('/')

    def post(self, route, body=None, timeout=5):
        request = urllib.request.Request(self.url + route, data=json.dumps(body or {}).encode(), method='POST',
                                         headers={'Content-Type': 'application/json', 'X-Manifest-Supervisor': '1'})
        try:
            with urllib.request.urlopen(request, timeout=timeout) as reply:
                return json.load(reply)
        except urllib.error.HTTPError as exc:
            try:
                why = json.loads(exc.read()).get('error')
            except ValueError:
                why = None
            raise Refused(str(why or f'HTTP {exc.code}').replace('ValueError: ', '')) from None

    def lease(self, kind, seconds, **reference):
        seen = self.post('/supervisor/observe')
        reply = self.post('/supervisor/action', dict(observation_id=seen['observation_id'], seconds=seconds, reference=dict(kind=kind, **reference)))
        return reply['lease']['id']

    def beat(self, lease):
        return self.post('/supervisor/heartbeat', dict(lease_id=lease))['lease']

    def end(self, lease):
        try:
            self.post('/supervisor/stop', dict(lease_id=lease))
        except Exception:
            self.post('/supervisor/hold', dict(source='guided setup'))


class Refused(RuntimeError):
    pass


def plan(station, rig_file, models_dir, arm='left'):
    """{arm, q, target, moved_m, need_m, model} for this arm's second pose, from where it is now. target None: no safe one."""
    import numpy as np
    seen = station.post('/supervisor/observe')
    rig = json.loads(Path(rig_file).read_text())['arms']
    slot = rig[arm]
    q = [float(v) for v in seen['measured'][arm]['positions_rad']]
    model = _Model(Path(models_dir) / slot['model'], slot['T_table_base'], len(q))
    need = needed_travel(slot)
    other, avoid = 'right' if arm == 'left' else 'left', None
    row = (seen.get('measured') or {}).get(other)
    if row and isinstance(rig.get(other), dict) and not isinstance(rig[other].get('parked'), dict):
        them = _Model(Path(models_dir) / rig[other]['model'], rig[other]['T_table_base'], len(row['positions_rad']))
        avoid = np.array(them.at([float(v) for v in row['positions_rad']])[1])
    found = choose_pose(q, seen['limits'][arm], model, need, avoid)
    return dict(arm=arm, q=q, target=found and found[0], moved_m=found and found[1], need_m=need, model=model)


def move_to(station, step, sleep=time.sleep):
    """The move itself: one minimum-jerk curve, run by the station, which stops by itself if this program goes away.
    Returns None on arrival, else why it ended. Ctrl-C and Stop both leave the arm holding where it is."""
    travel = max(abs(a - b) for a, b in zip(step['q'], step['target']))
    seconds = min(12.0, max(2.0, 1.875 * travel / .2) + 1.5)
    lease = station.lease('setup_pose', seconds, arm=step['arm'], target_rad=step['target'])
    try:
        while True:
            sleep(.25)
            row = station.beat(lease)
            if not row.get('active'):
                return None if row.get('reason') == 'action_goal_reached' else str(row.get('reason'))
    except BaseException:
        station.end(lease)
        raise


def hand_move(station, step, ask, say, sleep=time.sleep):
    """The arm goes limp while the person moves it. Returns how far the tool went."""
    import numpy as np
    done = threading.Event()
    lease = station.lease('hand_guide', 170.0, arm=step['arm'])

    def keep_alive():
        while not done.wait(.5):
            try:
                station.beat(lease)
            except Exception:
                return

    threading.Thread(target=keep_alive, daemon=True).start()
    try:
        ask(f"Move the gripper about {step['need_m'] * 100:.0f} cm sideways over the table, keep it in the overhead camera's view, "
            'hold it still, then press Enter. ')
    finally:
        done.set()
        station.end(lease)
    sleep(.5)
    now = station.post('/supervisor/observe')['measured'][step['arm']]['positions_rad']
    return float(np.hypot(*(step['model'].at(now)[0] - step['model'].at(step['q'])[0])[:2]))


def _latest(backend, state):
    try:
        return read(backend) or state
    except Exception:
        return state


MAX_MOVES = 3              # the robot's own moves for one setup: a first second pose that agrees but cannot settle the heading gets more
STILL_WORKING = ('calibrating', 'starting', 'waiting')
EXTRA_WAIT_S = 180          # a fit over several poses can take minutes on a busy computer: wait for its verdict, not past it


def _settle(backend, wait_s, sleep, clock=time.monotonic):
    """The camera's state once it has a verdict on the new pose: calibrated, or after wait_s. A camera still working on the fit
    at that point gets up to EXTRA_WAIT_S more, so setup never reports 'not confirmed' while the answer is still coming."""
    state, end, extended = None, clock() + wait_s, False
    while True:
        if clock() >= end:
            if extended or not (state and state.get('state') in STILL_WORKING):
                return state
            end, extended = clock() + EXTRA_WAIT_S, True
        sleep(2)
        try:
            state = read(backend) or state
        except Exception:
            continue
        if state and state.get('state') == 'calibrated':
            return state


def second_pose_helps(state):
    """A rough pose, or a picture where more than one place fits the arm: a second, clearly different arm pose settles both."""
    state = state or {}
    return state.get('state') == 'rough' or (state.get('state') == 'needs_attention' and state.get('code') == 'ambiguous')


def improve(backend, confirm, *, station=None, rig_file=None, models_dir=None, ask=None, moves_allowed=True, wait_s=90, say=print, sleep=time.sleep):
    """A rough pose made good by showing the camera a second, clearly different arm pose. Without a yes nothing moves.

    The pose is chosen from where the arm is now, so an arm already at Home moves too. confirm: question -> bool. ask (a
    prompt that waits for Enter) offers moving the arm by hand instead. moves_allowed=False: only by hand."""
    state = check(backend, wait_s=30, sleep=sleep)
    if not second_pose_helps(state):
        return state
    station = station or Station()
    arm = state.get('arm') if state.get('arm') in ('left', 'right') else 'left'
    try:
        step = plan(station, rig_file, models_dir, arm)
    except Exception as exc:
        say(f'The arm cannot be asked to move from here ({exc}).')
        step = None
    moved = robot_moved = False
    if moves_allowed and step and step['target']:
        degrees = max(abs(a - b) for a, b in zip(step['q'], step['target'])) * 57.3
        question = (f"The {arm} arm will make one slow move (up to {MAX_MOVES} if the camera needs more): up to {degrees:.0f} degrees "
                    f"at a joint, the gripper about {step['moved_m'] * 100:.0f} cm across the table. Is the space around it clear, "
                    "and are its jaws empty?")
        if confirm(question):
            try:
                why = move_to(station, step, sleep=sleep)
            except Refused as exc:
                why = str(exc)
            if why:
                say(f'The move did not finish ({why}). The arm is holding where it stopped.')
                return _latest(backend, state)
            moved = robot_moved = True
    elif moves_allowed and step:
        say('No safe pose was found from where the arm is now.')
    question = f'Move the {arm} arm by hand instead? Hold it first: it goes limp until you press Enter.'
    while not moved and step and ask is not None and confirm(question):
        try:
            went = hand_move(station, step, ask, say, sleep=sleep)
        except Refused as exc:
            say(f'That did not work ({exc}).')
            return _latest(backend, state)
        moved = went >= step['need_m'] * .8
        if not moved:
            say(f"The gripper moved {went * 100:.0f} cm, less than the {step['need_m'] * 100:.0f} cm the camera needs.")
            question = f'Try again? Hold the {arm} arm first: it goes limp until you press Enter.'
    if not moved:
        say('Nothing was confirmed. Tasks can preview, and will not run, until the arm has been seen in a second pose. '
            'To try again: roborun setup')
        return _latest(backend, state)
    say('Checking the second pose with the overhead camera...')
    state = _settle(backend, wait_s, sleep) or state
    moves = 1
    while (moves_allowed and step and step.get('target') and moves < MAX_MOVES and state.get('state') == 'rough'
           and robot_moved):                              # the poses agree but are not enough yet: one more, further on
        say('overhead camera: ' + describe(state))
        try:
            step = plan(station, rig_file, models_dir, arm)
        except Exception:
            break
        if not step['target']:
            break
        say(f"One more slow move, the gripper about {step['moved_m'] * 100:.0f} cm across the table...")
        try:
            why = move_to(station, step, sleep=sleep)
        except Refused as exc:
            why = str(exc)
        if why:
            say(f'The move did not finish ({why}). The arm is holding where it stopped.')
            break
        moves += 1
        state = _settle(backend, wait_s, sleep) or state
    say('overhead camera: ' + describe(state))
    return state
