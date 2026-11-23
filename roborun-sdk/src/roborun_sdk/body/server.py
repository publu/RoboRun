"""The local robot API for a humanoid profile (127.0.0.1:8120, what `roborun up --profile booster_k1|galaxea_r1` starts in place
of the arm runtime). Programs on this computer, such as your LLM's tools, drive the body here; nothing reaches it from outside.

    GET  /v1/status                  ready, profile, simulated, the running move (if any), base and head state
    GET  /v1/body                    the profile: label, robot descriptor, head camera, limits, the arms this API drives (arms_offered)
    GET  /v1/body/state              base odometry and velocity, head pitch and yaw
    GET  /v1/body/frame.jpg          the head camera now (JPEG; not face-blurred yet)
    GET  /v1/body/model              the body's 3-D display model (its body tree, meshes, head joints, head camera; the simulated
                                     room when simulated), posed by the page from /v1/body/state
    POST /v1/body/move               {prompt, duration_s (2|3|4|6|8, default 4)} -> 202 {move}
    GET  /v1/body/moves/<id>         that move: stage, status, what the reply said, what was played
    POST /v1/stop  (or /api/stop)    Stop now: the base stops, the head holds, a move being made is cancelled
    GET  /v1/robot-profiles          the profiles this robot can run (switch: roborun up --profile <id>)
    GET  /v1/tasks                   the moves, newest first
    GET  /v1/health                  is it up;  GET /v1/stop/status (or /api/stop/status): is the body still moving
    GET  /v1/docs, /v1/openapi.json  this list, for people and for code generators

Errors are {"error": {"code", "message", "retryable"}}, as on an arm profile's local API.

A move: the head frame now (not face-blurred yet) -> the hosted API (a body job) -> the signed 10 Hz timeline, checked ->
played on the base and the head at 50 Hz. One move at a time (409 while one runs). Stop works at every stage.
"""
from __future__ import annotations

import json
import os
import re
import signal
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from . import profiles
from .jobs import BodyJobError, BodyJobs, build_request
from .player import Watch, planned_end, play, settle, stop_reason
from .record import MoveRecorder

DOOR_HEADER = 'X-Roborun-Door'
MAX_BODY = 1 << 20
KEEP_MOVES = 50
MOVE_FIELDS = {'prompt', 'duration_s', 'resolution'}          # resolution: accepted from older programs and ignored (the service picks it)
# the stages a move record shows: the service's own working steps all read 'planning'
PUBLIC_STAGES = ('capturing', 'uploading', 'queued', 'reconnecting', 'planning', 'moving', 'done', 'stopped', 'failed')
# the steps a hosted job reports that the record shows as they are; every other one (its own steps, and its 'done' before
# the robot has moved) reads 'planning' until the robot moves
JOB_STAGES = ('uploading', 'queued', 'reconnecting')
MOVE_SCHEMA = {'type': 'object', 'properties': {
    'id': {'type': 'string'}, 'prompt': {'type': 'string'}, 'duration_s': {'type': 'integer'},
    'status': {'type': 'string', 'enum': ['running', 'done', 'stopped', 'failed']},
    'stage': {'type': 'string', 'enum': list(PUBLIC_STAGES)}, 'job': {'type': 'string'}, 'verified': {'type': 'boolean'},
    'rows': {'type': 'integer', 'description': 'timeline rows, 10 a second; the body may play longer than duration_s'},
    'summary': {'type': 'object'}, 'warnings': {'type': 'array'},
    'deferred': {'type': 'array', 'items': {'type': 'string'}, 'description': 'parts of the prompt left for a next move'},
    'outcome': {'type': 'object', 'description': "what this move did of the prompt: status 'done' (all of it), 'partial' (its first "
                "step; remaining are the next moves, in order) or 'incomplete' (missed says what it could not do); did"},
    'played': {'type': 'object', 'description': 'played_s, stopped, odometry_start, odometry_end, settle {settled, error_m, error_deg}'},
    'moved': {'type': 'object', 'description': 'what the body measured it did: distance_m, forward_m, left_m, turn_deg'},
    'recording': {'type': 'object', 'description': "this move as an episode of the robot's LeRobot dataset: recorded, frames, fps, or why not"},
    'error': {'type': 'object', 'properties': {'code': {'type': 'string'}, 'message': {'type': 'string'}}},
    'created_at': {'type': 'number'}, 'finished_at': {'type': 'number'}}}
STOP_SCHEMA = {'type': 'object', 'properties': {
    'stopped': {'type': 'boolean'}, 'state': {'type': 'string', 'enum': ['stopping', 'not_confirmed']}, 'confirmed': {'type': 'boolean'},
    'stopped_tasks': {'type': 'array', 'items': {'type': 'string'}},
    'move': {'type': ['string', 'null']}, 'source': {'type': 'string'}, 'message': {'type': 'string'}}}
STILL_M_S, STILL_RAD_S = .01, .02            # slower than this, the base is at rest
RETRYABLE = {'busy', 'robot_error', 'unavailable'}         # the same codes as an arm robot's API: invalid, not_found, busy, ...


class BodyRobot:
    """The body behind the local API: its adapters, the hosted API client and the moves."""

    def __init__(self, profile_id, base, head, jobs=None, *, robot='robot', simulated=None, close=None):
        self.profile_id, self.profile = profile_id, profiles.load(profile_id)
        self.base, self.head, self.jobs = base, head, jobs
        self.robot, self.simulated = robot, simulated            # simulated: the scene name, or None for a real body
        self.record = True                                       # each move -> one episode of this robot's dataset (if installed)
        self._close = close
        self.lock = threading.Lock()
        self.moves = {}                                          # id -> record, newest last
        self.current = None                                      # (record, stop event)

    # ------------------------------------------------------------------------------------------------ reading
    def state(self):
        out = dict(base=self.base.state(), head=self.head.state())
        return out

    def status(self):
        cur = self.current
        try:
            st = self.state()
        except Exception as exc:                                 # a real body that stopped answering
            st = dict(error=str(exc)[:300])
        return dict(ready=True, kind='humanoid', profile=self.profile_id, label=self.profile['label'], simulated=bool(self.simulated),
                    scene=self.simulated, robot=self.robot, task=self._public(cur[0]) if cur else None, guard=self.guard_status(), **st)

    def guard_status(self):
        """Whether moves have an obstacle guard: it needs head depth (asked once, then known from each move)."""
        if getattr(self, 'guard_depth', None) is None:
            try:
                self.guard_depth = self.head.depth() is not None
            except Exception:
                self.guard_depth = False
        return dict(obstacle_guard=True) if self.guard_depth else dict(obstacle_guard=False, note=NO_GUARD)

    def describe(self):
        p = self.profile
        return dict(profile=self.profile_id, label=p['label'], maker=p['maker'], description=p['description'], robot=p['robot'],
                    camera={k: v for k, v in p['camera'].items() if k != 'note'}, simulated=bool(self.simulated),
                    move=dict(durations_s=[2, 3, 4, 6, 8]),
                    arms_offered=[],                     # the arms this local API drives: none yet (the page shows arm controls for these)
                    model='/v1/body/model')

    def model(self):
        """The 3-D display model of this body (GET /v1/body/model), with the simulated room when this is a simulation."""
        from .model import display_model
        return display_model(self.profile_id, self.simulated)

    def _public(self, rec):
        return {k: v for k, v in rec.items() if not k.startswith('_')}

    def tasks(self, limit=20):
        rows = list(self.moves.values())[::-1][:limit]
        return dict(tasks=[self._public(r) for r in rows])

    # ------------------------------------------------------------------------------------------------ moving
    def move(self, prompt, duration_s=4):
        """Starts a move in the background. (202, record) or (code, error)."""
        if self.jobs is None:
            return 503, dict(error=dict(code='no_motion_service', message='This robot has no motion service key'))
        if not isinstance(prompt, str) or not prompt.strip() or len(prompt) > 4000:
            return 400, dict(error=dict(code='invalid', message='prompt: what the robot should do, up to 4000 characters'))
        if duration_s not in (2, 3, 4, 6, 8):
            return 400, dict(error=dict(code='invalid', message='duration_s: 2, 3, 4, 6 or 8'))
        with self.lock:
            if self.current is not None:
                return 409, dict(error=dict(code='busy', message='A task is running: wait for it, or Stop it'))
            rec = dict(id='mv-' + uuid.uuid4().hex[:12], prompt=prompt, duration_s=duration_s,
                       status='running', stage='capturing', created_at=time.time())
            stop = threading.Event()
            self.current = (rec, stop)
            self.moves[rec['id']] = rec
            while len(self.moves) > KEEP_MOVES:
                self.moves.pop(next(iter(self.moves)))
        threading.Thread(target=self._run, args=(rec, stop), name='body-move', daemon=True).start()
        return 202, self._public(rec)

    def _run(self, rec, stop):
        try:
            frame = self.head.frame()
            if stop.is_set():
                raise BodyJobError('stopped', 'Stopped')
            request = build_request(self.profile_id, frame, rec['prompt'], duration_s=rec['duration_s'])
            rec['job'] = request['id']

            def said(s):
                stage = s.get('stage')
                if stage:                                    # every step of the service between queued and the reply: 'planning'
                    rec['stage'] = stage if stage in JOB_STAGES else 'planning'
                deferred = s.get('deferred')
                if isinstance(deferred, list):               # the parts of the request left for a next move
                    rec['deferred'] = [str(d)[:200] for d in deferred]
            reply = self.jobs.run(request, frame['jpeg'], on_status=said, cancelled=stop.is_set)
            loco = reply['locomotion']
            rec.update(summary=loco.get('summary'), warnings=loco.get('warnings') or [], rows=len(loco['rows']),
                       verified=True)
            if stop.is_set():
                raise BodyJobError('stopped', 'Stopped')
            rec['stage'] = 'moving'
            before = self.base.state()
            if isinstance(loco.get('outcome'), dict):           # what this move does of the prompt: done, partial or incomplete
                o = loco['outcome']
                rec['outcome'] = dict(status=str(o.get('status'))[:20], did=o.get('did'),
                                      remaining=[str(r)[:200] for r in o.get('remaining') or []],
                                      missed=[str(m)[:200] for m in o.get('missed') or []])
            watch = Watch(self.profile_id, self.base, self.head).start()      # the obstacle guard, for the move and its settle
            self.guard_depth = watch.has_depth
            recorder = MoveRecorder(self.robot, self.profile_id, self.base, self.head).start() if self.record else None
            head = _Tapped(self.head, recorder) if recorder is not None else self.head
            try:
                played = play(loco['rows'], self.base, head, self.profile_id, rate_hz=loco.get('rate_hz') or 10., stop=stop,
                              watch=watch)
                if not played['stopped'] and not stop.is_set():   # close what the body did not reach of the plan's end
                    played['settle'] = settle(planned_end(loco['rows'], loco.get('rate_hz') or 10.), before, self.base,
                                              self.profile_id, stop=stop, watch=watch)
                    played['stopped'] = played['settle'].get('why') == 'stopped'
                    played['end'] = self.base.state()
            finally:
                watch.close()
                if recorder is not None:                          # this move as one episode of the robot's dataset
                    rec['recording'] = recorder.stop(prompt=rec['prompt'], extras=dict(
                        task_id=rec['id'], recorded_at=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), mode='run',
                        outcome='stopped' if stop.is_set() else 'done', source='roborun-body-move'))
            rec['guard'] = watch.summary()
            stop_ev = played.pop('guard_stop', None) or (played.get('settle') or {}).pop('guard_stop', None)
            if stop_ev is not None:                               # the guard stopped it: stopped, and says why; it stays put
                o = rec.setdefault('outcome', dict(status='done', did=rec['prompt'], remaining=[], missed=[]))
                o['missed'] = o.get('missed', []) + [stop_reason(stop_ev, watch.has_depth)]
                o['status'] = 'incomplete'
                played['stopped'] = True
            elif 'settle' in played:
                st = played['settle']
                if not st.get('settled') and not played['stopped']:   # it did not get there: never reported as done
                    o = rec.setdefault('outcome', dict(status='done', did=rec['prompt'], remaining=[], missed=[]))
                    why = {'blocked': 'something is in the way', 'too_far': 'too far off to correct without looking',
                           'timeout': 'it did not close the gap in time'}.get(st.get('why'), str(st.get('why')))
                    o['missed'] = o.get('missed', []) + [f"ended {st.get('error_m', 0):.2f} m and {abs(st.get('error_deg', 0)):.0f}° "
                                                         f'from where the move should end ({why})']
                    o['status'] = 'incomplete'
            if not watch.has_depth:
                rec['warnings'] = list(rec.get('warnings') or []) + [NO_GUARD]
            after = played.pop('end')
            rec['played'] = dict(played, odometry_start=_pose(before), odometry_end=_pose(after))
            rec['moved'] = _moved(before, after)                 # measured on the body, after the settle: not the plan
            if played['stopped'] and stop.is_set():               # Stopped by you: whatever was planned, it did not all happen
                o = rec.setdefault('outcome', dict(status='done', did=rec['prompt'], remaining=[], missed=[]))
                if o.get('status') in ('done', 'partial'):
                    o['status'] = 'incomplete'
                    o['missed'] = o.get('missed', []) + ['stopped by you before the move ended']
            rec.update(status='stopped' if played['stopped'] else 'done', stage='stopped' if played['stopped'] else 'done')
        except BodyJobError as e:
            stopped = e.code in ('stopped', 'cancelled')
            rec.update(status='stopped' if stopped else 'failed', stage='stopped' if stopped else 'failed',
                       **({} if stopped else dict(error=dict(code=e.code, message=e.message))))
        except Exception as e:                                   # the body itself failed (a real robot that stopped answering)
            rec.update(status='failed', stage='failed', error=dict(code='robot_error', message=str(e)[:300]))
        finally:
            try:
                self.base.stop()
            except Exception:
                pass
            rec['message'] = _message(rec)                       # one plain line, as an arm task has
            rec['finished_at'] = time.time()
            with self.lock:
                if self.current is not None and self.current[0] is rec:
                    self.current = None

    def stop(self, source='api'):
        """Stop: the base stops now, a running move ends (a job still being made is cancelled), the head holds."""
        cur = self.current
        if cur is not None:
            cur[1].set()
        errors = []
        try:
            self.base.stop()
        except Exception as exc:
            errors.append(f'base: {exc}'[:200])
        confirmed = False
        if not errors and source != 'shutdown':              # the base is already braking: wait (briefly) to see it at rest
            end = time.monotonic() + STOP_CONFIRM_S
            while time.monotonic() < end:
                code, st = self.stop_status()
                if code == 200 and st.get('confirmed'):
                    confirmed = True
                    break
                time.sleep(.02)
        out = dict(stopped=not errors, source=source, move=cur[0]['id'] if cur else None,
                   stopped_tasks=[cur[0]['id']] if cur else [],             # the same key as an arm robot's Stop
                   state='stopped' if confirmed else 'stopping' if not errors else 'not_confirmed', confirmed=confirmed,
                   message='Stopped: the body is at rest and the head holds' if confirmed else
                   'The Stop was taken: the base brakes and the head holds (GET /v1/stop/status)' if not errors
                   else 'The Stop could not be confirmed: keep clear of the robot and cut its power')
        if errors:
            out['errors'] = errors
            out['error'] = dict(code='stop_not_confirmed', message=out['message'], retryable=True)
        return out

    def stop_status(self):
        """(status, body): is the body at rest? stopped (confirmed), stopping (a move still ends), running, or moving."""
        cur = self.current
        try:
            st = self.base.state()
        except Exception as exc:
            return 503, dict(state='unreachable', confirmed=False, task=None, message=f'The body does not answer ({exc})'[:300],
                             error=dict(code='unreachable', message='The body does not answer', retryable=True))
        speed = max(abs(float(st.get('vx') or 0.)), abs(float(st.get('vy') or 0.)))
        turn = abs(float(st.get('vyaw') or 0.))
        moving = speed > STILL_M_S or turn > STILL_RAD_S
        task = dict(id=cur[0]['id'], status='stopping' if cur[1].is_set() else 'running') if cur else None
        if cur is not None and cur[1].is_set():
            state, message = 'stopping', 'The move is stopping'
        elif cur is not None:
            state, message = 'running', 'A move is running'
        elif moving:
            state, message = 'stopping', 'The base is braking'
        else:
            state, message = 'stopped', 'The body is at rest'
        return 200, dict(state=state, confirmed=state == 'stopped', task=task, base_speed_m_s=round(speed, 3),
                         base_turn_rad_s=round(turn, 3), message=message)

    def close(self):
        self.stop('shutdown')
        if self._close:
            self._close()


STOP_CONFIRM_S = 2.0              # a Stop answers once the body is at rest (it brakes at once), or says it is still stopping
NO_GUARD = ('No obstacle guard: this body sends no head depth, so nothing stops it before an obstacle. It moves at most '
            '0.4 m per move, slowly; Stop always works')


class _Tapped:
    """The head, with every target it is sent also told to the move's recorder (the action it records)."""

    def __init__(self, head, recorder):
        self._head, self._recorder = head, recorder

    def command(self, pitch, yaw):
        self._recorder.head_target(pitch, yaw)
        return self._head.command(pitch, yaw)

    def __getattr__(self, name):
        return getattr(self._head, name)


def _message(rec):
    """What happened, in one line: what the body measured it did, and what is left or went wrong."""
    m, o = rec.get('moved') or {}, rec.get('outcome') or {}
    did = []
    if abs(m.get('turn_deg') or 0.) >= 3:
        did.append(f"turned {abs(m['turn_deg']):.0f}° {'left' if m['turn_deg'] > 0 else 'right'}")
    if (m.get('distance_m') or 0.) >= .05:
        did.append(f"moved {m['distance_m']:.2f} m {'forward' if (m.get('forward_m') or 0.) >= 0 else 'back'}")
    text = ('The robot ' + ' and '.join(did)) if did else 'The robot did not move'
    if rec.get('status') == 'failed':
        return (rec.get('error') or {}).get('message') or 'The task did not finish'
    if o.get('missed'):
        text += '. Did not finish: ' + '; '.join(o['missed'])
    elif o.get('remaining'):
        text += '. Still to do: ' + ', then '.join(o['remaining'])
    return text[:400] + ('' if text.endswith('.') else '.')


def _moved(a, b):
    """What the body really did, from its odometry: {distance_m, forward_m, left_m, turn_deg} in the frame it started in."""
    import math
    dx, dy = float(b.get('x', 0.)) - float(a.get('x', 0.)), float(b.get('y', 0.)) - float(a.get('y', 0.))
    c, s = math.cos(float(a.get('yaw', 0.))), math.sin(float(a.get('yaw', 0.)))
    turn = (float(b.get('yaw', 0.)) - float(a.get('yaw', 0.)) + math.pi) % (2 * math.pi) - math.pi
    return dict(distance_m=round(math.hypot(dx, dy), 3), forward_m=round(c * dx + s * dy, 3), left_m=round(-s * dx + c * dy, 3),
                turn_deg=round(math.degrees(turn), 1))


def _pose(s):
    return {k: round(float(s.get(k, 0.)), 4) for k in ('x', 'y', 'yaw')}


# ---------------------------------------------------------------------------------------------------- HTTP
def profile_listing(active_id, simulated):
    rows = []
    for r in profiles.rows():
        rows.append(dict(r, kind_of_profile='humanoid', active=r['id'] == active_id,
                         switch=dict(available=r['id'] != active_id, reason='Loaded on this robot.' if r['id'] == active_id else None,
                                     command=f"roborun up --profile {r['id']}")))
    try:
        from ..arm_profiles import models
        from ..robot import bundled
        for pid in sorted(set(bundled()) | set(models())):
            rows.append(dict(id=pid, label=pid.replace('_', ' '), kind_of_profile='arms', active=False,
                             switch=dict(available=True, reason=None, command=f'roborun up --profile {pid}')))
    except Exception:
        pass
    return dict(active_id=active_id, simulated=bool(simulated), profiles=rows, profiles_set_by='sdk')


def handler(robot):
    class H(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'
        server_version, sys_version = 'roborun', ''

        def log_message(self, *a):
            pass

        def _send(self, code, body=None, data=None, ctype='application/json'):
            data = data if data is not None else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype); self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            if self.command != 'HEAD':
                self.wfile.write(data)

        def _err(self, code, error, message, retryable=None):
            self._send(code, dict(error=dict(code=error, message=message,
                                             retryable=error in RETRYABLE if retryable is None else retryable)))

        def _sendj(self, code, out):
            """A JSON answer; an error object there gets its retryable flag."""
            err = out.get('error') if isinstance(out, dict) else None
            if isinstance(err, dict) and 'retryable' not in err:
                out = dict(out, error=dict(err, retryable=err.get('code') in RETRYABLE))
            self._send(code, out)

        def _allowed(self):
            """Browsers only through the page connector (it marks what it passes on); programs on this computer directly."""
            if self.headers.get('Origin') and not self.headers.get(DOOR_HEADER):
                self._err(403, 'browser_refused', 'Browsers reach this robot through the Roborun page only'); return False
            return True

        def _json_body(self):
            try:
                n = int(self.headers.get('Content-Length') or 0)
            except ValueError:
                n = -1
            if n < 0 or n > MAX_BODY:
                self._err(413, 'too_large', 'request body too large'); return None
            try:
                out = json.loads(self.rfile.read(n) or b'{}') if n else {}
            except ValueError:
                self._err(400, 'invalid', 'the body is not JSON'); return None
            if not isinstance(out, dict):
                self._err(400, 'invalid', 'the body must be a JSON object'); return None
            return out

        def do_GET(self):
            if not self._allowed(): return
            path, _, query = self.path.partition('?')
            try:
                if path == '/v1/health':
                    return self._send(200, dict(ok=True, profile=robot.profile_id, simulated=bool(robot.simulated), version=API_VERSION))
                if path in ('/v1/stop/status', '/api/stop/status'):
                    return self._send(*robot.stop_status())
                if path == '/v1/openapi.json':
                    return self._send(200, openapi())
                if path == '/v1/docs':
                    return self._send(200, data=DOCS_HTML.encode(), ctype='text/html; charset=utf-8')
                if path == '/v1/settings':
                    return self._send(200, dict(robot=robot.robot, profile=robot.profile_id, simulated=bool(robot.simulated)))
                if path == '/v1/status':
                    return self._send(200, robot.status())
                if path == '/v1/body':
                    return self._send(200, robot.describe())
                if path in ('/v1/body/state', '/v1/robot'):
                    return self._send(200, robot.state())
                if path in ('/v1/body/frame.jpg', '/v1/cameras/head.jpg'):
                    return self._send(200, data=robot.head.frame()['jpeg'], ctype='image/jpeg')
                if path == '/v1/body/model':
                    try:
                        out = robot.model()
                    except Exception as exc:
                        return self._err(503, 'model_unavailable', f'The 3-D model could not be built ({exc})'[:300], retryable=False)
                    return self._send(200, data=json.dumps(out, separators=(',', ':')).encode())
                if path == '/v1/robot-profiles':
                    return self._send(200, profile_listing(robot.profile_id, robot.simulated))
                if path in ('/v1/tasks', '/v1/body/moves'):
                    m = re.search(r'(?:^|&)limit=(\d{1,3})', query)
                    return self._send(200, robot.tasks(int(m.group(1)) if m else 20))
                m = re.fullmatch(r'/v1/(?:body/moves|tasks)/(mv-[0-9a-f]{12})', path)
                if m:
                    rec = robot.moves.get(m.group(1))
                    return self._send(200, robot._public(rec)) if rec else self._err(404, 'not_found', 'no such task')
            except Exception as exc:
                return self._err(503, 'robot_error', str(exc)[:300])
            self._err(404, 'not_found', f'no such route: GET {path}; see /v1/docs')

        do_HEAD = do_GET

        def _not_allowed(self):
            if not self._allowed(): return
            self.close_connection = True
            self._err(405, 'method_not_allowed', f'{self.command} {self.path.partition("?")[0]}: not offered; see /v1/docs')

        do_PUT = do_PATCH = do_DELETE = _not_allowed

        def do_POST(self):
            if not self._allowed(): return
            path = self.path.partition('?')[0]
            if path in ('/v1/stop', '/api/stop', '/v1/body/stop'):
                try:                                             # read and ignore: Stop takes no arguments and never fails on them
                    n = int(self.headers.get('Content-Length') or 0)
                    if 0 < n <= MAX_BODY: self.rfile.read(n)
                except (ValueError, OSError):
                    self.close_connection = True
                out = robot.stop('page' if self.headers.get(DOOR_HEADER) else 'api')
                return self._send(200 if out['stopped'] else 503, out)
            body = self._json_body()
            if body is None: return
            if path in ('/v1/tasks', '/v1/body/move'):             # one way to start a task, on an arm robot and on a humanoid
                if path == '/v1/tasks' and body.get('mode', 'run') != 'run':
                    return self._err(400, 'invalid', 'mode: a humanoid runs its moves ("run"); there is no preview')
                body = {k: v for k, v in body.items() if not (path == '/v1/tasks' and k == 'mode')}
                unknown = sorted(set(body) - MOVE_FIELDS)
                if unknown:
                    return self._err(400, 'invalid', f'unknown fields: {", ".join(str(k)[:40] for k in unknown[:8])}')
                code, out = robot.move(body.get('prompt'), body.get('duration_s', 4))
                return self._sendj(code, out)
            if re.fullmatch(r'/v1/robot-profiles/[a-z][a-z0-9_+]{0,79}/apply', path):
                return self._err(409, 'profile_set_on_robot', 'Profiles are set on the robot: roborun up --profile <name>')
            self._err(404, 'not_found', f'no such route: POST {path}; see /v1/docs')

    return H


API_VERSION = '1.0'
DOCS_HTML = """<!doctype html><html><head><meta charset="utf-8"><title>Roborun robot API</title>
<style>body{font:15px/1.5 system-ui,sans-serif;max-width:760px;margin:40px auto;padding:0 20px}pre{font:13px ui-monospace,monospace;background:#f4f4f2;border-radius:6px;padding:12px;overflow:auto}</style></head><body>
<h1>Roborun robot API: humanoid</h1><p>The body on this computer, at <code>http://127.0.0.1:8120</code>. <a href="/v1/openapi.json">openapi.json</a></p>
<pre>curl -X POST http://127.0.0.1:8120/v1/body/move -H 'Content-Type: application/json' -d '{"prompt": "walk to the red box", "duration_s": 4}'
curl http://127.0.0.1:8120/v1/body/moves/ID
curl -X POST http://127.0.0.1:8120/v1/stop
curl http://127.0.0.1:8120/v1/stop/status
curl http://127.0.0.1:8120/v1/body/state
curl http://127.0.0.1:8120/v1/body/model
curl -o head.jpg http://127.0.0.1:8120/v1/cameras/head.jpg
curl http://127.0.0.1:8120/v1/status</pre>
<p>The head camera frame is not face-blurred yet, also in what a move sends to the hosted API: keep it on this computer.</p>
<p>Errors: <code>{"error": {"code", "message", "retryable"}}</code>. Stop is never refused.</p>
<p>An AI assistant running this robot for someone: <code>roborun docs agents</code> is the one guide it needs (every command, route, error and what to do). This robot's own viewer (tasks, recordings, cameras, Stop): <a href="http://127.0.0.1:8135/">http://127.0.0.1:8135/</a>.</p></body></html>"""


def openapi():
    """The humanoid local API, for code generators (the arm profile's own spec is served by the arm runtime)."""
    error = {'description': 'error', 'content': {'application/json': {'schema': {'type': 'object', 'properties': {'error': {
        'type': 'object', 'required': ['code', 'message', 'retryable'],
        'properties': {'code': {'type': 'string'}, 'message': {'type': 'string'}, 'retryable': {'type': 'boolean'}}}}}}}}

    def op(summary, **more):
        return {'summary': summary, 'responses': {'200': {'description': 'ok', **more}, 'default': error}}
    move = {'type': 'object', 'required': ['prompt'], 'additionalProperties': False, 'properties': {
        'prompt': {'type': 'string', 'minLength': 1, 'maxLength': 4000},
        'duration_s': {'type': 'integer', 'enum': [2, 3, 4, 6, 8], 'default': 4}}}
    record = {'content': {'application/json': {'schema': MOVE_SCHEMA}}}
    return {'openapi': '3.1.0', 'info': {'title': 'Roborun robot API (humanoid)', 'version': API_VERSION,
                                         'description': 'Errors: {"error": {"code", "message", "retryable"}}.'},
            'servers': [{'url': 'http://127.0.0.1:8120/v1'}],
            'paths': {'/body/move': {'post': {'summary': 'Move: the head frame and your words become a checked timeline, played on the body',
                                              'requestBody': {'required': True, 'content': {'application/json': {'schema': move}}},
                                              'responses': {'202': {'description': 'started', **record}, '409': dict(error, description='a move is running'),
                                                            'default': error}}},
                      '/body/moves/{id}': {'get': op('One move', **record)}, '/tasks': {'get': op('The moves, newest first')},
                      '/stop': {'post': op('Stop now, at any stage. Never refused', content={'application/json': {'schema': STOP_SCHEMA}})},
                      '/stop/status': {'get': op('Is the body at rest: stopped, stopping, running')},
                      '/body/state': {'get': op('Base odometry and velocity, head pitch and yaw')},
                      '/body/model': {'get': op('The 3-D display model: body tree, meshes, head joints and camera, the simulated room')},
                      '/cameras/head.jpg': {'get': op('The head camera now (JPEG). Not face-blurred yet: keep it on this computer')},
                      '/body': {'get': op('The profile: robot descriptor, camera, limits, arms_offered (the arms this API drives)')},
                      '/status': {'get': op('Ready, profile, the running move and the body state')},
                      '/health': {'get': op('Is it up')}}}


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 128      # the stdlib backlog of 5 drops a Stop's SYN behind a few pollers (kernel retry after 1 s)


def serve(robot, host='127.0.0.1', port=8120):
    if host not in ('127.0.0.1', 'localhost'):
        raise SystemExit('the local robot API listens on this computer only (127.0.0.1)')
    return _Server((host, port), handler(robot))


# ---------------------------------------------------------------------------------------------------- the service
def open_body(profile_id, scene=None, agent=None):
    """(base, head, close, simulated) for a simulated scene or a real body's agent. A real body that does not answer is an error,
    never replaced by a simulation."""
    if scene:
        from .sim import BodySim
        sim = BodySim(profile_id, scene=scene)
        return sim.base, sim.head, sim.close, scene
    if not agent or not agent.get('url'):
        raise SystemExit(f'{profile_id}: no robot connected (roborun up --profile {profile_id} --body-agent http://ROBOT:PORT '
                         '--body-agent-key-file FILE), and it is not set up to simulate (--sim room)')
    from .adapters import AgentBody, AgentError
    key = Path(agent['key_file']).expanduser().read_text().strip()
    body = AgentBody(profile_id, agent['url'], key)
    try:
        body.check()
    except AgentError as exc:
        raise SystemExit(f'{profile_id}: {exc}') from None
    return body.base, body.head, lambda: None, None


def cmd_serve(a) -> int:
    from .. import robot as robot_mod
    c = json.loads(robot_mod.CONFIG.read_text())
    b = c.get('body') or {}
    pid = b.get('profile') or c.get('profile')
    if not profiles.is_body(pid):
        raise SystemExit(f'{pid!r} is not a humanoid profile')
    base, head, close, simulated = open_body(pid, b.get('sim'), b.get('agent'))
    key = Path(c['key_file']).expanduser().read_text().strip()
    body = BodyRobot(pid, base, head, BodyJobs(c['server'], key), robot=c.get('robot', 'robot'), simulated=simulated, close=close)
    import urllib.parse
    port = urllib.parse.urlsplit(c.get('backend') or 'http://127.0.0.1:8120').port or 8120
    srv = serve(body, port=port)

    def down(*_):
        body.close()
        threading.Thread(target=srv.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, down)
    signal.signal(signal.SIGINT, down)
    print(json.dumps(dict(event='body_ready', profile=pid, simulated=bool(simulated), scene=simulated, address=f'127.0.0.1:{port}')),
          flush=True)
    try:
        srv.serve_forever()
    finally:
        srv.server_close()
    return 0


def add_parser(sub):
    p = sub.add_parser('body', help='the local robot API of a humanoid profile (roborun up starts it)')
    p.add_argument('action', choices=('serve',))
    p.set_defaults(func=cmd_serve)
