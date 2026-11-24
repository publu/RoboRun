"""A typed Python client for the robot on this computer (the local API at http://127.0.0.1:8120). Standard library only.

    from roborun_sdk.client import Robot, RobotError

    robot = Robot()
    task = robot.run('pick up the cup')          # start it and wait until it ends; mode='preview' moves nothing
    print(task['status'], task['message'])        # done | stopped | failed, and what happened
    jpeg = robot.frame('top')                     # the overhead camera now, faces blurred (safe to send to an LLM)
    robot.stop()                                  # all motion, like the Stop button

Every call raises RobotError(status, code, message, retryable) when the robot refuses, and RobotUnavailable (a RobotError
with status 0) when nothing answers at the address: Roborun is not running, start it with `roborun up`. `retryable` says
whether the same call may work later (another task is running, a camera has no fresh frame) or the request must change.

The same calls work on an arm profile and on a humanoid profile where the robot offers them; `start`/`run` are for arm
profiles (a humanoid moves with POST /v1/body/move, `Robot.body_move`).
"""
from __future__ import annotations

import datetime
import json
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Callable, Dict, Iterator, List, Literal, NoReturn, Optional, TypedDict

from .errors import RoborunError


def _utc_now() -> str:
    """The clock and format of a policy run's record ('ended'), so a Stop's start can be compared with it."""
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds')

__all__ = ['API', 'Robot', 'RobotError', 'RobotUnavailable', 'Task', 'ErrorInfo', 'Outcome', 'HeldObject', 'Placed', 'StopResult', 'StopStatus',
           'TERMINAL']

API = 'http://127.0.0.1:8120'
TERMINAL = ('done', 'stopped', 'failed')
STOP_CONFIRM_S = 2.0              # Robot.stop: how long the backend has to answer POST /v1/stop before this computer's policy
                                  # runs are stopped at the arms' station without it (they do not need the backend to move)

Mode = Literal['run', 'preview']
Arm = Literal['left', 'right', 'both']
Camera = Literal['top', 'left', 'right', 'head']
TaskCamera = Literal['top', 'left', 'right']     # the camera a task's motion is generated from


class ErrorInfo(TypedDict, total=False):
    """A failed task's cause when Roborun knows it: what your program can do about it."""
    code: str
    retryable: bool
    retry_after_s: Optional[float]
    next: str


class HeldObject(TypedDict, total=False):
    """What one arm holds when a task ended, and which of the task's own checks say so."""
    object: Optional[str]           # what the task was asked to pick up ('cup'); None when the prompt did not name it
    lifted_mm: Optional[int]        # how far it was measured above where it was grasped
    evidence: Optional[str]         # the checks that agree, joined with '+': grip, lift, wrist_camera, overhead_camera
    verified: bool                  # every check the task runs for this step passed


class Placed(TypedDict, total=False):
    """An object a task let go of."""
    arm: str
    object: Optional[str]
    destination: Optional[str]      # what the prompt put it on or in, when it named one
    distance_mm: Optional[int]      # how far from that destination it was let go
    verified: bool


class Outcome(TypedDict, total=False):
    """What an ended task achieved, from the task's own checks; never guessed.

    holding['left'] is a HeldObject, None (the arm was measured not holding anything: its jaw open or empty, the object let go,
    or a camera saw the object stay behind) or 'unknown' (this task ran no check on that arm). placed is 'unknown' when the task
    has no checks to read. verified_by lists the checks behind it all."""
    holding: Dict[str, Any]         # 'left' / 'right' -> HeldObject | None | 'unknown'
    placed: Any                     # List[Placed] | 'unknown'
    verified_by: List[str]


class Task(TypedDict, total=False):
    """A task as GET /v1/tasks/{id} returns it. outcome is None while it runs, for a preview and when nothing moved."""
    id: str
    status: Literal['running', 'done', 'stopped', 'failed']
    mode: Mode
    model: Optional[str]
    arm: Optional[str]
    camera: Optional[TaskCamera]
    prompt: Optional[str]
    moves: int
    message: str
    created: str
    elapsed_s: Optional[float]
    recording_url: Optional[str]
    error: Optional[str]
    error_info: Optional[ErrorInfo]
    outcome: Optional[Outcome]


class StopResult(TypedDict, total=False):
    """POST /v1/stop: whether the Stop was taken and what the arms are doing."""
    state: Literal['stopped', 'stopping', 'not_confirmed']
    confirmed: bool
    stopped_tasks: List[str]
    arms: Dict[str, str]
    message: str


class StopStatus(TypedDict, total=False):
    """GET /v1/stop/status: has everything stopped."""
    state: str
    confirmed: bool
    arms: Dict[str, Dict[str, Any]]
    task: Optional[Dict[str, str]]
    max_speed_rad_s: Optional[float]
    message: str


class RobotError(RoborunError):
    """The robot answered with an error, or did not answer.

    status: the HTTP status (0 when nothing answered). code: a stable word to branch on (invalid, not_found, busy,
    unavailable, stop_not_confirmed, not_running, ...). message: plain words, fine to show a person or an LLM.
    retryable: the same call may work later. body: the robot's whole answer, when it sent one."""

    def __init__(self, status: int, code: str, message: str, retryable: bool = False, body: Any = None):
        super().__init__(f'{status} {code}: {message}' if status else message)
        self.status, self.code, self.message, self.retryable, self.body = status, code, message, retryable, body


class RobotUnavailable(RobotError):
    """Nothing answers at the robot's address: Roborun is not running on this computer (start it with `roborun up`)."""


def error_from(status: int, raw: bytes) -> RobotError:
    """A RobotError from an error answer, whatever its shape: {"error": {code, message, retryable}}, {"error": "text"}, or text."""
    try:
        body = json.loads(raw or b'{}')
    except ValueError:
        body = None
    err = body.get('error') if isinstance(body, dict) else None
    if isinstance(err, dict):
        return RobotError(status, str(err.get('code') or 'error'), str(err.get('message') or ''), bool(err.get('retryable')), body)
    text = err if isinstance(err, str) else (raw or b'').decode('utf-8', 'replace').strip()[:300]
    return RobotError(status, 'error', text or f'HTTP {status}', status in (429, 502, 503, 504), body)


def _stop_not_taken(finished, exc, out, bound):
    """What Roborun's backend did with a POST /v1/stop it did not confirm, as a clause for the Stop's message."""
    if not finished:
        return f"Roborun's backend has not confirmed the Stop within {bound:g} s"
    if isinstance(exc, RobotUnavailable):
        return "Roborun's backend is not answering"
    if isinstance(exc, RobotError):
        return f"Roborun's backend answered the Stop with an error ({exc.status} {exc.code})"
    if exc is not None:
        return f"Roborun's backend answered the Stop with something broken ({type(exc).__name__})"
    if isinstance(out, dict) and 'error' in out:
        err = out.get('error')
        code = err.get('code') if isinstance(err, dict) else None
        return f"Roborun's backend answered the Stop without confirming it ({code or 'error'})"
    return "Roborun's backend answered the Stop without saying what it stopped"


class Robot:
    """The robot on this computer. `api`: its local address (default http://127.0.0.1:8120). `timeout`: seconds per call."""

    def __init__(self, api: str = API, timeout: float = 10.0):
        self.api, self.timeout = api.rstrip('/'), timeout

    # ------------------------------------------------------------------------------------------------ plumbing
    def _open(self, method: str, path: str, body: Optional[dict] = None, headers: Optional[dict] = None,
              timeout: Optional[float] = None):
        data = None if body is None else json.dumps(body).encode()
        request = urllib.request.Request(self.api + path, data=data, method=method,
                                         headers={'Content-Type': 'application/json', **(headers or {})})
        try:
            return urllib.request.urlopen(request, timeout=self.timeout if timeout is None else timeout)
        except urllib.error.HTTPError as exc:
            raise error_from(exc.code, exc.read()) from None
        except (urllib.error.URLError, ConnectionError, TimeoutError, OSError) as exc:
            reason = getattr(exc, 'reason', exc)
            raise RobotUnavailable(0, 'not_running', f'Roborun is not answering at {self.api} ({reason}): start it with roborun up',
                                   True) from None

    def call(self, method: str, path: str, body: Optional[dict] = None) -> Any:
        """Any local API route, for what this class has no method for: the decoded JSON answer."""
        with self._open(method, path, body) as response:
            raw = response.read()
        try:
            return json.loads(raw) if raw else None
        except ValueError:
            raise RobotError(response.status, 'not_json', 'the robot answered with something that is not JSON') from None

    def _policy(self):
        """roborun_sdk.policy.control when this client talks to the robot on this computer (a loopback address), else None:
        a policy run (`roborun policy run`) is a task of this computer's robot that the backend lists but does not drive."""
        if urllib.parse.urlsplit(self.api).hostname not in ('127.0.0.1', 'localhost', '::1'):
            return None
        from .policy import control
        return control

    def _ask_policy(self, name, *args, default=None):
        """policy.control.<name>(*args) for this computer's policy runs, or `default` when this client is not on the robot's
        computer or the lookup fails in any way (a damaged record, a robot.json that is not an object): these lookups
        never stand between a Stop, a task view or a wait and Roborun's backend."""
        try:
            control = self._policy()
            return default if control is None else getattr(control, name)(*args)
        except Exception:
            return default

    def _bytes(self, path: str) -> bytes:
        with self._open('GET', path) as response:
            return response.read()

    # ------------------------------------------------------------------------------------------------ the robot
    def health(self) -> Dict[str, Any]:
        """{ok, ...}: is Roborun answering."""
        return self.call('GET', '/v1/health')

    def status(self) -> Dict[str, Any]:
        """Profile, arms, cameras and the running task (a policy run on this computer counts as the running task)."""
        out = self.call('GET', '/v1/status')
        return self._ask_policy('merge_status', out, default=out)

    def state(self) -> Dict[str, Any]:
        """The arms now (GET /v1/robot): joints_rad, gripper (0 closed .. 1 open), tool_mm per arm, cameras, safety."""
        return self.call('GET', '/v1/robot')

    def frame(self, camera: Camera = 'top', blur: bool = True) -> bytes:
        """The newest JPEG from a camera (top, left, right; head on a humanoid). blur=True (the default) covers faces, so the
        image may leave this computer, for example to an LLM. blur=False gives the sharp frame for use on this computer.
        Exception: a humanoid's head frame is not face-blurred yet (blur is ignored for 'head'): keep it on this computer."""
        if camera == 'head':
            return self._bytes('/v1/cameras/head.jpg')
        return self._bytes(f'/v1/cameras/{urllib.parse.quote(camera)}.jpg' + ('' if blur else '?blur=0'))

    # ------------------------------------------------------------------------------------------------ tasks (arm profiles)
    def start(self, prompt: str, mode: Mode = 'run', arm: Optional[Arm] = None, camera: Optional[TaskCamera] = None) -> Task:
        """Start a task and return at once (status running). mode='preview' plans the motion; nothing moves.
        camera: the camera the motion is generated from: 'top' (the overhead, the default) or a wrist camera ('left', 'right'),
        which plans its own arm. Raises RobotError code 'busy' (retryable) while another task runs."""
        body: Dict[str, Any] = {'prompt': prompt, 'mode': mode}
        if arm:
            body['arm'] = arm
        if camera:
            body['camera'] = camera
        return self.call('POST', '/v1/tasks', body)

    def task(self, task_id: str) -> Task:
        """One task now (GET /v1/tasks/{id}): status running, done, stopped or failed; message, moves, outcome, error_info.
        A policy run of this computer is read from its own record while Roborun's backend is down."""
        try:
            return self.call('GET', f'/v1/tasks/{urllib.parse.quote(task_id)}')
        except RobotUnavailable:
            rec = self._ask_policy('policy_record', task_id)
            view = self._ask_policy('view', rec) if rec is not None else None
            if view is None:
                raise
            return view

    def tasks(self, limit: int = 10) -> List[Task]:
        """Recent tasks, newest first."""
        return self.call('GET', f'/v1/tasks?limit={int(limit)}')['tasks']

    def live(self, task_id: str) -> Iterator[Task]:
        """The task each time it changes (and every 5 s), until it ends: the server-sent event stream of /live."""
        with self._open('GET', f'/v1/tasks/{urllib.parse.quote(task_id)}/live', headers={'Accept': 'text/event-stream'},
                        timeout=max(self.timeout, 30.0)) as stream:
            for line in stream:
                if line.startswith(b'data: '):
                    yield json.loads(line[6:])

    def wait(self, task_id: str, timeout: Optional[float] = None, on_update: Optional[Callable[[Task], None]] = None,
             poll_s: float = 1.0) -> Task:
        """Block until the task ends; returns it (status done, stopped or failed). on_update(task) runs on every change.
        Follows the live stream and falls back to polling if the stream drops. timeout: give up (TimeoutError) after this
        many seconds; the task keeps running (stop it with stop_task)."""
        deadline = None if timeout is None else time.monotonic() + timeout
        last = None
        try:
            for view in self.live(task_id):
                if on_update and view != last:
                    on_update(view)
                last = view
                if view.get('status') in TERMINAL:
                    return self.task(task_id)
                if deadline is not None and time.monotonic() > deadline:
                    raise TimeoutError(f'task {task_id} still running after {timeout} s')
        except RobotUnavailable:
            if self._ask_policy('policy_record', task_id) is None:
                raise                                              # a policy run is polled from its record below
        except (OSError, ValueError, RobotError):
            pass                                                   # the stream dropped: poll instead
        while True:
            view = self.task(task_id)
            if on_update and view != last:
                on_update(view)
            last = view
            if view.get('status') in TERMINAL:
                return view
            if deadline is not None and time.monotonic() > deadline:
                raise TimeoutError(f'task {task_id} still running after {timeout} s')
            time.sleep(poll_s)

    def run(self, prompt: str, mode: Mode = 'run', arm: Optional[Arm] = None, timeout: Optional[float] = None,
            on_update: Optional[Callable[[Task], None]] = None, camera: Optional[TaskCamera] = None) -> Task:
        """start() then wait(): the finished task. If waiting is interrupted (Ctrl-C, timeout), the task is stopped."""
        task = self.start(prompt, mode=mode, arm=arm, camera=camera)
        try:
            return self.wait(task['id'], timeout=timeout, on_update=on_update)
        except BaseException:
            try:
                self.stop_task(task['id'])
            except RobotError:
                pass
            raise

    def stop_task(self, task_id: str) -> Task:
        """Stop this task; the arms hold where they are. Stopping a finished task is fine. A policy run (`roborun policy run`)
        is stopped at the arms' station by its id, the same hold, also while Roborun's backend is down; it answers 'stopping'
        until the run has closed its record (then 'stopped'). Raises RobotError code 'stop_not_confirmed' (status 503) when
        the station could not take it. Anything else that goes wrong looking for a policy run (a damaged record, a robot.json
        that is not an object) sends the Stop to Roborun's backend, as for any task."""
        if self._ask_policy('policy_record', task_id) is not None:
            control = self._policy()
            try:
                rec = control.stop_run(task_id)
            except control.StopNotConfirmed as exc:
                raise RobotError(503, 'stop_not_confirmed', str(exc), True) from None
            except (KeyError, IndexError) as exc:                 # a damaged record, not "no such policy run"
                return self._stop_damaged_run(task_id, exc)
            except LookupError:                                    # not a policy run after all: the backend's own Stop
                rec = None
            except Exception as exc:                               # anything else: the backend's Stop, never silence
                return self._stop_damaged_run(task_id, exc)
            if rec is not None:
                try:
                    view = self.task(task_id)
                except RobotError:
                    return self._ask_policy('view', rec, default=rec)
                if rec.get('status') == 'stopping' and view.get('status') == 'running':
                    view = dict(view, status='stopping')
                return view
        return self.call('POST', f'/v1/tasks/{urllib.parse.quote(task_id)}/stop', {})

    def _stop_damaged_run(self, task_id: str, exc: BaseException) -> Task:
        """A policy run whose Stop could not be worked out here: the backend's own Stop all the same, and if the backend does
        not know the task either, the Stop is not confirmed (never a quiet 'no such task' while a run may still move)."""
        try:
            return self.call('POST', f'/v1/tasks/{urllib.parse.quote(task_id)}/stop', {})
        except RobotError:
            raise RobotError(503, 'stop_not_confirmed', f'the Stop of policy run {task_id} could not be sent to the arms\' '
                             f'station ({exc!r}); use roborun stop, or the arm\'s power switch if anything moves',
                             True) from None

    def recording(self, task_id: str) -> Dict[str, Any]:
        """What the cameras recorded during the task: frame counts (frames are numbered from 0) and frame URLs."""
        return self.call('GET', f'/v1/tasks/{urllib.parse.quote(task_id)}/recording')

    def clip(self, task_id: str) -> NoReturn:
        """Removed: always raises RobotError (status 410, code 'removed', not retryable) and asks the robot for nothing.

        It existed in 0.1.20261119203708 and was removed in the next release. What the cameras saw during the task is
        recording(task_id). Kept so a program written for the older release gets this message, not an AttributeError."""
        raise RobotError(410, 'removed', f'Robot.clip() was removed in 0.1.20261119224249. '
                                         f'What the cameras saw during task {task_id}: Robot.recording({task_id!r}).')

    # ------------------------------------------------------------------------------------------------ humanoid profiles
    def body_move(self, prompt: str, duration_s: int = 4) -> Dict[str, Any]:
        """A humanoid profile's move (POST /v1/body/move): 202 with the move; follow it with body_move_status(id)."""
        return self.call('POST', '/v1/body/move', {'prompt': prompt, 'duration_s': duration_s})

    def body_move_status(self, move_id: str) -> Dict[str, Any]:
        """One move now (GET /v1/body/moves/{id}): status running, done, stopped or failed; stage; played (odometry_start,
        odometry_end, played_s, stopped); deferred (parts of the prompt left for a next move); error {code, message}."""
        return self.call('GET', f'/v1/body/moves/{urllib.parse.quote(move_id)}')

    def body_run(self, prompt: str, duration_s: int = 4, timeout: Optional[float] = 180.0,
                 on_update: Optional[Callable[[Dict[str, Any]], None]] = None, poll_s: float = 0.5) -> Dict[str, Any]:
        """body_move() then wait until the move ends: the finished move (status done, stopped or failed). on_update(move) runs
        on every change. If waiting is interrupted (Ctrl-C, timeout: TimeoutError), the body is stopped."""
        move = self.body_move(prompt, duration_s=duration_s)
        deadline = None if timeout is None else time.monotonic() + timeout
        last = None
        try:
            while True:
                view = self.body_move_status(move['id'])
                if on_update and view != last:
                    on_update(view)
                last = view
                if view.get('status') in TERMINAL:
                    return view
                if deadline is not None and time.monotonic() > deadline:
                    raise TimeoutError(f"move {move['id']} still running after {timeout} s")
                time.sleep(poll_s)
        except BaseException:
            if (last or {}).get('status') not in TERMINAL:
                try:
                    self.stop()
                except RobotError:
                    pass
            raise

    # ------------------------------------------------------------------------------------------------ Stop
    def stop(self) -> StopResult:
        """Stop all motion now, like the Stop button. Never refused. Returns state 'stopped' or 'stopping' (an arm still
        brakes: see stop_status()). Raises RobotError code 'stop_not_confirmed' (status 503) when it could not be
        confirmed: support the arm or cut its power. A policy run stops with everything else (stopped_tasks names it); while
        Roborun's backend is down, does not answer within STOP_CONFIRM_S, or answers the Stop with anything but a 2xx JSON
        answer with no error in it (an error status, an error in the body, a broken or non-JSON reply), the policy runs on this computer are still stopped at the arms'
        station (the robot-wide hold first)."""
        control = self._policy()
        if control is None:                                    # another computer's robot: its backend answers, as for any call
            return self.call('POST', '/v1/stop', {})
        since = _utc_now()                                     # a clock reading only: nothing is looked up first
        box, done = {}, threading.Event()

        def send():
            try:
                box['out'] = self.call('POST', '/v1/stop', {})  # first, before anything else is looked up
            except BaseException as exc:                       # noqa: BLE001 - handed back below
                box['exc'] = exc
            finally:
                done.set()
        threading.Thread(target=send, name='roborun-stop', daemon=True).start()
        finished = done.wait(STOP_CONFIRM_S)                # read once: the send thread may finish just after this
        exc, out = box.get('exc'), box.get('out')
        if finished and exc is None and isinstance(out, dict) and 'error' not in out:
            return self._with_stopped_runs(out, since)     # the backend confirmed it: a 2xx JSON answer with no error in it
        # anything else is not the backend taking the Stop (a Stop is never refused): the policy runs here do not need the
        # backend to move, so they are stopped at the arms' station
        backend = _stop_not_taken(finished, exc, out, STOP_CONFIRM_S)
        try:
            result = control.stop_all()
        except Exception as failed:                            # the runs could not be read or stopped: not confirmed
            if not self._running_policy_runs():
                return self._backend_stop_answer(box, done, since)
            raise RobotError(503, 'stop_not_confirmed', str(failed) or 'the arms\' station did not take the Stop',
                             True) from None
        if not result['ids']:                                  # no policy run here: the backend's own answer, as before
            return self._backend_stop_answer(box, done, since)
        message = control.stop_all_message(result, backend)
        if result['failed'] and not result['held']:
            raise RobotError(503, 'stop_not_confirmed', message, True)
        return {'state': 'stopping', 'confirmed': False, 'stopped_tasks': list(result['ids']), 'arms': {},
                'message': message}

    def _backend_stop_answer(self, box, done, since) -> StopResult:
        """The backend's own answer to POST /v1/stop (waited for, within this client's timeout), or its error raised."""
        done.wait(self.timeout + 1.0)
        if not done.is_set():
            raise RobotUnavailable(0, 'not_running', f'Roborun is not answering at {self.api} (the Stop was not answered)',
                                   True)
        if 'exc' in box:
            raise box['exc']
        return self._with_stopped_runs(box.get('out'), since)

    def _with_stopped_runs(self, out, since):
        """The backend's Stop answer, with the policy runs this Stop stopped (closed or not) in stopped_tasks."""
        runs = self._ask_policy('stopped_since', since, default=[]) or []
        runs = [r for r in runs if isinstance(r, str)]
        if runs and isinstance(out, dict):
            out = dict(out, stopped_tasks=list(dict.fromkeys(list(out.get('stopped_tasks') or []) + runs)))
        return out

    def _running_policy_runs(self) -> List[str]:
        """The ids of this computer's running policy runs; [] when there are none or they cannot be read."""
        runs = self._ask_policy('running', default=[]) or []
        return [r['id'] for r in runs if isinstance(r, dict) and isinstance(r.get('id'), str)]

    def stop_status(self) -> StopStatus:
        """Has everything stopped: state 'stopped' (confirmed), 'stopping', 'running' (a task runs) or 'moving'. A policy
        run on this computer counts as the running task."""
        out = self.call('GET', '/v1/stop/status')
        return self._ask_policy('merge_stop_status', out, default=out)
