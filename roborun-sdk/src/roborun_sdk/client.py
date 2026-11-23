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

import json
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Callable, Dict, Iterator, List, Literal, Optional, TypedDict

from .errors import RoborunError

__all__ = ['API', 'Robot', 'RobotError', 'RobotUnavailable', 'Task', 'ErrorInfo', 'Outcome', 'HeldObject', 'Placed', 'StopResult', 'StopStatus',
           'TERMINAL']

API = 'http://127.0.0.1:8120'
TERMINAL = ('done', 'stopped', 'failed')

Mode = Literal['run', 'preview']
Arm = Literal['left', 'right', 'both']
Camera = Literal['top', 'left', 'right', 'head']


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

    def _bytes(self, path: str) -> bytes:
        with self._open('GET', path) as response:
            return response.read()

    # ------------------------------------------------------------------------------------------------ the robot
    def health(self) -> Dict[str, Any]:
        """{ok, ...}: is Roborun answering."""
        return self.call('GET', '/v1/health')

    def status(self) -> Dict[str, Any]:
        """Profile, arms, cameras and the running task."""
        return self.call('GET', '/v1/status')

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
    def start(self, prompt: str, mode: Mode = 'run', arm: Optional[Arm] = None) -> Task:
        """Start a task and return at once (status running). mode='preview' plans the motion; nothing moves.
        Raises RobotError code 'busy' (retryable) while another task runs."""
        body: Dict[str, Any] = {'prompt': prompt, 'mode': mode}
        if arm:
            body['arm'] = arm
        return self.call('POST', '/v1/tasks', body)

    def task(self, task_id: str) -> Task:
        """One task now (GET /v1/tasks/{id}): status running, done, stopped or failed; message, moves, outcome, error_info."""
        return self.call('GET', f'/v1/tasks/{urllib.parse.quote(task_id)}')

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
            raise
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
            on_update: Optional[Callable[[Task], None]] = None) -> Task:
        """start() then wait(): the finished task. If waiting is interrupted (Ctrl-C, timeout), the task is stopped."""
        task = self.start(prompt, mode=mode, arm=arm)
        try:
            return self.wait(task['id'], timeout=timeout, on_update=on_update)
        except BaseException:
            try:
                self.stop_task(task['id'])
            except RobotError:
                pass
            raise

    def stop_task(self, task_id: str) -> Task:
        """Stop this task; the arms hold where they are. Stopping a finished task is fine."""
        return self.call('POST', f'/v1/tasks/{urllib.parse.quote(task_id)}/stop', {})

    def recording(self, task_id: str) -> Dict[str, Any]:
        """What the cameras recorded during the task: frame counts (frames are numbered from 0) and frame URLs."""
        return self.call('GET', f'/v1/tasks/{urllib.parse.quote(task_id)}/recording')

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
        confirmed: support the arm or cut its power."""
        return self.call('POST', '/v1/stop', {})

    def stop_status(self) -> StopStatus:
        """Has everything stopped: state 'stopped' (confirmed), 'stopping', 'running' (a task runs) or 'moving'."""
        return self.call('GET', '/v1/stop/status')
