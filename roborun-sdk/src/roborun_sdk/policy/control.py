"""The per-task Stop of a policy run, and the policy runs the task routes must know about. Standard library only.

A policy run is a Roborun task (its record is <runtime>/log/tasks/<id>.json, kind 'policy'), but its motion is driven by the
process that runs it (`roborun policy run`, policy.run), not by the robot's backend. So the task's own Stop goes to the arms'
station, by the run's id:

    stop_run(task_id)      POST /supervisor/policy/stop {run_id} at the station. A running lease of that run ends the way an
                           ordinary task's Stop ends its lease (/supervisor/stop: the arms hold where they are, nothing more
                           plays); a run still warming up is refused its lease and ends itself, unmoved, within 0.25 s.
                           Other runs and leases are left alone. The record answers 'stopping' until the run closes it.
    running()              the policy runs still running on this computer (their records), newest first.
    policy_record(task_id) a policy run's record, or None when the id is not one.

Robot.stop_task, Robot.stop_status, Robot.stop and the hosted page's per-task Stop (through roborun_sdk.local_proxy) use these,
and so can the robot's backend for POST /v1/tasks/{id}/stop. They work while the backend is down, like `roborun stop`.
Callers never let these lookups stand between a Stop and the backend: Robot.stop sends POST /v1/stop first, and any failure
here (a damaged record, a robot.json that is not an object) falls through to the backend's own Stop.
"""
from __future__ import annotations

import json
import re
import time
import urllib.error
import urllib.request
from pathlib import Path

TASK_ID = re.compile(r'[0-9a-f]{32}')
HEADERS = {'Content-Type': 'application/json', 'X-Manifest-Supervisor': '1'}
RUNNING = ('running', 'stopping')
STOPPED_BY = ('operator_stop', 'signal')   # a closed run's stop_reason when a Stop or the hold ended it
CLOSE_WAIT_S = 0.3             # how long stop_run waits for the run to close its own record before answering 'stopping'


class NoSuchRun(LookupError):
    """The id is not a policy run on this computer (stop_run). A KeyError or IndexError from a damaged answer is not this."""


class StopNotConfirmed(RuntimeError):
    """The station did not take the run's Stop (it is not running or refused). The run cannot move without the station; use
    `roborun stop` or the arm's power switch if anything moves."""


def _where(runtime=None, station=None):
    from .. import robot
    config = {}
    try:
        config = json.loads(robot.CONFIG.read_text())
    except (OSError, ValueError):
        pass
    if not isinstance(config, dict):                      # robot.json that is JSON but not an object: as if it were absent
        config = {}
    station_url = config.get('station') if isinstance(config.get('station'), str) else None
    from .record import robot_runtime
    root = Path(runtime) if runtime else robot_runtime(config)
    url = (station or station_url or f'http://127.0.0.1:{robot.STATION_PORT}').rstrip('/')
    return root, url


def _read(path):
    try:
        rec = json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None
    return rec if isinstance(rec, dict) else None


def policy_record(task_id, runtime=None):
    """The record of policy run `task_id` on this computer, or None (not a task id, no such record, or not a policy run)."""
    if not isinstance(task_id, str) or not TASK_ID.fullmatch(task_id):
        return None
    root, _ = _where(runtime)
    rec = _read(root / 'log' / 'tasks' / f'{task_id}.json')
    return rec if rec is not None and rec.get('kind') == 'policy' else None


def running(runtime=None):
    """Policy runs whose record says running (a run whose process is gone is closed first, as `roborun stop` does)."""
    root, _ = _where(runtime)
    try:
        from .record import close_orphans
        close_orphans(root)
    except Exception:
        pass
    out = []
    try:
        paths = sorted((root / 'log' / 'tasks').glob('*.json'), key=lambda p: p.stat().st_mtime, reverse=True)
    except OSError:
        return out
    for path in paths[:200]:
        rec = _read(path)
        if (rec is not None and rec.get('kind') == 'policy' and rec.get('status') in RUNNING
                and isinstance(rec.get('id'), str) and TASK_ID.fullmatch(rec['id'])):
            out.append(rec)                                # a record without a task id is not a run anything can stop
    return out


def _post(url, path, body, timeout=2.0):
    req = urllib.request.Request(url + path, data=json.dumps(body).encode(), method='POST', headers=HEADERS)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as reply:
            return reply.status, json.loads(reply.read() or b'{}')
    except urllib.error.HTTPError as exc:
        try:
            payload = json.loads(exc.read() or b'{}')
        except ValueError:
            payload = {}
        return exc.code, payload if isinstance(payload, dict) else {}


def _get(url, path, timeout=2.0):
    req = urllib.request.Request(url + path, method='GET', headers=HEADERS)
    with urllib.request.urlopen(req, timeout=timeout) as reply:
        return json.loads(reply.read() or b'{}')


def _station_stop(url, task_id):
    """Stop one run at the station: its own route; on an older station without it, the run's lease by its id, or the hold
    when no task has a lease (the run is still warming up and reads the hold); never another task's lease."""
    try:
        code, reply = _post(url, '/supervisor/policy/stop', dict(run_id=task_id))
    except (OSError, ValueError) as exc:
        raise StopNotConfirmed(f'the arms\' station is not answering ({exc}); the run cannot move without it') from None
    if code == 200:
        return dict(how='policy_stop', lease_ended=bool(reply.get('lease_ended')))
    if code != 404:
        raise StopNotConfirmed(f'the arms\' station refused the Stop (HTTP {code}: {reply.get("error")})')
    try:                                                  # a station without the route: end the run's lease by its id
        lease = (_get(url, '/supervisor/pose') or {}).get('execution_lease') or {}
        if lease.get('active') and lease.get('kind') == 'policy' and lease.get('run_id') == task_id:
            code, reply = _post(url, '/supervisor/stop', dict(lease_id=lease['id']))
            if code == 200:
                return dict(how='lease_stop', lease_ended=True)
        elif lease.get('active'):                         # another task holds the arms: never stop it for this one
            raise StopNotConfirmed('the arms\' station is older than this Roborun and another task is running; '
                                   'update Roborun (roborun upgrade), or use roborun stop')
        # no lease at all (this run is still warming up): the hold is the only Stop an older station has, and it ends the run
        code, reply = _post(url, '/supervisor/hold', dict(source='task stop', reason='operator'))
        if code == 200:
            return dict(how='hold', lease_ended='lease_ended' in (reply.get('actions') or []))
    except (OSError, ValueError) as exc:
        raise StopNotConfirmed(f'the arms\' station is not answering ({exc})') from None
    raise StopNotConfirmed(f'the arms\' station refused the Stop (HTTP {code})')


def stop_run(task_id, runtime=None, station=None, wait_s=CLOSE_WAIT_S):
    """The per-task Stop of policy run `task_id`: the station ends that run's lease (or refuses it one) and the arms hold where
    they are. Returns its record: status 'stopping' until the run closes it (then 'stopped'), or as it is when the run had
    already ended (stopping a finished task is fine). Raises NoSuchRun (a LookupError) when the id is not a policy run on this
    computer, StopNotConfirmed when the station could not take the Stop."""
    root, url = _where(runtime, station)
    rec = policy_record(task_id, root)
    if rec is None:
        raise NoSuchRun('no such policy run')
    if rec.get('status') not in RUNNING:
        return rec
    done = _station_stop(url, task_id)
    deadline = time.monotonic() + max(0.0, wait_s)
    while True:
        now = policy_record(task_id, root) or rec
        if now.get('status') not in RUNNING or time.monotonic() >= deadline:
            break
        time.sleep(0.02)
    if now.get('status') in RUNNING:
        now = dict(now, status='stopping', phase='Stopping')
    return dict(now, stop_request=done)


def _hold(url):
    """The station's robot-wide hold (POST /supervisor/hold, as `roborun stop` sends): it ends any lease, every policy run's
    too. True when the station took it."""
    try:
        code, _ = _post(url, '/supervisor/hold', dict(source='robot stop', reason='operator'))
    except (OSError, ValueError):
        return False
    return code == 200


def stop_all(runtime=None, station=None, wait_s=CLOSE_WAIT_S):
    """The robot-wide Stop of this computer's policy runs while Roborun's backend is down: the station's hold first, then each
    run's own Stop in its own try, so one failure never leaves a later run going. Returns dict(held, stopped, failed) where
    failed is [(run_id, message)]. The Stop is confirmed at the station when held is True or failed is empty."""
    root, url = _where(runtime, station)
    ids = [r['id'] for r in running(root)]
    if not ids:                                           # nothing of ours runs: the caller says the backend is down
        return dict(ids=[], held=False, stopped=[], failed=[])
    held = _hold(url)
    stopped, failed = [], []
    for run_id in ids:
        try:
            stop_run(run_id, root, url, wait_s=wait_s)
            stopped.append(run_id)
        except NoSuchRun:                                 # its record went between the lookup and its Stop: nothing left to stop
            stopped.append(run_id)
        except Exception as exc:                           # StopNotConfirmed or anything else (a KeyError from a damaged
                                                           # answer too): noted, never counted as stopped; the next run still stops
            failed.append((run_id, str(exc) or type(exc).__name__))
    return dict(ids=ids, held=held, stopped=stopped, failed=failed)


BACKEND_DOWN = "Roborun's backend is not answering"


def stop_all_message(result, backend=BACKEND_DOWN):
    """One sentence for stop_all's result. `backend`: what Roborun's backend did with the Stop (it is down, it did not answer
    in time, or it answered with an error)."""
    if result['failed']:
        names = '; '.join(f'{run_id}: {why}' for run_id, why in result['failed'])
        if result['held']:
            return (f"The arms' station took the robot-wide hold, so every policy run has stopped; the per-run Stop of some "
                    f"was not taken ({names}). {backend}.")
        return (f"The Stop of these policy runs was not confirmed: {names}. Use roborun stop, or the arm's power switch if "
                f"anything moves.")
    if result['held']:
        return f"The arms' station took the robot-wide hold and the Stop of each policy run; {backend}"
    return f"The arms' station took the Stop of the policy runs; {backend}"


def stopped_since(since, runtime=None):
    """The ids of the policy runs still running, and of those that closed at or after `since` (a utc_now() string) because the
    station ended them (stop_reason operator_stop: a hold or a Stop; or a signal): the runs a robot-wide Stop sent at `since`
    stopped, even when they closed their records before the Stop was answered. A run that ended on its own then (done, failed,
    the provider's session restarted, Ctrl-C) is not named."""
    root, _ = _where(runtime)
    out = [r['id'] for r in running(root)]
    try:
        paths = sorted((root / 'log' / 'tasks').glob('*.json'), key=lambda p: p.stat().st_mtime, reverse=True)
    except OSError:
        return out
    for path in paths[:200]:
        rec = _read(path)
        if (rec is not None and rec.get('kind') == 'policy' and rec.get('status') == 'stopped'
                and rec.get('stop_reason') in STOPPED_BY and isinstance(rec.get('ended'), str) and rec['ended'] >= since
                and isinstance(rec.get('id'), str) and TASK_ID.fullmatch(rec['id']) and rec['id'] not in out):
            out.append(rec['id'])
    return out


def view(rec):
    """A small public view of a policy record, for an answer when the robot's backend is not there to give its own."""
    keys = ('id', 'kind', 'status', 'prompt', 'execute', 'created', 'elapsed_s', 'stop_reason', 'model', 'phase', 'final',
            'error')
    out = {k: rec.get(k) for k in keys if k in rec}
    out['message'] = rec.get('final') or rec.get('phase')
    return out


def merge_status(out, runtime=None):
    """GET /v1/status with this computer's policy runs: the backend does not drive them, so it names no task while one runs.
    When it names none, the newest running policy run is the task; anything else is unchanged."""
    if not isinstance(out, dict) or out.get('task'):
        return out
    runs = running(runtime)
    if not runs:
        return out
    rec = runs[0]
    task = {'id': rec.get('id'), 'status': 'stopping' if rec.get('status') == 'stopping' else 'running', 'kind': 'policy'}
    if isinstance(rec.get('prompt'), str) and rec['prompt']:
        task['prompt'] = rec['prompt']
    return dict(out, task=task)


def merge_stop_status(out, runtime=None):
    """GET /v1/stop/status with this computer's policy runs: the backend does not drive them, so it names no task while one
    runs. A running policy run is the running task ('running', or 'stopping' after its Stop); anything else is unchanged."""
    if not isinstance(out, dict) or out.get('task') or out.get('state') not in ('stopped', 'moving'):
        return out
    runs = running(runtime)
    if not runs:
        return out
    rec = runs[0]
    state = 'stopping' if rec.get('status') == 'stopping' else 'running'
    return dict(out, state=state, confirmed=False, task={'id': rec.get('id'), 'status': state, 'kind': 'policy'},
                message='The task is stopping' if state == 'stopping' else 'A task is running')
