"""This robot's door for the hosted page: app.roborun.app, open in a browser on THIS computer, reaches this computer's robot
here and nowhere else. It listens on loopback only, at a fixed address the page knows (127.0.0.1:8135), and passes the page's
calls to the robot's own backend.

    GET  /roborun/hello              the handshake: answers the allowed page only, and hands it this run's key
    ANY  /k/<key>/<api path>         the robot's API, cameras, live state and recordings

It also serves this robot's own viewer, for the person at this computer, signed in or not, online or not:

    GET  /  (and /view/...)          the viewer page (roborun_sdk/viewer): status, live cameras, recent tasks, recordings, Stop
    GET  /view/session               the viewer's own key: same-origin pages only (no CORS header, so no other site can read it)
    GET  /v/<view key>/<api path>    read-only routes of the robot (VIEW_GET) and POST /v1/stop: nothing that starts or changes
                                     anything. A request from a page must come from this address itself.

Who gets in:
- Host must name this address (a DNS-rebinding page arrives with its own host name).
- A request that carries an Origin must carry an allowed one, exactly; anything else is refused, preflight included.
- Images, video and links carry no Origin, so nothing can be concluded from its absence. They need the key, which only the
  handshake gives out, and they can only read (GET / HEAD). This is what the key is for: without it any website could put
  this robot's cameras in an <img>.
The backend itself refuses browsers. What comes through here is marked with DOOR_HEADER, which no web page can send to it (that
would need a preflight it never grants), so this door stays the page's only way in.
Programs on this computer can reach the backend directly anyway, so this guards against web pages, not local programs.
"""
from __future__ import annotations

import hashlib
import hmac
import http.client
import json
import os
import re
import secrets
import subprocess
import sys
import threading
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from . import net

PORT = 8135
HELLO = '/roborun/hello'
UI_ORIGINS = ('https://app.roborun.app',)
API_PREFIXES = ('/api/', '/v1/', '/internal/')
DOOR_HEADER = 'X-Roborun-Door'
METHODS = ('GET', 'HEAD', 'POST', 'PUT', 'PATCH', 'DELETE')
# what the page may send, and all that is passed on: never the headers that mark a native client or a running task
REQUEST_HEADERS = ('accept', 'cache-control', 'content-type', 'if-modified-since', 'if-none-match', 'if-range', 'last-event-id',
                   'range', 'x-robot-control')
RESPONSE_DROP = {'connection', 'keep-alive', 'proxy-authenticate', 'proxy-authorization', 'te', 'trailers', 'transfer-encoding',
                 'upgrade', 'set-cookie', 'server', 'date'}
EXPOSE = 'Accept-Ranges, Content-Disposition, Content-Length, Content-Range, Retry-After'
MAX_BODY = 64 << 20
READ_S = 130.                      # one read from the backend; live streams send far more often than this
STOP_CONFIRM_S = 2.0               # POST /v1/stop: how long the backend has to answer before this computer's policy runs are
                                   # stopped at the arms' station without it (they do not need the backend to move)
PROFILE_APPLY = re.compile(r'/v1/robot-profiles/([a-z][a-z0-9_+]{0,79})/apply')
COMPOSE = '/v1/robot-profiles/compose'
PROFILES = '/v1/robot-profiles'
TASK_STOP = re.compile(r'/v1/tasks/([0-9a-f]{32})/stop')
STOP_STATUS = '/v1/stop/status'
STATUS = '/v1/status'
USE_ARMS = '/v1/robot-profiles/use-connected-arms'
BAD_PATH = re.compile(r'(^|/)(\.|%2e){1,2}(/|$)|\\|%2f|%5c|%00', re.I)
VIEW_FILES = {'/': ('index.html', 'text/html; charset=utf-8'), '/view': ('index.html', 'text/html; charset=utf-8'),
              '/view/': ('index.html', 'text/html; charset=utf-8'),
              '/view/viewer.js': ('viewer.js', 'text/javascript; charset=utf-8'), '/view/viewer.css': ('viewer.css', 'text/css; charset=utf-8')}
VIEW_SESSION = '/view/session'
VIEW_CSP = ("default-src 'none'; script-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self'; "
            "base-uri 'none'; form-action 'none'; frame-ancestors 'none'")
# what the viewer may read: status, state, tasks and moves, the robot's own camera frames and recorded frames. Never a route that
# starts, changes or configures anything, and never generated media.
VIEW_GET = re.compile(r'/v1/(?:health|status|robot|body|body/state|body/model|stop/status|tasks|body/moves|body/frame\.jpg'
                      r'|tasks/[A-Za-z0-9_-]{1,80}(?:/recording)?|body/moves/mv-[0-9a-f]{12}|cameras/[a-z0-9_]{1,32}\.jpg'
                      r'|video-motion/[A-Za-z0-9_-]{1,120}/recording-[A-Za-z0-9_-]{1,120}/[a-z0-9_]{1,32}/[0-9]{1,7}\.jpg)')
VIEW_POST = ('/v1/stop',)
ORIGIN_OK = re.compile(r'https://[a-z0-9.-]+(:\d+)?|http://(localhost|127\.0\.0\.1)(:\d+)?')
_TOKEN_ID = re.compile(r'([a-z]{2}-[A-Za-z0-9]+)\.[a-z]{2}-[A-Za-z0-9]+$')


def account_of(token):
    """The account a token belongs to, as the sign-in page computes it (never the token or its secret). A Roborun key
    (rr_live_/rr_test_) does not name its account by itself: None, and the page does not compare accounts."""
    m = _TOKEN_ID.match((token or '').strip())
    return hashlib.sha256(m.group(1).encode()).hexdigest()[:10] if m else None


def allowed_origins(extra=()):
    """The hosted page, plus exact origins named in the robot's config (a local development page). No wildcards."""
    out, ignored = list(UI_ORIGINS), []
    for origin in extra or ():
        (out if isinstance(origin, str) and ORIGIN_OK.fullmatch(origin) else ignored).append(origin)
    if ignored:
        print(json.dumps(dict(event='ui_origin_ignored', origins=[str(o)[:80] for o in ignored])), flush=True)
    return frozenset(out)


def compose_profile(want):
    """Build a pair on the page: {left, right} arm ids (either may be empty) -> (profile id, None), or (None, why not) in the page's
    words (the command line's own messages name flags and files the page does not have)."""
    from .arm_profiles import models
    from .arms import catalog, is_own_driver
    picked = {}
    for side in ('left', 'right'):
        value = want.get(side)
        if isinstance(value, str):
            value = value.strip()
        if value in (None, ''):
            continue
        if not isinstance(value, str) or len(value) > 80:
            return None, f'Choose the {side} arm from the list.'
        picked[side] = value
    if not picked:
        return None, 'Choose at least one arm.'
    rows, have = catalog(), set(models())
    for side, arm in picked.items():
        if is_own_driver(arm):
            return None, 'An arm with its own driver needs a profile made on the robot first; it then appears under Custom.'
        if arm not in have and arm not in rows:
            return None, f'There is no arm called {arm!r}. Choose one from the list.'
        if arm not in have and rows[arm].get('profile') not in have:
            name = rows[arm].get('label') or arm
            return None, f'{name} has no 3D model yet, so it cannot run here. Choose another arm.'
    try:
        from .arms.settings import _profile_for
        return _profile_for({s: dict(arm=a) for s, a in picked.items()}), None
    except (SystemExit, Exception):
        return None, 'That pair cannot be put together. Choose other arms.'


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 64


class LocalProxy:
    def __init__(self, backend_host, backend_port, port=PORT, origins=(), robot=None, token=None):
        self.backend_host, self.backend_port, self.port = backend_host, backend_port, port
        self.origins = allowed_origins(origins)
        self.robot, self.account = robot, account_of(token)
        self.key = secrets.token_urlsafe(24)       # a new one each start; kept in memory, handed to the allowed page only
        self.view_key = secrets.token_urlsafe(24)  # the viewer's own: it opens only the read-only routes and Stop
        self.server = None
        self._backend = (0., False)
        self.switch_job = None                     # a profile switch the page asked for (it restarts this service too)
        self.switch_command = None                 # tests: what runs instead of `roborun up --profile <id>`
        self._switch_lock = threading.Lock()

    def _backend_json(self, path):
        conn = http.client.HTTPConnection(self.backend_host, self.backend_port, timeout=5)
        try:
            conn.request('GET', path); r = conn.getresponse()
            return r.status, json.loads(r.read() or b'{}')
        except (OSError, http.client.HTTPException, ValueError):
            return None, None
        finally:
            conn.close()

    def profile_catalog(self, active_id):
        """What the page offers. Defaults: one or two of the same arm, for every arm with a model. Custom: every other profile on
        this computer (a mixed pair, an example rig, your own from roborun profiles create), and any pair can be put together."""
        from .arm_profiles import RESERVED, models, name_for, parse
        try:
            from .arms import catalog
            labels = {k: v.get('label') or k for k, v in catalog().items()}
        except Exception:
            labels = {}
        ms = models(); label = lambda m: labels.get(m) or m
        defaults = [dict(arm=m, label=label(m), one=name_for(m), two='yam' if m == 'yam' else name_for(m, m)) for m in ms]
        default_ids = {d['one'] for d in defaults} | {d['two'] for d in defaults}
        home = net.roborun_home()
        try: saved = json.loads((home / 'robot.json').read_text())
        except (OSError, ValueError): saved = {}
        root = Path(saved['manifest']) if saved.get('manifest') else None
        found = set()
        sim = [home / 'sim' / 'robot' / 'runtime' / 'profiles'] if saved.get('sim') else []   # a simulated robot's own runtime
        for d in ([root / 'profiles'] if root else []) + sim + [home / 'profiles']:
            if d.is_dir():
                found |= {f.parent.name for f in d.glob('*/rig.json')}
        if active_id: found.add(active_id)
        from .body import profiles as body_profiles
        found -= set(body_profiles.ids())                  # a humanoid is listed under bodies, never as a custom arm profile
        known = dict(yam_metal='YAM + MakerMods Metal', yam_single='YAM + parked Metal (example rig)', makerarm='MakerArm pair (example rig)')
        custom = []
        for pid in sorted(found - default_ids):
            if pid == 'makerarm_urdf':                     # the MakerArm pair's alternate model, not a robot of its own
                continue
            pair = parse(pid)
            custom.append(dict(id=pid, label=known.get(pid) or (' + '.join(label(m) for m in pair if m) if pair else pid.replace('_', ' '))))
        return dict(defaults=defaults, custom=custom, active_id=active_id, arms=[dict(id=m, label=label(m)) for m in ms],
                    bodies=body_profiles.rows())

    def _known_profile(self, pid, active_id):
        from .body import profiles as body_profiles
        if body_profiles.is_body(pid):
            return True
        cat = self.profile_catalog(active_id)
        return pid in {d['one'] for d in cat['defaults']} | {d['two'] for d in cat['defaults']} | {c['id'] for c in cat['custom']} or \
            __import__('roborun_sdk.arm_profiles', fromlist=['parse']).parse(pid) is not None

    def hardware(self):
        """The arms this robot runs and what is plugged in now: the configured arms, whether it simulates (only when its owner set
        that up with --sim), and the arm adapters on this computer. Read-only: adapters are listed, no motor is talked to."""
        at, value = getattr(self, '_hw', (0., None))
        if value is not None and time.monotonic() - at < 2.:
            return value
        try: c = json.loads((net.roborun_home() / 'robot.json').read_text())
        except (OSError, ValueError): c = {}
        try:
            from .arms import connect
            rows, _ = connect.adapter_rows()
        except Exception:
            rows = []
        connected = [dict(address=r.get('address'), bus=r.get('bus'), detail=r.get('detail')) for r in rows if r.get('address')]
        have = {r['address'] for r in connected}
        configured = {side: dict(arm=v.get('arm'), address=v.get('bus'))
                      for side, v in ((c.get('hardware') or {}).items()) if side in ('left', 'right') and isinstance(v, dict)}
        def present(addr):
            addr = str(addr or '')
            return addr in have or os.path.exists(addr) or bool(re.fullmatch(r'[\w.-]+\.[\w.-]+(:\d+)?', addr) and not addr.startswith('/'))
        for v in configured.values(): v['connected'] = present(v['address'])
        value = dict(simulated=bool(c.get('sim')), scene=c.get('sim'), configured=configured, connected=connected,
                     arms_ready=bool(configured) and all(v['connected'] for v in configured.values()),
                     body=(c.get('body') or {}).get('profile'),
                     bodies_connected=sorted({k for k, v in (c.get('body_agents') or {}).items() if isinstance(v, dict) and v.get('url')}
                                             | {k for k, v in (c.get('body_adapters') or {}).items() if isinstance(v, dict) and v.get('ref')}))
        self._hw = (time.monotonic(), value)
        return value

    def switch_profile(self, profile_id, command=None):
        """The page's Profiles switch, done the way the robot does it: `roborun up --profile <id>` (it writes this install's config and
        restarts the backend, the simulation or station, and this service). (status, body) for the page."""
        with self._switch_lock:
            if self.switch_job and self.switch_job['state'] == 'running':
                return 409, dict(error=dict(code='switch_running', message='A profile switch is already running'))
            code, inv = self._backend_json(PROFILES)
            if code != 200 or not isinstance(inv, dict):
                return 503, dict(error=dict(code='robot_unavailable', message='The robot is not answering: try again in a moment'))
            if profile_id is None:                                   # Use connected arms: leave the simulation for the arms set up
                hw = self.hardware()
                if not hw['simulated']:
                    return 409, dict(error=dict(code='not_simulated', message='This robot already runs its arms'))
                if not hw['configured']:
                    return 409, dict(error=dict(code='arms_not_set_up' if hw['connected'] else 'no_arms', message=(
                        'Arms are plugged in but not set up: run roborun setup on this computer' if hw['connected'] else 'No arms connected')))
                if not hw['arms_ready']:
                    return 409, dict(error=dict(code='no_arms', message='The arms set up on this robot are not connected'))
            elif not self._known_profile(profile_id, inv.get('active_id')):
                return 404, dict(error=dict(code='unknown_profile', message=f'This robot has no profile {profile_id!r}'))
            if profile_id is not None and inv.get('active_id') == profile_id:
                return 409, dict(error=dict(code='already_active', message='That profile is already running'))
            _, status = self._backend_json('/v1/status')
            if isinstance(status, dict) and status.get('task'):
                return 409, dict(error=dict(code='task_running', message='A task is running: stop it before switching the robot'))
            hw = self.hardware()
            from .body import profiles as body_profiles
            if profile_id is not None and body_profiles.is_body(profile_id):
                if not hw['simulated'] and profile_id not in hw['bodies_connected']:     # a real humanoid: only one connected here
                    return 409, dict(error=dict(code='body_not_connected', message=(
                        f'This robot has no {profile_id} connected: connect it on the robot once (roborun up --profile {profile_id} '
                        '--body my_body.py:MyBody, or --body-agent http://ROBOT:PORT --body-agent-key-file FILE), or set the robot up '
                        'with --sim to try it in simulation.')))
            elif not hw['simulated'] and not hw['arms_ready']:     # real arms only: never a simulation nobody set up
                gone = [f"{side} {v['arm']} at {v['address']}" for side, v in hw['configured'].items() if not v['connected']]
                return 409, dict(error=dict(code='no_arms', message=('No arms connected (' + '; '.join(gone) + '): plug them in, then switch.'
                                                                     if gone else 'No arms are set up on this robot: run roborun setup on it.')))
            if not hw['simulated'] and profile_id is not None and not body_profiles.is_body(profile_id):   # real arms: the profile is the arms plugged in
                try:
                    from .arm_profiles import runtime_profile
                    from .arms.settings import _profile_for
                    fits = runtime_profile(_profile_for({s: dict(arm=v['arm']) for s, v in hw['configured'].items()})) == profile_id
                except (Exception, SystemExit):
                    fits = False
                if not fits:
                    return 409, dict(error=dict(code='other_arms', message='This robot runs the arms plugged into it: plug in those arms, '
                                                                           'or set the robot up with --sim to try other arms in simulation.'))
            logs = net.roborun_home() / 'logs'; logs.mkdir(parents=True, exist_ok=True)
            log = open(logs / 'profile-switch.log', 'ab')
            # the page names the runtime's profile id: passed as is (the names people type, `--profile yam`, are mapped to ids)
            argv = [sys.executable, '-m', 'roborun_sdk.cli', 'up'] + (['--no-sim'] if profile_id is None else ['--profile-id', profile_id])
            proc = subprocess.Popen(command or self.switch_command or argv, stdin=subprocess.DEVNULL,
                                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            job = self.switch_job = dict(state='running', profile_id=profile_id or inv.get('active_id'), started_at=time.time(),
                                         message=(f'Restarting the robot with the {profile_id} profile.' if profile_id else
                                                  'Restarting the robot on its connected arms.') + ' The page reconnects by itself.')
            def watch():
                code_ = proc.wait(); log.close()
                try: tail = (logs / 'profile-switch.log').read_text(errors='replace').strip().splitlines()[-1:]
                except OSError: tail = []
                job.update(state='done' if code_ == 0 else 'failed', exit_code=code_,
                           message=('Switched to ' + (profile_id or 'the connected arms')) if code_ == 0 else (tail[0][:300] if tail else 'The profile switch did not finish'))
            threading.Thread(target=watch, daemon=True, name='profile-switch').start()
            return 202, dict(inv, configured_id=profile_id, switch_job=job)

    # ---------------------------------------------------------------- policy runs: tasks of this robot the backend does not drive
    def policy_stop(self, task_id):
        """The page's per-task Stop when the task is a policy run (roborun_sdk.policy.control): (status, body), or None when
        it is not one (the backend answers it). Any failure looking for the run (a damaged record, a robot.json that is not
        an object) is None too, so the Stop still goes on to the backend, as for any task."""
        from .policy import control
        try:
            if control.policy_record(task_id) is None:
                return None
        except Exception:
            return None
        try:
            rec = control.stop_run(task_id)
        except control.StopNotConfirmed as exc:
            return 503, dict(error=dict(code='stop_not_confirmed', message=str(exc), retryable=True))
        except (KeyError, IndexError) as exc:              # a damaged record, not "no such policy run"
            return self._stop_damaged_run(task_id, exc)
        except LookupError:                                # not a policy run after all: the backend answers
            return None
        except Exception as exc:
            return self._stop_damaged_run(task_id, exc)
        code, view = self._backend_json(f'/v1/tasks/{task_id}')
        if code != 200 or not isinstance(view, dict):
            try:
                return 200, control.view(rec)
            except Exception:
                return 200, rec
        if rec.get('status') == 'stopping' and view.get('status') == 'running':
            view = dict(view, status='stopping')
        return 200, view

    def _stop_damaged_run(self, task_id, exc):
        """A policy run whose Stop could not be worked out here: the backend's own Stop all the same; when the backend does not
        take it either, the Stop is not confirmed (never a quiet 'no such task' while the run may still move)."""
        conn = http.client.HTTPConnection(self.backend_host, self.backend_port, timeout=10)
        try:
            conn.request('POST', f'/v1/tasks/{task_id}/stop', body=b'{}', headers={'Content-Type': 'application/json'})
            r = conn.getresponse()
            code, out = r.status, json.loads(r.read() or b'{}')
        except (OSError, http.client.HTTPException, ValueError):
            code, out = None, None
        finally:
            conn.close()
        if code == 200:
            return code, out
        message = (f"the Stop of policy run {task_id} could not be sent to the arms' station ({exc!r}); use roborun stop, "
                   "or the arm's power switch if anything moves")
        return 503, dict(error=dict(code='stop_not_confirmed', message=message, retryable=True))

    def stop_status(self):
        """GET /v1/stop/status with this computer's policy runs counted as the running task: (status, body)."""
        code, out = self._backend_json(STOP_STATUS)
        if code is None:
            return 502, dict(error='the robot backend is not answering')
        if code != 200:
            return code, out
        try:
            from .policy import control
            return code, control.merge_stop_status(out)
        except Exception:                                  # the policy runs cannot be read: the backend's own answer
            return code, out

    def status(self):
        """GET /v1/status with this computer's policy runs: when the backend names no task, a running policy run is the task
        (the backend does not drive policy runs). (status, body)."""
        code, out = self._backend_json(STATUS)
        if code is None:
            return 502, dict(error='the robot backend is not answering')
        if code != 200:
            return code, out
        try:
            from .policy import control
            return code, control.merge_status(out)
        except Exception:                                  # the policy runs cannot be read: the backend's own answer
            return code, out

    def stop_robot(self, target, headers, body):
        """POST /v1/stop: sent to the backend first. When the backend cannot be reached, has not answered within
        STOP_CONFIRM_S, or answers with anything but a 2xx JSON answer (an error, a broken or non-JSON reply), the policy
        runs on this computer are stopped at the arms' station without it (the robot-wide hold first). (status, body); a run this Stop stopped is named in stopped_tasks."""
        try:
            from .policy.record import utc_now
            since = utc_now()
        except Exception:
            since = None
        box, done = {}, threading.Event()

        def send():
            conn = http.client.HTTPConnection(self.backend_host, self.backend_port, timeout=5)
            try:
                conn.connect(); conn.sock.settimeout(READ_S)
                conn.request('POST', target, body=body or None, headers=headers)
                r = conn.getresponse()
                box['answer'] = (r.status, r.read())
            except (OSError, http.client.HTTPException) as exc:
                box['down'] = exc
            finally:
                conn.close()
                done.set()
        threading.Thread(target=send, name='door-stop', daemon=True).start()
        finished = done.wait(STOP_CONFIRM_S)                # read once: the send thread may finish just after this
        answer = box.get('answer') if finished else None
        out = _json_or_text(answer[1]) if answer is not None else None
        if answer is not None and 200 <= answer[0] < 300 and isinstance(out, dict) and 'error' not in out:
            return answer[0], self._stopped_runs_named(out, since)   # the backend confirmed it
        # anything else is not the backend taking the Stop (a Stop is never refused): the policy runs here do not need the
        # backend to move, so they are stopped at the arms' station; with none, the backend's own answer is passed on
        if not finished:
            why = f"Roborun's backend has not confirmed the Stop within {STOP_CONFIRM_S:g} s"
        elif answer is None:
            why = None
        elif answer[0] >= 300:
            why = f"Roborun's backend answered the Stop with an error ({answer[0]})"
        else:
            why = "Roborun's backend answered the Stop without saying what it stopped"
        instead = self.stop_without_backend(why)
        if instead is not None:
            return instead
        done.wait(READ_S + 10)                             # no policy run here: the backend's own answer, as before
        answer = box.get('answer')
        if answer is None:
            return 502, dict(error='the robot backend is not answering')
        return answer[0], _json_or_text(answer[1])

    def _stopped_runs_named(self, out, since):
        if since is None or not isinstance(out, dict):
            return out
        try:
            from .policy import control
            runs = [r for r in control.stopped_since(since) if isinstance(r, str)]
        except Exception:
            return out
        if not runs:
            return out
        return dict(out, stopped_tasks=list(dict.fromkeys(list(out.get('stopped_tasks') or []) + runs)))

    def stop_without_backend(self, why=None):
        """POST /v1/stop while the backend does not answer (or answers too late, or with an error: `why`): the policy runs on
        this computer are stopped at the arms' station (they do not need the backend to move). (status, body), or None when
        none runs or none can be read."""
        try:
            from .policy import control
            result = control.stop_all()                    # finds the runs itself; nothing of ours runs: ids == []
        except Exception:
            return None
        if not result['ids']:
            return None
        message = control.stop_all_message(result, why or control.BACKEND_DOWN)
        if result['failed'] and not result['held']:
            return 503, dict(state='not_confirmed', confirmed=False, stopped_tasks=[r for r in result['ids'] if r in result['stopped']],
                             arms={}, message=message, error=dict(code='stop_not_confirmed', message=message, retryable=True))
        return 200, dict(state='stopping', confirmed=False, stopped_tasks=list(result['ids']), arms={}, message=message)

    def hello(self):
        return dict(roborun=1, key=self.key, robot=self.robot, account=self.account, ready=self.backend_state())

    def _backend_status(self, path):
        conn = http.client.HTTPConnection(self.backend_host, self.backend_port, timeout=1.5)
        try:
            conn.request('GET', path); return conn.getresponse().status
        except (OSError, http.client.HTTPException):
            return None
        finally:
            conn.close()

    def backend_state(self):
        """Whether the robot's backend answers, looked up at most once a second. (The page runs no code from the robot.)"""
        at, ready = self._backend
        if time.monotonic() - at > 1.:
            status = self._backend_status('/v1/settings')
            ready = status is not None and status < 500
            self._backend = (time.monotonic(), ready)
        return ready

    def bind(self):
        self.server = _Server(('127.0.0.1', self.port), _handler(self))
        self.port = self.server.server_address[1]
        return self

    def start(self):
        """Serve in the background. A busy port (a local service that is still stopping) is retried."""
        def run():
            said = False
            while self.server is None:
                try:
                    self.bind()
                except OSError as exc:
                    if not said:
                        print(json.dumps(dict(event='local_page_port_busy', port=self.port, error=str(exc)[:120])), flush=True); said = True
                    time.sleep(2.)
            print(json.dumps(dict(event='local_page_ready', address=f'127.0.0.1:{self.port}')), flush=True)
            self.server.serve_forever()
        threading.Thread(target=run, daemon=True, name='local-page').start()
        return self

    def stop(self):
        if self.server is not None:
            self.server.shutdown(); self.server.server_close()


def _json_or_text(raw):
    """A backend's answer body for the door to answer with: its JSON, or its text as an error."""
    try:
        return json.loads(raw) if raw else None
    except ValueError:
        return dict(error=(raw or b'').decode('utf-8', 'replace').strip()[:300])


def _handler(proxy):
    prefix = f'/k/{proxy.key}/'.encode()
    view_prefix = f'/v/{proxy.view_key}/'.encode()
    viewer_dir = Path(__file__).resolve().parent / 'viewer'

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'
        server_version, sys_version = 'roborun', ''
        timeout = 75                               # an idle or stalled browser connection ends
        disable_nagle_algorithm = True

        def log_message(self, *a):                 # no request lines: they would carry the key
            pass

        # ---------------------------------------------------------------- answers
        def _cors(self, origin):
            if origin:
                self.send_header('Access-Control-Allow-Origin', origin)
                self.send_header('Access-Control-Expose-Headers', EXPOSE)
            self.send_header('Vary', 'Origin')

        def _safety(self, content_type=''):
            if not content_type.lower().startswith('multipart/'):        # browsers refuse a camera stream marked nosniff in an <img>
                self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.send_header('Content-Security-Policy', 'sandbox')       # nothing served here runs as a page of this address

        def _plain(self, status, origin=None, body=None):
            """A short answer. `origin` only once the Origin is known to be allowed."""
            data = json.dumps(body).encode() if body is not None else b''
            self.send_response(status); self._cors(origin); self._safety()
            self.send_header('Cache-Control', 'no-store')
            if data: self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            if status >= 400:
                self.send_header('Connection', 'close'); self.close_connection = True
            self.end_headers()
            if data and self.command != 'HEAD': self.wfile.write(data)

        # ---------------------------------------------------------------- who is asking
        def _admit(self):
            """(ok, allowed origin or None). Refuses a wrong Host and any Origin that is not exactly allowed."""
            host, origin = self.headers.get_all('Host') or [], self.headers.get_all('Origin') or []
            if len(host) != 1 or host[0] not in (f'127.0.0.1:{proxy.port}', f'localhost:{proxy.port}'):
                self._plain(421); return False, None
            if not origin:
                return True, None
            if len(origin) != 1 or origin[0] not in proxy.origins:
                self._plain(403); return False, None
            return True, origin[0]

        def _target(self):
            """The backend path of a keyed request, or None."""
            raw = self.path.encode('latin-1', 'replace')
            if not hmac.compare_digest(raw[:len(prefix)], prefix):
                return None
            rest = self.path[len(prefix) - 1:]
            path = rest.split('?', 1)[0]
            return rest if path.startswith(API_PREFIXES) and not BAD_PATH.search(path) else None

        def do_OPTIONS(self):
            ok, origin = self._admit()
            if not ok: return
            method = (self.headers.get('Access-Control-Request-Method') or '').strip()
            wanted = [h.strip().lower() for h in (self.headers.get('Access-Control-Request-Headers') or '').split(',') if h.strip()]
            path = self.path.split('?', 1)[0]
            if not origin or method not in METHODS or any(h not in REQUEST_HEADERS for h in wanted) \
                    or not (path == HELLO and method == 'GET' or path.startswith('/k/')):
                return self._plain(403)
            self.send_response(204); self._cors(origin); self._safety()
            self.send_header('Access-Control-Allow-Methods', ', '.join(METHODS))
            if wanted: self.send_header('Access-Control-Allow-Headers', ', '.join(wanted))
            self.send_header('Access-Control-Max-Age', '600')
            self.send_header('Vary', 'Access-Control-Request-Method, Access-Control-Request-Headers, '
                                     'Access-Control-Request-Private-Network, Access-Control-Request-Local-Network')
            if (self.headers.get('Access-Control-Request-Private-Network') or '').lower() == 'true':
                self.send_header('Access-Control-Allow-Private-Network', 'true')
            if (self.headers.get('Access-Control-Request-Local-Network') or '').lower() == 'true':
                self.send_header('Access-Control-Allow-Local-Network', 'true')
            self.send_header('Content-Length', '0'); self.end_headers()

        # ---------------------------------------------------------------- this robot's own viewer
        def _own_origins(self):
            return (f'http://127.0.0.1:{proxy.port}', f'http://localhost:{proxy.port}')

        def _view_request(self):
            """True when this request was for the viewer (answered here, either way)."""
            path = self.path.split('?', 1)[0]
            raw = self.path.encode('latin-1', 'replace')
            keyed = hmac.compare_digest(raw[:len(view_prefix)], view_prefix)
            if path not in VIEW_FILES and path != VIEW_SESSION and not keyed and not path.startswith('/v/'):
                return False
            host, origin = self.headers.get_all('Host') or [], self.headers.get_all('Origin') or []
            if len(host) != 1 or host[0] not in (f'127.0.0.1:{proxy.port}', f'localhost:{proxy.port}'):
                self._plain(421); return True
            site = (self.headers.get('Sec-Fetch-Site') or '').strip().lower()
            if len(origin) > 1 or (origin and origin[0] not in self._own_origins()) or site not in ('', 'none', 'same-origin'):
                self._plain(403); return True                  # only this page itself, or a person typing the address
            if path in VIEW_FILES:
                if self.command not in ('GET', 'HEAD'): return self._plain(405) or True
                name, ctype = VIEW_FILES[path]
                try:
                    data = (viewer_dir / name).read_bytes()
                except OSError:
                    return self._plain(404) or True
                self.send_response(200)
                self.send_header('Content-Type', ctype); self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store'); self.send_header('X-Content-Type-Options', 'nosniff')
                self.send_header('Referrer-Policy', 'no-referrer'); self.send_header('X-Frame-Options', 'DENY')
                self.send_header('Content-Security-Policy', VIEW_CSP); self.end_headers()
                if self.command != 'HEAD': self.wfile.write(data)
                return True
            if path == VIEW_SESSION:
                if self.command != 'GET': return self._plain(405) or True
                return self._plain(200, None, dict(key=proxy.view_key, robot=proxy.robot, ready=proxy.backend_state())) or True
            if not keyed:
                return self._plain(404) or True
            target = self.path[len(view_prefix) - 1:]
            api_path = target.split('?', 1)[0]
            if BAD_PATH.search(api_path):
                return self._plain(404) or True
            if self.command == 'GET' and api_path == STOP_STATUS:
                code, out = proxy.stop_status()
                return self._plain(code, None, out) or True
            if self.command == 'GET' and api_path == STATUS:
                code, out = proxy.status()
                return self._plain(code, None, out) or True
            if self.command in ('GET', 'HEAD') and VIEW_GET.fullmatch(api_path):
                self._forward(target, None); return True
            if self.command == 'POST' and api_path in VIEW_POST:
                self._stop(target, origin[0] if origin else self._own_origins()[0], cors=False); return True
            return self._plain(403, None, dict(error=dict(code='forbidden', retryable=False,
                                                          message='The viewer only reads; start tasks from the page or the local API'))) or True

        def _any(self):
            if self._view_request(): return
            ok, origin = self._admit()
            if not ok: return
            if self.path.split('?', 1)[0] == HELLO:
                if not origin or self.command != 'GET':
                    return self._plain(403)
                return self._plain(200, origin, proxy.hello())
            target = self._target()
            if target is None:
                return self._plain(404, origin)
            if not origin and self.command not in ('GET', 'HEAD'):       # a state change always names its page
                return self._plain(403)
            path = target.split('?', 1)[0]; m = PROFILE_APPLY.fullmatch(re.sub(r'%2[bB]', '+', path))   # the page encodes a pair's '+'
            if self.command == 'POST' and path == COMPOSE:              # a custom pair: {left, right} arm ids (right may be empty)
                body = self._body()
                if body is None: return
                try:
                    want = json.loads(body or b'{}')
                    if not isinstance(want, dict): raise TypeError
                except (ValueError, TypeError):
                    return self._plain(400, origin, dict(error=dict(code='bad_request', message='Choose a left arm, a right arm, or both.')))
                pid, problem = compose_profile(want)
                if pid is None:
                    return self._plain(400, origin, dict(error=dict(code='no_profile', message=problem)))
                code, out = proxy.switch_profile(pid); return self._plain(code, origin, dict(out, profile_id=pid))
            if self.command == 'POST' and (m or path == USE_ARMS):       # switched here, on the robot (the backend only lists them)
                if self._body() is None: return
                code, out = proxy.switch_profile(m.group(1) if m and path != USE_ARMS else None); return self._plain(code, origin, out)
            stop = TASK_STOP.fullmatch(path)
            if self.command == 'POST' and stop:                          # a policy run: stopped at the station, by its id
                raw = self._body()
                if raw is None: return
                answer = proxy.policy_stop(stop.group(1))
                if answer is not None:
                    return self._plain(answer[0], origin, answer[1])
                return self._forward(target, origin, body=raw)
            if self.command == 'GET' and path == STOP_STATUS:
                code, out = proxy.stop_status()
                return self._plain(code, origin, out)
            if self.command == 'GET' and path == STATUS:
                code, out = proxy.status()
                return self._plain(code, origin, out)
            if self.command == 'POST' and path == '/v1/stop':
                return self._stop(target, origin)
            if path == PROFILES and self.command == 'GET':
                code, inv = proxy._backend_json(PROFILES)
                out = dict(inv if isinstance(inv, dict) else {}, hardware=proxy.hardware())
                try: out['catalog'] = proxy.profile_catalog(out.get('active_id'))
                except Exception: pass
                if proxy.switch_job: out['switch_job'] = proxy.switch_job
                return self._plain(code or 503, origin, out)
            self._forward(target, origin)

        do_GET = do_HEAD = do_POST = do_PUT = do_PATCH = do_DELETE = _any

        # ---------------------------------------------------------------- on to the backend
        def _body(self):
            if self.headers.get('Transfer-Encoding'):
                self._plain(411); return None
            try:
                size = int(self.headers.get('Content-Length') or 0)
            except ValueError:
                self._plain(400); return None
            if size < 0 or size > MAX_BODY:
                self._plain(413); return None
            return self.rfile.read(size) if size else b''

        def _stop(self, target, origin, cors=True):
            """POST /v1/stop: the backend first, this computer's policy runs at the arms' station when it fails or is slow."""
            body = self._body()
            if body is None: return
            headers = {k: self.headers[k] for k in self.headers.keys() if k.lower() in REQUEST_HEADERS}
            headers['Origin'] = origin
            headers[DOOR_HEADER] = '1'
            code, out = proxy.stop_robot(target, headers, body)
            return self._plain(code, origin if cors else None, out)

        def _forward(self, target, origin, cors=True, body=None, unreachable=None):
            """On to the backend. `body`: already read here; `unreachable`: () -> (status, body) or None, answered instead when
            the backend cannot be reached (nothing was sent)."""
            body = self._body() if body is None else body
            if body is None: return
            headers = {k: self.headers[k] for k in self.headers.keys() if k.lower() in REQUEST_HEADERS}
            if origin:                                     # the backend still refuses browsers on its native-only routes
                headers['Origin'] = origin
                headers[DOOR_HEADER] = '1'
            conn = http.client.HTTPConnection(proxy.backend_host, proxy.backend_port, timeout=5)
            try:
                try:
                    conn.connect(); conn.sock.settimeout(READ_S)
                    conn.request(self.command, target, body=body or None, headers=headers)
                    r = conn.getresponse()
                except (OSError, http.client.HTTPException):
                    instead = unreachable() if unreachable is not None else None
                    if instead is not None:
                        return self._plain(instead[0], origin if cors else None, instead[1])
                    return self._plain(502, origin, dict(error='the robot backend is not answering'))
                self.send_response(r.status); self._cors(origin if cors else None); self._safety(r.getheader('Content-Type') or '')
                sized = False
                for k, v in r.getheaders():
                    low = k.lower()
                    if low in RESPONSE_DROP or low.startswith('access-control-') or low == 'vary':
                        continue
                    sized = sized or low == 'content-length'
                    self.send_header(k, v)
                bodyless = self.command == 'HEAD' or r.status in (204, 304)
                if not sized and not bodyless:             # a live stream, or a length only the end of it tells
                    self.send_header('Connection', 'close'); self.close_connection = True
                self.end_headers()
                if not bodyless:
                    while True:
                        data = r.read1(65536)
                        if not data: break
                        self.wfile.write(data)
            except (OSError, http.client.HTTPException):   # the page left, or the backend went away mid-answer
                self.close_connection = True
            finally:
                conn.close()

    return Handler
