"""`roborun login`: get a key for this computer without copying one by hand.

This computer asks the API for a sign-in code and prints the link to approve it on app.roborun.app (signed in to your
account). `roborun login` then exits: the next `roborun up` waits for the approval and saves the new robot key in the key
file (the same file `roborun up` reads). `roborun login --wait` waits right there instead.
"""
import json
import socket
import time
from pathlib import Path


def _post(server, path, body, retry=False):
    """(status, reply). A dropped connection is (None, {}): the caller decides whether to keep trying."""
    from . import net
    try:
        status, _, payload = net.request('POST', server + path, body=body, timeout=30, retry=retry, tries=2 if retry else 1)
    except net.Offline as e:
        return None, {'error': {'code': 'offline', 'message': e.reason}}
    try:
        return status, json.loads(payload or b'{}')
    except ValueError:
        return status, {}


def _code(out):
    return ((out or {}).get('error') or {}).get('code')


def _say(text):
    print(text, flush=True)             # the link shows at once, also when the output goes to a log or a coding agent


SLOW_DOWN_S = 5          # RFC 8628: each slow_down adds this to the polling interval


def _pending_file():
    from . import robot
    return Path(robot.HOME) / 'login-pending.json'


def pending(file=None, clock=time.time):
    """The sign-in `roborun login` started and nobody has finished yet (for this key file), or None."""
    try:
        p = json.loads(_pending_file().read_text())
    except (OSError, ValueError):
        return None
    if not isinstance(p, dict) or p.get('expires_at', 0) <= clock():
        _pending_file().unlink(missing_ok=True)
        return None
    if file is not None and Path(p.get('file') or '').expanduser() != Path(file).expanduser():
        return None
    return p


def start(server, file: Path, name=None, out=_say, clock=time.time, keep=False):
    """Ask for a sign-in code and print the link to approve it. Returns what finish() needs; `keep`: also kept in a small file
    (0600, removed once used), so a later `roborun up` finishes it."""
    from .arms.settings import store_token
    server = server.rstrip('/')
    status, start = _post(server, '/v1/login/device', dict(name=name or socket.gethostname()), retry=True)
    if status is None:
        raise SystemExit(f"could not start sign-in: the Roborun API could not be reached ({start['error']['message']}). "
                         'Next: check this computer\'s internet connection, then run roborun login again')
    if status != 200:
        raise SystemExit(f'could not start sign-in ({status}): {((start or {}).get("error") or {}).get("message", "")}')
    out(f"Open {start['verification_uri_complete']} and approve this computer.")
    out("Sign in there with your Roborun key. Roborun is invite-only: if you don't have a key, email support@roborun.dev.")
    out(f"Check that the page shows the code {start['user_code']}.")
    p = dict(server=server, file=str(Path(file).expanduser()), device_code=start['device_code'], user_code=start['user_code'],
             url=start['verification_uri_complete'], expires_at=clock() + start.get('expires_in', 600),
             interval=max(1, int(start.get('interval', 3))))
    if keep:
        store_token(_pending_file(), json.dumps(p))               # 0600: the device code fetches the key once approved
    return p


def finish(p, out=_say, sleep=time.sleep, clock=time.time):
    """Wait for the approval of a started sign-in, then save the key. Raises SystemExit when it expires or is refused."""
    from .arms.settings import store_token
    try:
        return _wait(p['server'], Path(p['file']), p, p['expires_at'], p['interval'], False, out, sleep, clock, store_token)
    finally:                                                      # approved, expired or refused: never asked for again
        try:
            if json.loads(_pending_file().read_text()).get('device_code') == p['device_code']:
                _pending_file().unlink(missing_ok=True)
        except (OSError, ValueError, AttributeError):
            pass


def login(server, file: Path, name=None, out=_say, sleep=time.sleep, clock=time.time):
    """Sign in and wait for the approval (`roborun login --wait`, and the guided setup)."""
    return finish(start(server, file, name, out, clock), out, sleep, clock)


def _wait(server, file, start, deadline, interval, offline_said, out, sleep, clock, store_token):
    while clock() < deadline:
        sleep(interval)
        # the token call is not repeated inside one poll (a reply carrying the key that got lost cannot be asked for twice,
        # the next poll asks again); a dropped connection is simply one more poll
        status, got = _post(server, '/v1/login/device/token', dict(device_code=start['device_code']))
        if status == 200 and got.get('key'):
            file = Path(file).expanduser()
            if file.is_file() and file.read_text().strip() and file.read_text().strip() != got['key']:
                previous = file.with_name(file.name + '.previous')
                store_token(previous, file.read_text().strip())
                out(f'The key that was in {file} is kept in {previous}.')
            store_token(file, got['key'])
            out(f"Signed in. A new key ({got.get('prefix', 'robot key')}) is saved in {file}.")
            return got
        if status is None:
            if not offline_said:
                out('The Roborun API cannot be reached right now; still waiting for the approval...')
                offline_said = True
            interval = min(30, interval + 1)
            continue
        offline_said = False
        if _code(got) == 'slow_down':
            interval += SLOW_DOWN_S
            continue
        if _code(got) != 'authorization_pending' and status not in (429, 500, 502, 503, 504):
            raise SystemExit(f"sign-in failed: {((got or {}).get('error') or {}).get('message', status)}")
    raise SystemExit('the sign-in code expired: run roborun login again')


def cmd_login(a):
    from . import robot
    from .arms import settings
    file = Path(a.key_file) if a.key_file else settings.token_path(a)
    if a.wait:
        login(a.server or robot.SERVER, file, name=a.name)
        return 0
    start(a.server or robot.SERVER, file, name=a.name, keep=True)
    _say('Once it is approved, roborun up saves the key and starts (or wait here: roborun login --wait).')
    return 0


def add_parser(sub):
    p = sub.add_parser('login', help='sign this computer in: prints a link to approve on app.roborun.app; roborun up then saves the key')
    p.add_argument('--name', help='what to call this key (default: this computer\'s name)')
    p.add_argument('--key-file', help='where to save the key (default: the file roborun up reads)')
    p.add_argument('--server', help='the API (default: https://api.roborun.app, or ROBORUN_SERVER)')
    p.add_argument('--wait', action='store_true', help='wait here until the sign-in is approved, then save the key')
    p.set_defaults(func=cmd_login)
