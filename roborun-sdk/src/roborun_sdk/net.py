"""The SDK's calls to the hosted API: one User-Agent, one retry rule, plain words for what went wrong.

    status, headers, body = request('GET', server + '/v1/keys', key)            # retried: GET / HEAD / PUT
    request('POST', server + '/v1/login/device/token', body=..., retry=True)     # only when the call is safe to repeat

A dropped connection, a timeout and 408 / 425 / 429 / 500 / 502 / 503 / 504 are retried (TRIES in all, exponential backoff
with jitter, Retry-After honoured up to RETRY_AFTER_CAP_S). After the last try a dropped connection raises Offline; an HTTP
answer is returned as it is, and problem(status, body) says it in one line. Importing this module performs no I/O.

Releases: every call names this release (User-Agent and X-Roborun-SDK: the build id), and the API names the newest and the
oldest supported release on every reply (X-Roborun-SDK-Latest / -Minimum). Those are kept in <roborun home>/update.json;
update_status() and update_notice() read them (`update available: <build>, run: roborun upgrade`). Below the minimum the
API answers 426 sdk_upgrade_required, except for what a robot needs to stop and upgrade.
"""
from __future__ import annotations

import http.client
import ipaddress
import json
import os
import platform
import random
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

from . import __version__

RETRY_STATUS = (408, 425, 429, 500, 502, 503, 504)
TRIES, BACKOFF_S, BACKOFF_CAP_S, RETRY_AFTER_CAP_S = 3, 0.5, 8.0, 15.0
BUILD_FILE = 'build.json'          # install.sh keeps the release's manifest beside the SDK's Python (~/.roborun/build.json)


class Offline(OSError):
    """The API could not be reached at all (no network, DNS, a reset, a timeout), after the retries."""

    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


def build():
    """This release's build id (e.g. 20261119-172831-b6abbc5e80), or None for a source checkout. A release wheel carries it
    (roborun_sdk/_release.py, stamped by the release build, so a `pip install roborun-sdk==<version>` names it too); an
    install without it falls back to the manifest install.sh keeps beside the Python (build.json)."""
    try:
        from ._release import BUILD
        return BUILD
    except ImportError:
        pass
    for folder in (Path(sys.prefix).parent, Path(sys.prefix)):
        try:
            return json.loads((folder / BUILD_FILE).read_text()).get('build')
        except (OSError, ValueError, AttributeError):
            continue
    return None


def runtime_version():
    """The robot runtime `roborun up` installed (its RUNTIME.json), or None."""
    root = Path(os.environ.get('MANIFEST_DREAMAPI') or Path.home() / 'roborun' / 'robot' / 'runtime').expanduser()
    try:
        return json.loads((root / 'RUNTIME.json').read_text()).get('version')
    except (OSError, ValueError, AttributeError):
        return None


def user_agent(runtime=None):
    """roborun-sdk/<version> (build <id>; runtime <version>; python <x.y>; <os>): the API can tell which client calls it.
    A plain named client also matters: the API's edge refuses Python's default User-Agent."""
    runtime = runtime or runtime_version()
    parts = [f'build {build() or "source"}']
    if runtime:
        parts.append(f'runtime {runtime}')
    parts += [f'python {sys.version_info[0]}.{sys.version_info[1]}', f'{platform.system().lower() or sys.platform}']
    return f'roborun-sdk/{__version__} ({"; ".join(parts)})'


# ---- releases: what the API says is the newest and the oldest supported Roborun
UPDATE_FILE = 'update.json'
NOTICE_EVERY_S = 86400
UPGRADE = 'roborun upgrade'


def roborun_home():
    """This install's Roborun home: ROBORUN_HOME (set but empty counts as unset), else $HOME/.config/roborun. Always absolute
    and with ~ expanded, so the systemd units roborun up starts (which run in the account's home folder, not this shell's
    folder) get the same folder as the command that started them; symlinks are kept as given."""
    return Path(os.path.abspath(os.path.expanduser(os.environ.get('ROBORUN_HOME') or Path.home() / '.config' / 'roborun')))


def client_headers(runtime=None):
    """The headers that name this client on every call to the API."""
    h = {'User-Agent': user_agent(runtime)}
    if build():
        h['X-Roborun-SDK'] = build()
    return h


def _read_update():
    try:
        out = json.loads((roborun_home() / UPDATE_FILE).read_text())
        return out if isinstance(out, dict) else {}
    except (OSError, ValueError):
        return {}


def _write_update(state):
    path = roborun_home() / UPDATE_FILE
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(path.name + f'.{os.getpid()}.tmp')
        tmp.write_text(json.dumps(state, indent=1))
        os.replace(tmp, path)
    except OSError:
        pass


def note_release(headers):
    """Keep the newest / oldest supported release an API reply named (only when they change)."""
    if headers is None:
        return
    latest, minimum = headers.get('X-Roborun-SDK-Latest'), headers.get('X-Roborun-SDK-Minimum')
    if not latest and not minimum:
        return
    state = _read_update()
    if state.get('latest') == latest and state.get('minimum') == minimum:
        return
    state.update(latest=latest, minimum=minimum, seen=round(time.time()))
    _write_update(state)


def update_status():
    """{available, latest, required, current}: available when the API named a newer release than this one; required when this
    one is older than the oldest it supports. A source checkout (no build) is never told to update."""
    state, current = _read_update(), build()
    latest, minimum = state.get('latest'), state.get('minimum')
    return dict(available=bool(current and latest and current < latest), latest=latest or None,
                required=bool(current and minimum and current < minimum), current=current)


def update_notice(daily=True, now=None):
    """`update available: <build>, run: roborun upgrade` (or that an upgrade is required), at most once a day when `daily`;
    None when this is current."""
    st = update_status()
    if not st['available'] and not st['required']:
        return None
    now = time.time() if now is None else now
    state = _read_update()
    if daily and not st['required'] and state.get('said') is not None and now - float(state['said']) < NOTICE_EVERY_S:
        return None
    state['said'] = round(now)
    _write_update(state)
    if st['required']:
        return f"this Roborun ({st['current']}) is no longer supported by the API: run {UPGRADE}"
    return f"update available: {st['latest']}, run: {UPGRADE}"


def reason(exc):
    """A dropped connection in plain words (no errno, no address)."""
    text = str(getattr(exc, 'reason', None) or exc).lower()
    for needle, said in (('timed out', 'timed out'), ('timeout', 'timed out'), ('reset', 'connection reset'),
                         ('refused', 'connection refused'), ('name or service', 'name not resolved'),
                         ('temporary failure in name', 'name not resolved'), ('nodename', 'name not resolved'),
                         ('unreachable', 'network unreachable'), ('ssl', 'secure connection failed'),
                         ('certificate', 'secure connection failed'), ('broken pipe', 'connection dropped'),
                         ('remote end closed', 'connection dropped'), ('incomplete', 'connection dropped')):
        if needle in text:
            return said
    return 'network error'


def retry_after(headers, body=b''):
    try:
        err = (json.loads(body or b'{}') or {}).get('error')
    except (ValueError, AttributeError):
        err = None
    for value in ((err or {}).get('retry_after_s') if isinstance(err, dict) else None, headers.get('Retry-After') if headers else None):
        try:
            if value is not None:
                return max(0., float(value))
        except (TypeError, ValueError):
            pass
    return None


def backoff(attempt, wait=None):
    if wait is not None:
        return min(RETRY_AFTER_CAP_S, wait)
    return min(BACKOFF_CAP_S, BACKOFF_S * 2 ** attempt) * (0.5 + random.random() / 2)


def request(method, url, key=None, body=None, data=None, headers=None, timeout=30, retry=None, tries=TRIES, sleep=time.sleep):
    """(status, headers, body bytes). `retry` (default: GET, HEAD, PUT) says the call is safe to repeat."""
    hdrs = client_headers()
    if key:
        hdrs['Authorization'] = 'Bearer ' + key
    if body is not None:
        data = json.dumps(body).encode()
        hdrs['Content-Type'] = 'application/json'
    hdrs.update(headers or {})
    req = urllib.request.Request(url, data=data, method=method, headers=hdrs)
    tries = tries if (method in ('GET', 'HEAD', 'PUT') if retry is None else retry) else 1
    for attempt in range(tries):
        last = attempt + 1 >= tries
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                note_release(r.headers)
                return r.status, r.headers, r.read()
        except urllib.error.HTTPError as e:
            note_release(e.headers)
            try:
                payload = e.read()
            except (OSError, http.client.HTTPException):
                payload = b''
            if e.code in RETRY_STATUS and not last:
                wait = retry_after(e.headers, payload)
                if e.code != 429 or wait is None or wait <= RETRY_AFTER_CAP_S:
                    sleep(backoff(attempt, wait))
                    continue
            return e.code, e.headers, payload
        except (urllib.error.URLError, OSError, http.client.HTTPException) as e:
            if last:
                raise Offline(reason(e)) from None
            sleep(backoff(attempt))
    raise Offline('network error')                         # not reached


def error_of(payload):
    """The API's {"error": {"code", "message"}} from a reply body, or {}."""
    try:
        err = json.loads(payload or b'{}').get('error')
    except (ValueError, AttributeError):
        return {}
    return err if isinstance(err, dict) else ({'message': err} if isinstance(err, str) else {})


def problem(status, payload, what='the Roborun API'):
    """One line for an HTTP refusal, in the API's own words when it gave some (never an HTML page)."""
    err = error_of(payload)
    code, said = err.get('code') or '', (err.get('message') or '').strip()
    if status == 401:
        return f'{what} does not accept this key ({said or "unknown or revoked"}). Next: run roborun login, or put a new key in the key file'
    if status == 403 and code in ('read_only_key', 'scope'):
        return f'this key cannot run a robot ({said or "read-only key"}). Next: use a robot key (roborun login)'
    if status == 426 or code in ('client_too_old', 'sdk_upgrade_required'):
        return f'this Roborun version is no longer supported by {what}. Next: {UPGRADE}'
    if status == 429:
        wait = retry_after(None, payload)
        return f'{what} has too many requests from this key' + (f': try again in {wait:.0f} s' if wait is not None else ': try again shortly')
    if status >= 500:
        return f'{what} is having trouble (HTTP {status}{": " + said if said else ""}). Next: try again in a minute'
    return f'{what} refused the request (HTTP {status}{": " + said if said else ""})'


def offline_text(exc, what='the Roborun API'):
    return f'{what} could not be reached ({exc.reason if isinstance(exc, Offline) else reason(exc)}). Next: check this computer\'s internet connection'


def secure(url):
    """https, or plain http to this computer only (the key travels in every request)."""
    parts = urllib.parse.urlsplit(url)
    if parts.scheme == 'https':
        return True
    host = (parts.hostname or '').lower()
    if host == 'localhost':
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def check_server(url, insecure=False):
    """Refuse a server address that would send the key unencrypted to another computer."""
    if secure(url):
        return url
    if insecure:
        print(f'warning: {url} is plain http: your key travels unencrypted (--insecure)', file=sys.stderr)
        return url
    raise SystemExit(f'{url} is plain http to another computer: your key would travel unencrypted. Use https, '
                     'or pass --insecure on a network you trust')
