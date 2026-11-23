"""Where a simulation runs: never in the live robot's runtime, camera socket or station port.

A simulation writes into the runtime it runs from (the overhead camera's exact calibration in its log folder, motion
sessions, the managed config.json of the profile it simulates) and serves a station and a camera broker. Run against the
live robot's runtime and socket it would replace the robot's newest calibration with the simulated one and take the robot's
camera socket away. So:

- the runtime: a copy of the live one in <Roborun home>/sim/ (robot/runtime and its helper folders, without log/), refreshed
  when the live one changes; the copy keeps its own log/, and its config.json follows the live one's only when that one
  changes. `roborun up --sim` runs its backend and its simulation from there too. `roborun sim --runtime FOLDER` simulates
  another runtime the same way, from its own copy in <Roborun home>/sim/runtimes/<12 hex>/: a simulation never writes into
  a runtime folder that is not a simulation's own.
- the camera socket: bare `roborun sim` serves <runtime dir>/manifest/[roborun-<8 hex>/]sim-camera.sock. `roborun up --sim` is
  the robot itself as a simulation, so it serves the robot's own camera.sock, and only when no camera service serves it.
- the station: bare `roborun sim` listens on SIM_PORT (8191); `roborun up --sim` on the robot's 8091, only when it is free.
  Bare `roborun sim` never takes the robot's 8091 or a robot's camera.sock, even when they are free at that moment (the live
  robot may be between restarts): only `roborun up --sim` does, after it checked that the live robot is not running.

A socket is only ever removed when nothing serves it and a Roborun simulation made it (its .owner file names the same file);
a served port or socket is never taken over. Nothing here starts, stops or writes to a live service.
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import socket
import stat
import urllib.request
from pathlib import Path

SIM_PORT = 8191                     # bare `roborun sim`'s station: beside the robot's 8091, never on it
SKIP_ANYWHERE = {'.git', '__pycache__', 'node_modules', '.pytest_cache', '.mypy_cache'}
OWN = ('config.json', 'log')        # the copy's own: never overwritten from the live runtime
OWNER = '.owner'                    # <socket>.owner: written by the simulation that made the socket


def folder(home=None):
    from . import robot
    return Path(home or robot.HOME) / 'sim'


def runtime(home=None, base=None):
    """The simulation's own runtime: <Roborun home>/sim/robot/runtime (the bundle's layout, so its helpers sit beside it), or
    <base>/robot/runtime for the copy of another runtime (copy_folder)."""
    return Path(base or folder(home)) / 'robot' / 'runtime'


def copy_folder(source, home=None):
    """Where the simulation's copy of the runtime `source` lives: <home>/sim for the live robot's runtime (the one `roborun up
    --sim` uses too), <home>/sim/runtimes/<12 hex of its path> for any other (`roborun sim --runtime FOLDER`)."""
    if same(source, live_runtime()):
        return folder(home)
    return folder(home) / 'runtimes' / hashlib.sha256(os.path.realpath(str(source)).encode()).hexdigest()[:12]


def is_copy(path, home=None):
    """True when path is inside this install's simulation folder: a simulation's own runtime, used as it is."""
    top = os.path.realpath(str(folder(home)))
    here = os.path.realpath(str(path))
    return here == top or here.startswith(top + os.sep)


def robot_socket(path):
    """True when path is a robot's camera socket: this install's (camera_socket(), $MANIFEST_CAMERA_SOCKET included) or any
    Roborun install's camera.sock under <runtime dir>/manifest/ (the default install's, or roborun-<8 hex>/camera.sock)."""
    from .cameras import _socket_path
    if same(path, _socket_path()):
        return True
    p = Path(os.path.realpath(str(path)))
    if p.name != 'camera.sock':
        return False
    base = Path(os.path.realpath(os.path.join(os.environ.get('XDG_RUNTIME_DIR') or f'/tmp/manifest-{os.getuid()}', 'manifest')))
    return p.parent == base or (p.parent.parent == base and p.parent.name.startswith('roborun-'))


def camera_socket():
    """Bare `roborun sim`'s camera socket: sim-camera.sock beside this install's camera.sock ($MANIFEST_CAMERA_SOCKET, which
    names the robot's, is not followed)."""
    from .robot import install_suffix
    base = os.environ.get('XDG_RUNTIME_DIR') or f'/tmp/manifest-{os.getuid()}'
    path = os.path.join(base, 'manifest', *(['roborun' + install_suffix()] if install_suffix() else []))
    os.makedirs(path, mode=0o700, exist_ok=True)
    return os.path.join(path, 'sim-camera.sock')


def live_runtime(c=None):
    """The live robot's runtime: robot.json's, else the default install's place for it."""
    from . import robot
    if c is None:
        try: c = json.loads(robot.CONFIG.read_text())
        except (OSError, ValueError): c = {}
    return Path(c.get('manifest') or robot._manifest_root()).expanduser().absolute()


def same(a, b):
    return os.path.realpath(str(a)) == os.path.realpath(str(b))


# ---- the copy -------------------------------------------------------------------------------------------------------------

def _pairs(live, home=None, base=None):
    """(source folder, copy folder) for the runtime and its helper folders beside it (depth, harness, supervisor)."""
    from .robot import INSTALL_DIRS
    live = Path(live)
    base = Path(base or folder(home))
    out = [(live, runtime(base=base))]
    if len(live.parents) < 2:                       # a runtime at /rt: nothing beside it
        return out
    for name in sorted(INSTALL_DIRS - {'robot'}):
        if (live.parents[1] / name).is_dir():
            out.append((live.parents[1] / name, base / name))
    return out


def _files(src, top_skip=()):
    """Every file (and symlink) under src as relative paths, without the skipped folders."""
    out = []
    for here, dirs, files in os.walk(src):
        rel = os.path.relpath(here, src)
        dirs[:] = [d for d in dirs if d not in SKIP_ANYWHERE and not (rel == '.' and d in top_skip)
                   and not os.path.islink(os.path.join(here, d))]
        for d in os.listdir(here):                    # a symlinked folder is copied as the link, not walked
            if d not in SKIP_ANYWHERE and os.path.islink(os.path.join(here, d)) and os.path.isdir(os.path.join(here, d)):
                if not (rel == '.' and d in top_skip):
                    out.append(os.path.normpath(os.path.join(rel, d)))
        for f in files:
            if not (rel == '.' and f in top_skip):
                out.append(os.path.normpath(os.path.join(rel, f)))
    return out


def fingerprint(live):
    """What the copy was made from: every source file's path, size and modification time (no file is read)."""
    h = hashlib.sha256()
    for i, (src, _) in enumerate(_pairs(live)):
        for rel in sorted(_files(src, OWN if i == 0 else ('log',))):
            try: st = os.lstat(os.path.join(src, rel))
            except OSError: continue
            h.update(f'{i}\0{rel}\0{st.st_size}\0{st.st_mtime_ns}\n'.encode())
    return h.hexdigest()


def _stamp(home=None, base=None):
    return Path(base or folder(home)) / '.source.json'


def _config_mark(live):
    """The live config.json's size and modification time ('' without one): the copy takes it again only when this changes."""
    try:
        st = os.stat(Path(live) / 'config.json')
    except OSError:
        return ''
    return f'{st.st_size}:{st.st_mtime_ns}'


def current(live, home=None, base=None):
    """True when the copy exists and was made from this live runtime as it is now."""
    try:
        s = json.loads(_stamp(home, base).read_text())
    except (OSError, ValueError):
        return False
    return ((runtime(home, base) / 'kin.py').exists() and s.get('source') == str(Path(live).absolute())
            and s.get('fingerprint') == fingerprint(live) and s.get('config', '') == _config_mark(live))


def _sync(src, dst, skip):
    """dst becomes src's files (changed ones copied, size+mtime compared), except dst's own `skip` entries; a .py file that src
    no longer has is removed (as a runtime update does), so nothing of an older runtime stays importable."""
    src, dst = Path(src), Path(dst)
    dst.mkdir(parents=True, exist_ok=True)
    have = set(_files(src, skip))
    for rel in sorted(have):
        s, d = src / rel, dst / rel
        d.parent.mkdir(parents=True, exist_ok=True)
        if s.is_symlink():
            target = os.readlink(s)
            if d.is_symlink() and os.readlink(d) == target: continue
            if d.is_symlink() or d.is_file(): d.unlink()
            elif d.is_dir(): shutil.rmtree(d)
            os.symlink(target, d); continue
        ss = s.stat()
        if d.is_file() and not d.is_symlink():
            ds = d.stat()
            if ds.st_size == ss.st_size and ds.st_mtime_ns == ss.st_mtime_ns: continue
        elif d.is_symlink(): d.unlink()
        shutil.copy2(s, d)
    for rel in _files(dst, skip):
        if rel.endswith('.py') and rel not in have and not rel.startswith(os.path.join('models', 'custom') + os.sep):
            (dst / rel).unlink(missing_ok=True)


def prepare_copy(live, *, refresh=True, say=print, home=None, base=None):
    """The simulation's runtime, made from the live one when missing and refreshed when the live one changed (refresh=False:
    an existing copy is used as it is), in `base` (default <home>/sim; copy_folder names the one for another runtime). Its
    config.json starts as the live one's and is its own after that, until the live one's changes (a runtime update, another
    profile): then the copy takes it again. Its log/ is its own. The live runtime is only read."""
    live = Path(live).expanduser().absolute()
    if not (live / 'kin.py').exists():
        raise SystemExit(f'no Roborun runtime at {live} to simulate from: run roborun up once (it installs it), or pass --runtime')
    base = Path(base or folder(home))
    root = runtime(base=base)
    if is_copy(live, home) or same(live, base) or str(os.path.realpath(root)).startswith(os.path.realpath(live) + os.sep):
        raise SystemExit(f'the runtime {live} and the simulation folder {base} overlap: nothing was copied')
    exists = (root / 'kin.py').exists()
    if exists and (not refresh or current(live, base=base)):
        return root
    say(f"{'refreshing' if exists else 'making'} the simulation's own runtime in {base} (a copy of {live}, without "
        'its log/; the simulation never writes into the runtime it copies)')
    for i, (src, dst) in enumerate(_pairs(live, base=base)):
        _sync(src, dst, OWN if i == 0 else ('log',))
    try:
        was = json.loads(_stamp(base=base).read_text()).get('config', '')
    except (OSError, ValueError):
        was = None
    mark = _config_mark(live)
    if mark and (not (root / 'config.json').exists() or was != mark):
        shutil.copy2(live / 'config.json', root / 'config.json')
    _stamp(base=base).write_text(json.dumps(dict(source=str(live), fingerprint=fingerprint(live), config=mark)))
    return root


# ---- what is already running --------------------------------------------------------------------------------------------

def station_status(port, timeout=1.0):
    """A station's /status on this computer's port, {} when something answers that is not a station, None when nothing does."""
    from .robot import _port_taken
    if not _port_taken(port):
        return None
    try:
        with urllib.request.urlopen(f'http://127.0.0.1:{int(port)}/status', timeout=timeout) as r:
            body = json.load(r)
        return body if isinstance(body, dict) else {}
    except Exception:
        return {}


def port_user(port):
    """None when the port is free; else what serves it, in words."""
    st = station_status(port)
    if st is None:
        return None
    if st.get('sim'):
        return 'a simulation\'s station'
    if 'arm_status' in st or 'state' in st:
        return 'a live arms station'
    return 'another program'


def _answers(path):
    try:
        with socket.socket(socket.AF_UNIX) as s:
            s.settimeout(1.0); s.connect(str(path))
        return True
    except OSError:
        return False


def socket_user(path):
    """None when nothing serves the camera socket at path; else what does, in words."""
    if not _answers(path):
        return None
    try:
        from .cameras import broker_status
        st = broker_status(str(path)) or {}
    except Exception:
        st = {}
    return 'a simulation\'s cameras' if st.get('sim') else 'a live camera service'


def live_parts(c=None, backend=True, port=True):
    """What of the live robot runs now, in words ([] when nothing): this install's station, camera service and (backend=True,
    when robot.json does not say simulation) robot backend, and (port=True) a station on the robot's port that is not a
    simulation's, whichever install runs it."""
    from . import robot
    if c is None:
        try: c = json.loads(robot.CONFIG.read_text())
        except (OSError, ValueError): c = {}
    out = []
    for unit, what in (('roborun-station', 'the arms station'), ('roborun-cameras', 'the camera service')):
        if robot._state(unit) in ('active', 'running'):
            out.append(f'{what} ({robot.unit_name(unit)})')
    if backend and not c.get('sim') and robot._state('roborun-robot') in ('active', 'running'):
        out.append(f"this robot's Roborun ({robot.unit_name('roborun-robot')})")
    ours = robot._state('roborun-sim') in ('active', 'running')        # this install's simulation holds the port: not live
    user = None if ours or not port else port_user(robot.STATION_PORT)
    if user and user != 'a simulation\'s station' and not out:
        out.append(f'{user} on 127.0.0.1:{robot.STATION_PORT}')
    return out


# ---- the camera socket ----------------------------------------------------------------------------------------------------

def _owner(path):
    return Path(str(path) + OWNER)


def _made_by_a_simulation(path):
    try:
        row = json.loads(_owner(path).read_text())
        return row.get('by') == 'roborun sim' and row.get('inode') == os.lstat(path).st_ino
    except (OSError, ValueError, AttributeError):
        return False


def claim_socket(path):
    """Make way for the simulation's camera socket at path, or refuse in words. A socket something serves is never touched; a
    left-over one is removed only when a Roborun simulation made it."""
    path = Path(path)
    try:
        st = os.lstat(path)
    except FileNotFoundError:
        path.parent.mkdir(parents=True, exist_ok=True)
        return
    if not stat.S_ISSOCK(st.st_mode):
        raise SystemExit(f'{path} exists and is not a socket: the simulation does not replace it. Next: pass --camera-socket '
                         '<another path>')
    user = socket_user(path)
    if user:
        raise SystemExit(f'the camera socket {path} is already served by {user}: the simulation does not take it over. Next: '
                         'stop that service first (roborun down), or leave out --camera-socket (the simulation then uses its own)')
    if not _made_by_a_simulation(path):
        raise SystemExit(f'{path} is a socket left by another program (no Roborun simulation made it): it is not removed. '
                         'Next: remove it yourself if nothing uses it, or pass --camera-socket <another path>')
    path.unlink()
    _owner(path).unlink(missing_ok=True)


def own_socket(path):
    """Record that this simulation made the socket at path (just bound); returns the function that removes both again, only
    while the socket is still the one it made."""
    path = Path(path)
    ino = os.lstat(path).st_ino
    _owner(path).write_text(json.dumps(dict(by='roborun sim', pid=os.getpid(), inode=ino)))

    def release():
        try:
            if os.lstat(path).st_ino == ino:
                path.unlink()
                _owner(path).unlink(missing_ok=True)
        except OSError:
            pass
    return release
