"""roborun up / down / status: run this robot on this computer with one command.

    roborun login         # once: saves this computer's key
    roborun up            # the robot is named after this computer; a simulation: roborun up --sim cup
    roborun down | status

up = setup (first time) + the robot's backend on 127.0.0.1:8120 (its managed config, loopback only) + the local service:
the door the app.roborun.app page uses on this computer (127.0.0.1:8135) and the history upload. Nothing reaches this robot
from outside this computer. The hosted API is only called from here, for motion jobs and for the history upload.

setup writes the chosen profile's rig to <roborun home>/robot/rig.json (edit it for your own arms and placement, then run
setup again with --rig) and <roborun home>/robot.json, which everything else reads.
"""
from __future__ import annotations

import argparse
import json
import os
import http.client
import shutil
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

from . import net

HOME = Path(os.environ.get('ROBORUN_HOME', Path.home() / '.config' / 'roborun'))
UA = net.user_agent()            # roborun-sdk/<version> (build ...): a named client (the API's edge refuses Python's default)
CONFIG = HOME / 'robot.json'
SERVER = os.environ.get('ROBORUN_SERVER', 'https://api.roborun.app')     # the hosted API: motion jobs and history


def _http(method, url, key=None, body=None):
    """One JSON call to the API (retried when safe: net.request); any failure is a SystemExit in plain words."""
    try:
        status, _, payload = net.request(method, url, key, body=body, timeout=30)
    except net.Offline as e:
        raise SystemExit(net.offline_text(e)) from None
    if status >= 400:
        raise SystemExit(net.problem(status, payload))
    try:
        return json.loads(payload or b'{}')
    except ValueError:
        raise SystemExit('the Roborun API answered with a page that is not its reply (a captive portal or a proxy in the way?). '
                         'Next: check the network this computer is on') from None


def token_state(server, key):
    """(state, words) for this key: ok, revoked, read_only, trouble (the API answered 5xx / 429) or offline."""
    try:
        status, _, payload = net.request('GET', server.rstrip('/') + '/v1/account', key, timeout=15, tries=2)
    except net.Offline as e:
        return 'offline', f'not reachable: {e.reason}; check the internet connection'
    if status < 400:
        return 'ok', 'token accepted'
    code = net.error_of(payload).get('code')
    if status == 403 and code in ('read_only_key', 'scope'):
        return 'read_only', 'read-only key: robots need a robot key (roborun login)'
    if status in (401, 403):
        return 'revoked', 'token not accepted (unknown or revoked): roborun login, or put a new key in the key file'
    if status == 426 or code == 'sdk_upgrade_required':
        return 'upgrade', 'this Roborun version is no longer supported: roborun upgrade'
    return 'trouble', f'the API is having trouble (HTTP {status}); try again shortly'


def check_token(server, key):
    """Sign-in: the API says whether this access token is valid. An unreachable API decides nothing here."""
    state = token_state(server, key)[0]
    return True if state == 'ok' else False if state in ('revoked', 'read_only') else None


def profile_of(robot, rig):
    """manifest.robot-profile.v1: what the server keeps about the arms (slot, gripper, active / parked, base position)."""
    arms = []
    for slot in ('left', 'right'):
        a = (rig.get('arms') or {}).get(slot)
        if not isinstance(a, dict): continue
        T = a.get('T_table_base') or [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]]
        arms.append(dict(slot=slot, gripper=str(a.get('variant') or a.get('model') or 'unknown'),
                         state='parked' if isinstance(a.get('parked'), dict) else 'active',
                         placement=dict(side=slot, base_xy_m=[float(T[0][3]), float(T[1][3])])))
    return dict(schema='manifest.robot-profile.v1', robot=robot, arms=arms)


PROFILES = Path(__file__).parent / 'data' / 'profiles'     # the default profiles, shipped with the SDK


def bundled():
    return {f.stem: f.read_bytes() for f in sorted(PROFILES.glob('*.json'))}


def _installed_rig(root, name):
    """The rig of a profile made on this machine (roborun profiles create: in the install, or in the
    runtime's config folder), byte for byte; None for the pre-made profiles the server serves."""
    if not name or name in bundled():
        return None
    root = Path(root).expanduser()
    for folder in (root / 'profiles' / name, HOME / 'profiles' / name):
        cfg = folder / 'config.json'
        if cfg.exists():
            rel = json.loads(cfg.read_text()).get('rig')
            for base in (root, folder):
                if rel and (base / rel).exists():
                    return (base / rel).read_bytes()
    return None


def profile_bytes(api, name, key=None):
    """A pre-made profile: the bundled copy, unless the server's differs (then the server's, byte for byte: the robot
    refuses motions planned against any other rig file). Works offline with the bundled copy."""
    import hashlib
    local = bundled().get(name)
    try:
        listed = {p['name']: p.get('sha256') for p in _http('GET', api + '/v1/profiles', key)['profiles']}
    except SystemExit:
        listed = None                                       # server unreachable: the bundled copy
    if listed is not None and name not in listed and local is None:
        raise SystemExit(f'no profile {name!r} (server has: {", ".join(sorted(listed))})')
    if local is not None and (listed is None or listed.get(name) in (None, hashlib.sha256(local).hexdigest())):
        return local
    if local is None and listed is None:
        raise SystemExit(f'no bundled profile {name!r} and the server is unreachable')
    try:
        status, _, raw = net.request('GET', api + '/v1/profiles/' + urllib.parse.quote(name), key, timeout=30)
    except net.Offline as e:
        raise SystemExit(net.offline_text(e)) from None
    if status >= 400:
        raise SystemExit(f'profile {name}: ' + net.problem(status, raw))
    json.loads(raw)
    print(f'profile {name}: the server has a newer version, using it')
    return raw


PROFILE_NAMES = 2       # robot.json: written by an SDK where `yam` is one YAM arm (before: `yam` in robot.yaml meant two)
OLD_TWO_YAMS = 'set_up_as_two_yams_before_rename'   # robot.json: this robot ran two YAMs under the old meaning of `yam`


def migrate_profile_names(say=print):
    """Once per robot, after upgrading from an SDK where `profile: yam` meant two YAM arms: robot.yaml's `profile: yam` becomes
    `yam_dual` when this robot ran the two-arm runtime profile (robot.json) or robot.yaml names two arms. Said in one line."""
    from .arms import settings
    c = None
    if CONFIG.exists():
        try:
            c = json.loads(CONFIG.read_text())
        except ValueError:
            return
        if c.get('profile_names') == PROFILE_NAMES:
            return
    try:
        s = settings.load(warn=lambda _: None)
    except SystemExit:
        return
    arms = s.get('arms') if isinstance(s.get('arms'), dict) else {}
    two = (c or {}).get('profile') == 'yam' or (arms.get('left') and arms.get('right'))
    if s.get('profile') == 'yam' and two:
        settings.update_file(dict(profile='yam_dual'))
        say(f"{settings.path()}: profile yam now means one YAM arm, and this robot has two: changed it to yam_dual")
    if c is not None:
        c['profile_names'] = PROFILE_NAMES
        if c.get('profile') == 'yam':                # set up by the old SDK as two YAMs: old `--profile yam` instructions keep meaning two
            c[OLD_TWO_YAMS] = True
        CONFIG.write_text(json.dumps(c, indent=1))


def keep_two_yams(a, say=print):
    """`--profile yam` (or `profile: yam`) on a robot the OLD SDK set up as two YAM arms that still runs them (the runtime's
    `yam`, per robot.json): an old instruction meaning two arms. The robot keeps its two arms, and says how to name them now. A
    robot set up by this SDK follows the new names: `yam` is one arm."""
    if getattr(a, 'profile_id', None) or getattr(a, 'profile', None) != 'yam' or not CONFIG.exists():
        return
    try:
        saved = json.loads(CONFIG.read_text())
    except ValueError:
        return
    if saved.get('profile') == 'yam' and saved.get(OLD_TWO_YAMS):
        say('profile yam is now one YAM arm; your robot ran two, so it keeps two: use yam_dual '
            '(to switch it to one arm on purpose: --profile yam_solo)')
        a.profile = 'yam_dual'


def _rid(a):
    """The runtime's id of the profile asked for: the page's switch gives it as is (--profile-id); a name typed is mapped."""
    from .arm_profiles import runtime_profile
    return getattr(a, 'profile_id', None) or runtime_profile(getattr(a, 'profile', None))


def _premade_lines():
    from .arm_profiles import PREMADE
    return [f'  {name:<12} {label}' for name, label in PREMADE.items()]


def cmd_setup(a) -> int:
    if getattr(a, 'detect', False) or not (a.robot and a.key_file):      # plain `roborun setup`: the guided setup
        from .guided import main
        return main(a)
    from .arm_profiles import user_profile
    if a.profile and not getattr(a, 'profile_id', None):
        a.profile = user_profile(a.profile)
    pid = _rid(a)
    key = Path(a.key_file).expanduser().read_text().strip()
    api = net.check_server(a.server.rstrip('/'), getattr(a, 'insecure', False))
    if check_token(api, key) is False:
        raise SystemExit(f'the access token in {a.key_file} was not accepted: check it and run this again')
    if not a.profile and not a.rig:
        print('Pre-made profiles (pass one with --profile; every catalog arm too: roborun profiles):')
        print('\n'.join(_premade_lines()))
        return 0
    rig_file = HOME / 'robot' / 'rig.json'
    rig_file.parent.mkdir(parents=True, exist_ok=True)
    if a.rig:
        if Path(a.rig).expanduser().resolve() != rig_file.resolve():
            shutil.copyfile(Path(a.rig).expanduser(), rig_file)
    else:
        from .arm_profiles import ensure
        ensure(a.manifest or _manifest_root(), pid)          # an arm of the catalog: its model and profile are written here first
        mine = _installed_rig(a.manifest or _manifest_root(), pid)
        rig_file.write_bytes(mine if mine is not None else profile_bytes(api, pid, key))
    rig = json.loads(rig_file.read_text())
    cfg = dict(profile_names=PROFILE_NAMES, server=api, robot=a.robot, key_file=str(Path(a.key_file).expanduser().resolve()),
               profile=pid or 'custom', rig=str(rig_file), backend=a.backend, data=_data_config(a),
               manifest=str(Path(a.manifest or _manifest_root()).expanduser().resolve()))
    if getattr(a, 'insecure', False):
        cfg['insecure'] = True
    try:                                             # an old two-YAM robot stays one across later setups
        if json.loads(CONFIG.read_text()).get(OLD_TWO_YAMS):
            cfg[OLD_TWO_YAMS] = True
    except (OSError, ValueError):
        pass
    cfg['jobs'] = _write_jobs(cfg)
    HOME.mkdir(parents=True, exist_ok=True)
    CONFIG.write_text(json.dumps(cfg, indent=1)); os.chmod(CONFIG, 0o600)
    arms = ', '.join(f"{x['slot']} {x['gripper']} ({x['state']})" for x in profile_of(a.robot, rig)['arms'])
    print(f"set up {a.robot}: {arms}")
    print(f'rig: {rig_file}   config: {CONFIG}')
    _say_data(cfg['data'])
    if not getattr(a, 'starting', False):
        print('next: roborun up')
    return 0


def _manifest_root():
    """This robot's Roborun install (the backend's code and its managed config.json)."""
    return os.environ.get('MANIFEST_DREAMAPI') or str(Path.home() / 'roborun' / 'robot' / 'runtime')


def _write_jobs(cfg):
    """The backend's API settings (jobs.mode api): where the API is, its key, the API's signing keys it trusts, this robot."""
    keys = _http('GET', cfg['server'] + '/v1/keys', Path(cfg['key_file']).read_text().strip())['keys']
    # the API client (signing, codec): shipped in the runtime bundle (vendor/)
    vendored = Path(cfg.get('manifest') or '') / 'vendor'
    client = vendored
    jobs = dict(mode='api', url=cfg['server'], key_file=cfg['key_file'], trusted_keys=keys, client_src=str(client),
                robot=cfg['robot'], timeout_s=300)
    if cfg.get('insecure'):
        jobs['insecure'] = True
    path = HOME / 'robot' / 'jobs.json'; path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(jobs, indent=1)); os.chmod(path, 0o600)
    return str(path)


KEY_WARN_DAYS = 14


def refresh_keys(c, say=print):
    """Every `roborun up`: the API's current signing keys into jobs.json (a rotated key is trusted from the next start; the
    runtime also fetches them once when a reply names a key it does not have). Offline: the pinned keys stay."""
    import datetime
    path = Path(c.get('jobs') or HOME / 'robot' / 'jobs.json')
    try:
        jobs = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    try:
        status, _, payload = net.request('GET', c['server'].rstrip('/') + '/v1/keys', Path(c['key_file']).read_text().strip(), timeout=15)
        keys = json.loads(payload).get('keys') if status == 200 else None
    except (net.Offline, OSError, ValueError, AttributeError):
        keys = None
    if isinstance(keys, list) and keys and keys != jobs.get('trusted_keys'):
        jobs['trusted_keys'] = keys
        tmp = path.with_name(path.name + '.tmp'); tmp.write_text(json.dumps(jobs, indent=1)); os.chmod(tmp, 0o600); os.replace(tmp, path)
    today = datetime.date.today().isoformat()
    soon = (datetime.date.today() + datetime.timedelta(days=KEY_WARN_DAYS)).isoformat()
    dates = [str(k.get('not_after'))[:10] for k in jobs.get('trusted_keys') or [] if isinstance(k, dict) and k.get('not_after')]
    if dates and max(dates) < today:
        say('warning: every signing key this robot trusts is past its date; tasks will fetch the current ones (check the connection)')
    elif dates and max(dates) <= soon:
        say(f'note: the API signing key this robot trusts ends on {max(dates)}; roborun up fetches its successor once it is published')
    return jobs.get('trusted_keys')


def key_date_notice(c, today=None):
    """roborun status: a line when the newest API signing key this robot trusts (jobs.json) is past or near its not_after."""
    import datetime
    try:
        trusted = json.loads(Path(c.get('jobs') or HOME / 'robot' / 'jobs.json').read_text()).get('trusted_keys') or []
    except (OSError, ValueError, AttributeError):
        return None
    dates = [str(k.get('not_after'))[:10] for k in trusted if isinstance(k, dict) and k.get('not_after')]
    if not dates:
        return None
    today = today or datetime.date.today()
    if max(dates) < today.isoformat():
        return f'past its date ({max(dates)}): run roborun up to fetch the current keys'
    if max(dates) <= (today + datetime.timedelta(days=KEY_WARN_DAYS)).isoformat():
        return f'ends {max(dates)}: roborun up fetches its successor once it is published'
    return None


def dataset_root(robot):
    """This robot's LeRobot v2.1 dataset (one episode per finished task or move): ~/roborun/datasets/<robot>/, beside the
    recordings, so `roborun uninstall` keeps it unless asked to purge (never inside ~/.roborun, which uninstall removes)."""
    return Path.home() / 'roborun' / 'datasets' / str(robot)


def _data_sources(runtime, data_root=None, video_motion=None):
    """What the history upload sends: the backend's task records and its clip / recording folders (the runtime's log/)."""
    root = Path(data_root or Path(runtime) / 'log').expanduser().absolute()
    vm = Path(video_motion).expanduser().absolute() if video_motion else root / 'video-motion'
    # logs/: the station's hourly telemetry (joints, jaws, speeds, temperatures, events such as Home), task or not
    return {'video-motion': str(vm), 'tasks': str(root / 'tasks'), 'logs': str(root / 'telemetry')}


def _data_config(a):
    """The history upload: on unless --no-data-sync (remembered)."""
    runtime = a.manifest or _manifest_root()
    off = bool(getattr(a, 'no_data_sync', False))
    checkout = (Path(runtime).expanduser() / 'kin.py').exists() and not _ours(runtime)     # never uploaded unless asked
    return dict(sources=_data_sources(runtime, a.data_root, a.video_motion), custom=bool(a.data_root or a.video_motion),
                state=str(HOME / 'robot' / 'sync-state.json'), enabled=not off and not checkout, opt_out=off)


def _say_data(d):
    if d.get('enabled'):
        print("history: this robot's task records and recordings are uploaded to your Roborun account; "
              "turn off with: roborun up --no-data-sync")
    else:
        print('history: not uploaded (--no-data-sync); turn on again with: roborun up --data-sync')


def _recording_tools(a, c, persist=True):
    """`roborun up`, before the robot starts: the recording tools (the dataset extra: pyarrow + PyAV, about 290 MB) are
    installed once. Off with --no-dataset or the remembered --no-data-sync (both remembered); never while a task runs. A failed
    install is said once and remembered (roborun_sdk.dataset.unavailable() reports it on every recording); up carries on."""
    r = c.setdefault('recording', {})
    before = dict(r)
    if getattr(a, 'no_dataset', False): r['dataset'] = False
    elif getattr(a, 'dataset', False): r['dataset'] = True
    if persist and r != before and CONFIG.exists():
        CONFIG.write_text(json.dumps(c, indent=1))
    if r.get('dataset') is False or (c.get('data') or {}).get('opt_out'):
        return 'off'
    from . import dataset
    if dataset.unavailable() is None:
        return None
    if _busy(c.get('backend') or 'http://127.0.0.1:8120'):
        print('a task is running on this robot: recording setup skipped until the next roborun up', flush=True)
        return 'busy'
    return dataset.setup_tools(log=lambda line: print(line, flush=True))


def face_blur(c=None):
    """The privacy.face_blur setting (robot.yaml): True unless the user set it to false."""
    from .arms.settings import load
    hardware = (c or {}).get('hardware') or {}
    value = (hardware.get('privacy') or load().get('privacy') or {}).get('face_blur', True)
    if not isinstance(value, bool):
        raise SystemExit(f'privacy face_blur must be true or false, not {value!r}')
    return value


def _syncer(c, background=False):
    from roborun_sdk.datasync import DataSync
    backend = c.get('backend') or 'http://127.0.0.1:8120'
    d = c.get('data') or {}
    sources = dict(d.get('sources') or {})
    if not d.get('custom'):                                   # the robot's LeRobot dataset goes up too (blurred, like everything)
        sources.setdefault('datasets', str(dataset_root(c['robot'])))
    return DataSync(c['server'], Path(c['key_file']).read_text().strip(), c['robot'], sources, d.get('state'),
                    log=lambda m: print(m, flush=True), busy=(lambda: _busy(backend)) if background else None, face_blur=face_blur(c))


def cmd_sync(a) -> int:
    if not CONFIG.exists():
        raise SystemExit(f'{CONFIG} is missing: run roborun up first')
    s = _syncer(json.loads(CONFIG.read_text()))
    print(json.dumps(dict(event='data_synced', **s.once())), flush=True)
    return 0


def cmd_local(a) -> int:
    """The local service: the page's door on this computer, and the history upload."""
    import signal
    from roborun_sdk.local_proxy import LocalProxy, PORT
    if not CONFIG.exists():
        raise SystemExit(f'{CONFIG} is missing: run roborun up first')
    c = json.loads(CONFIG.read_text())
    backend = urllib.parse.urlsplit(c.get('backend') or 'http://127.0.0.1:8120')
    if backend.scheme != 'http' or backend.hostname not in ('127.0.0.1', 'localhost'):
        raise SystemExit('the robot backend must be on this computer (http://127.0.0.1:<port>)')
    port = int(c.get('live_port', PORT) or 0)
    if port:
        LocalProxy(backend.hostname, backend.port or 80, port, c.get('ui_origins') or (), c['robot'],
                   Path(c['key_file']).read_text().strip()).start()
    if (c.get('data') or {}).get('enabled'):
        try: os.nice(10)                                    # the history upload never competes with the robot
        except OSError: pass
        _syncer(c, background=True).start()
    print(json.dumps(dict(event='local_service', robot=c['robot'], page_port=port or None)), flush=True)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    while True:
        signal.pause()


def camera_socket():
    """Where the robot's camera broker listens: $MANIFEST_CAMERA_SOCKET, else a short per-user path (Unix sockets allow
    ~108 characters, so not under a deep home folder)."""
    import os
    if os.environ.get('MANIFEST_CAMERA_SOCKET'):
        return os.environ['MANIFEST_CAMERA_SOCKET']
    base = os.environ.get('XDG_RUNTIME_DIR') or f'/tmp/manifest-{os.getuid()}'
    os.makedirs(os.path.join(base, 'manifest'), mode=0o700, exist_ok=True)
    return os.path.join(base, 'manifest', 'camera.sock')


UNITS = ('roborun-local', 'roborun-robot', 'roborun-sim', 'roborun-station', 'roborun-cameras')
LABEL = {'roborun-local': 'page and history', 'roborun-robot': 'robot', 'roborun-sim': 'simulation', 'roborun-station': 'arms', 'roborun-cameras': 'cameras'}
BODY_PROFILE = {'yam': 'yam', 'yam_metal': 'yam_metal', 'makerarm': 'makerarm'}


def install_profiles(root):
    """{id: (label, how to build its config)} of the arm profiles this Roborun install has: the saved configs
    (profiles/<id>/config.json) and the arm-pair records (workspaces/<id>/profile.json: a saved config + its own rig)."""
    root = Path(root); out = {}
    for f in sorted((root / 'profiles').glob('*/config.json')):
        out[f.parent.name] = dict(label=f.parent.name, config=f, record=None)
    for f in sorted((root / 'workspaces').glob('*/profile.json')):
        try: rec = json.loads(f.read_text())
        except ValueError: continue
        if rec.get('kind') == 'arm_pair' and rec.get('id') and (root / 'profiles' / str(rec.get('body')) / 'config.json').exists():
            out[rec['id']] = dict(label=rec.get('label') or rec['id'], config=root / 'profiles' / rec['body'] / 'config.json', record=rec)
    return out


def current_profile(root):
    """The profile the install's managed config is: a record's id, else the saved profile whose rig it uses."""
    c = json.loads((Path(root) / 'config.json').read_text())
    if c.get('profile_id'):
        return c['profile_id']
    for f in sorted((Path(root) / 'profiles').glob('*/config.json')):
        try:
            if json.loads(f.read_text()).get('rig') == c.get('rig'): return f.parent.name
        except ValueError:
            pass
    return BODY_PROFILE.get(c.get('body') or 'yam')


def profile_config_matches(root, pid):
    """Whether the install's managed config runs profile `pid` with that profile's own rig (a config can keep a profile's
    name while pointing at another profile's rig; `roborun up --profile` then writes it again)."""
    try:
        row = install_profiles(root).get(pid)
        if row is None:
            return True                                  # not an install profile: set_profile decides
        want = row['record']['rig'] if row['record'] is not None else json.loads(row['config'].read_text()).get('rig')
        return json.loads((Path(root) / 'config.json').read_text()).get('rig') == want
    except (OSError, ValueError, KeyError, TypeError):
        return True


def set_profile(root, pid):
    """Write this install's managed config for profile `pid` (what the backend's own profile switch writes); the previous
    config is kept in the runtime's config-backups folder. The backend must be stopped or restarted after."""
    import time as _t
    from .arm_profiles import ensure
    ensure(root, pid)
    root = Path(root); profiles = install_profiles(root)
    mine = HOME / 'profiles' / pid
    if pid not in profiles and (mine / 'config.json').exists():
        # a profile made with `roborun profiles create` on a machine without the install: copied in now
        shutil.copytree(mine, root / 'profiles' / pid, dirs_exist_ok=True)
        rig = json.loads((mine / 'rig.json').read_text()) if (mine / 'rig.json').exists() else {}
        names = {Path(a['model']).parent.name for a in (rig.get('arms') or {}).values() if isinstance(a, dict) and str(a.get('model', '')).startswith('custom/')} | {pid}
        for name in names:                              # the profile's own model folder, or the catalog models it uses
            if (HOME / 'models' / 'custom' / name).exists():
                shutil.copytree(HOME / 'models' / 'custom' / name, root / 'models' / 'custom' / name, dirs_exist_ok=True)
        profiles = install_profiles(root)
    if pid not in profiles:
        raise SystemExit(f"no profile {pid!r} in {root} (available: {', '.join(sorted(profiles))})")
    row = profiles[pid]
    cfg = json.loads(row['config'].read_text())
    if row['record'] is not None:
        cfg = dict(cfg, profile_id=pid, rig=row['record']['rig'])
    else:
        cfg.pop('profile_id', None)
    managed = root / 'config.json'
    backups = HOME / 'robot' / 'config-backups'; backups.mkdir(parents=True, exist_ok=True)
    (backups / f"{_t.strftime('%Y%m%d-%H%M%S')}-{current_profile(root)}.json").write_bytes(managed.read_bytes())
    tmp = managed.with_name('.config.json.tmp'); tmp.write_text(json.dumps(cfg, indent=2)); tmp.replace(managed)


# profiles whose arms each have their own native controller on a real robot (YAM + MakerMods Metal: the Metal's owner on :8093).
# The simulation drives every arm through its one station on :8091, so a simulated robot runs these on the station config.
NATIVE_BODIES = {'yam_metal'}


def sim_config(root, on):
    """The install's managed config for simulating (on) or for the real arms (off). On: a profile with native arm controllers
    runs on the station config (profiles/yam/config.json) with its own rig, marked sim_of; the profile keeps its name (the rig
    still matches it). Off: a config written for the simulation is written again from the profile. True when it changed."""
    root = Path(root); managed = root / 'config.json'
    c = json.loads(managed.read_text())
    if on and c.get('body') in NATIVE_BODIES:
        base = root / 'profiles' / 'yam' / 'config.json'
        cfg = json.loads(base.read_text()) if base.exists() else dict(body='yam', station='http://127.0.0.1:8091')
        cfg.update(rig=c['rig'], sim_of=c['body'])
        cfg.pop('profile_id', None)
        if c.get('profile_id'): cfg['profile_id'] = c['profile_id']
        tmp = managed.with_name('.config.json.tmp'); tmp.write_text(json.dumps(cfg, indent=2)); tmp.replace(managed)
        return True
    if not on and c.get('sim_of'):
        set_profile(root, current_profile(root))
        return True
    return False


def _busy(backend):
    try:
        with urllib.request.urlopen(backend + '/v1/tasks?limit=3', timeout=3) as r:
            d = json.load(r)
        rows = d.get('tasks') or d.get('data') or (d if isinstance(d, list) else [])
        return any(isinstance(t, dict) and t.get('status') == 'running' for t in rows)
    except Exception:
        return False


SCENES = ('cup', 'pickup-bench', 'mixed-objects', 'cluttered-bench', 'low-light')    # the arm scenes (`--sim`); a humanoid's: room, empty


# runtime profiles `roborun profiles` does not list: older profile names, the MakerArm pair's alternate model,
# and the runtime's own ids of the names it does list
UNLISTED = {'yam_single', 'makerarm', 'makerarm_urdf'}


def cmd_profiles(a) -> int:
    from .arm_profiles import PREMADE, models, parse, shown_profile
    c = json.loads(CONFIG.read_text()) if CONFIG.exists() else {}
    root = Path(a.manifest or c.get('manifest') or _manifest_root())
    cur = shown_profile(current_profile(root)) if (root / 'config.json').exists() and not c.get('body') else None
    rows = dict(PREMADE)
    found = [pid for pid in install_profiles(root)] + [f.parent.name for f in (HOME / 'profiles').glob('*/config.json')]
    for pid in found:
        name = shown_profile(pid)
        if pid not in UNLISTED and name not in rows and parse(pid) is None:      # a catalog pair is listed by the line below
            rows[name] = 'made here'
    if cur and cur not in rows:
        rows[cur] = 'running'
    print(f'arm profiles (simulate one: --sim {", ".join(SCENES)}):')
    for name, label in rows.items():
        print(f"{'*' if name == cur else ' '} {name:<12} {label}")
    print('catalog arms (any of these: <arm> for one, <arm>_dual for two, <left>+<right> for two different ones): ' + ', '.join(models()))
    from .body import profiles as body_profiles
    running = (c.get('body') or {}).get('profile')
    print('humanoids (simulate one: --sim room or empty):')
    for row in body_profiles.rows():
        print(f"{'*' if row['id'] == running else ' '} {row['id']:<12} {row['label']} (humanoid, {row['base']} base; --sim room, or --body-agent)")
    print('switch: roborun up --profile <name>    your own arm: roborun profiles create --urdf ...')
    return 0


def _systemd():
    """systemd --user is usable here (Linux desktops/servers); otherwise plain background processes."""
    import subprocess
    if not shutil.which('systemd-run') or not shutil.which('systemctl'):
        return False
    return subprocess.run(['systemctl', '--user', 'is-system-running'], capture_output=True, text=True).stdout.strip() not in ('', 'offline')


def _pidfile(name):
    d = HOME / 'run'; d.mkdir(parents=True, exist_ok=True)
    return d / f'{name}.pid'


def _alive(pid):
    try:
        os.kill(pid, 0); return True
    except (OSError, ValueError):
        return False


STOP_WAIT_S = {'roborun-station': 60}                    # it may be parking the arms before it exits


def _stop(name):
    import signal, subprocess, time
    if _systemd():
        subprocess.run(['systemctl', '--user', 'stop', name], capture_output=True)
        subprocess.run(['systemctl', '--user', 'reset-failed', name], capture_output=True)
    pf = _pidfile(name)
    try:
        pid = int(pf.read_text())
    except (OSError, ValueError):
        return
    if _alive(pid):
        os.killpg(os.getpgid(pid), signal.SIGTERM)
        for _ in range(STOP_WAIT_S.get(name, 5) * 10):
            if not _alive(pid): break
            time.sleep(0.1)
        if _alive(pid): os.killpg(os.getpgid(pid), signal.SIGKILL)
    pf.unlink(missing_ok=True)


def _unit(name, argv, env):
    """Start a long-running part: a systemd --user unit where there is one (restarts on failure), else a background
    process in its own session with its log in <home>/logs/<name>.log."""
    import subprocess
    _stop(name)
    if _systemd():
        out = subprocess.run(['systemd-run', '--user', f'--unit={name}', '-p', 'Restart=on-failure', '-p', 'RestartSec=3',
                              *[f'--setenv={k}={v}' for k, v in env.items()], *argv], capture_output=True, text=True)
        if out.returncode:
            raise SystemExit(f'could not start {name}: {out.stderr.strip()[:300]}')
        return
    logs = HOME / 'logs'; logs.mkdir(parents=True, exist_ok=True)
    log = open(logs / f'{name}.log', 'ab')
    proc = subprocess.Popen(argv, env=dict(os.environ, **env), stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            start_new_session=True)
    _pidfile(name).write_text(str(proc.pid))


def _state(name):
    import subprocess
    if _systemd():
        st = subprocess.run(['systemctl', '--user', 'is-active', name], capture_output=True, text=True).stdout.strip()
        if st == 'active': return st
    try:
        return 'running' if _alive(int(_pidfile(name).read_text())) else 'stopped'
    except (OSError, ValueError):
        return 'stopped' if not _systemd() else 'inactive'


def _backend_up(url, wait_s=180):
    import time
    for _ in range(int(wait_s)):
        try:
            with urllib.request.urlopen(url + '/v1/settings', timeout=2): return True
        except Exception:
            time.sleep(1)
    return False


def _sdk_path():
    """PYTHONPATH for the units: the SDK's own folder plus the caller's (a source checkout of the station package)."""
    parts = [str(Path(__file__).resolve().parents[1])] + [p for p in os.environ.get('PYTHONPATH', '').split(os.pathsep) if p]
    return os.pathsep.join(dict.fromkeys(parts))


def install_root():
    """Where install.sh put Roborun (~/.roborun): the folder holding this Python's environment, else ~/.roborun."""
    here = Path(sys.prefix).parent
    return here if (here / 'tools').is_dir() or here.name == '.roborun' else Path.home() / '.roborun'


GL_MISSING = ('the simulation needs an OpenGL library to render its cameras, and this computer has none. Install one (this needs '
              'an administrator), then run roborun up again:\n'
              '  Ubuntu/Debian:  sudo apt install libosmesa6        (or libegl1 on a computer with a GPU)\n'
              '  Fedora:         sudo dnf install mesa-libOSMesa\n'
              '  Arch:           sudo pacman -S mesa')


def gl_env():
    """How MuJoCo renders the simulated cameras on this machine: the user's MUJOCO_GL if set; EGL with a GPU render node;
    OSMesa (CPU) otherwise. Neither library installed: say what to install instead of crashing in a loop. The renderer's shader
    cache goes inside Roborun's folder, so `roborun uninstall` leaves nothing in ~/.cache."""
    out = _gl_env()
    if not os.environ.get('MESA_SHADER_CACHE_DIR'):
        out['MESA_SHADER_CACHE_DIR'] = str(install_root() / 'cache' / 'mesa')
    return out


def _gl_env():
    import ctypes.util, glob
    if os.environ.get('MUJOCO_GL'):
        out = dict(MUJOCO_GL=os.environ['MUJOCO_GL'])
        if os.environ.get('PYOPENGL_PLATFORM'): out['PYOPENGL_PLATFORM'] = os.environ['PYOPENGL_PLATFORM']
        return out
    if glob.glob('/dev/dri/renderD*') and ctypes.util.find_library('EGL'):
        return dict(MUJOCO_GL='egl')
    if ctypes.util.find_library('OSMesa'):
        return dict(MUJOCO_GL='osmesa', PYOPENGL_PLATFORM='osmesa')
    if sys.platform == 'darwin':
        return dict(MUJOCO_GL='glfw')
    raise SystemExit(GL_MISSING)


def installed_runtime(root):
    try:
        return json.loads((Path(root).expanduser() / 'RUNTIME.json').read_text()).get('version')
    except (OSError, ValueError):
        return None


INSTALL_DIRS = {'robot', 'depth', 'harness', 'supervisor'}     # what a runtime update replaces
KEPT_DIRS = {'datasets'}            # the robot's LeRobot episodes, written beside it: part of the install, never replaced or removed


def _ours(root):
    """True only for a runtime `roborun up` installed: its RUNTIME.json, and an install folder holding nothing else."""
    root = Path(root).expanduser()
    if not (root / 'RUNTIME.json').is_file() or root.parent.name != 'robot' or root.name != 'runtime':
        return False
    top = root.parents[1]
    return all(p.name in INSTALL_DIRS | KEPT_DIRS or p.name.startswith('.') for p in top.iterdir())


def ensure_runtime(root, server, key, update=False):
    """The Roborun robot runtime at `root`: downloaded from the server (with this robot's token), checked against its
    SHA-256, unpacked, and its Python packages installed into this environment. `update`: replace an installed runtime
    when the server publishes a newer one (its logs, recordings and this robot's config.json are kept)."""
    import hashlib, subprocess, tarfile, tempfile
    root = Path(root).expanduser()
    have = installed_runtime(root) if (root / 'kin.py').exists() else None
    if (root / 'kin.py').exists() and not update:
        return False
    if (root / 'kin.py').exists() and not _ours(root):
        return False                                # a source checkout or someone else's folder: never written to
    try:
        meta = _http('GET', server.rstrip('/') + '/v1/runtime', key)
    except SystemExit as exc:
        if have or (root / 'kin.py').exists():
            print(f'runtime update check skipped ({exc}); keeping {have or "the installed runtime"}', flush=True)
            return False
        raise SystemExit(f'could not get the Roborun runtime: {exc}') from None
    if (root / 'kin.py').exists() and meta['version'] == have:
        return False
    need = meta.get('min_sdk_build')
    if need and (net.build() or '') and net.build() < str(need):
        text = (f"the Roborun runtime {meta['version']} needs a newer Roborun (build {need} or later; this is {net.build()}). "
                'Next: curl -fsSL https://roborun.app/install.sh | sh')
        if (root / 'kin.py').exists():
            print(f'{text}; keeping {have or "the installed runtime"}', flush=True)
            return False
        raise SystemExit(text)
    kept = (root / 'config.json').read_text() if (root / 'config.json').exists() else None
    verb = f'updating the Roborun runtime {have} ->' if have else 'installing the Roborun runtime'
    print(f"{verb} {meta['version']} ({meta['bytes'] / 1e6:.0f} MB) into {root}", flush=True)
    with tempfile.TemporaryDirectory() as tmp:
        bundle = Path(tmp) / meta['file']
        download(server.rstrip('/') + '/v1/runtime/' + meta['file'], key, bundle, meta.get('bytes'))
        if hashlib.sha256(bundle.read_bytes()).hexdigest() != meta['sha256']:
            raise SystemExit('the downloaded runtime does not match its checksum. Next: run roborun up again')
        with tarfile.open(bundle) as tar:
            names = tar.getnames()
            if any(n.startswith(('/', '..')) or '/../' in n or not n.startswith('manifest/') for n in names):
                raise SystemExit('the runtime bundle has unexpected paths; not installed')
            tar.extractall(tmp, filter='data') if hasattr(tarfile, 'data_filter') else tar.extractall(tmp)
        top = root.parents[1]                       # the bundle is the whole install: <top>/robot/runtime + its helpers
        if top.exists() and any(p.name not in INSTALL_DIRS | KEPT_DIRS and not p.name.startswith('.') for p in top.iterdir()):
            raise SystemExit(f'{top} holds other files: the runtime installs only into an empty folder (pass --runtime '
                             '<empty folder>/robot/runtime)')
        top.mkdir(parents=True, exist_ok=True)
        if root.exists():                           # an update replaces the code: nothing of the previous bundle stays importable
            fresh = Path(tmp) / 'manifest' / root.relative_to(top)
            for old in root.rglob('*.py'):
                rel = old.relative_to(root)
                if rel.parts[0] != 'log' and rel.parts[:2] != ('models', 'custom') and not (fresh / rel).exists():
                    old.unlink()
            for folder in INSTALL_DIRS - {'robot'}:
                shutil.rmtree(top / folder, ignore_errors=True)
        shutil.copytree(Path(tmp) / 'manifest', top, dirs_exist_ok=True)
        if not (root / 'kin.py').exists():
            raise SystemExit(f'the runtime bundle did not unpack to {root}')
        if kept is not None:                        # this robot's managed config wins over the bundle's defaults
            try:
                merged = dict(json.loads((root / 'config.json').read_text()), **json.loads(kept))
                (root / 'config.json').write_text(json.dumps(merged, indent=2))
            except ValueError:
                (root / 'config.json').write_text(kept)
    print('installing its Python packages (requirements.txt) ...', flush=True)
    # pip's download cache lives inside Roborun's own folder, so `roborun uninstall` (or removing ~/.roborun) leaves nothing behind
    out = subprocess.run([sys.executable, '-m', 'pip', 'install', '-q', '-r', str(root / 'requirements.txt')], capture_output=True, text=True,
                         env=dict(os.environ, PIP_CACHE_DIR=str(install_root() / 'cache' / 'pip')))
    if out.returncode:
        raise SystemExit('could not install the runtime\'s packages:\n' + out.stderr[-1500:])
    return True


DOWNLOAD_TRIES = 4


def download(url, key, target, size=None, tries=DOWNLOAD_TRIES, sleep=None):
    """A file from the API into `target`, resumed with a Range request after a dropped connection (up to `tries` attempts,
    with backoff). Any failure is a SystemExit that says it was the download, not the robot."""
    import time
    sleep = sleep or time.sleep
    target = Path(target)
    target.write_bytes(b'')
    why = 'network error'
    for attempt in range(tries):
        have = target.stat().st_size
        if size and have >= int(size):
            return target
        headers = dict(net.client_headers(), Authorization='Bearer ' + key)
        if have:
            headers['Range'] = f'bytes={have}-'
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers=headers), timeout=60) as r:
                net.note_release(r.headers)
                mode = 'ab' if have and r.status == 206 else 'wb'
                with open(target, mode) as f:
                    shutil.copyfileobj(r, f)
            if not size or target.stat().st_size >= int(size):
                return target
            why = 'connection dropped'
        except urllib.error.HTTPError as e:
            if e.code not in net.RETRY_STATUS:
                raise SystemExit('could not download the Roborun runtime: ' + net.problem(e.code, e.read())) from None
            why = f'HTTP {e.code}'
        except (urllib.error.URLError, OSError, http.client.HTTPException) as e:
            why = net.reason(e)
        if attempt + 1 < tries:
            sleep(net.backoff(attempt))
    raise SystemExit(f'could not download the Roborun runtime ({why}). Next: check this computer\'s internet connection, '
                     'then run roborun up again (the download resumes)')


def _port_taken(port):
    import socket
    with socket.socket() as s:
        s.settimeout(.5)
        return s.connect_ex(('127.0.0.1', int(port))) == 0


def _check_ports(c, hardware):
    """Every port this start needs is free, or already ours. Otherwise nothing starts, and the port is named."""
    from .local_proxy import PORT
    wanted = [('roborun-robot', urllib.parse.urlsplit(c.get('backend') or 'http://127.0.0.1:8120').port or 8120, 'the local robot API')]
    if int(c.get('live_port', PORT) or 0):
        wanted.append(('roborun-local', int(c.get('live_port', PORT)), 'the connection to the app.roborun.app page'))
    if c.get('sim'):
        wanted.append(('roborun-sim', 8091, 'the simulated arms'))
    elif hardware or c.get('hardware'):
        wanted.append(('roborun-station', 8091, 'the arms'))
    for unit, port, what in wanted:
        if _state(unit) not in ('active', 'running') and _port_taken(port):
            raise SystemExit(f'port {port} on this computer is in use by another program, and Roborun needs it for {what}. '
                             f'Next: stop that program (see it with: lsof -i :{port}), then run roborun up again')


def resolve_profile(a):
    """`roborun up`'s first steps: the profile asked for (flag, robot.yaml, the page's switch), after the one-time robot.yaml
    migration and the first-run setup. a.profile is then a name people type; _rid(a) is the runtime's id (None: keep the
    profile that runs)."""
    from .arm_profiles import shown_profile, user_profile
    from .arms.settings import first_run
    if getattr(a, 'profile_id', None):              # the page's switch: the runtime's id, exactly
        a.profile = shown_profile(a.profile_id)
    elif getattr(a, 'profile', None):
        a.profile = user_profile(a.profile)
    migrate_profile_names()                          # robot.yaml from before `yam` meant one arm: once, said
    first_run(a)                                     # a bare `roborun up` works: robot.yaml, then the guided setup when no arms are named
    keep_two_yams(a)                                 # an old `--profile yam` never takes an arm away from a two-YAM robot


def cmd_up(a) -> int:
    import shlex
    if sys.platform == 'win32':
        raise SystemExit('roborun runs on Linux: on Windows, install WSL2 with Ubuntu and run it there (wsl --install -d Ubuntu)')
    from .arm_profiles import shown_profile
    if getattr(a, 'sim', None):
        gl_env()                                     # a missing OpenGL library says so first, before any setup is written
    resolve_profile(a)
    if a.key_file and not Path(a.key_file).expanduser().is_file():
        raise SystemExit(f'no access token at {a.key_file}: save the token you were given in that file (one line)')
    from .body import up as body_up
    saved = json.loads(CONFIG.read_text()) if CONFIG.exists() else {}
    if body_up.wanted(a, saved):                     # a humanoid profile: its own local robot API, no arm runtime
        _recording_tools(a, saved)
        return body_up.up(a)
    carry = {}
    if body_up.leave(saved):                         # from a humanoid profile to arms: the arm setup runs if it never has
        carry = {k: saved[k] for k in ('sim', 'body_agents') if k in saved}
        CONFIG.write_text(json.dumps(saved, indent=1))
        if not saved.get('rig'):
            a.robot, a.server, a.key_file = a.robot or saved.get('robot'), a.server or saved.get('server'), a.key_file or saved.get('key_file')
    if not CONFIG.exists():
        a.server = a.server or SERVER
    if a.server:
        net.check_server(a.server, getattr(a, 'insecure', False))
    if not CONFIG.exists() or (a.robot and a.server):
        if a.server and a.key_file:              # first run on a clean machine: the runtime comes from the server
            ensure_runtime(a.manifest or _manifest_root(), a.server, Path(a.key_file).expanduser().read_text().strip())
        if a.server and a.robot and a.key_file and not (a.profile or a.rig):
            from .arm_profiles import DEFAULT
            a.profile = DEFAULT                        # nothing names the arms: one YAM arm
        if not (a.server and a.robot and a.key_file):
            raise SystemExit('no key: run roborun login (Roborun is invite-only: if you don\'t have a key, email support@roborun.dev)')
        a.starting = True
        if cmd_setup(a): return 1
    c = json.loads(CONFIG.read_text())
    if carry and any(c.get(k) != v for k, v in carry.items()):
        c.update(carry); CONFIG.write_text(json.dumps(c, indent=1))
    if a.key_file and str(Path(a.key_file).expanduser().resolve()) != str(Path(c.get('key_file') or '').expanduser().resolve()):
        c['key_file'] = str(Path(a.key_file).expanduser().resolve())     # a key given now replaces the saved one
        CONFIG.write_text(json.dumps(c, indent=1)); print(f"key: {c['key_file']}", flush=True)
    from .arms.settings import settle_token
    settle_token(c['key_file'], c['server'])                           # a revoked token stops here, before anything starts
    refresh_keys(c)                                                    # the API's current signing keys, every start
    busy = _busy(c.get('backend') or 'http://127.0.0.1:8120')          # 3 s; a stopped backend is not busy
    if busy:
        print('a task is running on this robot: runtime update check skipped', flush=True)
    ensure_runtime(c['manifest'], c['server'], Path(c['key_file']).read_text().strip(), update=not busy)
    if getattr(a, 'sim', None):
        c['sim'] = a.sim; CONFIG.write_text(json.dumps(c, indent=1))
    if getattr(a, 'no_sim', False) and c.get('sim'):
        c.pop('sim'); CONFIG.write_text(json.dumps(c, indent=1)); _stop('roborun-sim')
    from .arms.hardware import config_from_args
    from .arms.settings import _profile_for, one_arm_on_the_left
    from .arms import parse_spec
    if getattr(a, 'sim', None) and (a.left or a.right) and not a.profile:
        # simulate exactly the arms named, any mix: roborun up --sim cup --left yam --right maker_arm
        a.profile = _profile_for(one_arm_on_the_left({s: dict(arm=parse_spec(getattr(a, s))[0]) for s in ('left', 'right') if getattr(a, s)}))
    hardware = None if getattr(a, 'sim', None) else config_from_args(a, c.get('hardware'))
    if hardware and (a.left or a.right) and not a.profile:
        a.profile = _profile_for(hardware)                               # the arms plugged in decide the profile, not a preset
    if busy and hardware and hardware != c.get('hardware'):
        raise SystemExit('a task is running on this robot: stop it before changing its arms or settings')
    if not getattr(a, 'sim', None) and (hardware != c.get('hardware') or (hardware and c.get('sim'))):
        c['hardware'] = hardware; c.pop('sim', None); CONFIG.write_text(json.dumps(c, indent=1)); _stop('roborun-sim')
    _check_ports(c, hardware)
    d = c.setdefault('data', {})
    before = json.dumps(d, sort_keys=True)
    for k in ('telemetry', 'telemetry_db'): d.pop(k, None)
    ours = _ours(c['manifest'])                      # a source checkout's log/ may hold other data: left exactly as configured
    if ours and not d.get('custom'): d['sources'] = _data_sources(c['manifest'])  # the runtime's own log/
    if getattr(a, 'no_data_sync', False): d.update(enabled=False, opt_out=True)
    elif getattr(a, 'data_sync', False): d.update(enabled=True, opt_out=False)
    elif ours and not d.get('opt_out'): d['enabled'] = True
    d.setdefault('state', str(HOME / 'robot' / 'sync-state.json'))
    if json.dumps(d, sort_keys=True) != before:
        CONFIG.write_text(json.dumps(c, indent=1)); _say_data(d)
    _recording_tools(a, c)                           # the dataset extra, once, before the robot starts
    root = Path(c['manifest'])
    if not (root / 'config.json').exists():
        raise SystemExit(f'no Roborun install at {root} (its config.json); pass --runtime on the first run')
    rid = _rid(a)
    if rid and (rid != current_profile(root) or not profile_config_matches(root, rid)):
        if _busy(c['backend']):
            raise SystemExit('a task is running on this robot: stop it before changing the profile')
        _stop('roborun-robot')
        set_profile(root, rid)
        c['profile'] = rid; CONFIG.write_text(json.dumps(c, indent=1))
        print(f'profile: {shown_profile(rid)}')
    if c.get('profile') != current_profile(root):
        c['profile'] = current_profile(root)                             # robot.json names the profile that runs, every start
        CONFIG.write_text(json.dumps(c, indent=1))
    sim_config(root, bool(c.get('sim')))             # before the backend starts: it reads its config then
    port = urllib.parse.urlsplit(c['backend']).port or 8120
    camera = camera_socket()
    if c.get('sim'):
        # the MuJoCo simulation of this profile: its station on :8091 and its cameras, before the backend looks for them
        _unit('roborun-sim', [sys.executable, '-m', 'roborun_sdk.cli', 'sim', '--scene', c['sim'], '--runtime', str(root),
                               '--camera-socket', camera], dict(PYTHONPATH=_sdk_path(), **gl_env(),
                                                                PYTHONUNBUFFERED='1', ROBORUN_HOME=str(HOME)))
        print(f"simulation: {shown_profile(current_profile(root))}, scene {c['sim']}")
        import time as _t
        for _ in range(60):
            try:
                with urllib.request.urlopen('http://127.0.0.1:8091/status', timeout=1): break
            except Exception:
                _t.sleep(1)
    elif c.get('hardware'):
        hw_file = HOME / 'robot' / 'hardware.json'
        hw_file.parent.mkdir(parents=True, exist_ok=True)
        from .arms.connect import _failure_path
        from .arms.hardware import loaded_rig
        hw_file.write_text(json.dumps(dict(c['hardware'], **loaded_rig(root)), indent=1))
        run = [sys.executable, '-m', 'roborun_sdk.cli']
        hw_env = dict(PYTHONPATH=_sdk_path(), PYTHONUNBUFFERED='1', ROBORUN_HOME=str(HOME))
        if _state('roborun-station') in ('active', 'running'):          # replaced below: it lets go of the arms first
            from .arms.hardware import let_go_of_running_station
            let_go_of_running_station()
        if c['hardware'].get('cameras'):
            _unit('roborun-cameras', [*run, 'cameras', '--hardware', str(hw_file), '--socket', camera], hw_env)
        _unit('roborun-station', [*run, 'hardware-station', '--hardware', str(hw_file),
                                   '--sessions', str(root / 'log' / 'video-motion' / 'sessions')], hw_env)
        import time as _t
        status = None
        for _ in range(60):
            if _failure_path().exists():
                raise SystemExit('the arms did not come up')    # the reason is in that file, and is what gets printed
            try:
                with urllib.request.urlopen('http://127.0.0.1:8091/status', timeout=1) as r:
                    status = json.load(r)
                break
            except Exception:
                _t.sleep(1)
        else:
            raise SystemExit('the arms did not come up: ' + ('journalctl --user -u roborun-station' if _systemd()
                                                             else f"see {HOME / 'logs' / 'roborun-station.log'}"))
        for side, row in sorted((status.get('arm_status') or {}).items()):
            for text in row.get('warnings') or ():
                print(f'warning: {text}')
            if row.get('dry_run'):
                print(f'{side} arm: DRY RUN, nothing is enabled or written')
        print('arms: ' + ', '.join(f"{s} {c['hardware'][s]['arm']}" for s in ('left', 'right') if c['hardware'].get(s)))
    # the backend: the robot's own Roborun, on its managed config (profile switches from the UI rewrite it and restart this unit)
    blur = face_blur(c)
    if not blur:
        print('face blur is OFF (privacy: face_blur: false): camera frames go to the motion service unblurred, and history images are not uploaded')
    entry = shlex.split(os.environ.get('MANIFEST_BACKEND_ENTRY', '')) or [str(root / 'server.py')]
    env = dict(MANIFEST_JOBS='api', MANIFEST_JOBS_CONFIG=c['jobs'], MANIFEST_WEB_UNIT='roborun-robot.service', MANIFEST_PROFILES_FROM_SDK='1',
               MANIFEST_PROFILE_ID=current_profile(root), MANIFEST_CAMERA_SOCKET=camera,
               MANIFEST_DREAMAPI=str(root), PYTHONUNBUFFERED='1', PYTHONPATH=_sdk_path(), ROBORUN_HOME=str(HOME),
               ROBORUN_FACE_BLUR='1' if blur else '0')
    if c.get('sim'):
        env['MANIFEST_SIMULATION'] = str(c['sim'])        # the backend says it is a simulation (and of which scene) to the page and the API
    for k in ('MUJOCO_GL', 'MANIFEST_MODULE_LOG', 'MANIFEST_PLUGINS_DIR'):
        if os.environ.get(k): env[k] = os.environ[k]
    sep = ['--'] if '--' in entry and entry[-1] != '--' else []
    _unit('roborun-robot', [sys.executable, *entry, *sep, '--config', str(root / 'config.json'), '--port', str(port), '--host', '127.0.0.1'], env)
    print(f'robot: 127.0.0.1:{port}, profile {shown_profile(current_profile(root))}')
    if not _backend_up(c['backend']):
        raise SystemExit('the robot did not start: ' + ('journalctl --user -u roborun-robot' if _systemd()
                                                        else f"see {HOME / 'logs' / 'roborun-robot.log'}"))
    _unit('roborun-local', [sys.executable, '-m', 'roborun_sdk.cli', 'local'],
          dict(PYTHONPATH=_sdk_path(), PYTHONUNBUFFERED='1', ROBORUN_HOME=str(HOME)))
    from .overhead import check as overhead_check, describe as overhead_line
    overhead = None if c.get('sim') else overhead_check(c['backend'])
    if overhead:
        print('overhead camera: ' + overhead_line(overhead))
    ready(c)
    return 0


APP_URL = 'https://app.roborun.app'
LINGER_DIR = Path('/var/lib/systemd/linger')
LINGER_MARK = 'linger-by-roborun'            # in HOME: Roborun turned linger on, so `roborun uninstall` turns it off again


def viewer_url(c=None):
    """This robot's own read-only viewer, served by the page connector on this computer (roborun_sdk/viewer), or None."""
    from .local_proxy import PORT
    c = c if c is not None else (json.loads(CONFIG.read_text()) if CONFIG.exists() else {})
    port = int(c.get('live_port', PORT) or 0)
    return f'http://127.0.0.1:{port}/' if port else None


def _user():
    import getpass
    try:
        return getpass.getuser()
    except Exception:
        return os.environ.get('USER') or ''


def linger_state():
    """Whether this user's Roborun keeps running after they log out: True / False under systemd --user (its "linger"), None
    where that does not apply (no systemd: plain background processes, which a logout does not stop)."""
    if not _systemd() or not shutil.which('loginctl'):
        return None
    return (LINGER_DIR / _user()).exists()


def keep_running_after_logout(say=print):
    """systemd stops a user's services when their last session ends, unless "linger" is on. `roborun up` turns it on for this
    user where that needs no password (a desktop session; ROBORUN_NO_LINGER=1 skips it). Where it needs an administrator (an
    SSH session on Ubuntu Server, for one), it says the one command to run."""
    import subprocess
    if linger_state() is not False:
        return
    user = _user()
    if not os.environ.get('ROBORUN_NO_LINGER'):
        try:
            subprocess.run(['loginctl', 'enable-linger', user], capture_output=True, stdin=subprocess.DEVNULL, timeout=15)
        except (OSError, subprocess.SubprocessError):
            pass
        if linger_state():
            HOME.mkdir(parents=True, exist_ok=True); (HOME / LINGER_MARK).write_text(user + '\n')
            say('kept running after you log out: Roborun turned on `loginctl enable-linger` for you '
                '(roborun uninstall turns it off)')
            return
    say(f'note: Roborun stops when you log out of this computer. To keep it running (an administrator command): '
        f'sudo loginctl enable-linger {user}')


def ready(c, say=print):
    """The last lines of every `roborun up`, the same for an arm and a humanoid profile."""
    keep_running_after_logout(say)
    view = viewer_url(c)
    say(f'ready: open {APP_URL} on this computer' + (f", or this robot's own viewer: {view}" if view else ''))
    say(f"local API: {c['backend']}/v1/docs   (code and AI assistants: roborun docs agents)")
    notice = net.update_notice()
    if notice:
        say(notice)


def cmd_down(a) -> int:
    """Everything stops, and no arm is dropped: an arm that is not resting on something stays held until it is parked or supported."""
    from .arms.hardware import ask_station
    held = None
    bad = {}
    if _state('roborun-station') in ('active', 'running'):
        held = ask_station('park' if a.park else 'release' if a.release else 'down')
        from .arms.hardware import faulted
        bad = {side: held.pop(side) for side in faulted(held)}           # faulted: not holding, so never a reason to keep running
    for u in UNITS:
        if u != 'roborun-station' or not held:
            _stop(u)
    if bad:
        print('These arms faulted and their motors may be off: if one is raised, support it now, then check its power and cable '
              '(roborun hw check):')
        for side, why in bad.items():
            print(f'  {side} arm: {why}')
    if not held:
        print('Roborun is stopped on this computer')
        return 1 if bad else 0
    print('Roborun is stopped on this computer, except that these arms are still powered and holding their position:')
    for side, why in held.items():
        print(f'  {side} arm: {why}')
    print('Switching them off now would let them fall. To finish, either:')
    print('  roborun down --park      moves them slowly to their rest pose, then switches the motors off')
    print('  roborun down --release   after you have supported them by hand: switches the motors off where they are')
    return 0


def _get(url, key=None, timeout=3):
    headers = {'User-Agent': UA}
    if key: headers['Authorization'] = 'Bearer ' + key
    with urllib.request.urlopen(urllib.request.Request(url, headers=headers), timeout=timeout) as r:
        return json.load(r)


def cmd_status(a) -> int:
    if not CONFIG.exists():
        print('Roborun is not set up on this computer yet: roborun up')
        return 1
    c = json.loads(CONFIG.read_text())
    states = {u: _state(u) for u in UNITS}
    on = {u for u, st in states.items() if st in ('active', 'running')}
    hardware = c.get('hardware') or {}
    expected = {'roborun-robot', 'roborun-local'} | ({'roborun-sim'} if c.get('sim') and not c.get('body') else set())   # a humanoid simulates inside its robot service
    if hardware and not c.get('sim'):
        expected |= {'roborun-station'} | ({'roborun-cameras'} if hardware.get('cameras') else set())
    for u in ('roborun-robot', 'roborun-local', 'roborun-station', 'roborun-cameras', 'roborun-sim'):
        if u in on or u in expected:
            print(f"{LABEL[u]:<17} {'running' if u in on else 'stopped'}")
    print(f"{'robot name':<17} {c['robot']}")
    backend = c.get('backend') or 'http://127.0.0.1:8120'
    try:
        st = _get(backend + '/v1/status')
        from .arm_profiles import shown_profile
        print(f"{'profile':<17} {shown_profile(st.get('profile'))}")
        print(f"{'local API':<17} {backend} (answering)")
        task = st.get('task')
        print(f"{'task':<17} {task['id'] + ' running' if isinstance(task, dict) and task.get('id') else 'none running'}")
        if isinstance(st.get('guard'), dict):                   # a humanoid: whether anything stops it before an obstacle
            g = st['guard']
            print(f"{'obstacle guard':<17} " + ('on (head depth)' if g.get('obstacle_guard') else f"NONE: {g.get('note')}"))
        if 'overhead_camera' in st:
            from .overhead import describe
            print(f"{'overhead camera':<17} {describe(st['overhead_camera'])}")
    except Exception:
        print(f"{'local API':<17} {backend} (not answering)")
    from .local_proxy import PORT
    port = int(c.get('live_port', PORT) or 0)
    if port:
        try:
            with urllib.request.urlopen(urllib.request.Request(f'http://127.0.0.1:{port}/roborun/hello', headers={'Origin': 'https://app.roborun.app'}), timeout=2) as r:
                ready = json.load(r).get('ready')
            print(f"{'page connection':<17} 127.0.0.1:{port} ({'ready' if ready else 'waiting for the robot'})")
        except Exception:
            print(f"{'page connection':<17} 127.0.0.1:{port} (not answering)")
    try:
        key = Path(c['key_file']).read_text().strip()
    except OSError:
        key = None
    said = token_state(c['server'], key)[1] if key else f"no key in {c.get('key_file')}: roborun login"
    print(f"{'motion service':<17} {c['server']} ({said})")
    signing = key_date_notice(c)
    if signing:
        print(f"{'signing key':<17} {signing}")
    runtime = installed_runtime(c['manifest']) if c.get('manifest') else None
    print(f"{'version':<17} {UA}" + (f', runtime {runtime}' if runtime else ''))
    notice = net.update_notice()
    if notice:
        print(f"{'update':<17} {notice}")
    blur = face_blur(c)
    print(f"{'face blur':<17} {'on: every image is blurred before it leaves this computer' if blur else 'OFF: frames go to the motion service unblurred (privacy: face_blur: false)'}")
    d = c.get('data') or {}
    print(f"{'history upload':<17} {'off' if not d.get('enabled') else 'on' if blur else 'task records only (face blur is off)'}")
    try:                                                       # episodes the dataset refused since the last one it added
        from .dataset import refusal_line
        refused = refusal_line(dataset_root(c['robot']))
        if refused:
            print(f"{'dataset':<17} {refused}")
    except Exception:
        pass
    view = viewer_url(c)
    if view and 'roborun-local' in on:
        print(f"{'viewer':<17} {view}  (this robot's tasks, recordings, cameras and Stop; no sign-in)")
    lingering = linger_state()
    if lingering is not None:
        print(f"{'after logout':<17} " + ('keeps running' if lingering else
                                          f'stops (to keep it running: sudo loginctl enable-linger {_user()})'))
    if 'roborun-robot' not in on:
        print('next: roborun up   (no arms: roborun up --sim cup; logs: ' +
              ('journalctl --user -u roborun-robot' if _systemd() else str(HOME / 'logs')) + ')')
    return 0


INSTALL_URL = os.environ.get('ROBORUN_INSTALL_URL', 'https://roborun.app/install.sh')


LATEST_URL = os.environ.get('ROBORUN_DOWNLOADS', 'https://roborun.app/dl').rstrip('/') + '/latest.json'


def latest_build(timeout=15):
    """The build install.sh installs now (roborun.app/dl/latest.json). Raises OSError / ValueError when it cannot be read."""
    request = urllib.request.Request(LATEST_URL, headers={'User-Agent': UA, 'Cache-Control': 'no-cache'})
    with urllib.request.urlopen(request, timeout=timeout) as reply:
        return json.load(reply)['build']


def installed_build():
    """The build install.sh last installed here (~/.roborun/build.json), read from disk: after an upgrade it is the new one."""
    try:
        return json.loads((Path(sys.prefix).parent / net.BUILD_FILE).read_text())['build']
    except (OSError, ValueError, KeyError, TypeError):
        return net.build()


def cmd_upgrade(a) -> int:
    """The newest Roborun, the way install.sh installs it; then the robot restarts on it, unless a task is running (it then
    keeps running on the version it started with, and `roborun up` after the task switches). Nothing changes when this is
    already the newest, or when roborun.app cannot be reached."""
    import subprocess, tempfile
    have = installed_build()
    try:
        latest = latest_build()
    except (OSError, ValueError, KeyError, TypeError) as exc:
        why = getattr(exc, 'reason', None) or exc
        raise SystemExit(f'could not reach roborun.app ({str(why)[:120]}): check the internet connection or proxy. '
                         f'Still on {have or "this version"}; nothing was changed') from None
    if latest == have and not getattr(a, 'force', False):
        print(f'already on the newest Roborun ({have}): nothing to upgrade')
        return 0
    print(f'upgrading Roborun {have or "(source checkout)"} -> {latest} ...', flush=True)
    with tempfile.TemporaryDirectory() as tmp:
        script = Path(tmp) / 'install.sh'
        got = subprocess.run(['curl', '-fsSL', '--retry', '2', INSTALL_URL, '-o', str(script)])
        if got.returncode or not script.is_file() or not script.stat().st_size:
            raise SystemExit(f'could not download the installer from {INSTALL_URL} (curl exit {got.returncode}): check the internet '
                             f'connection or proxy. Still on {have or "this version"}; nothing was changed')
        done = subprocess.run(['sh', str(script)], env=dict(os.environ, ROBORUN_UPGRADING='1'))
    now = installed_build()
    if done.returncode or (now != latest and now == have):
        raise SystemExit(f'the upgrade did not finish (see above). Still on {have or "this version"}; nothing was restarted: the '
                         'robot keeps running as it was. Next: check the internet connection and run roborun upgrade again')
    state = f' ({now})' if now else ''
    if _state('roborun-robot') not in ('active', 'running'):
        print('upgraded' + state + '. Next: roborun up')
        return 0
    c = json.loads(CONFIG.read_text()) if CONFIG.exists() else {}
    if _busy(c.get('backend') or 'http://127.0.0.1:8120'):
        print('upgraded' + state + '. A task is running, so the robot was not restarted: run roborun up after the task ends')
        return 0
    roborun = shutil.which('roborun') or str(Path.home() / '.local' / 'bin' / 'roborun')
    print('upgraded' + state + '; restarting the robot on the new version ...', flush=True)
    return subprocess.run([roborun, 'up']).returncode


RC_BLOCK = '\n# Roborun\nexport PATH="$HOME/.local/bin:$PATH"\n'          # what install.sh adds to a shell's startup file
RC_FILES = ('.bashrc', '.bash_profile', '.zshrc', '.profile')


def _ask_yes(question):
    if not sys.stdin.isatty():
        return False
    try:
        return input(question + ' [y/N] ').strip().lower() in ('y', 'yes')
    except EOFError:
        return False


def cmd_uninstall(a) -> int:
    """Remove Roborun from this computer: stop it (no arm is dropped), then ~/.roborun, the `roborun` command and the PATH line
    install.sh added. Your settings, key and recordings are kept unless you say otherwise (--purge, or yes when asked)."""
    import subprocess
    root, home = install_root(), Path.home()
    if not (a.yes or _ask_yes(f'Remove Roborun from this computer ({root})?')):
        raise SystemExit('nothing was removed. To remove Roborun without being asked: roborun uninstall --yes')
    if any(_state(u) in ('active', 'running') for u in UNITS):
        print('stopping Roborun ...', flush=True)
        cmd_down(argparse.Namespace(park=a.park, release=a.release))
        if any(_state(u) in ('active', 'running') for u in UNITS):
            raise SystemExit('nothing was removed: an arm is still powered and held (see above). Finish with roborun down --park '
                             '(or --release while you support it), then run roborun uninstall again')
    c = json.loads(CONFIG.read_text()) if CONFIG.exists() else {}
    runtime = Path(c.get('manifest') or home / 'roborun' / 'robot' / 'runtime').expanduser()
    top = runtime.parents[1] if _ours(runtime) else None
    removed, kept = [], []
    mark = HOME / LINGER_MARK
    if mark.is_file() and shutil.which('loginctl'):
        subprocess.run(['loginctl', 'disable-linger', _user()], capture_output=True, stdin=subprocess.DEVNULL)
        removed.append('keep-running-after-logout (loginctl disable-linger)')
    for name in RC_FILES:
        rc = home / name
        try:
            text = rc.read_text()
        except OSError:
            continue
        if RC_BLOCK in text:
            rc.write_text(text.replace(RC_BLOCK, '\n', 1)); removed.append(f'the PATH line in {rc}')
    link = home / '.local' / 'bin' / 'roborun'
    if link.is_symlink() and str(Path(os.readlink(link))).startswith(str(root)):
        link.unlink(); removed.append(str(link))
    data = [p for p in (HOME, top) if p is not None and p.exists()]
    purge = a.purge or (not a.keep_data and data and _ask_yes(
        'Also delete your settings, key and recordings (' + ', '.join(str(p) for p in data) + ')?'))
    for p in data:
        if purge:
            shutil.rmtree(p, ignore_errors=True); removed.append(str(p))
        else:
            kept.append(str(p))
    if (root / 'python').is_dir() or (root / 'tools').is_dir():
        shutil.rmtree(root, ignore_errors=True); removed.append(str(root))
    print('Roborun is removed from this computer.' if removed else 'Roborun was not installed here: nothing to remove.')
    for r in removed:
        print(f'  removed {r}')
    if kept:
        print('Kept (your settings, key and recordings): ' + ', '.join(kept))
        print('  delete them when you no longer want them: rm -rf ' + ' '.join(kept))
    print('Install again any time: curl -fsSL https://roborun.app/install.sh | sh')
    return 0


def cmd_view(a) -> int:
    """Print (and, with a desktop, open) this robot's own viewer."""
    url = viewer_url()
    if not url:
        raise SystemExit('the viewer is off on this robot (live_port is 0 in its config)')
    if _state('roborun-local') not in ('active', 'running'):
        print(f'Roborun is not running here, so the viewer at {url} does not answer. Next: roborun up')
        return 1
    print(url)
    if not a.no_open and (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY') or sys.platform == 'darwin'):
        import webbrowser
        webbrowser.open(url)
    return 0


def _read_update_after():
    try:
        return f" ({json.loads((Path(sys.prefix).parent / net.BUILD_FILE).read_text())['build']})"
    except (OSError, ValueError, KeyError, TypeError):
        return ''


def add_parsers(sub):
    u = sub.add_parser('up', help='start Roborun for this robot on this computer (first run: also sets it up)')
    for flag, text in (('--server', 'the Roborun API (default https://api.roborun.app)'), ('--robot', "a name for this robot (default: this computer's name)"),
                       ('--key-file', 'a file holding your key (default: the one roborun login saved)'),
                       ('--profile', 'an arm profile: yam (one YAM arm), yam_dual (two), any catalog arm; see roborun profiles'),
                       ('--rig', 'your own rig.json instead of a profile'), ('--runtime', 'where the robot runtime is installed'),
                       ('--data-root', 'folder of task records (default: inside the runtime)'), ('--video-motion', 'folder of recordings')):
        u.add_argument(flag, help=text, **({'dest': 'manifest'} if flag == '--runtime' else {}))
    u.add_argument('--profile-id', help=argparse.SUPPRESS)          # the page's profile switch: the runtime's id, as is
    u.add_argument('--backend', default='http://127.0.0.1:8120', help='address of the local robot API (this computer only)')
    u.add_argument('--insecure', action='store_true', help='allow a plain http --server on another computer (the key travels unencrypted)')
    u.add_argument('--data-sync', action='store_true', help='upload the history again after --no-data-sync')
    u.add_argument('--no-data-sync', action='store_true', help="do not upload this robot's task records and recordings (history)")
    u.add_argument('--dataset', action='store_true', help="record episodes into this robot's dataset again after --no-dataset")
    u.add_argument('--no-dataset', action='store_true', help="do not install the recording tools or record episodes (remembered)")
    u.add_argument('--sim', help='simulate instead of using real arms: a scene (' + ', '.join(SCENES) + '; a humanoid: room or '
                                 'empty). With no profile: two simulated YAM arms (yam_dual)')
    u.add_argument('--no-sim', action='store_true', help='stop simulating and use the real arms')
    u.add_argument('--left', help='the real left arm: ARM@ADDRESS, e.g. yam@slcan://SERIAL (see roborun hw arms)')
    u.add_argument('--right', help='the real right arm: ARM@ADDRESS')
    u.add_argument('--camera', action='append', help='a real camera: top=SOURCE, left=SOURCE or right=SOURCE (repeatable)')
    u.add_argument('--arm-options', action='append', help='left=FILE or right=FILE: a JSON file of driver options (limits, home, ...)')
    u.add_argument('--body-agent', help='a humanoid profile on a real robot: the address of the agent on the robot, http://ROBOT:PORT')
    u.add_argument('--body-agent-key-file', help='with --body-agent: the file holding that agent\'s key')
    u.set_defaults(func=cmd_up)
    d = sub.add_parser('down', help='stop Roborun on this computer; an arm that would fall stays held and you are told what to do')
    d.add_argument('--park', action='store_true', help='move held arms slowly to their rest pose, then switch their motors off')
    d.add_argument('--release', action='store_true', help='you are supporting the arms: switch their motors off where they are')
    d.set_defaults(func=cmd_down)
    sub.add_parser('status', help='what is running here, and whether the token and the motion service work').set_defaults(func=cmd_status)
    g = sub.add_parser('upgrade', help='install the newest Roborun, then restart the robot on it (not while a task runs)')
    g.add_argument('--force', action='store_true', help='install the newest release again even when it is the one installed')
    g.set_defaults(func=cmd_upgrade)
    x = sub.add_parser('uninstall', help='remove Roborun from this computer (asks before deleting your settings and recordings)')
    x.add_argument('--yes', action='store_true', help='do not ask whether to remove Roborun')
    x.add_argument('--keep-data', action='store_true', help='keep your settings, key and recordings without asking')
    x.add_argument('--purge', action='store_true', help='also delete your settings, key and recordings, without asking')
    x.add_argument('--park', action='store_true', help='if an arm is held: move it slowly to its rest pose first')
    x.add_argument('--release', action='store_true', help='if an arm is held and you support it: switch its motors off first')
    x.set_defaults(func=cmd_uninstall)
    v = sub.add_parser('view', help="this robot's own viewer on this computer: tasks, recordings, cameras, Stop (no sign-in)")
    v.add_argument('--no-open', action='store_true', help='print the address only')
    v.set_defaults(func=cmd_view)
    pr = sub.add_parser('profiles', help='the arm profiles available here (* = running); profiles create: one for your own arm'); pr.add_argument('--runtime', dest='manifest', help='where the robot runtime is installed')
    pr.set_defaults(func=cmd_profiles)
    s = sub.add_parser('setup', help='guided setup of your arms and cameras (run it again any time to change them)')
    s.add_argument('--detect', action='store_true', help='print what is connected and which arms it could be, asking nothing and moving nothing')
    s.add_argument('--json', action='store_true', help='with --detect: as JSON')
    s.add_argument('--restart', action='store_true', help='forget the answers of an unfinished guided setup')
    s.add_argument('--server', default=SERVER, help='the Roborun API'); s.add_argument('--robot', help='without the guided setup: a name for this robot')
    s.add_argument('--key-file', help='without the guided setup: a file holding your access token'); s.add_argument('--profile', help='a pre-made profile (omit to list them)')
    s.add_argument('--rig', help='your own rig.json instead of a pre-made profile')
    s.add_argument('--backend', default='http://127.0.0.1:8120', help='address of the local robot API (this computer only)')
    s.add_argument('--insecure', action='store_true', help='allow a plain http --server on another computer (the key travels unencrypted)')
    s.add_argument('--runtime', dest='manifest', help='where the robot runtime is installed (default ~/roborun/robot/runtime)')
    s.add_argument('--data-root', help='folder of task records (default: inside the runtime)')
    s.add_argument('--video-motion', help='folder of clips and recordings')
    s.add_argument('--no-data-sync', action='store_true', help="do not upload this robot's task records and recordings (history)")
    s.set_defaults(func=cmd_setup)
    sub.add_parser('local', help='run the page connection and history upload (roborun up starts this)').set_defaults(func=cmd_local)
    y = sub.add_parser('sync', help='upload new task records and recordings now')
    y.add_argument('--once', action='store_true', default=True, help='one pass, then exit (the default)')
    y.set_defaults(func=cmd_sync)
