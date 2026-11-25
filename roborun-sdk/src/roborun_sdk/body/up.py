"""roborun up for a humanoid profile:

    roborun up --profile booster_k1 --sim room                    the packaged simulation (scenes: room, empty)
    roborun up --profile galaxea_r1 --body-agent http://R1:8302 --body-agent-key-file ~/r1-agent.key     a real R1
    roborun up --profile my_rover --body ~/my-rover/my_rover.py:MyRover        your own body: your profile and your class
    roborun up                                                    afterwards: the same again

Your own body profiles are <ROBORUN_HOME>/bodies/<id>.yaml (profiles.py); your own body class is named with --body or `body:`
in robot.yaml (custom.py), and remembered per profile like an agent.

It starts the body's local robot API on the backend address (127.0.0.1:8120) as the roborun-robot service, and the page
connector (roborun-local); both are this home's units (robot.unit_name: another ROBORUN_HOME gets its own). No arm runtime
is installed or started for a humanoid profile. A real body is used only when its agent is given (and remembered per
profile); a profile that is neither connected nor set up to simulate does not start, and is never replaced by a simulation.
"""
from __future__ import annotations

import json
import sys
import urllib.parse
from pathlib import Path

from . import profiles


def _arm_name(name, config=None):
    """True when `name` means arms: a catalog arm or pair, a pre-made or renamed profile, or one made on this computer."""
    from .. import robot
    from ..arm_profiles import FROM_RUNTIME, PREMADE, RENAMED, RESERVED, TO_RUNTIME, models, parse
    if any(name in known for known in (PREMADE, RENAMED, RESERVED, TO_RUNTIME, FROM_RUNTIME, models(), robot.bundled())):
        return True
    try:
        if parse(name) is not None:
            return True
    except Exception:
        pass
    from ..devkit import runtime_root
    roots = [robot.HOME / 'profiles', Path.home() / '.config' / 'manifest' / 'profiles', runtime_root() / 'profiles']
    if (config or {}).get('manifest'):
        roots.append(Path(config['manifest']).expanduser() / 'profiles')
    return any((r / name).is_dir() for r in roots)


def from_file(a, config=None):
    """Put robot.yaml's body: (settings leave it on a.body_file) on a.body when this `roborun up` is for its profile: the one its
    section names (profile:), else the one its class names (`profile = 'my_rover'`), else robot.yaml's profile:. It is not used
    with --body or --body-agent, with arms named (--rig, --left, --right, --sim cup), or with --profile naming anything else (another body,
    arms, the page's switch to them): robot.yaml's class never runs as, nor is remembered for, another profile."""
    from ..arms.settings import arms_named
    fb = getattr(a, 'body_file', None)
    if not fb or getattr(a, 'body', None) or getattr(a, 'body_agent', None) or arms_named(a):
        return
    own = fb.get('profile')
    if a.profile and own and a.profile != own:
        return
    if a.profile and not own and not profiles.is_body(a.profile):
        return                                                # arms (or an unknown name): the class is not even imported
    if not own:
        from .custom import BodyLoadError, default_profile, resolve_ref
        try:
            own = default_profile(resolve_ref(fb.get('adapter') or '', fb.get('base_dir'))) or fb.get('top_profile')
        except BodyLoadError as exc:
            raise SystemExit(f'robot.yaml body: {exc}') from None
    if a.profile and a.profile != own:
        return
    a.body = dict(fb, profile=own) if own else dict(fb)
    a.body.pop('top_profile', None)
    a.body_from_file = True
    a.profile = a.profile or own
    if not own:
        raise SystemExit(f'robot.yaml body: name the profile it is for (body:\n  adapter: {fb.get("adapter")}\n  profile: <id>), '
                         f'or give the class `profile = \'<id>\'` ({profiles.available()})')


def _body_ref(a):
    """--body, or robot.yaml's body: (from_file() puts it on a.body): {ref, options, profile} with a file path made absolute
    (profile: the one robot.yaml's section names, else None), or None."""
    value = getattr(a, 'body', None)
    if not value:
        return None
    from .custom import BodyLoadError, resolve_ref
    if isinstance(value, dict):
        value = dict(value)
        ref, base_dir = value.pop('adapter', None), value.pop('base_dir', None)
        value.pop('top_profile', None)
        profile = value.pop('profile', None)
        options = value
    else:
        ref, base_dir, options, profile = value, None, {}, None
    if not ref:
        raise SystemExit('robot.yaml: body needs "adapter:" (your body class: my_body.py:MyBody)')
    try:
        return dict(ref=resolve_ref(ref, base_dir), options=options, profile=profile)
    except BodyLoadError as exc:
        raise SystemExit(f'--body: {exc}') from None


def wanted(a, config):
    """True when this `roborun up` runs a body profile: one is named, or a body class is given, or the robot runs one and no
    arm or profile is named. A name that is neither a body nor arms, or one given with a body's flags, says which bodies there are."""
    from_file(a, config)
    body_flags = bool((getattr(a, 'body', None) and not getattr(a, 'body_from_file', False)) or getattr(a, 'body_agent', None)
                      or getattr(a, 'sim', None) in profiles.SCENES)
    if a.profile:
        if profiles.is_body(a.profile):
            return True
        if body_flags or not _arm_name(a.profile, config):
            raise SystemExit(str(profiles.unknown(a.profile)) + ('. Arm profiles: roborun profiles' if not body_flags else
                             '. (--body, --body-agent and --sim room|empty are for a body profile)'))
        return False
    if getattr(a, 'body', None):
        return True
    from ..arms.settings import arms_named
    if arms_named(a):                                         # --rig, --left/--right, --sim cup: arms, even after a body ran
        return False
    return bool((config or {}).get('body'))


def up(a) -> int:
    from .. import robot
    from ..arms.settings import settle_token
    c = json.loads(robot.CONFIG.read_text()) if robot.CONFIG.exists() else {}
    from_file(a, c)                                           # robot.yaml's body: only for its own profile
    own = _body_ref(a)                                        # your own body class (--body, robot.yaml body:)
    named = _class_profile(own['ref']) if own else None       # it loads now (the service makes the body) and may name its profile
    stated = own.pop('profile') if own else None              # the profile robot.yaml's body: section names for it
    if own and a.profile and named and named != a.profile and stated != a.profile:
        raise SystemExit(f"--body {own['ref']} is the body class of profile {named!r} (its profile attribute), not {a.profile!r}: "
                         f"run roborun up --profile {named} --body ..., or name it in robot.yaml (body: profile: {a.profile}) "
                         f"if it really drives {a.profile}")
    pid = a.profile or named or (c.get('body') or {}).get('profile')
    if not pid:
        raise SystemExit(f'name the body\'s profile: roborun up --profile <id> --body ... ({profiles.available()})')
    if not profiles.is_body(pid):
        raise SystemExit(str(profiles.unknown(pid)))
    try:
        profile = profiles.load(pid)                          # your own profile is checked here, before anything starts
    except profiles.ProfileError as exc:
        raise SystemExit(str(exc)) from None
    if getattr(a, 'body_agent', None) and not profile.get('agent'):
        raise SystemExit(f'{pid}: its profile names no agent protocol (agent: protocol: booster|galaxea): run your own body '
                         f'class instead (roborun up --profile {pid} --body my_body.py:MyBody)')
    if not c:
        if not (a.robot and a.key_file):
            raise SystemExit(f'first run: roborun up --robot <a name for this robot> --key-file <file with your access token> '
                             f'--profile {pid} --sim room   (or --body-agent http://ROBOT:PORT for a real one)')
        c = dict(server=(a.server or robot.SERVER).rstrip('/'), robot=a.robot, backend=a.backend,
                 data=dict(enabled=False, opt_out=False, sources={}, state=str(robot.HOME / 'robot' / 'sync-state.json')))
    if a.server:
        from .. import net
        net.check_server(a.server, getattr(a, 'insecure', False))
        if a.server.rstrip('/') != str(c.get('server') or '').rstrip('/'):
            c['server'] = a.server.rstrip('/')
            robot.CONFIG.write_text(json.dumps(c, indent=1)); print(f"server: {c['server']}", flush=True)
    if a.key_file:
        c['key_file'] = str(Path(a.key_file).expanduser().resolve())
    if not c.get('key_file'):
        raise SystemExit('no access token: pass --key-file <file with your access token>')
    settle_token(c['key_file'], c['server'])
    if robot._busy(c.get('backend') or 'http://127.0.0.1:8120') and pid != c.get('profile'):
        raise SystemExit('a task is running on this robot: stop it before changing the profile')

    agents = c.setdefault('body_agents', {})
    adapters = c.setdefault('body_adapters', {})
    if own and getattr(a, 'body_agent', None):
        raise SystemExit('--body and --body-agent: give one (your own body class, or the agent on the robot)')
    before = adapters.get(pid), agents.get(pid)               # put back if your class does not start: never remember a broken one
    if own:
        adapters[pid] = own
        agents.pop(pid, None)
    if getattr(a, 'body_agent', None):
        adapters.pop(pid, None)
        if not getattr(a, 'body_agent_key_file', None) and not (agents.get(pid) or {}).get('key_file'):
            raise SystemExit('--body-agent needs --body-agent-key-file <the file holding the agent key from the robot>')
        url = a.body_agent.rstrip('/')
        if urllib.parse.urlsplit(url).scheme not in ('http', 'https'):
            raise SystemExit('--body-agent: the agent\'s address, http://ROBOT:PORT')
        key_file = getattr(a, 'body_agent_key_file', None) or agents[pid]['key_file']
        agents[pid] = dict(url=url, key_file=str(Path(key_file).expanduser().resolve()))
    scene = None
    if getattr(a, 'sim', None):
        if a.sim not in profiles.SCENES:
            raise SystemExit(f"scene {a.sim!r}: a humanoid simulates one of {', '.join(profiles.SCENES)}")
        scene = a.sim
    elif c.get('sim') and not getattr(a, 'no_sim', False) and not getattr(a, 'body_agent', None) and not agents.get(pid) \
            and not own and not adapters.get(pid) and (profile.get('sim') or {}).get('model'):
        # a profile with a saved real agent or body class runs that; only an explicit --sim simulates it
        scene = c['sim'] if c['sim'] in profiles.SCENES else profiles.DEFAULT_SCENE     # this robot simulates: so does its next profile
    if scene and not (profile.get('sim') or {}).get('model'):
        raise SystemExit(f'{pid} has no simulation model: add sim: model: <your MJCF file> to its profile '
                         f'({profile.get("file") or "packaged"}), or run your body class (--body my_body.py:MyBody)')
    if scene is None and not agents.get(pid) and not adapters.get(pid):
        raise SystemExit(f'{pid} is not connected here: run your own body class (roborun up --profile {pid} --body my_body.py:MyBody), '
                         f'give its agent (--body-agent http://ROBOT:PORT --body-agent-key-file FILE), or simulate it '
                         f'(roborun up --profile {pid} --sim room)')
    if scene:
        _refuse_live_for_sim(c, pid, scene)
        c['sim'] = scene
    else:
        c.pop('sim', None)
    body = dict(profile=pid, sim=scene, agent=None if scene or adapters.get(pid) else agents[pid])
    if not scene and adapters.get(pid):
        body['adapter'] = adapters[pid]
    c.update(profile=pid, body=body)
    c.setdefault('backend', a.backend or 'http://127.0.0.1:8120')
    robot.HOME.mkdir(parents=True, exist_ok=True)
    robot.CONFIG.write_text(json.dumps(c, indent=1)); robot.CONFIG.chmod(0o600)
    robot.remember_ports(a, c)

    for unit in ('roborun-sim', 'roborun-station', 'roborun-cameras', 'roborun-robot'):
        robot._stop(unit)                                    # this home's arm robot parts, or this body's previous run
    robot._check_ports(c, None)                              # its API and page ports: free, or a clear error (never stopped)
    env = dict(PYTHONPATH=robot._sdk_path(), PYTHONUNBUFFERED='1', ROBORUN_HOME=str(robot.HOME))
    if scene:
        env.update(robot.gl_env())
    robot._unit('roborun-robot', [sys.executable, '-m', 'roborun_sdk.cli', 'body', 'serve'], env)
    what = (f'simulation, scene {scene}' if scene else f"your body class {body['adapter']['ref']}" if body.get('adapter')
            else f"the robot at {agents[pid]['url']}")
    print(f"{profile['label']} ({pid}): {what}", flush=True)
    if not robot._backend_up(c['backend'], wait_s=90):
        log = robot.logs_hint('roborun-robot')
        if own and body.get('adapter'):
            robot._stop('roborun-robot')                     # it would retry the class that did not start, every few seconds
            _forget(c, pid, *before)
            raise SystemExit(f"the robot did not start with your body class {own['ref']}, so it is not remembered for {pid}: {log}")
        raise SystemExit('the robot did not start: ' + log)
    robot._unit('roborun-local', [sys.executable, '-m', 'roborun_sdk.cli', 'local'], dict(env))
    robot.ready(c)                                           # the same last lines as an arm profile
    print('move it: POST /v1/body/move {"prompt": "go to the table"}   stop: roborun stop (or POST /v1/stop)')
    return 0


def _refuse_live_for_sim(c, pid, scene):
    """A body simulation stops this home's robot parts before it starts (below): never while they run the live robot. Refused,
    with nothing changed, while this install's arms station or camera service runs, or its robot service runs a real robot
    (robot.json does not say simulation): stopping them here would be a bare stop, without the park or hold that roborun down
    asks the arms station for. The same rule as an arm simulation (robot._refuse_live_for_sim); a body simulation uses no
    station port or camera socket, so a station on :8091 from another install is not counted."""
    from .. import sim_home
    running = sim_home.live_parts(c, backend=True, port=False)
    if running:
        raise SystemExit(f"roborun up --profile {pid} --sim {scene}: the live robot is running here ({', '.join(running)}), and "
                         'a simulation does not stop it. Nothing was changed. Next: roborun down (it parks or holds the arms as '
                         f'usual), then roborun up --profile {pid} --sim {scene}')


def _forget(c, pid, adapter, agent):
    """robot.json as it was for `pid` before a body class that did not start: its earlier class or agent, or none."""
    from .. import robot
    for key, value in (('body_adapters', adapter), ('body_agents', agent)):
        if value:
            c.setdefault(key, {})[pid] = value
        else:
            c.setdefault(key, {}).pop(pid, None)
    body = c.get('body') or {}
    body.pop('adapter', None)
    if adapter:
        body.update(adapter=adapter, agent=None)
    elif agent:
        body['agent'] = agent
    robot.CONFIG.write_text(json.dumps(c, indent=1)); robot.CONFIG.chmod(0o600)


def _class_profile(ref):
    """The profile your body class names (`profile = 'my_rover'`), or None; the class is imported here, so a missing file, a
    missing class or an error in it stops roborun up before anything starts."""
    from .custom import BodyLoadError, default_profile
    try:
        return default_profile(ref)
    except BodyLoadError as exc:
        raise SystemExit(f'--body: {exc}') from None


def leave(c):
    """The config an arm profile starts from after a humanoid one: no body; a simulating robot keeps simulating, in an arm scene."""
    if not c.get('body'):
        return False
    c.pop('body', None)
    if c.get('sim') in profiles.SCENES:
        c['sim'] = 'cup'
    return True
