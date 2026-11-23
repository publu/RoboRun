"""roborun up for a humanoid profile:

    roborun up --profile booster_k1 --sim room                    the packaged simulation (scenes: room, empty)
    roborun up --profile galaxea_r1 --body-agent http://R1:8302 --body-agent-key-file ~/r1-agent.key     a real R1
    roborun up                                                    afterwards: the same again

It starts the body's local robot API on the backend address (127.0.0.1:8120) as the roborun-robot service, and the page
connector (roborun-local). No arm runtime is installed or started for a humanoid profile. A real body is used only when its agent
is given (and remembered per profile); a profile that is neither connected nor set up to simulate does not start, and is never
replaced by a simulation.
"""
from __future__ import annotations

import json
import sys
import urllib.parse
from pathlib import Path

from . import profiles


def wanted(a, config):
    """True when this `roborun up` runs a humanoid profile: one is named, or the robot runs one and no arm or profile is named."""
    if a.profile:
        return profiles.is_body(a.profile)
    if a.rig or a.left or a.right:
        return False
    return bool((config or {}).get('body'))


def up(a) -> int:
    from .. import robot
    from ..arms.settings import settle_token
    c = json.loads(robot.CONFIG.read_text()) if robot.CONFIG.exists() else {}
    pid = a.profile or (c.get('body') or {}).get('profile')
    if not c:
        if not (a.robot and a.key_file):
            raise SystemExit(f'first run: roborun up --robot <a name for this robot> --key-file <file with your access token> '
                             f'--profile {pid} --sim room   (or --body-agent http://ROBOT:PORT for a real one)')
        c = dict(server=(a.server or robot.SERVER).rstrip('/'), robot=a.robot, backend=a.backend,
                 data=dict(enabled=False, opt_out=False, sources={}, state=str(robot.HOME / 'robot' / 'sync-state.json')))
    if a.key_file:
        c['key_file'] = str(Path(a.key_file).expanduser().resolve())
    if not c.get('key_file'):
        raise SystemExit('no access token: pass --key-file <file with your access token>')
    settle_token(c['key_file'], c['server'])
    if robot._busy(c.get('backend') or 'http://127.0.0.1:8120') and pid != c.get('profile'):
        raise SystemExit('a task is running on this robot: stop it before changing the profile')

    agents = c.setdefault('body_agents', {})
    if getattr(a, 'body_agent', None):
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
    elif c.get('sim') and not getattr(a, 'no_sim', False) and not getattr(a, 'body_agent', None) and not agents.get(pid):
        # a profile with a saved real agent runs that agent; only an explicit --sim simulates it
        scene = c['sim'] if c['sim'] in profiles.SCENES else profiles.DEFAULT_SCENE     # this robot simulates: so does its next profile
    if scene is None and not agents.get(pid):
        raise SystemExit(f'{pid} is not connected here: give its agent (roborun up --profile {pid} --body-agent http://ROBOT:PORT '
                         f'--body-agent-key-file FILE), or simulate it (roborun up --profile {pid} --sim room)')
    if scene:
        c['sim'] = scene
    else:
        c.pop('sim', None)
    c.update(profile=pid, body=dict(profile=pid, sim=scene, agent=None if scene else agents[pid]))
    c.setdefault('backend', a.backend or 'http://127.0.0.1:8120')
    robot.HOME.mkdir(parents=True, exist_ok=True)
    robot.CONFIG.write_text(json.dumps(c, indent=1)); robot.CONFIG.chmod(0o600)

    for unit in ('roborun-sim', 'roborun-station', 'roborun-cameras', 'roborun-robot'):
        robot._stop(unit)                                    # an arm robot's parts, or this body's previous run
    port = urllib.parse.urlsplit(c['backend']).port or 8120
    if robot._port_taken(port):
        raise SystemExit(f'port {port} on this computer is in use by another program, and Roborun needs it for the local robot API. '
                         f'Next: stop that program (see it with: lsof -i :{port}), then run roborun up again')
    env = dict(PYTHONPATH=robot._sdk_path(), PYTHONUNBUFFERED='1', ROBORUN_HOME=str(robot.HOME))
    if scene:
        env.update(robot.gl_env())
    robot._unit('roborun-robot', [sys.executable, '-m', 'roborun_sdk.cli', 'body', 'serve'], env)
    what = f'simulation, scene {scene}' if scene else f"the robot at {agents[pid]['url']}"
    print(f"{profiles.load(pid)['label']} ({pid}): {what}", flush=True)
    if not robot._backend_up(c['backend'], wait_s=90):
        raise SystemExit('the robot did not start: ' + ('journalctl --user -u roborun-robot' if robot._systemd()
                                                        else f"see {robot.HOME / 'logs' / 'roborun-robot.log'}"))
    robot._unit('roborun-local', [sys.executable, '-m', 'roborun_sdk.cli', 'local'], dict(env))
    robot.ready(c)                                           # the same last lines as an arm profile
    print('move it: POST /v1/body/move {"prompt": "go to the table"}   stop: roborun stop (or POST /v1/stop)')
    return 0


def leave(c):
    """The config an arm profile starts from after a humanoid one: no body; a simulating robot keeps simulating, in an arm scene."""
    if not c.get('body'):
        return False
    c.pop('body', None)
    if c.get('sim') in profiles.SCENES:
        c['sim'] = 'cup'
    return True
