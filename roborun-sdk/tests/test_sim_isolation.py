"""A simulation never runs in the live robot: its own runtime copy (<home>/sim), camera socket and station port; a port or socket
something serves is refused in words, a socket is removed only when a simulation made it, and the live runtime is only read.

Everything here is stubbed: no MuJoCo, no systemd (systemctl is a stub on PATH that answers inactive), no real service."""
import hashlib
import json
import os
import shutil
import socket
import tempfile
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace

import pytest

from roborun_sdk import mujoco_sim, robot, sim_home

STUB = '''#!/bin/sh
echo "{name} $*" >> "{log}"
case "$*" in
  *is-system-running*) echo running ;;
  *is-active*) echo inactive ;;
esac
exit 0
'''


@pytest.fixture
def box(tmp_path, monkeypatch):
    """A Roborun home, a live runtime in the installer's layout (<top>/robot/runtime + its helper folders), a recording systemd stub."""
    stubs = tmp_path / 'stubs'; stubs.mkdir()
    log = tmp_path / 'systemd.log'; log.write_text('')
    for name in ('systemctl', 'systemd-run', 'loginctl'):
        (stubs / name).write_text(STUB.format(name=name, log=log)); (stubs / name).chmod(0o755)
    monkeypatch.setenv('PATH', f"{stubs}{os.pathsep}{os.environ.get('PATH', '')}")
    # sockets go under XDG_RUNTIME_DIR/<...>/roborun-<hex>/: a short folder of its own, as pytest's tmp_path can pass the
    # AF_UNIX path limit (108 bytes on Linux, 104 on macOS), deepest under -n
    run = Path(tempfile.mkdtemp(prefix='rr', dir='/tmp' if os.path.isdir('/tmp') else None))
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(run))
    for k in ('MANIFEST_CAMERA_SOCKET', 'DBUS_SESSION_BUS_ADDRESS', 'MANIFEST_DREAMAPI', 'MANIFEST_BACKEND_ENTRY'):
        monkeypatch.delenv(k, raising=False)
    home = tmp_path / 'home'; home.mkdir()
    monkeypatch.setenv('ROBORUN_HOME', str(home))
    monkeypatch.setattr(robot, 'HOME', home); monkeypatch.setattr(robot, 'CONFIG', home / 'robot.json')
    monkeypatch.setattr(robot, 'account_home', lambda: tmp_path / 'account' / '.config' / 'roborun')
    top = tmp_path / 'data'
    live = top / 'robot' / 'runtime'
    for rel, text in {'kin.py': '# kin\n', 'server.py': '# backend\n', 'old_module.py': '# goes away\n', 'RUNTIME.json': '{"version": "r1"}',
                      'profiles/yam/config.json': json.dumps(dict(body='yam', rig='profiles/yam/rig.json', station='http://127.0.0.1:8091')),
                      'profiles/yam/rig.json': '{}',
                      'profiles/yam_metal/config.json': json.dumps(dict(body='yam_metal', rig='profiles/yam_metal/rig.json',
                                                                        limbs=dict(right=dict(model='metal')))),
                      'profiles/yam_metal/rig.json': '{}',
                      'log/real-calibration/calibration.json': '{"real": true}',
                      '../../depth/camera-policy.json': '{}', '../../depth/log/run.txt': 'helper log'}.items():
        p = (live / rel).resolve(); p.parent.mkdir(parents=True, exist_ok=True); p.write_text(text)
    (live / 'config.json').write_text((live / 'profiles' / 'yam_metal' / 'config.json').read_text())
    robot.CONFIG.write_text(json.dumps(dict(manifest=str(live), robot='r', server='http://127.0.0.1:9', key_file=str(home / 'key'),
                                            backend='http://127.0.0.1:8120', jobs=str(home / 'robot' / 'jobs.json'))))
    (home / 'key').write_text('rr_test\n')
    calls = lambda: [line.split() for line in log.read_text().splitlines()]
    yield SimpleNamespace(home=home, live=live, top=top, calls=calls, tmp=tmp_path, run=run)
    shutil.rmtree(run, ignore_errors=True)


def tree(folder):
    """Every file under folder with its content's hash: what "untouched" is checked against."""
    folder = Path(folder)
    return {str(p.relative_to(folder)): hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else 'dir'
            for p in sorted(folder.rglob('*'))}


def serve_unix(path):
    srv = socket.socket(socket.AF_UNIX); srv.bind(str(path)); srv.listen(4)
    return srv


def station(body):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def do_GET(self):
            data = json.dumps(body).encode(); self.send_response(200); self.send_header('Content-Length', str(len(data)))
            self.end_headers(); self.wfile.write(data)
    srv = ThreadingHTTPServer(('127.0.0.1', 0), H); threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


# ------------------------------------------------------------------------------------------------ the runtime copy
def test_the_copy_is_made_without_log_and_the_live_runtime_is_only_read(box):
    before = tree(box.top)
    said = []
    root = sim_home.prepare_copy(box.live, say=said.append)
    assert root == box.home / 'sim' / 'robot' / 'runtime' and (root / 'kin.py').read_text() == '# kin\n'
    assert not (root / 'log').exists() and not (box.home / 'sim' / 'depth' / 'log').exists()
    assert (box.home / 'sim' / 'depth' / 'camera-policy.json').exists()            # the helpers beside the runtime
    assert json.loads((root / 'config.json').read_text())['body'] == 'yam_metal'        # starts as the live one's
    assert said and 'never writes into the runtime it copies' in said[0]
    assert robot._ours(root)                                                            # the installer's layout
    assert tree(box.top) == before
    said.clear()
    assert sim_home.prepare_copy(box.live, say=said.append) == root and not said              # unchanged: nothing copied again


def test_a_changed_live_runtime_refreshes_the_copy_and_the_copy_keeps_its_own_config_and_log(box):
    root = sim_home.prepare_copy(box.live, say=lambda _: None)
    (root / 'config.json').write_text('{"mine": 1}')
    (root / 'log' / 'video-motion' / 'sim-calibration').mkdir(parents=True)
    (root / 'log' / 'video-motion' / 'sim-calibration' / 'calibration.json').write_text('{"sim": true}')
    (box.live / 'kin.py').write_text('# kin v2\n'); (box.live / 'old_module.py').unlink()
    os.utime(box.live / 'kin.py', ns=(1, 2))
    assert not sim_home.current(box.live)
    assert sim_home.prepare_copy(box.live, refresh=False, say=lambda _: None) == root and (root / 'kin.py').read_text() == '# kin\n'
    sim_home.prepare_copy(box.live, say=lambda _: None)
    assert (root / 'kin.py').read_text() == '# kin v2\n' and not (root / 'old_module.py').exists()
    assert json.loads((root / 'config.json').read_text()) == {'mine': 1}
    assert (root / 'log' / 'video-motion' / 'sim-calibration' / 'calibration.json').exists()
    assert not (box.live / 'log' / 'video-motion' / 'sim-calibration').exists()


def test_a_folder_without_a_runtime_or_overlapping_the_copy_is_refused(box, tmp_path):
    with pytest.raises(SystemExit, match='no Roborun runtime at'):
        sim_home.prepare_copy(tmp_path / 'nothing')
    inside = box.home / 'sim' / 'robot' / 'runtime'; inside.mkdir(parents=True); (inside / 'kin.py').write_text('')
    with pytest.raises(SystemExit, match='overlap'):
        sim_home.prepare_copy(inside)


# ------------------------------------------------------------------------------------------------ the camera socket
def test_a_served_socket_is_refused_and_left_served(box):
    path = box.run / 'cam.sock'
    srv = serve_unix(path)
    try:
        with pytest.raises(SystemExit, match='already served by a live camera service.*does not take it over'):
            sim_home.claim_socket(path)
        with socket.socket(socket.AF_UNIX) as c:
            c.connect(str(path))                                                         # still the live one's
    finally:
        srv.close()


def test_a_left_over_socket_is_removed_only_when_a_simulation_made_it(box):
    path = box.run / 'cam.sock'
    serve_unix(path).close()                                                             # left over, nobody's marker
    with pytest.raises(SystemExit, match='no Roborun simulation made it.*not removed'):
        sim_home.claim_socket(path)
    assert path.exists()
    path.unlink()
    srv = serve_unix(path); release = sim_home.own_socket(path); srv.close()            # a simulation's, then it died
    sim_home.claim_socket(path)
    assert not path.exists() and not Path(str(path) + sim_home.OWNER).exists()
    path.write_text('not a socket')
    with pytest.raises(SystemExit, match='is not a socket'):
        sim_home.claim_socket(path)
    assert path.read_text() == 'not a socket'
    release()                                                                            # not the socket it made: left alone
    assert path.read_text() == 'not a socket'


def test_release_removes_only_the_socket_this_simulation_made(box):
    path = box.run / 'cam.sock'
    srv = serve_unix(path); release = sim_home.own_socket(path); srv.close()
    path.unlink(); other = serve_unix(path)                                              # someone else's now, same path
    try:
        release()
        assert path.exists()
    finally:
        other.close()


# ------------------------------------------------------------------------------------------------ roborun sim
def _sim_stubs(monkeypatch, seen):
    """cmd_sim without MuJoCo: a world with one arm and no cameras, the station recorded instead of served."""
    from roborun_sdk import arm_profiles, embodiment
    from roborun_sdk.arms import settings

    class World:
        def __init__(self, root, rig, scene, homes=None, cameras=None):
            seen['world'] = (Path(root), Path(rig), scene)
            self.arms, self.spec, self.cameras, self.objects = ['left'], dict(arms=dict(left={})), {}, []
        def renderer(self): return None
    monkeypatch.setattr(mujoco_sim, 'World', World)
    monkeypatch.setattr(mujoco_sim, 'MuJoCoArm', lambda world, side, rules: side)
    monkeypatch.setattr(mujoco_sim, 'write_calibration', lambda world, root: seen.setdefault('calibration', Path(root)) and None)
    monkeypatch.setattr(arm_profiles, 'ensure', lambda root, pid: seen.setdefault('ensure', (Path(root), pid)))
    monkeypatch.setattr(settings, 'load', lambda warn=None: {})

    def run_station(left, right, port, sessions_dir, predict):
        seen.update(port=port, sessions=sessions_dir, predict=predict)
        seen['socket_served'] = sim_home._answers(seen['sock'])
        raise KeyboardInterrupt                                                           # stands for Ctrl-C once it serves
    monkeypatch.setattr(embodiment, 'run_station', run_station)


def args(**kw):
    return SimpleNamespace(**dict(dict(profile=None, scene='cup', port=None, camera_socket=None, sessions=None, predict=None,
                                       manifest=None, as_robot=False), **kw))


def test_bare_roborun_sim_runs_from_its_copy_on_its_own_port_and_socket(box, monkeypatch):
    live_sock = Path(robot.camera_socket())                                              # the live camera service, serving
    live_srv = serve_unix(live_sock)
    seen = dict(sock=sim_home.camera_socket())
    _sim_stubs(monkeypatch, seen)
    before = tree(box.top)
    try:
        assert mujoco_sim.cmd_sim(args()) == 0
        with socket.socket(socket.AF_UNIX) as c:
            c.connect(str(live_sock))                                                    # untouched, still served
    finally:
        live_srv.close()
    root = box.home / 'sim' / 'robot' / 'runtime'
    assert seen['port'] == sim_home.SIM_PORT == 8191 and seen['predict'] is None
    assert seen['world'][0] == root and seen['calibration'] == root and seen['ensure'] == (root, 'yam_metal')
    assert Path(seen['sessions']).is_relative_to(root)
    assert seen['socket_served'] and seen['sock'] != str(live_sock) and seen['sock'].endswith('sim-camera.sock')
    assert not Path(seen['sock']).exists()                                               # removed again on the way out
    assert tree(box.top) == before


def test_a_served_port_is_refused_before_anything_is_copied(box, monkeypatch):
    seen = dict(sock=sim_home.camera_socket()); _sim_stubs(monkeypatch, seen)
    srv = station(dict(state='idle', arm_status={}))                                     # a live station
    try:
        with pytest.raises(SystemExit, match=r'already served by a live arms station.*Nothing was started'):
            mujoco_sim.cmd_sim(args(port=srv.server_address[1]))
    finally:
        srv.shutdown()
    assert not (box.home / 'sim').exists() and 'port' not in seen


def test_a_served_camera_socket_is_refused(box, monkeypatch):
    seen = dict(sock=sim_home.camera_socket()); _sim_stubs(monkeypatch, seen)
    srv = serve_unix(seen['sock'])
    try:
        with pytest.raises(SystemExit, match='already served'):
            mujoco_sim.cmd_sim(args())
    finally:
        srv.close()
    assert not (box.home / 'sim').exists() and 'port' not in seen


def test_runtime_flag_at_the_live_runtime_runs_from_the_copy_and_never_writes_there(box, monkeypatch):
    seen = dict(sock=sim_home.camera_socket()); _sim_stubs(monkeypatch, seen)
    before = tree(box.top)
    root = box.home / 'sim' / 'robot' / 'runtime'
    for state in ('active', 'inactive'):                                                 # the live robot running, then stopped
        monkeypatch.setattr(robot, '_state', lambda unit, state=state: state if unit == 'roborun-station' else 'inactive')
        seen.pop('calibration', None)
        assert mujoco_sim.cmd_sim(args(manifest=str(box.live))) == 0
        assert seen['calibration'] == root and seen['world'][0] == root and Path(seen['sessions']).is_relative_to(root)
    assert tree(box.top) == before                                                       # no sim calibration in the live log/


def test_runtime_flag_at_another_install_s_runtime_runs_from_its_own_copy(box, monkeypatch):
    """A second install (or a sandboxed HOME) given the default install's live runtime while that robot runs: the runtime is
    copied into this home's sim/runtimes/<hex>, never written; its live station keeps answering."""
    seen = dict(sock=sim_home.camera_socket()); _sim_stubs(monkeypatch, seen)
    other = box.tmp / 'default-install' / 'robot' / 'runtime'
    for rel, text in {'kin.py': '# kin\n', 'profiles/yam/config.json': json.dumps(dict(body='yam', rig='profiles/yam/rig.json')),
                      'profiles/yam/rig.json': '{}', 'log/real/calibration.json': '{"real": true}'}.items():
        (other / rel).parent.mkdir(parents=True, exist_ok=True); (other / rel).write_text(text)
    (other / 'config.json').write_text((other / 'profiles' / 'yam' / 'config.json').read_text())
    live = station(dict(state='idle', arm_status={}))                                    # the other install's live station
    before = tree(other.parents[1])
    try:
        assert mujoco_sim.cmd_sim(args(manifest=str(other))) == 0
    finally:
        live.shutdown()
    root = seen['world'][0]
    assert root.is_relative_to(box.home / 'sim' / 'runtimes') and root.name == 'runtime' and (root / 'kin.py').exists()
    assert seen['calibration'] == root and Path(seen['sessions']).is_relative_to(root) and seen['ensure'][0] == root
    assert tree(other.parents[1]) == before
    assert not (box.home / 'sim' / 'robot').exists()                                     # the live runtime's copy is separate
    seen.clear(); seen['sock'] = sim_home.camera_socket()
    assert mujoco_sim.cmd_sim(args(manifest=str(root))) == 0 and seen['world'][0] == root     # a copy is used as it is


def test_the_copy_takes_the_live_config_again_only_when_it_changes(box):
    root = sim_home.prepare_copy(box.live, say=lambda _: None)
    (root / 'config.json').write_text('{"mine": 1}')
    assert sim_home.current(box.live) and sim_home.prepare_copy(box.live, say=lambda _: None) == root
    assert json.loads((root / 'config.json').read_text()) == {'mine': 1}                # the copy's own, while the live one is the same
    (box.live / 'config.json').write_text((box.live / 'profiles' / 'yam' / 'config.json').read_text())
    os.utime(box.live / 'config.json', ns=(5, 6))
    assert not sim_home.current(box.live)
    sim_home.prepare_copy(box.live, say=lambda _: None)
    assert json.loads((root / 'config.json').read_text())['body'] == 'yam'               # a runtime update or another profile


@pytest.mark.parametrize('robot_s', ['socket', 'port', 'default install socket'])
def test_bare_sim_never_takes_the_robot_s_port_or_socket_even_while_free(box, monkeypatch, robot_s):
    """The live station is up and its camera service is between restarts (camera.sock missing), or its backend runs with the
    station down (8091 free): bare roborun sim still does not take them."""
    seen = dict(sock=sim_home.camera_socket()); _sim_stubs(monkeypatch, seen)
    monkeypatch.setattr(robot, '_state', lambda unit: 'active' if unit == 'roborun-robot' else 'inactive')
    default_sock = box.run / 'manifest' / 'camera.sock'                         # the default install's, from this second one
    kw = dict(socket=dict(camera_socket=robot.camera_socket()), port=dict(port=robot.STATION_PORT),
              **{'default install socket': dict(camera_socket=str(default_sock))})[robot_s]
    assert not os.path.exists(kw.get('camera_socket', '/nonexistent'))
    with pytest.raises(SystemExit, match=r"is the robot's station port|is a robot's camera socket"):
        mujoco_sim.cmd_sim(args(**kw))
    assert 'port' not in seen and not (box.home / 'sim').exists() and not default_sock.exists()


def test_up_sim_s_simulation_takes_the_robot_s_port_and_socket_only_while_the_live_robot_is_down(box, monkeypatch):
    seen = dict(sock=robot.camera_socket()); _sim_stubs(monkeypatch, seen)
    root = sim_home.prepare_copy(box.live, say=lambda _: None)
    kw = dict(manifest=str(root), port=robot.STATION_PORT, camera_socket=robot.camera_socket(), as_robot=True)
    monkeypatch.setattr(robot, '_state', lambda unit: 'active' if unit == 'roborun-station' else 'inactive')
    with pytest.raises(SystemExit, match=r'the live robot is running here.*the arms station.*Nothing was started'):
        mujoco_sim.cmd_sim(args(**kw))
    assert 'port' not in seen
    monkeypatch.setattr(robot, '_state', lambda unit: 'active' if unit == 'roborun-sim' else 'inactive')   # itself, the unit
    c = json.loads(robot.CONFIG.read_text()); c['sim'] = 'cup'; robot.CONFIG.write_text(json.dumps(c))
    assert mujoco_sim.cmd_sim(args(**kw)) == 0
    assert seen['port'] == robot.STATION_PORT and seen['socket_served'] and seen['world'][0] == root


def test_a_second_simulation_beside_roborun_up_sim_is_refused(box, monkeypatch):
    seen = dict(sock=sim_home.camera_socket()); _sim_stubs(monkeypatch, seen)
    c = json.loads(robot.CONFIG.read_text()); c['sim'] = 'cup'; robot.CONFIG.write_text(json.dumps(c))
    monkeypatch.setattr(robot, '_state', lambda unit: 'active' if unit == 'roborun-sim' else 'inactive')
    with pytest.raises(SystemExit, match=r'runs as a simulation now.*roborun down first'):
        mujoco_sim.cmd_sim(args())


def test_the_simulated_station_says_sim_and_a_live_one_does_not(box):
    sim, live, other = station(dict(state='idle', sim=True)), station(dict(state='idle')), station(dict(hello=1))
    try:
        assert sim_home.port_user(sim.server_address[1]) == "a simulation's station"
        assert sim_home.port_user(live.server_address[1]) == 'a live arms station'
        assert sim_home.port_user(other.server_address[1]) == 'another program'
    finally:
        for s in (sim, live, other): s.shutdown()


# ------------------------------------------------------------------------------------------------ roborun up --sim
def _up_stubs(monkeypatch, units):
    from roborun_sdk.arms import settings
    monkeypatch.setattr(robot, 'resolve_profile', lambda a: None)
    monkeypatch.setattr(settings, 'settle_token', lambda *a: None)
    monkeypatch.setattr(robot, 'refresh_keys', lambda c, say=print: None)
    monkeypatch.setattr(robot, 'ensure_runtime', lambda *a, **k: False)
    monkeypatch.setattr(robot, '_busy', lambda url: False)
    monkeypatch.setattr(robot, '_port_taken', lambda port: False)
    monkeypatch.setattr(robot, 'gl_env', lambda: {})
    monkeypatch.setattr(robot, '_recording_tools', lambda a, c, persist=True: None)
    monkeypatch.setattr(robot, '_backend_up', lambda url, wait_s=0: True)
    monkeypatch.setattr(robot, 'ready', lambda c, say=print: None)
    monkeypatch.setattr(robot, '_unit', lambda name, argv, env: units.append((name, argv, env)))
    from roborun_sdk import overhead
    monkeypatch.setattr(overhead, 'check', lambda backend, **k: None)            # the real arms' overhead check

    class Up:
        def __enter__(self): return self
        def __exit__(self, *a): return False
    monkeypatch.setattr(robot.urllib.request, 'urlopen', lambda *a, **k: Up())       # the simulated station answers at once


def up_args(**kw):
    return SimpleNamespace(**dict(dict(manifest=None, sim=None, no_sim=False, profile=None, profile_id=None, rig=None, left=None,
                                       right=None, robot=None, server=None, key_file=None, backend=None, live_port=None,
                                       camera=None, body=None, body_agent=None, data_sync=False, no_data_sync=False), **kw))


def test_up_sim_runs_the_backend_and_the_simulation_from_the_copy_and_back(box, monkeypatch):
    units = []; _up_stubs(monkeypatch, units)
    before = tree(box.live)
    assert robot.cmd_up(up_args(sim='cup')) == 0
    root = box.home / 'sim' / 'robot' / 'runtime'
    run = {name: (argv, env) for name, argv, env in units}
    sim_argv = run['roborun-sim'][0]
    flag = lambda argv, f: argv[argv.index(f) + 1]
    assert flag(sim_argv, '--runtime') == str(root) and flag(sim_argv, '--port') == '8091'
    assert flag(sim_argv, '--predict') == 'http://127.0.0.1:8120/internal/video-motion/predict'
    assert '--as-robot' in sim_argv                                                      # the only way to the robot's port and socket
    assert flag(sim_argv, '--camera-socket') == robot.camera_socket()                   # the robot's: the robot is the simulation
    robot_argv, robot_env = run['roborun-robot']
    assert robot_argv[1] == str(root / 'server.py') and flag(robot_argv, '--config') == str(root / 'config.json')
    assert robot_env['MANIFEST_DREAMAPI'] == str(root) and robot_env['MANIFEST_SIMULATION'] == 'cup'
    assert json.loads((root / 'config.json').read_text())['sim_of'] == 'yam_metal'       # the copy runs the station config
    c = json.loads(robot.CONFIG.read_text())
    assert c['sim'] == 'cup' and c['data']['sources']['tasks'] == str(root / 'log' / 'tasks')
    assert tree(box.live) == before                                                      # the live config was not rewritten
    units.clear()
    assert robot.cmd_up(up_args(no_sim=True)) == 0                                       # back to the arms: the live runtime
    run = {name: (argv, env) for name, argv, env in units}
    assert 'roborun-sim' not in run and run['roborun-robot'][1]['MANIFEST_DREAMAPI'] == str(box.live)
    assert json.loads(robot.CONFIG.read_text())['data']['sources']['tasks'] == str(box.live / 'log' / 'tasks')
    assert tree(box.live) == before


def test_up_sim_is_refused_while_the_live_robot_runs_and_changes_nothing(box, monkeypatch):
    units = []; _up_stubs(monkeypatch, units)
    monkeypatch.setattr(robot, '_state', lambda unit: 'active' if unit in ('roborun-station', 'roborun-cameras') else 'inactive')
    saved = robot.CONFIG.read_text(); before = tree(box.top)
    with pytest.raises(SystemExit, match=r'live robot is running here.*the arms station.*the camera service.*Nothing was changed.*'
                                         r'roborun down .*then roborun up --sim cup'):
        robot.cmd_up(up_args(sim='cup'))
    assert robot.CONFIG.read_text() == saved and not units and tree(box.top) == before and not (box.home / 'sim').exists()


def test_up_sim_is_refused_while_a_live_camera_service_serves_the_robot_s_socket(box, monkeypatch):
    units = []; _up_stubs(monkeypatch, units)
    srv = serve_unix(robot.camera_socket())
    try:
        with pytest.raises(SystemExit, match='a live camera service on'):
            robot.cmd_up(up_args(sim='cup'))
    finally:
        srv.close()
    assert not units


def test_up_sim_forgets_only_its_own_unserved_camera_socket(box, monkeypatch):
    path = robot.camera_socket()
    serve_unix(path).close()                                                             # left by this install's camera service
    robot._forget_stale_socket(path)
    assert not os.path.exists(path)
    other = str(box.run / 'other.sock'); serve_unix(other).close()
    robot._forget_stale_socket(other)                                                    # not this install's path: left alone
    assert os.path.exists(other)
    srv = serve_unix(path)
    try:
        robot._forget_stale_socket(path)                                                 # served: left alone
        assert os.path.exists(path)
    finally:
        srv.close()


# ------------------------------------------------------------------------------------------------ body simulations
def test_a_body_simulation_is_refused_while_the_live_arms_run_and_stops_nothing(box, monkeypatch):
    from roborun_sdk.arms import settings
    from roborun_sdk.body import up as body_up
    monkeypatch.setattr(settings, 'settle_token', lambda *a: None)
    monkeypatch.setattr(robot, '_busy', lambda url: False)
    stopped, units = [], []
    monkeypatch.setattr(robot, '_stop', lambda name: stopped.append(name))
    monkeypatch.setattr(robot, '_unit', lambda name, argv, env: units.append(name))
    monkeypatch.setattr(robot, '_backend_up', lambda url, wait_s=0: True)
    monkeypatch.setattr(robot, '_port_taken', lambda port: False)
    monkeypatch.setattr(robot, 'gl_env', lambda: {})
    monkeypatch.setattr(robot, 'ready', lambda c, say=print: None)
    a = lambda: SimpleNamespace(profile='booster_k1', rig=None, left=None, right=None, sim='room', no_sim=False, robot=None,
                                key_file=None, server=None, backend='http://127.0.0.1:8120', body_agent=None,
                                body_agent_key_file=None, body=None)
    saved = robot.CONFIG.read_text()
    monkeypatch.setattr(robot, '_state', lambda unit: 'active' if unit == 'roborun-station' else 'inactive')
    with pytest.raises(SystemExit, match=r'live robot is running here.*the arms station.*Nothing was changed.*roborun down'):
        body_up.up(a())
    assert not stopped and not units and robot.CONFIG.read_text() == saved
    monkeypatch.setattr(robot, '_state', lambda unit: 'active' if unit == 'roborun-robot' else 'inactive')   # live arms backend
    with pytest.raises(SystemExit, match=r"this robot's Roborun"):
        body_up.up(a())
    assert not stopped and not units
    monkeypatch.setattr(robot, '_state', lambda unit: 'inactive')                                       # after roborun down
    assert body_up.up(a()) == 0 and 'roborun-robot' in units
