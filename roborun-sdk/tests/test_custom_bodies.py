"""Your own robot body from your own files: a profile in <ROBORUN_HOME>/bodies, a body class named with --body or robot.yaml's
body:, a clear error for a body there is not, and the robot description a move sends taken from your profile."""
import json
import os
import subprocess
import sys
import textwrap
import time
import types
from pathlib import Path

import pytest

from roborun_sdk.body import BaseAdapter, HeadAdapter, profiles
from roborun_sdk.body.custom import BodyLoadError, open_custom, resolve_ref
from roborun_sdk.body.jobs import build_request

EXAMPLES = Path(__file__).resolve().parents[1] / 'examples'

ROVER = textwrap.dedent("""\
    label: Yard rover
    maker: me
    kind: rover
    limits: {vx_m_s: [-0.2, 0.5], vyaw_rad_s: [-1.0, 1.0]}
    camera: {size_px: [640, 480], hfov_deg: 90, eye_height_m: 0.35, pitch_rad: 0.2, lens: a 90 degree USB webcam}
    footprint_m: 0.3
    """)

BODY_FILE = textwrap.dedent("""\
    import io, time
    from roborun_sdk.body import BaseAdapter, HeadAdapter

    def jpeg(w=320, h=240):
        from PIL import Image
        out = io.BytesIO(); Image.new('RGB', (w, h), (90, 120, 150)).save(out, 'JPEG'); return out.getvalue()

    class Base(BaseAdapter):
        kind = 'wheeled'
        def __init__(self):
            self.cmds = []; self.x = 0.; self.v = 0.; self.at = time.monotonic()
        def command(self, vx, vy, vyaw):
            self.state(); self.v = vx; self.cmds.append((vx, vy, vyaw))
        def stop(self):
            self.state(); self.v = 0.
        def state(self):
            now = time.monotonic(); self.x += self.v * (now - self.at); self.at = now
            return dict(x=self.x, y=0., yaw=0., vx=self.v, vy=0., vyaw=0., commanded=dict(vx=self.v, vy=0., vyaw=0.), t=time.time())

    class Cam(HeadAdapter):
        dof = 0
        def command(self, pitch, yaw): pass
        def state(self): return dict(pitch=0.2, yaw=0.)
        def frame(self): return dict(jpeg=jpeg())

    class Rover:
        profile = 'yard_rover'
        made = []
        def __init__(self, profile, port='/dev/null', speed=1.0):
            self.profile_given, self.port, self.speed = profile, port, speed
            self.base, self.head = Base(), Cam()
            Rover.made.append(self)
        def close(self): self.closed = True

    class Both(Base, Cam):
        dof = 0

    class BaseWithHead(Base):
        def __init__(self):
            super().__init__(); self.head = Cam()

    class NoHead:
        def __init__(self): self.base = Base()

    class NotABody:
        pass
    """)


@pytest.fixture
def home(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBORUN_HOME', str(tmp_path))
    monkeypatch.setattr(profiles, '_added', {})                    # profiles.add_file() stays inside the test
    (tmp_path / 'bodies').mkdir()
    return tmp_path


def write(home, name, text):
    f = home / 'bodies' / name
    f.write_text(text)
    return f


# ------------------------------------------------------------------------------------------------ profiles
def test_the_packaged_profiles_pass_the_same_check_as_yours():
    for pid in profiles.packaged_ids():
        raw = json.loads((profiles.BODIES / f'{pid}.json').read_text())
        p = profiles.validate(raw, source=f'packaged {pid}')            # a packaged model path is relative to the package's data
        assert p['robot'] == raw['robot'] and p['sim']['model'] == raw['sim']['model'] and p['guard'] == raw['guard'] and p['camera']['eye_height_m'] == raw['camera']['eye_height_m']


def test_your_own_profile_is_read_from_roborun_home_and_completed(home):
    write(home, 'yard_rover.yaml', ROVER)
    assert profiles.ids() == ['booster_k1', 'galaxea_r1', 'yard_rover'] and profiles.is_body('yard_rover')
    p = profiles.load('yard_rover')
    assert p['robot'] == dict(schema='manifest.robot-descriptor.v2', kind='wheeled', family='yard_rover', arms=0, base='wheeled',
                              head_dof=0, limits=dict(vx_m_s=[-.2, .5], vy_m_s=[0., 0.], vyaw_rad_s=[-1., 1.],
                                                      head_rate_rad_s=1.5, deadband_m_s=0.))
    assert p['camera'] == dict(name='head', size_px=[640, 480], hfov_deg=90., eye_height_m=.35, pitch_rad=.2, lens='a 90 degree USB webcam')
    assert p['motion']['deadman_s'] == .3 and p['guard']['footprint_radius_m'] == .3 and p['guard']['obstacle_z_m'] == [.06, .55]
    assert 'sim' not in p and 'agent' not in p
    row = next(r for r in profiles.rows() if r['id'] == 'yard_rover')
    assert row == dict(id='yard_rover', label='Yard rover', kind='wheeled', base='wheeled', arms=0, head_dof=0, own=True)
    assert not profiles.is_packaged('yard_rover') and profiles.problems() == []


def test_a_humanoid_profile_in_json_with_a_moving_head(home):
    write(home, 'tall_bot.json', json.dumps(dict(
        label='Tall bot', kind='humanoid', base='legged', arms=2, head_dof=2,
        limits=dict(vx_m_s=[-.2, .4], vy_m_s=[-.1, .1], vyaw_rad_s=[-.5, .5], head_pitch_rad=[-.3, .8], head_yaw_rad=[-1, 1],
                    walk_min_pitch_rad=.3),
        camera=dict(size_px=[960, 540], hfov_deg=100, eye_height_m=1.4, pitch_rad=.3))))
    r = profiles.descriptor('tall_bot')
    assert (r['kind'], r['base'], r['arms'], r['head_dof']) == ('humanoid', 'legged', 2, 2)
    assert r['limits']['head_yaw_rad'] == [-1., 1.] and r['limits']['walk_min_pitch_rad'] == .3


@pytest.mark.parametrize('change, words', [
    (dict(kind='tank'), 'kind: wheeled (or rover) or humanoid'),
    (dict(kind='humanoid'), 'base: legged or wheeled (a humanoid has to say which)'),
    (dict(base='legged'), 'base: a wheeled body has a wheeled base'),
    (dict(arms=3), 'arms: a whole number from 0 to 2'),
    (dict(head_dof=1), 'limits.head_pitch_rad: required for a head that pitches'),
    (dict(limits=dict(vx_m_s=[0.1, 0.5], vyaw_rad_s=[-1, 1])), 'limits.vx_m_s: the range has to include 0'),
    (dict(limits=dict(vx_m_s=[-0.2, 0.5])), 'limits.vyaw_rad_s: required'),
    (dict(limits=dict(vx_m_s=[-0.2, 9], vyaw_rad_s=[-1, 1])), 'limits.vx_m_s[1]: between -5 and 5'),
    (dict(camera=dict(size_px=[640, 480], hfov_deg=90)), 'camera.eye_height_m: required'),
    (dict(camera=dict(size_px=[640, 480], hfov_deg=90, eye_height_m=5)), 'camera.eye_height_m: between 0.1 and 3.0'),
    (dict(wheels=2), "unknown field 'wheels'"),
    (dict(limits=dict(vx_m_s=[-0.2, 0.5], vyaw_rad_s=[-1, 1], top_speed=3)), "limits: unknown field 'top_speed'"),
    (dict(id='other_name'), "id: 'other_name' is not the file name 'yard_rover'"),
    (dict(sim=dict(model='missing.xml')), 'sim.model: no file at'),
    (dict(guard=dict(proximity_stop_m='near')), 'guard.proximity_stop_m: a number'),
    (dict(agent=dict(protocol='ros2')), 'agent.protocol: booster or galaxea'),
])
def test_a_profile_mistake_is_named_with_its_file_and_field(home, change, words):
    import yaml
    raw = dict(yaml.safe_load(ROVER), **change)
    f = write(home, 'yard_rover.yaml', yaml.safe_dump(raw))
    with pytest.raises(profiles.ProfileError) as e:
        profiles.load('yard_rover')
    assert str(e.value).startswith(f'{f}: ') and words in str(e.value), str(e.value)
    assert profiles.problems() == [(str(f), str(e.value))]                # roborun profiles says it
    assert 'yard_rover' not in [r['id'] for r in profiles.rows()]          # and the lists leave it out


def test_bad_yaml_a_packaged_name_and_a_bad_file_name_are_problems_not_crashes(home):
    write(home, 'broken.yaml', 'label: [unclosed')
    write(home, 'booster_k1.yaml', ROVER)
    write(home, 'Bad Name.yaml', ROVER)
    found = dict(profiles.problems())
    assert 'not valid YAML' in found[str(home / 'bodies' / 'broken.yaml')]
    assert 'ships with Roborun' in found[str(home / 'bodies' / 'booster_k1.yaml')]
    assert 'the file name is the profile id' in found[str(home / 'bodies' / 'Bad Name.yaml')]
    assert profiles.load('booster_k1')['label'] == 'Booster K1'           # the packaged one is never replaced
    assert [r['id'] for r in profiles.rows()] == ['booster_k1', 'galaxea_r1']


def test_an_unknown_body_says_what_there_is(home):
    write(home, 'yard_rover.yaml', ROVER)
    with pytest.raises(profiles.UnknownBody) as e:
        profiles.load('yard_rovr')
    text = str(e.value)
    assert "no body profile 'yard_rovr'" in text and 'booster_k1, galaxea_r1' in text and 'yard_rover' in text
    assert str(home / 'bodies') in text and 'Your own robot body' in text


def test_a_profile_file_changed_on_disk_is_read_again(home):
    f = write(home, 'yard_rover.yaml', ROVER)
    assert profiles.load('yard_rover')['camera']['eye_height_m'] == .35
    time.sleep(.01)
    f.write_text(ROVER.replace('eye_height_m: 0.35', 'eye_height_m: 0.5'))
    os.utime(f, ns=(time.time_ns(), time.time_ns() + 10 ** 9))
    assert profiles.load('yard_rover')['camera']['eye_height_m'] == .5


# ------------------------------------------------------------------------------------------------ the description a move sends
def test_a_move_describes_your_body_from_your_profile_not_r1(home):
    write(home, 'yard_rover.yaml', ROVER)
    req = build_request('yard_rover', dict(jpeg=_jpeg(640, 480)), 'drive to the gate', duration_s=3)
    assert req['robot'] == profiles.descriptor('yard_rover')
    assert (req['robot']['kind'], req['robot']['arms'], req['robot']['base']) == ('wheeled', 0, 'wheeled')
    cam = req['camera']
    assert cam['eye_height_m'] == .35 and cam['lens'] == 'a 90 degree USB webcam' and cam['head_pitch_rad'] == .2
    assert cam['intrinsics'] == profiles.intrinsics(640, 480, 90)          # from the frame's size and the profile's lens
    assert json.loads(json.dumps(req)) == req                                # plain JSON, as the move sends it


def test_a_robot_side_agent_with_your_profile_reports_your_geometry(home):
    write(home, 'yard_rover.yaml', ROVER + 'agent: {protocol: galaxea, port: 9000}\n')
    import base64
    from roborun_sdk.body.adapters import AgentBody
    body = AgentBody('yard_rover', 'http://127.0.0.1:9', 'key')
    body.agent.call = lambda method, path, body=None, timeout=None: dict(rgb_b64=base64.b64encode(_jpeg(640, 480)).decode()) \
        if path.startswith('/frame') else {}
    frame = body.head.frame()
    assert frame['eye_height_m'] == .35 and frame['intrinsics']['fx'] == 320.        # not the R1's 1.55 m and 110 degree lens
    req = build_request('yard_rover', frame, 'drive on', duration_s=2)
    assert req['robot']['arms'] == 0 and req['camera']['lens'] == 'a 90 degree USB webcam'


def _jpeg(w, h):
    import io
    from PIL import Image
    out = io.BytesIO(); Image.new('RGB', (w, h), (10, 20, 30)).save(out, 'JPEG'); return out.getvalue()


# ------------------------------------------------------------------------------------------------ your body class
def test_a_body_class_from_a_file_or_a_package_in_every_shape(home, tmp_path, monkeypatch):
    write(home, 'yard_rover.yaml', ROVER)
    p = profiles.load('yard_rover')
    src = tmp_path / 'my_body.py'
    src.write_text(BODY_FILE)
    base, head, close, warns = open_custom(f'{src}:Rover', p, dict(port='/dev/ttyACM0', colour='red'))
    assert isinstance(base, BaseAdapter) and isinstance(head, HeadAdapter)
    assert warns == ["body: setting 'colour' is not taken by Rover"]
    for cls in ('Both', 'BaseWithHead'):
        b, h, c, _ = open_custom(f'{src}:{cls}', p)
        assert isinstance(b, BaseAdapter) and isinstance(h, HeadAdapter)
    pkg = tmp_path / 'site' / 'acme_bodies'
    pkg.mkdir(parents=True)
    (pkg / '__init__.py').write_text('')
    (pkg / 'rover.py').write_text(BODY_FILE)
    monkeypatch.syspath_prepend(str(tmp_path / 'site'))
    b, h, c, _ = open_custom('acme_bodies.rover:Rover', p, dict(speed=2.0))
    made = sys.modules['acme_bodies.rover'].Rover.made[-1]
    assert made.profile_given['id'] == 'yard_rover' and made.speed == 2.0
    c()
    assert made.closed
    assert resolve_ref('my_body.py:Rover', tmp_path) == f'{src}:Rover' and resolve_ref('acme_bodies.rover:Rover') == 'acme_bodies.rover:Rover'


@pytest.mark.parametrize('ref, words', [
    ('{dir}/missing.py:Rover', 'no body file at'),
    ('{dir}/my_body.py:Rovr', 'has no class Rovr'),
    ('{dir}/my_body.py:NoHead', 'has no head camera'),
    ('{dir}/my_body.py:NotABody', 'is not a body'),
    ('no_such_package_xyz.mod:Rover', 'could not be imported'),
])
def test_a_body_class_that_cannot_run_says_why(home, tmp_path, ref, words):
    write(home, 'yard_rover.yaml', ROVER)
    (tmp_path / 'my_body.py').write_text(BODY_FILE)
    with pytest.raises(BodyLoadError, match=words):
        open_custom(ref.format(dir=tmp_path), profiles.load('yard_rover'))


def test_resolve_ref_refuses_what_is_not_a_class_name():
    with pytest.raises(BodyLoadError, match='file.py:ClassName'):
        resolve_ref('my_body.py')


def test_the_service_opens_your_body_class(home, tmp_path):
    from roborun_sdk.body.server import open_body
    write(home, 'yard_rover.yaml', ROVER)
    (tmp_path / 'my_body.py').write_text(BODY_FILE)
    base, head, close, simulated = open_body('yard_rover', adapter=dict(ref=f'{tmp_path}/my_body.py:Rover', options={}))
    assert simulated is None and type(base).__name__ == 'Base' and head.frame()['jpeg'][:2] == b'\xff\xd8'
    with pytest.raises(SystemExit, match='has no simulation model'):
        open_body('yard_rover', scene='room')
    with pytest.raises(SystemExit, match='--body my_body.py:MyBody'):
        open_body('yard_rover')
    with pytest.raises(SystemExit, match='yard_rover: no body file'):
        open_body('yard_rover', adapter=dict(ref=f'{tmp_path}/nope.py:Rover'))


# ------------------------------------------------------------------------------------------------ roborun up
def _up_args(**kw):
    base = dict(profile=None, rig=None, left=None, right=None, sim=None, no_sim=False, robot=None, key_file=None, server=None,
                backend='http://127.0.0.1:8120', body_agent=None, body_agent_key_file=None, body=None)
    return types.SimpleNamespace(**dict(base, **kw))


@pytest.fixture
def up_env(home, monkeypatch):
    from roborun_sdk import robot
    from roborun_sdk.arms import settings
    monkeypatch.setattr(robot, 'HOME', home); monkeypatch.setattr(robot, 'CONFIG', home / 'robot.json')
    monkeypatch.setattr(settings, 'settle_token', lambda *a: None)
    started = []
    monkeypatch.setattr(robot, '_stop', lambda name: started.append(('stop', name)))
    monkeypatch.setattr(robot, '_unit', lambda name, argv, env: started.append((name, argv, env)))
    monkeypatch.setattr(robot, '_backend_up', lambda url, wait_s=0: True)
    monkeypatch.setattr(robot, '_port_taken', lambda port: False)
    monkeypatch.setattr(robot, '_busy', lambda url: False)
    monkeypatch.setattr(robot, 'ready', lambda c: None)
    monkeypatch.setattr(robot, 'gl_env', lambda: dict(MUJOCO_GL='egl'))
    (home / 'token').write_text('rr_test_key\n')
    write(home, 'yard_rover.yaml', ROVER)
    (home / 'my_body.py').write_text(BODY_FILE)
    return robot, home, started


def test_roborun_up_runs_your_body_class_and_remembers_it(up_env):
    from roborun_sdk.body import up as body_up
    robot, home, started = up_env
    a = _up_args(profile='yard_rover', body=str(home / 'my_body.py') + ':Rover', robot='r', key_file=str(home / 'token'))
    assert body_up.wanted(a, {}) and body_up.up(a) == 0
    c = json.loads(robot.CONFIG.read_text())
    ref = dict(ref=f'{home / "my_body.py"}:Rover', options={})
    assert c['body'] == dict(profile='yard_rover', sim=None, agent=None, adapter=ref) and c['body_adapters'] == dict(yard_rover=ref)
    unit = next(s for s in started if s[0] == 'roborun-robot')
    assert unit[1][-3:] == ['roborun_sdk.cli', 'body', 'serve'] and 'MUJOCO_GL' not in unit[2]
    started.clear()
    assert body_up.wanted(_up_args(), c) and body_up.up(_up_args()) == 0          # a bare roborun up: the same again
    assert json.loads(robot.CONFIG.read_text())['body']['adapter'] == ref
    assert body_up.up(_up_args(profile='booster_k1', sim='room')) == 0           # switching away and back keeps it
    assert body_up.up(_up_args(profile='yard_rover')) == 0
    assert json.loads(robot.CONFIG.read_text())['body']['adapter'] == ref


def test_the_class_names_its_profile_when_roborun_up_does_not(up_env):
    from roborun_sdk.body import up as body_up
    robot, home, _ = up_env
    assert body_up.up(_up_args(body=f'{home}/my_body.py:Rover', robot='r', key_file=str(home / 'token'))) == 0
    assert json.loads(robot.CONFIG.read_text())['profile'] == 'yard_rover'
    assert body_up.up(_up_args(body=f'{home}/my_body.py:Both')) == 0             # no profile named: the one this robot runs
    robot.CONFIG.unlink()
    with pytest.raises(SystemExit, match="name the body's profile"):
        body_up.up(_up_args(body=f'{home}/my_body.py:Both', robot='r', key_file=str(home / 'token')))


def test_roborun_up_refuses_an_unknown_body_with_the_list(up_env):
    from roborun_sdk.body import up as body_up
    robot, home, started = up_env
    for a in (_up_args(profile='yard_rovr', body=f'{home}/my_body.py:Rover'), _up_args(profile='yard_rovr', sim='room'),
              _up_args(profile='yard_rovr')):
        with pytest.raises(SystemExit) as e:
            body_up.wanted(a, {})
        assert "no body profile 'yard_rovr'" in str(e.value) and 'booster_k1, galaxea_r1' in str(e.value) and 'yard_rover' in str(e.value)
    assert not body_up.wanted(_up_args(profile='yam'), {}) and not body_up.wanted(_up_args(profile='so101+piper'), {})
    assert not body_up.wanted(_up_args(profile='yam_dual', sim='cup'), {})
    with pytest.raises(SystemExit, match='--body: .*no body file'):
        body_up.up(_up_args(profile='yard_rover', body=f'{home}/nope.py:Rover', robot='r', key_file=str(home / 'token')))
    with pytest.raises(SystemExit, match='has no simulation model'):
        body_up.up(_up_args(profile='yard_rover', sim='room', robot='r', key_file=str(home / 'token')))
    with pytest.raises(SystemExit, match='run your own body class'):
        body_up.up(_up_args(profile='yard_rover', robot='r', key_file=str(home / 'token')))
    with pytest.raises(SystemExit, match='names no agent protocol'):
        body_up.up(_up_args(profile='yard_rover', body_agent='http://rover.local:9000', body_agent_key_file=str(home / 'token'),
                            robot='r', key_file=str(home / 'token')))
    assert not any(s[0] == 'roborun-robot' for s in started)                    # nothing started for any of these


def test_an_invalid_profile_stops_roborun_up_before_anything_starts(up_env):
    from roborun_sdk.body import up as body_up
    robot, home, started = up_env
    write(home, 'yard_rover.yaml', ROVER.replace('eye_height_m: 0.35', 'eye_height_m: 7'))
    with pytest.raises(SystemExit, match='camera.eye_height_m: between 0.1 and 3.0'):
        body_up.up(_up_args(profile='yard_rover', body=f'{home}/my_body.py:Rover', robot='r', key_file=str(home / 'token')))
    assert started == [] and not robot.CONFIG.exists()


def test_robot_yaml_body_names_the_class_and_its_options(up_env):
    from roborun_sdk.arms import settings
    robot, home, _ = up_env
    (home / 'robot.yaml').write_text('body:\n  adapter: my_body.py:Rover\n  profile: yard_rover\n  port: /dev/ttyACM0\n')
    warned = []
    s = settings.load(warn=warned.append)
    assert warned == []                                                            # body is a known setting
    a = _up_args()
    settings._apply_file(a, s)
    assert a.profile is None and a.body is None                                   # the profile is decided by from_file()
    assert a.body_file['adapter'] == 'my_body.py:Rover' and a.body_file['base_dir'] == str(home)
    from roborun_sdk.body import up as body_up
    a.robot, a.key_file = 'r', str(home / 'token')
    assert body_up.wanted(a, {}) and a.profile == 'yard_rover' and body_up.up(a) == 0
    adapter = json.loads(robot.CONFIG.read_text())['body']['adapter']
    assert adapter == dict(ref=f'{home / "my_body.py"}:Rover', options=dict(port='/dev/ttyACM0'))
    (home / 'robot.yaml').write_text('body: my_body.py:Rover\n')
    a = _up_args(); settings._apply_file(a, settings.load())
    assert a.body_file == dict(adapter='my_body.py:Rover', base_dir=str(home)) and a.profile is None and a.body is None
    assert body_up.wanted(a, {}) and a.profile == 'yard_rover'                   # the class names its profile


def _yaml_up(home, text, **kw):
    """`roborun up` with robot.yaml holding `text`: the args after robot.yaml is read, as cmd_up has them."""
    from roborun_sdk.arms import settings
    (home / 'robot.yaml').write_text(text)
    a = _up_args(**kw)
    settings._apply_file(a, settings.load())
    return a


@pytest.mark.parametrize('text', ['body:\n  adapter: my_body.py:Rover\n  profile: yard_rover\n', 'body: my_body.py:Rover\n'])
def test_robot_yaml_body_is_only_for_its_own_profile(up_env, text):
    """robot.yaml's body class never runs as another profile, is never remembered under one, and blocks no switch to arms."""
    from roborun_sdk.body import up as body_up
    robot, home, started = up_env
    key = dict(robot='r', key_file=str(home / 'token'))
    # another body, real: not the rover class (it is not connected, so it says how to connect it) and nothing is saved
    a = _yaml_up(home, text, profile='booster_k1', **key)
    assert body_up.wanted(a, {}) and a.body is None
    with pytest.raises(SystemExit, match='booster_k1 is not connected here'):
        body_up.up(a)
    assert not robot.CONFIG.exists()
    # another body, simulated: runs, and the rover class is not remembered for it
    a = _yaml_up(home, text, profile='galaxea_r1', sim='room', **key)
    assert body_up.wanted(a, {}) and body_up.up(a) == 0
    c = json.loads(robot.CONFIG.read_text())
    assert c['body'] == dict(profile='galaxea_r1', sim='room', agent=None) and c['body_adapters'] == {}
    # the page's switch to another body takes the same path
    a = _yaml_up(home, text, profile='booster_k1', sim='room')
    assert body_up.wanted(a, c) and body_up.up(a) == 0
    c = json.loads(robot.CONFIG.read_text())
    assert 'adapter' not in c['body'] and c['body_adapters'] == {}
    # arms: not a body, the class is not even imported
    (home / 'my_body.py').write_text('raise RuntimeError("imported")\n')
    assert not body_up.wanted(_yaml_up(home, text, profile='yam_dual', sim='cup'), c)
    assert not body_up.wanted(_yaml_up(home, text, profile='yam'), c)
    (home / 'my_body.py').write_text(BODY_FILE)
    # a robot-side agent for another body: the agent, not "--body and --body-agent: give one"
    a = _yaml_up(home, text, profile='galaxea_r1', body_agent='http://r1.local:8302', body_agent_key_file=str(home / 'token'))
    assert body_up.wanted(a, c) and body_up.up(a) == 0
    c = json.loads(robot.CONFIG.read_text())
    assert c['body']['agent']['url'] == 'http://r1.local:8302' and c['body_adapters'] == {}
    # its own profile (named, or by a bare roborun up): the class, remembered for it alone
    for a in (_yaml_up(home, text, profile='yard_rover'), _yaml_up(home, text)):
        assert body_up.wanted(a, c) and body_up.up(a) == 0
        c = json.loads(robot.CONFIG.read_text())
        assert c['body']['profile'] == 'yard_rover' and c['body']['adapter']['ref'] == f'{home / "my_body.py"}:Rover'
        assert set(c['body_adapters']) == {'yard_rover'}
    # robot.yaml's body: removed: galaxea_r1 real still runs its agent, never the rover class
    a = _yaml_up(home, 'robot: r\n', profile='galaxea_r1')
    assert body_up.wanted(a, c) and body_up.up(a) == 0
    assert json.loads(robot.CONFIG.read_text())['body'] == dict(profile='galaxea_r1', sim=None, agent=c['body_agents']['galaxea_r1'])


def test_robot_yaml_body_with_no_profile_anywhere_says_to_name_one(up_env):
    from roborun_sdk.body import up as body_up
    robot, home, _ = up_env
    with pytest.raises(SystemExit, match='robot.yaml body: name the profile it is for'):
        body_up.wanted(_yaml_up(home, 'body: my_body.py:Both\n'), {})
    assert not body_up.wanted(_yaml_up(home, 'body: my_body.py:Both\n', profile='yam'), {})      # arms: not asked
    a = _yaml_up(home, 'profile: yard_rover\nbody: my_body.py:Both\n', robot='r', key_file=str(home / 'token'))
    assert a.profile == 'yard_rover' and body_up.wanted(a, {}) and body_up.up(a) == 0               # robot.yaml's profile: names it
    assert json.loads(robot.CONFIG.read_text())['body']['adapter']['ref'].endswith('my_body.py:Both')


def test_robot_yaml_body_section_profile_can_override_the_class_attribute(up_env):
    from roborun_sdk.body import up as body_up
    robot, home, _ = up_env
    write(home, 'yard_rover2.yaml', ROVER)
    text = 'body:\n  adapter: my_body.py:Rover\n  profile: yard_rover2\n'
    a = _yaml_up(home, text, robot='r', key_file=str(home / 'token'))
    assert body_up.wanted(a, {}) and body_up.up(a) == 0
    c = json.loads(robot.CONFIG.read_text())
    assert c['profile'] == 'yard_rover2' and set(c['body_adapters']) == {'yard_rover2'}
    a = _yaml_up(home, text, profile='yard_rover')                                  # the class's own: not robot.yaml's section
    assert body_up.wanted(a, c) and a.body is None


def test_a_body_class_for_another_profile_is_refused(up_env):
    from roborun_sdk.body import up as body_up
    robot, home, started = up_env
    a = _up_args(profile='booster_k1', body=f'{home}/my_body.py:Rover', robot='r', key_file=str(home / 'token'))
    assert body_up.wanted(a, {})
    with pytest.raises(SystemExit, match="body class of profile 'yard_rover' .*not 'booster_k1'"):
        body_up.up(a)
    assert started == [] and not robot.CONFIG.exists()
    with pytest.raises(SystemExit, match="no body profile 'yam_dual'"):           # a body class with arms named: said
        body_up.wanted(_up_args(profile='yam_dual', body=f'{home}/my_body.py:Rover'), {})


SECTION = 'body:\n  adapter: my_body.py:Rover\n  profile: yard_rover\n'
STRING = 'body: my_body.py:Rover\n'
TOP = 'profile: yard_rover\nbody: my_body.py:Both\n'


@pytest.mark.parametrize('text', [SECTION, STRING, TOP])
@pytest.mark.parametrize('flags', [dict(rig='rig.json'), dict(left='so101@serial:///dev/ttyACM0'),
                                   dict(right='so101@serial:///dev/ttyACM0'), dict(sim='cup'), dict(sim='cup', left='yam'),
                                   dict(rig='rig.json', sim='cup')])
@pytest.mark.parametrize('ran_body', [False, True])
def test_arms_named_on_the_command_line_leave_robot_yaml_body_out(up_env, text, flags, ran_body):
    """--rig, --left/--right and --sim cup run arms while robot.yaml has body: (its section's profile, the class's, or robot.yaml's
    profile: naming the body), whether or not the robot ran the body before: the class is not imported, not run, not remembered."""
    from roborun_sdk.arms import settings
    from roborun_sdk.body import up as body_up
    robot, home, started = up_env
    config = {}
    if ran_body:
        assert body_up.up(_up_args(profile='yard_rover', body=f'{home}/my_body.py:Rover', robot='r', key_file=str(home / 'token'))) == 0
        config = json.loads(robot.CONFIG.read_text())
        started.clear()
    (home / 'my_body.py').write_text('raise RuntimeError("imported")\n')         # importing it would fail the test
    a = _yaml_up(home, text, **flags)
    assert a.profile is None and a.body is None
    assert not body_up.wanted(a, config)
    assert a.body is None and not getattr(a, 'body_from_file', False) and a.profile is None
    assert started == []
    if ran_body:
        assert json.loads(robot.CONFIG.read_text()) == config                     # nothing remembered or changed for the body


@pytest.mark.parametrize('text', [SECTION, STRING, TOP])
def test_with_no_arms_named_robot_yaml_body_still_runs(up_env, text):
    """The same robot.yaml with no arms named (a bare up, its own profile, --sim room): the body class or the body simulation."""
    from roborun_sdk.body import up as body_up
    robot, home, _ = up_env
    key = dict(robot='r', key_file=str(home / 'token'))
    a = _yaml_up(home, text, **key)
    assert body_up.wanted(a, {}) and a.profile == 'yard_rover' and a.body['adapter'].startswith('my_body.py:')
    assert body_up.up(a) == 0
    assert set(json.loads(robot.CONFIG.read_text())['body_adapters']) == {'yard_rover'}
    a = _yaml_up(home, text, profile='booster_k1', sim='room')
    assert body_up.wanted(a, {}) and a.body is None


def test_robot_yaml_sim_is_not_arms_named(up_env):
    """robot.yaml's own `sim: cup` says how this robot runs; it does not take the body away the way the --sim cup flag does."""
    from roborun_sdk.arms import settings
    from roborun_sdk.body import up as body_up
    robot, home, _ = up_env
    a = _yaml_up(home, SECTION + 'sim: cup\n')
    assert a.sim == 'cup' and a.sim_from_file and not settings.arms_named(a)
    assert body_up.wanted(a, {}) and a.profile == 'yard_rover'
    assert settings.arms_named(_up_args(sim='cup')) and not settings.arms_named(_up_args(sim='room'))
    assert settings.arms_named(_up_args(rig='r.json')) and settings.arms_named(_up_args(right='yam'))


def test_first_run_with_robot_yaml_body_and_arms_named_picks_the_arms(up_env, monkeypatch):
    """A first `roborun up --sim cup` (no robot.json yet) with robot.yaml's body: simulates the default arms, as without body:."""
    from roborun_sdk.arm_profiles import SIM_DEFAULT
    from roborun_sdk.arms import settings
    from roborun_sdk.body import up as body_up
    robot, home, _ = up_env
    monkeypatch.setattr(settings, 'path', lambda: home / 'robot.yaml')
    monkeypatch.setattr(settings, 'token_path', lambda a, s: home / 'token')
    monkeypatch.setattr(settings, '_token_server', lambda a: 'http://127.0.0.1:9')
    monkeypatch.setattr(settings, '_wants_setup', lambda a, s: False)
    (home / 'robot.yaml').write_text(SECTION)
    a = _up_args(sim='cup', robot='r')
    settings.first_run(a)
    assert a.profile == SIM_DEFAULT and not body_up.wanted(a, {})
    a = _up_args(robot='r')
    settings.first_run(a)
    assert a.profile is None and body_up.wanted(a, {}) and a.profile == 'yard_rover'


def test_a_body_profile_cannot_take_the_name_of_arms(up_env):
    from roborun_sdk.body import up as body_up
    robot, home, _ = up_env
    for name in ('so101', 'yam_dual', 'yam', 'so101+piper'):
        write(home, f'{name}.yaml', ROVER)
        assert not profiles.is_body(name) and name not in profiles.ids()
        assert not body_up.wanted(_up_args(profile=name), {})                    # it stays the arms
    said = dict(profiles.problems())
    assert "'so101' is the name of arms" in said[str(home / 'bodies' / 'so101.yaml')]
    assert 'yard_rover' in profiles.ids()
    other = home / 'elsewhere'; other.mkdir(); (other / 'so101.yaml').write_text(ROVER)
    with pytest.raises(profiles.ProfileError, match="'so101' is the name of arms"):
        profiles.add_file(other / 'so101.yaml')


def test_a_body_class_given_to_profile_points_to_body(up_env):
    from roborun_sdk.body import up as body_up
    robot, home, _ = up_env
    for name in (f'{home}/my_body.py:Rover', 'my_body.py:Rover', 'my_body.py', 'mypkg.rover:Rover'):
        with pytest.raises(SystemExit) as e:
            body_up.wanted(_up_args(profile=name), {})
        assert 'is not a profile name: your body class goes on --body' in str(e.value) and 'yard_rover' in str(e.value)


def test_a_body_class_that_does_not_start_is_not_remembered(up_env, monkeypatch):
    from roborun_sdk.body import up as body_up
    robot, home, started = up_env
    key = dict(robot='r', key_file=str(home / 'token'))
    assert body_up.up(_up_args(profile='yard_rover', body=f'{home}/my_body.py:Rover', **key)) == 0
    good = json.loads(robot.CONFIG.read_text())
    monkeypatch.setattr(robot, '_backend_up', lambda url, wait_s=0: False)       # the service never answers: the class failed
    started.clear()
    with pytest.raises(SystemExit, match='did not start with your body class .*Both, so it is not remembered for yard_rover'):
        body_up.up(_up_args(profile='yard_rover', body=f'{home}/my_body.py:Both'))
    c = json.loads(robot.CONFIG.read_text())
    assert c['body_adapters'] == good['body_adapters'] and c['body']['adapter'] == good['body']['adapter']   # the earlier one
    assert started[-1] == ('stop', 'roborun-robot')                              # not left retrying the broken class
    robot.CONFIG.unlink()
    with pytest.raises(SystemExit, match='not remembered'):
        body_up.up(_up_args(profile='yard_rover', body=f'{home}/my_body.py:Both', **key))
    c = json.loads(robot.CONFIG.read_text())
    assert c['body_adapters'] == {} and 'adapter' not in c['body']
    with pytest.raises(SystemExit, match='yard_rover is not connected here'):    # a bare roborun up does not retry it
        body_up.up(_up_args())
    with pytest.raises(SystemExit, match='the robot did not start: '):           # a simulation that fails: said as before
        body_up.up(_up_args(profile='booster_k1', sim='room'))


def test_roborun_profiles_lists_your_bodies_and_what_is_wrong(up_env, capsys):
    robot, home, _ = up_env
    write(home, 'broken.yaml', ROVER.replace('kind: rover', 'kind: boat'))
    robot.cmd_profiles(types.SimpleNamespace(manifest=str(home / "no-runtime")))
    out = capsys.readouterr().out
    assert 'yard_rover   Yard rover (wheeled, wheeled base, 0 arms, head_dof 0)' in out
    assert 'not usable: ' + str(home / 'bodies' / 'broken.yaml') + ': kind: wheeled (or rover) or humanoid' in out


# ------------------------------------------------------------------------------------------------ the example rover
def test_the_example_rover_profile_and_class(home):
    pid = profiles.add_file(EXAMPLES / 'my_rover.yaml')
    p = profiles.load(pid)
    assert pid == 'my_rover' and p['robot']['kind'] == 'wheeled' and p['sim']['model'] == str(EXAMPLES / 'my_rover.xml')
    base, head, close, warns = open_custom(f'{EXAMPLES / "rover.py"}:Rover', p)
    assert warns == []
    req = build_request(pid, head.frame(), 'drive to the door', duration_s=2)
    assert req['camera']['eye_height_m'] == .35 and req['robot']['limits']['vy_m_s'] == [0., 0.]
    close()


def test_the_example_rover_runs_offline():
    out = subprocess.run([sys.executable, str(EXAMPLES / 'rover.py')], capture_output=True, text=True, timeout=120,
                         env=dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[1] / 'src')))
    assert out.returncode == 0, out.stderr
    assert '"kind": "wheeled"' in out.stdout and '"eye_height_m": 0.35' in out.stdout and 'now at rest: True' in out.stdout


def test_the_example_rover_runs_in_the_mujoco_simulation():
    pytest.importorskip('mujoco')
    out = subprocess.run([sys.executable, str(EXAMPLES / 'rover.py'), '--mujoco'], capture_output=True, text=True, timeout=180,
                         env=dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[1] / 'src'),
                                  MUJOCO_GL=os.environ.get('MUJOCO_GL', 'egl')))
    if out.returncode and 'cannot render here' in out.stderr:
        pytest.skip('no OpenGL for the simulated camera here')
    assert out.returncode == 0, out.stderr
    assert 'T_base_camera' in out.stdout and '"eye_height_m": 0.35' in out.stdout and 'now at rest: True' in out.stdout


def test_the_example_install_puts_the_rover_where_roborun_up_reads_it(home):
    out = subprocess.run([sys.executable, str(EXAMPLES / 'rover.py'), 'install'], capture_output=True, text=True, timeout=60,
                         env=dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[1] / 'src')))
    assert out.returncode == 0, out.stderr
    assert (home / 'bodies' / 'my_rover.yaml').is_file() and (home / 'bodies' / 'my_rover.xml').is_file()
    assert profiles.load('my_rover')['sim']['model'] == str(home / 'bodies' / 'my_rover.xml')
    assert 'roborun up --profile my_rover --sim room' in out.stdout
