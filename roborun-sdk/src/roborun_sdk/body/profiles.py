"""The body profiles: the packaged ones (data/bodies/<id>.json) and your own (<ROBORUN_HOME>/bodies/<id>.yaml, .yml or .json).
Each names the body (legged or wheeled base, arms, head), the robot descriptor a body job sends (manifest.robot-descriptor.v2),
the head camera, how commands are shaped, the simulation's model and, for a packaged body, the robot-side agent a real body is
reached through.

Your own profile is checked when it is read (validate() below): every mistake is named with its field, before anything starts.
It can be short; what it leaves out is filled in (see USER_TEMPLATE and docs/BUILDING.md, "Your own robot body"):

    label: My rover
    kind: wheeled            # wheeled (also: rover) or humanoid
    base: wheeled            # wheeled or legged (default: wheeled for a wheeled kind)
    arms: 0
    head_dof: 0              # 0 a fixed camera, 1 pitch, 2 pitch and yaw
    limits: {vx_m_s: [-0.2, 0.5], vy_m_s: [0, 0], vyaw_rad_s: [-1.0, 1.0]}
    camera: {size_px: [640, 480], hfov_deg: 90, eye_height_m: 0.35, pitch_rad: 0.2, lens: a 90 degree USB webcam}
    sim: {model: my_rover.xml, base_body: base, head_camera: head, footprint_m: 0.2}     # optional: --sim room
"""
from __future__ import annotations

import copy
import json
import math
import os
import re
from pathlib import Path

DATA = Path(__file__).resolve().parents[1] / 'data'
BODIES = DATA / 'bodies'
SCENES = ('room', 'empty')            # the simulated bodies' scenes (arm scenes are cup, pickup-bench, ...)
DEFAULT_SCENE = 'room'
SUFFIXES = ('.yaml', '.yml', '.json')
ID = re.compile(r'[a-z0-9][a-z0-9_-]{0,47}')
LABEL = re.compile(r'[A-Za-z0-9][A-Za-z0-9_. -]{0,47}')            # the hosted API's robot.family
ROBOT_SCHEMA = 'manifest.robot-descriptor.v2'
KINDS = {'humanoid': 'humanoid', 'wheeled': 'wheeled', 'rover': 'wheeled'}    # rover: a wheeled body, as the hosted API names it
BASES = ('legged', 'wheeled')
TOP_KEYS = {'id', 'label', 'maker', 'description', 'kind', 'base', 'arms', 'head_dof', 'limits', 'robot', 'camera', 'motion',
            'guard', 'sim', 'agent', 'footprint_m'}
LIMIT_KEYS = {'vx_m_s', 'vy_m_s', 'vyaw_rad_s', 'head_pitch_rad', 'head_yaw_rad', 'walk_min_pitch_rad', 'head_rate_rad_s', 'deadband_m_s'}
CAMERA_KEYS = {'name', 'size_px', 'hfov_deg', 'eye_height_m', 'pitch_rad', 'lens', 'note'}
MOTION_DEFAULTS = dict(accel_m_s2=[0.5, 0.5], accel_yaw_rad_s2=1.0, stop_decel_m_s2=1.5, stop_decel_yaw_rad_s2=3.0, deadman_s=0.3,
                       command_hz=50)
SIM_KEYS = {'model', 'base_body', 'head_camera', 'head_camera_frame', 'head_joints', 'stance', 'footprint_m'}
AGENT_PROTOCOLS = ('booster', 'galaxea')


class UnknownBody(KeyError):
    """No body profile by that name; the message lists the ones there are."""

    def __str__(self):
        return str(self.args[0]) if self.args else 'unknown body'


class ProfileError(ValueError):
    """A body profile that is not valid; the message names the file and the field."""


# ---------------------------------------------------------------------------------------------------- where profiles are
def user_dir() -> Path:
    """Your own body profiles: <ROBORUN_HOME>/bodies (ROBORUN_HOME defaults to ~/.config/roborun)."""
    return Path(os.environ.get('ROBORUN_HOME') or Path.home() / '.config' / 'roborun') / 'bodies'


_added = {}                           # id -> path: profiles added in this process with add_file()


def add_file(path) -> str:
    """Use a body profile file from anywhere in this process (a test, an offline example): checked now, returns its id.
    `roborun up` reads profiles from <ROBORUN_HOME>/bodies only."""
    path = Path(path).expanduser().resolve()
    p = _read_user(path)
    if p['id'] in _packaged():
        raise ProfileError(f'{path}: {p["id"]!r} is a profile that ships with Roborun: give yours another name')
    if _arm_name(p['id']):
        raise ProfileError(f'{path}: {p["id"]!r} is the name of arms (roborun profiles): give your body profile another name')
    _added[p['id']] = path
    return p['id']


def _packaged():
    return {p.stem: p for p in BODIES.glob('*.json')}


def _user_files():
    """id -> path of the profiles in the user's folder (a stem that is not a valid id is reported by problems())."""
    out = {}
    d = user_dir()
    if d.is_dir():
        for f in sorted(d.iterdir()):
            if f.suffix in SUFFIXES and f.is_file() and not f.name.startswith('.'):
                out.setdefault(f.stem, f)
    return out


def _arm_name(name):
    """True when `name` means arms (a pre-made, catalog or runtime profile, or one made on this computer): `roborun up --profile`
    runs arms for it, so a body profile of that name could never run and is refused."""
    from .up import _arm_name as means_arms
    try:
        return means_arms(name)
    except Exception:                                         # an arm catalog that cannot be read takes no name away
        return False


def _files():
    """id -> (path, packaged?). A packaged id always means the packaged profile; an arm's name is never a body."""
    out = {k: (v, False) for k, v in _user_files().items() if ID.fullmatch(k) and not _arm_name(k)}
    out.update({k: (v, False) for k, v in _added.items()})
    out.update({k: (v, True) for k, v in _packaged().items()})
    return out


def ids():
    return sorted(_files())


def packaged_ids():
    return sorted(_packaged())


def is_body(profile_id):
    return isinstance(profile_id, str) and profile_id in _files()


def available() -> str:
    """The body profiles there are, for an error message."""
    own = [i for i in ids() if i not in _packaged()]
    return (f'packaged: {", ".join(packaged_ids())}; yours ({user_dir()}): {", ".join(own) if own else "none yet"}')


def unknown(profile_id) -> UnknownBody:
    if isinstance(profile_id, str) and (':' in profile_id or '/' in profile_id or profile_id.endswith('.py')):
        return UnknownBody(f'{profile_id!r} is not a profile name: your body class goes on --body (roborun up --profile <id> '
                           f'--body {profile_id}), and --profile names its profile ({available()})')
    return UnknownBody(f'no body profile {profile_id!r} ({available()}). Your own: put {profile_id}.yaml in {user_dir()} '
                       '(docs/BUILDING.md, "Your own robot body")')


_cache = {}


def _load(profile_id):
    files = _files()
    if profile_id not in files:
        raise unknown(profile_id)
    path, packaged = files[profile_id]
    try:
        stamp = path.stat().st_mtime_ns
    except OSError:
        raise unknown(profile_id) from None
    hit = _cache.get(profile_id)
    if hit and hit[0] == (path, stamp):
        return hit[1]
    p = json.loads(path.read_text()) if packaged else _read_user(path)
    _cache[profile_id] = ((path, stamp), p)
    return p


def load(profile_id):
    """The profile record (a copy: callers may change it). UnknownBody for a name there is no profile for, ProfileError for
    your own profile when it is not valid."""
    return copy.deepcopy(_load(profile_id))


def is_packaged(profile_id):
    return profile_id in _packaged()


def descriptor(profile_id):
    """manifest.robot-descriptor.v2 of this body, exactly as a body job sends it."""
    return load(profile_id)['robot']


def model_path(profile_id):
    sim = load(profile_id).get('sim') or {}
    if not sim.get('model'):
        raise ProfileError(f'{profile_id} has no simulation model: add sim: model: <your MJCF file> to its profile')
    return DATA / sim['model']                  # a user profile's model is an absolute path (DATA / absolute = absolute)


def rows():
    """What the profile lists show: id, label, kind, base, arms, head, and whose it is. A profile of yours that is not valid
    is left out here (problems() says why)."""
    out = []
    for pid in ids():
        try:
            p = load(pid)
        except (ProfileError, UnknownBody):
            continue
        r = p['robot']
        out.append(dict(id=pid, label=p['label'], kind=r['kind'], base=r['base'], arms=r['arms'], head_dof=r['head_dof'],
                        own=not is_packaged(pid)))
    return out


def problems():
    """[(file, message)]: your profile files that cannot be used, and why."""
    out = []
    packaged = _packaged()
    for stem, f in _user_files().items():
        if not ID.fullmatch(stem):
            out.append((str(f), f'the file name is the profile id: lowercase letters, digits, _ and -, starting with a letter or digit'))
        elif stem in packaged:
            out.append((str(f), f'{stem!r} ships with Roborun: give your profile another name'))
        elif _arm_name(stem):
            out.append((str(f), f'{stem!r} is the name of arms (roborun profiles): give your body profile another name'))
        else:
            try:
                _load(stem)
            except (ProfileError, UnknownBody) as exc:
                out.append((str(f), str(exc)))
    return out


# ---------------------------------------------------------------------------------------------------- your own profile
def _parse(path: Path):
    text = path.read_text()
    if path.suffix == '.json':
        try:
            return json.loads(text)
        except ValueError as exc:
            raise ProfileError(f'{path}: not valid JSON ({exc})') from None
    import yaml
    try:
        return yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ProfileError(f'{path}: not valid YAML ({exc})') from None


def _read_user(path: Path):
    if not path.is_file():
        raise ProfileError(f'no body profile file at {path}')
    raw = _parse(path)
    return validate(raw, source=path)


def _num(v, where, lo=None, hi=None):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ProfileError(f'{where}: a number')
    if (lo is not None and v < lo) or (hi is not None and v > hi):
        raise ProfileError(f'{where}: between {lo} and {hi} (got {v})')
    return float(v)


def _int(v, where, lo, hi):
    if isinstance(v, bool) or not isinstance(v, int) or not lo <= v <= hi:
        raise ProfileError(f'{where}: a whole number from {lo} to {hi}')
    return v


def _pair(v, where, lo, hi):
    if not isinstance(v, (list, tuple)) or len(v) != 2:
        raise ProfileError(f'{where}: [low, high]')
    a, b = _num(v[0], f'{where}[0]', lo, hi), _num(v[1], f'{where}[1]', lo, hi)
    if a > b:
        raise ProfileError(f'{where}: [low, high] with low <= high')
    return [a, b]


def _section(v, where):
    if v is None:
        return {}
    if not isinstance(v, dict):
        raise ProfileError(f'{where}: a section (key: value lines)')
    return v


def _unknown(d, known, where):
    extra = sorted(set(d) - set(known))
    if extra:
        raise ProfileError(f'{where}: unknown field{"s" if len(extra) > 1 else ""} {", ".join(map(repr, extra))} '
                           f'(known: {", ".join(sorted(known))})')


def guard_defaults(footprint_m, eye_height_m):
    """The obstacle guard's settings for a body of this size (the packaged bodies tune theirs)."""
    r = float(footprint_m)
    return dict(schema='roborun-body-guard/1', cell_m=0.05, extent_m=3.0, memory_s=20, floor_band_m=0.04,
                obstacle_z_m=[0.06, round(max(0.5, float(eye_height_m) + 0.2), 3)], self_radius_m=round(r + 0.04, 3),
                footprint_radius_m=round(r, 3), depth_range_m=[0.15, 6.0], depth_max_age_s=0.2, min_points=6, pixel_stride=2,
                unknown_blind_fraction=0.1, stall_command_m_s=0.1, stall_measured_m_s=0.02, stall_s=1.0, speed_window_s=0.2,
                proximity_stop_m=0.3, corridor_halfwidth_m=round(r + 0.1, 3), blind_v_m_s=0.15, blind_travel_max_m=0.4)


def validate(raw, source='profile'):
    """A body profile, checked and completed: raises ProfileError naming the file and the field. Accepts the short form in this
    module's docstring, or the packaged form (a `robot:` descriptor section). Returns the full profile every part of the body
    stack reads (robot descriptor, camera, motion, guard, sim)."""
    src = Path(source) if not isinstance(source, Path) else source
    name = str(source)
    if not isinstance(raw, dict):
        raise ProfileError(f'{name}: expected a profile like "label: My rover" at the top level')
    stem = src.stem if isinstance(source, Path) else None

    def bad(msg):
        return ProfileError(f'{name}: {msg}')
    try:
        _unknown(raw, TOP_KEYS, 'profile')
        pid = raw.get('id', stem)
        if not isinstance(pid, str) or not ID.fullmatch(pid):
            raise ProfileError('id: lowercase letters, digits, _ and -, starting with a letter or digit (up to 48)')
        if stem is not None and pid != stem:
            raise ProfileError(f'id: {pid!r} is not the file name {stem!r} (the file name is the id; leave id out)')
        label = raw.get('label', pid)
        if not isinstance(label, str) or not label.strip() or len(label) > 80:
            raise ProfileError('label: the name people read, up to 80 characters')
        for key in ('maker', 'description'):
            if raw.get(key) is not None and (not isinstance(raw[key], str) or len(raw[key]) > 400):
                raise ProfileError(f'{key}: text, up to 400 characters')

        # the robot descriptor: the short form, or a packaged-style robot: section
        if raw.get('robot') is not None:
            short = sorted(k for k in ('kind', 'base', 'arms', 'head_dof', 'limits') if k in raw)
            if short:
                raise ProfileError(f'{", ".join(short)}: give these inside robot:, or leave robot: out')
            r = dict(_section(raw['robot'], 'robot'))
            _unknown(r, {'schema', 'kind', 'family', 'arms', 'base', 'head_dof', 'limits'}, 'robot')
            if r.get('schema', ROBOT_SCHEMA) != ROBOT_SCHEMA:
                raise ProfileError(f'robot.schema: {ROBOT_SCHEMA}')
            where = 'robot.'
        else:
            r = {k: raw[k] for k in ('kind', 'base', 'arms', 'head_dof', 'limits') if k in raw}
            where = ''
        if r.get('kind') not in KINDS:
            raise ProfileError(f'{where}kind: wheeled (or rover) or humanoid')
        kind = KINDS[r['kind']]
        base = r.get('base', 'wheeled' if kind == 'wheeled' else None)
        if base not in BASES:
            raise ProfileError(f'{where}base: legged or wheeled' + (' (a humanoid has to say which)' if kind == 'humanoid' else ''))
        if kind == 'wheeled' and base != 'wheeled':
            raise ProfileError(f'{where}base: a wheeled body has a wheeled base')
        arms = _int(r.get('arms', 0), f'{where}arms', 0, 2)
        head_dof = _int(r.get('head_dof', 0), f'{where}head_dof', 0, 2)
        family = r.get('family', pid)
        if not isinstance(family, str) or not LABEL.fullmatch(family):
            raise ProfileError(f'{where}family: a short name (letters, digits, _ . - and spaces, up to 48)')
        lim_in = _section(r.get('limits'), f'{where}limits')
        _unknown(lim_in, LIMIT_KEYS, f'{where}limits')
        lw = f'{where}limits.'
        for key in ('vx_m_s', 'vyaw_rad_s'):
            if key not in lim_in:
                raise ProfileError(f'{lw}{key}: required, [low, high] (the base\'s speed range)')
        lim = dict(vx_m_s=_pair(lim_in['vx_m_s'], f'{lw}vx_m_s', -5, 5),
                   vy_m_s=_pair(lim_in.get('vy_m_s', [0., 0.]), f'{lw}vy_m_s', -5, 5),
                   vyaw_rad_s=_pair(lim_in['vyaw_rad_s'], f'{lw}vyaw_rad_s', -10, 10))
        for key, text in (('vx_m_s', 'forward'), ('vyaw_rad_s', 'turning')):
            if not lim[key][0] <= 0 <= lim[key][1]:
                raise ProfileError(f'{lw}{key}: the range has to include 0 (standing still); it is the {text} speed range')
        if not lim['vy_m_s'][0] <= 0 <= lim['vy_m_s'][1]:
            raise ProfileError(f'{lw}vy_m_s: the range has to include 0 ([0, 0] for a base that cannot move sideways)')
        if head_dof >= 1 and 'head_pitch_rad' not in lim_in:
            raise ProfileError(f'{lw}head_pitch_rad: required for a head that pitches (head_dof {head_dof})')
        if head_dof >= 2 and 'head_yaw_rad' not in lim_in:
            raise ProfileError(f'{lw}head_yaw_rad: required for a head that turns (head_dof 2)')
        for key in ('head_pitch_rad', 'head_yaw_rad'):
            if key in lim_in:
                lim[key] = _pair(lim_in[key], f'{lw}{key}', -math.pi, math.pi)
        if 'walk_min_pitch_rad' in lim_in:
            lim['walk_min_pitch_rad'] = _num(lim_in['walk_min_pitch_rad'], f'{lw}walk_min_pitch_rad', -math.pi / 2, math.pi / 2)
        lim['head_rate_rad_s'] = _num(lim_in.get('head_rate_rad_s', 1.5), f'{lw}head_rate_rad_s', 0.01, 20)
        lim['deadband_m_s'] = _num(lim_in.get('deadband_m_s', 0.), f'{lw}deadband_m_s', 0, 1)
        robot = dict(schema=ROBOT_SCHEMA, kind=kind, family=family, arms=arms, base=base, head_dof=head_dof, limits=lim)

        cam_in = _section(raw.get('camera'), 'camera')
        _unknown(cam_in, CAMERA_KEYS, 'camera')
        for key in ('size_px', 'hfov_deg', 'eye_height_m'):
            if key not in cam_in:
                raise ProfileError(f'camera.{key}: required (size_px [width, height], hfov_deg, eye_height_m: the camera\'s '
                                   'height above the floor)')
        size = cam_in['size_px']
        if not isinstance(size, (list, tuple)) or len(size) != 2:
            raise ProfileError('camera.size_px: [width, height] in pixels')
        size = [_int(size[0], 'camera.size_px[0]', 16, 16384), _int(size[1], 'camera.size_px[1]', 16, 16384)]
        camera = dict(name='head', size_px=size, hfov_deg=_num(cam_in['hfov_deg'], 'camera.hfov_deg', 10, 179),
                      eye_height_m=_num(cam_in['eye_height_m'], 'camera.eye_height_m', 0.1, 3.0),
                      pitch_rad=_num(cam_in.get('pitch_rad', 0.), 'camera.pitch_rad', -math.pi / 2, math.pi / 2))
        if cam_in.get('name', 'head') != 'head':
            raise ProfileError('camera.name: head (the body\'s one camera)')
        if head_dof >= 1 and not lim['head_pitch_rad'][0] <= camera['pitch_rad'] <= lim['head_pitch_rad'][1]:
            raise ProfileError(f'camera.pitch_rad: within {lw}head_pitch_rad {lim["head_pitch_rad"]}')
        for key in ('lens', 'note'):
            if cam_in.get(key) is not None:
                if not isinstance(cam_in[key], str) or len(cam_in[key]) > 200:
                    raise ProfileError(f'camera.{key}: text, up to 200 characters')
                camera[key] = cam_in[key]

        mo_in = _section(raw.get('motion'), 'motion')
        _unknown(mo_in, MOTION_DEFAULTS, 'motion')
        motion = dict(MOTION_DEFAULTS, **mo_in)
        motion['accel_m_s2'] = [_num(v, f'motion.accel_m_s2[{i}]', 0.01, 20) for i, v in enumerate(
            motion['accel_m_s2'] if isinstance(motion['accel_m_s2'], (list, tuple)) and len(motion['accel_m_s2']) == 2 else [None, None])]
        for key, lo, hi in (('accel_yaw_rad_s2', .01, 50), ('stop_decel_m_s2', .01, 20), ('stop_decel_yaw_rad_s2', .01, 50),
                            ('deadman_s', .05, 2), ('command_hz', 5, 200)):
            motion[key] = _num(motion[key], f'motion.{key}', lo, hi)

        sim = None
        if raw.get('sim') is not None:
            sim = dict(_section(raw['sim'], 'sim'))
            _unknown(sim, SIM_KEYS, 'sim')
            if not isinstance(sim.get('model'), str) or not sim['model']:
                raise ProfileError('sim.model: the body\'s MuJoCo model (an MJCF .xml file, relative to this profile)')
            model = Path(sim['model']).expanduser()
            if not model.is_absolute():
                model = (src.parent / model) if isinstance(source, Path) else model
            if isinstance(source, Path) or model.is_absolute():
                if not model.is_file():
                    raise ProfileError(f'sim.model: no file at {model}')
                sim['model'] = str(model.resolve())
            for key in ('base_body', 'head_camera', 'head_camera_frame'):
                if sim.get(key) is not None and not isinstance(sim[key], str):
                    raise ProfileError(f'sim.{key}: the name of a body (or camera) in the model')
            sim.setdefault('base_body', 'base')
            if not sim.get('head_camera_frame'):
                sim.setdefault('head_camera', 'head')
            if 'footprint_m' in sim:
                sim['footprint_m'] = _num(sim['footprint_m'], 'sim.footprint_m', 0.05, 3)
        footprint = raw.get('footprint_m', (sim or {}).get('footprint_m', 0.25))
        footprint = _num(footprint, 'footprint_m', 0.05, 3)
        if sim is not None:
            sim.setdefault('footprint_m', footprint)

        g_in = _section(raw.get('guard'), 'guard')
        guard = guard_defaults(footprint, camera['eye_height_m'])
        _unknown(g_in, set(guard) | {'self_mask_reach_m'}, 'guard')
        guard.update(g_in)
        if guard['schema'] != 'roborun-body-guard/1':
            raise ProfileError('guard.schema: roborun-body-guard/1')
        for key, value in guard.items():
            if key == 'schema':
                continue
            if isinstance(value, (list, tuple)):
                guard[key] = _pair(value, f'guard.{key}', 0, 100)
            else:
                guard[key] = _num(value, f'guard.{key}', 0, 1000)

        agent = None
        if raw.get('agent') is not None:
            agent = dict(_section(raw['agent'], 'agent'))
            _unknown(agent, {'protocol', 'port', 'routes'}, 'agent')
            if agent.get('protocol') not in AGENT_PROTOCOLS:
                raise ProfileError(f'agent.protocol: {" or ".join(AGENT_PROTOCOLS)} (the routes a robot-side agent serves; '
                                   'or leave agent out and run your own BaseAdapter/HeadAdapter with --body)')
    except ProfileError as exc:
        raise bad(str(exc)) from None

    out = dict(id=pid, label=label.strip(), maker=raw.get('maker') or '', description=raw.get('description') or '', robot=robot,
               motion=motion, camera=camera, guard=guard, own=True)
    if sim is not None:
        out['sim'] = sim
    if agent is not None:
        out['agent'] = agent
    if isinstance(source, Path):
        out['file'] = str(source)
    return out


# ---------------------------------------------------------------------------------------------------- helpers
def intrinsics(width, height, hfov_deg):
    """Pinhole intrinsics (OpenCV convention, pixel centres at +0.5) of a camera `hfov_deg` wide."""
    fx = (width / 2.) / math.tan(math.radians(hfov_deg) / 2.)
    return dict(width=int(width), height=int(height), fx=round(fx, 4), fy=round(fx, 4), ppx=round(width / 2. - .5, 4),
                ppy=round(height / 2. - .5, 4))


def clamp(value, pair):
    lo, hi = pair
    return min(hi, max(lo, float(value)))
