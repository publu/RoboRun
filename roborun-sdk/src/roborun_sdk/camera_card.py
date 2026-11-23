"""Camera cards (roborun.camera/1): one description per camera, saying where it is, what it delivers and what it can be used for.

A card is a plain JSON object. Each field is filled by exactly one layer:

  configured  robot.yaml `cameras.<id>` (the source string or section, and its optional `card:` sub-section)
  derived     this module: robot.yaml + the profile's rig (or a body profile) + what the camera reports when it is opened
  reported    the robot runtime, from its calibration records: the `calibration` block (null here)
  computed    the robot runtime: the `planning` block (null here)

The SDK never reads the runtime's calibration records and never computes planning: cards made here carry `calibration: null`
and `planning: null`, and their poses say `preview: true`. The robot's local API (GET /v1/cameras) merges in the runtime's
answer. `roborun cameras describe` prints the SDK's cards.

Conventions:
- camera axes are OpenCV's: x right, y down, z forward (out of the lens), origin at the optical centre;
- a pose is `T_<parent>_camera`, a 4x4 row-major rigid transform in metres mapping camera points to the parent frame;
- parents: `table` (mount `world`: the rig's table frame, the one the planner uses, never an arm base), `<arm>.tool` (that arm's
  grasp_site), `body.head` (a body's head-camera link);
- intrinsics are the camera service's dict: {width, height, fx, fy, ppx, ppy, distortion_model, coeffs}, in pixels of the image
  as delivered (after `rotate`). `null` means "not known", never zero.

robot.yaml:

  cameras:
    top: realsense:SERIAL                 # ids top, left, right are their own alias, as always
    stereo:                               # any other id: lower-case letters, digits and _
      source: my_cam.py:StereoHead
      card:                               # optional; every key optional
        alias: top                        # top | left | right: the task camera it answers as (free ids only)
        label: Stereo head
        mount: world                      # world | left.tool | right.tool (default from the alias)
        pose: [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]   # reported only, never planned from
        depth: false                      # a statement; the camera's frames decide
        rectified: true                   # the source serves undistorted pinhole frames
        sees: [table, left.gripper]
        device: stereo head camera
"""
from __future__ import annotations

import math
import re

SCHEMA = 'roborun.camera/1'
LIST_SCHEMA = 'roborun.cameras/1'
ALIASES = ('top', 'left', 'right')                    # the v1 task cameras (a task's `camera`)
BODY_ALIAS = 'head'
MOUNTS = ('world', 'left.tool', 'right.tool', 'body.head')
ARM_MOUNTS = MOUNTS[:3]
RESERVED = ('overhead', 'both', 'all', 'head')
CARD_KEYS = ('alias', 'label', 'mount', 'pose', 'depth', 'rectified', 'sees', 'device')
ID = re.compile(r'[a-z][a-z0-9_]{0,31}')
SEE = re.compile(r'[a-z][a-z0-9_.]*')
DEFAULT_MOUNT = {'top': 'world', 'left': 'left.tool', 'right': 'right.tool', 'head': 'body.head'}
LABELS = {'top': 'Overhead', 'left': 'Left wrist', 'right': 'Right wrist', 'head': 'Head'}
ROLE_OF_MOUNT = {'world': 'overhead', 'left.tool': 'wrist', 'right.tool': 'wrist', 'body.head': 'egocentric'}
DISTORTION_MODELS = ('none', 'brown_conrady', 'inverse_brown_conrady', 'rational', 'kannala_brandt')
MODEL_NAMES = {'pinhole': 'none', 'brown': 'brown_conrady', 'plumb_bob': 'brown_conrady', 'fisheye': 'kannala_brandt',
               'equidistant': 'kannala_brandt'}
LENS_SOURCE = {'realsense': 'factory', 'custom': 'driver', 'sim': 'sim'}
WRIST_REACH_M = 0.5                                   # wrist_calibration.check's bound
FLAT = 1e-9                                           # a coefficient this small is no distortion
SAME = 1e-6                                           # outbound: frame intrinsics and pose equal the card's within this
DEFAULT_SIZE = (640, 480)

ROTATED_POSE = 'a design or rig mount is for the camera unrotated; calibrate it as delivered'
ASK_ROBOT = "the robot decides this camera's mount; ask the robot (GET /v1/cameras)"
ASK_ROBOT_TEXT = 'calibration, the pose the robot plans from, and planning: ask the robot (GET /v1/cameras)'


class CardError(ValueError):
    """A camera's card or robot.yaml entry is not valid; the message is the sentence to show."""


# ---------------------------------------------------------------------------------------------------- small helpers
def _is_number(v):
    return not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v)


def rigid(T, tol=1e-4):
    """T as a 4x4 list of floats when it is a rigid transform (rotation, metres, last row 0 0 0 1), else None."""
    try:
        rows = [[float(v) for v in row] for row in T]
    except (TypeError, ValueError):
        return None
    if len(rows) != 4 or any(len(r) != 4 for r in rows) or any(not math.isfinite(v) for r in rows for v in r):
        return None
    if any(abs(a - b) > 1e-6 for a, b in zip(rows[3], (0, 0, 0, 1))):
        return None
    R = [r[:3] for r in rows[:3]]
    for i in range(3):
        for j in range(3):
            dot = sum(R[i][k] * R[j][k] for k in range(3))
            if abs(dot - (1. if i == j else 0.)) > tol:
                return None
    det = (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
           + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))
    if abs(det - 1.) > 1e-3:
        return None
    return rows


def _matrix(T):
    return [[float(v) for v in row] for row in T] if T is not None else None


def _close(a, b, tol=SAME):
    return all(abs(float(x) - float(y)) <= tol for ra, rb in zip(a, b) for x, y in zip(ra, rb))


def delivered_size(width, height, rotate):
    """The served image's size: robot.yaml's width/height are the capture size, before `rotate` (90/270 swap them)."""
    if width is None or height is None:
        return None, None
    return (height, width) if rotate in (90, 270) else (width, height)


def normalise_intrinsics(k):
    """Any intrinsics form -> the card's K-dict {width, height, fx, fy, ppx, ppy, distortion_model, coeffs}, or None.

    Accepts the camera service's dict, a calibration record's lens ({model, K, D, size}) or its `rectified` block, and
    wrist_calibration.intrinsics()' dict: the key `model` becomes `distortion_model`, `pinhole` becomes `none`, `brown`
    becomes `brown_conrady`, K becomes fx/fy/ppx/ppy, D becomes coeffs and size becomes width/height."""
    if not k:
        return None
    k = dict(k)
    out = dict(width=None, height=None, fx=None, fy=None, ppx=None, ppy=None, distortion_model=None, coeffs=None)
    size = k.get('size')
    if size is not None and len(size) == 2:
        out['width'], out['height'] = int(size[0]), int(size[1])
    for key in ('width', 'height'):
        if k.get(key) is not None:
            out[key] = int(k[key])
    K = k.get('K')
    if K is not None:
        flat = [float(v) for row in K for v in (row if isinstance(row, (list, tuple)) else [row])]
        if len(flat) == 9:
            out.update(fx=flat[0], fy=flat[4], ppx=flat[2], ppy=flat[5])
    for key in ('fx', 'fy', 'ppx', 'ppy'):
        if k.get(key) is not None:
            out[key] = float(k[key])
    model = k.get('distortion_model', k.get('model'))
    if model is not None:
        model = str(model).lower()
        out['distortion_model'] = MODEL_NAMES.get(model, model)
    coeffs = k.get('coeffs', k.get('D'))
    if coeffs is not None:
        out['coeffs'] = [float(v) for v in (coeffs if not coeffs or not isinstance(coeffs[0], (list, tuple))
                                            else [x for row in coeffs for x in row])]
    elif out['distortion_model'] == 'none':
        out['coeffs'] = [0.] * 5
    return out


def _pinhole_of(k):
    """The pinhole (distortion none) with k's size and K."""
    return dict(normalise_intrinsics(k), distortion_model='none', coeffs=[0.] * 5)


def nominal_pinhole(width, height, hfov_deg=None, vfov_deg=None):
    """A nominal pinhole from a field of view (body.profiles.intrinsics convention): ppx = (w-1)/2, ppy = (h-1)/2,
    fx = fy from the horizontal FOV, or from the vertical one (a rig's fovy_deg), rounded to 4 decimals."""
    if hfov_deg is not None:
        f = (width / 2.) / math.tan(math.radians(hfov_deg) / 2.)
    elif vfov_deg is not None:
        f = (height / 2.) / math.tan(math.radians(vfov_deg) / 2.)
    else:
        raise ValueError('nominal_pinhole: give hfov_deg or vfov_deg')
    return dict(width=int(width), height=int(height), fx=round(f, 4), fy=round(f, 4), ppx=round(width / 2. - .5, 4),
                ppy=round(height / 2. - .5, 4), distortion_model='none', coeffs=[0.] * 5)


def _fov(k):
    if not k or k.get('fx') in (None, 0) or k.get('fy') in (None, 0) or k.get('width') is None:
        return None, None
    return (round(math.degrees(2 * math.atan(k['width'] / 2. / k['fx'])), 1),
            round(math.degrees(2 * math.atan(k['height'] / 2. / k['fy'])), 1))


def _flat(k):
    return k.get('distortion_model') == 'none' or (
        k.get('fx') is not None and k.get('coeffs') is not None and all(abs(float(c)) <= FLAT for c in k['coeffs']))


def rectified(lens, declared=None):
    """True: frames are served as a pinhole (distortion none, or every coefficient 0 with a known K, or declared so).
    False: K is known and has distortion. None: nothing is known and nothing is declared."""
    if declared is True:
        return True
    if lens and (lens.get('distortion_model') == 'none' or (lens.get('fx') is not None and _flat(lens))):
        return True
    if lens and lens.get('fx') is not None:
        return False
    return False if declared is False else None


def _rig_arms(rig):
    """The rig's arm slots that act (a parked slot has no arm), or None when there is no rig."""
    if not rig:
        return None
    return tuple(s for s, a in (rig.get('arms') or {}).items() if isinstance(a, dict) and not isinstance(a.get('parked'), dict))


def _quoted(cid):
    return f'"{cid}"' if not (isinstance(cid, str) and ID.fullmatch(cid)) else cid


# ---------------------------------------------------------------------------------------------------- robot.yaml: check
def check_id(cid, body=False):
    if not isinstance(cid, str) or not ID.fullmatch(cid):
        raise CardError(f'camera "{cid}": a camera name is lower-case letters, digits and _ (up to 32), starting with a letter')
    if cid == 'overhead':
        raise CardError('camera "overhead": that name is taken: call it top, or pick another name')
    if cid in RESERVED and not (body and cid == BODY_ALIAS):
        raise CardError(f'camera "{cid}": that name is taken: pick another name')
    return cid


def card_section(cid, entry):
    """The `card:` sub-section of one robot.yaml camera entry (a dict, possibly empty)."""
    if not isinstance(entry, dict) or entry.get('card') is None:
        return {}
    card = entry['card']
    if not isinstance(card, dict):
        raise CardError(f'camera {cid}: card must be a section (alias, mount, ...)')
    unknown = [k for k in card if k not in CARD_KEYS]
    if unknown:
        raise CardError(f'camera {cid}: card does not take "{unknown[0]}"; it takes {", ".join(CARD_KEYS)}')
    return card


def check_card(cid, card, *, arms=None, body=False):
    """(checked card config, [warnings]) of one camera. `arms`: the rig's acting arm slots, when the profile is known."""
    warnings = []
    alias = card.get('alias')
    if cid in ALIASES:
        if alias is not None and alias != cid:
            raise CardError(f'camera {cid}: it is already the {cid} camera; remove alias or rename the camera')
        alias = cid
    elif body and cid == BODY_ALIAS:
        alias = BODY_ALIAS
    elif alias is not None and alias not in ALIASES:
        raise CardError(f'camera {cid}: alias must be top, left or right')
    mount = card.get('mount')
    if mount is not None:
        if mount == 'body.head' and not body:
            raise CardError(f'camera {cid}: body.head is for body profiles; on an arm robot use world, left.tool or right.tool')
        if mount not in (MOUNTS if body else ARM_MOUNTS):
            raise CardError(f'camera {cid}: mount must be world, left.tool or right.tool, not {mount}')
    else:
        mount = DEFAULT_MOUNT.get(alias)
    if alias == 'top' and mount != 'world':
        raise CardError(f'camera {cid}: the top camera is fixed to the table (mount world)')
    if mount in ('left.tool', 'right.tool'):
        arm = mount.split('.')[0]
        if alias not in (None, arm):
            raise CardError(f'camera {cid}: a camera on the {arm} gripper can only be the {arm} camera (a wrist camera plans its own arm)')
        if arms is not None and arm not in arms:
            if card.get('mount') is not None:
                raise CardError(f'camera {cid}: this robot has no {arm} arm')
            warnings.append(f'camera {cid}: this robot has no {arm} arm, so this camera has no gripper to ride')
    if mount == 'world' and alias in ('left', 'right'):
        raise CardError(f'camera {cid}: a camera fixed to the table cannot be the {alias} camera: the {alias} camera rides the '
                        f'{alias} gripper (mount {alias}.tool)')
    for key in ('depth', 'rectified'):
        if card.get(key) is not None and not isinstance(card[key], bool):
            raise CardError(f'camera {cid}: {key} must be true or false')
    pose = card.get('pose')
    if pose is not None:
        T = rigid(pose)
        if T is None:
            raise CardError(f'camera {cid}: pose must be a 4x4 rigid transform (rotation and metres, last row 0 0 0 1)')
        if mount is None:
            raise CardError(f'camera {cid}: a pose needs a mount (world, left.tool or right.tool) to be measured from')
        if mount.endswith('.tool') and math.sqrt(sum(T[i][3] ** 2 for i in range(3))) > WRIST_REACH_M:
            raise CardError(f'camera {cid}: pose puts the camera more than {WRIST_REACH_M:g} m from the gripper')
        pose = T
    sees = card.get('sees')
    if sees is not None:
        if not isinstance(sees, list) or len(sees) > 8 or any(not isinstance(s, str) or len(s) > 40 or not SEE.fullmatch(s) for s in sees):
            raise CardError(f'camera {cid}: sees is a list of up to 8 short names such as table or left.gripper')
    for key in ('label', 'device'):
        value = card.get(key)
        if value is not None and (not isinstance(value, str) or not value.strip() or len(value) > 80):
            raise CardError(f'camera {cid}: {key} is a short text (up to 80 characters)')
    out = dict(alias=alias, label=card.get('label') or LABELS.get(alias) or cid, mount=mount, pose=pose,
               depth=card.get('depth'), rectified=card.get('rectified'), sees=list(sees) if sees is not None else None,
               device=card.get('device'))
    return out, warnings


def check_config(cameras_section, rig=None, *, body=False):
    """({id: card config}, [warnings]) for robot.yaml's `cameras:` section, after the back-compat rewrites (a single right
    arm moved to the left slot, `--camera` overlays). Raises CardError (a ValueError) with the sentence to show."""
    cameras_section = cameras_section or {}
    if not isinstance(cameras_section, dict):
        raise CardError('cameras must be a section: top, left, right or your own camera names')
    arms = _rig_arms(rig)
    out, warnings, holder = {}, [], {}
    for cid, entry in cameras_section.items():
        check_id(cid, body=body)
        cfg, more = check_card(cid, card_section(cid, entry), arms=arms, body=body)
        warnings += more
        alias = cfg['alias']
        if alias is not None:
            if alias in holder:
                raise CardError(f'cameras {holder[alias]} and {cid} both answer as {alias}: give one of them another alias or none')
            holder[alias] = cid
        out[cid] = cfg
    return out, warnings


def alias_of(cid, entry):
    """The alias a robot.yaml entry answers as (None for a free id without one), without checking anything else."""
    if cid in ALIASES:
        return cid
    card = entry.get('card') if isinstance(entry, dict) else None
    alias = card.get('alias') if isinstance(card, dict) else None
    return alias if alias in ALIASES else None


def without_alias(entry):
    """A copy of a robot.yaml camera entry with its card's alias taken away (`--camera ROLE=...` took that role for this run)."""
    entry = dict(entry)
    entry['card'] = {k: v for k, v in entry['card'].items() if k != 'alias'}
    return entry


def overlay_cameras(cameras_section, role, source, say=None):
    """`roborun up --camera ROLE=SOURCE`: the entry with id ROLE becomes SOURCE (as always), and a camera of another id that
    answers as ROLE loses that alias for this run, so the two never both answer as ROLE."""
    cams = dict(cameras_section or {})
    for cid, entry in list(cams.items()):
        if cid != role and alias_of(cid, entry) == role:
            cams[cid] = without_alias(entry)
            if say:
                say(f'camera {cid}: --camera {role} replaces it as the {role} camera for this run')
    cams[role] = source
    return cams


def move_right_to_left(cameras_section):
    """A single right arm runs in the left slot: every card written for the right gripper is moved with it."""
    out = {}
    for cid, entry in (cameras_section or {}).items():
        card = entry.get('card') if isinstance(entry, dict) else None
        if isinstance(card, dict) and (card.get('mount') == 'right.tool' or card.get('alias') == 'right'):
            card = dict(card)
            if card.get('mount') == 'right.tool':
                card['mount'] = 'left.tool'
            if card.get('alias') == 'right':
                card['alias'] = 'left'
            entry = dict(entry, card=card)
        out[cid] = entry
    return out


# ---------------------------------------------------------------------------------------------------- cards
def _source(settings, kind=None):
    from .cameras import parse_source, redact
    text = str(settings.get('source') or '')
    if kind is None:
        try:
            kind = parse_source(text)[0]
        except ValueError:
            kind = None
    return dict(kind=kind, ref=redact(text) if kind != 'sim' else text)


def _rig_camera(rig, alias):
    if not rig or alias is None:
        return {}
    cam = (rig.get('cameras') or {}).get(alias)
    return cam if isinstance(cam, dict) else {}


def _pose(cid, cfg, rotate, rig, *, sim_pose=None, cad=None):
    mount = cfg['mount']
    if mount is None:
        return None
    alias = cfg['alias']
    parent = 'table' if mount == 'world' else mount
    out = dict(parent=parent, T=None, source='none', preview=True, nominal=True, warning=None)
    if sim_pose is not None:
        out.update(T=_matrix(sim_pose), source='sim', preview=False, nominal=False)
    elif mount == 'world':
        T = _rig_camera(rig, alias).get('T_table_camera') if alias == 'top' else None
        if T is not None and rigid(T) is not None:
            out.update(T=_matrix(T), source='nominal')
    elif mount.endswith('.tool'):
        arm = mount.split('.')[0]
        own = alias == arm                                   # the rig's wrist camera of this arm
        T = _rig_camera(rig, arm).get('T_tool_camera') if own else None
        spec = ((rig or {}).get('arms') or {}).get(arm) or {}
        if T is not None and rigid(T) is not None:
            out.update(T=_matrix(T), source='rig_mount')
        elif own and spec.get('variant') == 'yam':
            if cad is not None:
                out.update(T=_matrix(cad), source='cad')
            else:
                out['warning'] = ASK_ROBOT
        elif own and (not rig or not spec.get('variant')) and 'yam' in str(spec.get('model') or 'yam'):
            out['warning'] = ASK_ROBOT                       # the runtime may give the YAM design mount here: it decides
        elif own:
            variant = spec.get('variant') or spec.get('model')
            out['warning'] = (f'The {arm} wrist camera has no measured mount on this robot ({variant}): its position on the '
                              'gripper is not known yet, so motion cannot be planned from it')
        else:
            out['warning'] = f'no mount is known for camera {cid} on the {arm} gripper: it is described, not planned from'
    if rotate and out['source'] in ('cad', 'rig_mount', 'nominal'):
        out.update(T=None, source='none', nominal=True, warning=ROTATED_POSE)
    if cfg.get('pose') is not None:
        out['configured'] = _matrix(cfg['pose'])
    return out


def derive(cid, settings, cfg, rig=None, model=None, sim=None, cad=None):
    """The SDK layer of one camera's card.

    cid       the robot.yaml id; settings: cameras.camera_settings() of its entry (source, width, height, fps, rotate)
    cfg       its checked card config (check_config)
    rig       the profile's rig.json (dict), when known: nominal poses and rig mounts
    model     what the camera reported when it was opened (the broker's status models[role]), when it runs
    sim       in `roborun sim`: {'pose': the 4x4 the simulation draws it at, 'intrinsics': the K it serves, 'depth': bool}
    cad       the YAM's design wrist mount (4x4), when the runtime is at hand to give it
    """
    settings = dict(settings or {})
    rotate = int(settings.get('rotate') or 0)
    model = model or {}
    alias, mount = cfg['alias'], cfg['mount']
    w, h = delivered_size(settings.get('width'), settings.get('height'), rotate)
    lens = None
    kind = 'sim' if sim else None
    src = _source(settings, kind)
    if sim:
        lens = dict(normalise_intrinsics(sim['intrinsics']), source='sim')
    elif model.get('intrinsics'):
        k = normalise_intrinsics(model['intrinsics'])
        lens = dict(k, source='none' if k.get('fx') is None else LENS_SOURCE.get(src['kind'], 'driver'))
    if lens and lens.get('width') is not None:
        w, h = lens['width'], lens['height']
    declared = cfg.get('rectified') if cfg.get('rectified') is not None else model.get('rectified')
    is_rectified = rectified(lens, declared)
    pinhole, nominal_k = None, False
    if lens and lens.get('fx') is not None and (sim or is_rectified):
        pinhole = _pinhole_of(lens)
    elif mount == 'world' and alias == 'top' and not model and _rig_camera(rig, 'top').get('fovy_deg'):
        pw, ph = (w, h) if w is not None else DEFAULT_SIZE
        pinhole, nominal_k = nominal_pinhole(pw, ph, vfov_deg=float(_rig_camera(rig, 'top')['fovy_deg'])), True
    hfov, vfov = _fov(pinhole or lens)
    if sim:
        available = bool(sim.get('depth', True))
        depth = dict(available=available, scale_m=sim.get('depth_scale_m') if available else None,
                     aligned_to='color' if available else None, declared=cfg.get('depth'))
    else:
        depth = dict(available=model.get('depth_available'), scale_m=model.get('depth_scale_m'),
                     aligned_to=model.get('depth_aligned_to'), declared=cfg.get('depth'))
    optics = dict(delivered=dict(width=w, height=h, rotate=rotate, fps=settings.get('fps')), lens=lens, rectified=is_rectified,
                  pinhole=pinhole)
    if nominal_k:
        optics['pinhole_nominal'] = True
    optics.update(hfov_deg=hfov, vfov_deg=vfov, depth=depth)
    card = dict(schema=SCHEMA, id=cid, alias=alias, label=cfg.get('label') or LABELS.get(alias) or cid,
                role=ROLE_OF_MOUNT.get(mount), source=src, mount=mount,
                pose=_pose(cid, cfg, 0 if sim else rotate, rig, sim_pose=(sim or {}).get('pose'), cad=cad),
                optics=optics)
    if cfg.get('device'):
        card['device'] = cfg['device']
    if cfg.get('sees') is not None:
        card['sees'] = list(cfg['sees'])
    card.update(calibration=None, planning=None)
    return card


def body_card(profile, sim=None):
    """The head camera of a body profile (an id, or the loaded profile record) as a card. `sim`: {'intrinsics': the K the
    simulation serves} in `roborun sim --body`."""
    from .body import profiles
    record = profiles.load(profile) if isinstance(profile, str) else profile
    pid = profile if isinstance(profile, str) else (record.get('id') or 'body')
    cam = record['camera']
    w, h = (int(v) for v in cam['size_px'])
    head = dict(eye_height_m=cam['eye_height_m'], pitch_rad=cam.get('pitch_rad', 0.), head_dof=record['robot'].get('head_dof', 0))
    pose = dict(parent='body.head', T=None, source='profile', preview=False, nominal=True, warning=None, body=head)
    if sim:
        frame = (record.get('sim') or {}).get('head_camera_frame')
        identity = [[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]]
        pose.update(T=identity if frame else None, source='sim', nominal=False)
        pinhole, nominal_k = _pinhole_of(sim['intrinsics']), False
    else:
        pinhole, nominal_k = nominal_pinhole(w, h, hfov_deg=float(cam['hfov_deg'])), True
    hfov, vfov = _fov(pinhole)
    optics = dict(delivered=dict(width=w, height=h, rotate=0, fps=None), lens=None, rectified=True if sim else None,
                  pinhole=pinhole)
    if nominal_k:
        optics['pinhole_nominal'] = True
    optics.update(hfov_deg=hfov, vfov_deg=vfov, depth=dict(available=None, scale_m=None, aligned_to=None, declared=None))
    return dict(schema=SCHEMA, id=BODY_ALIAS, alias=BODY_ALIAS, label=LABELS[BODY_ALIAS], role='egocentric',
                source=dict(kind='body', ref=pid), mount='body.head', pose=pose, optics=optics, calibration=None, planning=None)


def card_warnings(card):
    """Plain sentences about one card: a pose that could not be given, depth declared against what the camera gives."""
    out = []
    pose = card.get('pose') or {}
    if pose.get('warning'):
        out.append(f"camera {card['id']}: {pose['warning']}")
    depth = card['optics']['depth']
    if depth.get('declared') is True and depth.get('available') is False:
        out.append(f"camera {card['id']}: card says depth but the camera gives none: depth comes from the camera")
    elif depth.get('declared') is False and depth.get('available') is True and card['source']['kind'] != 'sim':
        out.append(f"camera {card['id']}: card says no depth but the camera gives depth: depth comes from the camera")
    return out


# ---------------------------------------------------------------------------------------------------- validation of a card
REQUIRED = ('schema', 'id', 'alias', 'role', 'source', 'mount', 'pose', 'optics', 'calibration', 'planning')


def validate(card):
    """Raise CardError ('card: ...') when a card object does not follow roborun.camera/1; return it otherwise."""
    def bad(msg):
        raise CardError(f'card: {msg}')
    if not isinstance(card, dict):
        bad('a card is a JSON object')
    missing = [k for k in REQUIRED if k not in card]
    if missing:
        bad(f'it needs {", ".join(missing)}')
    if card['schema'] != SCHEMA:
        bad(f'schema must be {SCHEMA}')
    body = card['mount'] == 'body.head'
    try:
        check_id(card['id'], body=body)
    except CardError as exc:
        bad(str(exc))
    if card['alias'] not in (*ALIASES, BODY_ALIAS, None):
        bad('alias must be top, left, right, head or null')
    if card['mount'] not in (*MOUNTS, None):
        bad(f"mount must be world, left.tool, right.tool, body.head or null, not {card['mount']}")
    if card['role'] != ROLE_OF_MOUNT.get(card['mount']):
        bad(f"role must be {ROLE_OF_MOUNT.get(card['mount'])} for mount {card['mount']}")
    pose = card['pose']
    if (pose is None) != (card['mount'] is None):
        bad('pose is null exactly when mount is null')
    if pose is not None:
        want = 'table' if card['mount'] == 'world' else card['mount']
        if pose.get('parent') != want:
            bad(f'pose.parent must be {want}')
        if pose.get('source') not in ('calibration', 'rig_mount', 'cad', 'nominal', 'sim', 'profile', 'none'):
            bad('pose.source must be calibration, rig_mount, cad, nominal, sim, profile or none')
        if pose.get('T') is not None and rigid(pose['T']) is None:
            bad('pose.T must be a 4x4 rigid transform')
        if pose.get('source') == 'none' and pose.get('T') is not None:
            bad('pose.T must be null when pose.source is none')
        if not isinstance(pose.get('preview'), bool) or not isinstance(pose.get('nominal'), bool):
            bad('pose.preview and pose.nominal are true or false')
    optics = card['optics']
    if not isinstance(optics, dict) or not {'delivered', 'lens', 'rectified', 'pinhole', 'depth'} <= set(optics):
        bad('optics needs delivered, lens, rectified, pinhole and depth')
    if optics['delivered'].get('rotate') not in (0, 90, 180, 270):
        bad('optics.delivered.rotate must be 0, 90, 180 or 270')
    for key in ('lens', 'pinhole'):
        k = optics[key]
        if k is None:
            continue
        if 'model' in k:
            bad(f'optics.{key}: the key is distortion_model, never model')
        if k.get('distortion_model') not in (*DISTORTION_MODELS, None):
            bad(f'optics.{key}.distortion_model must be one of {", ".join(DISTORTION_MODELS)} or null')
    lens, delivered = optics['lens'], optics['delivered']
    if lens is not None and (lens.get('width'), lens.get('height')) != (delivered.get('width'), delivered.get('height')):
        bad('optics.lens must have the delivered image\'s width and height')
    if optics['pinhole'] is not None and optics['pinhole'].get('distortion_model') != 'none':
        bad('optics.pinhole has distortion_model none')
    if optics['rectified'] not in (True, False, None):
        bad('optics.rectified is true, false or null')
    if card.get('sees') is not None and (not isinstance(card['sees'], list) or len(card['sees']) > 8):
        bad('sees is a list of up to 8 short names')
    return card


# ---------------------------------------------------------------------------------------------------- outbound
def outbound(card, *, intrinsics, T_table_camera, T_tool_camera, table_z_m=None):
    """The hosted API's `camera` block for a wrist job, from a wrist card and the values the exposure was planned with:
    {name, role, intrinsics, T_table_camera, T_tool_camera[, table_z_m]}. Camera facts only: never a joint, a serial or a
    path. intrinsics = the prepared frame's frame['intrinsics']; T_table_camera = fk(q_src) . T_tool_camera, computed on the
    robot; T_tool_camera = the mount used for that composition. Raises ValueError with the sentence to show."""
    if card.get('role') != 'wrist' or card.get('alias') not in ('left', 'right'):
        raise ValueError("outbound() is for wrist cameras; the top camera's and the body's jobs keep their own camera block")
    name = card['alias']
    pinhole = card['optics'].get('pinhole')
    if pinhole is None or card['optics'].get('pinhole_nominal') or pinhole.get('fx') is None:
        raise ValueError(f'the {name} camera has no pinhole intrinsics yet: it needs a calibration before motion can be planned from it')
    k = normalise_intrinsics(intrinsics)
    if not k or any(k.get(key) is None for key in ('fx', 'fy', 'ppx', 'ppy')) or \
            (k['width'], k['height']) != (pinhole['width'], pinhole['height']) or \
            any(abs(k[key] - pinhole[key]) > SAME for key in ('fx', 'fy', 'ppx', 'ppy')):
        raise ValueError(f"the {name} frame's intrinsics are not the camera's pinhole: the frame was not prepared with this "
                         "camera's calibration")
    if not _flat(k):
        raise ValueError(f"the {name} frame is not rectified: its intrinsics still carry lens distortion "
                         f"({k.get('distortion_model') or 'unknown model'}), so it cannot be sent as a pinhole; rectify it "
                         "with this camera's calibration first")
    pose = card.get('pose') or {}
    if pose.get('source') in (None, 'none') or pose.get('T') is None:
        raise ValueError(f'the {name} wrist camera has no known mount: motion cannot be planned from it')
    tool = rigid(T_tool_camera) if T_tool_camera is not None else None
    if tool is None:
        raise ValueError(f'the {name} wrist camera has no known mount: motion cannot be planned from it')
    if pose.get('preview') is False and not _close(tool, pose['T']):
        raise ValueError(f"the {name} wrist camera's mount is not the one the robot plans from")
    table = rigid(T_table_camera) if T_table_camera is not None else None
    if table is None:
        raise ValueError(f"the {name} wrist camera's pose at the exposure is needed (fk(q) · T_tool_camera, computed on the robot)")
    out = dict(name=name, role='wrist',
               intrinsics=dict(width=k['width'], height=k['height'], fx=k['fx'], fy=k['fy'], ppx=k['ppx'], ppy=k['ppy'],
                               distortion_model='none', coeffs=[0.] * 5),
               T_table_camera=table, T_tool_camera=tool)
    if table_z_m is not None:
        out['table_z_m'] = float(table_z_m)
    return out


# ---------------------------------------------------------------------------------------------------- sentences
def _m(v):
    return f'{v:.3f}'.rstrip('0').rstrip('.') if v is not None else '?'


def sentence(card):
    """One plain sentence describing a card, e.g. 'left (Left wrist): a wrist camera on the left gripper, USB, 640x480 turned
    180°, RGB only; no measured mount yet.'"""
    o = card['optics']; d = o['delivered']; pose = card.get('pose') or {}
    where = {'world': 'fixed to the table', 'left.tool': 'on the left gripper', 'right.tool': 'on the right gripper',
             'body.head': "in the body's head"}.get(card['mount'], 'with no mount (viewing and recording only)')
    kind = {'overhead': 'a table camera', 'wrist': 'a wrist camera', 'egocentric': 'a head camera'}.get(card['role'], 'a camera')
    src = {'realsense': 'RealSense', 'usb': 'USB', 'stream': 'network stream', 'custom': 'your own class', 'sim': 'simulated',
           'body': 'body profile'}.get(card['source']['kind'], 'unknown source')
    size = f"{d['width']}x{d['height']}" if d.get('width') else 'size not known yet'
    if d.get('rotate'):
        size += f"{'' if d.get('width') else ','} turned {d['rotate']}°"
    depth = {True: 'RGB + depth', False: 'RGB only', None: 'depth not known yet'}[o['depth'].get('available')]
    parts = [f"{kind} {where}", src, size, depth]
    if o.get('pinhole'):
        p = o['pinhole']
        parts.append(f"{'nominal ' if o.get('pinhole_nominal') else ''}pinhole fx {p['fx']:.1f}"
                     + (f", {o['hfov_deg']:.0f}° wide" if o.get('hfov_deg') else ''))
    elif o.get('lens') and o['lens'].get('fx') is not None:
        parts.append('lens with distortion (the robot rectifies it)')
    else:
        parts.append('no intrinsics yet')
    if pose:
        if pose.get('T') is not None:
            t = [pose['T'][i][3] for i in range(3)]
            what = {'calibration': 'calibrated', 'rig_mount': "rig's mount", 'cad': 'design mount', 'nominal': 'nominal',
                    'sim': 'simulated', 'profile': 'profile'}.get(pose['source'], pose['source'])
            parts.append(f"pose ({what}{', preview' if pose.get('preview') else ''}) at ({_m(t[0])}, {_m(t[1])}, {_m(t[2])}) m "
                         f"in the {pose['parent']} frame")
        elif pose.get('body'):
            b = pose['body']
            parts.append(f"eye height {_m(b['eye_height_m'])} m, pitch {_m(b['pitch_rad'])} rad")
        else:
            parts.append('no known pose')
    head = card['id'] if card['label'] == card['id'] else f"{card['id']} ({card['label']})"
    if card['alias'] and card['alias'] != card['id']:
        head += f", answers as {card['alias']}"
    return f"{head}: {', '.join(parts)}."


# ---------------------------------------------------------------------------------------------------- describe
def rig_for(profile):
    """The rig.json (dict) of a profile name as robot.yaml or --profile names it: a catalog profile, a bundled one, or one
    made on this computer (roborun profiles create). None when it is not found."""
    import json
    from . import arm_profiles
    if not profile:
        return None
    rid = arm_profiles.runtime_profile(arm_profiles.user_profile(str(profile), say=lambda _: None))
    try:
        named = arm_profiles.rig(rid)
    except (OSError, ValueError, KeyError):
        named = None
    if named is not None:
        return named
    from .robot import HOME, PROFILES
    for path in (PROFILES / f'{rid}.json', HOME / 'profiles' / rid / 'rig.json'):
        try:
            return json.loads(path.read_text())
        except (OSError, ValueError):
            continue
    return None


def is_body_profile(profile):
    try:
        from .body import profiles
        return bool(profile) and profiles.is_body(str(profile))
    except Exception:                                   # a broken own body profile is the body commands' to report
        return False


def describe(cameras_section, *, profile=None, rig=None, status=None, cad=None):
    """({"schema": "roborun.cameras/1", "profile", "table_frame", "cameras": [card, ...]}, [warnings]) for robot.yaml's
    cameras (after its rewrites), the profile's rig (or body profile) and, when the camera service answers, its status.
    Raises CardError when a camera's entry is not valid."""
    from .cameras import camera_settings
    body = is_body_profile(profile)
    if rig is None and profile and not body:
        rig = rig_for(profile)
    warnings = []
    if profile and not body and rig is None:
        warnings.append(f'profile {profile} is not known here, so which arms the cameras ride is not checked')
    cards_cfg, more = check_config(cameras_section, rig, body=body)
    warnings += more
    status = status or {}
    sim_cards = {c['alias'] or c['id']: c for c in status.get('cards') or [] if isinstance(c, dict)} if status.get('sim') else {}
    models = status.get('models') or {}
    cards = []
    if body:
        cards.append(body_card(str(profile)))
    for cid, entry in (cameras_section or {}).items():
        cfg = cards_cfg[cid]
        role = cfg['alias'] or cid
        if role in sim_cards:
            cards.append(sim_cards.pop(role))
            continue
        settings, _ = camera_settings(cid, entry)
        cards.append(derive(cid, settings, cfg, rig=rig, model=models.get(role), cad=cad))
    cards += list(sim_cards.values())                   # the simulation's own cameras robot.yaml does not name
    for card in cards:
        validate(card)
        warnings += card_warnings(card)
    doc = dict(schema=LIST_SCHEMA, profile=str(profile) if profile else None,
               table_frame=(rig or {}).get('frame') if not body else None, cameras=cards)
    return doc, warnings


def describe_text(doc, warnings=()):
    """`roborun cameras describe` as people read it."""
    lines = []
    if doc.get('profile'):
        lines.append(f"Profile {doc['profile']}" + (f", table frame {doc['table_frame']}" if doc.get('table_frame') else '') + '.')
    if not doc['cameras']:
        lines.append('No cameras are set up. Add them under cameras: in robot.yaml, or with roborun setup.')
    for card in doc['cameras']:
        lines.append(sentence(card))
    for w in warnings:
        lines.append(f'warning: {w}')
    if any((c.get('pose') or {}).get('preview') for c in doc['cameras']):
        lines.append('Poses marked preview are this computer\'s best guess; ' + ASK_ROBOT_TEXT + '.')
    return '\n'.join(lines)
