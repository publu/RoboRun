"""The humanoid profiles: data/bodies/<id>.json. Each names the body (legged or wheeled base, arms, head), the robot
descriptor a body job sends (manifest.robot-descriptor.v2), the head camera, how commands are shaped, the simulation's model
and the robot-side agent a real body is reached through."""
from __future__ import annotations

import copy
import json
import math
from functools import lru_cache
from pathlib import Path

DATA = Path(__file__).resolve().parents[1] / 'data'
BODIES = DATA / 'bodies'
SCENES = ('room', 'empty')            # the simulated bodies' scenes (arm scenes are cup, pickup-bench, ...)
DEFAULT_SCENE = 'room'


def ids():
    return sorted(p.stem for p in BODIES.glob('*.json'))


def is_body(profile_id):
    return isinstance(profile_id, str) and profile_id in ids()


@lru_cache(maxsize=None)
def _load(profile_id):
    if not is_body(profile_id):
        raise KeyError(f'no humanoid profile {profile_id!r} (have: {", ".join(ids())})')
    return json.loads((BODIES / f'{profile_id}.json').read_text())


def load(profile_id):
    """The profile record (a copy: callers may change it)."""
    return copy.deepcopy(_load(profile_id))


def descriptor(profile_id):
    """manifest.robot-descriptor.v2 of this body, exactly as a body job sends it."""
    return load(profile_id)['robot']


def model_path(profile_id):
    return DATA / load(profile_id)['sim']['model']


def rows():
    """What the profile lists show: id, label, kind, base, arms, head."""
    out = []
    for pid in ids():
        p = load(pid); r = p['robot']
        out.append(dict(id=pid, label=p['label'], kind=r['kind'], base=r['base'], arms=r['arms'], head_dof=r['head_dof']))
    return out


def intrinsics(width, height, hfov_deg):
    """Pinhole intrinsics (OpenCV convention, pixel centres at +0.5) of a camera `hfov_deg` wide."""
    fx = (width / 2.) / math.tan(math.radians(hfov_deg) / 2.)
    return dict(width=int(width), height=int(height), fx=round(fx, 4), fy=round(fx, 4), ppx=round(width / 2. - .5, 4),
                ppy=round(height / 2. - .5, 4))


def clamp(value, pair):
    lo, hi = pair
    return min(hi, max(lo, float(value)))
