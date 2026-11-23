"""The profiles of the arm catalog: a model per arm ships with the SDK (data/arm_models/<id>/), and the profile for your arms is put together on this machine.

    so101                  one arm, on the left slot
    so101_dual             two of them
    so101+koch_v1_1        left arm + right arm of different kinds

ensure() writes the profile into the Roborun install (or, before one exists, the SDK's folder) exactly like `roborun profiles create` does:
models/custom/<id>/ and profiles/<name>/{rig,config}.json. The arms stand on the table in front of the overhead camera, the camera looking at where they work.
"""
from __future__ import annotations

import json
import math
import shutil
from pathlib import Path

MODELS = Path(__file__).parent / 'data' / 'arm_models'
CUP_MM = 81                              # the YAM's demo cup (mm across): the rig's overhead camera is placed for a cup this size
CAMERA_MIN_Z = .65                       # the lowest the overhead camera comes down for a small arm's cup
OBJECT_XY = (-.425, .45)                 # the demo cup: ~0.4 m off the axis of the rig's overhead camera (above (-.125, .75)), so the
                                         # camera sees its side and open top (straight below the camera a cup is only a ring, hard to recognise)


def models():
    return sorted(p.parent.name for p in MODELS.glob('*/arm.json'))


def info(model):
    return json.loads((MODELS / model / 'arm.json').read_text())


# the install's own tuned profiles: a catalog name never takes one of these over (a single YAM is yam_solo, not yam)
RESERVED = {'yam', 'yam_single', 'yam_metal', 'makerarm', 'makerarm_urdf'}

# The names people type and read (`yam` is ONE YAM, `yam_dual` two, like `so101` / `so101_dual`) beside the ids the robot
# runtime, the page and the hosted API know (there `yam` is the tuned pair and `yam_solo` one YAM). The ids never change: the
# names are mapped here, at the command line's edge (`--profile`, robot.yaml, what the commands print).
TO_RUNTIME = {'yam': 'yam_solo', 'yam_dual': 'yam'}
FROM_RUNTIME = {rid: name for name, rid in TO_RUNTIME.items()}
RENAMED = {'yam_single': 'yam', 'makerarm': 'maker_arm_dual'}      # old names that keep working for a while
DEFAULT = 'yam'                                                     # one YAM arm
SIM_DEFAULT = 'yam_dual'                                            # `roborun up --sim cup` with no profile: the proven two-arm scene
PREMADE = {'yam': 'one YAM arm', 'yam_dual': 'two YAM arms', 'yam_metal': 'a YAM arm and a Metal arm'}


def user_profile(name, say=print):
    """A profile name as typed: an old name is mapped to its new one, said in one line."""
    if name in RENAMED:
        say(f'profile {name} was renamed to {RENAMED[name]}')
        return RENAMED[name]
    return name


def runtime_profile(name):
    """The runtime's id of a profile name (`yam` -> `yam_solo`, `yam_dual` -> `yam`; every other name is its own id)."""
    return TO_RUNTIME.get(name, name) if name else name


def shown_profile(rid):
    """The name people read for a runtime id (`yam` -> `yam_dual`, `yam_solo` -> `yam`)."""
    return FROM_RUNTIME.get(rid, rid) if rid else rid


def parse(profile):
    """(left model, right model or None) of a catalog profile name, else None."""
    if not profile or profile in RESERVED:
        return None
    have = set(models())
    if profile.endswith('_solo') and profile[:-5] in have:
        return profile[:-5], None
    if '+' in profile:
        left, _, right = profile.partition('+')
        return (left, right) if {left, right} <= have else None
    if profile.endswith('_dual') and profile[:-5] in have:
        return profile[:-5], profile[:-5]
    return (profile, None) if profile in have else None


def name_for(left, right=None):
    """The profile name of these catalog models."""
    if right is None:
        return f'{left}_solo' if left in RESERVED else left
    return f'{left}_dual' if left == right else f'{left}+{right}'


def _T(x, y, z, yaw_deg):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return [[round(c, 9), round(-s, 9), 0, round(x, 5)], [round(s, 9), round(c, 9), 0, round(y, 5)], [0, 0, 1, round(z, 5)], [0, 0, 0, 1]]


def _entry(a, side, T, parked=False):
    row = a['sides'].get(side) or a['sides']['left']
    out = dict(variant=a['id'], gripper=f"{a['id']}_gripper", model=f"custom/{a['id']}/{row['model']}", T_table_base=T, joints=row['joints'],
               home=row['home'], rest_rad=row['rest'], jaw_gap_m=row['jaw_gap_m'])
    if row.get('jaw'):
        out['jaw'] = row['jaw']
    if parked:
        out['parked'] = dict(positions_rad=row['home'], gripper=1.0, source='catalog profile: this slot has no arm', never_commanded=True)
    return out


def layout(left, right=None):
    """Where the arms stand: the demo object at OBJECT_XY (in the overhead camera's view, off its axis), straight ahead (+y) of the
    left arm's shoulder, about 55 % of its reach out, along the home pose's finger heading (+y). Two arms stand side by side, near enough that both reach the table between them. Returns the arms'
    placements and the demo object's."""
    a, b = left, right
    reach = a['reach_m'] if b is None else max(a['reach_m'], b['reach_m'])
    r = a.get('object_r_m') or .55 * a['reach_m']     # how far ahead of the shoulder the demo object stands
    c, sn = math.cos(math.radians(a['yaw_deg'])), math.sin(math.radians(a['yaw_deg'])); sx, sy, _ = a['sides']['left']['shoulder']
    shoulder = (c * sx - sn * sy, sn * sx + c * sy)   # the left shoulder from its base, on the table
    y0 = max(-.1, OBJECT_XY[1] - shoulder[1] - r)
    xl = OBJECT_XY[0] - shoulder[0]
    if b is None or a.get('shared_base'):
        xr = xl                                     # one arm, or the two arms of one robot: one base, one torso
    else:
        xr = xl + max(.3, min(.9, .75 * reach))
    out = dict(left=(xl, y0))
    if b is not None:
        out['right'] = (xr, y0)
    out['object'] = (xl + shoulder[0], y0 + shoulder[1] + r)
    return out


def rig(profile):
    """The rig.json of a catalog profile (a dict), or None when `profile` is not one."""
    named = parse(profile)
    if named is None:
        return None
    from .robot import PROFILES
    base = json.loads((PROFILES / 'yam.json').read_text())
    left, right = [info(m) if m else None for m in named]
    where = layout(left, right)
    arms = {'left': _entry(left, 'left', _T(*where['left'], left['base_z_m'], left['yaw_deg']))}
    if right is None:
        arms['right'] = _entry(left, 'right', _T(3., 0., left['base_z_m'], 0.), parked=True)      # one arm: the other slot stands off the table, never commanded
    else:
        arms['right'] = _entry(right, 'right', _T(*where['right'], right['base_z_m'], right['yaw_deg']))
    size = left['sim_object_mm']
    out = {k: v for k, v in base.items() if k not in ('arms', 'description', 'source', 'base_spacing_m', 'base_spacing_source')}
    out['cameras'] = _camera_for(base['cameras'], size[0])
    out.update(arms=arms, description=f"catalog profile {profile}: {left['label']}" + (f" + {right['label']}" if right else ''),
               source='roborun catalog (data/arm_models)', nominal=True,
               sim_object=dict(id='orange_cup', shape='cup', position_mm=[round(where['object'][0] * 1000), round(where['object'][1] * 1000), round(size[2] / 2)],
                               size_mm=size, color=[1.0, .42, .05], fill=.75))
    return out


def _camera_for(cameras, cup_mm):
    """The rig's cameras with the overhead one placed for a cup `cup_mm` across: a smaller arm's cup is smaller, so the camera
    comes down (to CAMERA_MIN_Z at the lowest) and in toward the cup by the same factor, seeing it from the same angle and larger
    in the image than the YAM's (by the square of the size ratio).
    The pose stays nominal: the overhead check calibrates a real camera."""
    out = json.loads(json.dumps(cameras)); top = out.get('top') or {}
    T = top.get('T_table_camera')
    if not T or cup_mm >= CUP_MM:
        return out
    s = max(CAMERA_MIN_Z / T[2][3], (cup_mm / CUP_MM) ** 2)
    T[0][3] = round(OBJECT_XY[0] + (T[0][3] - OBJECT_XY[0]) * s, 4); T[1][3] = round(OBJECT_XY[1] + (T[1][3] - OBJECT_XY[1]) * s, 4)
    T[2][3] = round(T[2][3] * s, 4)
    top['note'] = f'NOMINAL: placed for this arm\'s {cup_mm} mm demo cup ({T[2][3]} m over the table, looking straight down); ' + \
        'the overhead check calibrates the real camera'
    return out


def bundled_models(profile):
    """The catalog models a bundled profile (data/profiles/<name>.json) stands on."""
    from .robot import PROFILES
    f = PROFILES / f'{profile}.json' if profile else None
    if f is None or not f.exists():
        return []
    arms = json.loads(f.read_text()).get('arms') or {}
    named = {Path(x['model']).parts[1] for x in arms.values() if isinstance(x, dict) and str(x.get('model', '')).startswith('custom/')}
    return sorted(named & set(models()))


def _display_arm(xml, entry):
    """One arm as the 3D view draws it: the model's meshes and bodies (ui/robot-kinematics.js makeArm: joint `index` is the place in [arm joints.., finger, finger]; both finger values are the jaw's)."""
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(xml)); n = entry['joints']; jaw = entry.get('jaw'); opening = float(jaw['open_rad']) if jaw else None
    round_list = lambda a, nd=6: [[round(float(x), nd) for x in row] for row in a]
    meshes = [dict(vertices=round_list(m.mesh_vert[m.mesh_vertadr[i]:m.mesh_vertadr[i] + m.mesh_vertnum[i]]),
                   faces=m.mesh_face[m.mesh_faceadr[i]:m.mesh_faceadr[i] + m.mesh_facenum[i]].tolist()) for i in range(m.nmesh)]
    ratio = {}
    if jaw:                                       # a revolute jaw is drawn with positive values: its axis carries the sign of the opening (and of a second jaw's coupling)
        ratio[n] = 1. if opening > 0 else -1.
        for e in range(m.neq):
            if int(m.eq_type[e]) == mujoco.mjtEq.mjEQ_JOINT: ratio[int(m.jnt_qposadr[m.eq_obj1id[e]])] = float(m.eq_data[e][1]) * ratio[n]
    bodies = []
    for b in range(m.nbody):
        joints = []
        for j in range(m.njnt):
            if int(m.jnt_bodyid[j]) != b: continue
            i = int(m.jnt_qposadr[j]); axis = m.jnt_axis[j] * ratio.get(i, 1.)
            joints.append(dict(index=i, type='slide' if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_SLIDE else 'hinge', axis=[round(float(x), 6) for x in axis], pos=[round(float(x), 6) for x in m.jnt_pos[j]]))
        geoms = [dict(mesh=int(m.geom_dataid[g]), pos=[round(float(x), 6) for x in m.geom_pos[g]], quat=[round(float(x), 6) for x in m.geom_quat[g]])
                 for g in range(m.ngeom) if int(m.geom_bodyid[g]) == b and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
        bodies.append(dict(name=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b), parent=int(m.body_parentid[b]), pos=[round(float(x), 6) for x in m.body_pos[b]],
                           quat=[round(float(x), 6) for x in m.body_quat[b]], joints=joints, geoms=geoms))
    site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'grasp_site')
    travel = abs(opening) if jaw else float(m.jnt_range[n][1])
    return dict(body=entry['variant'], meshes=meshes, bodies=bodies, arm_joints=n, jaw_travel_m=round(travel, 5), gripper_calibrated=True,
                grasp_site=dict(body=mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.site_bodyid[site])), pos=[round(float(x), 6) for x in m.site_pos[site]]))


def display_model(root, profile, rig_spec):
    """robot-model.json of a catalog profile: what the 3D view draws (GET /v1/video-motion/models/<profile>). One arm's geometry, or one per side when they differ."""
    arms = rig_spec['arms']; models = root / 'models'
    sides = {side: _display_arm(models / arms[side]['model'], arms[side]) for side in ('left', 'right')}
    out = dict(sides['left'], source=f'roborun catalog profile {profile}', rig=dict(arms={s: dict(T_table_base=arms[s]['T_table_base'], variant=arms[s]['variant']) for s in arms}))
    if arms['left']['model'] != arms['right']['model']:
        out['arm_models'] = sides
    return out


def ensure(root, profile):
    """Puts a catalog profile, or a bundled profile standing on catalog models, into the install at `root` (the SDK's folder when no
    install is there yet). False when `profile` is neither."""
    named = parse(profile)
    needed = {m for m in named if m} if named is not None else set(bundled_models(profile))
    if not needed:
        return False
    from .robot import HOME, PROFILES
    root = Path(root).expanduser()
    if not (root / 'kin.py').exists():
        root = HOME
    for model in needed:
        target = root / 'models' / 'custom' / model
        if target.exists():
            shutil.rmtree(target)
        target.mkdir(parents=True)
        for f in (MODELS / model).rglob('*'):
            if f.is_file() and f.suffix in ('.xml', '.stl'):
                to = target / f.relative_to(MODELS / model); to.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(f, to)
    folder = root / 'profiles' / profile; folder.mkdir(parents=True, exist_ok=True)
    # a bundled profile is written byte for byte: motions are planned against exactly that rig file
    text = json.dumps(rig(profile), indent=1) if named is not None else (PROFILES / f'{profile}.json').read_text()
    if not (folder / 'rig.json').exists() or (folder / 'rig.json').read_text() != text:
        (folder / 'rig.json').write_text(text)
    base = root / 'profiles' / 'yam' / 'config.json'
    cfg = json.loads(base.read_text()) if base.exists() else dict(body='yam', station='http://127.0.0.1:8091')
    cfg.pop('profile_id', None); cfg['rig'] = f'profiles/{profile}/rig.json'
    (folder / 'config.json').write_text(json.dumps(cfg, indent=2))
    try:
        (folder / 'robot-model.json').write_text(json.dumps(display_model(root, profile, json.loads(text)), separators=(',', ':')))
    except ImportError:                       # MuJoCo arrives with the runtime's packages; the 3D view then draws no custom geometry until the next `up`
        pass
    return True
