"""roborun profiles create: a robot profile for YOUR arm, from its URDF.

    roborun profiles create --name my_arm --urdf my_arm.urdf \
        --joints j1,j2,j3,j4,j5,j6,j7 --fingers left_finger_joint,right_finger_joint --tcp-link grasp_link \
        --base-link base_link --first-link link1 --home 0,-0.8,0,-2.3,0,1.5,0.8 --arms 1 --place left:-0.35,0,0

The motion Manifest provides plans for arms of this shape: N joints (as many as --joints names; 6 or more for a free gripper
pose; more than 6 are held near --home by the solver), then a gripper, with a grasp point. The gripper is two prismatic fingers
moving together (--fingers left,right) or one revolute driver (--fingers driver --jaw-closed-rad A --jaw-open-rad B; joints the
model couples to it follow). --urdf also takes a MuJoCo MJCF file. This command converts it into the model the planner reads
(MuJoCo), names its parts as the planner expects (joint1..jointN, joint<N+1>/joint<N+2> for the fingers on bodies tip_left /
tip_right, the gripper body, the grasp_site, the base and first link), places the arm(s) on the table, checks the planner can use
it (forward / inverse kinematics, reach), and writes the profile into this robot's Roborun install:
    models/custom/<name>/<name>.xml (+ its meshes), profiles/<name>/rig.json, profiles/<name>/config.json.
Then: roborun up --profile <name>.
"""
from __future__ import annotations

import json
import math
import shutil
import sys
from pathlib import Path


MIN_JOINTS, MAX_JOINTS = 3, 32          # the planner's bounds


def _transform(x, y, z=0.0, yaw_deg=0.0):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return [[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def _place(text):
    """'left:x,y[,z[,yaw_deg]]' -> (slot, 4x4)."""
    slot, _, xyz = text.partition(':')
    v = [float(t) for t in xyz.split(',')]
    if slot not in ('left', 'right') or not 2 <= len(v) <= 4:
        raise SystemExit(f'--place {text!r}: left|right:x,y[,z[,yaw_deg]] (metres, degrees, table frame)')
    x, y, z, yaw = (v + [0.0, 0.0])[:4]
    return slot, _transform(x, y, z, yaw)


def convert(urdf, out_dir, name, joints, fingers, tcp_link, tcp_offset=(0, 0, 0), base_link=None, first_link=None, tcp_quat=None):
    """URDF or MJCF -> <out_dir>/<name>.xml with the planner's names. Returns the xml path."""
    import mujoco
    urdf = Path(urdf).resolve()
    spec = mujoco.MjSpec.from_file(str(urdf))
    spec.compiler.fusestatic = False                    # keep fixed links (the grasp link is usually one)
    spec.compiler.discardvisual = False
    spec.option.integrator = 0                          # the scene's own default: attaching the arm then has nothing to reconcile
    mesh_root = urdf.parent / (spec.meshdir or '') if urdf.suffix.lower() == '.xml' else urdf.parent
    spec.meshdir = str(mesh_root)
    for kind in (spec.keys, spec.actuators, spec.tendons, spec.sensors, spec.excludes, spec.pairs):   # the source file's poses, drives and
        for item in list(kind):                                            # contact lists: unused here, and they name what is renamed
            spec.delete(item)
    by_joint = {j.name: j for j in spec.joints}
    missing = [j for j in [*joints, *fingers] if j not in by_joint]
    if missing:
        raise SystemExit(f"not in {urdf.name}: {', '.join(missing)} (its joints: {', '.join(sorted(by_joint))})")
    n = len(joints)
    if not MIN_JOINTS <= n <= MAX_JOINTS or len(set(joints)) != n:
        raise SystemExit(f'--joints: the arm joints, base to wrist ({MIN_JOINTS} to {MAX_JOINTS} different joints; got {n})')
    if len(fingers) not in (1, 2):
        raise SystemExit('--fingers: the 2 finger joints (left, right), or the 1 joint that drives a revolute jaw')
    hinge = lambda j: by_joint[j].type == mujoco.mjtJoint.mjJNT_HINGE
    if not all(hinge(j) or by_joint[j].type == mujoco.mjtJoint.mjJNT_SLIDE for j in joints):
        raise SystemExit('--joints: every arm joint must be revolute or prismatic')
    if len(fingers) == 1 and not hinge(fingers[0]):
        raise SystemExit('--fingers: a single jaw joint must be revolute (two prismatic fingers are named as two joints)')
    bodies = {b.name: b for b in spec.bodies}
    for link in (tcp_link, base_link, first_link):
        if link and link not in bodies:
            raise SystemExit(f"link {link!r} is not in {urdf.name} (its links: {', '.join(sorted(k for k in bodies if k))})")
    jaw_names = (f'joint{n + 1}', f'joint{n + 2}')
    renamed = {j: f'joint{i + 1}' for i, j in enumerate(joints)}
    renamed.update({f: jaw_names[i] for i, f in enumerate(fingers)})
    taken = set(renamed.values()) - set(renamed)        # a source joint already called what another one becomes is renamed out of the way
    for j in by_joint:
        if j in taken and j not in renamed:
            raise SystemExit(f'{urdf.name}: joint {j!r} is neither an arm joint nor a finger but has the name the planner needs')
    for j, new in renamed.items():
        by_joint[j].name = '__roborun_' + new           # two steps: joint2 -> joint1 while joint1 -> joint2 never collide
    for j, new in renamed.items():
        by_joint[j].name = new
    for j, new_j, new_body in zip(fingers, jaw_names, ('tip_left', 'tip_right')):
        by_joint[j].parent.name = new_body
    if base_link: bodies[base_link].name = 'base'
    if first_link: bodies[first_link].name = 'link1'
    tcp = bodies[tcp_link]
    site_quat = list(map(float, tcp_quat)) if tcp_quat else [1, 0, 0, 0]
    tcp.add_site(name='grasp_site', pos=list(map(float, tcp_offset)), quat=site_quat, size=[0.004, 0, 0])
    tcp.add_site(name='tcp_site', pos=list(map(float, tcp_offset)), quat=site_quat, size=[0.004, 0, 0])
    if not any(b.name == 'gripper' for b in spec.bodies) and tcp.name not in ('base', 'link1'):
        tcp.name = 'gripper'                            # the body the planner hangs a held object on
    coupled = False
    for eq in spec.equalities:                          # the URDF's own constraints (mimic joints) follow the renames
        eq.name1, eq.name2 = renamed.get(eq.name1, eq.name1), renamed.get(eq.name2, eq.name2)
        coupled |= eq.type == mujoco.mjtEq.mjEQ_JOINT and {eq.name1, eq.name2} == set(jaw_names)
    if not coupled and len(fingers) == 2:               # the fingers move together
        eq = spec.add_equality()
        eq.type = mujoco.mjtEq.mjEQ_JOINT; eq.name1, eq.name2 = jaw_names; eq.data[:5] = [0, 1, 0, 0, 0]
    model = spec.compile()
    for i in range(n):                                  # the planner reads the arm joints as the first coordinates, in order
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'joint{i + 1}')
        if int(model.jnt_qposadr[jid]) != i:
            raise SystemExit(f'--joints: {joints[i]} is not coordinate {i + 1} of the model: list the joints base to wrist along one chain')
    out_dir = Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True)
    # the meshes next to the model, the model pointing at them
    meshes = out_dir / 'meshes'; meshes.mkdir(exist_ok=True)
    for m in spec.meshes:
        src = Path(m.file) if Path(m.file).is_absolute() else mesh_root / m.file
        if src.exists():
            shutil.copy2(src, meshes / src.name); m.file = src.name
    spec.meshdir = str(meshes.resolve())                # saving recompiles: from here, then the file points next to itself
    xml = out_dir / f'{name}.xml'
    xml.write_text(spec.to_xml().replace(f'meshdir="{spec.meshdir}"', 'meshdir="meshes"'))
    return xml


def _spec_of(joints, jaw=None, visual_group=None, home=None):
    """The rig fields that describe this arm to the planner (absent for what the planner assumes: six joints, two fingers)."""
    out = {}
    if joints != 6: out['joints'] = joints
    if jaw: out['jaw'] = jaw
    if visual_group is not None: out['visual_geom_group'] = visual_group
    if home is not None: out['home'] = home
    return out


def visual_group(xml):
    """The geom group that is only drawn, when the model has separate drawn and collision geometry (MuJoCo's group 2 convention)."""
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(xml))
    drawn = [g for g in range(m.ngeom) if int(m.geom_group[g]) == 2 and not m.geom_contype[g] and not m.geom_conaffinity[g]]
    return 2 if drawn and len(drawn) < m.ngeom else None


def check_model(xml, home, n=6, jaw=None):
    """Without a Roborun install: MuJoCo alone checks the converted arm (the planner's own check runs at registration)."""
    import mujoco
    import numpy as np
    m = mujoco.MjModel.from_xml_path(str(xml)); d = mujoco.MjData(m)
    site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'grasp_site')
    names = [m.joint(i).name for i in range(m.njnt)]
    jaw_joint = (jaw or {}).get('joint') or f'joint{n + 1}'
    need = [f'joint{i + 1}' for i in range(n)] + ([jaw_joint] if jaw else [f'joint{n + 1}', f'joint{n + 2}'])
    missing = [k for k in need if k not in names]
    if site < 0 or missing:
        raise SystemExit(f"the converted model lacks {', '.join(missing + ([] if site >= 0 else ['grasp_site']))}")
    limits = np.array(m.jnt_range[:n], float)
    rng = np.random.default_rng(0); base = None; reach = 0.0
    for q in rng.uniform(limits[:, 0], limits[:, 1], (2000, n)):
        d.qpos[:n] = q; mujoco.mj_kinematics(m, d)
        if base is None:
            body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'link1')
            base = d.xpos[body].copy() if body >= 0 else np.zeros(3)
        reach = max(reach, float(np.linalg.norm(d.site_xpos[site] - base)))
    j7 = names.index(jaw_joint)
    q = np.asarray(home if home else np.clip(np.zeros(n), limits[:, 0], limits[:, 1]), float)
    return dict(joints=int(m.nq), arm_joints=n, reach_m=round(reach, 3), limits=limits.round(3).tolist(), ik_error_mm=None,
                jaw_stroke_m=round(float(m.jnt_range[j7][1] - m.jnt_range[j7][0]), 4), home=q.tolist(),
                note='MuJoCo check only (no Roborun install here); the planner checks it when the robot registers')


def check(manifest_root, xml, home, n=6, jaw=None, visual=None):
    """The planner's own model loader on the converted arm: joints, limits, reach, FK -> IK round trip at home.
    Without a Roborun install on this machine, the MuJoCo-only check."""
    import numpy as np
    if not manifest_root or not (Path(manifest_root) / 'kin.py').exists():
        return check_model(xml, home, n, jaw)
    sys.path.insert(0, str(manifest_root))
    from kin import Arm
    arm = Arm(xml, np.eye(4), 'left', spec=_spec_of(n, jaw, visual, home))
    q = np.asarray(home if home else np.clip(np.zeros(n), arm.limits[:, 0], arm.limits[:, 1]), float)
    T = arm.fk(q)
    ok, q2 = arm.ik(T, q + 0.05)
    err = float(np.abs(arm.fk(q2)[:3, 3] - T[:3, 3]).max())
    if not ok or err > 0.002:
        raise SystemExit(f'the planner cannot solve this arm: IK from 0.05 rad away ended {err * 1000:.1f} mm off (converged {ok}); '
                         'check --joints order, --tcp-link and the joint limits')
    jaw_joint = (jaw or {}).get('joint') or f'joint{n + 1}'
    j7 = [i for i in range(arm.model.njnt) if arm.model.joint(i).name == jaw_joint]
    stroke = arm.model.jnt_range[j7[0]] if j7 else (float('nan'), float('nan'))
    return dict(joints=int(arm.model.nq), arm_joints=n, reach_m=round(float(arm.reach_m), 3), limits=arm.limits.round(3).tolist(),
                ik_error_mm=round(err * 1000, 2), jaw_stroke_m=round(float(stroke[1] - stroke[0]), 4), home=q.tolist())


def create(a):
    root = Path(a.manifest).resolve() if a.manifest else None
    installed = bool(root) and (root / 'profiles' / a.base_profile / 'config.json').exists()
    if not installed:
        # no Roborun install here: the bundled profiles give the table and cameras; the profile goes in the SDK's folder
        from roborun_sdk.robot import HOME, PROFILES
        if not (PROFILES / f'{a.base_profile}.json').exists():
            raise SystemExit(f"--base-profile {a.base_profile!r}: not a bundled profile ({', '.join(sorted(p.stem for p in PROFILES.glob('*.json')))})")
        root = HOME
    base_profile = root / 'profiles' / a.base_profile
    joints = [s.strip() for s in a.joints.split(',')]; fingers = [s.strip() for s in a.fingers.split(',')]
    offset = [float(v) for v in a.tcp_offset.split(',')] if a.tcp_offset else [0, 0, 0]
    quat = [float(v) for v in a.tcp_quat.split(',')] if getattr(a, 'tcp_quat', None) else None
    if quat is not None and len(quat) != 4:
        raise SystemExit('--tcp-quat: w,x,y,z')
    n = len(joints)
    home = [float(v) for v in a.home.split(',')] if a.home else None
    if home is not None and len(home) != n:
        raise SystemExit(f'--home: {n} joint angles, one per --joints entry (got {len(home)})')
    jaw = None
    if len(fingers) == 1:
        if getattr(a, 'jaw_closed_rad', None) is None or getattr(a, 'jaw_open_rad', None) is None:
            raise SystemExit('a revolute jaw (one --fingers joint) needs --jaw-closed-rad and --jaw-open-rad')
        jaw = dict(type='revolute', joint=f'joint{n + 1}', closed_rad=float(a.jaw_closed_rad), open_rad=float(a.jaw_open_rad))
    out = root / 'models' / 'custom' / a.name
    xml = convert(a.urdf, out, a.name, joints, fingers, a.tcp_link, offset, a.base_link, a.first_link, quat)
    visual = visual_group(xml)
    report = check(root if installed else None, xml, home, n, jaw, visual)
    described = _spec_of(n, jaw, visual)
    if n != 6 and home: described['rest_rad'] = home
    if getattr(a, 'jaw_gap_m', None): described['jaw_gap_m'] = float(a.jaw_gap_m)
    # the profile: the table, cameras and obstacles of --base-profile, the arms replaced by yours
    if installed:
        base_cfg = json.loads((base_profile / 'config.json').read_text())
        base_rig = json.loads((root / base_cfg['rig']).read_text())
    else:
        from roborun_sdk.robot import PROFILES
        base_cfg = dict(body='yam', station='http://127.0.0.1:8091')
        base_rig = json.loads((PROFILES / f'{a.base_profile}.json').read_text())
    places = dict(_place(p) for p in a.place) if a.place else {}
    slots = ['left'] if a.arms == 1 else ['left', 'right']
    arms = {}
    for slot in ('left', 'right'):
        T = places.get(slot) or (base_rig.get('arms', {}).get(slot) or {}).get('T_table_base') or _transform(0, 0)
        if slot not in slots and slot not in places:
            T = _transform(3.0, 0.0)                  # one arm: the empty slot's placeholder sits off the table, never in the way
        entry = dict(variant=a.name, gripper=f'{a.name}_fingers', model=f'custom/{a.name}/{a.name}.xml', T_table_base=T,
                     home=report['home'], **described)
        if slot not in slots:                             # one arm: the other slot is your arm, parked, never commanded
            entry['parked'] = dict(positions_rad=report['home'], gripper=1.0, source='roborun profiles create', never_commanded=True)
        arms[slot] = entry
    rig = {k: v for k, v in base_rig.items() if k not in ('arms', 'description', 'source')}
    rig.update(arms=arms, description=f'{a.name} from {Path(a.urdf).name} (roborun profiles create)', source='roborun profiles create',
               nominal=True)
    pdir = root / 'profiles' / a.name; pdir.mkdir(parents=True, exist_ok=True)
    (pdir / 'rig.json').write_text(json.dumps(rig, indent=1))
    cfg = dict(base_cfg, rig=f'profiles/{a.name}/rig.json'); cfg.pop('profile_id', None)
    (pdir / 'config.json').write_text(json.dumps(cfg, indent=2))
    (pdir / 'arm.json').write_text(json.dumps(dict(name=a.name, urdf=str(Path(a.urdf).resolve()), joints=joints, fingers=fingers,
                                                   tcp_link=a.tcp_link, tcp_offset=offset, arms=a.arms, check=report), indent=1))
    print(f"profile {a.name}: {a.arms} arm(s), model {xml}")
    if not installed:
        print(f'  written to {pdir} (no Roborun install on this machine; roborun up copies it into the install)')
    ik = f"IK check {report['ik_error_mm']} mm" if report.get('ik_error_mm') is not None else report.get('note', '')
    print(f"  reach {report['reach_m']} m, jaw stroke {report['jaw_stroke_m'] * 1000:.1f} mm, {ik}")
    print(f'  next: try it in simulation: roborun up --profile {a.name} --sim cup; for your hardware, write its driver on StreamedArm '
          f'(roborun new my-robot), then: roborun up --profile {a.name}')
    return 0


def add_parser(sub):
    p = sub.add_parser('profile', help='robot profiles for your own arms')
    ps = p.add_subparsers(dest='profile_cmd', required=True)
    c = ps.add_parser('create', help='a profile from your arm\'s URDF')
    c.add_argument('--name', required=True); c.add_argument('--urdf', required=True)
    c.add_argument('--joints', required=True, help='the arm joints, base to wrist (comma separated; as many as the arm has)')
    c.add_argument('--fingers', required=True, help='the 2 finger joints: left,right; or the 1 joint driving a revolute jaw')
    c.add_argument('--jaw-closed-rad', type=float, help='revolute jaw: the driver angle when closed')
    c.add_argument('--jaw-open-rad', type=float, help='revolute jaw: the driver angle when fully open')
    c.add_argument('--jaw-gap-m', type=float, help='the fingertip gap when fully open (default: the two fingers\' strokes added)')
    c.add_argument('--tcp-link', required=True, help='the link at the grasp point (between the fingertips)')
    c.add_argument('--tcp-offset', help='x,y,z from that link to the grasp point (metres)')
    c.add_argument('--tcp-quat', help='w,x,y,z of the grasp frame in that link: +z points out along the fingers, +y is the closing axis')
    c.add_argument('--base-link', help='the link bolted to the table'); c.add_argument('--first-link', help='the first moving link')
    c.add_argument('--home', help='the joint angles (rad) the arm rests at, one per --joints entry'); c.add_argument('--arms', type=int, choices=(1, 2), default=1)
    c.add_argument('--place', action='append', help='left|right:x,y[,z[,yaw_deg]] where each arm base sits on the table')
    c.add_argument('--base-profile', default='yam', help="this install's profile whose table, cameras and obstacles to keep")
    c.add_argument('--runtime', dest='manifest', default=None, help="this robot's Roborun install (default: the one roborun up uses; none: the SDK's folder)")
    c.set_defaults(func=_run)


def _run(a):
    from .arm_profiles import RENAMED, RESERVED, TO_RUNTIME
    if a.name in RESERVED | set(TO_RUNTIME) | set(RENAMED) | {'yam_solo'}:
        raise SystemExit(f'{a.name} is one of Roborun\'s own profiles: give yours another --name')
    if not a.manifest:
        from roborun_sdk.robot import CONFIG, _manifest_root
        a.manifest = json.loads(CONFIG.read_text()).get('manifest') if CONFIG.exists() else _manifest_root()
    if a.manifest and not Path(a.manifest).exists():
        a.manifest = None
    return create(a)
