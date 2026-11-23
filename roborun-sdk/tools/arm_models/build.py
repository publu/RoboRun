#!/usr/bin/env python3
"""Builds data/arm_models/<id>/ from each maker's public robot model (see sources.py): one small MuJoCo file per arm, with the SDK's joint and body names.

    python tools/arm_models/build.py --src <folder with the checked-out sources> --kin ~/roborun/robot/runtime [--only so101,ur5e] [--dump so101]

Needs the robot runtime (its kin.py, given with --kin), which `roborun up` downloads with a Roborun key.

Works from the COMPILED source model, so an MJCF and a URDF go through the same steps:
  - the arm chain is the listed joints; each becomes body link<i> with joint<i>, posed as in the source at the reference pose (a joint's `zero` moves
    the model's zero to the angle the arm's driver calls zero, `flip` reverses a joint whose positive direction differs from the driver's);
  - each body's geometry (visual geoms if it has any, else its collision geoms; fixed children included) is merged into one mesh, simplified, written as binary STL;
  - fingers become tip_left / tip_right: two sliding fingers (0 = closed, stroke = open), linkage grippers included (their pads move as a parallel jaw), or one revolute jaw;
  - grasp_site: between the pads, +z along the fingers, +y on the closing axis; the sign is the one that lets the arm reach grasps around its home.
"""
import argparse
import hashlib
import json
import math
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE.parents[1] / 'src' / 'roborun_sdk' / 'data' / 'arm_models'
sys.path.insert(0, str(HERE))
J = mujoco.mjtJoint
HINGE, SLIDE = J.mjJNT_HINGE, J.mjJNT_SLIDE


def body_name(m, b): return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or f'body{b}'
def joint_id(m, name):
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
    if j < 0: raise SystemExit(f'no joint {name!r} (has: {[m.joint(i).name for i in range(m.njnt)]})')
    return j
def body_id(m, name):
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
    if b < 0: raise SystemExit(f'no body {name!r} (has: {[body_name(m, i) for i in range(m.nbody)]})')
    return b


def frame(d, b):
    T = np.eye(4); T[:3, :3] = d.xmat[b].reshape(3, 3); T[:3, 3] = d.xpos[b]; return T


def quat_of(R):
    q = np.zeros(4); mujoco.mju_mat2Quat(q, np.asarray(R).reshape(9)); return q


def geom_mesh(m, g):
    """(vertices in the geom frame, faces) of a mesh or primitive geom, or None for planes and the like."""
    t = m.geom_type[g]; s = m.geom_size[g]
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        k = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k] + m.mesh_vertnum[k]].astype(float)
        f = m.mesh_face[m.mesh_faceadr[k]:m.mesh_faceadr[k] + m.mesh_facenum[k]].copy()
        return v, f
    import trimesh
    if t == mujoco.mjtGeom.mjGEOM_BOX: b = trimesh.creation.box(extents=2 * s[:3])
    elif t == mujoco.mjtGeom.mjGEOM_SPHERE: b = trimesh.creation.icosphere(subdivisions=1, radius=s[0])
    elif t == mujoco.mjtGeom.mjGEOM_CYLINDER: b = trimesh.creation.cylinder(radius=s[0], height=2 * s[1], sections=12)
    elif t == mujoco.mjtGeom.mjGEOM_CAPSULE: b = trimesh.creation.capsule(radius=s[0], height=2 * s[1], count=[8, 8]); b.apply_translation([0, 0, -s[1]])
    elif t == mujoco.mjtGeom.mjGEOM_ELLIPSOID: b = trimesh.creation.icosphere(subdivisions=1, radius=1.); b.apply_scale(s[:3])
    else: return None
    return np.asarray(b.vertices, float), np.asarray(b.faces, int)


def body_geoms(m, b):
    """A body's drawn geoms if it has any, else the ones that collide (MuJoCo convention: group 2 drawn, 3 collision)."""
    all_ = [g for g in range(m.ngeom) if m.geom_bodyid[g] == b and m.geom_type[g] != mujoco.mjtGeom.mjGEOM_PLANE]
    drawn = [g for g in all_ if m.geom_group[g] in (1, 2) and not (m.geom_contype[g] or m.geom_conaffinity[g])]
    return drawn or [g for g in all_ if m.geom_group[g] != 4] or all_


def gather(m, d, bodies, T_frame):
    """All geometry of `bodies` at the current pose as (V, F) in frame T_frame."""
    Vs, Fs, n = [], [], 0; inv = np.linalg.inv(T_frame)
    for b in bodies:
        for g in body_geoms(m, b):
            got = geom_mesh(m, g)
            if got is None: continue
            v, f = got; R = d.geom_xmat[g].reshape(3, 3); p = d.geom_xpos[g]
            w = v @ R.T + p; Vs.append(w @ inv[:3, :3].T + inv[:3, 3]); Fs.append(f + n); n += len(v)
    return (np.vstack(Vs), np.vstack(Fs)) if Vs else (np.zeros((0, 3)), np.zeros((0, 3), int))


def cluster(V, F, cell):
    key = np.floor(V / cell).astype(np.int64); _, inv, n = np.unique(key, axis=0, return_inverse=True, return_counts=True); inv = inv.reshape(-1)
    C = np.zeros((len(n), 3)); np.add.at(C, inv, V); C /= n[:, None]; G = inv[F]
    G = G[(G[:, 0] != G[:, 1]) & (G[:, 1] != G[:, 2]) & (G[:, 0] != G[:, 2])]
    return C, np.unique(G, axis=0)


def simplify(V, F, faces):
    """Vertex clustering on a grid just fine enough for the face budget: it keeps the outline (and so the convex hull the collision checks use) on any mesh, watertight or not."""
    if len(F) == 0: return V, F
    V, F = cluster(V, F, 1e-5)
    cell = 1e-3
    while len(F) > faces and cell < .05:
        V2, F2 = cluster(V, F, cell)
        if len(F2) < 8: break
        V, F = V2, F2; cell *= 1.3
    return V, F


def write_stl(path, V, F):
    tri = V[F].astype(np.float32); n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); l = np.linalg.norm(n, axis=1, keepdims=True); n = (n / np.maximum(l, 1e-12)).astype(np.float32)
    rec = np.zeros(len(F), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]); rec['n'] = n; rec['v'] = tri
    path.write_bytes(b'roborun arm model'.ljust(80, b' ') + struct.pack('<I', len(F)) + rec.tobytes())


def fmt(v, nd=6):
    return ' '.join(f'{float(x):.{nd}f}'.rstrip('0').rstrip('.') if abs(float(x)) > 1e-12 else '0' for x in np.ravel(v))


def children(m):
    out = {}
    for b in range(1, m.nbody): out.setdefault(int(m.body_parentid[b]), []).append(b)
    return out


def subtree(kids, b, stop=()):
    out, todo = [], [b]
    while todo:
        x = todo.pop()
        if x in stop: continue
        out.append(x); todo += kids.get(x, [])
    return out


def load(spec, src):
    from sources import resolve
    return resolve(spec, Path(src))


def build_side(side, a, src, out):
    """One arm model from its source. Returns the facts the profile needs."""
    spec, path = load(a, src); spec.compiler.fusestatic = False; spec.compiler.discardvisual = False
    for add in a.get('attach', []):                       # a gripper the source arm lacks, from its own source model
        import sources; add_spec, _ = sources.resolve(add, Path(src))
        add_spec.compiler.fusestatic = False
        for kind in (add_spec.keys, add_spec.actuators, add_spec.tendons, add_spec.sensors):
            for item in list(kind): add_spec.delete(item)
        for kind in (spec.keys, spec.actuators, spec.tendons, spec.sensors, spec.excludes):
            for item in list(kind): spec.delete(item)
        host = spec.body(add['to'])
        fr = host.add_frame(pos=add.get('pos', [0, 0, 0]), quat=add.get('quat', [1, 0, 0, 0])); spec.attach(add_spec, prefix=add.get('prefix', 'g_'), frame=fr)
    for kind in (spec.keys, spec.actuators, spec.tendons, spec.sensors, spec.excludes, spec.pairs):
        for item in list(kind): spec.delete(item)
    m = spec.compile(); d = mujoco.MjData(m); kids = children(m)
    names = a['joints']; n = len(names); ids = [joint_id(m, j) for j in names]
    flip = [1. if j not in a.get('flip', ()) else -1. for j in names]
    zero = [float(a.get('zero', {}).get(j, 0.)) for j in names]
    q = np.array(m.qpos0, float); q[:] = 0
    for j, v in {**a.get('pose', {}), **dict(zip(names, zero))}.items(): q[m.jnt_qposadr[joint_id(m, j)]] = v
    g = a['gripper']; kind = g['type']
    if kind == 'slide' or kind == 'linkage':
        for j, v in (g.get('open_pose') or {}).items(): q[m.jnt_qposadr[joint_id(m, j)]] = v
    elif kind == 'revolute':
        q[m.jnt_qposadr[joint_id(m, g['joint'])]] = g['closed']
        if g.get('follower'): q[m.jnt_qposadr[joint_id(m, g['follower'])]] = g['closed'] * g.get('follower_ratio', -1.)
    d.qpos[:] = q; mujoco.mj_kinematics(m, d)

    chain = [int(m.jnt_bodyid[j]) for j in ids]
    for i in range(1, n):
        p = chain[i]
        while p and p != chain[i - 1]: p = int(m.body_parentid[p])
        if p != chain[i - 1]: raise SystemExit(f"{a['id']}: joint {names[i]} does not follow {names[i - 1]} along one chain")
    base_frame = frame(d, body_id(m, a['base_frame'])) if a.get('base_frame') else np.eye(4)
    fingers = [body_id(m, b) for b in g.get('left', [])], [body_id(m, b) for b in g.get('right', [])]
    if kind == 'revolute': fingers = ([int(m.jnt_bodyid[joint_id(m, g['joint'])])], [int(m.jnt_bodyid[joint_id(m, g['follower'])])] if g.get('follower') else [])
    finger_all = set()
    for f in fingers:
        for b in f: finger_all |= set(subtree(kids, b))

    last = chain[-1]; T_grip = frame(d, last)
    meshes = {}
    def part(key, bodies, T, budget):
        V, F = gather(m, d, bodies, T); V, F = simplify(V, F, budget)
        if len(F): meshes[key] = (V, F)
        return V
    stops = set(chain) | finger_all
    if a.get('base_bodies') is not None: base_bodies = [body_id(m, b) for b in a['base_bodies']]
    else:
        base_bodies = [b for b in subtree(kids, 0, stop={chain[0]}) if b != 0 or True]
    # the base frame: on the table under the first joint (the planner measures bearings and reach from it), the source's axes kept
    j1 = ids[0]; T1 = frame(d, chain[0]); anchor = (T1 @ np.r_[m.jnt_pos[j1], 1])[:3]; axis1 = T1[:3, :3] @ m.jnt_axis[j1]
    if a.get('recenter', True) and abs(base_frame[:3, 2] @ axis1) <= .99:         # a model built with another axis up: turned so its first joint stands vertical
        next_anchor = (frame(d, chain[1]) @ np.r_[m.jnt_pos[ids[1]], 1])[:3]; up = axis1 if axis1 @ (next_anchor - base_frame[:3, 3]) > 0 else -axis1
        v = np.cross(up, [0, 0, 1.]); c = float(up[2]); K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]); Q = np.eye(3) + K + K @ K / (1 + c) if c > -.999 else np.diag([1., -1., -1.])
        base_frame = base_frame.copy(); base_frame[:3, :3] = Q.T @ base_frame[:3, :3]
    upright = abs(base_frame[:3, 2] @ axis1) > .99 and a.get('recenter', True)
    if upright:
        c = np.linalg.inv(base_frame) @ np.r_[anchor, 1]; shift = np.array([c[0], c[1], 0.])
        base_frame = base_frame.copy(); base_frame[:3, 3] = (base_frame @ np.r_[shift, 1])[:3]
    V = part('base', [b for b in base_bodies if b not in finger_all], base_frame, a.get('faces', 500))
    if upright and len(V) and a.get('seat', True):        # ... and level with the bottom of the base
        lift = float(V[:, 2].min()); base_frame = base_frame.copy(); base_frame[:3, 3] = (base_frame @ np.r_[0, 0, lift, 1])[:3]
        part('base', [b for b in base_bodies if b not in finger_all], base_frame, a.get('faces', 500))
    for i in range(n):
        own = [b for b in subtree(kids, chain[i], stop=(stops - {chain[i]}))]
        if i == n - 1: continue
        part(f'link{i + 1}', own, frame(d, chain[i]), a.get('faces', 500))
    if a.get('capsules'):                                 # no meshes published: a capsule along each link, joint to joint
        import trimesh
        anchors = [(frame(d, c) @ np.r_[m.jnt_pos[j], 1])[:3] for c, j in zip(chain, ids)]
        for i in range(n - 1):
            Ti = frame(d, chain[i]); p0 = (np.linalg.inv(Ti) @ np.r_[anchors[i], 1])[:3]; p1 = (np.linalg.inv(Ti) @ np.r_[anchors[i + 1], 1])[:3]; L = np.linalg.norm(p1 - p0)
            if L < 1e-4: continue
            cap = trimesh.creation.capsule(radius=float(a['capsules']), height=float(L), count=[8, 8]); z = (p1 - p0) / L; x = np.cross([0, 1., 0], z); x = x / np.linalg.norm(x) if np.linalg.norm(x) > 1e-6 else np.array([1., 0, 0]); R = np.c_[x, np.cross(z, x), z]
            meshes[f'link{i + 1}'] = (np.asarray(cap.vertices) @ R.T + p0, np.asarray(cap.faces))
    hand = subtree(kids, last, stop=finger_all)
    if g.get('drop'): hand = [b for b in hand if body_name(m, b) not in g['drop'] and not any(body_name(m, p) in g['drop'] for p in ancestors(m, b))]
    part('gripper', hand, T_grip, a.get('faces', 500))

    # fingers: meshes at the source's open pose (slide / linkage) or closed pose (revolute), in the gripper frame
    tool = {}
    wrist_anchor = np.zeros(3)                            # the gripper frame's origin is the last joint's body
    inv_g = np.linalg.inv(T_grip)
    if kind in ('slide', 'linkage'):
        for key, bodies in (('tip_left', fingers[0]), ('tip_right', fingers[1])):
            allb = [x for b in bodies for x in subtree(kids, b)]
            tool[key] = part(key, allb, T_grip, a.get('finger_faces', 200))
        pad = lambda V, ax: V[(V @ ax) >= np.percentile(V @ ax, 70)]
        c = (tool['tip_left'].mean(0) + tool['tip_right'].mean(0)) / 2; ax = (c - wrist_anchor) / np.linalg.norm(c - wrist_anchor)
        pl, pr = pad(tool['tip_left'], ax).mean(0), pad(tool['tip_right'], ax).mean(0)
        y = pl - pr; gap_centres = float(np.linalg.norm(y)); y /= gap_centres
        centre = (pl + pr) / 2; z, y = roll_axis(m, ids[-1], centre - wrist_anchor, y)
        if g.get('stroke') is None:
            jn = g['joints']; s = [abs(float(np.ptp(m.jnt_range[joint_id(m, x)]))) for x in jn]
        else: s = [float(g['stroke'])] * 2
        finger_rows = [dict(body='tip_left', axis=y.tolist(), stroke=s[0]), dict(body='tip_right', axis=(-y).tolist(), stroke=s[1])]
        gap = float(s[0] + s[1]); revolute = None
    else:
        jid = joint_id(m, g['joint']); b = int(m.jnt_bodyid[jid]); bodies = subtree(kids, b); closed, opened = float(g['closed']), float(g['open'])
        T_b = frame(d, b); inv_b = np.linalg.inv(T_b)
        Vc = part('tip_left', bodies, T_b, a.get('finger_faces', 200))          # in the jaw's own frame, at the closed reference
        anchor = (T_b @ np.r_[m.jnt_pos[jid], 1])[:3]; axis = T_b[:3, :3] @ m.jnt_axis[jid]
        Vw = Vc @ T_b[:3, :3].T + T_b[:3, 3]; r = np.linalg.norm(np.cross(Vw - anchor, axis), axis=1)
        tip = Vw[r >= np.percentile(r, 80)].mean(0)                             # the jaw's far end, closed
        def turn(p, a_):
            R = np.empty(9); qq = np.empty(4); mujoco.mju_axisAngle2Quat(qq, axis, a_); mujoco.mju_quat2Mat(R, qq); return anchor + R.reshape(3, 3) @ (p - anchor)
        tip_open = turn(tip, opened - closed)
        tan = np.cross(axis, tip - anchor) * (1. if opened > closed else -1.); tan /= np.linalg.norm(tan)   # how the jaw leaves its closed pose
        gap = float(np.linalg.norm(tip_open - tip))
        pc = (inv_g @ np.r_[tip, 1])[:3]; y = inv_g[:3, :3] @ tan
        centre = pc + y * min(float(g.get('grasp_gap', .35 * gap)), .025)   # against the fixed jaw, a little open: where a held object's middle sits
        second = None
        if g.get('follower'):                             # two jaws moving together: the grasp is midway between their closed tips
            jf = joint_id(m, g['follower']); bf = int(m.jnt_bodyid[jf]); T_f = frame(d, bf); Vf = part('tip_right', subtree(kids, bf), T_f, a.get('finger_faces', 200))
            Vfw = Vf @ T_f[:3, :3].T + T_f[:3, 3]; af = (T_f @ np.r_[m.jnt_pos[jf], 1])[:3]; axf = T_f[:3, :3] @ m.jnt_axis[jf]
            rf = np.linalg.norm(np.cross(Vfw - af, axf), axis=1); tipf = Vfw[rf >= np.percentile(rf, 80)].mean(0); pf = (inv_g @ np.r_[tipf, 1])[:3]
            centre = (pc + pf) / 2; y = pc - pf if np.linalg.norm(pc - pf) > .004 else y; y = y / np.linalg.norm(y); gap *= 2.
            second = dict(rel=inv_g @ T_f, axis=m.jnt_axis[jf], pos=m.jnt_pos[jf], ratio=float(g.get('follower_ratio', -1.)))
        z, y = roll_axis(m, ids[-1], centre - wrist_anchor, y)
        rel = inv_g @ T_b; ax_local = m.jnt_axis[jid]; pos_local = m.jnt_pos[jid]
        finger_rows = []; revolute = dict(rel=rel, axis=ax_local, pos=pos_local, closed=closed, opened=opened, second=second)
    if g.get('tcp'):                                      # an explicit grasp point (gripper frame, source metres)
        centre = np.array(g['tcp'], float)
    if g.get('tcp_body'):                                 # the maker's own grasp frame, when the model has one
        centre = (inv_g @ np.r_[frame(d, body_id(m, g['tcp_body']))[:3, 3], 1])[:3]
    if g.get('tcp_shift'): centre = centre + z * float(g['tcp_shift'])
    limits = np.array([[lo, hi] for lo, hi in m.jnt_range[ids]], float)
    for i, j in enumerate(ids):
        if not m.jnt_limited[j] or hi_eq(limits[i]): limits[i] = a.get('free_range', [-2 * math.pi, 2 * math.pi])
    lim = []
    for i in range(n):
        lo, hi = (limits[i] - zero[i]) * flip[i]; lim.append(sorted([float(lo), float(hi)]))
    limits = np.array(a.get('limits_override') or lim, float)
    return dict(m=m, d=d, n=n, ids=ids, chain=chain, flip=flip, base_frame=base_frame, meshes=meshes, T_grip=T_grip, limits=limits, finger_rows=finger_rows, revolute=revolute, y=y, z=z, tcp=centre, gap=gap, kind=kind, names=names, path=path)


def roll_axis(m, last, toward, y):
    """The approach axis: the last joint's axis when that joint rolls the gripper (nearly along the fingers), else the direction to the pads; kept square to the closing axis."""
    t = toward / np.linalg.norm(toward); ax = m.jnt_axis[last] * (1. if m.jnt_axis[last] @ t >= 0 else -1.)
    if ax @ t > math.cos(math.radians(35)):           # the fingers lie along the roll axis: it is the approach axis exactly, the closing axis square to it
        y = y - ax * (y @ ax); return ax, y / np.linalg.norm(y)
    z = t - y * (t @ y); return z / np.linalg.norm(z), y


def ancestors(m, b):
    while b: b = int(m.body_parentid[b]); yield b


def hi_eq(r): return abs(r[1] - r[0]) < 1e-9


def emit(a, side, b, out, name):
    """Writes <name>.xml + meshes; y_sign picks the closing axis' sign."""
    m, d = b['m'], b['d']; n = b['n']; chain = b['chain']; meshdir = out / 'meshes'; meshdir.mkdir(parents=True, exist_ok=True)
    assets, used = [], set()
    def mesh_geom(key, indent):
        if key not in b['meshes']: return ''
        mname = f'{name}_{key}'
        if mname not in used:
            V, F = b['meshes'][key]; write_stl(meshdir / f'{mname}.stl', V, F); used.add(mname)
            assets.append(f'<mesh name="{mname}" file="{mname}.stl" inertia="shell"/>')
        return f'{indent}<geom type="mesh" mesh="{mname}" rgba="{a.get("rgba", ".72 .72 .75 1")}"/>\n'
    inertial = '<inertial pos="0 0 0" mass=".2" diaginertia=".001 .001 .001"/>'
    def rel(T_parent, T_child):
        R = np.linalg.inv(T_parent) @ T_child; return R[:3, 3], quat_of(R[:3, :3])
    body_frames = [frame(d, c) for c in chain]
    lines = [f'<mujoco model="{name}">', '  <compiler angle="radian" meshdir="meshes" autolimits="true" inertiafromgeom="false"/>']
    tree = []; ind = '    '
    base_T = b['base_frame']; tree.append(f'{ind}<body name="base">\n{ind}  {inertial}\n' + mesh_geom('base', ind + '  '))
    parent = base_T; depth = 2
    for i in range(n):
        T = body_frames[i]; p, q = rel(parent, T); ind2 = '  ' * (depth + 1)
        j = b['ids'][i]; axis = m.jnt_axis[j] * b['flip'][i]; t = 'hinge' if m.jnt_type[j] == HINGE else 'slide'; lo, hi = b['limits'][i]
        tree.append(f'{ind2[:-2]}<body name="link{i + 1}" pos="{fmt(p)}" quat="{fmt(q)}">\n{ind2}{inertial}\n'
                    f'{ind2}<joint name="joint{i + 1}" type="{t}" axis="{fmt(axis)}" pos="{fmt(m.jnt_pos[j])}" range="{fmt([lo, hi])}"/>\n' + (mesh_geom(f'link{i + 1}', ind2) if i < n - 1 else ''))
        parent = T; depth += 1
    ind2 = '  ' * (depth + 1)
    y = b['y'] * a.get('_y_sign', 1.); z = b['z']; x = np.cross(y, z); R = np.c_[x, y, z]; q = quat_of(R)
    tree.append(f'{ind2[:-2]}<body name="gripper">\n{ind2}{inertial}\n' + mesh_geom('gripper', ind2) +
                f'{ind2}<site name="grasp_site" pos="{fmt(b["tcp"])}" quat="{fmt(q)}" size=".004"/>\n{ind2}<site name="tcp_site" pos="{fmt(b["tcp"])}" quat="{fmt(q)}" size=".004"/>\n')
    jn = n
    if b['kind'] in ('slide', 'linkage'):
        for row in b['finger_rows']:
            jn += 1; ax = np.array(row['axis']); s = row['stroke']; key = row['body']
            tree.append(f'{ind2}<body name="{key}" pos="{fmt(-ax * s)}">\n{ind2}  {inertial}\n{ind2}  <joint name="joint{jn}" type="slide" axis="{fmt(ax)}" range="0 {s:.5f}"/>\n'
                        + mesh_geom(key, ind2 + '  ') + f'{ind2}</body>\n')
    else:
        r = b['revolute']; p, q = r['rel'][:3, 3], quat_of(r['rel'][:3, :3]); jn += 1
        tree.append(f'{ind2}<body name="tip_left" pos="{fmt(p)}" quat="{fmt(q)}">\n{ind2}  {inertial}\n'
                    f'{ind2}  <joint name="joint{jn}" type="hinge" axis="{fmt(r["axis"])}" pos="{fmt(r["pos"])}" range="{fmt(sorted([0, r["opened"] - r["closed"]]))}"/>\n'
                    + mesh_geom('tip_left', ind2 + '  ') + f'{ind2}</body>\n')
        if r.get('second'):
            f = r['second']; p, q = f['rel'][:3, 3], quat_of(f['rel'][:3, :3]); jn += 1; lo, hi = sorted([0., (r['opened'] - r['closed']) * f['ratio']])
            tree.append(f'{ind2}<body name="tip_right" pos="{fmt(p)}" quat="{fmt(q)}">\n{ind2}  {inertial}\n'
                        f'{ind2}  <joint name="joint{jn}" type="hinge" axis="{fmt(f["axis"])}" pos="{fmt(f["pos"])}" range="{fmt([lo, hi])}"/>\n'
                        + mesh_geom('tip_right', ind2 + '  ') + f'{ind2}</body>\n')
    tree.append(f'{ind2[:-2]}</body>\n')
    for i in range(n):
        depth -= 1; tree.append('  ' * (depth + 1) + '</body>\n') 
    tree.append(f'{ind}</body>\n')
    lines += ['  <asset>', *[f'    {x}' for x in assets], '  </asset>', '  <worldbody>', ''.join(tree).rstrip('\n'), '  </worldbody>']
    if b['kind'] in ('slide', 'linkage'):
        lines += ['  <equality>', f'    <joint joint1="joint{n + 1}" joint2="joint{n + 2}" polycoef="0 1 0 0 0"/>', '  </equality>']
    elif (b['revolute'] or {}).get('second'):
        lines += ['  <equality>', f'    <joint joint1="joint{n + 2}" joint2="joint{n + 1}" polycoef="0 {b["revolute"]["second"]["ratio"]:g} 0 0 0"/>', '  </equality>']
    lines += ['</mujoco>', '']
    xml = out / f'{name}.xml'; xml.write_text('\n'.join(lines)); return xml


def check_same_kinematics(a, b, xml, tol=1e-6):
    """The written model must move exactly like the source: the last link's pose relative to the base, over random joint angles."""
    m, d = b['m'], b['d']; out = mujoco.MjModel.from_xml_path(str(xml)); od = mujoco.MjData(out); n = b['n']
    last = mujoco.mj_name2id(out, mujoco.mjtObj.mjOBJ_BODY, f'link{n}'); rng = np.random.default_rng(0); zero = [float(a.get('zero', {}).get(j, 0.)) for j in b['names']]; adr = [int(m.jnt_qposadr[j]) for j in b['ids']]; worst = 0.
    ref = d.qpos.copy()
    for _ in range(50):
        q = rng.uniform(b['limits'][:, 0], b['limits'][:, 1])
        d.qpos[:] = ref
        for i in range(n): d.qpos[adr[i]] = zero[i] + b['flip'][i] * q[i]
        mujoco.mj_kinematics(m, d); od.qpos[:] = 0; od.qpos[:n] = q; mujoco.mj_kinematics(out, od)
        src = np.linalg.inv(b['base_frame']) @ frame(d, b['chain'][-1]); got = np.eye(4); got[:3, :3] = od.xmat[last].reshape(3, 3); got[:3, 3] = od.xpos[last]
        worst = max(worst, float(np.abs(src - got).max()))
    d.qpos[:] = ref; mujoco.mj_kinematics(m, d)
    if worst > tol * 1000: raise SystemExit(f"{a['id']}: the written model does not move like its source ({worst:.4f} off)")
    return worst


def grasp_radius(arm, shoulder, forward, n=20000):
    """How far out from the shoulder (and how high) the demo object stands for this arm: the middle of the radii where random joint poses put the gripper level, jaw axis level, low over the table,
    ahead of the arm. The demo object goes there."""
    rng = np.random.default_rng(2); Q = rng.uniform(arm.limits[:, 0], arm.limits[:, 1], (n, arm.n_joints)); rs = []; zs = []
    for q in Q:
        T = arm.fk_base(q); z, y, p = T[:3, 2], T[:3, 1], T[:3, 3]
        if abs(z[2]) > .35 or abs(y[2]) > .3: continue
        v = p[:2] - shoulder[:2]; r = float(np.linalg.norm(v))
        if r < .05 or v @ forward < r * .75 or z[:2] @ (v / r) < .75: continue
        rs.append(r); zs.append(float(p[2]))
    if len(rs) < 20: return round(.45 * arm.reach_m, 3), None
    rs = np.array(rs); zs = np.array(zs); low = zs <= np.percentile(zs, 40); work = float(arm.reach_m)
    return round(float(np.clip(np.percentile(rs[low], 65), .48 * work, .6 * work)), 3), round(float(np.median(zs[low])), 3)


def lowest(arm, base_z):
    m, d = arm.model, arm.data; lo = 9.
    for g in range(m.ngeom):
        if m.geom_dataid[g] < 0: continue
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
        if name in ('base', 'link1'): continue
        k = m.geom_dataid[g]; v = m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k] + m.mesh_vertnum[k]]
        lo = min(lo, float((v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])[:, 2].min()) + base_z)
    return lo


def auto_home(arm, base_z, facing, n=6000):
    """A resting pose: the gripper ahead of the arm, about level, well inside the joint limits, every link clear of the table."""
    rng = np.random.default_rng(3); lim = arm.limits; mid = lim.mean(1); half = (lim[:, 1] - lim[:, 0]) / 2; best = None
    arm.data.qpos[:] = 0; mujoco.mj_kinematics(arm.model, arm.data); sh = arm.data.xanchor[0].copy()
    for _ in range(n):
        q = mid + rng.uniform(-.85, .85, arm.n_joints) * half
        T = arm.fk_base(q); p, z = T[:3, 3], T[:3, 2]; v = p[:2] - sh[:2]; r = float(np.linalg.norm(v))
        if r < .15 * arm.reach_m or (facing is not None and v @ facing < .8 * r) or p[2] + base_z < .1 or abs(z[2]) > .35: continue
        if facing is None and z[:2] @ (v / r) < .7: continue
        if facing is not None and z[:2] @ facing < .6: continue
        low = lowest(arm, base_z)
        if low < .05: continue
        score = -float(np.mean(((q - mid) / half) ** 2)) - abs(r - .45 * arm.reach_m) - abs(p[2] + base_z - .3)
        if best is None or score > best[0]: best = (score, q.copy())
    if best is None: raise SystemExit(f'{arm.xml_path.name}: no clear resting pose found')
    return np.round(best[1], 4)


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def choose_tool_sign(a, b, out, name):
    """The closing axis' sign (the jaw is symmetric, the wrist's travel is not): the one with which the arm reaches more test grasps around its home, nearer its home pose."""
    sys.path.insert(0, str(KIN))
    import kin
    best = None; jaw = None if b['revolute'] is None else dict(type='revolute', joint=f"joint{b['n'] + 1}", closed_rad=0., open_rad=b['revolute']['opened'] - b['revolute']['closed'])
    for s in (1., -1.):
        a['_y_sign'] = s; xml = emit(a, 'x', b, out, name)
        arm = kin.Arm(xml, np.eye(4), 'left', spec=dict(joints=b['n'], jaw=jaw, home=a['home']))
        q0 = np.asarray(a['home'], float); h = arm.fk(q0)[:2, 3]; h = np.asarray(a.get('forward', (1., 0.)), float) if a.get('shared_base') or np.linalg.norm(h) < .05 else h / np.linalg.norm(h)
        arm.data.qpos[:] = 0; arm.data.qpos[:b['n']] = q0; mujoco.mj_kinematics(arm.model, arm.data); sh = arm.data.xanchor[0].copy()
        r0, z0 = grasp_radius(arm, sh, h)
        Rz = np.array([[h[0], -h[1], 0], [h[1], h[0], 0], [0, 0, 1.]]); ok = 0; cost = 0.
        for r in (.8 * r0, r0, 1.2 * r0):
            for pitch in (0., 20., 45.):
                pr = np.radians(pitch); f = np.array([np.cos(pr), 0, -np.sin(pr)]); yy = np.array([0., 1., 0.]); xx = np.cross(yy, f)
                p = np.r_[sh[:2], 0.] + Rz @ np.array([r, 0., max(.05, min(.25, z0 if z0 is not None else .12))]); T = np.eye(4); T[:3, :3] = Rz @ np.c_[xx, yy, f]; T[:3, 3] = p
                conv, q = arm.ik(T, q0, iters=250); F = arm.fk(q); err = np.linalg.norm(F[:3, 3] - p)
                turn = np.degrees(np.arccos(np.clip((np.trace(F[:3, :3].T @ T[:3, :3]) - 1) / 2, -1, 1)))
                if err < .004 and turn < 6: ok += 1; cost += float(np.abs(q - q0).sum())
        if best is None or (ok, -cost) > (best[0], -best[1]): best = (ok, cost, s)
    a['_y_sign'] = best[2]; return best


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--src', required=True); ap.add_argument('--only'); ap.add_argument('--dump'); ap.add_argument('--out', default=str(OUT)); ap.add_argument('--kin', required=True)
    args = ap.parse_args(); global KIN; KIN = Path(args.kin)
    import sources; arms = sources.ARMS; out_root = Path(args.out)
    if args.dump:
        a = arms[args.dump]; spec, p = sources.resolve(a, Path(args.src)); spec.compiler.fusestatic = False; m = spec.compile(); kids = children(m)
        def show(b, ind=0):
            js = [m.joint(j).name + ('*' if m.jnt_type[j] == HINGE else '=') for j in range(m.njnt) if m.jnt_bodyid[j] == b]
            print(' ' * ind + body_name(m, b), js, f'geoms {len(body_geoms(m, b))}/{sum(m.geom_bodyid == b)}', np.round(m.body_pos[b], 3).tolist())
            for k in kids.get(b, []): show(k, ind + 1)
        show(0); return
    ids = args.only.split(',') if args.only else sorted(arms)
    catalog_dir = HERE.parents[1] / 'src' / 'roborun_sdk' / 'data' / 'arms'
    for aid in ids:
        try: one(aid, arms, out_root, catalog_dir, args, sources)
        except SystemExit as exc: print('FAILED', aid, str(exc)[:300], flush=True)


def notice_comment(info):
    """The first line of every generated model: where it came from, under which license, the source's own copyright notice
    (when it has one; Apache-2.0 section 4(c)), and that it was modified."""
    src = info['source']; grip = info.get('attached_gripper')
    text = (f"Generated by roborun-sdk tools/arm_models/build.py from {src['repo']} @ {src['commit']}: {src['file']} "
            f"({src['license']}, see LICENSE.txt). "
            + (f"Upstream notice: {src['copyright']}, licensed under {src['license']}. " if src.get('copyright') else '')
            + "Modified: converted to MJCF, meshes converted to STL and simplified, "
            f"joints and frames renamed" + (f"; gripper attached: {grip} (see LICENSE.gripper.txt)" if grip else '') + '.')
    return '<!-- ' + text.replace('--', '- -') + ' -->'


def one(aid, arms, out_root, catalog_dir, args, sources):
    a = dict(arms[aid], id=aid); d = out_root / aid
    for f in list(d.glob('*.xml')) + list((d / 'meshes').glob('*.stl')) + list(d.glob('*.json')) + list(d.glob('LICENSE*')): f.unlink()
    d.mkdir(parents=True, exist_ok=True)
    row = json.loads((catalog_dir / f"{a.get('catalog', aid)}.json").read_text()); sides = {}; notes = []
    for side, over in a.get('sides', {'left': {}}).items():
        s = dict(a, **over); name = aid if side == 'left' else f'{aid}_right'
        b = build_side(side, s, args.src, d)
        cat = (row.get('options') or {}).get('side_limits', {}).get(side) or row.get('limits')
        cat = (row.get('options') or {}).get('limits_by_side', {}).get(side) or cat
        if cat and len(cat) >= b['n'] and not s.get('ignore_catalog_limits'):             # the driver's limits win where they are tighter; differences are recorded
            for i in range(b['n']):
                lo, hi = cat[i]
                if hi <= lo: continue
                mlo, mhi = b['limits'][i]
                if abs(lo - mlo) > .02 or abs(hi - mhi) > .02: notes.append(f"{side} joint {i + 1}: model range [{mlo:.3f}, {mhi:.3f}], driver [{lo:.3f}, {hi:.3f}]")
                nlo, nhi = max(mlo, lo), min(mhi, hi)
                if nhi > nlo: b['limits'][i] = [nlo, nhi]
                else: notes.append(f'{side} joint {i + 1}: the ranges do not overlap: check its zero and direction')
        home = np.clip(np.asarray(s['home'], float), b['limits'][:, 0], b['limits'][:, 1]); s['home'] = home.tolist()
        rest = np.clip(np.asarray(s.get('rest', s['home']), float), b['limits'][:, 0], b['limits'][:, 1])
        got = choose_tool_sign(s, b, d, name); xml = emit(s, side, b, d, name)
        worst = check_same_kinematics(s, b, xml)
        if s.get('shoulder_height'):                               # a two-armed robot stands with its shoulders this far over the table
            sys.path.insert(0, str(KIN)); import kin as _k
            arm0 = _k.Arm(xml, np.eye(4), 'left', spec=dict(joints=b['n'], home=s['home'])); arm0.data.qpos[:] = 0; mujoco.mj_kinematics(arm0.model, arm0.data)
            s['base_z'] = a['base_z'] = float(s['shoulder_height']) - float(arm0.data.xanchor[0][2])
        if s.get('auto_home'):                                     # no usable resting pose in the sources: the best one found, ahead of the arm and clear of the table
            sys.path.insert(0, str(KIN)); import kin as _k
            jaw0 = None if b['revolute'] is None else dict(type='revolute', joint=f"joint{b['n'] + 1}", closed_rad=0., open_rad=b['revolute']['opened'] - b['revolute']['closed'])
            arm0 = _k.Arm(xml, np.eye(4), 'left', spec=dict(joints=b['n'], jaw=jaw0, home=s['home']))
            home = auto_home(arm0, float(s.get('base_z', 0.)), np.asarray(s.get('forward', (1., 0.)), float) if s.get('shared_base') else None); s['home'] = home.tolist()
            rest = home.copy(); got = choose_tool_sign(s, b, d, name); xml = emit(s, side, b, d, name)
        sys.path.insert(0, str(KIN)); import kin
        jaw = None if b['revolute'] is None else dict(type='revolute', joint=f"joint{b['n'] + 1}", closed_rad=0., open_rad=round(b['revolute']['opened'] - b['revolute']['closed'], 5))
        arm = kin.Arm(xml, np.eye(4), 'left', spec=dict(joints=b['n'], jaw=jaw, home=home.tolist()))
        q = rest.copy(); T = arm.fk(q)[:3, 3]; yaw = 90. - math.degrees(math.atan2(T[1], T[0])) if np.hypot(T[0], T[1]) > .05 else 90.
        T0 = arm.T; ids = range(b['n']); j1a = arm.fk(rest)[:3, 3] * 0.; mm = arm.model; dd = arm.data; dd.qpos[:b['n']] = rest; mujoco.mj_kinematics(mm, dd); sh = dd.xanchor[0].copy()     # the first joint's anchor: the arm's shoulder
        forward = np.asarray(s.get('forward', (1., 0.)), float) if s.get('shared_base') else (arm.fk(rest)[:2, 3] / max(np.linalg.norm(arm.fk(rest)[:2, 3]), 1e-6))
        obj_r, obj_z = grasp_radius(arm, sh, forward)
        obj_r = float(s.get('object_r', obj_r))
        rng = np.random.default_rng(1); tips = [arm.fk(rng.uniform(arm.limits[:, 0], arm.limits[:, 1]))[:3, 3] for _ in range(1500)]; work = float(max(np.linalg.norm(np.array(tips) - sh, axis=1)))
        sides[side] = dict(object_r=obj_r, object_z=obj_z, shoulder=[round(float(v), 4) for v in sh], work_m=round(work, 3), model=xml.name, joints=b['n'], home=[round(float(x), 5) for x in home], rest=[round(float(x), 5) for x in rest], jaw=jaw, jaw_gap_m=round(b['gap'], 4), limits=np.round(b['limits'], 5).tolist(),
                           source_joints=b['names'], flipped=[j for j, f in zip(b['names'], b['flip']) if f < 0], zero_offsets=s.get('zero', {}), grasps_solved=f'{got[0]}/9', reach_m=round(float(arm.reach_m), 3), yaw_deg=round(yaw, 2))
    left = sides['left']; gap = left['jaw_gap_m']; w = min(90, round(gap * 850), round(left['work_m'] * 170)); h = round(w * 1.25)   # the sim's demo cup: a cup this gripper closes on, sized to the arm (then run sim_cup_check.py: so100, so101, koch_v1_1, trossen_wxai and unitree_r1_a7 (left) carry a hand-tucked home, and koch_v1_1 a hand-set 64 x 100 mm cup: a rebuild writes these back over)
    repo, _, rel = a['source'].partition(':'); r = sources.REPOS[repo]
    info = dict(id=aid, label=row['label'], reach_m=left['work_m'], object_r_m=left['object_r'], shared_base=bool(a.get('shared_base')), attached_gripper=a.get('attached_gripper') or (a.get('sides', {}).get('left', {}) or {}).get('attached_gripper'), yaw_deg=left['yaw_deg'] if not a.get('shared_base') else round(90. - math.degrees(math.atan2(a.get('forward', (1., 0.))[1], a.get('forward', (1., 0.))[0])), 2), base_z_m=float(a.get('base_z', 0.)), sides=sides, sim_object_mm=[w, w, h],
                gripper=dict(a, **a.get('sides', {}).get('left', {}))['gripper']['type'], source=dict(repo=r['url'], commit=r['commit'], file=rel, license=a.get('license', r.get('license')), license_file=a.get('license_file'), **({'license_text': 'tools/arm_models/' + a['license_text']} if a.get('license_text') else {}), **({'copyright': a['copyright']} if a.get('copyright') else {}), sha256=sha(b['path'])), simplified=dict(faces_per_link=a.get('faces', 500)),
                notes=notes + list(a.get('notes', [])))
    (d / 'arm.json').write_text(json.dumps(info, indent=1))
    for x in d.glob('*.xml'): x.write_text(notice_comment(info) + '\n' + x.read_text())     # the change notice (Apache-2.0 4(b))
    import shutil                                     # the licenses the models are redistributed under
    root = Path(args.src) / repo
    if a.get('license_text'):                         # the license's own text, kept here, when the source only declares it (pr2: package.xml)
        shutil.copyfile(Path(__file__).resolve().parent / a['license_text'], d / 'LICENSE.txt')
    elif a.get('license_file') and (root / a['license_file']).is_file(): shutil.copyfile(root / a['license_file'], d / 'LICENSE.txt')
    for side in a.get('sides', {'left': {}}).values():
        for add in dict(a, **side).get('attach', []):
            extra = Path(args.src) / add['source'].split(':')[0] / add.get('license_file', '')
            if add.get('license_file') and extra.is_file(): shutil.copyfile(extra, d / 'LICENSE.gripper.txt')
    size = sum(f.stat().st_size for f in d.rglob('*') if f.is_file())
    print(aid, f'{size / 1e3:.0f} kB', {k: (v['joints'], v['jaw_gap_m'], v['grasps_solved'], v['reach_m']) for k, v in sides.items()}, notes[:3], flush=True)


if __name__ == '__main__':
    main()
