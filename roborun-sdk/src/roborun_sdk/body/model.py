"""The humanoid's display model: what the page's 3-D view draws (GET /v1/body/model). Built once from the profile's packaged
MuJoCo model (data/body_models/<id>/...), the same file the simulation loads, so a simulated body and a real one look the same.

    display_model('booster_k1', scene='room')   # scene: the simulated room's walls, table and boxes; None for a real body

The answer (schema roborun.body-model/1):

    bodies      the body tree, root first: {name, parent (index, -1 for the root), pos, quat (w x y z) in the parent's frame,
                joints [{name, axis, pos, q}] (hinges, q = the standing pose), geoms}. The root is the floating base: the page
                places it at the odometry pose (x, y, yaw) with its origin `base_height_m` above the floor; `height_m` is how tall
                the body stands.
    geoms       {kind: mesh|box|sphere|capsule|cylinder|ellipsoid, size (MuJoCo's: half sizes, radius, half length), pos, quat,
                rgba, mesh (index, for kind mesh)}
    meshes      [{vertices [[x, y, z]...], faces [[i, j, k]...]}]: the visual meshes, decimated to a face budget
    head        {pitch, yaw}: the joints /v1/body/state's head pitch and yaw drive (absent: a fixed head)
    camera      the head camera: {body, pos, quat} (MuJoCo camera axes: it looks along its -z, +y up), hfov_deg, aspect
    room        the simulated scene ({floor, walls, boxes: [{name, pos, size (half sizes), rgba}]}) or null (a real body)

Nothing here moves the robot or reads a camera; a model that cannot be built is an error the page shows, never a guess.
"""
from __future__ import annotations

import math
import threading

import numpy as np

from . import profiles

SCHEMA = 'roborun.body-model/1'
FACES = 40000                     # the face budget of all the meshes together (the page draws them on any laptop)
_KINDS = {2: 'sphere', 3: 'capsule', 4: 'ellipsoid', 5: 'cylinder', 6: 'box', 7: 'mesh'}
_cache, _lock = {}, threading.Lock()


def _round(a, n=5):
    return [round(float(v), n) for v in np.asarray(a).ravel()]


def decimate(vertices, faces, target):
    """Vertex clustering: merge the vertices of each cubic cell (its size found by bisection) until at most `target` faces stay."""
    vertices = np.asarray(vertices, float)
    faces = np.asarray(faces, np.int64)
    if len(faces) <= target:
        return vertices, faces

    def cluster(cell):
        keys = np.floor(vertices / cell).astype(np.int64)
        _, inverse = np.unique(keys, axis=0, return_inverse=True)
        inverse = inverse.reshape(-1)
        counts = np.bincount(inverse)
        merged = np.zeros((len(counts), 3))
        np.add.at(merged, inverse, vertices)
        merged /= counts[:, None]
        f = inverse[faces]
        f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
        if len(f):
            _, first = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
            f = f[np.sort(first)]
        used = np.unique(f)
        remap = np.full(len(merged), -1, np.int64)
        remap[used] = np.arange(len(used))
        return merged[used], remap[f]
    span = float(np.ptp(vertices, axis=0).max()) or 1.
    lo, hi = span * 1e-4, span / 4
    best = cluster(hi)
    for _ in range(30):
        mid = math.sqrt(lo * hi)
        v, f = cluster(mid)
        if len(f) > target:
            lo = mid
        else:
            hi, best = mid, (v, f)
        if hi / lo < 1.03:
            break
    return best


def _compile(profile):
    import mujoco
    sim = profile['sim']
    spec = mujoco.MjSpec.from_file(str(profiles.DATA / sim['model']))
    if sim.get('head_camera_frame'):          # the same head camera the simulation adds: optical frame, looking along its -z
        spec.body(sim['head_camera_frame']).add_camera(name='roborun_head', pos=[0, 0, 0], quat=[0, 1, 0, 0])
    return mujoco, spec.compile()


def _build(profile_id, scene):
    profile = profiles.load(profile_id)
    if not (profile.get('sim') or {}).get('model'):
        raise ValueError(f'{profile_id} has no 3-D model: add sim: model: <your MJCF file> to its profile')
    sim, robot, camera = profile['sim'], profile['robot'], profile['camera']
    mujoco, m = _compile(profile)
    d = mujoco.MjData(m)
    name = lambda kind, i: mujoco.mj_id2name(m, kind, i) or ''
    FREE, HINGE = int(mujoco.mjtJoint.mjJNT_FREE), int(mujoco.mjtJoint.mjJNT_HINGE)
    root = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, sim['base_body'])
    if root < 0:
        raise ValueError(f'{profile_id}: no body {sim["base_body"]!r} in its model')

    def under(b):
        while b > 0:
            if b == root:
                return True
            b = int(m.body_parentid[b])
        return False
    tree = [b for b in range(m.nbody) if under(b)]                       # parents come before their children in MuJoCo
    index = {b: i for i, b in enumerate(tree)}

    # the geoms a person sees: a model with separate visual meshes (group 1) shows those; otherwise every solid shape it has
    shapes = [g for g in range(m.ngeom) if under(int(m.geom_bodyid[g])) and int(m.geom_type[g]) in _KINDS]
    visual = [g for g in shapes if int(m.geom_group[g]) == 1 and int(m.geom_type[g]) == 7]
    shown = visual or [g for g in shapes if int(m.geom_group[g]) <= 2]

    def rgba(g):
        mat = int(m.geom_matid[g])
        return m.mat_rgba[mat] if mat >= 0 else m.geom_rgba[g]
    shown = [g for g in shown if float(rgba(g)[3]) > .05]

    used = sorted({int(m.geom_dataid[g]) for g in shown if int(m.geom_type[g]) == 7})
    sizes = {k: int(m.mesh_facenum[k]) for k in used}
    weight = sum(math.sqrt(s) for s in sizes.values()) or 1.
    meshes, mesh_index = [], {}
    for k in used:
        v = m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k] + m.mesh_vertnum[k]]
        f = m.mesh_face[m.mesh_faceadr[k]:m.mesh_faceadr[k] + m.mesh_facenum[k]]
        v, f = decimate(v, f, min(sizes[k], max(60, int(FACES * math.sqrt(sizes[k]) / weight))))
        mesh_index[k] = len(meshes)
        meshes.append(dict(vertices=np.round(v, 4).tolist(), faces=np.asarray(f).tolist()))

    # the standing pose, and the base height that puts the lowest point of the body on the floor
    stance = sim.get('stance') or {}
    d.qpos[:] = m.qpos0
    for j in range(m.njnt):
        if int(m.jnt_type[j]) == HINGE and name(mujoco.mjtObj.mjOBJ_JOINT, j) in stance:
            d.qpos[m.jnt_qposadr[j]] = float(stance[name(mujoco.mjtObj.mjOBJ_JOINT, j)])
    free = [j for j in range(m.njnt) if int(m.jnt_type[j]) == FREE and int(m.jnt_bodyid[j]) == root]
    if free:
        d.qpos[m.jnt_qposadr[free[0]]:m.jnt_qposadr[free[0]] + 7] = [0, 0, 1., 1, 0, 0, 0]
    mujoco.mj_kinematics(m, d)
    low, high = math.inf, -math.inf
    for g in shown:
        R, p = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
        if int(m.geom_type[g]) == 7:
            k = int(m.geom_dataid[g])
            v = m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k] + m.mesh_vertnum[k]]
            z = (v @ R.T + p)[:, 2]
            low, high = min(low, float(z.min())), max(high, float(z.max()))
        else:
            c, h = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
            for corner in np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3):
                z = float((p + R @ (c + h * corner))[2])
                low, high = min(low, z), max(high, z)
    base_height = float(d.xpos[root][2] - low)

    bodies = []
    for b in tree:
        joints = [dict(name=name(mujoco.mjtObj.mjOBJ_JOINT, j), axis=_round(m.jnt_axis[j], 6), pos=_round(m.jnt_pos[j], 6),
                       q=round(float(d.qpos[m.jnt_qposadr[j]]), 5))
                  for j in range(m.njnt) if int(m.jnt_bodyid[j]) == b and int(m.jnt_type[j]) == HINGE]
        geoms = []
        for g in shown:
            if int(m.geom_bodyid[g]) != b:
                continue
            kind = _KINDS[int(m.geom_type[g])]
            geom = dict(kind=kind, pos=_round(m.geom_pos[g]), quat=_round(m.geom_quat[g], 6), rgba=_round(rgba(g), 3))
            if kind == 'mesh':
                geom['mesh'] = mesh_index[int(m.geom_dataid[g])]
            else:
                geom['size'] = _round(m.geom_size[g])
            geoms.append(geom)
        is_root = b == root
        bodies.append(dict(name=name(mujoco.mjtObj.mjOBJ_BODY, b), parent=-1 if is_root else index[int(m.body_parentid[b])],
                           pos=[0., 0., 0.] if is_root else _round(m.body_pos[b]), quat=[1., 0., 0., 0.] if is_root else _round(m.body_quat[b], 6),
                           joints=joints, geoms=geoms))

    head = {}
    for role, joint in (sim.get('head_joints') or {}).items():
        if role in ('pitch', 'yaw') and mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint) >= 0:
            head[role] = joint
    cam_name = 'roborun_head' if sim.get('head_camera_frame') else sim.get('head_camera')
    cam = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cam_name) if cam_name else -1
    cam_out = None
    if cam >= 0 and int(m.cam_bodyid[cam]) in index:
        w, h = camera['size_px']
        cam_out = dict(body=index[int(m.cam_bodyid[cam])], pos=_round(m.cam_pos[cam]), quat=_round(m.cam_quat[cam], 6),
                       hfov_deg=float(camera['hfov_deg']), aspect=round(float(w) / float(h), 4))

    room = None
    if scene == 'room':
        from . import sim as body_sim
        x0, x1 = body_sim.ROOM_X; y0, y1 = body_sim.ROOM_Y; t, hgt = body_sim.WALL_T, body_sim.ROOM_H
        cx, cy, hx, hy = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2
        walls = [dict(pos=[cx, y1 + t, hgt / 2], size=[hx + t, t, hgt / 2]), dict(pos=[cx, y0 - t, hgt / 2], size=[hx + t, t, hgt / 2]),
                 dict(pos=[x1 + t, cy, hgt / 2], size=[t, hy, hgt / 2]), dict(pos=[x0 - t, cy, hgt / 2], size=[t, hy, hgt / 2])]
        (tx, ty), (sx, sy, th), trgba = body_sim.TABLE
        boxes = [dict(name='table', pos=[tx, ty, th / 2], size=[sx, sy, th / 2], rgba=list(trgba))]
        boxes += [dict(name=n, pos=[x, y, sz / 2], size=[bx / 2, by / 2, sz / 2], rgba=list(c)) for n, (x, y), (bx, by, sz), c in body_sim.BOXES]
        room = dict(floor=dict(x=[x0, x1], y=[y0, y1]), walls=walls, boxes=boxes)
    elif scene:
        room = dict(floor=None, walls=[], boxes=[])

    return dict(schema=SCHEMA, profile=profile_id, label=profile['label'], base=robot['base'], head_dof=robot['head_dof'],
                base_height_m=round(base_height, 4), height_m=round(high - low, 4), bodies=bodies, meshes=meshes, head=head, camera=cam_out, room=room,
                faces=sum(len(x['faces']) for x in meshes))


def display_model(profile_id, scene=None):
    """The display model of `profile_id` (built once per process; scene adds the simulated room)."""
    key = (profile_id, scene)
    with _lock:
        if key not in _cache:
            _cache[key] = _build(profile_id, scene)
        return _cache[key]
