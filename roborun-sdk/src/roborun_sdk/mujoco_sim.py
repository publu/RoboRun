"""roborun sim: this robot's profile as a MuJoCo simulation that the Roborun platform drives like real arms.

    roborun sim                       the profile the robot runs (e.g. two YAMs: roborun up --profile yam_dual)
    roborun sim --scene pickup-bench  one of the runtime's scenes (pickup-bench, mixed-objects, cluttered-bench, low-light);
                                       default: an orange cup in front of the left arm
    roborun up --sim cup              the whole robot as a simulation (its backend and the page drive it); runs this

A simulation never runs in the live robot (sim_home.py): bare `roborun sim` runs from its own copy of the runtime
(<Roborun home>/sim/), serves its own camera socket (sim-camera.sock beside the robot's camera.sock) and its own station port
(8191), and asks no backend for motions (--predict URL to give one). It refuses, in words, a port or camera socket that is
already served, and a --runtime that is the live robot's while the live robot runs; it removes a socket only when a
simulation made it. `roborun up --sim` runs it from the same copy, on the robot's own 8091 and camera.sock (the robot is the
simulation then), after checking that no live station or camera service has them.

What runs:
- the scene: the runtime's own rig model, built from the profile's rig (arms where the planner expects them, the overhead camera at
  its calibrated pose), plus a table, lights and the scene's objects;
- the station (8191; 8091 under roborun up --sim): the Roborun station logic (leases, motion pulls, Stop/hold, Home) over one
  ArmAdapter per active arm (joints follow their commands at a bounded speed; the fingers follow the jaw; an object between
  closing fingers is held, and put down where it is when they open); its /status says sim: true;
- the cameras: a broker on the camera socket (the runtime's camera client protocol) rendering the profile's
  cameras and a wrist camera on every arm, overlaid by robot.yaml's cameras that have a `card:` section and answer as the
  same alias (their size and declared depth; entries without a card leave the simulation as it was): the overhead camera
  at the rig's pose and wrist cameras mounted where the runtime believes they are (the rig's T_tool_camera, else
  the runtime's default wrist mount). An RGB-only camera (card: {depth: false}, or the rig's depth: false) serves no depth, exactly
  like a real webcam. The status carries each camera's card (camera_card.py) with the pose and intrinsics it is drawn with.
"""
from __future__ import annotations

import json
import math
import os
import queue
import socket
import struct
import sys
import threading
import time
from pathlib import Path

from roborun_sdk._optional import lazy, require
from roborun_sdk.embodiment import ArmAdapter

np = lazy('numpy', 'sim', globals())       # `roborun --help` builds this module's parser without numpy

JOINT_SPEED = 0.6          # rad/s, as the YAM station's arms
JAW_SPEED = 0.6            # jaw fraction/s
GRAB_RADIUS_M = 0.045      # (kept for older scenes)
PAD_SPAN_M = (-0.084, 0.012)   # the YAM finger pads along the approach axis, in the grasp-site frame (from the finger meshes)
PAD_HALF_WIDTH_M = 0.034       # ... and across the pads
CENTRE_TOL_M = 0.03            # the object's axis this close to the closing centreline is squeezed between the pads
SIZE = (640, 480)
LIFTED_M = 0.03            # a held object this far above where it stood counts as lifted (the sim log's `lifted` event)
MAX_STEP_S = 0.005                  # the arms and jaws advance at most this much simulated time per contact check
WRIST_FOVY = 58.3          # the D405 wrist cameras (fx ~430 at 640x480)
CUP = dict(id='orange_cup', shape='cup', position_mm=[-350, 470, 57.5], size_mm=[90, 90, 115], color=[1.0, 0.42, 0.05], fill=0.75)   # a 16 oz cup (115 mm tall, ~93 mm at the rim): the demo object
CUP_BOTTOM_RATIO = 0.9     # nearly straight walls, like a real cup
AMBIENT = 0.35             # ambient light, as in a room (not only what faces the lamps)
CUP_INSIDE = [0.98, 0.97, 0.94]
TABLE_XY = ((-1.2, -0.15), (0.6, 1.05))   # the tabletop's corners (m, table frame): around the YAM rig and its cameras' view
TABLE_MARGIN_M = 0.55      # ... grown so that it reaches at least this far past every object (another rig's arms face elsewhere)
TABLE_RGB = ([0.64, 0.57, 0.49], [0.60, 0.53, 0.45], [0.42, 0.36, 0.30])   # the tabletop's two checker tones and its marks


def _hollow(r_top, r_bottom, z0, z1, wall, floor, n=40):
    """A closed, open-topped tapered cup solid (vertices, faces): outer wall, rim, inner wall, inner floor, base. Each
    surface has its own vertices (crisp edges) and its faces wound so their normals point out of the solid."""
    t = np.linspace(0, 2 * np.pi, n, endpoint=False); c, s = np.cos(t), np.sin(t)
    ring = lambda r, z: np.c_[r * c, r * s, np.full(n, z)]
    radial = lambda p: np.r_[p[0], p[1], 0.0]
    verts, faces = [], []

    def surface(points, tris, outward):
        k = sum(len(v) for v in verts); verts.append(points)
        for tri in tris:
            a, b, d = points[list(tri)]
            if np.dot(np.cross(b - a, d - a), outward((a + b + d) / 3)) < 0:
                tri = tri[::-1]
            faces.append([k + i for i in tri])

    quads = [t for i in range(n) for t in ((i, (i + 1) % n, n + (i + 1) % n), (i, n + (i + 1) % n, n + i))]
    fan = [(i, (i + 1) % n, n) for i in range(n)]
    up, down = (lambda p: np.array([0, 0, 1.0])), (lambda p: np.array([0, 0, -1.0]))
    zf = z0 + floor
    surface(np.vstack([ring(r_bottom, z0), ring(r_top, z1)]), quads, radial)                          # outer wall
    surface(np.vstack([ring(r_top, z1), ring(r_top - wall, z1)]), quads, up)                          # rim
    surface(np.vstack([ring(r_top - wall, z1), ring(r_bottom - wall, zf)]), quads, lambda p: -radial(p))   # inner wall
    surface(np.vstack([ring(r_bottom - wall, zf), [[0, 0, zf]]]), fan, up)                            # inner floor
    surface(np.vstack([ring(r_bottom, z0), [[0, 0, z0]]]), fan, down)                                 # base
    return np.vstack(verts), np.asarray(faces, int)


def cup_mesh(r_top, h, bottom_ratio=CUP_BOTTOM_RATIO):
    """The cup's outside (rim and outer wall, plus the closed solid behind them) and its inside lining, centred on the
    object's origin: z from -h/2 to h/2."""
    wall = max(0.002, 0.03 * r_top * 2)
    outer = _hollow(r_top, r_top * bottom_ratio, -h / 2, h / 2, wall, max(0.004, 0.04 * h))
    gap = 0.0004
    lining = _hollow(r_top - wall - gap, r_top * bottom_ratio - wall - gap, -h / 2 + max(0.004, 0.04 * h) + gap, h / 2 - 0.002,
                     0.001, 0.001)
    return outer, lining


DEPTH_SCALE = {'top': 0.001}                     # metres per depth unit; the wrist cameras' finer 0.0001 (the D405's)


def sim_cameras(rig_spec, cameras_section=None):
    """{alias: camera plan} the simulation renders: the rig's top/left/right cameras, and a wrist camera for each arm of the
    rig the rig lists no camera for (640x480 RGB-D at the CAD mount, as the simulation always drew it), overlaid by
    robot.yaml's cameras that have a `card:` section and answer as the same alias (their size as delivered, their declared
    depth), plus an alias such a card adds. robot.yaml entries without a card are left alone, as before cards: a size
    written for the real camera does not change the simulation. A plan: {id, alias, cfg (its checked card config),
    settings, size (w, h), depth (bool), fovy (deg or None: the rig's own)}."""
    from .camera_card import ALIASES, check_card, check_config, delivered_size
    from .cameras import camera_settings
    rig_cams = (rig_spec or {}).get('cameras') or {}
    out = {}

    def plan(alias, rig_cam):
        depth = rig_cam.get('depth')
        return dict(id=alias, alias=alias, cfg=check_card(alias, {})[0], settings=dict(source=f'sim:{alias}', rotate=0),
                    size=SIZE, depth=depth if isinstance(depth, bool) else True,
                    fovy=float(rig_cam['fovy_deg']) if rig_cam.get('fovy_deg') else None)
    rig_arms = (rig_spec or {}).get('arms') or {}
    for alias in ALIASES:
        if isinstance(rig_cams.get(alias), dict):
            out[alias] = plan(alias, rig_cams[alias])
        elif alias in ('left', 'right') and alias in rig_arms:
            out[alias] = plan(alias, {})                  # an arm the rig lists no camera for keeps its wrist camera
    if isinstance(cameras_section, dict):                 # only carded entries describe the simulation's cameras
        cameras_section = {cid: e for cid, e in cameras_section.items() if isinstance(e, dict) and 'card' in e}
    if cameras_section:
        cfgs, _ = check_config(cameras_section, rig_spec)
        for cid, cfg in cfgs.items():
            alias = cfg['alias']
            if alias not in ALIASES:
                continue                                  # a camera of your own name with no alias: not in the simulation
            settings, _ = camera_settings(cid, cameras_section[cid])
            row = out.get(alias) or plan(alias, rig_cams.get(alias) if isinstance(rig_cams.get(alias), dict) else {})
            w, h = delivered_size(settings.get('width'), settings.get('height'), settings['rotate'])
            if w:
                row['size'] = (int(w), int(h))
            if cfg['depth'] is not None:
                row['depth'] = cfg['depth']
            row.update(id=cid, cfg=cfg, settings=dict(settings, source=f'sim:{alias}'))
            out[alias] = row
    return out


class World:
    """One MuJoCo scene for every arm of the profile: kinematic arms, held objects, rendered cameras."""

    def __init__(self, root, rig_path, scene=None, homes=None, cameras=None):
        self.root = Path(root)
        for p in (str(self.root), str(self.root / 'workspaces' / 'yam' / 'policies')):
            if p not in sys.path: sys.path.insert(0, p)
        mujoco = require('mujoco', 'sim')
        from kin import Rig
        self.mujoco = mujoco
        self.rig_spec = json.loads(Path(rig_path).read_text())
        self.cameras = sim_cameras(self.rig_spec, cameras)    # robot.yaml's `cameras:` section overlays the rig's
        self.objects = self._objects(scene, rig_path)
        self.rig = Rig(rig_path, decorate=self._decorate)
        self.spec = self.rig.spec
        self.model, self.data = self.rig.scene.model, self.rig.scene.data
        m = self.model
        self.arms = list(self.rig.arms)
        self.qadr = self.rig.scene.qadr
        self.site = self.rig.scene.sites
        self.fingers = {}; self.pads = {}; self.stroke = {}
        for a in self.arms:                               # [(qpos address, value at jaw 0, value at jaw 1)]: the arm's own jaw joints
            arm = self.rig.arms[a]
            if arm.described:
                ids = [(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f'{a}_{k}'), c, o) for k, c, o in arm.jaw_map]
                self.fingers[a] = [(int(m.jnt_qposadr[j]), c, o) for j, c, o in ids if j >= 0]
                self.stroke[a] = float(arm.jaw_gap_m or sum(abs(o - c) for _, c, o in self.fingers[a]))
            else:
                ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f'{a}_joint{i}') for i in (7, 8)]
                self.fingers[a] = [(int(m.jnt_qposadr[j]), 0.0, float(m.jnt_range[j][1])) for j in ids if j >= 0]
                self.stroke[a] = sum(o for _, _, o in self.fingers[a]) or 0.095
        self.obj = {o['id']: int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{o['id']}_free")]) for o in self.objects}
        self.size = {o['id']: np.asarray(o['size_mm'], float) / 1000 for o in self.objects}
        self.q = {}; self.target = {}; self.jaw = {}; self.jaw_target = {}; self.held = {}; self.held_jaw = {}; self.lifted = set()
        for a in self.arms:
            spec = self.spec['arms'][a]
            n = self.rig.arms[a].n_joints
            home = (spec.get('parked') or {}).get('positions_rad') or spec.get('home') or (homes or {}).get(a) or [0.0] * n
            self.q[a] = np.asarray(home, float)[:n].copy(); self.target[a] = self.q[a].copy()
            self.jaw[a] = float((spec.get('parked') or {}).get('gripper', 1.0)); self.jaw_target[a] = self.jaw[a]
        for g in range(m.ngeom):                          # planner-only geometry: the felt plane, attachment envelopes; no camera sees them
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
            if name == 'felt' or name.endswith('_envelope'):
                m.geom_rgba[g, 3] = 0
        self.mount = self._mount()
        self.mounts = {}                                  # T_tool_camera each wrist camera is drawn with: the rig's, else the CAD
        for a in ('left', 'right'):
            T = ((self.rig_spec.get('cameras') or {}).get(a) or {}).get('T_tool_camera')
            self.mounts[a] = np.asarray(T, float) if T is not None and np.asarray(T).shape == (4, 4) else self.mount
        sizes = [tuple(c['size']) for c in self.cameras.values()] + [SIZE]
        self.render_size = (max(w for w, _ in sizes), max(h for _, h in sizes))   # one GL context: every camera is drawn at
        self.lock = threading.RLock()                                            # this size and a smaller one cut from its centre
        self.t = time.monotonic()
        self._apply()
        for a in self.arms:
            box = self.rig.arms[a].pad_box() if self.rig.arms[a].described else None
            self.pads[a] = box or (PAD_SPAN_M, PAD_HALF_WIDTH_M)

    # ---- the scene
    def _objects(self, scene, rig_path=None):
        if not scene or scene == 'cup':
            placed = json.loads(Path(rig_path).read_text()).get('sim_object') if rig_path else None    # a catalog profile sets its cup where its arm reaches
            return [dict(placed or CUP)]
        if str(scene).endswith('.json'):                  # your own objects: [{id, shape, position_mm, size_mm, color}, ...]
            return json.loads(Path(scene).read_text())
        import sim_environments
        presets = sim_environments.presets()
        if scene not in presets:
            raise SystemExit(f"scene {scene!r}: one of cup, {', '.join(sorted(presets))}")
        return presets[scene]['scene']['objects']

    def _decorate(self, spec):
        mj = self.mujoco
        spec.visual.headlight.ambient = [AMBIENT] * 3
        spec.worldbody.add_light(pos=[-0.3, 0.4, 1.6], dir=[0, 0, -1], diffuse=[0.85, 0.85, 0.85])
        spec.worldbody.add_light(pos=[0.4, 1.0, 1.2], dir=[-0.3, -0.5, -1], diffuse=[0.35, 0.35, 0.35])
        # a textured tabletop (grain, marks), like a real one
        spec.add_texture(name='table_grain', type=mj.mjtTexture.mjTEXTURE_2D, builtin=mj.mjtBuiltin.mjBUILTIN_CHECKER,
                         rgb1=TABLE_RGB[0], rgb2=TABLE_RGB[1], width=512, height=512,
                         mark=mj.mjtMark.mjMARK_RANDOM, random=0.03, markrgb=TABLE_RGB[2])
        mat = spec.add_material(name='table_wood', texrepeat=[6, 4])
        mat.textures[mj.mjtTextureRole.mjTEXROLE_RGB] = 'table_grain'
        lo, hi = np.array(TABLE_XY[0], float), np.array(TABLE_XY[1], float)   # the table, grown to reach well past every object
        for o in self.objects:
            xy = np.asarray(o['position_mm'][:2], float) / 1000
            lo, hi = np.minimum(lo, xy - TABLE_MARGIN_M), np.maximum(hi, xy + TABLE_MARGIN_M)
        spec.worldbody.add_geom(name='table_top', type=mj.mjtGeom.mjGEOM_BOX, size=[*((hi - lo) / 2).tolist(), 0.01],
                                pos=[*((hi + lo) / 2).tolist(), -0.01], material='table_wood', contype=0, conaffinity=0)
        spec.worldbody.add_geom(name='floor', type=mj.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1], pos=[0, 0, -0.75], rgba=[0.3, 0.3, 0.32, 1],
                                contype=0, conaffinity=0)
        shapes = dict(box=mj.mjtGeom.mjGEOM_BOX, pack=mj.mjtGeom.mjGEOM_BOX, bar=mj.mjtGeom.mjGEOM_BOX, hammer=mj.mjtGeom.mjGEOM_BOX,
                      cylinder=mj.mjtGeom.mjGEOM_CYLINDER, bottle=mj.mjtGeom.mjGEOM_CYLINDER, mug=mj.mjtGeom.mjGEOM_CYLINDER,
                      sphere=mj.mjtGeom.mjGEOM_SPHERE)
        for o in self.objects:
            sx, sy, sz = (np.asarray(o['size_mm'], float) / 1000).tolist()
            x, y = np.asarray(o['position_mm'][:2], float) / 1000
            body = spec.worldbody.add_body(name=o['id'], pos=[x, y, sz / 2])
            body.add_freejoint(name=f"{o['id']}_free")
            rgba = list(o.get('color', [0.8, 0.3, 0.1])) + [1]
            if o.get('shape') == 'cup':                   # a real cup: open at the top, tapered, its inside lighter than the outside
                outer, inner = cup_mesh(sx / 2, sz, float(o.get('bottom_ratio', CUP_BOTTOM_RATIO)))
                spec.add_mesh(name=f"{o['id']}_outer", uservert=outer[0].ravel().tolist(), userface=outer[1].ravel().tolist())
                spec.add_mesh(name=f"{o['id']}_inner", uservert=inner[0].ravel().tolist(), userface=inner[1].ravel().tolist())
                body.add_geom(name=f"{o['id']}_geom", type=mj.mjtGeom.mjGEOM_MESH, meshname=f"{o['id']}_outer", rgba=rgba,
                              contype=0, conaffinity=0, mass=float(o.get('mass_kg', 0.012)))
                body.add_geom(name=f"{o['id']}_inside", type=mj.mjtGeom.mjGEOM_MESH, meshname=f"{o['id']}_inner",
                              rgba=list(o.get('inside_color', CUP_INSIDE)) + [1], contype=0, conaffinity=0, mass=0.001)
                if o.get('fill'):                         # a drink in it: its surface at this fraction of the height
                    br = float(o.get('bottom_ratio', CUP_BOTTOM_RATIO)); f = float(o['fill'])
                    wall = max(0.002, 0.03 * sx)
                    rr = (sx / 2) * (br + (1 - br) * f) - wall - 0.0006
                    body.add_geom(name=f"{o['id']}_drink", type=mj.mjtGeom.mjGEOM_CYLINDER, size=[rr, 0.001, 0], pos=[0, 0, -sz / 2 + f * sz],
                                  rgba=list(o.get('fill_color', [0.24, 0.13, 0.06])) + [1], contype=0, conaffinity=0, mass=0.001)
                continue
            kind = shapes.get(o.get('shape'), mj.mjtGeom.mjGEOM_BOX)
            size = ([sx / 2, sz / 2, 0] if kind == mj.mjtGeom.mjGEOM_CYLINDER else [sx / 2, 0, 0] if kind == mj.mjtGeom.mjGEOM_SPHERE
                    else [sx / 2, sy / 2, sz / 2])
            body.add_geom(name=f"{o['id']}_geom", type=kind, size=size, rgba=rgba,
                          contype=0, conaffinity=0, mass=float(o.get('mass_kg', 0.12)))
        for a in ('left', 'right'):                       # the wrist cameras ride mocap bodies placed every step
            cam_body = spec.worldbody.add_body(name=f'wristcam_{a}', mocap=True, pos=[0, 0, -1])
            # optical frame (+z forward, +y down) -> MuJoCo camera (-z forward, +y up): a half turn about x
            fovy = (self.cameras.get(a) or {}).get('fovy') or WRIST_FOVY
            cam_body.add_camera(name=f'wrist_{a}', quat=[0, 1, 0, 0], fovy=fovy)
        sizes = [c['size'] for c in self.cameras.values()] + [SIZE]
        spec.visual.global_.offwidth = max(int(spec.visual.global_.offwidth), *(w for w, _ in sizes))
        spec.visual.global_.offheight = max(int(spec.visual.global_.offheight), *(h for _, h in sizes))

    def _mount(self):
        try:
            import video_calibration
            return np.asarray(video_calibration.mount(), float)
        except Exception as exc:
            print(json.dumps(dict(event='wrist_mount_default', why=str(exc)[:160])), flush=True)
            T = np.eye(4); T[:3, 3] = [0, -0.07, -0.08]
            return T

    # ---- stepping
    def step(self):
        with self.lock:
            now = time.monotonic(); elapsed = min(0.1, now - self.t); self.t = now
            n = max(1, math.ceil(elapsed / MAX_STEP_S)); dt = elapsed / n     # a late tick never jumps past a contact
            for _ in range(n):
                for a in self.arms:
                    d = np.clip(self.target[a] - self.q[a], -JOINT_SPEED * dt, JOINT_SPEED * dt); self.q[a] = self.q[a] + d
                    self.jaw[a] += float(np.clip(self.jaw_target[a] - self.jaw[a], -JAW_SPEED * dt, JAW_SPEED * dt))
                self._apply()
                self._grasping()

    def _apply(self):
        mj = self.mujoco; d = self.data
        for a in self.arms:
            d.qpos[self.qadr[a]] = self.q[a]
            for adr, closed, opened in self.fingers[a]:
                d.qpos[adr] = closed + float(np.clip(self.jaw[a], 0, 1)) * (opened - closed)
        mj.mj_kinematics(self.model, d)
        for oid, (a, T_rel) in self.held.items():         # held objects ride their gripper
            T = self._site(a) @ T_rel
            adr = self.obj[oid]; d.qpos[adr:adr + 3] = T[:3, 3]; d.qpos[adr + 3:adr + 7] = _quat(T[:3, :3])
        for a in ('left', 'right'):
            body = self.mujoco.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, f'wristcam_{a}')
            mid = int(self.model.body_mocapid[body])
            if a in self.site and mid >= 0:
                T = self._site(a) @ self.mounts[a]
                d.mocap_pos[mid] = T[:3, 3]; d.mocap_quat[mid] = _quat(T[:3, :3])
        mj.mj_forward(self.model, d)

    def _site(self, a):
        T = np.eye(4); T[:3, :3] = self.data.site_xmat[self.site[a]].reshape(3, 3); T[:3, 3] = self.data.site_xpos[self.site[a]]
        return T

    def _between_fingers(self, a, oid):
        """Some point of the object's vertical axis lies between this arm's finger pads: in the grasp frame, within the
        pads' span along the approach (PAD_SPAN_M) and across (PAD_HALF_WIDTH_M), and near the closing centreline."""
        adr = self.obj[oid]; c = self.data.qpos[adr:adr + 3]; h = self.size[oid][2]
        axis = np.c_[np.full(11, c[0]), np.full(11, c[1]), np.linspace(c[2] - h / 2, c[2] + h / 2, 11), np.ones(11)]
        local = (axis @ np.linalg.inv(self._site(a)).T)[:, :3]
        x, y, z = local[:, 0], local[:, 1], local[:, 2]
        span, half = self.pads[a]
        return bool(np.any((span[0] <= z) & (z <= span[1]) & (np.abs(x) <= half) & (np.abs(y) <= CENTRE_TOL_M)))

    def _contact_jaw(self, a, oid):
        """The jaw value whose finger gap equals the object's width (the fingers stop there, like real ones on a cup)."""
        return float(np.clip(float(min(self.size[oid][:2])) / self.stroke[a], 0.0, 1.0))

    def _grasping(self):
        """Closing on an object between the fingers stops the jaw at the object's width and holds the object (the stalled
        jaw is the contact real grippers show); opening the jaw past that width puts it down where it is."""
        for a in self.arms:
            held = [o for o, (arm, _) in self.held.items() if arm == a]
            if held:
                oid = held[0]; at = self.held_jaw[oid]
                if self.jaw_target[a] > at + 0.03:                  # opening: let go
                    adr = self.obj[oid]; z = self.size[oid][2] / 2
                    self.data.qpos[adr + 2] = z; self.data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
                    del self.held[oid]; del self.held_jaw[oid]; self.lifted.discard(oid)
                    print(json.dumps(dict(event='released', arm=a, object=oid, jaw_command=round(float(self.jaw_target[a]), 3), held_at=round(at, 3))), flush=True)
                    self.mujoco.mj_forward(self.model, self.data)
                else:
                    if self.jaw[a] < at:
                        self.jaw[a] = at                              # the fingers stand on the object
                    rise = float(self.data.qpos[self.obj[oid] + 2]) - self.size[oid][2] / 2
                    if rise >= LIFTED_M and oid not in self.lifted:   # off the table: the log says so once per grasp
                        self.lifted.add(oid)
                        print(json.dumps(dict(event='lifted', arm=a, object=oid, height_m=round(rise, 3))), flush=True)
                continue
            if self.jaw_target[a] >= self.jaw[a] - 1e-6:
                continue                                              # not closing
            for oid in self.obj:
                if oid in self.held or not self._between_fingers(a, oid):
                    continue
                at = self._contact_jaw(a, oid)
                if self.jaw[a] <= at:
                    self.jaw[a] = at
                    T_obj = np.eye(4); T_obj[:3, 3] = self.data.qpos[self.obj[oid]:self.obj[oid] + 3]
                    self.held[oid] = (a, np.linalg.inv(self._site(a)) @ T_obj); self.held_jaw[oid] = at
                    print(json.dumps(dict(event='grasped', arm=a, object=oid, jaw=round(at, 3))), flush=True)
                break

    def objects_state(self):
        """Where each object is now (its centre, metres, table frame), how far above where it stood, and which arm holds it."""
        with self.lock:
            return [dict(id=oid, position_m=[round(float(v), 4) for v in self.data.qpos[adr:adr + 3]],
                         rise_m=round(float(self.data.qpos[adr + 2]) - self.size[oid][2] / 2, 4),
                         held_by=self.held[oid][0] if oid in self.held else None) for oid, adr in self.obj.items()]

    # ---- rendering (on the camera thread only: one GL context)
    def renderer(self):
        if not os.environ.get('MUJOCO_GL'):
            from .robot import gl_env
            os.environ.update(gl_env())
        return self.mujoco.Renderer(self.model, self.render_size[1], self.render_size[0])

    def _camera(self, role):
        cam = {'top': 'top', 'left': 'wrist_left', 'right': 'wrist_right'}.get(role)
        cid = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_CAMERA, cam) if cam else -1
        if role not in self.cameras or cid < 0:
            raise ValueError(f'No fresh acquired frame: the {role} camera is not in this simulation')
        return cam, cid

    def _cut(self, role):
        """(left, top) of this camera's image in the full-size render."""
        (W, H), (w, h) = self.render_size, self.cameras[role]['size']
        return (W - w) // 2, (H - h) // 2

    def intrinsics(self, role):
        """The pinhole the simulation serves for this camera (MuJoCo renders the principal point at the centre)."""
        _, cid = self._camera(role)
        w, h = self.cameras[role]['size']
        (W, H), (x0, y0) = self.render_size, self._cut(role)
        f = (h / 2) / math.tan(math.radians(float(self.model.cam_fovy[cid])) / 2)
        return dict(width=w, height=h, fx=f, fy=f, ppx=W / 2 - x0, ppy=H / 2 - y0, distortion_model='none', coeffs=[0, 0, 0, 0, 0])

    def camera_pose(self, role):
        """T_parent_camera this camera is drawn at, optical axes (+z forward, +y down): T_table_camera for top, T_tool_camera
        (grasp site <- camera) for a wrist camera."""
        if role in ('left', 'right'):
            return self.mounts[role]
        mj = self.mujoco
        _, cid = self._camera(role)
        with self.lock:
            mj.mj_forward(self.model, self.data)
            T = np.eye(4)
            T[:3, :3] = self.data.cam_xmat[cid].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
            T[:3, 3] = self.data.cam_xpos[cid]
        return T

    def card(self, role):
        """This camera's card as the simulation draws it: pose and intrinsics are the simulation's own (source sim)."""
        from .camera_card import derive
        plan = self.cameras[role]
        depth = bool(plan['depth'])
        return derive(plan['id'], plan['settings'], plan['cfg'], rig=self.rig_spec,
                      sim=dict(pose=np.asarray(self.camera_pose(role), float).round(9).tolist(), intrinsics=self.intrinsics(role),
                               depth=depth, depth_scale_m=DEPTH_SCALE.get(role, 0.0001) if depth else None))

    def render(self, renderer, role):
        cam, cid = self._camera(role)
        plan = self.cameras[role]
        if (renderer.width, renderer.height) != self.render_size:
            raise ValueError(f'the renderer is {renderer.width}x{renderer.height}; this simulation draws at '
                             f'{self.render_size[0]}x{self.render_size[1]} (World.renderer())')
        k = self.intrinsics(role)
        (w, h), (x0, y0) = plan['size'], self._cut(role)
        opt = self.mujoco.MjvOption(); opt.sitegroup[:] = 0          # no planner markers (grasp sites) in the cameras
        with self.lock:                                   # only the scene copy holds up the physics; drawing can be slow (CPU)
            fovy = float(self.model.cam_fovy[cid])
            if (w, h) != self.render_size:                # the full render's field of view, so the cut keeps this camera's focal length
                self.model.cam_fovy[cid] = math.degrees(2 * math.atan((self.render_size[1] / 2) / k['fy']))
            try:
                renderer.update_scene(self.data, camera=cam, scene_option=opt)
            finally:
                self.model.cam_fovy[cid] = fovy
        cut = lambda image: image[y0:y0 + h, x0:x0 + w].copy() if (w, h) != self.render_size else image.copy()
        rgb = cut(renderer.render())
        meta = dict(reference_stream='color', timestamp_domain='global_time', acquisition_timestamp_verified=True,
                    serial=f'mujoco-{role}', simulated=True, intrinsics=k, camera_id=plan['id'],
                    rotate=int(plan['settings'].get('rotate') or 0))
        if not plan['depth']:                             # an RGB-only camera, declared as a real one is (cameras.py)
            return dict(meta, rgb=rgb, depth_z16=np.zeros(rgb.shape[:2], np.uint16), depth_available=False, depth_scale_m=None,
                        depth_aligned_to=None)
        renderer.enable_depth_rendering()
        depth = cut(renderer.render()); renderer.disable_depth_rendering()
        scale = DEPTH_SCALE.get(role, 0.0001)
        z16 = np.clip(np.nan_to_num(depth / scale), 0, 65535).astype(np.uint16)
        return dict(meta, rgb=rgb, depth_z16=z16, depth_scale_m=scale, depth_available=True, depth_aligned_to='color')


def write_calibration(world, root):
    """The overhead camera's calibration, exact: its pose in the table frame (the sim's world frame, where the planner's
    fk lives) in the optical convention (+z forward, +y down), the same for every arm. Saved where the runtime reads the
    newest calibration (its log folder), so a fresh install plans in the sim with no fitting."""
    mj = world.mujoco
    with world.lock:
        mj.mj_forward(world.model, world.data)
        cid = mj.mj_name2id(world.model, mj.mjtObj.mjOBJ_CAMERA, 'top')
        if cid < 0:
            return None
        T = np.eye(4)
        T[:3, :3] = world.data.cam_xmat[cid].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
        T[:3, 3] = world.data.cam_xpos[cid]
    out = Path(root) / 'log' / 'video-motion' / 'sim-calibration'
    out.mkdir(parents=True, exist_ok=True)
    cal = dict(plan_id='sim-calibration', method='MuJoCo simulation: the overhead camera pose, exact', calibrated=True,
               relative_motion_ready=True, motor_commands=0, sim=True,
               T_table_camera_by_arm={a: T.round(9).tolist() for a in world.arms})
    (out / 'calibration.json').write_text(json.dumps(cal, indent=1))
    (out / 'plan.json').write_text(json.dumps(dict(id='sim-calibration', kind='calibration', sim=True)))   # a clip folder
    return out / 'calibration.json'


def _quat(R):
    import mujoco
    q = np.zeros(4); mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(9)); return q


class MuJoCoArm(ArmAdapter):
    """One arm of the World, as an ArmAdapter (the station ticks it at 30 Hz). robot.yaml's `rules:` apply here exactly as on a
    real arm: a check() that answers False holds the arm where it is, one that changes the command is obeyed, and one that
    fails stops the arm (every later command is refused, saying so) -- the simulator never moves through a rule."""
    def __init__(self, world, slot, rules=None):
        self.world, self.slot = world, slot
        lim = world.rig.arms[slot].limits; self.joints = world.rig.arms[slot].n_joints
        self.home = world.q[slot].tolist(); self.limits = [tuple(map(float, l)) for l in lim[:self.joints]]
        from roborun_sdk.arms.base import _load_rule
        self._rules_path = rules
        self._rule = _load_rule(rules) if rules else None
        self.rule_error = None
        self._refused_at = None

    def _ruled(self, positions_rad, gripper):
        """(positions, gripper) to send, or None to hold where the arm is."""
        from roborun_sdk.arms.base import ArmHardwareError
        if self.rule_error:
            raise ArmHardwareError(self.rule_error)
        if self._rule is None:
            return positions_rad, gripper
        with self.world.lock:
            state = dict(positions_rad=self.world.q[self.slot].tolist(), gripper=float(self.world.jaw[self.slot]), side=self.slot)
        try:
            answer = self._rule(self.slot, [float(v) for v in positions_rad], float(gripper), state)
            if answer is False:
                if self._refused_at is None:
                    self._refused_at = time.monotonic()
                return None
            self._refused_at = None
            if answer not in (None, True):
                positions_rad, gripper = answer
                if len(positions_rad) != self.joints:
                    raise ValueError(f'returned {len(positions_rad)} joints')
        except Exception as exc:                          # a broken rule must not let motion through
            self.rule_error = f'your rules file failed: {type(exc).__name__}: {exc}'[:300]
            self.hold()
            raise ArmHardwareError(self.rule_error) from None
        return positions_rad, gripper

    def rule_refusal(self):
        """As StreamedArm.rule_refusal: the station ends the task with this instead of waiting out its window."""
        from roborun_sdk.arms.base import StreamedArm
        at = self._refused_at
        if at is None or time.monotonic() - at < StreamedArm.rule_refused_after_s:
            return None
        return f'your rules file refused this motion of the {self.slot} arm'

    def health(self):
        return dict(ok=False, error=self.rule_error, kind='rules') if self.rule_error else dict(ok=True)

    def reopen(self):
        """Reset: the rules file is read again (fixed by its owner, it applies from here) and the arm holds where it is."""
        from roborun_sdk.arms.base import _load_rule
        self._rule = _load_rule(self._rules_path) if self._rules_path else None
        self.rule_error, self._refused_at = None, None
        self.hold()

    def hold(self):
        with self.world.lock:
            self.world.target[self.slot] = self.world.q[self.slot].copy()
            self.world.jaw_target[self.slot] = float(self.world.jaw[self.slot])

    def command(self, positions_rad, gripper):
        ruled = self._ruled(positions_rad, gripper)
        if ruled is None:
            return self.hold()
        positions_rad, gripper = ruled
        with self.world.lock:
            self.world.target[self.slot] = np.asarray(positions_rad, float)[:self.joints]
            self.world.jaw_target[self.slot] = float(np.clip(gripper, 0, 1))

    def read(self):
        if self.slot == 'left':                           # one arm steps the shared world per tick
            self.world.step()
        with self.world.lock:
            w, a = self.world, self.slot
            pressing = any(arm == a and w.jaw_target[a] < w.held_jaw[o] - 0.005 for o, (arm, _) in w.held.items())
            return dict(positions_rad=w.q[a].tolist(), gripper=w.jaw[a], gripper_effort=1.0 if pressing else 0.0)


class Cameras:
    """The camera broker (camera_service protocol) rendering the World's cameras."""
    def __init__(self, world, path):
        self.world, self.path = world, Path(path)
        self.session = 'mujoco-' + os.urandom(6).hex()
        self.ids = {}
        self.jobs = queue.Queue()
        self.calibration_dir = None

    def serve(self):
        from roborun_sdk import sim_home
        sim_home.claim_socket(self.path)                  # a served or someone else's socket: refused, never removed
        srv = socket.socket(socket.AF_UNIX); srv.bind(str(self.path))
        self.release = sim_home.own_socket(self.path)
        srv.listen(16)
        threading.Thread(target=self._render_loop, daemon=True, name='mujoco-render').start()
        threading.Thread(target=self._accept, args=(srv,), daemon=True, name='camera-socket').start()
        return self.release

    def _render_loop(self):
        renderer = self.world.renderer()
        if self.calibration_dir is not None:                  # the calibration's reference overhead frame (view registration)
            try:
                f = self.world.render(renderer, 'top')
                meta = {k: v for k, v in f.items() if k not in ('rgb', 'depth_z16')}
                meta.update(frame_id=0, depth_frame_id=0, camera_session_id=self.session, received_at=time.time())
                np.savez(self.calibration_dir / 'source.npz', rgb=f['rgb'], depth_z16=f['depth_z16'], metadata=json.dumps(meta))
            except Exception as exc:
                print(json.dumps(dict(event='calibration_frame_failed', why=str(exc)[:200])), flush=True)
        while True:
            role, box = self.jobs.get()
            try: box['frame'] = self.world.render(renderer, role)
            except Exception as exc: box['error'] = str(exc)
            box['done'].set()

    def status(self):
        """The broker's status: only the cameras this simulation declares, with their cards."""
        roles = [r for r in ('top', 'left', 'right') if r in self.world.cameras]
        cards = []
        for r in roles:
            try:
                cards.append(self.world.card(r))
            except Exception as exc:                      # a card must never take the status (and so the robot) down
                print(json.dumps(dict(event='camera_card_failed', role=r, why=str(exc)[:200])), flush=True)
        return dict(state='streaming', camera_session_id=self.session, motor_access=False, sim=True,
                    frame_age_s={r: 0.01 for r in roles}, errors={}, objects=self.world.objects_state(),
                    available_roles=roles, unavailable_roles=[r for r in ('top', 'left', 'right') if r not in roles],
                    complete_observation=len(roles) == 3,
                    roles_without_depth=[r for r in roles if not self.world.cameras[r]['depth']], cards=cards)

    def _accept(self, srv):
        import camera_service as cs                       # the runtime's camera client: the same protocol both ways
        while True:
            conn, _ = srv.accept()
            threading.Thread(target=self._handle, args=(conn, cs), daemon=True).start()

    def _handle(self, conn, cs):
        with conn:
            try:
                body = json.loads(cs.receive(conn))
                op = body.get('operation')
                if op in ('status', 'stop', 'restart'):
                    return cs.send(conn, json.dumps(self.status()).encode())
                if op != 'frame':
                    return cs.send(conn, json.dumps(dict(error=f'Unknown camera operation {op!r}')).encode())
                role = body.get('role')
                box = dict(done=threading.Event()); self.jobs.put((role, box)); box['done'].wait(5)
                if 'frame' not in box:
                    return cs.send(conn, json.dumps(dict(error=box.get('error') or 'render timed out')).encode())
                now = time.monotonic(); fid = self.ids[role] = self.ids.get(role, 1000) + 1
                has_depth = box['frame'].get('depth_available', True)
                frame = dict(box['frame'], frame_id=fid, depth_frame_id=fid if has_depth else None, acquired_at_s=now - .02,
                             depth_acquired_at_s=now - .02 if has_depth else None, sdk_received_monotonic_s=now - .01,
                             received_monotonic_s=now, received_at=time.time(), camera_session_id=self.session, role=role)
                cs.send(conn, cs.encode_frame(frame))
            except Exception as exc:
                try: cs.send(conn, json.dumps(dict(error=str(exc))).encode())
                except OSError: pass


def _runtime(a, c):
    """(the runtime this simulation runs from, the profile it simulates). Always a simulation's own folder: the copy of the live
    runtime in <home>/sim (sim_home.prepare_copy), or with --runtime FOLDER a copy of FOLDER (<home>/sim/runtimes/<12 hex>); a
    FOLDER that already is one of this install's simulation folders (roborun up --sim passes its copy) is used as it is. No
    runtime folder outside <home>/sim is ever written: not the live robot's, not another install's, not a checkout."""
    from roborun_sdk import sim_home
    from roborun_sdk.robot import _state, current_profile, install_profiles
    source = sim_home.live_runtime(c)
    if a.manifest:
        given = Path(a.manifest).expanduser().absolute()
        if not (given / 'kin.py').exists():
            raise SystemExit(f'no Roborun runtime at {given} (--runtime)')
        source = given
    base = sim_home.copy_folder(source)
    shared = sim_home.same(source, sim_home.runtime()) or sim_home.same(base, sim_home.folder())   # roborun up --sim's copy
    if shared and c.get('sim') and _state('roborun-sim') in ('active', 'running') and not getattr(a, 'as_robot', False):
        raise SystemExit(f'this robot runs as a simulation now (roborun up --sim), from {sim_home.folder()}: a second one '
                         'would share its folder. Next: roborun down first, or pass --runtime <another runtime folder>')
    if sim_home.is_copy(source):
        return source, a.profile or current_profile(source)
    root = sim_home.prepare_copy(source, base=base)
    pid = a.profile
    if not pid:                                       # the profile the robot runs (robot.json), else the copy's own
        pid = c.get('profile') if c.get('profile') in install_profiles(root) else current_profile(root)
    return root, pid


def _refuse_the_robot_s_data(sessions, c):
    """--sessions never points into what the robot's history upload reads: the live runtime (its log/) or a folder robot.json's
    data sync lists. A simulation's own folder is always fine; anything else outside the robot's data is the user's to choose."""
    from roborun_sdk import sim_home
    if not sessions or sim_home.is_copy(sessions):
        return
    here = os.path.realpath(os.path.expanduser(str(sessions)))
    tops = [sim_home.live_runtime(c)] + [Path(v).expanduser() for v in ((c.get('data') or {}).get('sources') or {}).values() if v]
    for top in tops:
        t = os.path.realpath(str(top))
        if here == t or here.startswith(t + os.sep):
            raise SystemExit(f"--sessions {sessions} is the robot's own data ({top}): its history upload would send simulated "
                             'sessions as real ones. Nothing was started. Next: leave --sessions out (the simulation keeps its '
                             'sessions in its own folder) or pick a folder outside the robot\'s runtime and data')


def _refuse_the_robot_s_port_and_socket(a, c, port, sock):
    """Bare `roborun sim` never serves on the robot's station port (8091) or a robot's camera socket, even when nothing serves
    them right now: the live robot may be between restarts, and its backend would then plan real-arm motions from simulated
    frames. Only `roborun up --sim` runs the simulation as the robot there (its hidden --as-robot), and only while this
    install's live station, camera service and a live station on 8091 are all down."""
    from roborun_sdk import sim_home
    from roborun_sdk.robot import STATION_PORT
    robot_port = int(port) == STATION_PORT
    robot_sock = sim_home.robot_socket(sock)
    if not (robot_port or robot_sock):
        return
    what = ' and '.join(x for x in (f'port {port} is the robot\'s station port' if robot_port else '',
                                     f'{sock} is a robot\'s camera socket' if robot_sock else '') if x)
    if not getattr(a, 'as_robot', False):
        raise SystemExit(f'{what}: roborun sim does not use it, even while it is free (the live robot may only be between '
                         'restarts). Nothing was started. Next: leave out --port and --camera-socket (the simulation then uses '
                         f'its own, {sim_home.SIM_PORT} and sim-camera.sock), or run the simulation as this robot: roborun down, '
                         'then roborun up --sim <scene>')
    running = sim_home.live_parts(c, backend=False)
    if running:
        raise SystemExit(f"{what}, and the live robot is running here ({', '.join(running)}): a simulation does not take "
                         'its station or cameras over. Nothing was started. Next: roborun down, then roborun up --sim <scene>')


def cmd_sim(a) -> int:
    from roborun_sdk import sim_home
    from roborun_sdk.robot import CONFIG, install_profiles
    from roborun_sdk.embodiment import run_station
    c = json.loads(CONFIG.read_text()) if CONFIG.exists() else {}
    port = sim_home.SIM_PORT if a.port is None else a.port
    sock = Path(a.camera_socket or sim_home.camera_socket())
    _refuse_the_robot_s_port_and_socket(a, c, port, sock)
    _refuse_the_robot_s_data(getattr(a, 'sessions', None), c)
    user = sim_home.port_user(port)
    if user:
        raise SystemExit(f'port {port} on this computer is already served by {user}: the simulation does not take it over. '
                         'Nothing was started. Next: stop it first (roborun down if it is this robot), or pass --port <free port>')
    sim_home.claim_socket(sock)                           # checked before anything is copied or written
    root, pid = _runtime(a, c)
    for p in (str(root), str(root / 'workspaces' / 'yam' / 'policies')):
        if p not in sys.path: sys.path.insert(0, p)
    from roborun_sdk.arm_profiles import ensure
    ensure(root, pid)
    prof = install_profiles(root).get(pid)
    if prof is None:
        raise SystemExit(f'no profile {pid!r} in {root}')
    cfg = json.loads(prof['config'].read_text())
    rig = (prof['record'] or {}).get('rig') or cfg['rig']
    # a profile whose arms have native controllers keeps an arm's home there (YAM + MakerMods Metal: the Metal's)
    homes = {s: (l.get('home') or {}).get('positions_rad') for s, l in (cfg.get('limbs') or {}).items() if isinstance(l, dict)}
    rules, own_cameras = None, None                       # robot.yaml's `rules:` and `cameras:`, as the real robot takes them
    try:
        from roborun_sdk.arms import settings
        mine = settings.load(warn=lambda _: None)
        rules = settings._local(mine['rules']) if mine.get('rules') else None
        arms = mine.get('arms') if isinstance(mine.get('arms'), dict) else {}
        own_cameras = mine.get('cameras') or None
        if own_cameras and arms.get('right') and not arms.get('left'):
            own_cameras = settings.one_arm_on_the_left(dict(right=arms['right'], cameras=own_cameras)).get('cameras')
    except SystemExit as exc:
        raise SystemExit(f'robot.yaml: {exc}')
    try:
        world = World(root, root / rig, a.scene, homes={s: q for s, q in homes.items() if q}, cameras=own_cameras)
    except ValueError as exc:
        raise SystemExit(f'robot.yaml: {exc}')
    active = [s for s in world.arms if not isinstance(world.spec['arms'][s].get('parked'), dict)]
    left = MuJoCoArm(world, 'left', rules) if 'left' in active else None
    right = MuJoCoArm(world, 'right', rules) if 'right' in active else None
    if left is None:
        raise SystemExit('the simulated station drives the left arm first: this profile parks it')
    cameras = Cameras(world, sock)
    cal = write_calibration(world, root)
    cameras.calibration_dir = cal.parent if cal else None
    release = cameras.serve()
    import signal
    previous = signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))   # roborun down / systemctl stop: the socket goes with it
    srv = None
    try:
        if rules:
            print(json.dumps(dict(event='mujoco_sim_rules', rules=rules)), flush=True)
        station, srv = run_station(left, right, port=port, sessions_dir=a.sessions or str(root / 'log' / 'video-motion' / 'sessions'),
                                   predict=a.predict or None)
        status = station.status
        station.status = lambda: dict(status(), sim=True)    # a simulation's station says so (sim_home.port_user)
        print(json.dumps(dict(event='mujoco_sim', profile=pid, arms=active, objects=[o['id'] for o in world.objects], camera_socket=str(sock),
                              cameras={r: dict(id=c['id'], size=list(c['size']), depth=c['depth']) for r, c in world.cameras.items()},
                              station=srv.url, runtime=str(root))), flush=True)
        while True: time.sleep(3600)
    except KeyboardInterrupt:
        pass
    finally:
        if srv is not None: srv.close()
        release()
        signal.signal(signal.SIGTERM, previous)
    return 0


def add_parser(sub):
    import argparse
    s = sub.add_parser('sim', help='simulate this robot\'s arms and cameras, e.g. two YAMs')
    s.add_argument('--profile', help='a profile of the runtime (default: the one the robot runs)')
    s.add_argument('--scene', default='cup', help='cup (default), pickup-bench, mixed-objects, cluttered-bench, low-light, or a .json list of objects')
    s.add_argument('--port', type=int, default=None, help='the simulated station\'s port (default 8191; refused when served, and 8091 '
                   'is the robot\'s: roborun up --sim)')
    s.add_argument('--camera-socket', help='the simulated cameras\' socket (default: sim-camera.sock beside the robot\'s; '
                   'refused when served, and a robot\'s camera.sock is never used: roborun up --sim)')
    s.add_argument('--sessions')
    s.add_argument('--predict', help='the backend that answers motion leases (default: none; roborun up --sim gives its own)')
    s.add_argument('--runtime', dest='manifest', help='simulate this runtime folder (default: the live one); the simulation runs '
                   'from its own copy of it under the Roborun home and never writes into the folder')
    s.add_argument('--as-robot', action='store_true', help=argparse.SUPPRESS)   # roborun up --sim: the robot's 8091 and camera socket
    s.set_defaults(func=cmd_sim)
