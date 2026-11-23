"""roborun sim: this robot's profile as a MuJoCo simulation that the Roborun platform drives like real arms.

    roborun sim                       the profile the runtime has loaded (e.g. two YAMs: roborun up --profile yam_dual)
    roborun sim --scene pickup-bench  one of the runtime's scenes (pickup-bench, mixed-objects, cluttered-bench, low-light);
                                       default: an orange cup in front of the left arm

What runs:
- the scene: the runtime's own rig model, built from the profile's rig (arms where the planner expects them, the overhead camera at
  its calibrated pose), plus a table, lights and the scene's objects;
- the station on :8091: the Roborun station logic (leases, motion pulls, Stop/hold, Home) over one ArmAdapter per active arm
  (joints follow their commands at a bounded speed; the fingers follow the jaw; an object between closing fingers is held,
  and put down where it is when they open);
- the cameras: a broker on the runtime's camera socket (the runtime's camera client protocol) rendering RGB-D for the
  overhead camera and for wrist cameras mounted where the runtime believes they are.
Then `roborun up` connects it to Roborun like any robot.
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

import numpy as np

from roborun_sdk.embodiment import ArmAdapter

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


class World:
    """One MuJoCo scene for every arm of the profile: kinematic arms, held objects, rendered cameras."""

    def __init__(self, root, rig_path, scene=None, homes=None):
        self.root = Path(root)
        for p in (str(self.root), str(self.root / 'workspaces' / 'yam' / 'policies')):
            if p not in sys.path: sys.path.insert(0, p)
        import mujoco
        from kin import Rig
        self.mujoco = mujoco
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
        self.lock = threading.RLock()
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
            cam_body.add_camera(name=f'wrist_{a}', quat=[0, 1, 0, 0], fovy=WRIST_FOVY)

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
                T = self._site(a) @ self.mount
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
        return self.mujoco.Renderer(self.model, SIZE[1], SIZE[0])

    def render(self, renderer, role):
        cam = {'top': 'top', 'left': 'wrist_left', 'right': 'wrist_right'}.get(role)
        if cam is None or self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_CAMERA, cam) < 0:
            raise ValueError(f'No fresh acquired frame: the {role} camera is not in this simulation')
        opt = self.mujoco.MjvOption(); opt.sitegroup[:] = 0          # no planner markers (grasp sites) in the cameras
        with self.lock:                                   # only the scene copy holds up the physics; drawing can be slow (CPU)
            renderer.update_scene(self.data, camera=cam, scene_option=opt)
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        depth = renderer.render().copy(); renderer.disable_depth_rendering()
        fovy = float(self.model.cam_fovy[self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_CAMERA, cam)])
        f = (SIZE[1] / 2) / math.tan(math.radians(fovy) / 2)
        scale = 0.001 if role == 'top' else 0.0001
        z16 = np.clip(np.nan_to_num(depth / scale), 0, 65535).astype(np.uint16)
        return dict(rgb=rgb, depth_z16=z16, depth_scale_m=scale, reference_stream='color', timestamp_domain='global_time',
                    acquisition_timestamp_verified=True, serial=f'mujoco-{role}', simulated=True,
                    intrinsics=dict(width=SIZE[0], height=SIZE[1], fx=f, fy=f, ppx=SIZE[0] / 2, ppy=SIZE[1] / 2,
                                    distortion_model='none', coeffs=[0, 0, 0, 0, 0]))


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
        threading.Thread(target=self._render_loop, daemon=True, name='mujoco-render').start()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.exists(): self.path.unlink()
        srv = socket.socket(socket.AF_UNIX); srv.bind(str(self.path)); srv.listen(16)
        threading.Thread(target=self._accept, args=(srv,), daemon=True, name='camera-socket').start()

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
                    return cs.send(conn, json.dumps(dict(state='streaming', camera_session_id=self.session, motor_access=False, sim=True,
                                                         frame_age_s={r: 0.01 for r in ('top', 'left', 'right')}, errors={},
                                                         objects=self.world.objects_state())).encode())
                if op != 'frame':
                    return cs.send(conn, json.dumps(dict(error=f'Unknown camera operation {op!r}')).encode())
                role = body.get('role')
                box = dict(done=threading.Event()); self.jobs.put((role, box)); box['done'].wait(5)
                if 'frame' not in box:
                    return cs.send(conn, json.dumps(dict(error=box.get('error') or 'render timed out')).encode())
                now = time.monotonic(); fid = self.ids[role] = self.ids.get(role, 1000) + 1
                frame = dict(box['frame'], frame_id=fid, depth_frame_id=fid, acquired_at_s=now - .02, depth_acquired_at_s=now - .02,
                             sdk_received_monotonic_s=now - .01, received_monotonic_s=now, received_at=time.time(), camera_session_id=self.session)
                cs.send(conn, cs.encode_frame(frame))
            except Exception as exc:
                try: cs.send(conn, json.dumps(dict(error=str(exc))).encode())
                except OSError: pass


def cmd_sim(a) -> int:
    from roborun_sdk.robot import CONFIG, HOME, _manifest_root, install_profiles, current_profile
    from roborun_sdk.embodiment import run_station
    c = json.loads(CONFIG.read_text()) if CONFIG.exists() else {}
    root = Path(a.manifest or c.get('manifest') or _manifest_root())
    if not (root / 'kin.py').exists():
        raise SystemExit(f'no Roborun runtime at {root}: run roborun up once (it installs it), or pass --runtime')
    for p in (str(root), str(root / 'workspaces' / 'yam' / 'policies')):
        if p not in sys.path: sys.path.insert(0, p)
    pid = a.profile or current_profile(root)
    from roborun_sdk.arm_profiles import ensure
    ensure(root, pid)
    prof = install_profiles(root).get(pid)
    if prof is None:
        raise SystemExit(f'no profile {pid!r} in {root}')
    cfg = json.loads(prof['config'].read_text())
    rig = (prof['record'] or {}).get('rig') or cfg['rig']
    # a profile whose arms have native controllers keeps an arm's home there (YAM + MakerMods Metal: the Metal's)
    homes = {s: (l.get('home') or {}).get('positions_rad') for s, l in (cfg.get('limbs') or {}).items() if isinstance(l, dict)}
    world = World(root, root / rig, a.scene, homes={s: q for s, q in homes.items() if q})
    active = [s for s in world.arms if not isinstance(world.spec['arms'][s].get('parked'), dict)]
    from roborun_sdk.robot import camera_socket
    sock = Path(a.camera_socket or camera_socket())
    cameras = Cameras(world, sock)
    cal = write_calibration(world, root)
    cameras.calibration_dir = cal.parent if cal else None
    cameras.serve()
    rules = None                                          # robot.yaml's `rules:`, as the real arms take them
    try:
        from roborun_sdk.arms import settings
        mine = settings.load(warn=lambda _: None)
        rules = settings._local(mine['rules']) if mine.get('rules') else None
    except SystemExit as exc:
        raise SystemExit(f'robot.yaml: {exc}')
    if rules:
        print(json.dumps(dict(event='mujoco_sim_rules', rules=rules)), flush=True)
    left = MuJoCoArm(world, 'left', rules) if 'left' in active else None
    right = MuJoCoArm(world, 'right', rules) if 'right' in active else None
    if left is None:
        raise SystemExit('the simulated station drives the left arm first: this profile parks it')
    station, srv = run_station(left, right, port=a.port, sessions_dir=a.sessions or str(root / 'log' / 'video-motion' / 'sessions'),
                               predict=a.predict)
    print(json.dumps(dict(event='mujoco_sim', profile=pid, arms=active, objects=[o['id'] for o in world.objects], camera_socket=str(sock),
                          station=srv.url)), flush=True)
    try:
        while True: time.sleep(3600)
    except KeyboardInterrupt:
        srv.close()
    return 0


def add_parser(sub):
    s = sub.add_parser('sim', help='simulate this robot\'s arms and cameras, e.g. two YAMs')
    s.add_argument('--profile', help='a profile of the runtime (default: the loaded one)')
    s.add_argument('--scene', default='cup', help='cup (default), pickup-bench, mixed-objects, cluttered-bench, low-light, or a .json list of objects')
    s.add_argument('--port', type=int, default=8091); s.add_argument('--camera-socket')
    s.add_argument('--sessions'); s.add_argument('--predict', default='http://127.0.0.1:8120/internal/video-motion/predict')
    s.add_argument('--runtime', dest='manifest')
    s.set_defaults(func=cmd_sim)
