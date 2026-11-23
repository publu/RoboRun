"""The simulated bodies: the profile's packaged MuJoCo model in a room, moved kinematically by the same velocity and head commands
a real body takes, with the head camera rendered from the model.

    sim = BodySim('booster_k1', scene='room')        # or 'galaxea_r1'; scenes: room (walls, a table, coloured boxes), empty
    sim.base.command(.3, 0., 0.); sim.base.state()   # BaseAdapter
    sim.head.command(.4, .2); sim.head.frame()       # HeadAdapter: JPEG, intrinsics, T_base_camera, eye height
    sim.close()

The base is not balanced or rolled by physics: a 100 Hz motion thread follows the commanded velocity within the profile's
acceleration limits, stops at its stop deceleration when the command is older than the deadman or on stop(), and does not
pass through the walls, the table or the boxes (its footprint is a circle). The head follows its target at the head rate.
Rendering runs on one thread that owns the GL context (MUJOCO_GL=egl or osmesa on a machine without a display).
"""
from __future__ import annotations

import math
import queue
import threading
import time

from .. import _optional
from . import profiles
from .adapters import BaseAdapter, HeadAdapter

np = _optional.require('numpy', 'sim')

MOTION_HZ = 100.
ROOM_X, ROOM_Y, ROOM_H, WALL_T = (-2.5, 4.5), (-3.0, 3.0), 2.6, .1
ROOM_START = (-1.2, 0., 0.)                                           # 2.6 m of floor to the table, in view ahead
TABLE = ((1.75, 0.), (.375, .6, .75), (.55, .38, .22, 1))                # centre xy, half sizes (x, y, full height), colour
BOXES = (('red_box', (1.1, 1.5), (.2, .2, .2), (.85, .15, .12, 1)),
         ('blue_box', (2.6, -1.6), (.25, .25, .3), (.15, .3, .8, 1)),
         ('green_box', (-1.2, 2.1), (.2, .3, .25), (.2, .65, .3, 1)),
         ('yellow_box', (3.6, 1.2), (.2, .2, .4), (.95, .8, .15, 1)))


def _mj():
    return _optional.require('mujoco', 'sim')


class BodySim:
    def __init__(self, profile_id, scene=profiles.DEFAULT_SCENE, start=None):
        if scene not in profiles.SCENES:
            raise ValueError(f'scene {scene!r}: one of {", ".join(profiles.SCENES)}')
        mujoco = _mj()
        self.profile_id, self.scene = profile_id, scene
        self.profile = profiles.load(profile_id)
        self.robot, self.motion, self.camera = self.profile['robot'], self.profile['motion'], self.profile['camera']
        self.size = tuple(int(v) for v in self.camera['size_px'])
        self.hfov = float(self.camera['hfov_deg'])
        self.fovy = math.degrees(2 * math.atan(math.tan(math.radians(self.hfov) / 2) * self.size[1] / self.size[0]))
        self.model, self.obstacles = self._build()
        self.data = mujoco.MjData(self.model)
        self._index()
        self.lock = threading.RLock()
        if start is None:                                     # facing the table with open floor first: a first 'drive forward 1 m' works
            start = ROOM_START if scene == 'room' else (0., 0., 0.)
        self.pose = np.array(start, float)                    # x, y, yaw in the room
        self.vel = np.zeros(3)                                # measured, in the base frame
        self.target = np.zeros(3); self.target_t = 0.; self.stopping = True
        self.blocked = False
        sim = self.profile['sim']; lim = self.robot['limits']
        pitch0 = float(self.camera['pitch_rad']) if self.robot['head_dof'] >= 1 else 0.
        self.head_q = np.array([profiles.clamp(pitch0, lim['head_pitch_rad']) if self.robot['head_dof'] >= 1 else 0., 0.])
        self.head_target = self.head_q.copy()
        self.footprint = float(sim.get('footprint_m', .25))
        self._settle()
        self._renders = queue.Queue()
        self._closed = threading.Event()
        threading.Thread(target=self._render_loop, name='body-sim-render', daemon=True).start()
        threading.Thread(target=self._motion_loop, name='body-sim-motion', daemon=True).start()
        self.base = SimBase(self)
        self.head = SimHead(self)

    # ------------------------------------------------------------------------------------------------ the model and room
    def _build(self):
        mujoco = _mj()
        sim = self.profile['sim']
        spec = mujoco.MjSpec.from_file(str(profiles.DATA / sim['model']))
        for geom in list(spec.geoms):                         # the model's own floor and lights make way for the room
            if geom.name == 'ground':
                spec.delete(geom)
        for kind in (spec.lights, spec.textures, spec.materials):
            for item in list(kind):
                if getattr(item, 'name', '') not in ('body', 'dark', 'accent'):
                    spec.delete(item)
        spec.option.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        if sim.get('head_camera_frame'):                      # an optical frame (z forward, y down): the camera looks along its -z
            spec.body(sim['head_camera_frame']).add_camera(name='head', pos=[0, 0, 0], quat=[0, 1, 0, 0], fovy=self.fovy)
        else:
            cam = spec.camera(sim['head_camera'])
            if cam is None:
                raise ValueError(f'{self.profile_id}: no camera {sim["head_camera"]!r} in its model {sim["model"]} (sim.head_camera)')
            cam.fovy = self.fovy
            if sim['head_camera'] != 'head':
                cam.name = 'head'                             # the name every part of the simulation looks it up by
        world = spec.worldbody
        world.add_light(name='sun', pos=[1, 0, 6], dir=[0, .2, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, diffuse=[.75, .75, .72], castshadow=False)
        world.add_light(name='fill', pos=[-1, 1, 3], dir=[.3, -.2, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, diffuse=[.35, .35, .38], castshadow=False)
        spec.visual.headlight.ambient = [.35, .35, .35]
        floor = spec.add_texture(name='floor_tex', type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                                 rgb1=[.62, .6, .56], rgb2=[.55, .53, .5], width=256, height=256)
        spec.add_material(name='floor_mat', textures=['', 'floor_tex'], texrepeat=[8, 8], texuniform=True)
        world.add_geom(name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[12, 12, .1], material='floor_mat')
        obstacles = []
        if self.scene == 'room':
            cx, cy = sum(ROOM_X) / 2, sum(ROOM_Y) / 2; hx, hy = (ROOM_X[1] - ROOM_X[0]) / 2, (ROOM_Y[1] - ROOM_Y[0]) / 2
            walls = (((cx, ROOM_Y[1] + WALL_T), (hx + WALL_T, WALL_T)), ((cx, ROOM_Y[0] - WALL_T), (hx + WALL_T, WALL_T)),
                     ((ROOM_X[1] + WALL_T, cy), (WALL_T, hy)), ((ROOM_X[0] - WALL_T, cy), (WALL_T, hy)))
            for i, ((x, y), (sx, sy)) in enumerate(walls):
                world.add_geom(name=f'wall_{i}', type=mujoco.mjtGeom.mjGEOM_BOX, pos=[x, y, ROOM_H / 2], size=[sx, sy, ROOM_H / 2],
                               rgba=[.86, .85, .82, 1])
                obstacles.append((x, y, sx, sy))
            world.add_geom(name='ceiling', type=mujoco.mjtGeom.mjGEOM_BOX, pos=[cx, cy, ROOM_H + WALL_T / 2],
                           size=[hx + WALL_T, hy + WALL_T, WALL_T / 2], rgba=[.93, .93, .92, 1])
            (tx, ty), (sx, sy, h), rgba = TABLE
            world.add_geom(name='table_top', type=mujoco.mjtGeom.mjGEOM_BOX, pos=[tx, ty, h - .02], size=[sx, sy, .02], rgba=list(rgba))
            for dx in (-1, 1):
                for dy in (-1, 1):
                    world.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, pos=[tx + dx * (sx - .04), ty + dy * (sy - .04), (h - .04) / 2],
                                   size=[.025, .025, (h - .04) / 2], rgba=list(rgba))
            obstacles.append((tx, ty, sx, sy))
            for name, (x, y), (sx, sy, sz), rgba in BOXES:
                world.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, pos=[x, y, sz / 2], size=[sx / 2, sy / 2, sz / 2], rgba=list(rgba))
                obstacles.append((x, y, sx / 2, sy / 2))
        spec.visual.global_.offwidth = max(1280, self.size[0])
        spec.visual.global_.offheight = max(1024, self.size[1])
        spec.visual.map.znear = .01
        spec.visual.map.zfar = 40.
        return spec.compile(), obstacles

    def _index(self):
        mujoco = _mj(); m = self.model; sim = self.profile['sim']
        free = [j for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
        if not free:
            raise ValueError(f'{self.profile_id}: its model {sim["model"]} has no free joint: give the base body <freejoint/>')
        self.q_base = int(m.jnt_qposadr[free[0]])
        self.cam = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, 'head')
        if self.cam < 0:
            raise ValueError(f'{self.profile_id}: no camera {sim.get("head_camera")!r} in its model (sim.head_camera)')
        if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, sim['base_body']) < 0:
            raise ValueError(f'{self.profile_id}: no body {sim["base_body"]!r} in its model (sim.base_body)')
        joint = lambda n: int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)])
        self.q_head = {k: joint(n) for k, n in (sim.get('head_joints') or {}).items()}
        self.stance = {joint(n): float(v) for n, v in (sim.get('stance') or {}).items()}
        base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, sim['base_body'])
        self.robot_geoms = [g for g in range(m.ngeom) if self._under(int(m.geom_bodyid[g]), base)]

    def _under(self, body, root):
        m = self.model
        while body > 0:
            if body == root:
                return True
            body = int(m.body_parentid[body])
        return False

    def _settle(self):
        """The base's height: the body's lowest point on the floor."""
        mujoco = _mj(); m, d = self.model, self.data
        d.qpos[:] = m.qpos0
        for adr, v in self.stance.items():
            d.qpos[adr] = v
        d.qpos[self.q_base:self.q_base + 7] = [0, 0, 1., 1, 0, 0, 0]
        mujoco.mj_kinematics(m, d)
        low = math.inf
        for g in self.robot_geoms:
            c, h = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
            R, p = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        low = min(low, float((p + R @ (c + h * np.array([sx, sy, sz])))[2]))
        self.base_z = 1. - low

    def _qpos(self):
        """The model's qpos at the current pose and head (call under the lock)."""
        q = self.model.qpos0.copy()
        for adr, v in self.stance.items():
            q[adr] = v
        x, y, yaw = self.pose
        q[self.q_base:self.q_base + 7] = [x, y, self.base_z, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        if 'pitch' in self.q_head:
            q[self.q_head['pitch']] = self.head_q[0]
        if 'yaw' in self.q_head:
            q[self.q_head['yaw']] = self.head_q[1]
        return q

    # ------------------------------------------------------------------------------------------------ motion
    def _free(self, x, y):
        r = self.footprint
        for ox, oy, sx, sy in self.obstacles:
            dx, dy = max(abs(x - ox) - sx, 0.), max(abs(y - oy) - sy, 0.)
            if dx * dx + dy * dy < r * r:
                return False
        return True

    def _motion_loop(self):
        dt = 1. / MOTION_HZ
        acc = np.array(list(self.motion['accel_m_s2']) + [self.motion['accel_yaw_rad_s2']], float)
        brake = np.array([self.motion['stop_decel_m_s2']] * 2 + [self.motion['stop_decel_yaw_rad_s2']], float)
        deadman = float(self.motion['deadman_s'])
        rate = float(self.robot['limits'].get('head_rate_rad_s') or 1.5)
        last = time.monotonic()
        while not self._closed.is_set():
            time.sleep(dt)
            now = time.monotonic(); step = min(now - last, .05); last = now
            with self.lock:
                fresh = not self.stopping and now - self.target_t <= deadman
                want = self.target if fresh else np.zeros(3)
                limit = (acc if fresh else brake) * step
                self.vel += np.clip(want - self.vel, -limit, limit)
                x, y, yaw = self.pose
                c, s = math.cos(yaw), math.sin(yaw)
                nx = x + (c * self.vel[0] - s * self.vel[1]) * step
                ny = y + (s * self.vel[0] + c * self.vel[1]) * step
                if self._free(nx, ny):
                    self.pose[:2] = nx, ny; self.blocked = False
                else:                                          # a wall, the table or a box: the body stops against it
                    self.vel[:2] = 0.; self.blocked = True
                self.pose[2] = (yaw + self.vel[2] * step + math.pi) % (2 * math.pi) - math.pi
                self.head_q += np.clip(self.head_target - self.head_q, -rate * step, rate * step)

    def command(self, vx, vy, vyaw):
        lim = self.robot['limits']
        with self.lock:
            self.target = np.array([profiles.clamp(vx, lim['vx_m_s']), profiles.clamp(vy, lim['vy_m_s']),
                                    profiles.clamp(vyaw, lim['vyaw_rad_s'])])
            self.target_t = time.monotonic(); self.stopping = False

    def stop(self):
        with self.lock:
            self.target = np.zeros(3); self.stopping = True

    def state(self):
        with self.lock:
            x, y, yaw = (float(v) for v in self.pose)
            vx, vy, vyaw = (float(v) for v in self.vel)
            cmd = dict(vx=float(self.target[0]), vy=float(self.target[1]), vyaw=float(self.target[2]))
            return dict(x=x, y=y, yaw=yaw, vx=vx, vy=vy, vyaw=vyaw, commanded=cmd, blocked=self.blocked, t=time.time())

    def look(self, pitch, yaw):
        lim, dof = self.robot['limits'], self.robot['head_dof']
        with self.lock:
            if dof >= 1:
                self.head_target[0] = profiles.clamp(pitch, lim['head_pitch_rad'])
            if dof >= 2:
                self.head_target[1] = profiles.clamp(yaw, lim['head_yaw_rad'])

    def head_state(self):
        with self.lock:
            if self.robot['head_dof'] == 0:
                return dict(pitch=float(self.camera['pitch_rad']), yaw=0.)
            return dict(pitch=float(self.head_q[0]), yaw=float(self.head_q[1]))

    # ------------------------------------------------------------------------------------------------ the head camera
    def _render_loop(self):
        mujoco = _mj()
        renderer, data = None, mujoco.MjData(self.model)
        while True:
            job = self._renders.get()
            if job is None:
                break
            done, box, qpos, pose = job[:4]
            want_depth = len(job) > 4 and job[4]
            try:
                if renderer is None:
                    renderer = mujoco.Renderer(self.model, height=self.size[1], width=self.size[0])
                data.qpos[:] = qpos
                mujoco.mj_kinematics(self.model, data)
                mujoco.mj_camlight(self.model, data)
                renderer.update_scene(data, camera=self.cam)
                if want_depth:                                # metres along the optical axis, for the obstacle guard
                    renderer.enable_depth_rendering()
                    try:
                        img = renderer.render().copy()
                    finally:
                        renderer.disable_depth_rendering()
                else:
                    img = renderer.render().copy()
                R_mj, p = data.cam_xmat[self.cam].reshape(3, 3).copy(), data.cam_xpos[self.cam].copy()
                box['out'] = (img, R_mj, p, pose)
            except Exception as exc:                          # no GL here: said once, to the caller
                box['error'] = exc
            finally:
                done.set()
        if renderer is not None:
            renderer.close()

    def _render(self, depth=False):
        with self.lock:
            qpos, pose = self._qpos(), self.pose.copy()
        done, box = threading.Event(), {}
        self._renders.put((done, box, qpos, pose, depth))
        if not done.wait(30.):
            raise RuntimeError('the simulated head camera did not render in 30 s')
        if 'error' in box:
            raise RuntimeError(f'the simulated head camera cannot render here ({box["error"]}): set MUJOCO_GL=egl or osmesa')
        img, R_mj, p, pose = box['out']
        R_cv = R_mj @ np.diag([1., -1., -1.])
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        R_wb = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
        T = np.eye(4); T[:3, :3] = R_wb.T @ R_cv; T[:3, 3] = R_wb.T @ (p - np.array([x, y, 0.]))
        return img, T, pose

    def depth(self):
        """The head camera's depth for the obstacle guard: {depth_m (HxW metres), K, T_base_camera, odom}."""
        depth, T, pose = self._render(depth=True)
        intr = profiles.intrinsics(self.size[0], self.size[1], self.hfov)
        return dict(depth_m=np.asarray(depth, np.float32), K=dict(fx=intr['fx'], fy=intr['fy'], cx=intr['ppx'], cy=intr['ppy']),
                    T_base_camera=T, odom=dict(x=float(pose[0]), y=float(pose[1]), yaw=float(pose[2])))

    def frame(self):
        with self.lock:
            qpos, pose, head = self._qpos(), self.pose.copy(), self.head_q.copy()
        done, box = threading.Event(), {}
        self._renders.put((done, box, qpos, pose))
        if not done.wait(30.):
            raise RuntimeError('the simulated head camera did not render in 30 s')
        if 'error' in box:
            raise RuntimeError(f'the simulated head camera cannot render here ({box["error"]}): set MUJOCO_GL=egl or osmesa')
        rgb, R_mj, p, pose = box['out']
        R_cv = R_mj @ np.diag([1., -1., -1.])                 # MuJoCo camera (x right, y up, z back) -> OpenCV (x right, y down, z forward)
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        R_wb = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
        T = np.eye(4); T[:3, :3] = R_wb.T @ R_cv; T[:3, 3] = R_wb.T @ (p - np.array([x, y, 0.]))
        hs = self.head_state()
        return dict(jpeg=encode_jpeg(rgb), width=self.size[0], height=self.size[1],
                    intrinsics=profiles.intrinsics(self.size[0], self.size[1], self.hfov),
                    T_base_camera=[[round(float(v), 6) for v in row] for row in T], eye_height_m=round(float(T[2, 3]), 4),
                    pitch=hs['pitch'], yaw=hs['yaw'])

    def close(self):
        if not self._closed.is_set():
            self._closed.set()
            self._renders.put(None)


class SimBase(BaseAdapter):
    def __init__(self, sim):
        self.sim, self.kind = sim, sim.robot['base']

    def command(self, vx, vy, vyaw):
        self.sim.command(vx, vy, vyaw)

    def stop(self):
        self.sim.stop()

    def state(self):
        return self.sim.state()


class SimHead(HeadAdapter):
    def __init__(self, sim):
        self.sim, self.dof = sim, sim.robot['head_dof']

    def command(self, pitch, yaw):
        self.sim.look(pitch, yaw)

    def state(self):
        return self.sim.head_state()

    def frame(self):
        return self.sim.frame()

    def depth(self):
        return self.sim.depth()

    def close(self):
        self.sim.close()


def encode_jpeg(rgb, quality=90):
    try:
        import cv2
        ok, buf = cv2.imencode('.jpg', np.ascontiguousarray(rgb[:, :, ::-1]), [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if ok:
            return buf.tobytes()
    except ImportError:
        pass
    import io
    from PIL import Image
    out = io.BytesIO(); Image.fromarray(rgb).save(out, 'JPEG', quality=quality)
    return out.getvalue()
