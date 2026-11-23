"""The body equivalents of ArmAdapter: a BaseAdapter (a legged or wheeled base that takes a velocity) and a HeadAdapter (the
head's pitch and yaw, and the head camera's frame with its intrinsics). Write your own by subclassing them; the simulation
(body.sim) and the robot-side agents below are implementations.

    base.command(vx, vy, vyaw)    the base's velocity target (m/s, m/s, rad/s; +x forward, +y left, +yaw counter-clockwise).
                                  A target is fresh for the profile's deadman (0.3 s): keep sending it, or the base stops
    base.stop()                   zero velocity now (the base stands / brakes; never a power-off)
    base.state()                  {x, y, yaw (odometry), vx, vy, vyaw (measured), commanded: {vx, vy, vyaw}, t}
    head.command(pitch, yaw)      head targets (rad; + pitch looks down, + yaw looks left), within the profile's limits
    head.state()                  {pitch, yaw}
    head.frame()                  {jpeg: bytes, width, height, intrinsics: {width, height, fx, fy, ppx, ppy},
                                   T_base_camera (4x4, OpenCV camera axes in the floor-level base frame) or None,
                                   eye_height_m, pitch, yaw}. Only jpeg is required: what is left out comes from the
                                   body's profile (camera.hfov_deg, eye_height_m, pitch_rad; yaw 0)

Run your own with `roborun up --profile <your profile> --body my_body.py:MyBody` (body/custom.py says what the class may be;
your profile goes in <ROBORUN_HOME>/bodies, body/profiles.py).

A real Booster K1 or Galaxea R1 is reached through a small agent running on the robot (HTTP, with its own key, deadman and
caps): AgentBody(profile, url, key) gives its base and head. The agent's routes are in the profile (data/bodies/<id>.json).
"""
from __future__ import annotations

import base64
import json
import threading
import time
import urllib.error
import urllib.request

from . import profiles


class BaseAdapter:
    """A mobile base driven by velocity. Subclasses implement command, stop and state."""
    kind = 'legged'                       # legged | wheeled

    def command(self, vx: float, vy: float, vyaw: float) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def state(self) -> dict:
        raise NotImplementedError

    def close(self) -> None:
        try:
            self.stop()
        except Exception:
            pass


class HeadAdapter:
    """A head with a camera. `dof` 0 (fixed), 1 (pitch) or 2 (pitch + yaw)."""
    dof = 2

    def command(self, pitch: float, yaw: float) -> None:
        raise NotImplementedError

    def state(self) -> dict:
        raise NotImplementedError

    def frame(self) -> dict:
        raise NotImplementedError

    def depth(self):
        """The head camera's depth for the obstacle guard, or None when this head has none: {depth_m (HxW metres along the
        optical axis, 0 = no data), K {fx, fy, cx, cy}, T_base_camera (4x4), odom {x, y, yaw} when it was taken}. Without
        depth a body still moves, with no obstacle guard and at most the blind-travel limit per move."""
        return None

    def close(self) -> None:
        pass


class AgentError(RuntimeError):
    pass


class _Agent:
    """HTTP to the robot-side agent: X-Manifest-Agent-Key on every request, never an Origin."""

    def __init__(self, url, key, timeout=1.5):
        self.url, self.key, self.timeout = url.rstrip('/'), key, timeout

    def call(self, method, path, body=None, timeout=None):
        req = urllib.request.Request(self.url + path, method=method, data=json.dumps(body).encode() if body is not None else None,
                                     headers={'X-Manifest-Agent-Key': self.key, 'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=timeout or self.timeout) as r:
                return json.loads(r.read() or b'{}')
        except urllib.error.HTTPError as e:
            try:
                why = json.loads(e.read() or b'{}')
            except ValueError:
                why = {}
            raise AgentError(f"the robot refused {path}: {why.get('refused') or why.get('error') or e.code}") from None
        except (urllib.error.URLError, OSError, ValueError) as e:
            raise AgentError(f'the robot is not answering at {self.url}: {getattr(e, "reason", e)}') from None


class AgentBody:
    """A real body through its agent. `base` and `head` share one connection; for the Booster agent the head target travels
    with every base command (POST /cmd), for the Galaxea agent the base has POST /base and the head is fixed."""

    def __init__(self, profile_id, url, key):
        self.profile = profiles.load(profile_id)
        self.protocol = self.profile['agent']['protocol']
        self.agent = _Agent(url, key)
        self.head_target = (float(self.profile['camera']['pitch_rad']), 0.)
        self.lock = threading.Lock()
        self.base = _AgentBase(self)
        self.head = _AgentHead(self)

    def check(self):
        """The agent answers (raises AgentError otherwise): a real body is never replaced by a simulation."""
        return self.agent.call('GET', '/state', timeout=3.)


class _AgentBase(BaseAdapter):
    def __init__(self, body):
        self.body = body
        self.kind = body.profile['robot']['base']
        self.commanded = dict(vx=0., vy=0., vyaw=0.)

    def command(self, vx, vy, vyaw):
        self.commanded = dict(vx=float(vx), vy=float(vy), vyaw=float(vyaw))
        if self.body.protocol == 'booster':
            pitch, yaw = self.body.head_target
            self.body.agent.call('POST', '/cmd', dict(self.commanded, head_pitch=pitch, head_yaw=yaw))
        else:
            self.body.agent.call('POST', '/base', dict(self.commanded))

    def stop(self):
        self.commanded = dict(vx=0., vy=0., vyaw=0.)
        self.body.agent.call('POST', '/stop', {})

    def state(self):
        s = self.body.agent.call('GET', '/state')
        odom = s.get('odom') or {}
        return dict(x=float(odom.get('x', 0.)), y=float(odom.get('y', 0.)), yaw=float(odom.get('yaw', 0.)),
                    vx=float(odom.get('vx', 0.)), vy=float(odom.get('vy', 0.)), vyaw=float(odom.get('vyaw', 0.)),
                    commanded=dict(self.commanded), t=time.time())


class _AgentHead(HeadAdapter):
    def __init__(self, body):
        self.body = body
        self.dof = body.profile['robot']['head_dof']

    def command(self, pitch, yaw):
        if self.dof == 0:
            return
        lim = self.body.profile['robot']['limits']
        self.body.head_target = (profiles.clamp(pitch, lim['head_pitch_rad']),
                                 profiles.clamp(yaw, lim['head_yaw_rad']) if self.dof == 2 else 0.)
        if self.body.protocol == 'booster':          # the head moves with the next /cmd; send one now with the base's target
            c = self.body.base.commanded
            pitch, yaw = self.body.head_target
            self.body.agent.call('POST', '/cmd', dict(c, head_pitch=pitch, head_yaw=yaw))

    def state(self):
        if self.dof == 0:
            return dict(pitch=float(self.body.profile['camera']['pitch_rad']), yaw=0.)
        s = self.body.agent.call('GET', '/state')
        head = s.get('head') or {}
        pitch, yaw = self.body.head_target
        return dict(pitch=float(head.get('pitch', pitch)), yaw=float(head.get('yaw', yaw)))

    def frame(self):
        cam = self.body.profile['camera']
        if self.body.protocol == 'booster':
            f = self.body.agent.call('GET', '/frame?stride=1', timeout=5.)
            jpeg = base64.b64decode(f['rgb_jpg_b64'])
        else:
            f = self.body.agent.call('GET', '/frame?camera=head&stride=1', timeout=5.)
            jpeg = base64.b64decode(f['rgb_b64'])
        width, height = _jpeg_size(jpeg)
        K = f.get('K')
        if isinstance(K, list) and len(K) == 9:
            intr = dict(width=width, height=height, fx=float(K[0]), fy=float(K[4]), ppx=float(K[2]), ppy=float(K[5]))
        else:
            intr = profiles.intrinsics(width, height, cam['hfov_deg'])
        head = self.state()
        return dict(jpeg=jpeg, width=width, height=height, intrinsics=intr, T_base_camera=None,
                    eye_height_m=float(cam['eye_height_m']), pitch=head['pitch'], yaw=head['yaw'])

    def depth(self):
        """Head depth when the robot's agent sends it (the R1 agent does: depth_zlib_b64, depth_size, depth_unit_m, K,
        T_base_cam); None otherwise."""
        import zlib
        import numpy as np
        path = '/frame?stride=1' if self.body.protocol == 'booster' else '/frame?camera=head&stride=1'   # full density: the guard needs it
        try:
            f = self.body.agent.call('GET', path, timeout=2.)
        except Exception:
            return None
        if not f.get('depth_zlib_b64') or not f.get('depth_size') or not isinstance(f.get('K'), list) or f.get('T_base_cam') is None:
            return None
        w, h = (int(v) for v in f['depth_size'])
        mm = np.frombuffer(zlib.decompress(base64.b64decode(f['depth_zlib_b64'])), np.uint16).reshape(h, w)
        K = f['K']
        T = f['T_base_cam']
        if isinstance(T, dict):                               # {xyz, xyzw}: the camera's pose in the base frame, or {error}
            if not T.get('xyz') or not T.get('xyzw'):
                return None
            T = _pose_matrix(T['xyz'], T['xyzw'])
        odom = self.body.agent.call('GET', '/state').get('odom') or {}
        return dict(depth_m=mm.astype(np.float32) * float(f.get('depth_unit_m') or .001),
                    K=dict(fx=float(K[0]), fy=float(K[4]), cx=float(K[2]), cy=float(K[5])), T_base_camera=T,
                    odom=dict(x=float(odom.get('x', 0.)), y=float(odom.get('y', 0.)), yaw=float(odom.get('yaw', 0.))))


def _pose_matrix(xyz, q):
    x, y, z, w = (float(v) for v in q)
    R = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    return [R[0] + [float(xyz[0])], R[1] + [float(xyz[1])], R[2] + [float(xyz[2])], [0., 0., 0., 1.]]


def _jpeg_size(data):
    """(width, height) from a JPEG's SOF marker."""
    i = 2
    while i + 9 < len(data):
        if data[i] != 0xFF:
            i += 1; continue
        marker = data[i + 1]
        if marker in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
            return int.from_bytes(data[i + 7:i + 9], 'big'), int.from_bytes(data[i + 5:i + 7], 'big')
        i += 2 + int.from_bytes(data[i + 2:i + 4], 'big')
    raise ValueError('not a JPEG with a frame header')
