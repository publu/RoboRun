"""The policy runner's side of the station (127.0.0.1:8091) and of the cameras: lease, push, heartbeat, end; pose reads turned
into Observations (grippers as open fractions); camera frames as JPEGs, blurred for a remote provider. Standard library at
import; numpy and OpenCV only when frames are read."""
from __future__ import annotations

import json
import time
import urllib.error
import urllib.request

from .provider import ArmInfo, ArmState, Frame, Observation, PolicyRefused

POSE_MAX_AGE_S = 0.2          # an older pose sample is stale: no act() on it
FRAME_MAX_AGE_S = 0.2         # an older camera frame skips the tick
HEADERS = {"Content-Type": "application/json", "X-Manifest-Supervisor": "1"}

# The arm models and registry semantics of the premade profiles, by the runtime's profile id (robot.json `profile`).
PREMADE = {
    'yam': ('yam', 'yam'), 'yam_solo': ('yam', None), 'yam_metal': ('yam', 'metal_arm'),
    'makerarm': ('maker_arm', 'maker_arm'), 'maker_arm_dual': ('maker_arm', 'maker_arm'),
}
SEMANTICS = {'yam': 'yam-linear-4310.rad6.jaw-open-fraction.v1', 'metal_arm': 'makermods-metal.rad6.jaw-motor-rad.v1'}


def fraction(jaw, jaw_range):
    from roborun_station.policy import fraction_from_jaw, jaw_range_usable
    if jaw is None or not jaw_range_usable(jaw_range):
        return None
    return fraction_from_jaw(jaw, jaw_range)


class StationRefused(Exception):
    def __init__(self, code, reply):
        super().__init__(reply.get('error') or reply.get('refused') or f'HTTP {code}')
        self.code, self.reply = code, reply


class Station:
    def __init__(self, url, timeout=2.0):
        self.url, self.timeout = url.rstrip('/'), timeout

    def _call(self, method, path, body=None, timeout=None):
        data = None if body is None else json.dumps(body, allow_nan=False).encode()
        req = urllib.request.Request(self.url + path, data=data, method=method, headers=HEADERS)
        try:
            with urllib.request.urlopen(req, timeout=timeout or self.timeout) as reply:
                return json.loads(reply.read() or b'{}')
        except urllib.error.HTTPError as exc:
            try:
                payload = json.loads(exc.read() or b'{}')
            except ValueError:
                payload = {}
            raise StationRefused(exc.code, payload if isinstance(payload, dict) else {}) from None

    def pose(self):
        return self._call('GET', '/supervisor/pose')

    def observe(self):
        return self._call('POST', '/supervisor/observe', {})

    def lease(self, arms, seconds, run_id, *, gripper=True, max_tool_speed_m_s=None, since_hold_generation=None):
        oid = self.observe()['observation_id']
        ref = dict(kind='policy', arms=list(arms), run_id=run_id, gripper=gripper)
        if since_hold_generation is not None:         # the station refuses the lease when a Stop came after this
            ref['since_hold_generation'] = int(since_hold_generation)
        if max_tool_speed_m_s is not None:
            ref['max_tool_speed_m_s'] = float(max_tool_speed_m_s)
        return self._call('POST', '/supervisor/action', dict(observation_id=oid, seconds=float(seconds), reference=ref,
                                                             instruction='policy run'))

    def heartbeat(self, lease_id):
        return self._call('POST', '/supervisor/heartbeat', dict(lease_id=lease_id), timeout=1.0)

    def chunk(self, body):
        """(accepted?, reply). A refusal (409 with its reason) is a normal answer here."""
        try:
            return True, self._call('POST', '/supervisor/policy/chunk', body)
        except StationRefused as exc:
            if exc.code == 409 and 'refused' in exc.reply:
                return False, exc.reply
            raise

    def end(self, lease_id, reason):
        return self._call('POST', '/supervisor/policy/end', dict(lease_id=lease_id, reason=reason))


# ---- the arms ----------------------------------------------------------------------------------------------------------
def arm_models(config):
    """{side: (catalog model, semantics)} from robot.json's profile, as far as it says."""
    profile = (config or {}).get('profile') or ''
    pair = PREMADE.get(profile)
    if pair is None:
        try:
            from ..arm_profiles import parse
            pair = parse(profile)
        except Exception:
            pair = None
    out = {}
    for side, model in zip(('left', 'right'), pair or (None, None)):
        if model:
            out[side] = (model, SEMANTICS.get(model))
    return out


def hold_generation(reply):
    """The station's Stop counter from a pose, observe or status reply (None from a station that does not report it)."""
    if not isinstance(reply, dict):
        return None
    for got in (reply.get('hold_generation'), (reply.get('status') or {}).get('hold_generation')):
        if isinstance(got, int) and not isinstance(got, bool):
            return got
    return None


def arms_of(observed, config):
    """ArmInfo of every arm the station has (parked arms left out), left first."""
    status = observed.get('status') or {}
    parked = set(status.get('parked_arms') or [])
    measured = observed.get('measured') or {}
    limits = observed.get('limits') or {}
    jaws = observed.get('jaw_range') or {}
    models = arm_models(config)
    out = []
    for side in ('left', 'right'):
        row = measured.get(side)
        if side in parked or not row:
            continue
        q = tuple(float(v) for v in row.get('positions_rad') or ())
        from roborun_station.policy import jaw_range_usable
        jr = tuple(jaws.get(side) or ()) or None
        model, semantics = models.get(side, ('unknown', None))
        joint_limits = tuple(tuple(x) for x in limits.get(side) or ())[:len(q)]
        if joint_limits and len(joint_limits) != len(q):
            raise PolicyRefused(f'the station reports {len(q)} joints on {side} but {len(joint_limits)} limits')
        out.append(ArmInfo(side=side, model=model, semantics=semantics, joints=len(q), limits_rad=joint_limits,
                           gripper=jr is not None and jaw_range_usable(jr), home_rad=q, jaw_range=jr))
    return out


def arm_state(row, jaw_range):
    return ArmState(q_rad=tuple(float(v) for v in row['positions_rad']), gripper=fraction(row.get('gripper'), jaw_range),
                    commanded_rad=tuple(float(v) for v in row['commanded_rad']),
                    commanded_gripper=fraction(row.get('commanded_gripper'), jaw_range))


def fresh_rows(pose, sides, max_age_s=POSE_MAX_AGE_S):
    """{side: row} when every side has a sample younger than max_age_s, else None (stale: no act on it)."""
    measured = pose.get('measured') or {}
    arms = pose.get('arms') or {}
    out = {}
    for side in sides:
        row = measured.get(side)
        if row is None or (arms.get(side) or {}).get('state') == 'unavailable':
            return None
        age = row.get('age_ms')
        if age is None or not 0 <= float(age) / 1000.0 < max_age_s:
            return None
        out[side] = row
    return out


# ---- cameras -----------------------------------------------------------------------------------------------------------
def jpeg(rgb, size=None, quality=85):
    """JPEG bytes of an RGB uint8 image, downscaled to size=(w, h) first when given: (bytes, w, h)."""
    import cv2
    if size is not None and (rgb.shape[1], rgb.shape[0]) != tuple(size):
        rgb = cv2.resize(rgb, tuple(int(v) for v in size), interpolation=cv2.INTER_AREA)
    ok, data = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise ValueError('a frame could not be encoded')
    return data.tobytes(), int(rgb.shape[1]), int(rgb.shape[0])


class Cameras:
    """The camera broker's newest frames (read-only), by Roborun role."""

    def __init__(self, socket_path=None):
        if socket_path is None:
            from ..robot import camera_socket
            socket_path = camera_socket()
        self.socket_path = socket_path
        self._client = None

    def client(self):
        if self._client is None:
            from ..connectors.realsense import CameraServiceClient
            self._client = CameraServiceClient(self.socket_path, timeout_s=1.0)
        return self._client

    def roles(self):
        """The roles delivering frames now ([] when there is no camera service)."""
        try:
            status = self.client().status()
        except Exception:
            return []
        cams = status.get('cameras') or {}
        if 'available_roles' in status and not status.get('sim'):   # the robot's camera broker
            return [r for r in status['available_roles'] if (cams.get(r) or {}).get('state') == 'streaming']
        errors = status.get('errors') or {}             # the simulator's cameras: every role it renders, by frame age
        return [r for r, age in (status.get('frame_age_s') or {}).items()
                if r in ('top', 'left', 'right') and r not in errors and isinstance(age, (int, float)) and age < 5.0]

    def frame(self, role):
        return self.client().frame(role)


class Observer:
    """Builds Observations: one pose read (measured + commanded together), the newest frames, blur for a remote provider."""

    def __init__(self, station, arms, prompt, *, cameras=None, roles=(), frame_size=None, blur=False, clock=time.monotonic):
        self.station, self.arms, self.prompt = station, {a.side: a for a in arms}, prompt
        self.cameras, self.roles, self.frame_size, self.blur, self.clock = cameras, tuple(roles), frame_size, blur, clock
        self.blur_failed_since = None
        self.last_pose = None

    def tick(self):
        """An Observation, or (None, why) when this tick has none (stale pose or frame, blur unavailable)."""
        t = self.clock()
        pose = self.station.pose()
        self.last_pose = pose
        rows = fresh_rows(pose, self.arms)
        if rows is None:
            return None, 'stale_pose'
        arms = {side: arm_state(rows[side], self.arms[side].jaw_range) for side in self.arms}
        frames = {}
        for role in self.roles:
            try:
                got = self.cameras.frame(role)
            except Exception as exc:
                return None, f'camera {role}: {exc}'
            age = time.monotonic() - float(got.get('acquired_at_s', 0.0))
            if not 0 <= age < FRAME_MAX_AGE_S:
                return None, f'stale_frame:{role}'
            rgb = got['rgb']
            if self.blur:
                from .. import camera_privacy
                try:
                    rgb, _ = camera_privacy.clear_arrays(rgb, None, None)
                except camera_privacy.BlurUnavailable:
                    if self.blur_failed_since is None:
                        self.blur_failed_since = self.clock()
                    return None, 'blur_unavailable'
                self.blur_failed_since = None
            data, w, h = jpeg(rgb, self.frame_size)
            frames[role] = Frame(jpeg=data, width=w, height=h, captured_at=float(got['acquired_at_s']))
        return Observation(t=t, arms=arms, frames=frames, prompt=self.prompt), None
