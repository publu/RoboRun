"""The body-job client: a head-camera frame and a prompt -> the hosted API -> a signed 10 Hz locomotion timeline.

    jobs = BodyJobs('https://api.roborun.app', key)
    request = build_request('booster_k1', frame, 'walk to the table and stop in front of it', duration_s=4)
    result = jobs.run(request, frame['jpeg'], on_status=print, cancelled=event.is_set)
    result['locomotion']['rows']        # [{t, vx, vy, vyaw, head_pitch, head_yaw, confidence}, ...]

run() uploads the frame (PUT /v1/blobs/<sha256>), submits the job (POST /v1/motion/jobs, schema manifest.body-request.v1),
polls its status, then fetches the result and checks it before anything moves: its Ed25519 signature against the API's keys
(GET /v1/keys), that it is a locomotion_result for this job, and that its request_digest is this exact request. A reply that
fails any check raises BodyJobError and nothing is played.
"""
from __future__ import annotations

import hashlib
import json
import time
import uuid

from . import profiles
from .. import net
from ..rt import signing
from ..rt.canonical import canonical_bytes, unsigned_view

REQUEST_SCHEMA = 'manifest.body-request.v1'
RESULT_TYPE = 'locomotion_result'
LOCOMOTION_SCHEMA = 'manifest.locomotion.v1'
DURATIONS_S = (2, 3, 4, 6, 8)
TERMINAL = ('done', 'failed', 'cancelled')


class BodyJobError(RuntimeError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code, self.message = code, message


def new_id():
    return 'body-' + uuid.uuid4().hex[:20]


def build_request(profile_id, frame, prompt, *, duration_s=4, job_id=None):
    """manifest.body-request.v1 for this body and this head frame (frame: what HeadAdapter.frame() returns)."""
    if not isinstance(prompt, str) or not prompt.strip() or len(prompt) > 4000:
        raise BodyJobError('invalid_request', 'prompt: the words, up to 4000 characters')
    if duration_s not in DURATIONS_S:
        raise BodyJobError('invalid_request', f'duration_s: one of {", ".join(map(str, DURATIONS_S))}')
    p = profiles.load(profile_id)
    frame = complete_frame(p, frame)
    intr = dict(frame['intrinsics'])
    if (intr['width'], intr['height']) != (frame['width'], frame['height']):
        raise BodyJobError('invalid_request', 'the intrinsics are not the frame\'s size')
    camera = dict(name='head', role='egocentric', intrinsics=intr, eye_height_m=round(float(frame['eye_height_m']), 4),
                  head_pitch_rad=round(float(frame['pitch']), 5), head_yaw_rad=round(float(frame['yaw']), 5))
    if frame.get('T_base_camera') is not None:
        camera['T_base_camera'] = frame['T_base_camera']
    if p['camera'].get('lens'):
        camera['lens'] = p['camera']['lens']
    body = dict(schema=REQUEST_SCHEMA, id=job_id or new_id(), mode='body', prompt=prompt,
                frame=dict(source_sha256=hashlib.sha256(frame['jpeg']).hexdigest()), camera=camera, robot=p['robot'],
                options=dict(duration_s=int(duration_s)))           # the service picks the rest
    return body


def complete_frame(profile, frame):
    """A head frame with what a HeadAdapter may leave out filled in from the body's profile: width and height (from the JPEG),
    intrinsics (from camera.hfov_deg), eye_height_m (camera.eye_height_m), pitch (camera.pitch_rad) and yaw (0). What the
    frame gives is kept as given."""
    if not isinstance(frame, dict) or not isinstance(frame.get('jpeg'), (bytes, bytearray)):
        raise BodyJobError('robot_error', 'the head camera gave no frame: HeadAdapter.frame() returns {jpeg: bytes, ...}')
    out = dict(frame)
    cam = profile['camera']
    if out.get('width') is None or out.get('height') is None:
        from .adapters import _jpeg_size
        try:
            out['width'], out['height'] = _jpeg_size(out['jpeg'])
        except ValueError:
            raise BodyJobError('robot_error', 'the head camera\'s frame is not a JPEG') from None
    if out.get('intrinsics') is None:
        out['intrinsics'] = profiles.intrinsics(out['width'], out['height'], cam['hfov_deg'])
    if out.get('eye_height_m') is None:
        out['eye_height_m'] = cam['eye_height_m']
    if out.get('pitch') is None:
        out['pitch'] = cam.get('pitch_rad', 0.)
    if out.get('yaw') is None:
        out['yaw'] = 0.
    return out


def request_digest(request):
    """The digest the API binds its reply to: sha256 of the request's canonical JSON."""
    return hashlib.sha256(json.dumps(request, sort_keys=True, separators=(',', ':'), ensure_ascii=False,
                                     allow_nan=False).encode()).hexdigest()


def verify_result(msg, request, keys):
    """The signed reply, checked: signature (keys: {kid: 32-byte public key}), kind, job and request. Returns msg."""
    if not isinstance(msg, dict) or msg.get('type') != RESULT_TYPE:
        raise BodyJobError('bad_result', 'the reply is not a locomotion result')
    sig = msg.get('sig')
    if not isinstance(sig, dict) or sig.get('alg') != 'Ed25519' or not isinstance(sig.get('kid'), str):
        raise BodyJobError('bad_signature', 'the reply is not signed')
    public = keys.get(sig['kid'])
    if public is None:
        raise BodyJobError('unknown_key', f"the reply is signed with a key this robot does not trust ({sig['kid']})")
    try:
        value = signing.raw_bytes(sig.get('value', ''), 64)
        ok = signing._verify_raw(public, canonical_bytes(unsigned_view(msg)), value)
    except Exception:
        ok = False
    if not ok:
        raise BodyJobError('bad_signature', 'the reply\'s signature does not verify')
    loco = msg.get('locomotion')
    if msg.get('job') != request['id'] or not isinstance(loco, dict) or loco.get('job') != request['id']:
        raise BodyJobError('bad_result', 'the reply is for another job')
    if msg.get('request_digest') != request_digest(request):
        raise BodyJobError('bad_result', 'the reply is for another request')
    if loco.get('schema') != LOCOMOTION_SCHEMA or not isinstance(loco.get('rows'), list):
        raise BodyJobError('bad_result', 'the reply carries no locomotion timeline')
    return msg


def _deferred(status):
    """Only the parts of the request the service left for a next move (a list), else None: the status's own
    'deferred' list, or the one inside one of its sections. Nothing else of the status is kept."""
    found = status.get('deferred')
    if not isinstance(found, list):
        found = next((v['deferred'] for v in status.values() if isinstance(v, dict) and isinstance(v.get('deferred'), list)),
                     None)
    return found if isinstance(found, list) else None


class BodyJobs:
    def __init__(self, server, key, *, poll_s=1.5, timeout_s=600., http=None):
        self.server, self.key = server.rstrip('/'), key
        self.poll_s, self.timeout_s = poll_s, timeout_s
        self._keys = None
        self.http = http or self._http

    # ---------------------------------------------------------------------------------------------- transport
    def _http(self, method, path, body=None, raw=None, content_type='application/json'):
        # safe to repeat (net.request retries a dropped connection and 408/425/429/5xx): GET / HEAD / PUT, a submit (the
        # same id and body is the same job on the API, never run twice) and a cancel
        data = raw if raw is not None else (json.dumps(body).encode() if body is not None else None)
        retry = method != 'POST' or path == '/v1/motion/jobs' or path.endswith('/cancel')
        try:
            code, _, payload = net.request(method, self.server + path, self.key, data=data, timeout=60, retry=retry,
                                           headers={'Content-Type': content_type})
            return code, payload
        except net.Offline as e:
            raise BodyJobError('api_unreachable', f'the Roborun API could not be reached ({e.reason})') from None

    def _json(self, method, path, body=None, ok=(200,)):
        code, data = self.http(method, path, body)
        try:
            out = json.loads(data or b'{}')
        except ValueError:
            out = {}
        if code not in ok:
            err = out.get('error') if isinstance(out.get('error'), dict) else {}
            raise BodyJobError(err.get('code') or f'http_{code}', err.get('message') or f'{method} {path}: {code}')
        return out

    def keys(self, refresh=False):
        """{kid: public key} from GET /v1/keys (in date)."""
        if self._keys is None or refresh:
            self._keys = signing.keys_from_trusted(self._json('GET', '/v1/keys').get('keys') or [])
        return self._keys

    # ---------------------------------------------------------------------------------------------- one job
    def upload(self, data, content_type='image/jpeg'):
        sha = hashlib.sha256(data).hexdigest()
        code, _ = self.http('HEAD', f'/v1/blobs/{sha}')
        if code != 200:
            code, body = self.http('PUT', f'/v1/blobs/{sha}', raw=data, content_type=content_type)
            if code not in (200, 201):
                raise BodyJobError('upload_failed', f'the head frame was not accepted ({code})')
        return sha

    def submit(self, request):
        return self._json('POST', '/v1/motion/jobs', request, ok=(200, 202))

    def status(self, job_id):
        return self._json('GET', f'/v1/motion/jobs/{job_id}')

    def cancel(self, job_id):
        try:
            return self._json('POST', f'/v1/motion/jobs/{job_id}/cancel', {})
        except BodyJobError:
            return None

    def result(self, request):
        code, data = self.http('GET', f"/v1/motion/jobs/{request['id']}/result")
        if code != 200:
            raise BodyJobError('not_ready', f'the result is not available ({code})')
        msg = json.loads(data)
        try:
            return verify_result(msg, request, self.keys())
        except BodyJobError as e:
            if e.code != 'unknown_key':
                raise
            return verify_result(msg, request, self.keys(refresh=True))      # the API may have rotated its key

    def run(self, request, jpeg, *, on_status=None, cancelled=lambda: False, sleep=time.sleep):
        """Upload, submit, wait, fetch and verify. `cancelled()` true: the hosted job is cancelled and BodyJobError('cancelled')."""
        say = on_status or (lambda s: None)
        self.keys()
        say(dict(stage='uploading'))
        self.upload(jpeg)
        if cancelled():
            raise BodyJobError('cancelled', 'stopped before the job was sent')
        st = self.submit(request)
        say(dict(stage='queued', job=request['id']))
        deadline = time.monotonic() + self.timeout_s
        while st.get('status') not in TERMINAL:
            if cancelled():
                self.cancel(request['id'])
                raise BodyJobError('cancelled', 'stopped while the motion was being made')
            if time.monotonic() > deadline:
                self.cancel(request['id'])
                raise BodyJobError('timeout', 'the motion service took too long')
            sleep(self.poll_s)
            try:
                st = self.status(request['id'])
            except BodyJobError as e:       # the job runs on at the API: keep asking until the deadline, then cancel it
                if e.code not in ('api_unreachable', 'upstream_error', 'rate_limited') and not e.code.startswith('http_5'):
                    raise
                say(dict(stage='reconnecting', job=request['id']))
                continue
            say(dict(stage=st.get('stage') or st.get('status'), job=request['id'], deferred=_deferred(st)))
        if st['status'] != 'done':
            err = st.get('error') or {}
            raise BodyJobError(err.get('code') or st['status'], err.get('message') or f"the job {st['status']}")
        out = self.result(request)
        out['_status'] = st
        return out
