"""A local stand-in for DreamLabs' services, as dreamlabs-client 1.1.7 speaks to them (read from the client's source; the
real services are never called in tests). One ThreadingHTTPServer on 127.0.0.1 serves all of them:

- the gateway: POST /v1/get_action (a session minted on `session_id: null`, then echoed; the status envelope on every
  reply, HTTP 400 for DA-4000 and 200 otherwise, as the client reads only the envelope), POST /v1/end_session;
- the platform: GET /v1/models; GET /live/robot (the client's recorder) is answered 404 and logged;
- the data service: embodiments, datasets (register, complete, fail, status), fine-tuning jobs, training data;
- a minimal S3: PUT /<bucket>/<key> keeps the bytes.

Every request is logged (method, path, headers, body). `script` is a list of what the next get_action calls answer, in order
(then OK): {'code': 'DA-2000', 'reason': 'rate_limited', 'retryable': True, 'retry_after_s': 0.7}, {'delay_s': 2},
{'raw': b'not json'}, {'envelope': False}, {'chunk': fn(obs) -> {side: rows}}. Each reply is written in one write.
`latency_s` (lo, hi) delays every get_action by a random time in that range (the real round trip, see below).

What the real gateway does for dreamlabs/dream-1-beta, as seen with a real account on 2026-11-22 (made-up values here):
- the observation must carry state.<side>_eef_xyz [3], state.<side>_eef_rot6 [6] and state.<side>_grip [1] for both arms
  besides the joints and grippers, else HTTP 400 `DA-4000 BAD_REQUEST`, reason `malformed`, naming what is missing;
- a refusal carries `actions: null` and `timing: null` and still mints a session id;
- an answer's `actions` keys are q_r, q_l, q_r_chunk, q_l_chunk (48 x 7: six joints, then the gripper) and per_key with
  action.<side>_arm [48x6], action.<side>_eef_xyz [48x3], action.<side>_eef_rot6 [48x6] and action.<side>_grip [48] (a
  flat list): no action.<side>_gripper and no echo of the state. q_*_chunk equals per_key's arm and grip;
- session ids are UUIDs and trace ids 32 hex characters;
- a warm round trip is about 0.6 s at the median and 1.0 s at p90 (up to 1.3 s), a session's first one about 2 s.
Fine-tunes keep the reply below (action.<side>_gripper and an echo of the state): no real fine-tune reply has been seen.
"""
import json
import math
import secrets
import threading
import random
import time
import urllib.parse
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

NAMES = {'DA-0000': 'OK', 'DA-0001': 'OK_NEW_SESSION', 'DA-1000': 'UNAUTHORIZED', 'DA-2000': 'NO_CAPACITY',
         'DA-4000': 'BAD_REQUEST', 'DA-5000': 'MODEL_FAILED', 'DA-6000': 'DREAM_LABS_FAULT'}
H = 48
BASE_MODEL = 'dreamlabs/dream-1-beta'
EEF_STATE = ('eef_xyz', 'eef_rot6', 'grip')


def status(code='DA-0000', reason=None, retryable=False, retry_after_s=None, detail=None):
    """The envelope, keys in the real service's order."""
    return dict(code=code, trace_id=secrets.token_hex(16), reason=reason, retry_after_s=retry_after_s, detail=detail,
                name=NAMES.get(code, 'ERROR'), retryable=retryable)


def new_session():
    return str(uuid.uuid4())


def sway(obs, amplitude=0.02, t=0.0):
    """Rows around the measured pose: the last joint sways a little; the gripper stays where it is."""
    out = {}
    for side, key in (('right', 'q_r_chunk'), ('left', 'q_l_chunk')):
        arm, grip = obs.get(f'state.{side}_arm'), obs.get(f'state.{side}_gripper')
        if arm is None:
            continue
        g = float(grip[0]) if grip else 1.0
        rows = []
        for i in range(H):
            q = [float(v) for v in arm]
            q[-1] += amplitude * math.sin(2 * math.pi * (t + i / 30.0) / 2.0)
            rows.append(q + [g])
        out[side] = rows
    return out


class FakeDreamLabs:
    def __init__(self):
        self.requests = []
        self.script = []
        self.lock = threading.Lock()
        self.sessions = {}
        self.end_session_delay_s = 0.0
        self.models = [dict(id='dreamlabs/dream-1-beta', owner='public', base_model=None, embodiment='yam', servable=True,
                            created_at=None, cameras=['overhead', 'wrist_l', 'wrist_r'], embodiment_config=None)]
        self.embodiments = {}
        self.datasets = {}
        self.objects = {}
        self.jobs = {}
        self.fail_s3 = False
        self.latency_s = None
        self.user = 'tester'
        self.trained_prompts = ['fold the towel']
        self.t0 = time.monotonic()
        fake = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def log_message(self, *a):
                pass

            def _body(self):
                n = int(self.headers.get('Content-Length') or 0)
                data = self.rfile.read(n) if n else b''
                if self.headers.get('Transfer-Encoding', '').lower() == 'chunked':
                    data = b''
                    while True:
                        size = int(self.rfile.readline().strip().split(b';')[0], 16)
                        if size == 0:
                            while self.rfile.readline() not in (b'\r\n', b'\n', b''):
                                pass
                            break
                        data += self.rfile.read(size)
                        self.rfile.readline()
                return data

            def _send(self, code, body, ctype='application/json', extra=None):
                if isinstance(body, (dict, list)):
                    body = json.dumps(body).encode()
                head = [f'HTTP/1.1 {code} {"OK" if code < 400 else "Error"}', f'Content-Type: {ctype}',
                        f'Content-Length: {len(body)}']
                head += [f'{k}: {v}' for k, v in (extra or {}).items()]
                self.wfile.write(('\r\n'.join(head) + '\r\n\r\n').encode() + body)
                self.wfile.flush()

            def _handle(self, method):
                raw = self._body()
                try:
                    parsed = json.loads(raw) if raw and self.headers.get('Content-Type', '').startswith('application/json') else None
                except ValueError:
                    parsed = None
                path = urllib.parse.urlsplit(self.path).path
                with fake.lock:
                    fake.requests.append(dict(method=method, path=path, headers=dict(self.headers.items()), body=parsed,
                                              size=len(raw), raw=raw if path.startswith('/v1/') else b'', at=time.monotonic()))
                try:
                    fake.route(self, method, path, parsed, raw)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_GET(self):
                self._handle('GET')

            def do_POST(self):
                self._handle('POST')

            def do_PUT(self):
                self._handle('PUT')

            def do_DELETE(self):
                self._handle('DELETE')

            def do_HEAD(self):
                self._handle('HEAD')

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.server.daemon_threads = True
        self.url = f'http://127.0.0.1:{self.server.server_address[1]}'
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()

    def env(self, home, key='dl_test_key_not_real'):
        """The environment that points DreamLabs' client here (and never at a real file or service)."""
        return dict(DREAM_GATEWAY_URL=self.url, DREAM_API_URL=self.url, DREAM_DATA_URL=self.url, DREAM_API_KEY=key,
                    DREAM_CREDENTIALS=str(home / 'no-dreamlabs-credentials'), DREAM_LOG='off')

    def calls(self, path=None, method=None):
        with self.lock:
            return [r for r in self.requests if (path is None or r['path'] == path) and (method is None or r['method'] == method)]

    # ---- routes ----------------------------------------------------------------------------------------------------------
    def route(self, h, method, path, body, raw):
        if path == '/v1/get_action' and method == 'POST':
            return self.get_action(h, body, raw)
        if path == '/v1/end_session' and method == 'POST':
            if self.end_session_delay_s:
                time.sleep(self.end_session_delay_s)
            return h._send(200, dict(status=status()))
        if path == '/v1/models' and method == 'GET':
            return h._send(200, self.models)
        if path.startswith('/live/'):
            return h._send(404, dict(error=dict(code='not_found', message='no live service here')))
        if path.startswith('/v1/embodiments'):
            return self.embodiment(h, method, path, body)
        if path.startswith('/v1/datasets'):
            return self.dataset(h, method, path, body)
        if path.startswith('/v1/fine_tuning'):
            return self.fine_tuning(h, method, path, body)
        if method == 'PUT' and path.startswith('/bucket/'):
            if self.fail_s3:
                return h._send(500, b'<Error><Code>InternalError</Code><Message>fake</Message></Error>', 'application/xml')
            self.objects[path[len('/bucket/'):]] = raw
            return h._send(200, b'', 'application/xml', {'ETag': '"0123456789abcdef"'})
        if method == 'HEAD':
            return h._send(404, b'')
        return h._send(404, dict(error=dict(code='not_found', message=f'{method} {path}')))

    def get_action(self, h, body, raw):
        if body is None:
            return h._send(200, dict(status=status('DA-4000', 'malformed')))
        with self.lock:
            step = self.script.pop(0) if self.script else {}
        if self.latency_s:
            time.sleep(random.uniform(*self.latency_s))
        if step.get('delay_s'):
            time.sleep(step['delay_s'])
        if 'raw' in step:
            return h._send(200, step['raw'], 'text/plain')
        if step.get('envelope') is False:
            return h._send(200, dict(hello='world'))
        sid = body.get('session_id')
        obs = body.get('observation') or {}
        base = body.get('checkpoint') == BASE_MODEL
        if step.get('code') and step['code'] not in ('DA-0000', 'DA-0001'):
            return h._send(400 if step['code'] == 'DA-4000' else 200,
                           dict(session_id=sid or new_session(), status=status(step['code'], step.get('reason'),
                                                                               step.get('retryable', False),
                                                                               step.get('retry_after_s')),
                                actions=None, timing=None))
        missing = [f'state.{side}_{part}' for side in ('left', 'right') for part in EEF_STATE
                   if base and f'state.{side}_{part}' not in obs]
        if missing:
            detail = f'inference error: observation is missing {missing}. This checkpoint reads state ' \
                     f'{[f"state.{s}_{p}" for s in ("right", "left") for p in ("arm", "gripper")] + missing}'
            return h._send(400, dict(session_id=sid or new_session(), status=status('DA-4000', 'malformed', detail=detail),
                                     actions=None, timing=None))
        if step.get('expired') and sid:
            return h._send(200, dict(session_id=new_session(), actions=None, status=status('DA-0001', 'expired')))
        if not sid:
            sid = new_session()
            st = status('DA-0001', 'new')
        else:
            st = status()
        rows = (step.get('chunk') or (lambda o: sway(o, t=time.monotonic() - self.t0)))(obs)
        if base:
            return h._send(200, dict(session_id=sid, status=st, actions=self.base_actions(rows, obs, step),
                                     timing=dict(serve_infer_ms=12, gateway_ms=15)))
        actions = {}
        per_key = {}
        for side, key in (('right', 'q_r_chunk'), ('left', 'q_l_chunk')):
            if side in rows:
                actions[key] = rows[side]
                actions[key[:3]] = rows[side][0] if rows[side] else []
                ok = rows[side] and all(isinstance(r, list) and len(r) >= 2 for r in rows[side])
                per_key[f'action.{side}_arm'] = [r[:-1] for r in rows[side]] if ok else rows[side]
                per_key[f'action.{side}_gripper'] = [r[-1] for r in rows[side]] if ok else rows[side]
        for k in obs:
            if k.startswith('state.') and k.replace('state.', 'action.', 1) not in per_key:
                width = len(obs[k]) if isinstance(obs[k], list) else 1
                per_key[k.replace('state.', 'action.', 1)] = [list(obs[k]) if width > 1 else obs[k][0] for _ in range(H)]
        if step.get('per_key_only'):
            actions = {}
        actions['per_key'] = per_key
        return h._send(200, dict(session_id=sid, status=st, actions=actions, timing=dict(serve_infer_ms=12, gateway_ms=15)))

    def base_actions(self, rows, obs, step):
        """dream-1-beta's `actions`, in the real order and shape: no action.<side>_gripper, no echo of the state."""
        actions, per_key = {}, {}
        for side, key in (('right', 'q_r_chunk'), ('left', 'q_l_chunk')):
            if side in rows:
                actions[key[:3]] = rows[side][0] if rows[side] else []
        for side, key in (('right', 'q_r_chunk'), ('left', 'q_l_chunk')):
            if side in rows:
                actions[key] = rows[side]
        for side in ('right', 'left'):
            if side not in rows:
                continue
            table = rows[side]
            ok = table and all(isinstance(r, list) and len(r) >= 2 for r in table)
            per_key[f'action.{side}_arm'] = [r[:-1] for r in table] if ok else table
            per_key[f'action.{side}_eef_xyz'] = [list(obs.get(f'state.{side}_eef_xyz') or [0.25, 0.0, 0.16])] * len(table)
            per_key[f'action.{side}_eef_rot6'] = [list(obs.get(f'state.{side}_eef_rot6') or [0, 1, 0, 0, 0, 1])] * len(table)
            per_key[f'action.{side}_grip'] = [r[-1] for r in table] if ok else table
        if step.get('per_key_only'):
            actions = {}
        actions['per_key'] = per_key
        return actions

    def _now(self):
        return time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())

    def embodiment(self, h, method, path, body):
        if method == 'POST' and path == '/v1/embodiments':
            name = f"{self.user}/{body['name']}"
            if name in self.embodiments:
                return h._send(409, dict(error=dict(code='conflict', message='name taken')))
            rec = dict(name=name, embodiment_config=body['embodiment_config'], created_at=self._now(),
                       id='emb_' + secrets.token_hex(4))
            self.embodiments[name] = rec
            return h._send(200, rec)
        if method == 'GET' and path == '/v1/embodiments':
            return h._send(200, list(self.embodiments.values()))
        if method == 'GET':
            name = urllib.parse.unquote(path[len('/v1/embodiments/'):])
            if name in self.embodiments:
                return h._send(200, self.embodiments[name])
        return h._send(404, dict(error=dict(code='not_found', message='no such embodiment')))

    def dataset(self, h, method, path, body):
        parts = path.split('/')
        if method == 'POST' and path == '/v1/datasets':
            did = 'ds_' + secrets.token_hex(6)
            rec = dict(id=did, name=f"{self.user}/{body['name']}", embodiment=body['embodiment'], status='uploading',
                       size_bytes=body['size_bytes'], episodes=body['episodes'], error=None, created_at=self._now())
            self.datasets[did] = rec
            return h._send(200, dict(dataset=rec, credentials=dict(access_key_id='AKIAFAKE', secret_access_key='fake',
                                                                   session_token='fake'),
                                     region='us-east-1', endpoint_url=self.url, bucket='bucket', key=f'{did}.zip'))
        if method == 'GET' and path == '/v1/datasets':
            return h._send(200, list(self.datasets.values()))
        if len(parts) >= 4 and parts[3] in self.datasets:
            rec = self.datasets[parts[3]]
            if method == 'POST' and parts[-1] == 'complete':
                if f'{rec["id"]}.zip' not in self.objects:
                    return h._send(409, dict(error=dict(code='conflict', message='the zip is not there')))
                rec.update(status='ready_to_process')
                return h._send(200, rec)
            if method == 'POST' and parts[-1] == 'fail':
                rec.update(status='failed', error=(body or {}).get('error'))
                return h._send(200, rec)
            if method == 'GET':
                return h._send(200, rec)
        return h._send(404, dict(error=dict(code='not_found', message='no such dataset')))

    def fine_tuning(self, h, method, path, body):
        if method == 'POST' and path == '/v1/fine_tuning/jobs':
            ds = next((d for d in self.datasets.values() if d['name'] == body.get('dataset')), None)
            if ds is None:
                return h._send(404, dict(error=dict(code='not_found', message='no such dataset')))
            jid = 'ftj_' + secrets.token_hex(6)
            job = dict(id=jid, dataset=ds['name'], embodiment=ds['embodiment'], base_model='dreamlabs/dream-1-beta',
                       suffix=body['suffix'], fine_tuned_model=None, status='queued',
                       hyperparameters=body.get('hyperparameters') or {}, queue_position=1, created_at=self._now(),
                       started_at=None, finished_at=None, progress=None, error=None)
            self.jobs[jid] = job
            return h._send(200, job)
        if method == 'GET' and path.startswith('/v1/fine_tuning/jobs/'):
            job = self.jobs.get(path.rsplit('/', 1)[1])
            if job is None:
                return h._send(404, dict(error=dict(code='not_found', message='no such job')))
            return h._send(200, job)
        if method == 'GET' and path.endswith('/training_data'):
            return h._send(200, dict(url=None, video=None, prompts=self.trained_prompts,
                                     cameras={r: [[320, 176]] for r in ('overhead', 'wrist_l', 'wrist_r')}))
        return h._send(404, dict(error=dict(code='not_found', message=path)))

    def succeed(self, job_id):
        job = self.jobs[job_id]
        job.update(status='succeeded', fine_tuned_model=f'dream-1-ft:abcd1234-{job["suffix"]}', finished_at=self._now(),
                   queue_position=None)
        return job['fine_tuned_model']
