"""Run a DreamLabs policy (Dream-1, `dreamlabs/dream-1-beta`, or your own fine-tune) on this robot through Roborun.

    pip install 'roborun-sdk[dreamlabs]'
    roborun policy check --provider dreamlabs --model dreamlabs/dream-1-beta
    roborun policy run --provider dreamlabs --model dreamlabs/dream-1-beta --prompt "fold the towel" --seconds 60

The provider drives DreamLabs' own client, `dream.DreamClient` from the `dreamlabs-client` package (Apache-2.0), and Roborun
does everything around it as for any policy run: consent before anything is sent, face blur on the frames, the station's
checks and limits on every chunk, Stop, the task record, the recording and the dataset episode.

What it does with DreamLabs' client:
- credentials the way DreamLabs documents them (DREAM_API_KEY and the DREAM_*_URL variables, else $DREAM_CREDENTIALS or
  ~/.dreamlabs/credentials). Roborun never reads, stores or prints the key: `DreamClient()` reads it;
- the embodiment is passed explicitly, so the client never looks the model up before the run;
- DreamLabs' client-side recorder is off unless you pass `--option record=true`;
- the client's log goes to log/policy/<id>/provider/dreamlabs-log unless you set DREAM_LOG or DREAM_LOG_DIR.

Options (`--option k=v`): timeout_ms (10000), obs_window (9), obs_fps (5), action_hz (30), execution_horizon (30),
sync_above_ms (3000), record (false), frame_size (320x176), embodiment (a fine-tune's embodiment.json from `roborun
dreamlabs export`).

dream-1-beta reads each arm's tool pose as well as its joints and gripper (`state.<side>_eef_xyz`, `_eef_rot6`, `_grip`);
Roborun computes it from the joint angles in DreamLabs' own YAM tool frame (dreamlabs_yam.py) and says so in the consent.
Its reply carries `action.<side>_arm` and `action.<side>_grip`, which is what the embodiment's `action` declares.

Pacing: DreamLabs' client paces requests itself (`refill_due`) and switches to sync pacing (the arms run the whole chunk,
then hold while the next request is answered) once the median round trip passes `sync_above_ms`. Its own default is
700 ms; dream-1-beta's warm round trip through the internet was measured at p50 about 0.6 s and p90 about 1.0 s, which
would hold the arms for a fifth to a half of every run, so Roborun passes 3000 ms (still under the runner's own stall
rules).

Importing this module needs only the standard library; `dream` is imported by start().
"""
from __future__ import annotations

import base64
import hashlib
import importlib.metadata
import importlib.util
import json
import math
import os
import warnings
from pathlib import Path

from . import dreamlabs_yam
from .provider import Chunk, Needs, PolicyRefused, Provider, ProviderError

NAME = 'dreamlabs'
TITLE = 'DreamLabs'
DIST = 'dreamlabs-client'
EXTRA = 'dreamlabs'
TESTED = '1.1.'                     # the client series this Roborun was tested with (1.1.7)
BASE_MODEL = 'dreamlabs/dream-1-beta'
FINE_TUNE = 'dream-1-ft:'
SIDES = ('left', 'right')
CHUNK_KEY = {'left': 'q_l_chunk', 'right': 'q_r_chunk'}
# Roborun camera role -> the video role Dream-1 reads on a YAM body (and the export writes)
VIDEO_ROLE = {'top': 'overhead', 'left': 'wrist_l', 'right': 'wrist_r'}
DATASET_KEY = {'top': 'observation.images.top', 'left': 'observation.images.left_wrist',
               'right': 'observation.images.right_wrist'}
SAME_CODE_LIMIT = 3                 # a non-retryable server code or a bad action this many times in a row is retried; then it fails
MODELS_FILE = 'dreamlabs-models.json'
DEFAULT_GATEWAY = 'api.dreamlabs.ai'
EEF_PARTS = (('eef_xyz', 3), ('eef_rot6', 6), ('grip', 1))   # the tool pose state dream-1-beta reads, per arm


def conventions():
    """The joint conventions verified against DreamLabs' YAM robot config (policy/data/dreamlabs_conventions.json)."""
    path = Path(__file__).resolve().parent / 'data' / 'dreamlabs_conventions.json'
    try:
        return json.loads(path.read_text()).get('models') or {}
    except (OSError, ValueError):
        return {}


def client_version():
    try:
        return importlib.metadata.version(DIST)
    except importlib.metadata.PackageNotFoundError:
        return None


def installed():
    return importlib.util.find_spec('dream') is not None and client_version() is not None


def key_source(env=None):
    """Where DreamLabs' client will find the key, by DreamLabs' own order, without reading it: 'environment',
    'credentials file' or 'none'."""
    env = os.environ if env is None else env
    if str(env.get('DREAM_API_KEY') or '').strip():
        return 'environment (DREAM_API_KEY)'
    path = Path(os.path.expanduser(env.get('DREAM_CREDENTIALS') or '~/.dreamlabs/credentials'))
    if path.is_file():
        return f'credentials file ({path}; Roborun does not read it)'
    return 'none: set up DreamLabs credentials as DreamLabs documents them'


def gateway_host(env=None):
    """The host frames go to, when the environment names it; else DreamLabs' default (the credentials file may name another)."""
    env = os.environ if env is None else env
    url = str(env.get('DREAM_GATEWAY_URL') or '').strip()
    if url:
        return url.split('://', 1)[-1].split('/', 1)[0]
    return f'{DEFAULT_GATEWAY}, or the gateway your DreamLabs credentials name'


def models_file():
    from .. import net
    return net.roborun_home() / MODELS_FILE


def known_models():
    """Fine-tunes made from Roborun exports: {model id: {embodiment_config, roborun, dataset, ...}}."""
    try:
        data = json.loads(models_file().read_text())
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def remember_model(model, entry):
    data = known_models()
    data[model] = entry
    path = models_file()
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + f'.{os.getpid()}.tmp')
    tmp.write_text(json.dumps(data, indent=1, sort_keys=True))
    os.replace(tmp, path)


def _size(text):
    if text in (None, '', 'none', 'None'):
        return None
    if isinstance(text, (list, tuple)) and len(text) == 2:
        w, h = text
    else:
        w, _, h = str(text).lower().partition('x')
    try:
        w, h = int(w), int(h)
    except ValueError:
        raise PolicyRefused(f'frame_size {text!r}: write it as WIDTHxHEIGHT, e.g. 320x176') from None
    if not (16 <= w <= 1920 and 16 <= h <= 1080):
        raise PolicyRefused(f'frame_size {text!r} is out of range')
    return (w, h)


class DreamLabsProvider(Provider):
    """A DreamLabs model as a Roborun policy provider. See the module docstring."""
    name = NAME
    title = TITLE

    def __init__(self, timeout_ms=10000, obs_window=9, obs_fps=5, action_hz=30, execution_horizon=30, sync_above_ms=3000,
                 record=False, frame_size='320x176', embodiment=None):
        self.timeout_ms = int(timeout_ms)
        self.obs_window = int(obs_window)
        self.obs_fps = float(obs_fps)
        self.action_hz = float(action_hz)
        self.execution_horizon = int(execution_horizon)
        self.sync_above_ms = float(sync_above_ms)
        self.record = record if isinstance(record, bool) else str(record).strip().lower() in ('1', 'true', 'yes')
        self.frame_size = _size(frame_size)
        self.embodiment_file = embodiment
        self.client = None
        self._notices = []
        self._same = (None, 0)

    # ---- offline: what it needs ----------------------------------------------------------------------------------------
    def needs(self, spec):
        model = (spec.model or '').strip()
        if not model:
            raise PolicyRefused(f'name the DreamLabs model: --model {BASE_MODEL}, or your fine-tune\'s '
                                f'{FINE_TUNE}... id')
        if not installed():
            raise PolicyRefused(f"the DreamLabs provider needs pip install 'roborun-sdk[{EXTRA}]'")
        if key_source().startswith('none'):
            raise PolicyRefused('no DreamLabs key on this computer: set DREAM_API_KEY, or the credentials file '
                                '~/.dreamlabs/credentials ($DREAM_CREDENTIALS names another), as DreamLabs documents them. '
                                'Nothing was sent')
        if not 1 <= self.obs_window <= 64 or not 0 < self.obs_fps <= 30 or not 5 <= self.action_hz <= 200:
            raise PolicyRefused('obs_window is 1..64, obs_fps 0..30 and action_hz 5..200')
        if not 100 <= self.timeout_ms <= 120000:
            raise PolicyRefused('timeout_ms is 100..120000')
        if not 1 <= self.execution_horizon <= 1000 or not 100 <= self.sync_above_ms <= 60000:
            raise PolicyRefused('execution_horizon is 1..1000 rows and sync_above_ms 100..60000')
        arms = {a.side: a for a in spec.arms}
        self.model = model
        if model.startswith(FINE_TUNE):
            self._fine_tune(model, arms)
        else:
            self._base(arms)
        cams = {}
        for role in self.roles:
            if role not in spec.cameras:
                raise PolicyRefused(f'{model} reads the {role} camera ({VIDEO_ROLE[role]}), which is not configured or not '
                                    'delivering frames; nothing was sent')
            cams[VIDEO_ROLE[role]] = role
        self.cams = cams
        self.arms = arms
        w, h = self.frame_size or ('as', 'captured')
        frames = (f'up to {self.obs_window} recent frames per camera ({", ".join(self.roles)}) at {w}x{h}'
                  if self.roles else 'no camera frames')
        arms_named = f'{" and ".join(self.state_sides)} arm{"s" if len(self.state_sides) > 1 else ""}'
        what = ('the joint angles, grippers and tool poses (where each gripper is and how it is turned, computed from the '
                'joint angles)' if self.eef_sides else 'the joint angles and grippers')
        sends = [f'the prompt, {what} of the {arms_named}', frames,
                 'your DreamLabs key, as DreamLabs\' client sends it, and an end-of-session call when the run ends']
        if model.startswith(FINE_TUNE):
            sends.append('a lookup of what your fine-tune was trained on, made by DreamLabs\' client when the run starts')
        if self.record:
            sends.append('with record=true: every observation (the same frames and state) to DreamLabs\' recorder too')
        return Needs(arms=tuple(self.drive), cameras=dict(cams), frame_hz=self.obs_fps, frame_size=self.frame_size,
                     remote=True, sends=tuple(sends), version=f'{DIST} {client_version()}')

    def _base(self, arms):
        verified = conventions()
        for side in SIDES:
            arm = arms.get(side)
            if arm is None:
                raise PolicyRefused(f'{self.model} runs on two YAM arms; this robot has no {side} arm (or it is parked)')
            entry = verified.get(arm.model) or {}
            if not entry.get('verified'):
                raise PolicyRefused(f'no verified joint convention for {arm.model} with DreamLabs; nothing was sent '
                                    f'({self.model} runs on two YAM arms)')
            if arm.joints != len(entry.get('joints') or ()):
                raise PolicyRefused(f'the {side} arm has {arm.joints} joints; {self.model} drives {len(entry["joints"])}')
            if not arm.gripper:
                raise PolicyRefused(f'the {side} arm has no gripper with a jaw range; {self.model} drives one')
            if arm.model != 'yam':
                raise PolicyRefused(f'{self.model} reads each arm\'s tool pose, which Roborun computes for YAM arms only')
        # state: joints and gripper, then the tool pose and the gripper again as `grip` (dream-1-beta refuses an
        # observation without them, DA-4000 malformed). action: the reply's per_key has `<side>_grip`, not `_gripper`.
        self.parts = {side: (f'{side}_arm', f'{side}_gripper') for side in SIDES}
        self.action_parts = {side: (f'{side}_arm', f'{side}_grip') for side in SIDES}
        self.eef_sides = SIDES
        self.state_sides = SIDES
        self.drive = SIDES
        self.roles = ('top', 'left', 'right')
        state, action, i, j = {}, {}, 0, 0
        for side in ('right', 'left'):
            n = arms[side].joints
            state[f'{side}_arm'], state[f'{side}_gripper'] = [i, i + n], [i + n, i + n + 1]
            action[f'{side}_arm'], action[f'{side}_grip'] = [j, j + n], [j + n, j + n + 1]
            i, j = i + n + 1, j + n + 1
        for side in ('left', 'right'):
            for part, width in EEF_PARTS:
                state[f'{side}_{part}'] = [i, i + width]
                i += width
        self.embodiment = {'state': state, 'action': action, 'video': {VIDEO_ROLE[r]: DATASET_KEY[r] for r in self.roles}}
        self.robot = 'yam'

    def _fine_tune(self, model, arms):
        entry = None
        if self.embodiment_file:
            try:
                entry = json.loads(Path(os.path.expanduser(str(self.embodiment_file))).read_text())
            except (OSError, ValueError) as exc:
                raise PolicyRefused(f'embodiment={self.embodiment_file}: {exc}') from None
        else:
            entry = known_models().get(model)
        if not isinstance(entry, dict) or not isinstance(entry.get('embodiment_config'), dict):
            raise PolicyRefused(f'Roborun does not know which arms {model} was trained on: run it with --option '
                                'embodiment=<export folder>/embodiment.json (the export it was trained on), or check it '
                                'with roborun dreamlabs status <job id>; nothing was sent')
        config = entry['embodiment_config']
        own = entry.get('roborun') or {}
        state = config.get('state') or {}
        trained = own.get('arms') or {}
        sides = tuple(s for s in SIDES if f'{s}_arm' in state)
        if not sides:
            raise PolicyRefused(f'{model}\'s embodiment has no <side>_arm part Roborun can drive')
        for side in sides:
            arm = arms.get(side)
            if arm is None:
                raise PolicyRefused(f'{model} was trained on a {side} arm, which this robot does not have (or it is parked)')
            lo, hi = state[f'{side}_arm']
            if hi - lo != arm.joints:
                raise PolicyRefused(f'{model} was trained on {hi - lo} joints on the {side} arm; this one has {arm.joints}')
            if (f'{side}_gripper' in state) != bool(arm.gripper):
                raise PolicyRefused(f'{model} and the {side} arm differ on the gripper')
            want = (trained.get(side) or {}).get('model')
            if want and want != arm.model:
                raise PolicyRefused(f'{model} was trained on a {want} arm on the {side}; this one is {arm.model}')
        roles = []
        for role, rr in (('overhead', 'top'), ('wrist_l', 'left'), ('wrist_r', 'right')):
            if role in (config.get('video') or {}):
                roles.append(rr)
        unknown = sorted(set(config.get('video') or {}) - set(VIDEO_ROLE.values()))
        if unknown:
            raise PolicyRefused(f'{model} reads cameras Roborun does not export ({", ".join(unknown)})')
        self.parts = {side: (f'{side}_arm', f'{side}_gripper' if f'{side}_gripper' in state else None) for side in sides}
        action = config.get('action') or state
        self.action_parts = {side: (f'{side}_arm', next((f'{side}_{g}' for g in ('grip', 'gripper')
                                                         if f'{side}_{g}' in action), None)) for side in sides}
        self.eef_sides = tuple(s for s in sides if any(f'{s}_{part}' in state for part, _ in EEF_PARTS))
        for side in self.eef_sides:
            if not all(f'{side}_{part}' in state for part, _ in EEF_PARTS):
                raise PolicyRefused(f'{model} reads part of the {side} arm\'s tool pose; Roborun sends all of '
                                    f'{", ".join(f"{side}_{p}" for p, _ in EEF_PARTS)} or none')
            if arms[side].model != 'yam':
                raise PolicyRefused(f'{model} reads the {side} arm\'s tool pose, which Roborun computes for YAM arms only')
        self.state_sides = sides
        self.drive = sides
        self.roles = tuple(roles)
        self.embodiment = config
        self.robot = None

    # ---- consent ---------------------------------------------------------------------------------------------------------
    def consent_text(self, needs, face_blur):
        blur = 'face-blurred' if face_blur else 'NOT face-blurred: you turned face blur off'
        lines = [f'While this run is active, including while the model warms up and before any motion, Roborun sends to '
                 f'DreamLabs ({gateway_host()}):']
        lines += [f'  - {item}' for item in needs.sends]
        if needs.cameras:
            lines.append(f'  Camera frames are {blur}.')
        lines.append('  That is about 1.3 MB per request at about one request per second, roughly 80 MB per minute.'
                     if len(needs.cameras) >= 3 and self.obs_window >= 9 else
                     '  Requests go at about one per second while the run is active.')
        lines.append('Nothing else is sent to DreamLabs. This run\'s Roborun task record, recording and dataset episode are not '
                     'sent to DreamLabs; like any task\'s, they go to your Roborun history (roborun up --no-data-sync keeps '
                     'them on this computer). The request timing in log/policy/ never leaves this computer.')
        return '\n'.join(lines)

    def key_source(self):
        return key_source()

    def drain_notices(self):
        out, self._notices = self._notices, []
        return out

    # ---- the run ---------------------------------------------------------------------------------------------------------
    def start(self, spec):
        if not (os.environ.get('DREAM_LOG') or os.environ.get('DREAM_LOG_DIR')):
            os.environ['DREAM_LOG_DIR'] = str(Path(spec.log_dir) / 'dreamlabs-log')
        from .._optional import require
        dream = require('dream', EXTRA)
        version = client_version() or getattr(dream, '__version__', '?')
        if not str(version).startswith(TESTED):
            self._notices.append(f'{DIST} {version} was not tested with this Roborun; tested: 1.1.7')
        self.errors = dream
        config = dict(checkpoint=self.model, embodiment=self.embodiment, strict=True, timeout_ms=self.timeout_ms,
                      obs_window=self.obs_window, obs_fps=self.obs_fps, action_hz=self.action_hz,
                      execution_horizon=self.execution_horizon, sync_above_ms=self.sync_above_ms)
        if self.robot:
            config['robot'] = self.robot
        if not self.record:
            config['_record'] = False
        self.client = dream.DreamClient()
        try:
            with warnings.catch_warnings(record=True) as seen:
                warnings.simplefilter('always')
                self.client.policy(config)
        except dream.DreamConfigError as exc:
            if '_record' in str(exc):
                raise PolicyRefused(f'this {DIST} cannot turn its recorder off; Roborun will not run it without '
                                    '--option record=true') from None
            raise PolicyRefused(f'{TITLE} refused the model config: {exc}') from None
        self._keep(seen)

    def _keep(self, seen):
        for w in seen:
            text = ' '.join(str(w.message).split())
            if text and text not in self._notices:
                self._notices.append(f'{TITLE}: {text}'[:500])

    def _frames(self, obs):
        return {f'video.{VIDEO_ROLE[role]}': base64.b64encode(obs.frames[role].jpeg).decode('ascii') for role in self.roles}

    def _state(self, obs):
        out = {}
        for side in self.state_sides:
            st = obs.arms[side]
            arm_part, grip_part = self.parts[side]
            out[f'state.{arm_part}'] = [float(v) for v in st.q_rad]
            grip = float(st.gripper if st.gripper is not None else 1.0)
            if grip_part:
                out[f'state.{grip_part}'] = [grip]
            if side in self.eef_sides:
                xyz, rot6 = dreamlabs_yam.eef(st.q_rad)
                out[f'state.{side}_eef_xyz'] = xyz
                out[f'state.{side}_eef_rot6'] = rot6
                out[f'state.{side}_grip'] = [grip]
        out['prompt'] = obs.prompt
        return out

    def observe(self, obs):
        self.client.set_observation(self._frames(obs), self._state(obs), timestamp=float(obs.t))

    def _commanded(self, obs):
        out = {}
        for side in self.state_sides:
            st = obs.arms[side]
            row = [float(v) for v in st.commanded_rad]
            if self.parts[side][1]:
                row.append(float(st.commanded_gripper if st.commanded_gripper is not None else 1.0))
            out[CHUNK_KEY[side]] = row
        return out

    def act(self, obs):
        d = self.errors
        try:
            with warnings.catch_warnings(record=True) as seen:
                warnings.simplefilter('always')
                action = self.client.get_action(commanded=self._commanded(obs))
            self._keep(seen)
        except d.DreamSessionExpired as exc:
            raise ProviderError(f'DreamLabs restarted the session ({_reason(exc)})', session_restarted=True,
                                code=getattr(exc, 'code', None)) from None
        except d.DreamAuthError as exc:
            raise ProviderError(f'DreamLabs did not accept your DreamLabs API key ({_words(_reason(exc))}); check your '
                                f'DreamLabs credentials', code=getattr(exc, 'code', None)) from None
        except d.DreamBadRequest as exc:
            raise ProviderError(f'DreamLabs refused the request: {exc}', code=getattr(exc, 'code', None)) from None
        except d.DreamServerError as exc:
            code, reason = getattr(exc, 'code', None), getattr(exc, 'reason', None)
            if getattr(exc, 'retryable', False):
                self._same = (None, 0)
                raise ProviderError(str(exc), retryable=True, retry_after_s=getattr(exc, 'retry_after_s', None),
                                    code=code, warming=reason == 'pool_warming') from None
            self._repeat(f'{code}:{reason}', str(exc), code)
        except (d.DreamTimeout, d.DreamProtocolError) as exc:
            raise ProviderError(f'{type(exc).__name__}: {exc}', retryable=True, code=type(exc).__name__) from None
        except d.DreamActionError as exc:
            self._repeat(f'action:{exc}', f'DreamLabs sent an unusable action: {exc}', 'bad_action')
        except d.DreamError as exc:
            raise ProviderError(f'{type(exc).__name__}: {exc}', code=type(exc).__name__) from None
        self._same = (None, 0)
        return self._chunk(action)

    def _repeat(self, key, message, code):
        last, n = self._same
        n = n + 1 if key == last else 1
        self._same = (key, n)
        if n > SAME_CODE_LIMIT:
            raise ProviderError(f'{message} ({n} times in a row)', code=code) from None
        raise ProviderError(message, retryable=True, retry_after_s=0.5, code=code) from None

    def _chunk(self, action):
        if not isinstance(action, dict):
            raise ProviderError('DreamLabs answered no action', retryable=True, retry_after_s=0.5, code='no_action')
        per_key = action.get('per_key') if isinstance(action.get('per_key'), dict) else {}
        rows = {}
        for side in self.drive:
            table = action.get(CHUNK_KEY[side])
            if table is None:
                table = _from_parts(per_key, *self.action_parts[side])
            rows[side] = _plain_rows(table)
        timing = {}
        try:
            timing = dict(self.client.last_timing or {})
        except Exception:
            pass
        meta = dict(provider_ms=timing.get('serve_infer_ms'), gateway_ms=timing.get('gateway_ms'),
                    request_bytes=timing.get('request_bytes'), response_bytes=timing.get('response_bytes'),
                    session_id=_session(self.client))
        return Chunk(rows, hz=self.action_hz, meta=meta)

    def refill_due(self, executed, remaining):
        return bool(self.client.refill_due(int(executed), int(remaining)))

    def end_episode(self):
        if self.client is not None:
            try:
                self.client.reset_episode()
            except Exception:
                pass

    def close(self):
        if self.client is not None:
            try:
                self.client.close()
            except Exception:
                pass


def _reason(exc):
    return getattr(exc, 'reason', None) or str(exc)


def _words(reason):
    """DreamLabs' reason code as words (invalid_key -> invalid key): never one of Roborun's own error codes."""
    return str(reason).replace('_', ' ')


def _session(client):
    try:
        sid = client.session_id
    except Exception:
        return None
    return hashlib.sha256(sid.encode()).hexdigest()[:12] if isinstance(sid, str) else None


def _plain_rows(table):
    """Rows as lists of floats, as they came (the runner and the station check the shape and values)."""
    if hasattr(table, 'tolist'):
        table = table.tolist()
    if not isinstance(table, (list, tuple)):
        return []
    out = []
    for row in table:
        if hasattr(row, 'tolist'):
            row = row.tolist()
        if isinstance(row, (int, float)):
            row = [row]
        try:
            out.append([float(v) for v in row])
        except (TypeError, ValueError):
            out.append([math.nan])
    return out


def _from_parts(per_key, arm_part, grip_part):
    """[q.., gripper] rows from the reply's `per_key` `action.<part>` rows (`<side>_grip` for dream-1-beta, the
    export's `<side>_gripper` for a fine-tune made from it)."""
    arm = _plain_rows(per_key.get(f'action.{arm_part}'))
    if not grip_part:
        return arm
    grip = _plain_rows(per_key.get(f'action.{grip_part}'))
    if len(grip) != len(arm):
        return arm                     # a wrong shape: the runner refuses it and says why
    return [a + g[:1] for a, g in zip(arm, grip)]
