"""What a policy run leaves on this computer: a Roborun task record, a recording, a dataset episode, and the provider's timing.

1. <runtime>/log/tasks/<id>.json: the task record the robot's local API lists (/v1/tasks), the viewer and History show, and the
   history upload sends. Roborun's own counts only (chunks played, refusals, holds): never provider timings or keys.
2. The recording, in the runtime's recordings folder beside every task's, in the manifest-recording/1 layout the local API
   serves (/v1/tasks/<id>/recording): recording.json, frames.jsonl, <camera>.mjpg (sharp JPEGs at about 10 Hz), cameras.jsonl, and
   joints.npz (measured and played joints at 30 Hz, this module's own columns). Only when this robot records its tasks, by
   the runtime's own setting (config.json `video_record`, see recording_settings): its `enabled` (only true turns it on), its
   `cameras`, and its caps. Its retention is an ordinary task's (make_room): before the run, the oldest recordings go until
   there is room for `run_cap_mb` under `total_cap_mb`, by the installed runtime's own cleanup; during it, the frames stop
   at `run_cap_mb`, `day_cap_mb`, `min_free_mb` or a full disk, and the task record says which. With the recorder off, a
   policy run leaves no recording and no episode, as an ordinary task does.
3. One episode of this robot's LeRobot dataset (roborun_sdk.dataset, at robot.dataset_root), with the same features and units
   as the episodes of Roborun's own tasks: observation.state, action (the target the station played), observation.gripper,
   observation.effort, observation.motor_temperature_c and observation.images.<top|left_wrist|right_wrist>. Never for a
   simulation (robot.json `sim`, or a camera whose serial looks simulated, as the runtime's LeRobot export decides): the
   dataset is the real robot's, and an export or a fine-tune sends its episodes.
4. <runtime>/log/policy/<id>/: requests.jsonl (one line per act: round trip, sizes, age, outcome), chunks.jsonl (every chunk as
   it came), summary.json, shadow.json for a shadow run, and provider/ (the provider's own files). This folder is outside
   every source of the history upload: it stays on this computer.

The run id is 32 hex; the task id and the recording's clip id are the same value. The record names the process running it
(`runner`: pid, a hash of the boot id, the process start time). A run whose process was killed (SIGKILL, a crash) cannot close
its own record: close_orphans() closes it (failed, runner_gone) and ends its recording.json; `roborun policy run`,
`roborun policy runs` and `roborun stop` call it.
"""
from __future__ import annotations

import datetime
import hashlib
import json
import errno
import os
import re
import threading
import time
from pathlib import Path

SCHEMA = 'manifest-recording/1'
CAMERA_KEYS = {'top': 'top', 'left': 'left_wrist', 'right': 'right_wrist'}
POSE_HZ = 30.0
FRAME_HZ = 10.0
JPEG_QUALITY = 90
DEFAULT_FPS = 10.0
# a camera whose serial holds one of these is not a real camera: the runtime's LeRobot export refuses it, and so does this
SIM_SERIAL_WORDS = ('sim', 'mujoco', 'fake', 'virtual', 'render')


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds')


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + f'.{os.getpid()}.tmp')
    tmp.write_text(json.dumps(data, allow_nan=False, default=str))
    os.replace(tmp, path)


def append_line(path, data):
    with open(path, 'a') as f:
        f.write(json.dumps(data, allow_nan=False, default=str) + '\n')


def boot_id():
    """This boot, as a short hash of the kernel's boot id (Linux), else None: a record written in another boot belongs to a
    process that is gone. Hashed, because task records go to the user's Roborun history."""
    try:
        raw = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        return None
    return hashlib.sha256(raw.encode()).hexdigest()[:16] if raw else None


def process_start(pid):
    """When the process started, in clock ticks since boot (Linux), else None: tells a reused pid from the one recorded."""
    try:
        stat = Path(f'/proc/{int(pid)}/stat').read_text()
        return int(stat.rsplit(')', 1)[1].split()[19])
    except (OSError, ValueError, IndexError):
        return None


def runner_identity():
    pid = os.getpid()
    return dict(pid=pid, boot_id=boot_id(), process_start=process_start(pid))


def process_gone(ident):
    """True when the process that wrote a record ({pid, boot_id, process_start}) no longer runs; False when it does; None
    when this computer cannot tell (then the record is left alone)."""
    ident = ident if isinstance(ident, dict) else {}
    pid = ident.get('pid')
    if not isinstance(pid, int) or isinstance(pid, bool) or pid <= 0:
        return None
    here = boot_id()
    if ident.get('boot_id') and here and ident['boot_id'] != here:
        return True                                  # written before this computer last started
    if os.name == 'nt':
        return None                                  # os.kill(pid, 0) would end the process on Windows
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return True
    except PermissionError:
        pass                                         # someone else's process with that pid: compare start times below
    except OSError:
        return None
    started = process_start(pid)
    if ident.get('process_start') is not None and started is not None and started != ident['process_start']:
        return True                                  # the pid was reused by another process
    return False


RUNNER_GONE = ('the process running this policy run ended without closing it (it was killed or crashed). Nothing moves '
               'without that process: the arms\' station ends a policy lease about a second after its last heartbeat, and '
               'the arms hold.')


def close_orphans(runtime):
    """Close every policy task record still 'running' whose process is gone: failed, stop_reason runner_gone, and its
    recording.json ended. Returns the ids closed. Records of a live process, or of a process this computer cannot check,
    are left alone."""
    tasks = Path(runtime) / 'log' / 'tasks'
    closed = []
    try:
        paths = list(tasks.glob('*.json'))
    except OSError:
        return closed
    for path in paths:
        try:
            rec = json.loads(path.read_text())
        except (OSError, ValueError):
            continue
        if not isinstance(rec, dict) or rec.get('kind') != 'policy' or rec.get('status') != 'running':
            continue
        if process_gone(rec.get('runner')) is not True:
            continue
        rec.update(status='failed', stop_reason='runner_gone', error=RUNNER_GONE, final=RUNNER_GONE, phase=None,
                   ended=utc_now(), closed_by='roborun: the run\'s process was gone')
        try:
            write_json(path, rec)
        except OSError:
            continue
        rid = str(rec.get('id') or path.stem)
        meta_path = Paths(runtime, rid).recording / 'recording.json'
        try:
            meta = json.loads(meta_path.read_text())
            if isinstance(meta, dict) and meta.get('state') == 'recording':
                meta.update(state='ended', stop_reason='runner_gone', ended_at=utc_now(),
                            closed_by='roborun: the run\'s process was gone')
                write_json(meta_path, meta)
        except (OSError, ValueError):
            pass
        closed.append(rid)
    return closed


class Paths:
    def __init__(self, runtime, run_id):
        self.runtime = Path(runtime)
        self.id = run_id
        self.task = self.runtime / 'log' / 'tasks' / f'{run_id}.json'
        self.clip = self.runtime / 'log' / 'video-motion' / run_id
        self.recording = self.clip / f'recording-{run_id}-01'
        self.policy = self.runtime / 'log' / 'policy' / run_id
        self.provider = self.policy / 'provider'


class TaskRecord:
    """<runtime>/log/tasks/<id>.json, written atomically: at the start, every second while running, and at the end."""
    STATUS = ('running', 'done', 'stopped', 'failed')

    def __init__(self, paths, *, prompt, provider, model, policy):
        self.paths = paths
        self.started = time.monotonic()
        self.record = dict(id=paths.id, kind='policy', prompt=prompt, execute=True, status='running', created=utc_now(),
                           elapsed_s=0.0, final=None, error=None, stop_reason=None, phase='Starting the policy',
                           model=f'{provider}:{model}' if model else str(provider), plan_id=paths.id,
                           arm=None, policy=dict(policy), safety={}, runner=runner_identity())
        self._last = 0.0

    def update(self, force=False, **fields):
        self.record.update(fields)
        now = time.monotonic()
        if force or now - self._last >= 1.0:
            self.record['elapsed_s'] = round(now - self.started, 1)
            write_json(self.paths.task, self.record)
            self._last = now

    def close(self, status, *, final=None, error=None, stop_reason=None, **fields):
        assert status in self.STATUS[1:]
        self.update(force=True, status=status, final=final, error=error if status == 'failed' else None,
                    stop_reason=stop_reason, phase=None, ended=utc_now(), **fields)


class Recorder:
    """Samples the station's pose at 30 Hz and the cameras at about 10 Hz while the run moves the arms, into the recording
    folder; at the end writes joints.npz, recording.json and the dataset episode. Sampling never commands anything."""

    DISK_CHECK_S = 2.0

    def __init__(self, paths, station, cameras, roles, *, prompt, sides, jaw_range, clock=time.monotonic, cap_bytes=None,
                 day_room_bytes=None, min_free_bytes=None, cleanup=None):
        self.paths, self.station, self.cameras, self.roles = paths, station, cameras, list(roles)
        # the runtime recorder's caps while recording: no more frames past run_cap_mb (cap_bytes), once today's recordings
        # reach day_cap_mb (day_room_bytes: what was left of it at the start), when the disk's free space falls below
        # min_free_mb, or when a write finds the disk full. The joints go on; `capped` says which, for the task record.
        self.cap_bytes, self.bytes, self.capped = cap_bytes, 0, None
        self.day_room_bytes, self.min_free_bytes = day_room_bytes, min_free_bytes
        self.cleanup = list(cleanup or [])                      # the older recordings deleted to make room for this one
        self._next_disk = 0.0
        self.prompt, self.sides, self.jaw_range, self.clock = prompt, list(sides), dict(jaw_range), clock
        self.poses = []                              # (t, {side: row})
        self.frames = {r: [] for r in self.roles}    # role -> [(acq, offset, length, w, h)]
        self.errors = {r: 0 for r in self.roles}
        self.counts = {r: 0 for r in self.roles}
        self.serials = {}                            # role -> every serial its frames named
        self.simulated = set()                       # roles whose frames said simulated=True
        self._seen_session = {}
        self._stop = threading.Event()
        self._thread = None
        self._files = {}
        self._i = 0
        self.why_no_frames = None

    def start(self):
        d = self.paths.recording
        d.mkdir(parents=True, exist_ok=True)
        self.meta = dict(schema=SCHEMA, state='recording', task_id=self.paths.id, clip_id=self.paths.id, index=1,
                         label='policy', prompt=self.prompt[:500], created_at=utc_now(),
                         created_local_date=datetime.date.today().isoformat(),
                         clock=dict(monotonic_s=time.monotonic(), wall_s=time.time()),
                         config=dict(hz=FRAME_HZ, cameras=self.roles, jpeg_quality=JPEG_QUALITY, depth_every=0,
                                     pose_hz=POSE_HZ),
                         source='roborun policy run', pid=os.getpid(), runner=runner_identity())
        write_json(d / 'recording.json', self.meta)
        self._thread = threading.Thread(target=self._loop, name='policy-record', daemon=True)
        self._thread.start()
        return self

    def _loop(self):
        period, k = 1.0 / POSE_HZ, 0
        every = max(1, int(round(POSE_HZ / FRAME_HZ)))
        t0 = self.clock()
        while not self._stop.is_set():
            try:
                t = self.clock()
                pose = self.station.pose()
                measured = pose.get('measured') or {}
                self.poses.append((t, {s: measured.get(s) for s in self.sides}))
            except Exception:
                pass
            if k % every == 0 and self.roles:
                self._frames()
            k += 1
            self._stop.wait(max(0.0, t0 + k * period - self.clock()))

    def _out_of_room(self, size):
        """Which cap a frame of `size` bytes would cross, else None."""
        if self.cap_bytes is not None and self.bytes + size > self.cap_bytes:
            return 'run_cap_mb'
        if self.day_room_bytes is not None and self.bytes + size > self.day_room_bytes:
            return 'day_cap_mb'
        if self.min_free_bytes is not None:
            now = time.monotonic()
            if now >= self._next_disk:
                self._next_disk = now + self.DISK_CHECK_S
                import shutil
                try:
                    if shutil.disk_usage(self.paths.recording).free < self.min_free_bytes:
                        return 'min_free_mb'
                except OSError:
                    pass
        return None

    def _frames(self):
        from .station import jpeg
        if self.capped:                                         # a cap ends the frames for good, as the runtime's recorder
            return
        for role in self.roles:
            try:
                got = self.cameras.frame(role)
                data, w, h = jpeg(got['rgb'], None, JPEG_QUALITY)
            except Exception as exc:
                self.errors[role] += 1
                self.why_no_frames = f'{type(exc).__name__}: {exc}'[:200]
                continue
            self.serials.setdefault(role, set()).add(str(got.get('serial') or ''))
            if got.get('simulated'):
                self.simulated.add(role)
            sess = got.get('camera_session_id')
            if self._seen_session.get(role) != sess:
                self._seen_session[role] = sess
                append_line(self.paths.recording / 'cameras.jsonl', dict(
                    cam=role, serial=got.get('serial'), intrinsics=got.get('intrinsics'), depth_scale_m=got.get('depth_scale_m'),
                    reference_stream=got.get('reference_stream'), profiles=got.get('profiles'), camera_session_id=sess,
                    timestamp_domain=got.get('timestamp_domain'), fid=got.get('frame_id')))
            cap = self._out_of_room(len(data))
            if cap:
                self.capped = cap                               # the joints go on; no more frames (config.json video_record)
                return
            acq = float(got.get('acquired_at_s', time.monotonic()))
            try:
                f = self._files.get(role)
                if f is None:
                    f = self._files[role] = open(self.paths.recording / f'{role}.mjpg', 'ab')
                offset = f.tell()
                f.write(data)
                f.flush()
                append_line(self.paths.recording / 'frames.jsonl', dict(
                    i=self._i, cam=role, k=self.counts[role], t=time.monotonic(), acq=acq, fid=got.get('frame_id'),
                    sess=sess, jpg=[offset, len(data)], w=w, h=h))
            except OSError as exc:
                self.errors[role] += 1
                self.why_no_frames = f'{type(exc).__name__}: {exc}'[:200]
                if exc.errno in (errno.ENOSPC, errno.EDQUOT, errno.EROFS):
                    self.capped = 'disk_full'
                    return
                continue
            self.bytes += len(data)
            self._i += 1
            self.counts[role] += 1
            self.frames[role].append((acq, offset, len(data)))

    def stop(self, stop_reason):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        for f in self._files.values():
            f.close()
        info = dict(frames=dict(self.counts), poses=len(self.poses), errors=dict(self.errors))
        if self.capped:
            info['capped'] = self.capped
        if self.cleanup:
            info['cleanup'] = list(self.cleanup)
        try:
            info['joints'] = self._write_joints()
        except Exception as exc:
            info['joints_error'] = f'{type(exc).__name__}: {exc}'[:160]
        try:
            size = sum(p.stat().st_size for p in self.paths.recording.iterdir() if p.is_file())
        except OSError:
            size = None
        self.meta.update(state='ended', ended_s=time.monotonic(), stop_reason=stop_reason, frames=dict(self.counts),
                         errors=dict(self.errors), bytes=size, capped=self.capped, cleanup=list(self.cleanup))
        try:
            write_json(self.paths.recording / 'recording.json', self.meta)
        except OSError as exc:                                  # a full disk: the task record still says what happened
            info['recording_json_error'] = f'{type(exc).__name__}: {exc}'[:160]
        return info

    # ---- joints: this module's own columns (not the runtime's 51-column layout, so its export never picks this up)
    def columns(self):
        cols = ['t']
        for side in self.sides:
            n = self._joints(side)
            cols += [f'{side}_q{i}' for i in range(1, n + 1)] + [f'{side}_c{i}' for i in range(1, n + 1)]
            cols += [f'{side}_jaw', f'{side}_jaw_cmd', f'{side}_gripper_effort']
            cols += [f'{side}_T{i}' for i in range(1, n + 2)]
        return cols

    def _joints(self, side):
        for _, rows in self.poses:
            row = rows.get(side)
            if row and row.get('positions_rad') is not None:
                return len(row['positions_rad'])
        return 0

    def table(self):
        nan = float('nan')
        out = []
        for t, rows in self.poses:
            line = [t]
            for side in self.sides:
                n = self._joints(side)
                row = rows.get(side) or {}
                def vec(key, k):
                    v = row.get(key)
                    return [float(x) for x in v][:k] if isinstance(v, list) and len(v) >= k else [nan] * k
                temps = vec('motor_temps_c', n + 1)
                effort = row.get('gripper_motor_effort')
                line += vec('positions_rad', n) + vec('commanded_rad', n)
                line += [float(row.get('gripper', nan) if row.get('gripper') is not None else nan),
                         float(row.get('commanded_gripper') if row.get('commanded_gripper') is not None else nan),
                         float(effort) if isinstance(effort, (int, float)) else nan] + temps
            out.append(line)
        return out

    def _write_joints(self):
        import numpy as np
        rows = np.array(self.table(), dtype=np.float64) if self.poses else np.zeros((0, len(self.columns())))
        tmp = self.paths.recording / 'joints.tmp.npz'
        np.savez(tmp, rows=rows, columns=np.array(self.columns()), jaw_range=np.array(json.dumps(self.jaw_range)))
        os.replace(tmp, self.paths.recording / 'joints.npz')
        return dict(rows=int(len(rows)))

    # ---- the dataset episode
    def episode(self, fps):
        """append_episode's arguments (no robot, root or prompt) on a uniform clock at fps, or raises ValueError why not."""
        import numpy as np
        if len(self.poses) < 2:
            raise ValueError('too short to record')
        cols = self.columns()
        data = np.array(self.table(), dtype=np.float64)
        t = data[:, 0]
        roles = [r for r in self.roles if len(self.frames[r]) >= 2]
        if len(roles) != len(self.roles):
            missing = [r for r in self.roles if r not in roles]
            raise ValueError(f'the {missing[0]} camera recorded fewer than two frames')
        acq = {r: np.array([f[0] for f in self.frames[r]]) for r in roles}
        t0 = max([t[0]] + [a[0] for a in acq.values()])
        t1 = min([t[-1]] + [a[-1] for a in acq.values()])
        T = int(np.floor((t1 - t0) * fps + 1e-9)) + 1
        if t1 <= t0 or T < 2:
            raise ValueError('shorter than two frames at this fps')
        tf = t0 + np.arange(T) / fps

        def col(name, required):
            v = data[:, cols.index(name)]
            ok = np.isfinite(v)
            if ok.sum() < 2:
                if required:
                    raise ValueError(f'{name}: fewer than 2 readings')
                return np.full(T, np.nan)
            return np.interp(tf, t[ok], v[ok])
        state, action, grip, effort, temp = [], [], [], [], []
        names = dict(state=[], grip=[], effort=[], temp=[])
        units = []
        for side in self.sides:
            n = self._joints(side)
            for i in range(1, n + 1):
                state.append(col(f'{side}_q{i}', True)); action.append(col(f'{side}_c{i}', True))
                names['state'].append(f'{side}_joint{i}'); units.append('rad')
                effort.append(np.full(T, np.nan)); names['effort'].append(f'{side}_joint{i}_effort')
                temp.append(col(f'{side}_T{i}', False)); names['temp'].append(f'{side}_joint{i}_temp_c')
            jaw = col(f'{side}_jaw', True)
            state.append(jaw); action.append(col(f'{side}_jaw_cmd', True)); grip.append(jaw)
            names['state'].append(f'{side}_gripper'); units.append('0..1')
            names['grip'].append(f'{side}_gripper')
            effort.append(col(f'{side}_gripper_effort', False)); names['effort'].append(f'{side}_gripper_effort')
            temp.append(col(f'{side}_T{n + 1}', False)); names['temp'].append(f'{side}_gripper_temp_c')
        f32 = lambda c: np.stack(c, 1).astype(np.float32)
        features = {
            'observation.state': dict(names=names['state'], units=units, dtype='float32'),
            'action': dict(names=names['state'], units=units, dtype='float32'),
            'observation.gripper': dict(names=names['grip'], units='0..1', dtype='float32'),
            'observation.effort': dict(names=names['effort'], units='motor effort, normalized -1..1 (gripper motor only)',
                                       dtype='float32'),
            'observation.motor_temperature_c': dict(names=names['temp'], units='C', dtype='float32'),
        }
        rows = {'observation.state': f32(state), 'action': f32(action), 'observation.gripper': f32(grip),
                'observation.effort': f32(effort), 'observation.motor_temperature_c': f32(temp)}
        cameras = {}
        for role in roles:
            a = acq[role]
            k = np.clip(np.searchsorted(a, tf), 1, len(a) - 1)
            pick = np.where(np.abs(a[k - 1] - tf) <= np.abs(a[k] - tf), k - 1, k)
            with open(self.paths.recording / f'{role}.mjpg', 'rb') as f:
                items = []
                for i, p in enumerate(pick):
                    _, off, length = self.frames[role][int(p)]
                    f.seek(off)
                    items.append((i, f.read(length)))
            cameras['observation.images.' + CAMERA_KEYS[role]] = items
        return dict(fps=float(fps), features=features, rows=rows, cameras=cameras)


def robot_wants_dataset(config, runtime_config=None):
    """Whether this run may write a dataset episode: (True, None) or (False, why). The same rule as the runtime's LeRobot
    export: the runtime's config.json `lerobot_export.enabled`, when it is
    true or false, wins either way; otherwise an installed robot (robot.json) keeps its dataset unless its user turned it off
    with `roborun up --no-dataset` (recording.dataset false) or `--no-data-sync` (data.opt_out). No robot.json: no episode.
    A run's own --no-dataset can only turn an episode off, never on."""
    given = (runtime_config or {}).get('lerobot_export') if isinstance(runtime_config, dict) else None
    enabled = given.get('enabled') if isinstance(given, dict) else None
    if enabled is False:
        return False, 'the dataset export is off (config.json lerobot_export.enabled)'
    if enabled is True:
        return True, None
    if config is None:
        return False, 'no robot.json on this computer'
    if (config.get('recording') or {}).get('dataset') is False:
        return False, "the robot's dataset is off (roborun up --no-dataset)"
    if (config.get('data') or {}).get('opt_out'):
        return False, "this robot's data is not kept (roborun up --no-data-sync)"
    return True, None


RECORD_CAMERAS = ('top', 'left', 'right')
RECORD_DEFAULTS = dict(run_cap_mb=500.0, day_cap_mb=8000.0, total_cap_mb=10000.0, min_free_mb=2000.0)
RECORD_LIMITS = dict(run_cap_mb=(1.0, 100000.0), day_cap_mb=(1.0, 1000000.0), total_cap_mb=(1.0, 10000000.0),
                     min_free_mb=(0.0, 100000000.0))
MB = 1024 * 1024


def recording_settings(runtime_config):
    """Whether this robot records its tasks, as the runtime's recorder decides from config.json `video_record`: dict(enabled,
    why, cameras, run_cap_mb, day_cap_mb, total_cap_mb, min_free_mb). Off by default: only the JSON value true for `enabled`
    turns it on, and an invalid camera list or cap turns it off (the runtime's own rule; the other recorder settings are the
    runtime's to read)."""
    given = (runtime_config or {}).get('video_record') if isinstance(runtime_config, dict) else None
    given = given if isinstance(given, dict) else {}
    out = dict(RECORD_DEFAULTS)
    out.update({k: given[k] for k in RECORD_DEFAULTS if k in given})
    problems = []
    for k, (lo, hi) in RECORD_LIMITS.items():
        v = out[k]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not lo <= v <= hi:
            problems.append(f'invalid video_record.{k}={v!r} (allowed {lo}..{hi})')
            out[k] = RECORD_DEFAULTS[k]
    cams = given.get('cameras', list(RECORD_CAMERAS))
    if (not isinstance(cams, (list, tuple)) or not cams or any(c not in RECORD_CAMERAS for c in cams)
            or len(set(cams)) != len(cams)):
        problems.append(f'invalid video_record.cameras={cams!r}')
        cams = list(RECORD_CAMERAS)
    out['cameras'] = list(cams)
    if given.get('enabled') is not True:
        out.update(enabled=False, why='off (config.json video_record.enabled is not true)')
    elif problems:
        out.update(enabled=False, why=problems[0])
    else:
        out.update(enabled=True, why=None)
    return out


# ---- retention: the same rule as the runtime's own recorder (workspaces/yam/policies/video_record.py `_budget`)
DIR_RE = re.compile(r'recording-[0-9a-f]{8,32}-\d{2,4}')
CLIP_RE = re.compile(r'[0-9a-f]{8,64}')
RUNTIME_RECORDER = Path('workspaces') / 'yam' / 'policies' / 'video_record.py'
CAPS = dict(disk='min_free_mb', day='day_cap_mb', total='total_cap_mb', run='run_cap_mb')


def _recordings(root):
    """[(created_at, date, bytes, dir)] for every recording folder under root that is provably a recorder's own:
    <root>/<clip>/recording-<task>-<NN>, neither a symlink, holding a recording.json of this schema (the runtime's rule)."""
    out = []
    try:
        clips = list(os.scandir(root))
    except OSError:
        return out
    for c in clips:
        try:
            if not c.is_dir(follow_symlinks=False) or not CLIP_RE.fullmatch(c.name):
                continue
            subs = [e for e in os.scandir(c.path) if e.name.startswith('recording-')]
        except OSError:
            continue
        for e in subs:
            try:
                if not e.is_dir(follow_symlinks=False) or not DIR_RE.fullmatch(e.name):
                    continue
                mj = os.path.join(e.path, 'recording.json')
                if os.path.islink(mj) or not os.path.isfile(mj):
                    continue
                with open(mj) as f:
                    meta = json.load(f)
                if not isinstance(meta, dict) or meta.get('schema') != SCHEMA:
                    continue
                size = sum(f2.stat(follow_symlinks=False).st_size for f2 in os.scandir(e.path)
                           if f2.is_file(follow_symlinks=False))
                out.append((str(meta.get('created_at') or ''), str(meta.get('created_local_date') or ''), size, e.path))
            except (OSError, ValueError):
                continue
    return out


def _delete_recording(path):
    """Delete one recording folder: only the regular files (and symlinks, never followed) directly inside, then the folder
    (left in place when anything else is inside). Returns the bytes freed."""
    freed = 0
    for e in os.scandir(path):
        try:
            if e.is_file(follow_symlinks=False) or e.is_symlink():
                if e.is_file(follow_symlinks=False):
                    freed += e.stat(follow_symlinks=False).st_size
                os.unlink(e.path)
        except OSError:
            pass
    try:
        os.rmdir(path)
    except OSError:
        pass
    return freed


def _budget(cfg, here, root):
    """The runtime recorder's check before recording, line for line: (None or 'disk' | 'day' | 'total', today's bytes
    before this run, the recordings deleted). Deletes whole recordings, oldest first (created_at), never `here`, until
    all of them leave room for run_cap_mb under total_cap_mb."""
    import shutil
    try:
        if shutil.disk_usage(here).free < cfg['min_free_mb'] * MB:
            return 'disk', 0, []
    except OSError:
        pass
    mine = os.path.realpath(here)
    today = datetime.date.today().isoformat()
    recs = [r for r in _recordings(root) if os.path.realpath(r[3]) != mine]
    day = sum(r[2] for r in recs if r[1] == today)
    if day >= cfg['day_cap_mb'] * MB:
        return 'day', day, []
    total = sum(r[2] for r in recs)
    cleanup = []
    for created, date, size, path in sorted(recs):          # oldest first (created_at is ISO UTC)
        if total + cfg['run_cap_mb'] * MB <= cfg['total_cap_mb'] * MB:
            break
        freed = _delete_recording(path)
        total -= size
        cleanup.append(dict(dir=os.path.relpath(path, root), created_at=created, bytes=size, freed=freed))
        if date == today:
            day -= size
    if total + cfg['run_cap_mb'] * MB > cfg['total_cap_mb'] * MB:
        return 'total', day, cleanup
    return None, day, cleanup


_RUNTIME_RECORDERS = {}
_RUNTIME_LOCK = threading.Lock()


def runtime_budget(runtime):
    """The installed runtime's own `_budget` (<runtime>/workspaces/yam/policies/video_record.py), the function that prunes
    the recordings of Roborun's own tasks, or None when that file is missing or cannot be loaded. Loaded without writing
    bytecode into the runtime, once per file version."""
    if runtime is None:
        return None
    path = Path(runtime) / RUNTIME_RECORDER
    try:
        st = path.stat()
    except OSError:
        return None
    key = (str(path), st.st_mtime_ns, st.st_size)
    with _RUNTIME_LOCK:
        if key not in _RUNTIME_RECORDERS:
            fn = None
            try:
                import types
                mod = types.ModuleType('_roborun_runtime_video_record')
                mod.__file__ = str(path)
                exec(compile(path.read_text(), str(path), 'exec'), mod.__dict__)
                fn = getattr(mod, '_budget', None)
                fn = fn if callable(fn) else None
            except Exception:
                fn = None
            _RUNTIME_RECORDERS[key] = fn
        return _RUNTIME_RECORDERS[key]


def cap_text(code, settings):
    """A recorder cap as words for the task record: which config.json video_record setting stopped (or kept from
    starting) the recording."""
    if code in ('disk', 'min_free_mb'):
        return f"less than {settings['min_free_mb']:g} MB free on the disk (config.json video_record.min_free_mb)"
    if code in ('day', 'day_cap_mb'):
        return f"today's recordings reached {settings['day_cap_mb']:g} MB (config.json video_record.day_cap_mb)"
    if code in ('total', 'total_cap_mb'):
        return (f"no room for a {settings['run_cap_mb']:g} MB run under {settings['total_cap_mb']:g} MB of recordings, even "
                f"after deleting the oldest (config.json video_record.total_cap_mb)")
    if code in ('run', 'run_cap_mb'):
        return f"this run's recording reached {settings['run_cap_mb']:g} MB (config.json video_record.run_cap_mb)"
    if code == 'disk_full':
        return 'the disk is full: writing a frame failed'
    return str(code)


def make_room(settings, here, video_root, runtime=None):
    """Before a run records into `here` (its recording folder, which must exist) under `video_root`
    (the runtime's recordings folder): the cleanup an ordinary task's recording does, by the installed runtime's own `_budget`
    when it can be loaded, else by this module's copy of it. The oldest recordings (Roborun's tasks' and policy runs'
    alike) are deleted until there is room for run_cap_mb under total_cap_mb. Returns dict(why: None or why this run
    records nothing, cap: the cap's code, day_bytes: today's bytes before this run, cleanup: [{dir, created_at, bytes,
    freed}], rule: 'runtime' or 'roborun-sdk')."""
    cfg = {k: settings[k] for k in ('min_free_mb', 'day_cap_mb', 'total_cap_mb', 'run_cap_mb')}
    result, rule = None, 'roborun-sdk'
    fn = runtime_budget(runtime)
    if fn is not None:
        import contextlib
        import io
        try:
            with contextlib.redirect_stderr(io.StringIO()):   # its cleanup lines: the run reports the cleanup itself
                got = fn(dict(cfg), str(here), str(video_root))
            if (isinstance(got, tuple) and len(got) == 3 and got[0] in (None, 'disk', 'day', 'total')
                    and isinstance(got[2], list)):
                result, rule = got, 'runtime'
        except Exception:
            result = None
    if result is None:
        try:
            result = _budget(cfg, str(here), str(video_root))
        except Exception as exc:                          # as the runtime's recorder: a failed check records nothing
            return dict(why=f'the disk check failed ({type(exc).__name__}: {exc})'[:200], cap='disk', day_bytes=0,
                        cleanup=[], rule=rule)
    code, day, cleanup = result
    cleanup = [c for c in cleanup if isinstance(c, dict)]
    return dict(why=cap_text(code, settings) if code else None, cap=CAPS.get(code), day_bytes=int(day or 0),
                cleanup=cleanup, rule=rule)


def robot_runtime(config=None):
    """The runtime the robot runs from, by robot.json `config` (a dict): its `manifest`, else this install's default runtime;
    for a simulated arm robot (robot.json `sim`, `roborun up --sim`), the simulation's own copy of it (sim_home.runtime()),
    which its backend runs from and whose log/ its history lists, as `roborun profiles` and the data sync decide.
    A policy run's task record, recording and orphan state go there, and its Stop and `roborun policy runs` look there."""
    from .. import robot
    if isinstance(config, dict) and config.get('sim') and not config.get('body'):
        from .. import sim_home
        if (sim_home.runtime() / 'config.json').exists():
            return sim_home.runtime()
    manifest = config.get('manifest') if isinstance(config, dict) else None
    return Path(manifest if isinstance(manifest, str) and manifest else robot._manifest_root())


def runtime_config(runtime):
    """The runtime's config.json (<runtime>/config.json, what the backend runs with), or None."""
    try:
        got = json.loads((Path(runtime) / 'config.json').read_text())
    except (OSError, ValueError):
        return None
    return got if isinstance(got, dict) else None


def not_real(recorder, config=None):
    """Why this run's recording must not join the robot's dataset (a simulation), else None. The same rule as the runtime's
    LeRobot export: every recorded camera must name a serial, and none may look simulated."""
    if (config or {}).get('sim'):
        return 'simulated robot'
    for role in recorder.roles:
        serials = recorder.serials.get(role) or set()
        if role in recorder.simulated or any(any(w in s.lower() for w in SIM_SERIAL_WORDS) for s in serials):
            return 'simulated cameras'
        if recorder.counts.get(role) and (not serials or '' in serials):
            return f'the {role} camera names no serial, so it cannot be shown to be a real camera'
    return None


def write_episode(recorder, *, robot, profile, prompt, extras, dataset_root=None, say=None, config=None):
    """Stage the run's episode for this robot's dataset (encoded by a nice-19 child). {recorded, why?}. A simulated run's
    episode is not written: the dataset is the real robot's (its episodes are what an export or a fine-tune sends)."""
    why = not_real(recorder, config)
    if why:
        return dict(recorded=False, why=why)
    try:
        from .. import dataset
    except ImportError:
        return dict(recorded=False, why='the recording tools are not installed (roborun up sets them up)')
    why = dataset.unavailable()
    if why:
        return dict(recorded=False, why=why)
    if dataset_root is None:
        from ..robot import dataset_root as root_of
        dataset_root = root_of(robot)
    info = dataset.load_info(dataset_root)
    fps = float((info or {}).get('fps') or DEFAULT_FPS)        # the dataset's first episode fixes its fps
    try:
        ep = recorder.episode(fps)
    except ValueError as exc:
        return dict(recorded=False, why=str(exc))
    depth = {k[len(dataset.DEPTH_PREFIX):]: [] for k in ((info or {}).get('features') or {})
             if k.startswith(dataset.DEPTH_PREFIX)}            # a dataset with depth: this episode's depth index is all -1
    try:
        folder = dataset.append_episode_background(dataset_root, robot_type=profile, robot=robot, profile=profile,
                                                   prompt=prompt, extras=extras, depth=depth or None, **ep)
    except Exception as exc:
        return dict(recorded=False, why=f'{type(exc).__name__}: {exc}'[:200])
    return dict(recorded=True, fps=fps, frames=len(next(iter(ep['rows'].values()))), staged=str(folder))
