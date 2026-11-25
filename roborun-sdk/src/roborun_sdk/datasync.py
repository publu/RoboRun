"""This robot's history -> its Roborun account (on unless `roborun up --no-data-sync`).

Uploads, incrementally and at low priority, what this robot records:
- video-motion/: each motion folder (plan, without the hosted service's request fields; timeline; the robot's own source frame) with its recordings (recording-<task>-NN/:
  recording.json, joints.npz, frames.jsonl, camera frames) and execution-<task>.npz files;
- tasks/: the task records (nothing on the robot is changed or deleted).
Each file goes up once (content-addressed: an upload the API already has is skipped), then
POST /v1/robots/<robot>/data places it under robots/<robot>/data/<source>/<path>. Progress is kept in a state file, so
a restart resumes where it stopped.

Every image, video and depth frame is blurred on its way out (camera_privacy.py): what is on this computer stays sharp, what goes
up has faces hidden, including recordings older than this rule. Recording frames keep their exact bytes unless a face is in them;
the uploaded frames.jsonl follows the new byte offsets. Whatever cannot be blurred stays here, is never marked as done, and is
counted as `withheld`: a failed blur, a file type the blur cannot read, images embedded in text. With `privacy: {face_blur: false}`
no image or depth frame is uploaded at all; task records and other text still are.
"""
from __future__ import annotations

import ast
import hashlib
import json
import os
import re
import shutil
import tempfile
import threading
import time
import urllib.error
import urllib.request
import zipfile
from pathlib import Path

from . import camera_privacy as privacy

MAX_FILE = 64 << 20                 # the API's blob limit
UPLOAD_BPS = 2_000_000              # the history never takes the robot's whole uplink
BATCH = 200                         # files per placement call
SKIP_DIRS = {'sessions', '__pycache__', '.git'}
VIDEO_SUFFIXES = {'.mp4', '.m4v', '.mov', '.avi', '.mkv', '.webm', '.h264', '.h265', '.ts'}
# plan fields that stay on this computer: the words of the request (every field whose name starts with LOCAL_PLAN_PREFIXES),
# the hosted service's own sections (every field whose name ends with LOCAL_PLAN_SUFFIXES) and its request details
# (LOCAL_PLAN_KEYS)
LOCAL_PLAN_PREFIXES = ('prompt',)
LOCAL_PLAN_SUFFIXES = ('_request', '_result')
LOCAL_PLAN_KEYS = ('client_timings', 'queue', 'request_id', 'media_url', 'video_url', 'model', 'resolution', 'wording')


def _local(key):
    return key in LOCAL_PLAN_KEYS or key.startswith(LOCAL_PLAN_PREFIXES) or key.endswith(LOCAL_PLAN_SUFFIXES)


def strip_plan(data):
    """plan.json bytes without its local fields; anything that is not a JSON object goes up as nothing. Of the local
    fields only the goal's `used` and `why` go up (the history shows them), from the service's result section."""
    try:
        plan = json.loads(data)
    except ValueError:
        return b'{}'
    if not isinstance(plan, dict):
        return b'{}'
    out = {k: v for k, v in plan.items() if not _local(k)}
    for k in sorted((k for k in plan if _local(k)), key=lambda k: not k.endswith('_result')):
        goal = plan[k].get('goal') if isinstance(plan[k], dict) else None
        if isinstance(goal, dict) and goal:
            out[k] = dict(goal={g: goal.get(g) for g in ('used', 'why') if g in goal})
            break
    return json.dumps(out, indent=2).encode()


FRAME_SUFFIXES = {'.jpg', '.jpeg', '.png', '.webp', '.bmp', '.gif', '.tif', '.tiff', '.mjpg', '.mjpeg', '.depth',
                  '.mp4', '.m4v', '.mov', '.avi', '.mkv', '.webm', '.h264', '.h265', '.ts'}
TEXT_SUFFIXES = {'.json', '.jsonl', '.txt', '.log', '.csv', '.yaml', '.yml', '.md'}
MAGIC = (b'\xff\xd8\xff', b'\x89PNG\r\n', b'GIF8', b'RIFF', b'\x1aE\xdf\xa3', b'BM')
EMBEDDED = re.compile(rb'data:image/|data:video/|/9j/4[A-Za-z0-9+/]{64}|iVBORw0KGgo[A-Za-z0-9+/]{64}')
IMAGE_NAMES = re.compile(r'(^|[_/.-])(rgb|bgr|image|images|img|frame|frames|depth|video)($|[_/.-])', re.I)


def _npy_is_image(head):
    """From an .npy header: 8-bit data with 3 or more dimensions, or 2 dimensions of 16-bit (a depth map)."""
    try:
        text = head[head.index(b'{'):head.index(b'}') + 1].decode('latin1')
        info = ast.literal_eval(text)
        kind, shape = info['descr'], info['shape']
    except (ValueError, SyntaxError, KeyError):
        return True                                       # unreadable: treated as a picture until shown otherwise
    return (kind in ('|u1', '|i1') and len(shape) >= 3) or (kind in ('<u2', '>u2') and len(shape) == 2 and min(shape) > 16)


def carries_frames(path):
    """Does this file hold camera images, judged from its name and first bytes? Text files are checked once read."""
    suffix = path.suffix.lower()
    if suffix in FRAME_SUFFIXES:
        return True
    if suffix in TEXT_SUFFIXES:
        return False
    try:
        if suffix == '.npz':
            with zipfile.ZipFile(path) as bundle:
                for name in bundle.namelist():
                    if IMAGE_NAMES.search(name):
                        return True
                    with bundle.open(name) as member:
                        if _npy_is_image(member.read(256)):
                            return True
            return False
        with open(path, 'rb') as f:
            head = f.read(256)
    except (OSError, zipfile.BadZipFile):
        return True
    if suffix == '.npy':
        return _npy_is_image(head)
    return head.startswith(MAGIC) or head[4:8] == b'ftyp'


DATASET_DEPTH = re.compile(r'depth/chunk-(\d{3,})/([^/]+)/episode_(\d{6,})/(\d+)\.png$')


def dataset_depth(path):
    """(colour video, frame index) when `path` is a depth sidecar PNG of a LeRobot dataset (roborun_sdk.dataset's layout:
    <root>/depth/chunk-NNN/<camera>/episode_NNNNNN/<frame>.png beside <root>/meta/info.json), else None."""
    m = DATASET_DEPTH.search(Path(path).as_posix())
    if not m:
        return None
    root = Path(path).parents[4]
    if not (root / 'meta' / 'info.json').is_file():
        return None
    chunk, camera, episode, frame = m.groups()
    return root / 'videos' / f'chunk-{chunk}' / f'observation.images.{camera}' / f'episode_{episode}.mp4', int(frame)


IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png', '.webp', '.bmp', '.tif', '.tiff'}
VIDEO_SUFFIXES = {'.mp4', '.m4v', '.mov', '.avi'}
STREAM_SUFFIXES = {'.mjpg', '.depth'}
STILL_WRITING_S = 10


class DataSync:
    def __init__(self, api, key, robot, sources, state_file=None, log=print, busy=None, face_blur=True):
        self.face_blur = bool(face_blur)    # privacy.face_blur: false keeps every image and depth frame on this computer
        self.busy = busy                    # () -> a task is running: the upload waits, the robot's traffic comes first
        self.api, self.key, self.robot = api.rstrip('/'), key, robot
        self.sources = {name: Path(p) for name, p in sources.items() if p}       # e.g. {'video-motion': ..., 'tasks': ...}
        self.state_file = Path(state_file) if state_file else None
        self.state = self._load()
        self.log = log
        self.stats = dict(files=0, bytes=0, errors=0)
        self.withheld = {}
        self.scratch = None                 # blurred copies waiting to go up; deleted as soon as they have
        self.recording = None               # (folder, rewritten files) of the recording being sent
        self.video_hits = {}                # a dataset episode's video -> its hidden regions per frame (for its depth PNGs)

    # ---- state
    def _load(self):
        try:
            return json.loads(self.state_file.read_text()) if self.state_file and self.state_file.exists() else {}
        except ValueError:
            return {}

    def _save(self):
        if not self.state_file: return
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.state_file.with_name(self.state_file.name + '.tmp')
        tmp.write_text(json.dumps(self.state)); tmp.replace(self.state_file)

    # ---- API
    def _req(self, method, path, body=None, data=None, timeout=120):
        from . import net
        headers = {'Authorization': 'Bearer ' + self.key, 'User-Agent': net.user_agent()}
        if body is not None:
            data = json.dumps(body).encode(); headers['Content-Type'] = 'application/json'
        req = urllib.request.Request(self.api + path, data=data, method=method, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return r.status, (json.loads(r.read() or b'{}') if method != 'HEAD' else None)
        except urllib.error.HTTPError as e:
            if method == 'HEAD': return e.code, None
            raise RuntimeError(f'{method} {path}: {e.code} {e.read()[:200]!r}') from None

    def _blob(self, data):
        if not isinstance(data, privacy.Cleared):
            raise RuntimeError('a file that did not pass the face blur was not sent')
        sha = hashlib.sha256(data).hexdigest()
        if self._req('HEAD', f'/v1/blobs/{sha}')[0] != 200:
            self._req('PUT', f'/v1/blobs/{sha}', data=data)
            self.stats['bytes'] += len(data)
            if self.busy is not None: time.sleep(len(data) / UPLOAD_BPS)
        return sha

    def _place(self, files, bodies=None):
        for i in range(0, len(files), BATCH):
            part = dict(list(files.items())[i:i + BATCH])
            out = self._req('POST', f'/v1/robots/{self.robot}/data', dict(files=part))[1]
            if out.get('missing') and bodies:
                # the API no longer has those bytes (a newer version of the same file replaced them meanwhile): send them again
                for rel in out['missing']:
                    if rel in bodies:
                        self._req('PUT', f'/v1/blobs/{part[rel]}', data=bodies[rel]); self.stats['bytes'] += len(bodies[rel])
                out = self._req('POST', f'/v1/robots/{self.robot}/data', dict(files={k: part[k] for k in out['missing']}))[1]
            if out.get('missing'):
                raise RuntimeError(f"the API lost uploads for {out['missing'][:3]}")

    # ---- files
    def _files(self):
        for name, root in self.sources.items():
            if not root.is_dir(): continue
            for dirpath, dirs, names in os.walk(root, followlinks=False):
                dirs[:] = [d for d in dirs if d not in SKIP_DIRS and not d.startswith('.')]
                for n in names:
                    if n.startswith('.') or n.endswith(('.tmp', '.part', '.lock')): continue
                    p = Path(dirpath) / n
                    rel = p.relative_to(root).as_posix()
                    if rel.count('/') > 6: continue
                    yield f'{name}/{rel}', p

    def _wait(self, budget_s=None, started=None):
        """The robot's work comes first: while a task runs, the blur and the upload stand still."""
        while self.busy is not None and self.busy():
            if budget_s and started is not None and time.monotonic() - started > budget_s: break
            time.sleep(5)

    def _temp(self):
        if self.scratch is None:
            self.scratch = Path(tempfile.mkdtemp(prefix='roborun-outgoing-'))
            os.chmod(self.scratch, 0o700)
        return self.scratch

    def _drop_recording(self):
        if self.recording is not None:
            shutil.rmtree(self.recording[2], ignore_errors=True)
            self.recording = None

    def _from_recording(self, p):
        folder = p.parent
        if self.recording is None or self.recording[0] != folder:
            self._drop_recording()
            names = [folder / 'frames.jsonl', *folder.glob('*.mjpg'), *folder.glob('*.depth')]
            if any(time.time() - f.stat().st_mtime < STILL_WRITING_S for f in names):
                return None
            out = Path(tempfile.mkdtemp(dir=self._temp()))
            self.recording = (folder, privacy.clear_recording(folder, out, self._wait), out)
        if p.name not in self.recording[1]:
            raise privacy.BlurUnavailable(f'{privacy.NO_BLUR}: frames.jsonl does not list this file')
        return privacy.Cleared((self.recording[2] / p.name).read_bytes())

    def _outgoing(self, p, st):
        """The bytes to upload for this file, cleared; None to look again next pass."""
        suffix = p.suffix.lower()
        indexed = (p.parent / 'frames.jsonl').exists() and self.face_blur and (p.name == 'frames.jsonl' or suffix in STREAM_SUFFIXES)
        if indexed and st.st_size:
            return self._from_recording(p)
        if st.st_size == 0:
            return privacy.Cleared(b'')
        if p.name == 'plan.json':                               # request fields stay on this computer (strip_plan)
            return privacy.Cleared(strip_plan(p.read_bytes()))
        if suffix in TEXT_SUFFIXES:
            data = p.read_bytes()
            if EMBEDDED.search(data):
                raise privacy.BlurUnavailable('an image embedded in a text file cannot be blurred')
            return privacy.Cleared(data)
        if not carries_frames(p):
            return privacy.Cleared(p.read_bytes())
        if not self.face_blur:
            raise privacy.BlurUnavailable('face blur is off (privacy: face_blur: false): images stay on this computer')
        with open(p, 'rb') as f:
            head = f.read(16)
        if suffix in ('.npz',):
            return privacy.clear_arrays_file(p.read_bytes())
        if suffix == '.npy':
            raise privacy.BlurUnavailable(f'{privacy.NO_BLUR}: this file holds images in a form that cannot be blurred')
        if suffix in VIDEO_SUFFIXES or head[4:8] == b'ftyp':
            out = self._temp() / f'{hashlib.sha256(str(p).encode()).hexdigest()[:16]}.mp4'
            try:
                return privacy.Cleared(privacy.clear_video(p, out, self._wait).read_bytes())
            finally:
                out.unlink(missing_ok=True)
        hit = dataset_depth(p) if suffix == '.png' else None
        if hit is not None:
            return self._dataset_depth(p, *hit)
        if suffix in IMAGE_SUFFIXES or head.startswith((b'\xff\xd8\xff', b'\x89PNG\r\n')):
            return privacy.clear_image(p.read_bytes())
        raise privacy.BlurUnavailable(f'{privacy.NO_BLUR}: this file type cannot be blurred')

    def _dataset_depth(self, p, video, frame):
        """A LeRobot dataset's depth sidecar PNG (16-bit): the regions hidden in its colour frame (the episode's video, tracked
        as clear_video tracks it) get no reading; it stays 16-bit, single channel, on the same scale. No video: withheld."""
        if not video.is_file():
            raise privacy.BlurUnavailable(f'{privacy.NO_BLUR}: the colour video of this depth frame is missing')
        key = str(video)
        if key not in self.video_hits:
            self.video_hits = {key: privacy.video_hits(video, self._wait)}         # one episode's video at a time
        hits, shape, _ = self.video_hits[key]
        if not 0 <= frame < len(hits):
            raise privacy.BlurUnavailable(f'{privacy.NO_BLUR}: this depth frame is not in its colour video')
        return privacy.clear_depth_png(p.read_bytes(), hits[frame], shape)

    STORE_EVERY_S = 600

    def _check_store(self):
        """The API keeps what this robot sent in one store (GET /v1/health: store). A store that is new to this robot has none
        of it, so everything goes up again, once. Asked at most every STORE_EVERY_S; unknown keeps the state."""
        now = time.monotonic()
        if now - getattr(self, '_store_at', -1e9) < self.STORE_EVERY_S:
            return
        self._store_at = now
        try:
            out = self._req('GET', '/v1/health', timeout=30)[1] or {}
        except Exception:
            return
        store = out.get('store') if isinstance(out, dict) else None
        if not isinstance(store, str) or not store or self.state.get('store') == store:
            return
        if self.state.get('files'):
            self.log(f"data sync: the Roborun API keeps a new store ({store}); this robot's history goes up again")
        self.state['files'] = {}
        self.state['store'] = store
        self._save()

    def sync_files(self, budget_s=None):
        self._check_store()
        seen = self.state.setdefault('files', {})
        started = time.monotonic(); pending = {}; marks = {}; bodies = {}
        for rel, p in self._files():
            try:
                st = p.stat()
            except OSError:
                continue
            mark = f'{st.st_size}:{int(st.st_mtime)}'
            if seen.get(rel) == mark or st.st_size > MAX_FILE: continue
            if p.suffix.lower() in VIDEO_SUFFIXES and not rel.startswith('datasets/'):
                continue                                         # the robot records MJPEG; other video files are not robot recordings
            if time.time() - st.st_mtime < STILL_WRITING_S: continue          # still being written: next pass
            self._wait(budget_s, started)
            try:
                data = self._outgoing(p, st)
            except privacy.BlurUnavailable as exc:
                self.withheld[rel] = str(exc); continue          # re-judged every pass, never marked as done
            except OSError:
                continue
            if data is None:
                continue                                         # a recording still being written: next pass
            self.withheld.pop(rel, None)
            pending[rel] = self._blob(data); marks[rel] = mark; bodies[rel] = data
            if len(pending) >= BATCH:
                self._place(pending, bodies); seen.update(marks); self.stats['files'] += len(pending); pending, marks, bodies = {}, {}, {}; self._save()
            if budget_s and time.monotonic() - started > budget_s: break
        if pending:
            self._place(pending, bodies); seen.update(marks); self.stats['files'] += len(pending); self._save()
        self._drop_recording()
        self.video_hits = {}
        if self.scratch is not None:
            shutil.rmtree(self.scratch, ignore_errors=True); self.scratch = None

    def once(self, budget_s=None):
        t0 = time.monotonic()
        self.sync_files(budget_s)
        return dict(self.stats, withheld=len(self.withheld), seconds=round(time.monotonic() - t0, 1))

    def run_forever(self, every_s=120):
        reported = 0
        try:
            import cv2
            cv2.setNumThreads(1)            # the blur never takes more than one core from the robot
        except ImportError:
            pass
        while True:
            try:
                out = self.once(budget_s=600)
                if out['files']:
                    self.log(json.dumps(dict(event='data_synced', **out)))
                if out['withheld'] != reported:
                    reported = out['withheld']
                    self.log(json.dumps(dict(event='data_sync_withheld', files=reported, why='they could not be blurred, '
                                             'so they stay on this computer', example=next(iter(self.withheld), None))))
                self.stats = dict(files=0, bytes=0, errors=0)
            except Exception as exc:
                self.log(json.dumps(dict(event='data_sync_error', error=f'{type(exc).__name__}: {exc}'[:300])))
            time.sleep(every_s)

    def start(self, every_s=120):
        threading.Thread(target=self.run_forever, args=(every_s,), daemon=True, name='data-sync').start()
