"""This robot's LeRobot v2.1 dataset: one episode appended per finished recording (arms or humanoid).

One writer for every recorder (schema 'roborun-dataset/1'). The layout is LeRobot's v2.1 layout, which `LeRobotDataset(repo_id,
root=<folder>)` from lerobot 0.3.x loads as it is, plus a depth sidecar LeRobot ignores:

    <root>/meta/info.json               codebase_version v2.1, robot_type, fps, totals, splits, paths, features, "roborun" block
    <root>/meta/episodes.jsonl          {episode_index, tasks: [the user's prompt], length, ...extras}
    <root>/meta/episodes_stats.jsonl    per episode and feature: min, max, mean, std, count
    <root>/meta/tasks.jsonl             {task_index, task}, one line per distinct prompt
    <root>/meta/depth.json              {camera: {depth_scale_m, intrinsics}}
    <root>/data/chunk-000/episode_000000.parquet
    <root>/videos/chunk-000/observation.images.<camera>/episode_000000.mp4    H.264, yuv420p, frame i at i/fps
    <root>/depth/chunk-000/<camera>/episode_000000/<frame_index>.png         16-bit depth, value * depth_scale_m = metres

Crash safety: every file of the episode is written under a temporary name and renamed into place; the meta files are replaced
(write + rename) after the data, info.json last. info.json's total_episodes is what counts: an append first drops any meta line
and any file of an episode index >= total_episodes (left by a crash), so a half-written episode is never listed. Appends to one
root are serialised with a lock file.

Needs the `dataset` extra (pyarrow, PyAV), which `roborun up` installs once: pip install 'roborun-sdk[dataset]'. This module imports
without it; append_episode raises DatasetToolsMissing (an ImportError) naming what is missing.
"""
from __future__ import annotations

import fractions
import json
import math
import os
import shutil
from pathlib import Path

from . import net

SCHEMA = 'roborun-dataset/1'
CODEBASE_VERSION = 'v2.1'
CHUNKS_SIZE = 1000
DATA_PATH = 'data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet'
VIDEO_PATH = 'videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4'
DEPTH_DIR = 'depth/chunk-{episode_chunk:03d}/{camera}/episode_{episode_index:06d}'
IMAGE_PREFIX = 'observation.images.'
DEPTH_PREFIX = 'observation.depth_index.'
OWN = ('timestamp', 'frame_index', 'episode_index', 'index', 'task_index')
DTYPES = ('float32', 'float64', 'int64', 'int32', 'bool')
VIDEO_CODEC = dict(codec='h264', encoder='libx264', pix_fmt='yuv420p', crf='23', g=2)
STATS_IMAGES = 100            # frames sampled per camera and episode for the image stats (LeRobot samples about as many)
EXTRA = "roborun-sdk[dataset]"


class DatasetError(ValueError):
    """The episode cannot be appended (the message says why; `plain` says it in plain words for `roborun status`); the
    dataset on disk is unchanged, and the refusal is noted in <root>/.refused.jsonl."""

    def __init__(self, message, plain=None):
        super().__init__(message)
        self.plain = plain or message


def _camera_words(key):
    return key[len(IMAGE_PREFIX):].replace('_', ' ') if key.startswith(IMAGE_PREFIX) else key


TOOLS = ('pyarrow', 'av')
_found = False                                                    # cached: both modules were found once


def _state_file():
    """Where `roborun up` remembers a failed install (the same ROBORUN_HOME as roborun_sdk.robot.HOME)."""
    return net.roborun_home() / 'robot' / 'dataset-tools.json'


def _tools_found():
    global _found
    if not _found:
        import importlib.util
        _found = all(importlib.util.find_spec(m) is not None for m in TOOLS)
    return _found


def unavailable():
    """None when episodes can be written (pyarrow and PyAV are installed); otherwise the plain reason. Cheap (find_spec, cached
    once found; the remembered failure is one small file read) and safe to call on every move; never installs anything."""
    if _tools_found():
        return None
    try:
        why = json.loads(_state_file().read_text()).get('why')
    except (OSError, ValueError, AttributeError):
        why = None
    if why:
        return f'the recording tools could not be installed: {why}'
    return 'the recording tools are not installed: run roborun up'


def setup_tools(log=print):
    """`roborun up` only, before the robot starts: install the dataset extra once (packages.ensure('dataset')). A failure is said
    once, remembered for unavailable(), and never stops `up`. Returns unavailable() after the attempt."""
    state = _state_file()
    if _tools_found():
        state.unlink(missing_ok=True)
        return None
    log('Setting up recording (about 290 MB, once)…')
    why = None
    try:
        from .arms import packages
        packages.ensure('dataset', 'recording')
        import importlib
        import importlib.util
        importlib.invalidate_caches()
        if not _tools_found():
            why = f"{', '.join(m for m in TOOLS if importlib.util.find_spec(m) is None)} still missing after the install"
    except BaseException as exc:                                  # ensure() raises SystemExit when pip fails
        if isinstance(exc, KeyboardInterrupt): raise
        why = ' '.join(str(exc).split())[-300:] or type(exc).__name__
    if why is None:
        state.unlink(missing_ok=True)
        return None
    try:
        state.parent.mkdir(parents=True, exist_ok=True)
        _write_text(state, json.dumps(dict(why=why)))
    except OSError:
        pass
    log(f'recording: the recording tools could not be installed ({why}); the robot runs without recording episodes')
    return unavailable()


def _imports_ok():
    try:
        import av  # noqa: F401
        import numpy  # noqa: F401
        import pyarrow  # noqa: F401
        import pyarrow.parquet  # noqa: F401
        return None
    except ImportError as exc:
        return exc.name or str(exc)


class DatasetToolsMissing(ImportError):
    """pyarrow or PyAV is not installed (`roborun up` installs the dataset extra once; see DatasetToolsMissing.missing)."""

    def __init__(self, missing):
        self.missing = missing
        super().__init__(f"writing the robot's dataset needs pyarrow and PyAV, and {missing} is not installed: "
                         f"pip install '{EXTRA}' (roborun up installs it once unless --no-dataset)")


def _need():
    gone = _imports_ok()
    if gone is not None:
        raise DatasetToolsMissing(gone)


# ------------------------------------------------------------------------------------------------ small helpers
def _plain_fps(fps):
    try:
        fps = float(fps)
    except (TypeError, ValueError):
        raise DatasetError(f'fps must be a number, not {fps!r}') from None
    if not math.isfinite(fps) or not 0 < fps <= 1000:
        raise DatasetError(f'fps must be in (0, 1000], not {fps}')
    return int(fps) if fps == int(fps) else fps


def _write_bytes(path, data):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp')
    with open(tmp, 'wb') as f:
        f.write(data); f.flush(); os.fsync(f.fileno())
    os.replace(tmp, path)


def _write_text(path, text):
    _write_bytes(path, text.encode())


def _jsonl(path):
    out = []
    try:
        for line in Path(path).read_text().splitlines():
            if line.strip():
                try: out.append(json.loads(line))
                except ValueError: break                         # a cut-off last line
    except FileNotFoundError:
        pass
    return out


def _dumps(obj):
    return json.dumps(obj, allow_nan=True)                       # NaN only where a feature was never reported (see _stats)


class _Lock:
    def __init__(self, root):
        self.path = Path(root) / '.append.lock'; self.f = None

    def __enter__(self):
        import fcntl
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.f = open(self.path, 'a+'); fcntl.flock(self.f, fcntl.LOCK_EX)
        return self

    def __exit__(self, *exc):
        import fcntl
        try: fcntl.flock(self.f, fcntl.LOCK_UN)
        finally: self.f.close()


# ------------------------------------------------------------------------------------------------ features and rows
def _user_features(features, rows):
    """Validate the caller's features and rows: {name: (info dict for info.json, array (T, n))}, T."""
    import numpy as np
    if not isinstance(features, dict) or not features:
        raise DatasetError('features must be a non-empty dict name -> {names, units, dtype}')
    if set(features) != set(rows or {}):
        raise DatasetError(f'rows and features must name the same features (features only: {sorted(set(features) - set(rows or {}))}, '
                           f'rows only: {sorted(set(rows or {}) - set(features))})')
    out, T = {}, None
    for name, ft in features.items():
        if not isinstance(name, str) or not name or '/' in name or name in OWN or name.startswith((IMAGE_PREFIX, DEPTH_PREFIX)):
            raise DatasetError(f'feature name {name!r} is reserved or invalid')
        names = list(ft.get('names') or [])
        if not names or not all(isinstance(n, str) and n for n in names) or len(set(names)) != len(names):
            raise DatasetError(f'{name}: names must be distinct non-empty strings')
        dtype = ft.get('dtype', 'float32')
        if dtype not in DTYPES:
            raise DatasetError(f'{name}: dtype {dtype!r} is not one of {DTYPES}')
        units = ft.get('units')
        if isinstance(units, (list, tuple)) and len(units) != len(names):
            raise DatasetError(f'{name}: units has {len(units)} entries for {len(names)} names')
        a = np.asarray(rows[name])
        if a.ndim == 1 and len(names) == 1: a = a[:, None]
        if a.ndim != 2 or a.shape[1] != len(names):
            raise DatasetError(f'{name}: rows must be (T, {len(names)}), got {a.shape}')
        if T is None: T = a.shape[0]
        if a.shape[0] != T:
            raise DatasetError(f'{name}: {a.shape[0]} rows, other features have {T}')
        a = a.astype(dtype)
        if dtype.startswith('float') and np.isinf(a).any():
            raise DatasetError(f'{name}: infinite values')
        info = dict(dtype=dtype, shape=[len(names)], names=names)
        if units is not None: info['units'] = list(units) if isinstance(units, (list, tuple)) else str(units)
        out[name] = (info, a)
    if not T:
        raise DatasetError('an episode needs at least one frame')
    return out, T


def _check_frames(kind, key, items, T, every_row):
    seen = set()
    for item in items:
        if not isinstance(item, (tuple, list)) or len(item) != 2:
            raise DatasetError(f'{kind} {key}: items must be (frame_index, bytes)')
        fi, data = item
        if isinstance(fi, bool) or not isinstance(fi, int) or not 0 <= fi < T or fi in seen:
            raise DatasetError(f'{kind} {key}: bad or repeated frame_index {fi!r} (T={T})')
        if not isinstance(data, (bytes, bytearray, memoryview)) or not len(data):
            raise DatasetError(f'{kind} {key}: frame {fi} has no bytes')
        seen.add(fi)
    if every_row and seen != set(range(T)):
        raise DatasetError(f'{kind} {key}: needs exactly one frame per row ({len(seen)} of {T})')


# ------------------------------------------------------------------------------------------------ video
def _encode_video(jpegs, out_path, fps, sample_every):
    """JPEG bytes (one per frame, in order) -> H.264 mp4 at `fps` (frame i at pts i, time base 1/fps). Returns
    (height, width, sampled RGB frames for the stats)."""
    import av
    import numpy as np
    rate = fractions.Fraction(fps).limit_denominator(1001)
    dec = av.CodecContext.create('mjpeg', 'r')
    tmp = Path(str(out_path) + '.tmp.mp4'); tmp.parent.mkdir(parents=True, exist_ok=True)
    samples, shape = [], None
    try:
        enc_ok = True
        try:
            av.codec.Codec(VIDEO_CODEC['encoder'], 'w')
        except Exception:
            enc_ok = False
        if not enc_ok:
            raise DatasetError('this PyAV build cannot encode H.264 (libx264 is missing); install the PyAV wheel from PyPI')
        with av.open(str(tmp), 'w', format='mp4') as box:
            stream = None
            for i, data in enumerate(jpegs):
                frames = dec.decode(av.Packet(bytes(data)))
                if len(frames) != 1:
                    raise DatasetError(f'frame {i}: not one decodable JPEG')
                frame = frames[0]
                if stream is None:
                    w, h = frame.width - frame.width % 2, frame.height - frame.height % 2   # yuv420p needs even sizes
                    shape = (h, w)
                    stream = box.add_stream(VIDEO_CODEC['encoder'], rate=rate)
                    stream.width, stream.height, stream.pix_fmt = w, h, VIDEO_CODEC['pix_fmt']
                    stream.codec_context.time_base = fractions.Fraction(1) / rate
                    stream.codec_context.gop_size = VIDEO_CODEC['g']
                    stream.options = {'crf': VIDEO_CODEC['crf'], 'preset': 'medium'}
                elif (frame.height - frame.height % 2, frame.width - frame.width % 2) != shape:
                    raise DatasetError(f'frame {i}: size {frame.width}x{frame.height} differs from the first frame')
                rgb = frame.to_ndarray(format='rgb24')[:shape[0], :shape[1]]
                if i % sample_every == 0:
                    samples.append(rgb[::4, ::4])                  # LeRobot also downsamples before its image stats
                out = av.VideoFrame.from_ndarray(np.ascontiguousarray(rgb), format='rgb24').reformat(format=VIDEO_CODEC['pix_fmt'])
                out.pts = i
                for packet in stream.encode(out):
                    box.mux(packet)
            for packet in stream.encode():
                box.mux(packet)
        os.replace(tmp, out_path)
    finally:
        tmp.unlink(missing_ok=True)
    return shape, samples


def _video_info(shape, fps):
    h, w = shape
    return dict(dtype='video', shape=[h, w, 3], names=['height', 'width', 'channels'],
                info={'video.height': h, 'video.width': w, 'video.codec': VIDEO_CODEC['codec'], 'video.pix_fmt': VIDEO_CODEC['pix_fmt'],
                      'video.is_depth_map': False, 'video.fps': fps, 'video.channels': 3, 'has_audio': False})


# ------------------------------------------------------------------------------------------------ stats (LeRobot v2.1 shape)
def _stats(a):
    """min/max/mean/std/count over the episode, per element. NaN (never reported) is left out; a column that is NaN in every
    row reports NaN, which LeRobot's json reader accepts."""
    import numpy as np
    a = np.asarray(a, dtype=np.float64)
    if a.ndim == 1: a = a[:, None]
    finite = np.isfinite(a)
    with np.errstate(all='ignore'):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            return dict(min=np.nanmin(a, 0).tolist(), max=np.nanmax(a, 0).tolist(), mean=np.nanmean(a, 0).tolist(),
                        std=np.nanstd(a, 0).tolist(), count=[int(finite.all(1).sum()) if finite.all() else int(len(a))])


def _image_stats(samples):
    import numpy as np
    x = np.stack(samples).astype(np.float64) / 255.                  # (n, h, w, 3) in [0, 1], channels first in the stats
    per = lambda f: [[[float(v)]] for v in f(x, axis=(0, 1, 2))]
    return dict(min=per(np.min), max=per(np.max), mean=per(np.mean), std=per(np.std), count=[len(samples)])


# ------------------------------------------------------------------------------------------------ recovery
def _committed(root):
    try:
        return json.loads((Path(root) / 'meta' / 'info.json').read_text())
    except FileNotFoundError:
        return None
    except ValueError as exc:
        raise DatasetError(f'{root}/meta/info.json cannot be read ({exc}); not appending to a damaged dataset') from None


def _drop_uncommitted(root, info):
    """Remove what a crashed append left: meta lines and files of episode indexes >= the committed total."""
    root = Path(root); total = int(info['total_episodes']) if info else 0
    for name in ('episodes.jsonl', 'episodes_stats.jsonl'):
        p = root / 'meta' / name
        if p.exists():
            lines = _jsonl(p); keep = [x for x in lines if int(x.get('episode_index', -1)) < total]
            if len(keep) != len(lines) or p.read_text().count('\n') != len(lines):
                _write_text(p, ''.join(_dumps(x) + '\n' for x in keep))
    for sub in ('data', 'videos', 'depth'):
        base = root / sub
        if not base.is_dir(): continue
        for p in list(base.rglob('*')):
            name = p.name
            if '.tmp' in name:
                shutil.rmtree(p, ignore_errors=True) if p.is_dir() else p.unlink(missing_ok=True); continue
            if name.startswith('episode_'):
                try: idx = int(name[len('episode_'):].split('.')[0])
                except ValueError: continue
                if idx >= total:
                    shutil.rmtree(p, ignore_errors=True) if p.is_dir() else p.unlink(missing_ok=True)


# ------------------------------------------------------------------------------------------------ the writer
def append_episode(root, *, robot_type: str, robot: str, profile: str, fps: float, features: dict, rows: dict,
                   cameras: dict = None, depth: dict = None, depth_meta: dict = None, prompt: str, extras: dict = None) -> int:
    """Append one episode to the LeRobot v2.1 dataset at `root` (created on the first call). Returns its episode_index.

    features: name -> {"names": [...], "units": str | [...], "dtype": "float32" | "int64" | ...}; shape = [len(names)].
    rows:     name -> array (T, len(names)); row i is frame i of the uniform clock, at i / fps seconds.
    cameras:  video key ("observation.images.<camera>") -> [(frame_index, jpeg_bytes)], exactly one per row.
    depth:    camera -> [(frame_index, png16_bytes)] for the rows that have depth; written to the sidecar and indexed by
              observation.depth_index.<camera> (the frame_index, -1 = none this row), which this function adds.
    depth_meta: camera -> {depth_scale_m, intrinsics}, merged into meta/depth.json.
    prompt:   the user's prompt (the episode's task). extras: merged into the episode's episodes.jsonl line.
    timestamp, frame_index, episode_index, index and task_index are added here.
    Columns: the dataset's first episode fixes them. A per-element column that is NaN in every row of that episode (never
    reported by this robot) is left out, and a feature left with no column is left out entirely; later episodes keep the
    dataset's columns (a column partly NaN stays as it is). Raises DatasetError (dataset unchanged, the reason noted in
    <root>/.refused.jsonl for `roborun status`) when the episode does not fit the dataset (cameras, data, fps or robot) or
    the input is malformed."""
    _need()
    try:
        return _append(root, robot_type=robot_type, robot=robot, profile=profile, fps=fps, features=features, rows=rows,
                       cameras=cameras, depth=depth, depth_meta=depth_meta, prompt=prompt, extras=extras)
    except DatasetError as exc:
        e = extras or {}
        refuse(root, str(exc), exc.plain, task_id=e.get('task_id'), recorded_at=e.get('recorded_at'))
        raise


def _reported(user, info):
    """The episode's features cut to the columns the dataset keeps: the first episode drops never-reported (all-NaN) columns
    and empty features; later episodes take the dataset's columns, in its order (a column the dataset lacks is dropped)."""
    import numpy as np
    out = {}
    for name, (ft, a) in user.items():
        if info is None:
            if not ft['dtype'].startswith('float'):
                out[name] = (ft, a); continue
            keep = ~np.isnan(a).all(0)
            if not keep.any():
                continue
            if keep.all():
                out[name] = (ft, a); continue
            idx = np.flatnonzero(keep)
        else:
            have = info['features'].get(name)
            if have is None:
                continue                                         # the dataset never had it: the robot did not report it then
            names = list(have.get('names') or [])
            if not set(names) <= set(ft['names']):
                out[name] = (ft, a); continue                    # the comparison below refuses it and says where
            idx = np.array([ft['names'].index(n) for n in names], int)
        ft = dict(ft, shape=[len(idx)], names=[ft['names'][i] for i in idx])
        if isinstance(ft.get('units'), list):
            ft['units'] = [ft['units'][i] for i in idx]
        out[name] = (ft, a[:, idx])
    return out


def _append(root, *, robot_type, robot, profile, fps, features, rows, cameras, depth, depth_meta, prompt, extras):
    import numpy as np
    import pyarrow as pa
    import pyarrow.parquet as pq
    fps = _plain_fps(fps)
    if not isinstance(prompt, str) or not prompt.strip():
        raise DatasetError('prompt must be the user\'s prompt (a non-empty string)')
    for k, v in dict(robot_type=robot_type, robot=robot, profile=profile).items():
        if not isinstance(v, str) or not v.strip():
            raise DatasetError(f'{k} must be a non-empty string')
    user, T = _user_features(features, rows)
    cameras = dict(cameras or {}); depth = dict(depth or {}); depth_meta = dict(depth_meta or {})
    for key, items in cameras.items():
        if not isinstance(key, str) or not key.startswith(IMAGE_PREFIX) or len(key) == len(IMAGE_PREFIX) or '/' in key:
            raise DatasetError(f'camera key {key!r} must be "{IMAGE_PREFIX}<camera>"')
        _check_frames('camera', key, items, T, True)
    for cam, items in depth.items():
        if not isinstance(cam, str) or not cam or '/' in cam or '.' in cam:
            raise DatasetError(f'depth camera name {cam!r} is invalid')
        _check_frames('depth', cam, items, T, False)
        for fi, data in items:
            if bytes(data[:8]) != b'\x89PNG\r\n\x1a\n':
                raise DatasetError(f'depth {cam}: frame {fi} is not a PNG')
    root = Path(root).expanduser()
    block = dict(schema=SCHEMA, profile=profile, robot=robot)

    with _Lock(root):
        info = _committed(root)
        _drop_uncommitted(root, info)
        if info is not None:
            if info.get('codebase_version') != CODEBASE_VERSION:
                raise DatasetError(f"{root} is a {info.get('codebase_version')} dataset, this writer appends {CODEBASE_VERSION}")
            have = info.get('roborun') or {}
            if have.get('profile') != profile or have.get('robot') != robot or info.get('robot_type') != robot_type:
                raise DatasetError(f"{root} belongs to robot {have.get('robot')!r} / profile {have.get('profile')!r} "
                                   f"/ robot_type {info.get('robot_type')!r}, not {robot!r} / {profile!r} / {robot_type!r}",
                                   plain=f"it was recorded with another robot setup ({profile}) than the dataset's ({have.get('profile')})")
            if info.get('fps') != fps:
                raise DatasetError(f"{root} is recorded at {info.get('fps')} fps, this episode at {fps}",
                                   plain=f"it was recorded at {fps} frames a second, the dataset at {info.get('fps')}")
            had = {k for k, v in info['features'].items() if v.get('dtype') == 'video'}
            if had != set(cameras):
                gone, extra = sorted(had - set(cameras)), sorted(set(cameras) - had)
                words = [f'the {_camera_words(k)} camera sent no frames' for k in gone]
                words += [f'it has a camera the dataset does not have ({_camera_words(k)})' for k in extra]
                raise DatasetError(f'this episode\'s cameras differ from the dataset\'s (missing {gone}, extra {extra})',
                                   plain='; '.join(words))
        user = _reported(user, info)
        index = int(info['total_episodes']) if info else 0
        start = int(info['total_frames']) if info else 0
        chunk = index // CHUNKS_SIZE
        fmt = dict(episode_chunk=chunk, episode_index=index)

        # task index (tasks.jsonl deduplicated by prompt)
        tasks = _jsonl(root / 'meta' / 'tasks.jsonl')
        by_task = {t['task']: int(t['task_index']) for t in tasks}
        new_task = prompt not in by_task
        task_index = by_task.get(prompt, len(tasks))

        # every feature of this episode, in a fixed order
        feats = {name: info_ for name, (info_, _) in user.items()}
        columns = {name: a for name, (_, a) in user.items()}
        for cam in sorted(depth):
            idx = np.full((T, 1), -1, np.int64)
            for fi, _ in depth[cam]: idx[fi, 0] = fi
            feats[DEPTH_PREFIX + cam] = dict(dtype='int64', shape=[1], names=None, units='frame_index (-1 = no depth)')
            columns[DEPTH_PREFIX + cam] = idx
        frame_index = np.arange(T, dtype=np.int64)[:, None]
        own = dict(timestamp=(np.arange(T, dtype=np.float64) / fps).astype(np.float32)[:, None], frame_index=frame_index,
                   episode_index=np.full((T, 1), index, np.int64), index=frame_index + start,
                   task_index=np.full((T, 1), task_index, np.int64))
        own_feats = dict(timestamp=dict(dtype='float32', shape=[1], names=None, units='s'),
                         frame_index=dict(dtype='int64', shape=[1], names=None), episode_index=dict(dtype='int64', shape=[1], names=None),
                         index=dict(dtype='int64', shape=[1], names=None), task_index=dict(dtype='int64', shape=[1], names=None))

        written = []
        try:
            # videos first (their shapes complete the features)
            stats, video_feats = {}, {}
            every = max(1, T // STATS_IMAGES)
            for key in sorted(cameras):
                items = sorted(cameras[key], key=lambda x: x[0])
                path = root / VIDEO_PATH.format(video_key=key, **fmt)
                written.append(path)
                shape, samples = _encode_video([d for _, d in items], path, fps, every)
                video_feats[key] = _video_info(shape, fps)
                stats[key] = _image_stats(samples)
            all_feats = {**{k: video_feats[k] for k in sorted(video_feats)}, **feats, **own_feats}
            if info is not None:
                have = info['features']
                if have != all_feats:
                    diff = sorted(k for k in set(have) | set(all_feats) if have.get(k) != all_feats.get(k))
                    raise DatasetError(f'this episode\'s features differ from the dataset\'s at {diff[:6]} '
                                       f'(dataset: {[have.get(k) for k in diff[:2]]}, episode: {[all_feats.get(k) for k in diff[:2]]})',
                                       plain='it recorded different data than the dataset holds (' + ', '.join(diff[:3]) + ')')

            # parquet: scalars as scalar columns, vectors as fixed-length lists (what LeRobot's own writer produces)
            arrays, fields = [], []
            for name, a in {**columns, **own}.items():
                ft = all_feats[name]; pa_type = pa.from_numpy_dtype(np.dtype(ft['dtype']))
                if ft['shape'] == [1]:
                    arr = pa.array(a[:, 0], type=pa_type)
                else:
                    arr = pa.FixedSizeListArray.from_arrays(pa.array(a.reshape(-1), type=pa_type), ft['shape'][0])
                arrays.append(arr); fields.append(pa.field(name, arr.type))
                stats[name] = _stats(a)
            hf = {k: (dict(dtype=v['dtype'], _type='Value') if v['shape'] == [1] else
                      dict(feature=dict(dtype=v['dtype'], _type='Value'), length=v['shape'][0], _type='Sequence'))
                  for k, v in all_feats.items() if v['dtype'] != 'video'}
            schema = pa.schema(fields, metadata={b'huggingface': json.dumps(dict(info=dict(features=hf))).encode()})
            data_path = root / DATA_PATH.format(**fmt); data_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = data_path.with_name(data_path.name + '.tmp'); written.append(data_path)
            pq.write_table(pa.Table.from_arrays(arrays, schema=schema), tmp)
            os.replace(tmp, data_path)

            # depth sidecar: a temporary folder renamed into place
            for cam in sorted(depth):
                final = root / DEPTH_DIR.format(camera=cam, **fmt)
                tmpdir = final.with_name(final.name + '.tmp'); shutil.rmtree(tmpdir, ignore_errors=True)
                tmpdir.mkdir(parents=True); written.append(final)
                for fi, data in depth[cam]:
                    (tmpdir / f'{fi}.png').write_bytes(bytes(data))
                if final.exists(): shutil.rmtree(final)
                os.replace(tmpdir, final)
        except BaseException:
            for p in written:
                shutil.rmtree(p, ignore_errors=True) if Path(p).is_dir() else Path(p).unlink(missing_ok=True)
            raise

        # meta: everything else first, info.json last (it is the commit)
        meta = root / 'meta'
        _write_text(meta / 'episodes_stats.jsonl', ''.join(_dumps(x) + '\n' for x in _jsonl(meta / 'episodes_stats.jsonl'))
                    + _dumps(dict(episode_index=index, stats=stats)) + '\n')
        if new_task:
            _write_text(meta / 'tasks.jsonl', ''.join(_dumps(x) + '\n' for x in tasks)
                        + _dumps(dict(task_index=task_index, task=prompt)) + '\n')
        if depth_meta or depth:
            try: dm = json.loads((meta / 'depth.json').read_text())
            except (FileNotFoundError, ValueError): dm = {}
            for cam, m in depth_meta.items():
                dm[cam] = dict(depth_scale_m=m.get('depth_scale_m'), intrinsics=m.get('intrinsics'))
            _write_text(meta / 'depth.json', json.dumps(dm, indent=1))
        episode = dict(episode_index=index, tasks=[prompt], length=T)
        for k, v in (extras or {}).items():
            if k not in episode: episode[k] = v
        _write_text(meta / 'episodes.jsonl', ''.join(_dumps(x) + '\n' for x in _jsonl(meta / 'episodes.jsonl')) + _dumps(episode) + '\n')
        total_episodes = index + 1
        new_info = dict(codebase_version=CODEBASE_VERSION, robot_type=robot_type, total_episodes=total_episodes,
                        total_frames=start + T, total_tasks=len(tasks) + (1 if new_task else 0),
                        total_videos=total_episodes * len(cameras), total_chunks=chunk + 1, chunks_size=CHUNKS_SIZE, fps=fps,
                        splits={'train': f'0:{total_episodes}'}, data_path=DATA_PATH, video_path=VIDEO_PATH if cameras else None,
                        features=all_feats, roborun=dict(block, depth_path=DEPTH_DIR + '/{frame_index}.png' if depth else None))
        _write_text(meta / 'info.json', json.dumps(new_info, indent=4))
    return index


REFUSED = '.refused.jsonl'


def refuse(root, why, plain=None, *, task_id=None, recorded_at=None):
    """Note that an episode was not added (<root>/.refused.jsonl, local only: the upload skips dot files). Never raises."""
    import time
    try:
        root = Path(root).expanduser(); root.mkdir(parents=True, exist_ok=True)
        line = dict(at=time.time(), task_id=task_id, recorded_at=recorded_at, why=str(why)[:500], plain=str(plain or why)[:300])
        with open(root / REFUSED, 'a') as f:
            f.write(json.dumps(line, default=str) + '\n')
    except Exception:
        pass


def refusals(root):
    """The refusals since the dataset's last added episode (meta/info.json is rewritten by every append): [lines]."""
    root = Path(root).expanduser()
    try:
        since = (root / 'meta' / 'info.json').stat().st_mtime
    except OSError:
        since = 0.
    return [r for r in _jsonl(root / REFUSED) if float(r.get('at') or 0) > since]


def refusal_summary(root):
    """{refused: N, message: '3 episodes not added to the dataset: the right wrist camera sent no frames'} for the refusals
    since the last episode that was added, or None. Read-only and cheap (one stat, one small file): `roborun status` prints
    the message, the robot's /v1/status carries it as `dataset`."""
    found = refusals(root)
    if not found:
        return None
    counts = {}
    for r in found:
        counts[r.get('plain') or r.get('why')] = counts.get(r.get('plain') or r.get('why'), 0) + 1
    n = len(found)
    reasons = '; '.join(f'{why} ({k})' if len(counts) > 1 else why for why, k in sorted(counts.items(), key=lambda x: -x[1])[:3])
    return dict(refused=n, message=f"{n} episode{'s' if n != 1 else ''} not added to the dataset: {reasons}")


def refusal_line(root):
    """refusal_summary's message, or None."""
    summary = refusal_summary(root)
    return summary['message'] if summary else None


def load_info(root):
    """meta/info.json of a dataset, or None."""
    return _committed(root)


# ------------------------------------------------------------------------------------------------ background writes
PENDING = '.pending'            # <root>/.pending/<id>/: an episode handed over but not yet appended (kept until it is)


def stage_episode(root, **kw):
    """Hand an episode over for a later append without encoding anything: rows to rows.npz, JPEG/PNG bytes to files, the rest
    to episode.json, under <root>/.pending/<time>-<pid>-<n>/ (renamed into place when complete). Cheap: no pyarrow, no PyAV,
    no pip. Returns the folder."""
    import numpy as np
    root = Path(root).expanduser(); base = root / PENDING; base.mkdir(parents=True, exist_ok=True)
    import itertools
    import time
    for n in itertools.count():
        final = base / f'{time.time_ns():020d}-{os.getpid()}-{n}'
        if not final.exists(): break
    tmp = final.with_name(final.name + '.tmp'); tmp.mkdir()
    try:
        rows = kw.pop('rows'); cameras = kw.pop('cameras', None) or {}; depth = kw.pop('depth', None) or {}
        np.savez(tmp / 'rows.npz', **{f'r{i}': np.asarray(v) for i, v in enumerate(rows.values())})
        index = dict(rows=list(rows), cameras={}, depth={})
        for i, (key, items) in enumerate(cameras.items()):
            (tmp / f'c{i}').mkdir(); index['cameras'][key] = [f'c{i}', [fi for fi, _ in items]]
            for fi, data in items: (tmp / f'c{i}' / f'{fi}.jpg').write_bytes(bytes(data))
        for i, (cam, items) in enumerate(depth.items()):
            (tmp / f'd{i}').mkdir(); index['depth'][cam] = [f'd{i}', [fi for fi, _ in items]]
            for fi, data in items: (tmp / f'd{i}' / f'{fi}.png').write_bytes(bytes(data))
        (tmp / 'episode.json').write_text(json.dumps(dict(kwargs=kw, index=index)))
        os.replace(tmp, final)
    except BaseException:
        shutil.rmtree(tmp, ignore_errors=True)
        raise
    return final


def _load_staged(folder):
    import numpy as np
    folder = Path(folder); spec = json.loads((folder / 'episode.json').read_text())
    kw = dict(spec['kwargs']); idx = spec['index']
    with np.load(folder / 'rows.npz', allow_pickle=False) as z:
        kw['rows'] = {name: z[f'r{i}'] for i, name in enumerate(idx['rows'])}
    kw['cameras'] = {key: [(fi, (folder / sub / f'{fi}.jpg').read_bytes()) for fi in fis] for key, (sub, fis) in idx['cameras'].items()}
    kw['depth'] = {cam: [(fi, (folder / sub / f'{fi}.png').read_bytes()) for fi in fis] for cam, (sub, fis) in idx['depth'].items()}
    return kw


def append_pending(root, log=None):
    """Append every staged episode of `root`, oldest first. A failure is logged, written to that folder's error.txt and the folder
    is kept for the next call; a success removes it. Returns [(folder name, episode index or None)]."""
    root = Path(root).expanduser(); base = root / PENDING; out = []
    log = log or (lambda msg: print(msg, flush=True))
    for folder in sorted(p for p in base.glob('*') if p.is_dir() and not p.name.endswith('.tmp')) if base.is_dir() else []:
        try:
            index = append_episode(root, **_load_staged(folder))
        except Exception as exc:
            msg = f'{type(exc).__name__}: {exc}'[:500]
            try: (folder / 'error.txt').write_text(msg + '\n')
            except OSError: pass
            log(f'[dataset] {folder.name} kept for a retry: {msg}'); out.append((folder.name, None)); continue
        shutil.rmtree(folder, ignore_errors=True)
        log(f'[dataset] {folder.name} -> episode {index}'); out.append((folder.name, index))
    return out


def lower_priority():
    """nice 19 and idle I/O for this process (Linux; best effort elsewhere)."""
    try: os.nice(19 - os.nice(0))
    except OSError: pass
    try:
        import sys
        if sys.platform.startswith('linux'):
            import ctypes
            nr = {'x86_64': 251, 'aarch64': 30}.get(os.uname().machine)
            if nr is not None: ctypes.CDLL(None, use_errno=True).syscall(nr, 1, 0, 3 << 13)     # IOPRIO_CLASS_IDLE
    except Exception:
        pass


def append_episode_background(root, **kw):
    """append_episode's arguments, staged at once (stage_episode) and appended by a detached child process at nice 19 with
    idle I/O. Never imports pyarrow/PyAV in the caller (a missing extra fails in the child and the staged episode is kept).
    Returns the staged folder; the child's log is <root>/.pending/writer.log."""
    import subprocess
    import sys
    folder = stage_episode(root, **kw)
    root = Path(root).expanduser()
    src = str(Path(__file__).resolve().parents[1])
    code = f'import sys; sys.path.insert(0, {src!r}) if {src!r} not in sys.path else None; from roborun_sdk.dataset import main; sys.exit(main())'
    with open(root / PENDING / 'writer.log', 'ab') as logf:
        subprocess.Popen([sys.executable, '-I', '-B', '-c', code, '--pending', str(root)], stdin=subprocess.DEVNULL,
                         stdout=logf, stderr=subprocess.STDOUT, close_fds=True, start_new_session=True)
    return folder


def main(argv=None):
    import sys
    argv = list(sys.argv[1:] if argv is None else argv)
    if len(argv) == 2 and argv[0] == '--pending':
        lower_priority()
        done = append_pending(argv[1])
        return 0 if all(i is not None for _, i in done) else 1
    print('usage: python -m roborun_sdk.dataset --pending <dataset root>', file=sys.stderr)
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
