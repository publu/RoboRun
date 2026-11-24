"""`roborun dreamlabs ...`: your Roborun episodes in the dataset format DreamLabs accepts, and (only when you ask) uploaded to
your own DreamLabs account to fine-tune a model you then run through Roborun.

    roborun dreamlabs export [--episodes all|last:N|<task ids or episode numbers>] [--out DIR] [--dataset ROOT]
    roborun dreamlabs upload DIR --name NAME [--yes]
    roborun dreamlabs finetune --dataset ds_... --suffix NAME [--steps N] [--yes]
    roborun dreamlabs status [JOB_OR_DATASET_ID]

export reads this robot's LeRobot dataset and writes, under DIR (default <Roborun home>/dreamlabs/exports/<time>/):

    DIR/dataset/<session>/    one LeRobot v2.1 dataset: `observation.state` and `action` only (the joints in radians, each
                              gripper as an open fraction 0 closed .. 1 open), and every camera video face-blurred
    DIR/embodiment.json       the embodiment config DreamLabs reads (parts and camera roles), plus the arms it came from
    DIR/validation.json       what DreamLabs' own offline check (dream.datasets validate_dataset) said

Nothing is sent by export. A video that cannot be blurred is never copied: its episode is left out and listed. upload sends
DIR/dataset to the DreamLabs data service named by your DreamLabs credentials, after showing what and where and asking
(`--yes` skips the question); finetune queues a job on an uploaded dataset in your account; status shows a job or a dataset,
and once a fine-tune is ready it remembers which arms it was trained on, so
`roborun policy run --provider dreamlabs --model dream-1-ft:...` runs it. The uploads and jobs are listed in
<Roborun home>/dreamlabs-uploads.jsonl (this computer only).

Needs pip install 'roborun-sdk[dreamlabs-data]' (DreamLabs' client with its dataset tools, and OpenCV for the blur).
"""
from __future__ import annotations

import datetime
import hashlib
import json
import os
import re
import shutil
import sys
from pathlib import Path

from .dreamlabs import DATASET_KEY, VIDEO_ROLE, conventions, known_models, remember_model

EXTRA = 'dreamlabs-data'
UPLOADS = 'dreamlabs-uploads.jsonl'
MIN_SECONDS = 1.6               # DreamLabs trains on 48 rows at 30 fps: a shorter episode trains on nothing
DEFAULT_DATA = 'data.dreamlabs.ai'
ROLE_OF_KEY = {v: VIDEO_ROLE[k] for k, v in DATASET_KEY.items()}
JOINT = re.compile(r'(left|right)_joint(\d+)$')


class ExportError(Exception):
    pass


def _home():
    from .. import net
    return net.roborun_home()


def _log_env():
    if not (os.environ.get('DREAM_LOG') or os.environ.get('DREAM_LOG_DIR')):
        os.environ['DREAM_LOG_DIR'] = str(_home() / 'dreamlabs-log')


def _dream():
    _log_env()
    from .._optional import require
    return require('dream', 'dreamlabs')


def _jaw_range(model):
    """The station's jaw units (closed, open) of an arm model, for episodes that do not carry their own."""
    if model == 'metal_arm':
        from roborun_station.core import metal_spec
        s = metal_spec()
        return (s.jaw_closed, s.jaw_open)
    if model in ('yam', 'maker_arm'):
        return (0.0, 1.0)
    return None


def _robot_config():
    from .. import robot
    try:
        return json.loads(robot.CONFIG.read_text())
    except (OSError, ValueError):
        return {}


def _jsonl(path):
    try:
        return [json.loads(l) for l in Path(path).read_text().splitlines() if l.strip()]
    except FileNotFoundError:
        return []


def _parts(names):
    """{part: [start, end]} from the dataset's state names (left_joint1.., left_gripper, right_..), contiguous, in order."""
    parts, order = {}, []
    for i, name in enumerate(names):
        m = JOINT.fullmatch(name)
        part = f'{m.group(1)}_arm' if m else (name if name in ('left_gripper', 'right_gripper') else None)
        if part is None:
            raise ExportError(f'the dataset has a state value Roborun cannot map for DreamLabs: {name}')
        if part not in parts:
            parts[part] = [i, i + 1]
            order.append(part)
        elif parts[part][1] == i:
            parts[part][1] = i + 1
        else:
            raise ExportError(f'the dataset\'s {part} values are not side by side')
    return {p: parts[p] for p in order}


def _select(episodes, which):
    if which in (None, '', 'all'):
        return list(episodes)
    if str(which).startswith('last:'):
        n = int(str(which).split(':', 1)[1])
        return list(episodes)[-n:] if n > 0 else []
    wanted = [w.strip() for w in str(which).split(',') if w.strip()]
    out = []
    for ep in episodes:
        if str(ep.get('task_id')) in wanted or str(ep['episode_index']) in wanted:
            out.append(ep)
    missing = [w for w in wanted if not any(w in (str(e.get('task_id')), str(e['episode_index'])) for e in out)]
    if missing:
        raise ExportError(f'no episode for {", ".join(missing)} in this dataset')
    return out


def _session_name(robot):
    name = re.sub(r'[^A-Za-z0-9._-]+', '-', f'roborun-{robot or "robot"}').strip('-.')[:64]
    return name or 'roborun'


def export(*, episodes='all', out=None, dataset_root=None, robot=None, profile=None, say=print):
    """Write the export (see the module docstring). Returns a dict: out, dataset, episodes, withheld, validation."""
    made = []
    try:
        return _export(episodes=episodes, out=out, dataset_root=dataset_root, robot=robot, profile=profile, say=say,
                       made=made)
    except BaseException:
        for path in made:                       # a half-written export is never left behind
            shutil.rmtree(path, ignore_errors=True)
        raise


def _export(*, episodes, out, dataset_root, robot, profile, say, made):
    from .._optional import require
    np = require('numpy', 'dreamlabs-data')         # first: a base install names the dreamlabs-data extra, not [robot]
    pa = require('pyarrow', 'dreamlabs-data')
    pq = require('pyarrow.parquet', 'dreamlabs-data')
    require('cv2', 'dreamlabs-data')
    from .. import camera_privacy, dataset as ds
    from .station import arm_models
    config = _robot_config()
    robot = robot or config.get('robot') or 'robot'
    if dataset_root is None:
        from ..robot import dataset_root as root_of
        dataset_root = root_of(robot)
    root = Path(dataset_root).expanduser()
    info = ds.load_info(root)
    if not info:
        raise ExportError(f'there is no dataset at {root} yet (each finished task or policy run adds an episode)')
    profile = profile or (info.get('roborun') or {}).get('profile') or config.get('profile') or ''
    models = arm_models({'profile': profile})
    feats = info['features']
    names = (feats.get('observation.state') or {}).get('names') or []
    parts = _parts(names)
    sides = [s for s in ('left', 'right') if f'{s}_arm' in parts]
    fps = info['fps']
    cams = sorted(k for k, v in feats.items() if v.get('dtype') == 'video')
    roles = {}
    for key in cams:
        role = ROLE_OF_KEY.get(key) or re.sub(r'[^a-z0-9_]', '_', key[len('observation.images.'):].lower()) or 'camera'
        roles[key] = role if re.fullmatch(r'[a-z][a-z0-9_]*', role) else f'cam_{role}'
    picked = _select(_jsonl(root / 'meta' / 'episodes.jsonl'), episodes)
    if not picked:
        raise ExportError('no episode to export')
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%d-%H%M%S')
    out = Path(out).expanduser() if out else _home() / 'dreamlabs' / 'exports' / stamp
    session = out / 'dataset' / _session_name(robot)
    if out.exists() and any(out.iterdir()):
        raise ExportError(f'{out} is not empty; name another --out')
    made.append(out / 'dataset' if out.exists() else out)
    session.mkdir(parents=True)
    tasks = {}
    stats_src = {int(s['episode_index']): s.get('stats') or {} for s in _jsonl(root / 'meta' / 'episodes_stats.jsonl')}
    conv = conventions()
    written, withheld = [], []
    total = 0
    scratch = out / '.scratch'
    scratch.mkdir()
    for ep in picked:
        idx = int(ep['episode_index'])
        chunk = idx // int(info.get('chunks_size') or 1000)
        length = int(ep.get('length') or 0)
        if length < int(np.ceil(MIN_SECONDS * fps)) + 1:
            withheld.append(dict(episode=idx, task_id=ep.get('task_id'), why=f'shorter than {MIN_SECONDS} s'))
            continue
        jaw = {}
        for side in sides:
            given = (ep.get('jaw_range') or {}).get(side) if ep.get('gripper_units') == 'station' else None
            jaw[side] = tuple(given) if given else _jaw_range((models.get(side) or (None,))[0])
            if f'{side}_gripper' in parts and not jaw[side]:
                raise ExportError(f'the {side} gripper\'s units are unknown for this arm model; nothing was written')
        new = len(written)
        fmt = dict(episode_chunk=new // 1000, episode_index=new)
        videos, ok = {}, True
        for key in cams:
            src = root / info['video_path'].format(video_key=key, episode_chunk=chunk, episode_index=idx)
            tmp = scratch / f'{new}-{roles[key]}.mp4'
            try:
                clear = camera_privacy.clear_video(src, tmp)
            except camera_privacy.BlurUnavailable as exc:
                withheld.append(dict(episode=idx, task_id=ep.get('task_id'), why=f'{key}: {exc}'))
                ok = False
                break
            videos[key] = clear
        if not ok:
            continue
        table = pq.read_table(root / info['data_path'].format(episode_chunk=chunk, episode_index=idx))
        state = np.array(table.column('observation.state').to_pylist(), dtype=np.float64)
        action = np.array(table.column('action').to_pylist(), dtype=np.float64)
        for a in (state, action):
            for side in sides:
                lo, hi = parts[f'{side}_arm']
                m = (conv.get((models.get(side) or (None,))[0]) or {}).get('mapping')
                if m:
                    a[:, lo:hi] = a[:, lo:hi][:, m['joint_order']] * np.array(m['signs']) + np.array(m['offsets_rad'])
                if f'{side}_gripper' in parts:
                    g = parts[f'{side}_gripper'][0]
                    c, o = jaw[side]
                    a[:, g] = np.clip((a[:, g] - c) / (o - c), 0.0, 1.0)
        prompt = (ep.get('tasks') or ['task'])[0]
        task_index = tasks.setdefault(prompt, len(tasks))
        T = len(state)
        cols = {'observation.state': state.astype(np.float32), 'action': action.astype(np.float32)}
        own = dict(timestamp=np.asarray(table.column('timestamp').to_pylist(), np.float32),
                   frame_index=np.arange(T, dtype=np.int64), episode_index=np.full(T, new, np.int64),
                   index=np.arange(T, dtype=np.int64) + total, task_index=np.full(T, task_index, np.int64))
        arrays, fields = [], []
        for name, a in cols.items():
            arr = pa.FixedSizeListArray.from_arrays(pa.array(a.reshape(-1), type=pa.float32()), a.shape[1])
            arrays.append(arr); fields.append(pa.field(name, arr.type))
        for name, a in own.items():
            arr = pa.array(a); arrays.append(arr); fields.append(pa.field(name, arr.type))
        path = session / ds.DATA_PATH.format(**fmt)
        path.parent.mkdir(parents=True, exist_ok=True)
        pq.write_table(pa.Table.from_arrays(arrays, schema=pa.schema(fields)), path)
        for key, clear in videos.items():
            dst = session / ds.VIDEO_PATH.format(video_key=key, **fmt)
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(clear, dst)
        stats = {k: ds._stats(v) for k, v in cols.items()}
        stats.update({k: ds._stats(v[:, None]) for k, v in own.items()})
        stats.update({k: v for k, v in stats_src.get(idx, {}).items() if k in videos})
        written.append(dict(episode_index=new, tasks=[prompt], length=T, stats=stats, source_episode=idx,
                            task_id=ep.get('task_id')))
        total += T
    shutil.rmtree(scratch, ignore_errors=True)
    if not written:
        shutil.rmtree(out / 'dataset', ignore_errors=True)
        raise ExportError('no episode could be exported: ' + '; '.join(f"episode {w['episode']}: {w['why']}" for w in withheld))
    vec = lambda key: dict(dtype='float32', shape=[len(names)], names=list(names))
    features = {k: feats[k] for k in cams}
    features.update({'observation.state': vec('observation.state'), 'action': vec('action')})
    features.update(timestamp=dict(dtype='float32', shape=[1], names=None), frame_index=dict(dtype='int64', shape=[1], names=None),
                    episode_index=dict(dtype='int64', shape=[1], names=None), index=dict(dtype='int64', shape=[1], names=None),
                    task_index=dict(dtype='int64', shape=[1], names=None))
    n = len(written)
    meta = session / 'meta'
    meta.mkdir(parents=True, exist_ok=True)
    (meta / 'info.json').write_text(json.dumps(dict(
        codebase_version='v2.1', robot_type=info.get('robot_type') or profile or 'roborun', total_episodes=n, total_frames=total,
        total_tasks=len(tasks), total_videos=n * len(cams), total_chunks=(n - 1) // 1000 + 1, chunks_size=1000, fps=fps,
        splits={'train': f'0:{n}'}, data_path=ds.DATA_PATH, video_path=ds.VIDEO_PATH if cams else None, features=features),
        indent=4))
    (meta / 'episodes.jsonl').write_text(''.join(json.dumps(dict(episode_index=w['episode_index'], tasks=w['tasks'],
                                                                 length=w['length'])) + '\n' for w in written))
    (meta / 'tasks.jsonl').write_text(''.join(json.dumps(dict(task_index=i, task=t)) + '\n' for t, i in tasks.items()))
    (meta / 'episodes_stats.jsonl').write_text(''.join(json.dumps(dict(episode_index=w['episode_index'], stats=w['stats']))
                                                       + '\n' for w in written))
    body = dict(state=dict(parts), action=dict(parts), video={roles[k]: k for k in cams})
    if cams and 'overhead' not in body['video']:
        body['head_camera'] = sorted(body['video'])[0]
    sidecar = dict(embodiment_config=body, roborun=dict(
        robot=robot, profile=profile, fps=fps, gripper_units='open fraction (0 closed .. 1 open)',
        arms={s: dict(model=(models.get(s) or ('unknown',))[0], joints=parts[f'{s}_arm'][1] - parts[f'{s}_arm'][0])
              for s in sides},
        episodes=[dict(source_episode=w['source_episode'], task_id=w['task_id']) for w in written],
        withheld=withheld, videos='face-blurred'))
    (out / 'embodiment.json').write_text(json.dumps(sidecar, indent=1))
    report = validate(out)
    say(f'exported {n} episode{"s" if n != 1 else ""} ({total} frames at {fps:g} fps) to {out}'
        + (f'; left out {len(withheld)}: ' + '; '.join(f"episode {w['episode']} ({w['why']})" for w in withheld)
           if withheld else ''))
    say(('DreamLabs\' check: ok' if report['ok'] else f"DreamLabs' check: not ok: {report['error']}")
        + ''.join(f'\n  note: {n_}' for n_ in report.get('notes') or ()))
    return dict(out=str(out), dataset=str(out / 'dataset'), episodes=n, frames=total, withheld=withheld, validation=report)


def validate(out):
    """dream.datasets validate_dataset over DIR/dataset (local only); the report is kept in DIR/validation.json."""
    dream = _dream()
    from dream import datasets
    out = Path(out)
    config = json.loads((out / 'embodiment.json').read_text())['embodiment_config']
    try:
        rep = datasets.validate_dataset(str(out / 'dataset'), config)
        report = {k: v for k, v in vars(rep).items()} if hasattr(rep, '__dict__') else dict(rep._asdict())
    except dream.DreamConfigError as exc:
        report = dict(ok=False, error=str(exc))
    report = json.loads(json.dumps(report, default=str))
    report['client'] = f"dreamlabs-client {getattr(dream, '__version__', '?')}"
    (out / 'validation.json').write_text(json.dumps(report, indent=1))
    return report


def _size(path):
    return sum(p.stat().st_size for p in Path(path).rglob('*') if p.is_file())


def data_host():
    url = str(os.environ.get('DREAM_DATA_URL') or '').strip()
    return url.split('://', 1)[-1].split('/', 1)[0] if url else f'{DEFAULT_DATA}, or the data service your DreamLabs credentials name'


def _log(entry):
    path = _home() / UPLOADS
    path.parent.mkdir(parents=True, exist_ok=True)
    entry = dict(at=datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'), **entry)
    with open(path, 'a') as f:
        f.write(json.dumps(entry, default=str) + '\n')
    return entry


def uploads():
    return _jsonl(_home() / UPLOADS)


def _embodiment_name(config):
    digest = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()[:10]
    return f'roborun-{digest}'


def upload(out, name, *, yes=False, ask=None, say=print):
    """Send an export's dataset to your DreamLabs account (after the question). Returns the dataset as DreamLabs answered."""
    out = Path(out).expanduser()
    try:
        side = json.loads((out / 'embodiment.json').read_text())
        report = json.loads((out / 'validation.json').read_text())
    except (OSError, ValueError):
        raise ExportError(f'{out} is not a Roborun export for DreamLabs (roborun dreamlabs export writes one)') from None
    if not report.get('ok'):
        raise ExportError(f"DreamLabs' check refused this export: {report.get('error')}; nothing was sent")
    if not re.fullmatch(r'[a-z0-9-]{1,64}', name or ''):
        raise ExportError("--name uses a-z, 0-9 and '-' only, 1 to 64 characters, e.g. towels-v1")
    data = out / 'dataset'
    config = side['embodiment_config']
    own = side.get('roborun') or {}
    eps = report.get('episodes') or len(own.get('episodes') or [])
    say(f'This sends {data} to DreamLabs ({data_host()}), into your DreamLabs account, as dataset {name!r}:')
    say(f'  {eps} episodes, {_size(data) / 1e6:.1f} MB, cameras {", ".join(sorted(config.get("video") or {})) or "none"} '
        f'(videos face-blurred), joints and grippers of the {" and ".join((own.get("arms") or {}).keys())} arm(s), prompts.')
    say('  Your DreamLabs key is sent as DreamLabs\' client sends it. Nothing else of this computer is sent.')
    if not yes:
        if ask is None:
            raise ExportError('upload needs your yes: run it in a terminal, or add --yes; nothing was sent')
        if str(ask('Upload this to your DreamLabs account? [y/N] ') or '').strip().lower() not in ('y', 'yes'):
            raise ExportError('not agreed: nothing was sent')
    dream = _dream()
    client = dream.DreamClient()
    short = _embodiment_name(config)
    found = next((e for e in client.get_embodiments() if e.name.rsplit('/', 1)[-1] == short), None)
    emb = found or client.add_embodiment(short, config)
    with _warnings_to(say):
        got = client.upload_dataset(str(data), name, emb.name)
    try:
        got = client.get_dataset_status(got.id)
    except dream.DreamError:
        pass
    _log(dict(kind='upload', dataset_id=got.id, dataset_name=got.name, embodiment=emb.name, status=got.status,
              episodes=got.episodes, export=str(out), embodiment_config=config, roborun=own))
    say(f'uploaded: {got.id} ({got.name}), {got.status}. Check it with: roborun dreamlabs status {got.id}')
    return got


class _warnings_to:
    def __init__(self, say):
        self.say = say

    def __enter__(self):
        import warnings
        self._cm = warnings.catch_warnings(record=True)
        self.seen = self._cm.__enter__()
        warnings.simplefilter('always')
        return self

    def __exit__(self, *exc):
        self._cm.__exit__(*exc)
        for w in self.seen:
            self.say(f'DreamLabs: {w.message}')
        return False


def _upload_entry(dataset):
    for e in reversed(uploads()):
        if e.get('kind') == 'upload' and dataset in (e.get('dataset_id'), e.get('dataset_name')):
            return e
    return None


def finetune(dataset, suffix, *, steps=None, yes=False, ask=None, say=print):
    """Queue a fine-tuning job in your DreamLabs account on an uploaded dataset (by id or name)."""
    entry = _upload_entry(dataset)
    dream = _dream()
    client = dream.DreamClient()
    name = dataset
    if dataset.startswith('ds_'):
        name = client.get_dataset_status(dataset).name
    say(f'This asks DreamLabs to fine-tune Dream-1 on {name} in your DreamLabs account'
        + (f' ({steps} steps)' if steps else '') + '. DreamLabs\' own terms apply to it.')
    if not yes:
        if ask is None:
            raise ExportError('finetune needs your yes: run it in a terminal, or add --yes; nothing was sent')
        if str(ask('Queue this fine-tuning job? [y/N] ') or '').strip().lower() not in ('y', 'yes'):
            raise ExportError('not agreed: nothing was sent')
    job = client.finetune(name, suffix, steps=steps)
    _log(dict(kind='finetune', job_id=job.get('id'), dataset_name=name, status=job.get('status'),
              embodiment_config=(entry or {}).get('embodiment_config'), roborun=(entry or {}).get('roborun')))
    say(f"queued: {job.get('id')} ({job.get('status')}). Check it with: roborun dreamlabs status {job.get('id')}")
    return job


def status(what=None, *, say=print):
    """A job (ftj_...) or a dataset (ds_...) as DreamLabs holds it now; with nothing, the local list of uploads and jobs."""
    if not what:
        rows = uploads()
        if not rows:
            say('no uploads or fine-tuning jobs from this computer yet')
        for e in rows:
            say(f"{e['at']} {e['kind']}: {e.get('dataset_id') or e.get('job_id')} {e.get('dataset_name') or ''} "
                f"{e.get('status') or ''}".rstrip())
        return rows
    dream = _dream()
    client = dream.DreamClient()
    if what.startswith('ftj_'):
        job = client.get_finetuning_status(what)
        model = job.get('fine_tuned_model')
        say(f"{job.get('id')}: {job.get('status')}" + (f", model {model}" if model else '')
            + (f", queue position {job['queue_position']}" if job.get('queue_position') else '')
            + (f", error: {job['error']}" if job.get('error') else ''))
        if model:
            entry = next((e for e in reversed(uploads()) if e.get('job_id') == what and e.get('embodiment_config')), None) \
                or _upload_entry(job.get('dataset') or '')
            if entry and model not in known_models():
                remember_model(model, dict(embodiment_config=entry['embodiment_config'], roborun=entry.get('roborun'),
                                           dataset=job.get('dataset'), job=what))
            if model in known_models():
                say(f'run it: roborun policy run --provider dreamlabs --model {model} --prompt "..."')
        return job
    got = client.get_dataset_status(what)
    say(f'{got.id} ({got.name}): {got.status}' + (f', error: {got.error}' if got.error else ''))
    return got


# ---- the command -------------------------------------------------------------------------------------------------------
def _fail(exc):
    print(f'refused: {exc}', file=sys.stderr)
    return 2


def _ask():
    from .consent import terminal_ask
    return terminal_ask()


def cmd(a):
    from .._optional import MissingExtra
    try:
        if a.dreamlabs_cmd == 'export':
            got = export(episodes=a.episodes, out=a.out, dataset_root=a.dataset)
            return 0 if got['validation'].get('ok') else 1
        if a.dreamlabs_cmd == 'upload':
            upload(a.dir, a.name, yes=a.yes, ask=_ask())
            return 0
        if a.dreamlabs_cmd == 'finetune':
            finetune(a.dataset, a.suffix, steps=a.steps, yes=a.yes, ask=_ask())
            return 0
        status(a.id)
        return 0
    except MissingExtra as exc:
        return _fail(exc if f'[{EXTRA}]' in str(exc) else f"{exc} (this needs pip install 'roborun-sdk[{EXTRA}]')")
    except ExportError as exc:
        return _fail(exc)
    except Exception as exc:
        name = type(exc).__name__
        if name.startswith('Dream'):
            print(f'DreamLabs: {exc}', file=sys.stderr)
            return 1
        raise


def add_parser(sub):
    p = sub.add_parser('dreamlabs', help='DreamLabs: export your episodes in their dataset format; upload and fine-tune in '
                                         'your own DreamLabs account (only when you ask)')
    ps = p.add_subparsers(dest='dreamlabs_cmd', metavar='export|upload|finetune|status', required=True)
    s = ps.add_parser('export', help='write episodes of this robot\'s dataset in the format DreamLabs accepts (sends nothing)')
    s.add_argument('--episodes', default='all', help='all, last:N, or task ids / episode numbers, comma-separated')
    s.add_argument('--out', help='an empty folder (default: <Roborun home>/dreamlabs/exports/<time>)')
    s.add_argument('--dataset', help='a dataset folder (default: this robot\'s)')
    s = ps.add_parser('upload', help='send an export to your DreamLabs account (asks first)')
    s.add_argument('dir', help='the export folder')
    s.add_argument('--name', required=True, help="the dataset's name in your DreamLabs account (a-z, 0-9, -)")
    s.add_argument('--yes', action='store_true', help='do not ask')
    s = ps.add_parser('finetune', help='queue a fine-tuning job in your DreamLabs account on an uploaded dataset')
    s.add_argument('--dataset', required=True, help='ds_... or the dataset\'s name')
    s.add_argument('--suffix', required=True, help="the end of the new model's id (a-z, 0-9, -)")
    s.add_argument('--steps', type=int, help='training steps (DreamLabs\' default when left out)')
    s.add_argument('--yes', action='store_true', help='do not ask')
    s = ps.add_parser('status', help='a job or a dataset as DreamLabs holds it; with nothing, this computer\'s uploads')
    s.add_argument('id', nargs='?')
    p.set_defaults(func=cmd)
    for name in ('export', 'upload', 'finetune', 'status'):
        ps.choices[name].set_defaults(func=cmd)
