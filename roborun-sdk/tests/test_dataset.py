"""roborun_sdk.dataset: the robot's LeRobot v2.1 dataset writer, on frames and joints of a REAL YAM recording (read only;
copied nowhere). Set ROBORUN_TEST_RECORDING to a recording folder (recording-<task>-NN, with frames.jsonl, joints.npz and
the camera files); the tests that need it are skipped without it."""
import io
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from roborun_sdk import dataset

REAL = Path(os.environ.get('ROBORUN_TEST_RECORDING') or '/nonexistent/recording')    # a real recording, when given
FACE = Path(__file__).parent / 'data' / 'astronaut_face.jpg'
SRC = Path(__file__).resolve().parents[1] / 'src'

needs_real = pytest.mark.skipif(not (REAL / 'frames.jsonl').is_file(), reason='ROBORUN_TEST_RECORDING is not set to a real recording')
needs_tools = pytest.mark.skipif(dataset.unavailable() is not None, reason='pyarrow / PyAV are not installed')


def real_episode(cams=('top', 'left'), prompt='put the orange cup inside the green cup'):
    """append_episode arguments from the real recording: per frame of the top camera, its JPEG, the left camera's JPEG
    nearest in time, the real depth PNGs, and both arms' measured / commanded joints nearest in time (no made-up values)."""
    frames = [json.loads(line) for line in (REAL / 'frames.jsonl').read_text().splitlines()]
    by = {c: sorted((f for f in frames if f['cam'] == c), key=lambda f: f['acq']) for c in cams}
    base = by[cams[0]]
    with np.load(REAL / 'joints.npz') as z:
        rows, columns = z['rows'], [str(c) for c in z['columns']]
    t = np.where(np.isfinite(rows[:, 0]), rows[:, 0], rows[:, 1])
    names = [f'{s}_joint{i}' for s in ('left', 'right') for i in range(1, 7)]
    q = [columns.index(f'{s}_q{i}') for s in ('left', 'right') for i in range(1, 7)]
    c = [columns.index(f'{s}_c{i}') for s in ('left', 'right') for i in range(1, 7)]
    state, action = [], []
    for f in base:
        k = int(np.argmin(np.abs(t - f['acq'])))
        state.append(rows[k, q]); action.append(rows[k, c])

    def blob(cam, kind, span):
        with open(REAL / f'{cam}.{kind}', 'rb') as fh:
            fh.seek(span[0]); return fh.read(span[1])
    cameras, depth = {}, {}
    for cam in cams:
        pick = [min(by[cam], key=lambda e: abs(e['acq'] - f['acq'])) for f in base]
        cameras['observation.images.' + cam] = [(i, blob(cam, 'mjpg', e['jpg'])) for i, e in enumerate(pick)]
        depth[cam] = [(i, blob(cam, 'depth', e['png'])) for i, e in enumerate(pick) if e.get('png')]
    meta = {json.loads(line)['cam']: json.loads(line) for line in (REAL / 'cameras.jsonl').read_text().splitlines()}
    return dict(robot_type='yam', robot='lab-yam', profile='yam', fps=9,
                features={'observation.state': dict(names=names, units='rad', dtype='float32'),
                          'action': dict(names=names, units=['rad'] * len(names), dtype='float32')},
                rows={'observation.state': np.array(state), 'action': np.array(action)}, cameras=cameras, depth=depth,
                depth_meta={cam: dict(depth_scale_m=meta[cam]['depth_scale_m'], intrinsics=meta[cam]['intrinsics']) for cam in cams},
                prompt=prompt, extras=dict(task_id='task0001', mode='run', outcome='done', source=str(REAL)))


def test_imports_without_the_extra_and_says_why():
    code = ("import sys\n"                                     # -S: no site-packages at all, so neither pyarrow nor av
            f"sys.path.insert(0, {str(SRC)!r})\n"
            "from roborun_sdk import dataset\n"
            "print(dataset.unavailable())\n"
            "try:\n"
            "    dataset.append_episode('x', robot_type='a', robot='b', profile='c', fps=9, features={}, rows={}, prompt='p')\n"
            "except dataset.DatasetToolsMissing as exc:\n"
            "    print('missing', isinstance(exc, ImportError), exc.missing)\n")
    out = subprocess.run([sys.executable, '-I', '-S', '-c', code], capture_output=True, text=True, timeout=60, env={'ROBORUN_HOME': '/nonexistent'})
    assert out.returncode == 0, out.stderr
    lines = out.stdout.splitlines()
    assert lines[0] == 'the recording tools are not installed: run roborun up'
    assert lines[1].startswith('missing True ') and lines[1].split()[-1] in ('pyarrow', 'av')


def test_unavailable_reports_a_remembered_install_failure(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBORUN_HOME', str(tmp_path))
    monkeypatch.setattr(dataset, '_found', False)
    import importlib.util
    real = importlib.util.find_spec
    monkeypatch.setattr(importlib.util, 'find_spec', lambda name, *a: None if name in dataset.TOOLS else real(name, *a))
    assert dataset.unavailable() == 'the recording tools are not installed: run roborun up'
    from roborun_sdk.arms import packages

    def failing(extra, label=None):
        assert extra == 'dataset'
        raise SystemExit('could not install what recording needs on this computer (pyarrow>=17, av>=14):\nno wheel for this computer')
    monkeypatch.setattr(packages, 'ensure', failing)
    said = []
    got = dataset.setup_tools(log=said.append)
    assert said[0] == 'Setting up recording (about 290 MB, once)…' and len(said) == 2
    assert got.startswith('the recording tools could not be installed: could not install') and 'no wheel' in got
    assert dataset.unavailable() == got                                   # remembered: no retry per move
    monkeypatch.setattr(importlib.util, 'find_spec', real)
    if all(real(name) for name in dataset.TOOLS):                         # the tools are installed here: the failure is forgotten
        assert dataset.unavailable() is None and dataset.setup_tools(log=said.append) is None
        assert not (tmp_path / 'robot' / 'dataset-tools.json').exists() and len(said) == 2


@needs_real
@needs_tools
def test_two_real_episodes_make_one_lerobot_v21_dataset(tmp_path):
    import av
    import pyarrow.parquet as pq
    root = tmp_path / 'ds'
    assert dataset.append_episode(root, **real_episode()) == 0
    assert dataset.append_episode(root, **real_episode()) == 1                      # same prompt: one task
    assert dataset.append_episode(root, **real_episode(prompt='pick up the cup')) == 2
    info = json.loads((root / 'meta' / 'info.json').read_text())
    T = len(real_episode()['rows']['action'])
    assert info['codebase_version'] == 'v2.1' and info['robot_type'] == 'yam' and info['fps'] == 9
    assert (info['total_episodes'], info['total_frames'], info['total_tasks'], info['total_videos']) == (3, 3 * T, 2, 6)
    assert info['splits'] == {'train': '0:3'} and info['chunks_size'] == 1000
    assert info['roborun']['schema'] == 'roborun-dataset/1' and info['roborun']['robot'] == 'lab-yam'
    f = info['features']
    assert f['observation.images.top']['shape'] == [480, 640, 3] and f['observation.images.left']['shape'] == [270, 480, 3]
    assert f['observation.images.top']['info']['video.codec'] == 'h264'
    assert f['observation.state']['names'][0] == 'left_joint1' and f['observation.state']['units'] == 'rad'
    assert f['observation.depth_index.top'] == dict(dtype='int64', shape=[1], names=None, units='frame_index (-1 = no depth)')
    eps = [json.loads(x) for x in (root / 'meta' / 'episodes.jsonl').read_text().splitlines()]
    assert [e['tasks'] for e in eps] == [['put the orange cup inside the green cup']] * 2 + [['pick up the cup']]
    assert eps[0]['outcome'] == 'done' and eps[0]['length'] == T and eps[0]['source'] == str(REAL)
    tasks = [json.loads(x) for x in (root / 'meta' / 'tasks.jsonl').read_text().splitlines()]
    assert tasks == [dict(task_index=0, task='put the orange cup inside the green cup'), dict(task_index=1, task='pick up the cup')]
    stats = [json.loads(x) for x in (root / 'meta' / 'episodes_stats.jsonl').read_text().splitlines()]
    s = stats[2]['stats']
    assert len(s['observation.state']['mean']) == 12 and s['observation.images.top']['mean'][0] == [[pytest.approx(s['observation.images.top']['mean'][0][0][0])]]
    assert np.array(s['observation.images.top']['std']).shape == (3, 1, 1)
    table = pq.read_table(root / 'data' / 'chunk-000' / 'episode_000002.parquet')
    assert table.column('index').to_pylist() == list(range(2 * T, 3 * T)) and table.column('task_index').to_pylist() == [1] * T
    assert np.allclose(table.column('timestamp').to_numpy(), np.arange(T) / 9, atol=1e-6)
    with av.open(str(root / 'videos' / 'chunk-000' / 'observation.images.top' / 'episode_000001.mp4')) as box:
        st = box.streams.video[0]
        times = [float(fr.pts * st.time_base) for fr in box.decode(st)]
        assert st.codec_context.name == 'h264' and st.codec_context.pix_fmt == 'yuv420p'
    assert np.allclose(times, np.arange(T) / 9, atol=1e-4)
    di = table.column('observation.depth_index.top').to_pylist()
    for fi in (x for x in di if x >= 0):
        import cv2
        d = cv2.imread(str(root / 'depth' / 'chunk-000' / 'top' / 'episode_000002' / f'{fi}.png'), cv2.IMREAD_UNCHANGED)
        assert d.dtype == np.uint16 and d.shape == (480, 640)
    assert sum(x >= 0 for x in di) == len(real_episode()['depth']['top'])
    assert json.loads((root / 'meta' / 'depth.json').read_text())['top']['depth_scale_m'] == pytest.approx(0.001)


@needs_real
@needs_tools
def test_a_different_episode_is_refused_and_the_dataset_is_unchanged(tmp_path):
    root = tmp_path / 'ds'
    dataset.append_episode(root, **real_episode())
    before = {p: p.read_bytes() for p in root.rglob('*') if p.is_file() and not p.name.startswith('.')}
    with pytest.raises(dataset.DatasetError, match='cameras differ'):
        dataset.append_episode(root, **real_episode(cams=('top',)))                  # no left wrist camera
    with pytest.raises(dataset.DatasetError, match='fps'):
        dataset.append_episode(root, **dict(real_episode(), fps=10))
    with pytest.raises(dataset.DatasetError, match='belongs to robot'):
        dataset.append_episode(root, **dict(real_episode(), robot='another'))
    assert {p: p.read_bytes() for p in root.rglob('*') if p.is_file() and not p.name.startswith('.')} == before
    assert dataset.refusal_line(root) == ('3 episodes not added to the dataset: the left camera sent no frames (1); it was '
                                          'recorded at 10 frames a second, the dataset at 9 (1); it was recorded with another '
                                          'robot setup (yam) than the dataset\'s (yam) (1)')
    dataset.append_episode(root, **real_episode())                            # an episode added: the line is gone
    assert dataset.refusal_line(root) is None


@needs_real
@needs_tools
def test_columns_the_robot_never_reports_are_left_out(tmp_path):
    """The real gripper motor effort is reported, the joint efforts never are (NaN in every row): only the reported columns
    are written, a feature with none is left out, and later episodes follow the dataset's columns."""
    import pyarrow.parquet as pq
    with np.load(REAL / 'joints.npz') as z:
        rows, columns = z['rows'], [str(c) for c in z['columns']]
    ep = real_episode()
    T = len(ep['rows']['action'])
    g = rows[:, columns.index('left_motor_effort')]; g = g[np.isfinite(g)]
    grip = g[np.linspace(0, len(g) - 1, T).round().astype(int)]                 # real readings, spread over the frames
    names = [f'left_joint{i}_effort' for i in range(1, 7)] + ['left_gripper_effort']
    effort = np.full((T, 7), np.nan); effort[:, 6] = grip
    effort[:2, 6] = np.nan                                                      # partly silent: stays, NaN where silent
    ep['features']['observation.effort'] = dict(names=names, units='normalized', dtype='float32')
    ep['features']['observation.motor_temperature_c'] = dict(names=['left_joint1_temp_c'], units='C', dtype='float32')
    ep['rows']['observation.effort'] = effort
    ep['rows']['observation.motor_temperature_c'] = np.full((T, 1), np.nan)    # a robot reporting no temperature
    root = tmp_path / 'ds'
    dataset.append_episode(root, **ep)
    f = json.loads((root / 'meta' / 'info.json').read_text())['features']
    assert f['observation.effort']['names'] == ['left_gripper_effort'] and f['observation.effort']['shape'] == [1]
    assert 'observation.motor_temperature_c' not in f
    later = dict(ep, rows=dict(ep['rows'], **{'observation.effort': np.where(np.isnan(effort), 0.5, effort)}))
    assert dataset.append_episode(root, **later) == 1                          # a later episode with more: the dataset's columns
    t0 = pq.read_table(root / 'data' / 'chunk-000' / 'episode_000000.parquet').column('observation.effort').to_pylist()
    t1 = pq.read_table(root / 'data' / 'chunk-000' / 'episode_000001.parquet').column('observation.effort').to_pylist()
    assert all(isinstance(r, float) for r in t0 + t1)                          # one column: a scalar column, as LeRobot writes it
    assert np.isnan(t0[0]) and t0[5] == pytest.approx(grip[5]) and t1[0] == .5
    s = json.loads((root / 'meta' / 'episodes_stats.jsonl').read_text().splitlines()[0])['stats']['observation.effort']
    assert len(s['mean']) == 1 and np.isfinite(s['mean'][0])


@needs_real
@needs_tools
def test_a_crashed_append_is_never_listed_and_the_next_append_replaces_it(tmp_path):
    root = tmp_path / 'ds'
    dataset.append_episode(root, **real_episode())
    # a crash after the data and some meta of episode 1, before info.json (the commit)
    (root / 'data' / 'chunk-000' / 'episode_000001.parquet').write_bytes(b'half')
    (root / 'videos' / 'chunk-000' / 'observation.images.top' / 'episode_000001.mp4.tmp.mp4').write_bytes(b'half')
    with open(root / 'meta' / 'episodes.jsonl', 'a') as f:
        f.write(json.dumps(dict(episode_index=1, tasks=['ghost'], length=3)) + '\n{"cut off')
    assert dataset.append_episode(root, **real_episode(prompt='pick up the cup')) == 1
    eps = [json.loads(x) for x in (root / 'meta' / 'episodes.jsonl').read_text().splitlines()]
    assert [e['tasks'] for e in eps] == [['put the orange cup inside the green cup'], ['pick up the cup']]
    assert not list(root.rglob('*.tmp*'))
    assert (root / 'data' / 'chunk-000' / 'episode_000001.parquet').read_bytes()[:4] == b'PAR1'


@needs_real
@needs_tools
def test_staged_episodes_are_appended_later_and_kept_on_failure(tmp_path, monkeypatch):
    root = tmp_path / 'ds'
    dataset.stage_episode(root, **real_episode())
    monkeypatch.setattr(dataset, '_imports_ok', lambda: 'av')                    # the tools went missing
    log = []
    assert [i for _, i in dataset.append_pending(root, log.append)] == [None]
    kept = [p for p in (root / dataset.PENDING).iterdir() if p.is_dir()]
    assert len(kept) == 1 and 'DatasetToolsMissing' in (kept[0] / 'error.txt').read_text() and 'kept for a retry' in log[0]
    monkeypatch.undo()
    out = dataset.append_pending(root, log.append)
    assert [i for _, i in out] == [0] and not [p for p in (root / dataset.PENDING).iterdir() if p.is_dir()]
    assert json.loads((root / 'meta' / 'info.json').read_text())['total_episodes'] == 1


# ------------------------------------------------------------------------------------------------ the upload's face blur
@needs_real
@needs_tools
def test_dataset_video_and_depth_keep_their_format_through_the_face_blur(tmp_path):
    """A real face (the test photo) in the top camera: the uploaded video is H.264 again, and the depth PNGs stay 16-bit single
    channel, on the same scale, with no reading where the face was."""
    import av
    import cv2
    from roborun_sdk import camera_privacy, datasync
    ep = real_episode(cams=('top',))
    face = cv2.imread(str(FACE)); face = cv2.resize(face, (640, 480))
    ok, jpg = cv2.imencode('.jpg', face)
    ep['cameras']['observation.images.top'] = [(i, jpg.tobytes()) for i, _ in ep['cameras']['observation.images.top']]
    root = tmp_path / 'ds'
    dataset.append_episode(root, **ep)
    video = root / 'videos' / 'chunk-000' / 'observation.images.top' / 'episode_000000.mp4'
    pngs = sorted((root / 'depth' / 'chunk-000' / 'top' / 'episode_000000').glob('*.png'))
    assert pngs
    sync = datasync.DataSync('http://127.0.0.1:9', 'k', 'lab-yam', {'datasets': root})
    try:
        sent = sync._outgoing(video, video.stat())
        with av.open(io.BytesIO(bytes(sent))) as box:
            st = box.streams.video[0]
            assert st.codec_context.name == 'h264'
            times = [float(fr.pts * st.time_base) for fr in box.decode(st)]
        assert np.allclose(times, np.arange(len(ep['rows']['action'])) / 9, atol=1e-4)
        hits, shape, _ = camera_privacy.video_hits(video)
        assert all(len(h) for h in hits)                                       # the face is found in every frame
        for png in pngs:
            out = sync._outgoing(png, png.stat())
            assert isinstance(out, camera_privacy.Cleared)
            before = cv2.imread(str(png), cv2.IMREAD_UNCHANGED)
            after = cv2.imdecode(np.frombuffer(bytes(out), np.uint8), cv2.IMREAD_UNCHANGED)
            assert after.dtype == np.uint16 and after.shape == before.shape == (480, 640)
            hidden = after != before
            assert hidden.any() and (after[hidden] == 0).all()                  # no reading inside the face, same scale elsewhere
            x0, y0, x1, y1 = camera_privacy.covered_region(hits[int(png.stem)][0], shape)
            assert (after[y0:y1, x0:x1] == 0).all()
    finally:
        import shutil
        if sync.scratch is not None:
            shutil.rmtree(sync.scratch, ignore_errors=True)


@needs_real
def test_a_dataset_depth_png_without_its_video_is_withheld(tmp_path):
    from roborun_sdk import camera_privacy, datasync
    root = tmp_path / 'ds'; (root / 'meta').mkdir(parents=True); (root / 'meta' / 'info.json').write_text('{}')
    png = root / 'depth' / 'chunk-000' / 'top' / 'episode_000000' / '0.png'; png.parent.mkdir(parents=True)
    e = next(json.loads(x) for x in (REAL / 'frames.jsonl').read_text().splitlines() if json.loads(x).get('png'))
    with open(REAL / f"{e['cam']}.depth", 'rb') as f:
        f.seek(e['png'][0]); png.write_bytes(f.read(e['png'][1]))
    assert datasync.dataset_depth(png) == (root / 'videos' / 'chunk-000' / 'observation.images.top' / 'episode_000000.mp4', 0)
    sync = datasync.DataSync('http://127.0.0.1:9', 'k', 'lab-yam', {'datasets': root})
    with pytest.raises(camera_privacy.BlurUnavailable, match='video of this depth frame is missing'):
        sync._outgoing(png, png.stat())


def test_roborun_up_installs_the_recording_tools_once_unless_turned_off_or_busy(tmp_path, monkeypatch):
    import types
    from roborun_sdk import robot
    monkeypatch.setattr(robot, 'CONFIG', tmp_path / 'robot.json')
    calls = []
    monkeypatch.setattr(dataset, 'unavailable', lambda: 'the recording tools are not installed: run roborun up')
    monkeypatch.setattr(dataset, 'setup_tools', lambda log=print: calls.append('install') or None)
    busy = [False]
    monkeypatch.setattr(robot, '_busy', lambda backend: busy[0])
    a = lambda **kw: types.SimpleNamespace(**dict(dict(no_dataset=False, dataset=False), **kw))
    c = {}
    (tmp_path / 'robot.json').write_text('{}')
    assert robot._recording_tools(a(no_dataset=True), c) == 'off' and not calls
    assert json.loads((tmp_path / 'robot.json').read_text())['recording'] == {'dataset': False}       # remembered
    assert robot._recording_tools(a(), c) == 'off' and not calls
    assert robot._recording_tools(a(dataset=True), c) is None and calls == ['install']
    assert robot._recording_tools(a(), dict(c, data=dict(opt_out=True))) == 'off' and calls == ['install']   # --no-data-sync
    busy[0] = True
    assert robot._recording_tools(a(), c) == 'busy' and calls == ['install']                       # never during a task
