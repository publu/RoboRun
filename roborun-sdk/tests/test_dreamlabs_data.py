"""Roborun episodes in the dataset format DreamLabs accepts, checked offline by DreamLabs' own validator
(dream.datasets validate_dataset), and the opt-in upload / fine-tune against the local fake of DreamLabs' data service
(fake_dreamlabs.py, 127.0.0.1 only). Nothing reaches DreamLabs."""
import io
import json
import zipfile
from pathlib import Path

import pytest

pytest.importorskip('dream')
pytest.importorskip('boto3')
pq = pytest.importorskip('pyarrow.parquet')
cv2 = pytest.importorskip('cv2')
np = pytest.importorskip('numpy')

from fake_dreamlabs import FakeDreamLabs  # noqa: E402
from fake_policy_bits import own_home  # noqa: E402

from roborun_sdk import camera_privacy, dataset  # noqa: E402
from roborun_sdk.policy import dreamlabs as dl, dreamlabs_data as dd  # noqa: E402

HERE = Path(__file__).resolve().parent
PHOTO = cv2.imread(str(HERE / 'data' / 'astronaut_face.jpg'))            # NASA's public-domain portrait (BGR)
METAL = (0.1009054704, 1.9413522545)
NAMES = [f'{s}_joint{i}' for s in ('left',) for i in range(1, 7)] + ['left_gripper'] + \
        [f'right_joint{i}' for i in range(1, 7)] + ['right_gripper']


@pytest.fixture
def home(tmp_path, monkeypatch):
    return own_home(tmp_path, monkeypatch)              # never this computer's robot.json, runtime or $HOME


@pytest.fixture
def fake(home, monkeypatch):
    f = FakeDreamLabs()
    for k, v in f.env(home).items():
        monkeypatch.setenv(k, v)
    for k in ('AWS_PROFILE', 'AWS_CONFIG_FILE', 'AWS_SHARED_CREDENTIALS_FILE'):
        monkeypatch.delenv(k, raising=False)
    monkeypatch.setenv('AWS_EC2_METADATA_DISABLED', 'true')
    yield f
    f.close()


def frame(face, i):
    img = np.full((176, 320, 3), 80, np.uint8)
    img[:, :, 1] = (np.arange(320)[None, :] + i * 3) % 255
    if face:
        img[8:168, 80:240] = cv2.resize(PHOTO, (160, 160))
    return cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tobytes()


def episode(root, *, profile='yam', prompt='fold the towel', extras=None, T=30, face=False, right_jaw=None):
    """An episode as Roborun's recorders write it: the runtime export's features, three cameras, at 10 fps."""
    t = np.arange(T) / 10.0
    q = np.zeros((T, 14), np.float32)
    q[:, :6] = [0.0, 0.5, 0.8, 0.0, 0.3, 0.0]
    q[:, 5] += 0.1 * np.sin(t)
    q[:, 6] = 0.25
    q[:, 7:13] = [0.1, 0.6, 0.7, 0.0, 0.2, 0.0]
    q[:, 13] = 0.75 if right_jaw is None else right_jaw
    units = ['rad'] * 6 + ['0..1'] + ['rad'] * 6 + ['0..1']
    features = {'observation.state': dict(names=NAMES, units=units, dtype='float32'),
                'action': dict(names=NAMES, units=units, dtype='float32'),
                'observation.gripper': dict(names=['left_gripper', 'right_gripper'], units='0..1', dtype='float32'),
                'observation.effort': dict(names=[f'{n}_effort' for n in NAMES], units='x', dtype='float32')}
    rows = {'observation.state': q, 'action': q + 0.001, 'observation.gripper': q[:, [6, 13]],
            'observation.effort': np.full((T, 14), np.nan, np.float32)}
    cams = {f'observation.images.{k}': [(i, frame(face and k == 'top', i)) for i in range(T)]
            for k in ('top', 'left_wrist', 'right_wrist')}
    dataset.append_episode(root, robot_type=profile, robot='testbot', profile=profile, fps=10.0, features=features,
                           rows=rows, cameras=cams, prompt=prompt, extras=extras or {})


def faces_in(mp4):
    cap = cv2.VideoCapture(str(mp4))
    found = 0
    finder = camera_privacy.FaceBlur()
    for _ in range(5):
        ok, bgr = cap.read()
        if not ok:
            break
        found += len(finder.detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)))
    cap.release()
    return found


@pytest.fixture
def yam_root(tmp_path):
    root = tmp_path / 'ds'
    episode(root, prompt='runtime task', extras=dict(task_id='r' * 32, outcome='done'), face=True)
    episode(root, prompt='fold the towel', extras=dict(task_id='p' * 32, provider='dreamlabs', outcome='done',
                                                        gripper_units='station', jaw_range=dict(left=[0, 1], right=[0, 1])))
    return root


def test_an_export_passes_dreamlabs_own_check_and_its_videos_are_blurred(yam_root, tmp_path, home):
    out = tmp_path / 'export'
    got = dd.export(dataset_root=yam_root, out=out, robot='testbot', say=lambda *_: None)
    assert got['validation']['ok'] is True, got['validation']
    assert got['episodes'] == 2 and got['withheld'] == []
    side = json.loads((out / 'embodiment.json').read_text())
    cfg = side['embodiment_config']
    assert cfg['state'] == {'left_arm': [0, 6], 'left_gripper': [6, 7], 'right_arm': [7, 13], 'right_gripper': [13, 14]}
    assert cfg['video'] == {'overhead': 'observation.images.top', 'wrist_l': 'observation.images.left_wrist',
                            'wrist_r': 'observation.images.right_wrist'}
    assert side['roborun']['arms'] == {'left': {'model': 'yam', 'joints': 6}, 'right': {'model': 'yam', 'joints': 6}}
    session = out / 'dataset' / 'roborun-testbot'
    info = json.loads((session / 'meta' / 'info.json').read_text())
    assert set(info['features']) == {'observation.state', 'action', 'timestamp', 'frame_index', 'episode_index', 'index',
                                     'task_index', 'observation.images.top', 'observation.images.left_wrist',
                                     'observation.images.right_wrist'}           # only what DreamLabs reads: no effort, no temps
    t = pq.read_table(session / 'data' / 'chunk-000' / 'episode_000000.parquet')
    state = np.array(t.column('observation.state').to_pylist())
    assert abs(state[0, 6] - 0.25) < 1e-6 and abs(state[0, 13] - 0.75) < 1e-6
    src = yam_root / 'videos' / 'chunk-000' / 'observation.images.top' / 'episode_000000.mp4'
    dst = session / 'videos' / 'chunk-000' / 'observation.images.top' / 'episode_000000.mp4'
    assert faces_in(src) >= 1 and faces_in(dst) == 0
    report = json.loads((out / 'validation.json').read_text())
    assert report['ok'] and report['client'].startswith('dreamlabs-client')
    from dream import datasets
    assert datasets.validate_dataset(str(out / 'dataset'), cfg).ok                      # the check itself, once more


def test_a_metal_arms_gripper_is_converted_to_the_open_fraction(tmp_path, home):
    root = tmp_path / 'metal'
    mid = METAL[0] + 0.5 * (METAL[1] - METAL[0])
    episode(root, profile='yam_metal', right_jaw=mid, extras=dict(task_id='m' * 32))                 # a runtime episode
    episode(root, profile='yam_metal', right_jaw=METAL[1], extras=dict(task_id='n' * 32, gripper_units='station',
                                                                     jaw_range=dict(left=[0, 1], right=list(METAL))))
    out = tmp_path / 'export'
    got = dd.export(dataset_root=root, out=out, robot='testbot', say=lambda *_: None)
    assert got['validation']['ok'], got['validation']
    session = out / 'dataset' / 'roborun-testbot'
    a = np.array(pq.read_table(session / 'data/chunk-000/episode_000000.parquet').column('action').to_pylist())
    b = np.array(pq.read_table(session / 'data/chunk-000/episode_000001.parquet').column('observation.state').to_pylist())
    assert abs(a[0, 13] - 0.5) < 2e-3 and abs(b[0, 13] - 1.0) < 1e-6
    side = json.loads((out / 'embodiment.json').read_text())
    assert side['roborun']['arms']['right']['model'] == 'metal_arm'


def test_short_episodes_are_left_out_and_an_export_never_overwrites(yam_root, tmp_path, home):
    episode(yam_root, prompt='short', T=8, extras=dict(task_id='s' * 32))
    out = tmp_path / 'export'
    got = dd.export(dataset_root=yam_root, out=out, robot='testbot', say=lambda *_: None)
    assert got['episodes'] == 2 and got['withheld'][0]['task_id'] == 's' * 32 and got['validation']['ok']
    with pytest.raises(dd.ExportError, match='not empty'):
        dd.export(dataset_root=yam_root, out=out, robot='testbot', say=lambda *_: None)
    one = dd.export(dataset_root=yam_root, out=tmp_path / 'one', episodes='p' * 32, robot='testbot', say=lambda *_: None)
    assert one['episodes'] == 1


def test_upload_sends_nothing_without_a_yes(yam_root, tmp_path, home, fake):
    out = tmp_path / 'export'
    dd.export(dataset_root=yam_root, out=out, robot='testbot', say=lambda *_: None)
    said = []
    with pytest.raises(dd.ExportError, match='nothing was sent'):
        dd.upload(out, 'towels-v1', ask=None, say=said.append)
    with pytest.raises(dd.ExportError, match='not agreed'):
        dd.upload(out, 'towels-v1', ask=lambda q: 'n', say=said.append)
    assert fake.requests == []
    assert any('This sends' in s and '127.0.0.1' in s for s in said) and any('face-blurred' in s for s in said)


def test_upload_then_finetune_then_the_model_runs_through_roborun(yam_root, tmp_path, home, fake):
    out = tmp_path / 'export'
    dd.export(dataset_root=yam_root, out=out, robot='testbot', say=lambda *_: None)
    got = dd.upload(out, 'towels-v1', ask=lambda q: 'y', say=lambda *_: None)
    assert got.status == 'ready_to_process' and got.id.startswith('ds_')
    assert [r['method'] for r in fake.requests if r['path'].startswith('/v1/embodiments')] == ['GET', 'POST', 'GET']
    blob = fake.objects[f'{got.id}.zip']
    names = zipfile.ZipFile(io.BytesIO(blob)).namelist()
    assert 'roborun-testbot/meta/info.json' in names and not any('embodiment.json' in n or 'validation' in n for n in names)
    assert fake.calls(f'/v1/datasets/{got.id}/complete')
    line = dd.uploads()[-1]
    assert line['kind'] == 'upload' and line['dataset_id'] == got.id and 'key' not in json.dumps(line).lower()
    dd.upload(out, 'towels-v2', yes=True, say=lambda *_: None)                          # the embodiment is reused
    assert len([r for r in fake.requests if r['path'] == '/v1/embodiments' and r['method'] == 'POST']) == 1
    with pytest.raises(dd.ExportError, match='nothing was sent'):
        dd.finetune(got.id, 'towels', ask=None, say=lambda *_: None)
    job = dd.finetune(got.id, 'towels', steps=500, yes=True, say=lambda *_: None)
    assert job['status'] == 'queued' and fake.calls('/v1/fine_tuning/jobs', 'POST')[0]['body']['dataset'] == got.name
    said = []
    dd.status(job['id'], say=said.append)
    assert 'queued' in said[0] and dl.known_models() == {}
    model = fake.succeed(job['id'])
    said = []
    dd.status(job['id'], say=said.append)
    assert model in dl.known_models() and any(f'--model {model}' in s for s in said)
    assert dl.known_models()[model]['embodiment_config']['state']['left_arm'] == [0, 6]
    said = []
    dd.status(got.id, say=said.append)
    assert got.id in said[0]


def test_a_failed_transfer_is_reported_to_dreamlabs(yam_root, tmp_path, home, fake):
    import dream
    out = tmp_path / 'export'
    dd.export(dataset_root=yam_root, out=out, robot='testbot', say=lambda *_: None)
    fake.fail_s3 = True
    with pytest.raises(dream.errors.DreamUploadError):
        dd.upload(out, 'towels-v1', yes=True, say=lambda *_: None)
    fails = [r for r in fake.requests if r['path'].endswith('/fail')]
    assert fails and fails[0]['body']['error']


def test_an_export_that_fails_dreamlabs_check_cannot_be_uploaded(yam_root, tmp_path, home, fake):
    out = tmp_path / 'export'
    dd.export(dataset_root=yam_root, out=out, robot='testbot', say=lambda *_: None)
    (out / 'validation.json').write_text(json.dumps(dict(ok=False, error='broken')))
    with pytest.raises(dd.ExportError, match='refused this export'):
        dd.upload(out, 'towels-v1', yes=True, say=lambda *_: None)
    assert fake.requests == []


def test_the_command(yam_root, tmp_path, home, capsys):
    from roborun_sdk import cli
    out = tmp_path / 'export'
    assert cli.main(['dreamlabs', 'export', '--dataset', str(yam_root), '--out', str(out)]) == 0
    text = capsys.readouterr().out
    assert 'exported 2 episodes' in text and "DreamLabs' check: ok" in text
    assert cli.main(['dreamlabs', 'upload', str(out), '--name', 'towels-v1']) == 2     # no terminal, no --yes
    assert 'nothing was sent' in capsys.readouterr().err
    assert cli.main(['dreamlabs', 'status']) == 0


def test_an_arm_with_unknown_gripper_units_writes_nothing(tmp_path, home):
    root = tmp_path / 'custom'
    episode(root, profile='custom', extras=dict(task_id='c' * 32))
    out = tmp_path / 'export'
    with pytest.raises(dd.ExportError, match='units are unknown'):
        dd.export(dataset_root=root, out=out, robot='testbot', say=lambda *_: None)
    assert not out.exists()
