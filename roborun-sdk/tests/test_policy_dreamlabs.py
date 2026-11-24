"""The DreamLabs provider, end to end: DreamLabs' real client (dreamlabs-client) against a local fake of its services
(fake_dreamlabs.py, 127.0.0.1 only), a real station process logic over HTTP and a camera broker. Nothing reaches DreamLabs,
and no real credentials file is read (DREAM_CREDENTIALS points at a file that does not exist)."""
import base64
import json
import re
import time
from pathlib import Path

import pytest

pytest.importorskip('dream')
cv2 = pytest.importorskip('cv2')
np = pytest.importorskip('numpy')

from fake_dreamlabs import FakeDreamLabs, sway  # noqa: E402
from fake_policy_bits import own_home  # noqa: E402
from test_policy_runner import HOME_Q, Rig  # noqa: E402

from roborun_sdk import policy  # noqa: E402
from roborun_sdk.policy import PolicyRefused  # noqa: E402
from roborun_sdk.policy import dreamlabs as dl  # noqa: E402
from roborun_sdk.policy import dreamlabs_yam  # noqa: E402
from roborun_sdk.policy import registry  # noqa: E402

HERE = Path(__file__).resolve().parent
KEY = 'dl_test_key_not_real_7f3a'
MODEL = 'dreamlabs/dream-1-beta'
CAMS = ('top', 'left', 'right')


class FaceCam:
    """A 640x352 camera holding a face (NASA's public-domain portrait, tests/data/astronaut_face.jpg)."""
    def __init__(self, role='top'):
        import cv2 as _cv2
        photo = _cv2.cvtColor(_cv2.imread(str(HERE / 'data' / 'astronaut_face.jpg')), _cv2.COLOR_BGR2RGB)
        self.img = np.full((352, 640, 3), 90, np.uint8)
        self.img[48:304, 192:448] = photo
        self.last = 0.0

    def read(self):
        now = time.monotonic()
        if now - self.last < 1 / 30:
            return None
        self.last = now
        return self.img.copy()


@pytest.fixture
def home(tmp_path, monkeypatch):
    return own_home(tmp_path, monkeypatch)              # never this computer's robot.json, runtime or $HOME


@pytest.fixture
def fake(home, monkeypatch):
    f = FakeDreamLabs()
    for k, v in f.env(home, KEY).items():
        monkeypatch.setenv(k, v)
    monkeypatch.delenv('DREAM_LOG_DIR', raising=False)
    yield f
    f.close()


@pytest.fixture
def rig(tmp_path, home):
    r = Rig(tmp_path, cameras=CAMS)
    yield r
    r.close()


def lease_starts(rig):
    return [e for e in rig.station.events if e[1] == 'lease_start']


def run(rig, seconds=2.0, **kw):
    kw.setdefault('consent', True)
    kw.setdefault('model', MODEL)
    return policy.run('dreamlabs', prompt=kw.pop('prompt', 'fold the towel'), seconds=seconds, **kw, **rig.kw())


def everything_written(rig, home, result):
    """Every byte Roborun wrote for the run: the record, the recording's metadata, the local provider files, home."""
    out = []
    for root in (rig.runtime / 'log', home):
        for p in Path(root).rglob('*'):
            if p.is_file() and p.suffix not in ('.mjpg', '.npz', '.mp4', '.parquet'):
                out.append(p.read_bytes())
    return b'\n'.join(out)


# ---- the wire ------------------------------------------------------------------------------------------------------------
def test_a_run_sends_the_observation_dream_1_reads_and_mints_then_echoes_a_session(rig, fake, home, capsys):
    events = []
    result = run(rig, seconds=2.5, on_event=events.append)
    assert result.status == 'done', result
    calls = fake.calls('/v1/get_action')
    assert len(calls) >= 2
    first, later = calls[0]['body'], calls[1]['body']
    assert list(first) == ['checkpoint', 'session_id', 'reset', 'observation']
    assert first['checkpoint'] == MODEL and first['session_id'] is None and first['reset'] is False
    assert later['session_id'] and re.fullmatch(r'[0-9a-f]{8}(-[0-9a-f]{4}){3}-[0-9a-f]{12}', later['session_id'])
    obs = first['observation']
    assert len(obs['state.left_arm']) == 6 and len(obs['state.right_arm']) == 6
    assert obs['state.left_gripper'] and 0.0 <= obs['state.left_gripper'][0] <= 1.0
    for side in ('left', 'right'):                      # the tool pose dream-1-beta reads (else DA-4000 malformed)
        xyz, rot6 = dreamlabs_yam.eef(obs[f'state.{side}_arm'])
        assert obs[f'state.{side}_eef_xyz'] == pytest.approx(xyz, abs=1e-12)
        assert obs[f'state.{side}_eef_rot6'] == pytest.approx(rot6, abs=1e-12)
        assert obs[f'state.{side}_grip'] == obs[f'state.{side}_gripper']
    assert obs['prompt'] == 'fold the towel' and obs.get('embodiment') == 'yam'
    for role in ('video.overhead', 'video.wrist_l', 'video.wrist_r'):
        frames = obs[role]
        assert 1 <= len(frames) <= 9
        img = cv2.imdecode(np.frombuffer(base64.b64decode(frames[-1]), np.uint8), cv2.IMREAD_COLOR)
        assert img.shape[:2] == (176, 320)
    assert calls[0]['headers']['Authorization'] == f'Bearer {KEY}'
    # the embodiment is passed explicitly and the recorder is off: no model lookup, no recorder connection
    assert not fake.calls('/v1/models') and not [r for r in fake.requests if r['path'].startswith('/live/')]
    assert fake.calls('/v1/end_session'), 'the session is ended when the run ends'
    rec = json.loads((rig.runtime / 'log' / 'tasks' / f'{result.task_id}.json').read_text())
    assert rec['kind'] == 'policy' and rec['model'] == f'dreamlabs:{MODEL}' and rec['safety']['chunks_played'] >= 1
    assert rec['policy']['provider_version'].startswith('dreamlabs-client ')
    assert rig.station.lease['reason'] == 'policy_finished'
    reqs = [json.loads(l) for l in (rig.runtime / 'log' / 'policy' / result.task_id / 'requests.jsonl').read_text().splitlines()]
    assert any(r.get('provider_ms') == 12 and r.get('request_bytes') for r in reqs)
    notices = [e['text'] for e in events if e['event'] == 'policy_notice']
    assert any('dream.check' in n or 'input_check' in n for n in notices), notices       # DreamLabs' own reminder, shown once
    assert len([n for n in notices if 'input_check' in n]) <= 1
    blob = everything_written(rig, home, result) + capsys.readouterr().out.encode()
    assert KEY.encode() not in blob, 'the key is never written or printed'


def test_record_true_lets_dreamlabs_recorder_dial(rig, fake):
    result = run(rig, seconds=1.5, options={'record': True})
    assert result.status == 'done'
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline and not [r for r in fake.requests if r['path'].startswith('/live/')]:
        time.sleep(0.05)
    assert [r for r in fake.requests if r['path'].startswith('/live/')]


def test_check_and_a_refused_consent_send_nothing(rig, fake):
    out = policy.check('dreamlabs', model=MODEL, **rig.kw())
    assert out['ok'] and out['remote'] and set(out['arms']) == {'left', 'right'} and out['key'].startswith('environment')
    with pytest.raises(PolicyRefused, match='does not know which arms'):
        policy.check('dreamlabs', model='dream-1-ft:x', **rig.kw())
    with pytest.raises(PolicyRefused, match='consent needed'):
        policy.run('dreamlabs', model=MODEL, prompt='p', seconds=1, **rig.kw())
    assert fake.requests == [] and not lease_starts(rig)


def test_the_consent_text_says_what_goes_to_dreamlabs(rig, fake):
    r = policy.check('dreamlabs', model=MODEL, **rig.kw())
    prov = registry.create('dreamlabs')
    runner = policy._runner(prov, model=MODEL, prompt='p', **rig.kw())
    runner.admit()
    text = prov.consent_text(runner.needs, True)
    assert 'DreamLabs (127.0.0.1:' in text and 'before any motion' in text and 'up to 9 recent frames' in text
    assert '320x176' in text and 'face-blurred' in text and '80 MB per minute' in text
    assert 'not sent to DreamLabs' in text and 'Roborun history' in text and '--no-data-sync' in text
    assert 'stay here' not in text and 'never leaves this computer' in text
    assert 'the joint angles, grippers and tool poses' in text and 'left and right arms' in text
    rec_sends = ' '.join(runner.needs.sends)
    assert 'tool poses' in rec_sends, 'the task record\'s policy.sends names the tool poses too'
    assert 'NOT face-blurred' in prov.consent_text(runner.needs, False)
    assert r['version'].startswith('dreamlabs-client 1.1')


def test_a_face_reaches_dreamlabs_blurred(tmp_path, home, fake):
    from roborun_sdk import camera_privacy, cameras as cams
    import tempfile
    r = Rig(tmp_path, cameras=())
    r.sock = tempfile.mkdtemp(prefix='pc') + '/c.sock'
    r.broker = cams.Broker({role: cams.CustomSource('test_policy_dreamlabs:FaceCam', role) for role in CAMS})
    r.broker.start()
    r.server = cams.listen(r.broker, r.sock)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and len(r.broker.frames) < 3:
        time.sleep(0.02)
    try:
        result = run(r, seconds=1.5)
        assert result.status == 'done', result
    finally:
        r.close()
    finder = camera_privacy.FaceBlur()
    sharp = cv2.resize(FaceCam().img, (320, 176), interpolation=cv2.INTER_AREA)
    assert len(finder.detect(sharp)) >= 1, 'the test frame has a face the detector finds'
    sent = fake.calls('/v1/get_action')[0]['body']['observation']['video.overhead'][-1]
    got = cv2.cvtColor(cv2.imdecode(np.frombuffer(base64.b64decode(sent), np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    assert len(camera_privacy.FaceBlur().detect(got)) == 0


def test_commanded_is_the_refill_ticks_and_the_seam_after_skip_is_within_one_tick(rig, fake, monkeypatch):
    import dream
    from roborun_station import policy as pol
    seen = []
    real = dream.DreamClient.get_action

    def spy(self, *, commanded=None):
        seen.append(commanded)
        return real(self, commanded=commanded)
    monkeypatch.setattr(dream.DreamClient, 'get_action', spy)
    fake.script = [{'delay_s': 0.3}] * 6
    obs_seen = []
    orig = dl.DreamLabsProvider.act

    def act(self, obs):
        obs_seen.append(obs)
        return orig(self, obs)
    monkeypatch.setattr(dl.DreamLabsProvider, 'act', act)
    result = run(rig, seconds=3.0)
    assert result.status == 'done', result
    for obs, cmd in zip(obs_seen, seen):
        for side, key in (('left', 'q_l_chunk'), ('right', 'q_r_chunk')):
            st = obs.arms[side]
            assert cmd[key][:6] == list(st.commanded_rad) and cmd[key][6] == st.commanded_gripper
    chunks = [json.loads(l) for l in (rig.runtime / 'log' / 'policy' / result.task_id / 'chunks.jsonl').read_text().splitlines()]
    accepted = [c for c in chunks if c['accepted']]
    assert len(accepted) >= 2 and rig.station.lease['policy']['clipped_ticks'] == 0
    for c in accepted:
        assert c['skip_s'] >= 0.3 and c['skipped_rows'] >= int(0.3 * 30) - 1


# ---- what the real dream-1-beta reads and answers (seen with a real account; the fake answers the same, made-up values)
def test_the_embodiment_declares_the_tool_pose_state_and_the_reply_s_grip_action(rig, fake):
    prov = registry.create('dreamlabs')
    runner = policy._runner(prov, model=MODEL, prompt='p', **rig.kw())
    runner.admit()
    state, action = prov.embodiment['state'], prov.embodiment['action']
    widths = {k: b - a for k, (a, b) in state.items()}
    assert widths == {'right_arm': 6, 'right_gripper': 1, 'left_arm': 6, 'left_gripper': 1, 'left_eef_xyz': 3,
                      'left_eef_rot6': 6, 'left_grip': 1, 'right_eef_xyz': 3, 'right_eef_rot6': 6, 'right_grip': 1}
    spans = sorted(state.values())
    assert spans[0][0] == 0 and all(a[1] == b[0] for a, b in zip(spans, spans[1:]))       # contiguous, no overlap
    assert action == {'right_arm': [0, 6], 'right_grip': [6, 7], 'left_arm': [7, 13], 'left_grip': [13, 14]}
    assert prov.sync_above_ms == 3000


def test_a_reply_with_only_per_key_is_read_from_arm_and_grip(rig, fake):
    # the real reply has q_*_chunk too; without them the rows come from action.<side>_arm and action.<side>_grip
    grip = 0.25
    fake.script = [{'per_key_only': True, 'chunk': lambda o: {s: [row[:-1] + [grip] for row in r]
                                                               for s, r in sway(o).items()}}] * 100
    result = run(rig, seconds=2.0)
    assert result.status == 'done', result.message
    assert result.summary['chunks_accepted'] >= 1
    left = rig.station.left
    assert left.jaw_cmd < left.spec.jaw_open, 'the gripper followed action.left_grip (it starts open)'
    body = fake.calls('/v1/get_action')[-1]['body']
    assert body['observation']['state.left_grip'] == body['observation']['state.left_gripper']


def test_dream_1_beta_refuses_a_joints_only_observation_and_nothing_moves(rig, fake):
    eef = ('_eef_xyz', '_eef_rot6', '_grip')

    class JointsOnly(dl.DreamLabsProvider):          # what this provider sent before: joints and grippers only
        def _base(self, arms):
            super()._base(arms)
            self.embodiment['state'] = {k: v for k, v in self.embodiment['state'].items() if not k.endswith(eef)}

        def _state(self, obs):
            return {k: v for k, v in super()._state(obs).items() if not k.endswith(eef)}

    result = policy.run(JointsOnly(), model=MODEL, prompt='p', seconds=5.0, consent=True, **rig.kw())
    assert result.status == 'failed' and 'malformed' in result.message and 'state.left_eef_xyz' in result.message
    assert len(fake.calls('/v1/get_action')) == 1 and not lease_starts(rig)


def test_a_real_round_trip_keeps_async_pacing_where_dreamlabs_default_would_go_sync(rig, fake, monkeypatch):
    modes = []
    orig = dl.DreamLabsProvider.close

    def close(self):
        modes.append(self.client.mode)
        return orig(self)
    monkeypatch.setattr(dl.DreamLabsProvider, 'close', close)
    fake.latency_s = (0.75, 1.0)                        # the measured warm round trip, p50 0.6 s .. p90 1.0 s and over
    result = run(rig, seconds=4.0)
    assert result.status == 'done', result
    fake.latency_s = (0.75, 1.0)
    fake.requests.clear()
    result = run(rig, seconds=4.0, options={'sync_above_ms': 700})
    assert result.status == 'done', result
    assert modes == ['async', 'sync'], modes


# ---- the error table, in order -------------------------------------------------------------------------------------------
def test_an_expired_session_ends_the_run_with_no_large_move(rig, fake):
    fake.script = [{}, {}, {'expired': True}]
    result = run(rig, seconds=10.0)
    assert result.status == 'stopped' and result.stop_reason == 'provider_session_restarted', result
    assert max(abs(a - b) for a, b in zip(rig.station.left.q_cmd, HOME_Q)) < 0.2


def test_a_refused_key_fails_with_dreamlabs_reason(rig, fake):
    fake.script = [{'code': 'DA-1000'}]
    result = run(rig, seconds=5.0)
    assert result.status == 'failed' and 'DreamLabs did not accept your DreamLabs API key' in result.message
    assert not lease_starts(rig)


def test_a_bad_request_fails(rig, fake):
    fake.script = [{'code': 'DA-4000', 'reason': 'unknown_checkpoint'}]
    result = run(rig, seconds=5.0)
    assert result.status == 'failed' and 'unknown_checkpoint' in result.message and not lease_starts(rig)


def test_rate_limited_is_honoured_and_the_run_continues(rig, fake):
    fake.script = [{}, {'code': 'DA-2000', 'reason': 'rate_limited', 'retryable': True, 'retry_after_s': 0.7}]
    result = run(rig, seconds=3.0)
    assert result.status == 'done', result
    calls = fake.calls('/v1/get_action')
    assert len(calls) >= 3 and calls[2]['at'] - calls[1]['at'] >= 0.65


def test_pool_warming_before_the_first_chunk_leases_nothing_then_not_ready(rig, fake):
    fake.script = [{'code': 'DA-2000', 'reason': 'pool_warming', 'retryable': True, 'retry_after_s': 0.2}] * 50
    events = []
    result = run(rig, seconds=5.0, wait_ready=1.0, on_event=events.append)
    assert result.status == 'failed' and result.stop_reason == 'provider_not_ready' and not lease_starts(rig)
    assert any(e['event'] == 'policy_warming' and 'warming' in e['text'] for e in events)



def test_a_stop_while_dreamlabs_pool_warms_leases_nothing(rig, fake):
    fake.script = [{'code': 'DA-2000', 'reason': 'pool_warming', 'retryable': True, 'retry_after_s': 0.2}] * 50
    run_ = policy.start('dreamlabs', prompt='fold the towel', seconds=5.0, consent=True, model=MODEL, **rig.kw())
    time.sleep(1.5)
    rig.station.hold({'source': 'test'})
    result = run_.wait(15)
    assert result.status == 'stopped' and result.stop_reason == 'operator_stop', result
    assert result.message == 'Stopped before anything moved' and not lease_starts(rig)
    assert max(abs(a - b) for a, b in zip(rig.station.left.q_cmd, HOME_Q)) < 1e-9

def test_a_model_failure_three_times_is_retried_and_the_fourth_fails(rig, fake):
    fake.script = [{}] + [{'code': 'DA-5000'}] * 4
    result = run(rig, seconds=10.0)
    assert result.status == 'failed' and '4 times in a row' in result.message, result
    assert len(fake.calls('/v1/get_action')) == 5


def test_a_model_failure_then_ok_continues(rig, fake):
    fake.script = [{}, {'code': 'DA-5000'}, {}]
    result = run(rig, seconds=3.0)
    assert result.status == 'done', result


def test_a_stall_longer_than_timeout_holds_then_continues(rig, fake):
    fake.script = [{}, {'delay_s': 1.5}]
    events = []
    result = run(rig, seconds=4.0, options={'timeout_ms': 700}, on_event=events.append)
    assert result.status == 'done', result
    assert any(e['event'] == 'policy_hold' and 'DreamTimeout' in str(e.get('why')) for e in events)
    assert result.summary['chunks_accepted'] >= 2


def test_a_reply_that_is_not_the_dream_api_holds_then_fails_after_the_stall_limit(rig, fake):
    fake.script = [{}] + [{'raw': b'not json'}, {'envelope': False}] * 200
    result = run(rig, seconds=20.0, stall_limit=2.0)
    assert result.status == 'failed' and result.stop_reason == 'provider_stalled', result


@pytest.mark.parametrize('chunk,code', [
    (lambda o: {s: r[:12] if s == 'right' else r for s, r in sway(o).items()}, 'rows'),
    (lambda o: {s: [row[:-1] + [float('nan')] for row in r] for s, r in sway(o).items()}, 'shape'),
    (lambda o: {s: [[q + (2.0 if i == 0 else 0.0) for i, q in enumerate(row[:-1])] + row[-1:] for row in r]
                for s, r in sway(o).items()}, 'seam'),
])
def test_a_wrong_shape_nan_or_a_2_rad_seam_is_refused_and_never_jumps(rig, fake, chunk, code):
    fake.script = [{}] + [{'chunk': chunk}] * 60
    run_ = policy.start('dreamlabs', model=MODEL, prompt='p', seconds=6.0, consent=True, **rig.kw())
    qs = []
    while run_.wait(0) is None and len(qs) < 600:
        qs.append(list(rig.station.left.q) + list(rig.station.right.q))       # where the arms are
        time.sleep(1 / 60)
    result = run_.wait(15)
    assert result.status == 'failed', result
    refused = dict(result.summary.get('refused') or {}, **((result.summary.get('safety') or {}).get('refused') or {}))
    assert any(code in k for k in refused) or 'unusable action' in result.message, (refused, result.message)
    steps = [max(abs(a - b) for a, b in zip(x, y)) for x, y in zip(qs, qs[1:])]
    assert max(steps) <= 2 * 0.04 + 1e-6, (code, max(steps))      # never a jump: within the per-tick clip


def test_a_slow_end_session_never_delays_stop(rig, fake):
    fake.end_session_delay_s = 2.0
    run_ = policy.start('dreamlabs', model=MODEL, prompt='p', seconds=20, consent=True, **rig.kw())
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline and not (rig.station.lease and rig.station.lease.get('policy', {}).get('chunks')):
        time.sleep(0.05)
    rig.station.hold({'source': 'test'})
    t = time.monotonic()
    time.sleep(0.1)
    q_cmd = [list(rig.station.left.q_cmd), list(rig.station.right.q_cmd)]
    time.sleep(0.3)                                       # end_session is still blocking: the arms hold all the same
    assert [list(rig.station.left.q_cmd), list(rig.station.right.q_cmd)] == q_cmd
    result = run_.wait(15)
    assert result.status == 'stopped' and rig.station.lease['reason'] == 'supervisor_stop'
    assert time.monotonic() - t < 6


# ---- before anything is sent ---------------------------------------------------------------------------------------------
def test_a_missing_extra_names_it(rig, monkeypatch):
    monkeypatch.setattr(dl, 'installed', lambda: False)
    with pytest.raises(PolicyRefused, match=r"roborun-sdk\[dreamlabs\]"):
        policy.check('dreamlabs', model=MODEL, **rig.kw())
    entry = [p for p in registry.providers() if p.name == 'dreamlabs'][0]
    assert entry.available and entry.version.startswith('dreamlabs-client')


def test_an_untested_client_version_is_a_notice(rig, fake, monkeypatch):
    monkeypatch.setattr(dl, 'client_version', lambda: '1.4.0')
    events = []
    result = run(rig, seconds=1.0, on_event=events.append)
    assert result.status == 'done'
    assert any('1.4.0 was not tested' in e.get('text', '') for e in events if e['event'] == 'policy_notice')


def test_without_a_verified_convention_nothing_is_sent_and_nothing_leased(rig, fake, monkeypatch):
    monkeypatch.setattr(dl, 'conventions', lambda: {})
    with pytest.raises(PolicyRefused, match='no verified joint convention for yam'):
        run(rig, seconds=1.0)
    assert fake.requests == [] and not lease_starts(rig)


def test_a_one_arm_robot_and_a_missing_camera_are_refused(tmp_path, home, fake):
    r = Rig(tmp_path, cameras=('top',), right=False)
    try:
        with pytest.raises(PolicyRefused, match='two YAM arms'):
            policy.check('dreamlabs', model=MODEL, **r.kw())
    finally:
        r.close()
    r = Rig(tmp_path / 'b', cameras=('top', 'left'))
    try:
        with pytest.raises(PolicyRefused, match='right camera'):
            policy.check('dreamlabs', model=MODEL, **r.kw())
    finally:
        r.close()
    assert fake.requests == []


def test_arms_left_sends_both_arms_state_and_moves_only_the_left(rig, fake):
    result = run(rig, seconds=2.0, arms='left')
    assert result.status == 'done', result
    obs = fake.calls('/v1/get_action')[0]['body']['observation']
    assert 'state.right_arm' in obs and 'state.left_arm' in obs
    assert rig.station.lease['arms'] == ['left']


def test_unknown_options_are_refused_before_anything(rig, fake):
    with pytest.raises(PolicyRefused, match='does not take the option'):
        policy.check('dreamlabs', model=MODEL, options={'host': 'x'}, **rig.kw())
    assert fake.requests == []


def test_a_fine_tune_runs_from_its_export_embodiment(rig, fake, tmp_path):
    config = {'state': {'left_arm': [0, 6], 'left_gripper': [6, 7], 'right_arm': [7, 13], 'right_gripper': [13, 14]},
              'video': {'overhead': 'observation.images.top', 'wrist_l': 'observation.images.left_wrist',
                        'wrist_r': 'observation.images.right_wrist'}}
    config['action'] = dict(config['state'])
    emb = tmp_path / 'embodiment.json'
    emb.write_text(json.dumps(dict(embodiment_config=config, roborun=dict(arms=dict(left=dict(model='yam', joints=6),
                                                                                     right=dict(model='yam', joints=6))))))
    fake.script = [{'per_key_only': True}] * 100
    result = run(rig, seconds=2.0, model='dream-1-ft:abcd1234-towels', options={'embodiment': str(emb)})
    assert result.status == 'done', result.message
    assert result.summary['chunks_accepted'] >= 1
    assert fake.calls('/v1/fine_tuning/models/dream-1-ft%3Aabcd1234-towels/training_data')
    assert not fake.calls('/v1/models')


# ---- DreamLabs' words never read as Roborun's own errors in the local API's task view ------------------------------------
def _task_view():
    """The runtime's task view when its source is given (ROBORUN_RUNTIME_TASK_VIEW), else None (the pinned patterns are
    checked instead)."""
    import importlib.util
    import os
    path = Path(os.environ.get('ROBORUN_RUNTIME_TASK_VIEW') or '/nonexistent/task_view.py')
    if not path.exists():
        return None
    spec = importlib.util.spec_from_file_location('roborun_runtime_task_view', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ERROR_TABLE = [
    ([{'code': 'DA-1000', 'reason': 'invalid_key'}], {}),
    ([{'code': 'DA-1000'}], {}),
    ([{'code': 'DA-1000', 'reason': 'read_only_key'}], {}),
    ([{'code': 'DA-4000', 'reason': 'unknown_checkpoint'}], {}),
    ([{'code': 'DA-4000', 'reason': 'malformed'}], {}),
    ([{}] + [{'code': 'DA-5000'}] * 4, {}),
    ([{}] + [{'code': 'DA-5000', 'reason': 'upstream_error'}] * 4, {}),
    ([{}] + [{'code': 'DA-6000', 'reason': 'upstream_error'}] * 60, dict(stall_limit=3.0)),
    ([{}] + [{'code': 'DA-6000'}] * 60, dict(stall_limit=3.0)),
    ([{}] + [{'code': 'DA-2000', 'reason': 'rate_limited', 'retryable': False}] * 60, dict(stall_limit=3.0)),
    ([{'code': 'DA-2000', 'reason': 'pool_warming', 'retryable': True, 'retry_after_s': 0.2}] * 50, dict(wait_ready=1.0)),
    ([{}, {}, {'expired': True}], {}),
    ([{}] + [{'raw': b'not json'}, {'envelope': False}] * 200, dict(stall_limit=2.0)),
]


@pytest.mark.parametrize('script,kw', ERROR_TABLE)
def test_every_dreamlabs_error_reads_as_dreamlabs_not_as_roborun_s_own(rig, fake, script, kw):
    from roborun_sdk.policy.wording import roborun_cause
    fake.script = list(script)
    result = run(rig, seconds=8.0, **kw)
    assert result.status in ('failed', 'stopped'), result
    rec = json.loads((rig.runtime / 'log' / 'tasks' / f'{result.task_id}.json').read_text())
    for text in (rec.get('error'), rec.get('final'), result.message):
        assert roborun_cause(text) is None, text
    view = _task_view()
    if view is not None:
        public = view.public(rec)
        assert (public.get('error_info') or {}).get('code') in (None, 'task_failed'), public
        assert 'Roborun API' not in public['message'] and 'roborun login' not in json.dumps(public), public


def test_with_no_dreamlabs_key_check_and_run_refuse_and_send_nothing(rig, fake, monkeypatch):
    monkeypatch.delenv('DREAM_API_KEY')                       # DREAM_CREDENTIALS already names a file that does not exist
    with pytest.raises(PolicyRefused, match='no DreamLabs key on this computer.*Nothing was sent'):
        policy.check('dreamlabs', model=MODEL, **rig.kw())
    with pytest.raises(PolicyRefused, match='no DreamLabs key'):
        run(rig, seconds=2.0)
    assert fake.requests == [] and not lease_starts(rig)
