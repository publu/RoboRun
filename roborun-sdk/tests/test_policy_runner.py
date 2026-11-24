"""Policy runs, end to end on a real station process logic (run_station with KinematicArm adapters, over HTTP on loopback), a
camera broker on a Unix socket and pure-Python providers: the order before anything moves, the telemetry, Stop, holds,
anchoring, refusals, warm-up, shadow, consent, loading. Nothing remote."""
import json
import os
import sys
import tempfile
import threading
import time
from pathlib import Path

import pytest

from fake_policy_bits import FakeCam, Raises, Sway, Warming, own_home

from roborun_sdk import policy
from roborun_sdk.policy import PolicyNotStarted, PolicyRefused, ProviderError
from roborun_sdk.policy import consent as consent_mod
from roborun_sdk.policy import registry
from roborun_sdk.policy.runner import PolicyRunner

HERE = Path(__file__).resolve().parent
HOME_Q = [0.0, 0.5, 0.8, 0.0, 0.3, 0.0]

# the fields the runtime's public task view reads from a record (a pinned copy: the runtime is not importable here)
PUBLIC_FIELDS = ('id', 'prompt', 'execute', 'status', 'created', 'elapsed_s', 'model', 'plan_id', 'stop_reason')
STATUSES = ('running', 'done', 'stopped', 'failed')


class Rig:
    def __init__(self, tmp, cameras=('top',), right=True, camera='fake_policy_bits:FakeCam'):
        from roborun_sdk import cameras as cams
        from roborun_sdk.embodiment import KinematicArm, run_station
        self.station, self.srv = run_station(KinematicArm(home=HOME_Q), KinematicArm(home=HOME_Q) if right else None, port=0)
        self.tmp = Path(tmp)
        self.runtime = self.tmp / 'runtime'
        self.dataset = self.tmp / 'dataset'
        self.broker = None
        self.sock = None
        if cameras:
            self.sock = tempfile.mkdtemp(prefix='pc') + '/c.sock'
            sources = {r: cams.CustomSource(camera, r) for r in cameras}
            self.broker = cams.Broker(sources)
            self.broker.start()
            self.server = cams.listen(self.broker, self.sock)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and len(self.broker.frames) < len(cameras):
                time.sleep(0.02)
        self.config = dict(robot='testbot', profile='yam', manifest=str(self.runtime), station=self.srv.url)
        self.runtime.mkdir(parents=True, exist_ok=True)    # a runtime that records its tasks, as the yam profiles do
        (self.runtime / 'config.json').write_text(json.dumps(dict(video_record=dict(enabled=True))))

    def kw(self, **extra):
        out = dict(station_url=self.srv.url, runtime=self.runtime, camera_socket=self.sock or '/nonexistent/c.sock',
                   config=self.config, dataset_root=self.dataset)
        out.update(extra)
        return out

    def close(self):
        self.srv.close()
        if self.broker is not None:
            self.server.shutdown()
            self.broker.close()


@pytest.fixture
def home(tmp_path, monkeypatch):
    return own_home(tmp_path, monkeypatch)              # never this computer's robot.json, runtime or $HOME


def install(monkeypatch, rig, **extra):
    """This computer's installed robot.json (robot.CONFIG) for the rig: what decides the robot-wide dataset setting."""
    from roborun_sdk import robot
    cfg = rig.tmp / 'installed-robot.json'
    cfg.write_text(json.dumps(dict(rig.config, **extra)))
    monkeypatch.setattr(robot, 'CONFIG', cfg)
    return cfg


@pytest.fixture
def rig(tmp_path, home, monkeypatch):
    r = Rig(tmp_path)
    install(monkeypatch, r)
    yield r
    r.close()


def actions(rig):
    return [e for e in rig.station.events if e[1] == 'lease_start']


def test_end_to_end_files_task_record_recording_episode_and_local_logs(rig):
    prov = Sway(arms=('left',), amplitude=0.05)
    result = policy.run(prov, prompt='sway the wrist', arms='left', seconds=2.0, **rig.kw())
    assert result.status == 'done' and result.stop_reason == 'finished', result
    rec = json.loads((rig.runtime / 'log' / 'tasks' / f'{result.task_id}.json').read_text())
    for k in PUBLIC_FIELDS:
        assert k in rec, k
    assert rec['status'] in STATUSES and rec['status'] == 'done' and rec['execute'] is True and rec['kind'] == 'policy'
    assert rec['plan_id'] == rec['id'] == result.task_id and len(rec['id']) == 32
    assert rec['model'] == 'sway' and rec['prompt'] == 'sway the wrist'
    assert rec['safety']['chunks_played'] >= 1 and 'rtt' not in json.dumps(rec) and 'provider_ms' not in json.dumps(rec)
    rdir = rig.runtime / 'log' / 'video-motion' / result.task_id / f'recording-{result.task_id}-01'
    meta = json.loads((rdir / 'recording.json').read_text())
    assert meta['schema'] == 'manifest-recording/1' and meta['state'] == 'ended' and meta['task_id'] == result.task_id
    frames = [json.loads(l) for l in (rdir / 'frames.jsonl').read_text().splitlines()]
    assert frames and all(set(('i', 'cam', 'k', 't', 'acq', 'fid', 'jpg', 'w', 'h')) <= set(f) for f in frames)
    blob = (rdir / 'top.mjpg').read_bytes()
    off, n = frames[-1]['jpg']
    assert blob[off:off + 2] == b'\xff\xd8' and len(blob) >= off + n
    import numpy as np
    j = np.load(rdir / 'joints.npz')
    assert list(j['columns'])[:2] == ['t', 'left_q1'] and len(j['rows']) > 20
    assert list(j['columns']) != []                                # never the runtime's 51 columns: its export skips it
    from roborun_sdk import dataset
    if dataset.unavailable():                                      # no dataset extra (pyarrow, av): said, nothing written
        assert result.summary['dataset'] == dict(recorded=False, why=dataset.unavailable()), result.summary['dataset']
        assert not rig.dataset.exists() or not any(rig.dataset.rglob('*'))
    else:
        pending = list((rig.dataset / '.pending').glob('*'))
        assert pending or (rig.dataset / 'meta' / 'info.json').exists()
    pol = rig.runtime / 'log' / 'policy' / result.task_id
    reqs = [json.loads(l) for l in (pol / 'requests.jsonl').read_text().splitlines()]
    assert reqs and all('rtt_ms' in r for r in reqs)
    summary = json.loads((pol / 'summary.json').read_text())
    assert summary['status'] == 'done' and summary['rtt_ms']['p50'] is not None
    assert prov.calls[:2] == ['needs', 'start'] and 'end_episode' in prov.calls and 'close' in prov.calls
    assert rig.station.lease['reason'] == 'policy_finished'


def test_the_history_upload_never_sees_the_policy_folder(rig, tmp_path):
    from roborun_sdk.datasync import DataSync
    from roborun_sdk.robot import _data_sources
    result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **rig.kw())
    for sources in (_data_sources(rig.runtime), _data_sources(rig.runtime, video_motion=rig.runtime / 'log' / 'video-motion'),
                    _data_sources(rig.runtime, data_root=rig.runtime / 'log')):
        sources = dict(sources, datasets=str(rig.dataset))
        sync = DataSync('http://127.0.0.1:9', 'k', 'testbot', sources, str(tmp_path / 'state.json'), log=lambda m: None)
        names = [name for name, _ in sync._files()]
        assert any(n.startswith('tasks/') for n in names)
        assert not any('policy/' in n or 'requests.jsonl' in n or 'summary.json' in n for n in names), names
    assert (rig.runtime / 'log' / 'policy' / result.task_id / 'requests.jsonl').exists()


def test_stop_mid_run_ends_it_stopped_and_end_episode_runs_after_the_hold(rig):
    prov = Sway(amplitude=0.1)
    run = policy.start(prov, prompt='p', arms='left', seconds=20, **rig.kw())
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline and not (rig.station.lease and rig.station.lease.get('policy', {}).get('chunks')):
        time.sleep(0.05)
    rig.station.hold({'source': 'test'})
    held = time.monotonic()
    result = run.wait(15)
    assert result.status == 'stopped' and result.stop_reason == 'operator_stop'
    assert rig.station.lease['reason'] == 'supervisor_stop'
    rec = json.loads((rig.runtime / 'log' / 'tasks' / f'{result.task_id}.json').read_text())
    assert rec['status'] == 'stopped' and rec['stop_reason'] == 'operator_stop'
    assert prov.ended == 1 and time.monotonic() >= held


def test_a_slow_act_holds_without_a_jump_and_the_heartbeat_keeps_the_lease(rig):
    prov = Sway(amplitude=0.05, act_s=0.0)
    run = policy.start(prov, prompt='p', arms='left', seconds=6, **rig.kw())
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline and not (rig.station.lease and rig.station.lease.get('policy', {}).get('chunks')):
        time.sleep(0.05)
    prov.act_s = 3.0                                          # every later act takes 3 s: the buffer runs dry, the arm holds
    qs = []
    t_end = time.monotonic() + 3.5
    while time.monotonic() < t_end:
        qs.append(list(rig.station.left.q_cmd)); time.sleep(1 / 30)
    assert rig.station.lease['active']                        # the heartbeat runs on its own thread
    steps = [max(abs(a - b) for a, b in zip(x, y)) for x, y in zip(qs, qs[1:])]
    assert max(steps) <= 0.04 + 1e-9
    assert rig.station.lease['policy']['dry_ticks'] > 0
    run.stop()
    assert run.wait(15).status == 'stopped'


def test_the_chunk_is_anchored_to_a_fresh_refill_tick(rig):
    prov = Sway(amplitude=0.05)
    result = policy.run(prov, prompt='p', arms='left', seconds=2.5, **rig.kw())
    assert result.status == 'done'
    chunks = [json.loads(l) for l in (rig.runtime / 'log' / 'policy' / result.task_id / 'chunks.jsonl').read_text().splitlines()]
    assert len(chunks) >= 2
    for c in chunks:
        assert abs(c['skip_s'] - (c['pushed_at'] - c['observed_at'])) < 0.005
    for obs, called in prov.acted:                        # act() got the observation the last observe() got
        before = [o for o in prov.observed if o.t <= obs.t]
        assert before and before[-1] is obs


def test_a_provider_exception_fails_the_run(rig):
    prov = Raises(error=RuntimeError('model blew up'), after=1)
    result = policy.run(prov, prompt='p', arms='left', seconds=5, **rig.kw())
    assert result.status == 'failed' and 'model blew up' in result.message
    rec = json.loads((rig.runtime / 'log' / 'tasks' / f'{result.task_id}.json').read_text())
    assert rec['status'] == 'failed' and 'model blew up' in rec['error']


def test_a_stale_pose_means_no_act(rig):
    def broken():
        raise OSError('bus gone')
    rig.station.left.adapter.read = broken                   # no new sample: the pose ages and goes stale
    time.sleep(0.3)
    prov = Sway()
    result = PolicyRunner(prov, prompt='p', arms='left', seconds=1, wait_ready=0.5, **rig.kw()).run()
    assert result.status == 'failed' and result.stop_reason == 'provider_not_ready'
    assert prov.acted == [] and not actions(rig)


@pytest.mark.parametrize('prov, arms, why', [
    (Sway(arms=('left',), cameras={'wrist': 'right'}), 'left', 'needs the right camera'),
    (Sway(arms=('left',)), 'middle', '--arms takes left, right or both'),
])
def test_a_mismatch_is_refused_before_any_lease(rig, prov, arms, why):
    with pytest.raises(PolicyRefused, match=why):
        policy.run(prov, prompt='p', arms=arms, seconds=1, **rig.kw())
    assert not actions(rig) and prov.started == 0


def test_an_absent_side_and_a_wrong_joint_count_are_refused(tmp_path, home):
    r = Rig(tmp_path, cameras=(), right=False)
    try:
        with pytest.raises(PolicyRefused, match='right arm'):
            policy.run(Sway(arms=('right',)), prompt='p', seconds=1, **r.kw())
        bad = Sway(arms=('left',))
        bad.act = lambda obs: policy.Chunk({'left': [[0.0] * 3] * 10}, hz=30)
        result = policy.run(bad, prompt='p', arms='left', seconds=2, **r.kw())
        assert result.status == 'failed' and result.stop_reason == 'provider_bad_chunks'
        assert not actions(r)
    finally:
        r.close()


def test_arms_left_drops_the_other_side(rig):
    prov = Sway(arms=('left', 'right'))
    right_before = list(rig.station.right.q_cmd)
    result = policy.run(prov, prompt='p', arms='left', seconds=1.5, **rig.kw())
    assert result.status == 'done'
    assert actions(rig)[-1][2]['arms'] == ['left']
    assert max(abs(a - b) for a, b in zip(rig.station.right.q_cmd, right_before)) < 1e-6


class FakeClock:
    def __init__(self):
        self.offset = 0.0

    def __call__(self):
        return time.monotonic() + self.offset


def test_a_model_ready_after_60_s_does_not_start_moving(rig):
    clock = FakeClock()
    prov = Warming(warm=1, jump_s=61.0, clock=clock)
    with pytest.raises(PolicyNotStarted, match='model ready; run again'):
        policy.run(prov, prompt='p', arms='left', seconds=1, clock=clock, **rig.kw())
    assert not actions(rig)


def test_a_yes_after_a_late_start_asks_for_a_fresh_chunk_then_leases(rig):
    clock = FakeClock()
    prov = Warming(warm=1, jump_s=61.0, clock=clock)
    runner = PolicyRunner(prov, prompt='p', arms='left', seconds=1, clock=clock, ask=lambda q: 'y', **rig.kw())
    result = runner.run()
    assert result.status == 'done' and prov.n >= 3 and len(prov.acted) >= 2 and actions(rig)   # warming, late, fresh


def test_warming_past_wait_ready_ends_as_not_ready(rig):
    prov = Warming(warm=10 ** 6)
    result = policy.run(prov, prompt='p', arms='left', seconds=1, wait_ready=0.5, **rig.kw())
    assert result.status == 'failed' and result.stop_reason == 'provider_not_ready' and not actions(rig)



class SlowFirst(Sway):
    """The first act() takes `first_s` (a cold model); later ones are quick."""
    def __init__(self, first_s=3.0, **kw):
        super().__init__(**kw)
        self.first_s, self.n = first_s, 0

    def act(self, obs):
        self.n += 1
        if self.n == 1:
            time.sleep(self.first_s)
        return super().act(obs)


def _stopped_before_moving(rig, result):
    assert result.status == 'stopped' and result.stop_reason == 'operator_stop', result
    assert result.message == 'Stopped before anything moved'
    assert not actions(rig) and rig.station.lease is None                 # nothing leased, nothing moved
    assert max(abs(a - b) for a, b in zip(rig.station.left.q_cmd, HOME_Q)) < 1e-9
    rec = json.loads((rig.runtime / 'log' / 'tasks' / f'{result.task_id}.json').read_text())
    assert rec['status'] == 'stopped' and rec['stop_reason'] == 'operator_stop'


def test_a_stop_while_the_first_act_is_slow_ends_the_run_with_nothing_leased(rig):
    run = policy.start(SlowFirst(first_s=3.0), prompt='p', arms='left', seconds=4, **rig.kw())
    time.sleep(1.0)
    held = time.monotonic()
    assert rig.station.hold({'source': 'test'})['actions'] == ['nothing_moving']
    result = run.wait(15)
    assert time.monotonic() - held < 1.5                   # seen within a poll, not after the slow act()
    _stopped_before_moving(rig, result)


def test_a_stop_while_the_model_warms_ends_the_run_with_nothing_leased(rig):
    run = policy.start(Warming(warm=10 ** 6), prompt='p', arms='left', seconds=4, **rig.kw())
    time.sleep(1.5)
    rig.station.hold({'source': 'test'})
    time.sleep(1.0)                                         # past the station's 0.75 s latch
    _stopped_before_moving(rig, run.wait(15))


def test_a_stop_between_warm_up_and_the_lease_ends_the_run_with_nothing_leased(rig, monkeypatch):
    from roborun_sdk.policy import runner as runner_mod
    original = runner_mod.PolicyRunner._warm_up

    def warm_then_stop(self):
        first = original(self)
        rig.station.hold({'source': 'test'})
        time.sleep(1.0)                                     # the latch is over: only the Stop counter remembers
        return first
    monkeypatch.setattr(runner_mod.PolicyRunner, '_warm_up', warm_then_stop)
    _stopped_before_moving(rig, policy.run(Sway(), prompt='p', arms='left', seconds=2, **rig.kw()))


def test_a_stop_after_the_runner_s_last_read_is_refused_by_the_station(rig, monkeypatch):
    from roborun_sdk.policy import runner as runner_mod
    from roborun_sdk.policy.station import Station
    original = Station.lease

    refused = []

    def stop_then_lease(self, *a, **kw):
        assert kw.get('since_hold_generation') is not None
        rig.station.hold({'source': 'test'})
        time.sleep(1.0)
        try:
            return original(self, *a, **kw)
        except Exception as exc:
            refused.append(str(exc))
            raise
    monkeypatch.setattr(Station, 'lease', stop_then_lease)
    _stopped_before_moving(rig, policy.run(Sway(), prompt='p', arms='left', seconds=2, **rig.kw()))
    assert refused == ['ValueError: Stop was pressed after this run started; nothing moved; run again']
    # and with the runner's own reads switched off, the station alone keeps the arms still
    monkeypatch.setattr(runner_mod.PolicyRunner, '_check_hold', lambda self, pose=None, force=False: None)
    result = policy.run(Sway(), prompt='p', arms='left', seconds=2, **rig.kw())
    assert result.status == 'failed' and result.stop_reason == 'lease_refused' and 'nothing moved' in result.message
    assert not actions(rig) and rig.station.lease is None

def test_shadow_moves_nothing_and_reports_the_seam(rig):
    prov = Sway(amplitude=0.05)
    report = policy.run(prov, prompt='p', arms='left', seconds=1.5, shadow=True, **rig.kw())
    assert not actions(rig)
    data = json.loads(Path(report.path).read_text())
    assert report.chunks >= 1 and data['chunks'] and report.seam_rad_max is not None
    assert report.joint_speed_rad_s_max == pytest.approx(0.05 * 2 * 3.14159 / 2.0, rel=0.2)
    assert not (rig.runtime / 'log' / 'tasks').exists()


def test_consent_is_needed_before_start_and_stored_answers_are_reused(rig, home):
    prov = Sway(remote=True)
    with pytest.raises(PolicyRefused, match='consent needed'):
        policy.run(prov, prompt='p', arms='left', seconds=1, **rig.kw())
    assert prov.started == 0 and not actions(rig)
    asked = []
    PolicyRunner(prov, prompt='p', arms='left', seconds=0.6, ask=lambda q: asked.append(q) or 'y', **rig.kw()).run()
    assert asked and (home / 'policy-consent.json').exists()
    assert oct((home / 'policy-consent.json').stat().st_mode & 0o777) == '0o600'
    asked.clear()
    PolicyRunner(prov, prompt='p', arms='left', seconds=0.6, ask=lambda q: asked.append(q) or 'y', **rig.kw()).run()
    assert not asked                                       # stored
    assert not (home / 'robot.yaml').exists()
    assert consent_mod.revoke('sway') == ['sway:']


def test_consent_is_asked_again_when_face_blur_turns_off(rig, home, monkeypatch):
    prov = Sway(remote=True)
    PolicyRunner(prov, prompt='p', arms='left', seconds=0.6, ask=lambda q: 'y', **rig.kw()).run()
    monkeypatch.setattr(PolicyRunner, '_face_blur', lambda self: False)
    with pytest.raises(PolicyRefused, match='consent needed'):
        policy.run(prov, prompt='p', arms='left', seconds=0.6, **rig.kw())


def test_python_run_never_reads_stdin(rig, monkeypatch):
    monkeypatch.setattr('builtins.input', lambda *a: pytest.fail('read stdin'))
    with pytest.raises(PolicyRefused):
        policy.run(Sway(remote=True), prompt='p', arms='left', seconds=1, **rig.kw())


def test_a_remote_provider_gets_nothing_when_the_blur_fails(rig, monkeypatch):
    from roborun_sdk import camera_privacy
    monkeypatch.setattr('roborun_sdk.policy.runner._blur_problem', lambda: None)

    def broken(*a, **k):
        raise camera_privacy.BlurUnavailable('no detector')
    monkeypatch.setattr(camera_privacy, 'clear_arrays', broken)
    prov = Sway(remote=True, cameras={'overhead': 'top'})
    result = PolicyRunner(prov, prompt='p', arms='left', seconds=1, wait_ready=5, yes=True, **rig.kw()).run()
    assert result.status == 'failed' and result.stop_reason == 'blur_unavailable'
    assert prov.observed == [] and prov.acted == [] and not actions(rig)


def test_frames_reach_a_local_provider_as_jpegs(rig):
    prov = Sway(cameras={'overhead': 'top'})
    policy.run(prov, prompt='p', arms='left', seconds=1, **rig.kw())
    frame = prov.acted[0][0].frames['top']
    assert frame.jpeg[:2] == b'\xff\xd8' and (frame.width, frame.height) == (64, 48)


def test_providers_load_from_a_file_and_an_entry_point(rig, monkeypatch):
    kind = registry.resolve(f'{HERE / "fake_policy_bits.py"}:FileProvider')
    assert kind.name == 'file-sway'
    prov = registry.create(f'{HERE / "fake_policy_bits.py"}:FileProvider', {'amplitude': 0.02})
    assert prov.amplitude == 0.02
    with pytest.raises(PolicyRefused, match='does not take the option'):
        registry.create(f'{HERE / "fake_policy_bits.py"}:FileProvider', {'nope': 1})

    class EP:
        name, value, dist = 'mine', 'fake_policy_bits:Sway', None
        def load(self):
            return Sway
    monkeypatch.setattr(registry, 'entry_points', lambda: {'mine': EP()})
    assert registry.resolve('mine') is Sway
    assert any(p.name == 'mine' and p.source == 'entry point' for p in registry.providers())
    result = policy.run('mine', prompt='p', arms='left', seconds=0.6, **rig.kw())
    assert result.status == 'done'


def test_hold_moves_nothing(rig):
    before = list(rig.station.left.q)
    result = policy.run('hold', prompt='self-test', arms='left', seconds=1.0, **rig.kw())
    assert result.status == 'done'
    assert max(abs(a - b) for a, b in zip(rig.station.left.q, before)) < 1e-3


def test_a_second_install_writes_only_under_its_own_home(tmp_path, monkeypatch):
    a, b = tmp_path / 'a', tmp_path / 'b'
    monkeypatch.setenv('ROBORUN_HOME', str(b))
    consent_mod.remember('x', 'm', policy.Needs(arms=('left',), cameras={}, frame_hz=1, frame_size=None, remote=True,
                                                 sends=('s',), version='v'), True)
    assert (b / 'policy-consent.json').exists() and not a.exists()


def test_runs_lists_past_policy_runs(rig):
    from roborun_sdk.policy.cli import runs
    result = policy.run(Sway(), prompt='p', arms='left', seconds=0.6, **rig.kw())
    rows = runs(5, runtime=rig.runtime)
    assert rows[0]['id'] == result.task_id and rows[0]['status'] == 'done' and rows[0]['provider'] == 'sway'


def test_replay_plays_a_recorded_episode(rig, tmp_path):
    import numpy as np
    q = np.array([HOME_Q + [1.0] + HOME_Q + [1.0]] * 60, dtype=float)
    q[:, 5] += np.linspace(0, 0.1, 60)
    names = [f'{s}_joint{i}' for s in ('left',) for i in range(1, 7)] + ['left_gripper'] + \
            [f'right_joint{i}' for i in range(1, 7)] + ['right_gripper']
    np.savez(tmp_path / 'ep.npz', action=q, names=np.array(names), hz=np.array(30.0))
    result = policy.run('replay', prompt='replay', arms='left', seconds=2.5, options={'path': str(tmp_path / 'ep.npz')},
                        **rig.kw())
    assert result.status == 'done'
    assert rig.station.left.q_cmd[5] == pytest.approx(0.1, abs=0.01)


def test_the_policy_episode_appends_to_a_dataset_with_runtime_episodes(rig, tmp_path):
    from roborun_sdk import dataset
    if dataset.unavailable():
        pytest.skip('dataset tools not installed')
    from roborun_sdk.policy.record import Recorder
    import numpy as np
    runner_box = {}
    prov = Sway(amplitude=0.05)
    orig = Recorder.stop

    def keep(self, reason):
        runner_box['rec'] = self
        return orig(self, reason)
    Recorder.stop = keep
    try:
        policy.run(prov, prompt='p', arms='left', seconds=2.0, dataset=False, **rig.kw())
    finally:
        Recorder.stop = orig
    rec = runner_box['rec']
    ep = rec.episode(10.0)
    assert ep['features']['observation.state']['names'][:2] == ['left_joint1', 'left_joint2']
    assert set(ep['features']) == {'observation.state', 'action', 'observation.gripper', 'observation.effort',
                                   'observation.motor_temperature_c'}
    assert list(ep['cameras']) == ['observation.images.top']
    root = tmp_path / 'ds'
    # a runtime-shaped episode first (the runtime export's feature names and units), then the policy episode
    T = len(ep['rows']['action'])
    runtime_like = dict(fps=10.0, features=ep['features'], rows={k: np.array(v) for k, v in ep['rows'].items()},
                        cameras=ep['cameras'])
    dataset.append_episode(root, robot_type='yam', robot='testbot', profile='yam', prompt='runtime task',
                           extras=dict(task_id='r' * 32, outcome='done'), **runtime_like)
    dataset.append_episode(root, robot_type='yam', robot='testbot', profile='yam', prompt='p',
                           extras=dict(task_id='p' * 32, provider='sway', outcome='done', gripper_units='station'), **ep)
    info = dataset.load_info(root)
    assert info['total_episodes'] == 2 and T >= 2



@pytest.mark.parametrize('how', ['camera serials', 'robot.json sim'])
def test_a_simulated_run_never_joins_the_robot_s_dataset(tmp_path, home, monkeypatch, how):
    r = Rig(tmp_path, camera='fake_policy_bits:SimCam' if how == 'camera serials' else 'fake_policy_bits:FakeCam')
    try:
        if how == 'robot.json sim':
            r.config = dict(r.config, sim='cup')
        install(monkeypatch, r)
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **r.kw())
        assert result.status == 'done'
        want = 'simulated cameras' if how == 'camera serials' else 'simulated robot'
        assert result.summary['dataset'] == dict(recorded=False, why=want)
        rec = json.loads((r.runtime / 'log' / 'tasks' / f'{result.task_id}.json').read_text())
        assert rec['dataset'] == dict(recorded=False, why=want)
        assert not r.dataset.exists() or not any(r.dataset.rglob('*'))           # nothing staged, nothing written
        rdir = r.runtime / 'log' / 'video-motion' / result.task_id / f'recording-{result.task_id}-01'
        assert json.loads((rdir / 'recording.json').read_text())['state'] == 'ended'     # the recording is still kept
    finally:
        r.close()


def test_a_real_camera_s_run_joins_the_dataset(rig):
    from roborun_sdk import dataset
    if dataset.unavailable():
        pytest.skip(dataset.unavailable())
    result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **rig.kw())
    assert result.summary['dataset']['recorded'] is True, result.summary['dataset']
    cams = [json.loads(l) for l in (rig.runtime / 'log' / 'video-motion' / result.task_id /
                                     f'recording-{result.task_id}-01' / 'cameras.jsonl').read_text().splitlines()]
    assert {c['serial'] for c in cams} == {'testcam-top'}

def test_the_policy_package_imports_with_the_standard_library_only():
    import subprocess
    code = ('import sys; import roborun_sdk.policy, roborun_sdk.policy.cli, roborun_sdk.policy.runner, '
            'roborun_sdk.policy.builtin, roborun_sdk.policy.consent, roborun_sdk.policy.dreamlabs, '
            'roborun_sdk.policy.dreamlabs_data; '
            'heavy = [m for m in ("numpy", "cv2", "mujoco", "pyarrow", "dream", "boto3") if m in sys.modules]; print(heavy); '
            'sys.exit(1 if heavy else 0)')
    src = str(Path(__file__).resolve().parents[1] / 'src')
    station = str(Path(__file__).resolve().parents[2] / 'roborun-station' / 'src')
    out = subprocess.run([sys.executable, '-c', code], env=dict(os.environ, PYTHONPATH=f'{src}{os.pathsep}{station}'),
                         capture_output=True, text=True)
    assert out.returncode == 0, out.stdout + out.stderr


def test_the_cli_checks_offline_and_refuses_with_exit_2(rig, monkeypatch, capsys):
    from roborun_sdk import cli, robot
    cfg = rig.tmp / 'robot.json'
    cfg.write_text(json.dumps(dict(rig.config, manifest=str(rig.runtime))))
    monkeypatch.setattr(robot, 'CONFIG', cfg)
    monkeypatch.setenv('MANIFEST_CAMERA_SOCKET', rig.sock)
    assert cli.main(['policy', 'providers']) == 0
    assert 'hold' in capsys.readouterr().out
    file = f'{HERE / "fake_policy_bits.py"}:FileProvider'
    assert cli.main(['policy', 'check', '--provider', file, '--arms', 'left', '--json']) == 0
    out = json.loads(capsys.readouterr().out)
    assert out['ok'] and out['arms'] == ['left'] and out['arm_models'] == {'left': 'yam', 'right': 'yam'}
    assert cli.main(['policy', 'check', '--provider', file, '--option', 'nope=1']) == 2
    assert 'does not take the option' in capsys.readouterr().err
    assert not actions(rig)
    assert cli.main(['policy', 'run', '--provider', file, '--arms', 'left', '--seconds', '1', '--prompt', 'p', '--json',
                     '--no-dataset']) == 0
    lines = [json.loads(l) for l in capsys.readouterr().out.splitlines()]
    assert lines[-1]['event'] == 'policy_result' and lines[-1]['status'] == 'done'
    assert {'policy_start', 'policy_lease', 'policy_chunk', 'policy_end'} <= {l['event'] for l in lines}
    assert cli.main(['policy', 'runs', '--json']) == 0
    assert json.loads(capsys.readouterr().out)[0]['status'] == 'done'


@pytest.mark.parametrize('cfg, why', [
    (dict(recording=dict(dataset=False)), "the robot's dataset is off (roborun up --no-dataset)"),
    (dict(data=dict(opt_out=True)), "this robot's data is not kept (roborun up --no-data-sync)"),
])
def test_the_robot_wide_dataset_setting_wins_over_the_run(tmp_path, home, monkeypatch, cfg, why):
    r = Rig(tmp_path)
    try:
        r.config = dict(r.config, **cfg)
        install(monkeypatch, r)
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **r.kw())
        assert result.status == 'done'
        assert result.summary['dataset'] == dict(recorded=False, why=why)
        assert not r.dataset.exists() or not any(r.dataset.rglob('*'))
    finally:
        r.close()


def test_robot_wants_dataset_follows_the_runtime_rule():
    from roborun_sdk.policy.record import robot_wants_dataset
    assert robot_wants_dataset(None) == (False, 'no robot.json on this computer')
    assert robot_wants_dataset({}) == (True, None)
    assert robot_wants_dataset(dict(recording=dict(dataset=True), data=dict(opt_out=False))) == (True, None)
    assert robot_wants_dataset(dict(recording=None, data=None)) == (True, None)
    assert robot_wants_dataset(dict(recording=dict(dataset=False)))[0] is False
    assert robot_wants_dataset(dict(data=dict(opt_out=1)))[0] is False


def test_no_robot_json_means_no_episode(monkeypatch, tmp_path, home):
    from roborun_sdk import robot
    from roborun_sdk.policy import runner
    monkeypatch.setattr(robot, 'CONFIG', tmp_path / 'missing' / 'robot.json')
    assert runner._config() is None
    r = Rig(tmp_path)
    try:                                                   # a whole run with no robot.json (config=None reads it)
        kw = r.kw(config=None)
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **kw)
        assert result.status == 'done'
        assert result.summary['dataset'] == dict(recorded=False, why='no robot.json on this computer')
        assert not r.dataset.exists() or not any(r.dataset.rglob('*'))
        assert result.summary['recording']                 # the recording is kept, as for every task
    finally:
        r.close()


def test_the_runtime_config_json_overrides_either_way_as_the_runtime_s_export():
    from roborun_sdk.policy.record import robot_wants_dataset
    off = dict(recording=dict(dataset=False))
    assert robot_wants_dataset(off, dict(lerobot_export=dict(enabled=True))) == (True, None)
    assert robot_wants_dataset({}, dict(lerobot_export=dict(enabled=False))) == (
        False, 'the dataset export is off (config.json lerobot_export.enabled)')
    assert robot_wants_dataset(off, dict(lerobot_export=dict(enabled='yes')))[0] is False      # only true/false count
    assert robot_wants_dataset({}, dict(lerobot_export={})) == (True, None)
    assert robot_wants_dataset({}, None) == (True, None)


def test_config_json_lerobot_export_false_keeps_a_run_out_of_the_dataset(tmp_path, home, monkeypatch):
    r = Rig(tmp_path)
    try:
        install(monkeypatch, r)
        r.runtime.mkdir(parents=True, exist_ok=True)
        (r.runtime / 'config.json').write_text(json.dumps(dict(lerobot_export=dict(enabled=False), video_record=dict(enabled=True))))
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **r.kw())
        assert result.summary['dataset']['recorded'] is False and 'config.json' in result.summary['dataset']['why']
        assert not r.dataset.exists() or not any(r.dataset.rglob('*'))
        rec = json.loads((r.runtime / 'log' / 'tasks' / f'{result.task_id}.json').read_text())
        assert rec['dataset'] == result.summary['dataset']
    finally:
        r.close()


def test_the_run_s_own_flag_only_turns_the_episode_off(rig):
    result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, dataset=False, **rig.kw())
    assert result.summary['dataset'] == dict(recorded=False, why='--no-dataset')
    assert not rig.dataset.exists() or not any(rig.dataset.rglob('*'))


def test_config_json_lerobot_export_root_and_robot_name_the_dataset_as_for_every_task(tmp_path, home, monkeypatch):
    from roborun_sdk import dataset
    if dataset.unavailable():
        pytest.skip(dataset.unavailable())
    r = Rig(tmp_path)
    try:
        install(monkeypatch, r)
        r.runtime.mkdir(parents=True, exist_ok=True)
        own = tmp_path / 'own-dataset'
        (r.runtime / 'config.json').write_text(json.dumps(dict(lerobot_export=dict(root=str(own), robot='bench-7'),
                                                                video_record=dict(enabled=True))))
        kw = r.kw()
        kw.pop('dataset_root')
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **kw)
        assert result.summary['dataset']['recorded'] is True, result.summary['dataset']
        assert str(own) in result.summary['dataset']['staged']
        assert not r.dataset.exists() or not any(r.dataset.rglob('*'))
    finally:
        r.close()


@pytest.mark.parametrize('cfg', [dict(data=dict(opt_out=True)), dict(recording=dict(dataset=False))])
def test_a_caller_s_own_config_or_runtime_never_turns_the_robot_s_dataset_on(tmp_path, home, monkeypatch, cfg):
    """roborun up --no-dataset / --no-data-sync in the installed robot.json: a run whose caller passes its own config= (no
    such setting) and runtime= (a config.json that turns the export on) still writes no episode."""
    r = Rig(tmp_path)
    try:
        install(monkeypatch, r, **cfg)
        r.runtime.mkdir(parents=True, exist_ok=True)
        (r.runtime / 'config.json').write_text(json.dumps(dict(lerobot_export=dict(enabled=False), video_record=dict(enabled=True))))   # the installed runtime's
        other = tmp_path / 'other-runtime'
        other.mkdir()
        (other / 'config.json').write_text(json.dumps(dict(lerobot_export=dict(enabled=True))))
        own = dict(robot='testbot', profile='yam', manifest=str(other), station=r.srv.url)        # no opt-out in it
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **r.kw(config=own, runtime=other))
        assert result.status == 'done'
        assert result.summary['dataset']['recorded'] is False, result.summary['dataset']
        assert not r.dataset.exists() or not any(r.dataset.rglob('*'))
        (r.runtime / 'config.json').write_text(json.dumps(dict(video_record=dict(enabled=True))))   # robot.json decides then
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **r.kw(config=own, runtime=other))
        assert result.summary['dataset'] == dict(recorded=False, why=robot_wants_why(cfg))
        assert not r.dataset.exists() or not any(r.dataset.rglob('*'))
    finally:
        r.close()


def robot_wants_why(cfg):
    from roborun_sdk.policy.record import robot_wants_dataset
    return robot_wants_dataset(cfg)[1]


def test_a_caller_s_own_config_never_makes_a_simulated_robot_s_run_real(tmp_path, home, monkeypatch):
    r = Rig(tmp_path)
    try:
        install(monkeypatch, r, sim='cup')                 # the installed robot is a simulation
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **r.kw())   # config= without sim
        assert result.summary['dataset'] == dict(recorded=False, why='simulated robot')
    finally:
        r.close()


# ---------------------------------------------------------------- the robot's own recorder setting (config.json video_record)
def _recording_dir(r, task_id):
    return r.runtime / 'log' / 'video-motion' / task_id / f'recording-{task_id}-01'


@pytest.mark.parametrize('video_record', [None, dict(enabled=False), dict(enabled='yes'), dict(enabled=True, cameras=['x'])])
def test_a_robot_that_does_not_record_its_tasks_records_no_policy_run(tmp_path, home, monkeypatch, video_record):
    """config.json video_record not enabled (absent, false, not the JSON value true, or invalid): as an ordinary task, the run
    leaves no recording and no dataset episode, and its record says why."""
    r = Rig(tmp_path)
    try:
        install(monkeypatch, r)
        (r.runtime / 'config.json').write_text(json.dumps({} if video_record is None else dict(video_record=video_record)))
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **r.kw())
        assert result.status == 'done', result
        assert not (r.runtime / 'log' / 'video-motion' / result.task_id).exists()
        assert result.summary['recording']['recorded'] is False and 'video_record' in result.summary['recording']['why']
        assert result.summary['dataset'] == dict(recorded=False, why='no recording')
        rec = json.loads((r.runtime / 'log' / 'tasks' / f'{result.task_id}.json').read_text())
        assert rec['recording']['recorded'] is False and rec['dataset'] == dict(recorded=False, why='no recording')
        assert not r.dataset.exists() or not any(r.dataset.rglob('*'))
    finally:
        r.close()


def test_a_caller_s_own_runtime_never_turns_the_robot_s_recorder_on(tmp_path, home, monkeypatch):
    r = Rig(tmp_path)
    try:
        install(monkeypatch, r)
        (r.runtime / 'config.json').write_text(json.dumps(dict(video_record=dict(enabled=False))))   # the installed runtime's
        other = tmp_path / 'other-runtime'
        other.mkdir()
        (other / 'config.json').write_text(json.dumps(dict(video_record=dict(enabled=True))))
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **r.kw(runtime=other))
        assert result.summary['recording']['recorded'] is False
        assert not (other / 'log' / 'video-motion' / result.task_id).exists()
    finally:
        r.close()


def test_only_the_cameras_the_robot_records_are_recorded(tmp_path, home, monkeypatch):
    r = Rig(tmp_path, cameras=('top', 'left'))
    try:
        install(monkeypatch, r)
        (r.runtime / 'config.json').write_text(json.dumps(dict(video_record=dict(enabled=True, cameras=['top']))))
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **r.kw())
        rdir = _recording_dir(r, result.task_id)
        assert (rdir / 'top.mjpg').exists() and not (rdir / 'left.mjpg').exists()
        assert {json.loads(l)['cam'] for l in (rdir / 'frames.jsonl').read_text().splitlines()} == {'top'}
        assert json.loads((rdir / 'recording.json').read_text())['config']['cameras'] == ['top']
    finally:
        r.close()


@pytest.mark.parametrize('caps, word', [(dict(min_free_mb=99999999.0), 'min_free_mb'), (dict(day_cap_mb=1.0), 'day_cap_mb')])
def test_the_recorder_s_disk_caps_hold_for_a_policy_run(tmp_path, home, monkeypatch, caps, word):
    """Too little free disk, or today's recordings at the day cap: the run moves, records nothing and says which cap.
    Another recording of today (1.5 MB) is what fills the caps here; as with the runtime's recorder, these caps delete
    nothing."""
    r = Rig(tmp_path)
    try:
        install(monkeypatch, r)
        old = r.runtime / 'log' / 'video-motion' / ('a' * 32) / f'recording-{"a" * 32}-01'
        old.mkdir(parents=True)
        import datetime
        (old / 'recording.json').write_text(json.dumps(dict(schema='manifest-recording/1', state='ended',
                                                            created_local_date=datetime.date.today().isoformat())))
        (old / 'top.mjpg').write_bytes(b'\0' * (1536 * 1024))
        (r.runtime / 'config.json').write_text(json.dumps(dict(video_record=dict(enabled=True, **caps))))
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **r.kw())
        assert result.status == 'done', result
        assert result.summary['recording']['recorded'] is False and word in result.summary['recording']['why']
        assert result.summary['dataset'] == dict(recorded=False, why='no recording')
        assert not (r.runtime / 'log' / 'video-motion' / result.task_id).exists() and (old / 'top.mjpg').exists()
    finally:
        r.close()


# ---------------------------------------------------------------- retention: the same cleanup as an ordinary task's recording
def _old_recording(root, clip, created, size, today=False, schema='manifest-recording/1', name=None):
    import datetime
    rec = root / clip / (name or f'recording-{clip}-01')
    rec.mkdir(parents=True)
    date = datetime.date.today() if today else datetime.date.today() - datetime.timedelta(days=3)
    (rec / 'recording.json').write_text(json.dumps(dict(schema=schema, state='ended', created_at=created,
                                                        created_local_date=date.isoformat())))
    (rec / 'top.mjpg').write_bytes(b'\0' * size)
    return rec


def _small_mb(monkeypatch):
    from roborun_sdk.policy import record, runner
    monkeypatch.setattr(record, 'MB', 1024)               # the caps read as KB here
    monkeypatch.setattr(runner, 'MB', 1024)


def test_older_recordings_make_room_for_a_policy_run_as_for_every_task(tmp_path, home, monkeypatch):
    """The recordings at total_cap_mb: as the runtime's recorder does before an ordinary task's recording, the oldest go
    until there is room for run_cap_mb, and the run records. Only a recorder's own folders are touched: not a folder
    without a recording.json of the schema, not a symlink, not anything else in a clip folder. The record names what went."""
    _small_mb(monkeypatch)
    r = Rig(tmp_path)
    try:
        install(monkeypatch, r)
        root = r.runtime / 'log' / 'video-motion'
        oldest = _old_recording(root, 'a' * 32, '2026-01-01T00:00:00.000+00:00', 6 * 1024)
        older = _old_recording(root, 'b' * 32, '2026-01-02T00:00:00.000+00:00', 6 * 1024)
        newer = _old_recording(root, 'c' * 32, '2026-01-03T00:00:00.000+00:00', 6 * 1024)
        foreign = _old_recording(root, 'd' * 32, '2025-01-01T00:00:00.000+00:00', 6 * 1024, schema='something-else')
        (root / ('a' * 32) / 'source.npz').write_bytes(b'x')         # the clip's other files stay
        elsewhere = _old_recording(tmp_path / 'elsewhere', 'e' * 32, '2024-01-01T00:00:00.000+00:00', 6 * 1024)
        (root / ('e' * 32)).symlink_to(elsewhere.parent, target_is_directory=True)
        (r.runtime / 'config.json').write_text(json.dumps(dict(video_record=dict(
            enabled=True, run_cap_mb=8.0, total_cap_mb=20.0, day_cap_mb=1000.0, min_free_mb=0.0))))
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **r.kw())
        assert result.status == 'done', result
        # 18 KB + 8 KB > 20 KB: the oldest (a) and the next (b) go; 6 + 8 <= 20 then, so c stays
        assert not oldest.exists() and not older.exists() and newer.exists()
        assert (root / ('a' * 32) / 'source.npz').exists()
        assert foreign.exists() and elsewhere.exists() and (root / ('e' * 32)).is_symlink()
        rdir = _recording_dir(r, result.task_id)
        assert (rdir / 'top.mjpg').exists() and result.summary['recording'].get('recorded') is not False
        assert [c['dir'] for c in result.summary['recording']['cleanup']] == [
            f'{"a" * 32}/recording-{"a" * 32}-01', f'{"b" * 32}/recording-{"b" * 32}-01']
        rec = json.loads((r.runtime / 'log' / 'tasks' / f'{result.task_id}.json').read_text())
        assert [c['dir'] for c in rec['recording']['cleanup']] == [c['dir'] for c in result.summary['recording']['cleanup']]
        assert json.loads((rdir / 'recording.json').read_text())['cleanup'][0]['dir'].startswith('a' * 32)
    finally:
        r.close()


def test_policy_runs_never_stop_recording_as_their_recordings_pile_up(tmp_path, home, monkeypatch):
    """Run after run, each policy run's recording makes room for the next: past total_cap_mb they still record."""
    _small_mb(monkeypatch)
    r = Rig(tmp_path)
    try:
        install(monkeypatch, r)
        (r.runtime / 'config.json').write_text(json.dumps(dict(video_record=dict(
            enabled=True, run_cap_mb=4.0, total_cap_mb=10.0, day_cap_mb=1000.0, min_free_mb=0.0))))
        ids = []
        for _ in range(4):
            result = policy.run(Sway(), prompt='p', arms='left', seconds=0.8, **r.kw())
            assert result.status == 'done', result
            assert result.summary['recording'].get('recorded') is not False, result.summary['recording']
            ids.append(result.task_id)
        assert _recording_dir(r, ids[-1]).exists() and not _recording_dir(r, ids[0]).exists()
    finally:
        r.close()


def test_with_no_room_even_after_the_cleanup_the_record_says_so(tmp_path, home, monkeypatch):
    r = Rig(tmp_path)
    try:
        install(monkeypatch, r)
        root = r.runtime / 'log' / 'video-motion'
        old = _old_recording(root, 'a' * 32, '2026-01-01T00:00:00.000+00:00', 1024)
        (r.runtime / 'config.json').write_text(json.dumps(dict(video_record=dict(
            enabled=True, run_cap_mb=2.0, total_cap_mb=1.0, min_free_mb=0.0))))
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **r.kw())
        assert result.status == 'done', result
        assert not old.exists()                              # deleted trying, as the runtime's recorder does
        why = result.summary['recording']['why']
        assert result.summary['recording']['recorded'] is False and 'total_cap_mb' in why and 'no room' in why
        rec = json.loads((r.runtime / 'log' / 'tasks' / f'{result.task_id}.json').read_text())
        assert rec['recording']['why'] == why and rec['recording']['cleanup']
        assert not (root / result.task_id).exists()
    finally:
        r.close()


def test_the_installed_runtime_s_own_cleanup_decides(tmp_path, home, monkeypatch):
    """When the installed runtime has its recorder (workspaces/yam/policies/video_record.py), its own `_budget` makes the
    room, as for an ordinary task; nothing of it is written into the runtime (no bytecode)."""
    r = Rig(tmp_path)
    try:
        install(monkeypatch, r)
        here = r.runtime / 'workspaces' / 'yam' / 'policies'
        here.mkdir(parents=True)
        calls = tmp_path / 'calls.jsonl'
        (here / 'video_record.py').write_text(
            'import json, os, sys\n'
            'def _budget(cfg, here, root):\n'
            f'    open({str(calls)!r}, "a").write(json.dumps([cfg, here, root]) + "\\n")\n'
            '    print("[video_record] cleanup: deleted x", file=sys.stderr)\n'
            '    return "total", 0, [dict(dir="x/recording-x-01", created_at="", bytes=5, freed=5)]\n')
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **r.kw())
        cfg, rdir, root = json.loads(calls.read_text().splitlines()[0])
        assert cfg == dict(min_free_mb=2000.0, day_cap_mb=8000.0, total_cap_mb=10000.0, run_cap_mb=500.0)
        assert rdir == str(_recording_dir(r, result.task_id)) and root == str(r.runtime / 'log' / 'video-motion')
        assert result.summary['recording']['recorded'] is False and 'total_cap_mb' in result.summary['recording']['why']
        assert result.summary['recording']['cleanup'][0]['dir'] == 'x/recording-x-01'
        assert not (here / '__pycache__').exists()
    finally:
        r.close()


def test_the_cleanup_matches_the_runtime_s_recorder(tmp_path):
    """Parity with the runtime's own recorder file, when one is given (ROBORUN_RUNTIME_VIDEO_RECORD): the same tree, the
    same caps, the same recordings deleted and the same answer."""
    src = os.environ.get('ROBORUN_RUNTIME_VIDEO_RECORD')
    if not src or not Path(src).is_file():
        pytest.skip('ROBORUN_RUNTIME_VIDEO_RECORD names no runtime recorder file')
    from roborun_sdk.policy import record
    rt = tmp_path / 'rt'
    (rt / 'workspaces' / 'yam' / 'policies').mkdir(parents=True)
    (rt / 'workspaces' / 'yam' / 'policies' / 'video_record.py').write_text(Path(src).read_text())
    fn = record.runtime_budget(rt)
    assert fn is not None
    mb = 1024 * 1024
    for caps in (dict(min_free_mb=0.0, day_cap_mb=8000.0, total_cap_mb=10.0, run_cap_mb=3.0),
                 dict(min_free_mb=0.0, day_cap_mb=2.0, total_cap_mb=10.0, run_cap_mb=3.0),
                 dict(min_free_mb=0.0, day_cap_mb=8000.0, total_cap_mb=2.0, run_cap_mb=3.0)):
        outs = []
        for which in ('runtime', 'sdk'):
            root = tmp_path / which / 'video-motion'
            for i, clip in enumerate('abcd'):
                _old_recording(root, clip * 32, f'2026-01-0{i + 1}T00:00:00.000+00:00', 2 * mb, today=i >= 2)
            _old_recording(root, 'f' * 32, '2020-01-01T00:00:00.000+00:00', 2 * mb, schema='x')
            here = root / ('9' * 32) / f'recording-{"9" * 32}-01'
            here.mkdir(parents=True)
            got = fn(dict(caps), str(here), str(root)) if which == 'runtime' else record._budget(dict(caps), str(here),
                                                                                                  str(root))
            left = sorted(str(p.relative_to(root)) for p in root.glob('*/recording-*'))
            outs.append((got[0], got[1], [(c['dir'], c['bytes']) for c in got[2]], left))
            import shutil
            shutil.rmtree(tmp_path / which)
        assert outs[0] == outs[1], caps


def test_a_policy_run_s_recording_that_reaches_the_day_cap_says_so(tmp_path, home, monkeypatch):
    """The day cap is held during the run too: the frames stop, the run goes on, and the task record says why (while the
    run goes on, and at its end)."""
    _small_mb(monkeypatch)
    r = Rig(tmp_path)
    try:
        install(monkeypatch, r)
        root = r.runtime / 'log' / 'video-motion'
        _old_recording(root, 'a' * 32, '2026-01-01T00:00:00.000+00:00', 6 * 1024, today=True)
        (r.runtime / 'config.json').write_text(json.dumps(dict(video_record=dict(
            enabled=True, run_cap_mb=100.0, total_cap_mb=1000.0, day_cap_mb=8.0, min_free_mb=0.0))))
        seen = []
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.5, on_event=seen.append, **r.kw())
        assert result.status == 'done', result
        assert (_recording_dir(r, result.task_id) / 'top.mjpg').stat().st_size <= 2 * 1024
        assert result.summary['recording']['capped'] == 'day_cap_mb' and result.summary['recording']['poses'] > 20
        rec = json.loads((r.runtime / 'log' / 'tasks' / f'{result.task_id}.json').read_text())
        assert rec['recording']['capped'] == 'day_cap_mb' and 'day_cap_mb' in rec['recording']['why']
        assert any(e.get('event') == 'policy_notice' and 'recording stopped' in e.get('text', '') for e in seen)
    finally:
        r.close()


def test_a_full_disk_ends_the_frames_and_the_record_says_so(tmp_path, home, monkeypatch):
    import errno
    from roborun_sdk.policy import record
    real = record.append_line

    def full(path, data):
        if Path(path).name == 'frames.jsonl':
            raise OSError(errno.ENOSPC, 'No space left on device')
        return real(path, data)
    monkeypatch.setattr(record, 'append_line', full)
    r = Rig(tmp_path)
    try:
        install(monkeypatch, r)
        (r.runtime / 'config.json').write_text(json.dumps(dict(video_record=dict(enabled=True, min_free_mb=0.0))))
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **r.kw())
        assert result.status == 'done', result
        assert result.summary['recording']['capped'] == 'disk_full'
        rec = json.loads((r.runtime / 'log' / 'tasks' / f'{result.task_id}.json').read_text())
        assert rec['recording']['capped'] == 'disk_full' and 'disk is full' in rec['recording']['why']
    finally:
        r.close()


def test_a_policy_run_s_frames_stop_at_the_run_cap(tmp_path, home, monkeypatch):
    from roborun_sdk.policy import record
    monkeypatch.setattr(record, 'MB', 1024)               # run_cap_mb 1 reads as 1 KB here: a frame or two
    from roborun_sdk.policy import runner
    monkeypatch.setattr(runner, 'MB', 1024)
    r = Rig(tmp_path)
    try:
        install(monkeypatch, r)
        (r.runtime / 'config.json').write_text(json.dumps(dict(video_record=dict(enabled=True, run_cap_mb=8.0,
                                                                                  min_free_mb=0.0))))
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.5, **r.kw())
        rdir = _recording_dir(r, result.task_id)
        assert (rdir / 'top.mjpg').stat().st_size <= 8 * 1024
        assert result.summary['recording']['capped'] == 'run_cap_mb'
        assert result.summary['recording']['poses'] > 20                 # the joints go on
    finally:
        r.close()


def test_a_caller_s_own_config_never_names_another_robot_s_dataset(tmp_path, home, monkeypatch):
    from roborun_sdk import dataset
    if dataset.unavailable():
        pytest.skip(dataset.unavailable())
    r = Rig(tmp_path)
    try:
        install(monkeypatch, r)                            # the installed robot is testbot
        kw = r.kw(config=dict(r.config, robot='someone-else'))
        kw.pop('dataset_root')
        result = policy.run(Sway(), prompt='p', arms='left', seconds=1.0, **kw)
        assert result.summary['dataset']['recorded'] is True, result.summary['dataset']
        assert '/testbot/' in result.summary['dataset']['staged'] and 'someone-else' not in result.summary['dataset']['staged']
    finally:
        r.close()


def test_the_simulator_s_cameras_are_the_policy_s_cameras(tmp_path):
    """roborun sim's camera broker (its status now has available_roles and cards, but no per-camera state) gives a policy
    run every camera it renders, as the robot's broker does for its streaming ones."""
    from types import SimpleNamespace
    from roborun_sdk import mujoco_sim
    from roborun_sdk.policy.station import Cameras
    world = SimpleNamespace(cameras={'top': dict(depth=True), 'left': dict(depth=False), 'right': dict(depth=True)},
                            card=lambda role: dict(id=role), objects_state=lambda: {})
    sim = mujoco_sim.Cameras(world, tmp_path / 'c.sock')
    cams = Cameras(tmp_path / 'c.sock')
    cams._client = SimpleNamespace(status=sim.status)
    assert cams.roles() == ['top', 'left', 'right']
    world.cameras.pop('left')
    assert cams.roles() == ['top', 'right']
