"""A policy run whose process is ended from outside: SIGTERM and SIGHUP close it like Ctrl-C; SIGKILL leaves a record that
`roborun policy run`, `roborun policy runs` and `roborun stop` close as failed (runner_gone), with its recording ended."""
import json
import os
import signal
import subprocess
import sys
import textwrap
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from fake_policy_bits import own_home
from test_policy_runner import HERE, Rig, actions  # noqa: F401

from roborun_sdk.policy import record as rec_mod
from roborun_sdk.policy.wording import roborun_cause

pytestmark = pytest.mark.skipif(os.name == 'nt', reason='POSIX signals')


@pytest.fixture
def home(tmp_path, monkeypatch):
    return own_home(tmp_path, monkeypatch)              # never this computer's robot.json, runtime or $HOME


@pytest.fixture
def rig(tmp_path, home):
    r = Rig(tmp_path)
    yield r
    r.close()


def task(rig, run_id):
    return json.loads((rig.runtime / 'log' / 'tasks' / f'{run_id}.json').read_text())


def recording(rig, run_id):
    return json.loads(rec_mod.Paths(rig.runtime, run_id).recording.joinpath('recording.json').read_text())


def start_cli_run(rig, seconds=30):
    """`roborun policy run` in its own process against the rig's station; returns (process, run id once leased)."""
    cfg = rig.tmp / 'robot.json'
    cfg.write_text(json.dumps(dict(rig.config, manifest=str(rig.runtime))))
    code = textwrap.dedent(f'''
        import sys
        from pathlib import Path
        from roborun_sdk import cli, robot
        robot.CONFIG = Path({str(cfg)!r})
        sys.exit(cli.main(['policy', 'run', '--provider', {str(HERE / "fake_policy_bits.py") + ":FileProvider"!r},
                           '--arms', 'left', '--seconds', '{seconds}', '--prompt', 'p', '--json', '--no-dataset']))
    ''')
    env = dict(os.environ, MANIFEST_CAMERA_SOCKET=rig.sock, PYTHONPATH=os.pathsep.join(sys.path))
    proc = subprocess.Popen([sys.executable, '-c', code], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        lease = rig.station.lease
        if lease and lease.get('active') and (lease.get('policy') or {}).get('chunks'):
            return proc, lease['run_id']
        assert proc.poll() is None, proc.communicate()
        time.sleep(0.05)
    proc.kill()
    raise AssertionError('the run never leased')


@pytest.mark.parametrize('sig', ['SIGTERM', 'SIGHUP'])
def test_sigterm_and_sighup_close_the_run_like_ctrl_c(rig, sig):
    proc, run_id = start_cli_run(rig)
    proc.send_signal(getattr(signal, sig))
    out, err = proc.communicate(timeout=20)
    assert proc.returncode == 0, (out, err)
    assert rig.station.lease['reason'] != 'client_disconnected'           # the runner ended its lease itself
    rec = task(rig, run_id)
    assert rec['status'] == 'stopped' and rec['stop_reason'] == 'signal' and sig in rec['final'], rec
    assert recording(rig, run_id)['state'] == 'ended'
    assert json.loads(out.splitlines()[-1])['status'] == 'stopped'


def test_a_killed_run_is_closed_as_failed_by_policy_runs_and_its_recording_ended(rig):
    from roborun_sdk.policy.cli import runs
    proc, run_id = start_cli_run(rig)
    proc.kill()
    proc.wait(10)
    rec = task(rig, run_id)
    assert rec['status'] == 'running' and rec['runner']['pid'] == proc.pid
    assert recording(rig, run_id)['state'] == 'recording'
    rows = runs(5, runtime=rig.runtime)
    assert rows[0]['id'] == run_id and rows[0]['status'] == 'failed' and rows[0]['stop_reason'] == 'runner_gone'
    rec = task(rig, run_id)
    assert rec['status'] == 'failed' and 'killed or crashed' in rec['error'] and rec.get('ended')
    assert roborun_cause(rec['error']) is None and roborun_cause(rec['final']) is None
    meta = recording(rig, run_id)
    assert meta['state'] == 'ended' and meta['stop_reason'] == 'runner_gone'


def _stale(rig, run_id, **runner):
    paths = rec_mod.Paths(rig.runtime, run_id)
    rec = rec_mod.TaskRecord(paths, prompt='p', provider='sway', model=None, policy={})
    rec.record['runner'] = dict(rec.record['runner'], **runner)
    rec.update(force=True)
    paths.recording.mkdir(parents=True, exist_ok=True)
    rec_mod.write_json(paths.recording / 'recording.json', dict(schema=rec_mod.SCHEMA, state='recording', task_id=run_id))
    return paths


def dead_pid():
    p = subprocess.Popen([sys.executable, '-c', 'pass'])
    p.wait()
    return p.pid


def test_close_orphans_closes_only_records_whose_process_is_gone(rig):
    dead, other_boot, reused, alive, legacy = 'a' * 32, 'b' * 32, 'c' * 32, 'd' * 32, 'e' * 32
    _stale(rig, dead, pid=dead_pid())
    _stale(rig, other_boot, boot_id='0' * 16)
    _stale(rig, reused, process_start=-1)                  # this pid, but another process
    _stale(rig, alive)                                      # this very process
    legacy_paths = _stale(rig, legacy)
    data = task(rig, legacy)
    data.pop('runner')
    legacy_paths.task.write_text(json.dumps(data))
    closed = rec_mod.close_orphans(rig.runtime)
    assert sorted(closed) == sorted([dead, other_boot, reused])
    for rid in closed:
        assert task(rig, rid)['status'] == 'failed' and recording(rig, rid)['state'] == 'ended'
    for rid in (alive, legacy):
        assert task(rig, rid)['status'] == 'running' and recording(rig, rid)['state'] == 'recording'


def test_roborun_stop_and_a_new_run_close_a_killed_run(rig, monkeypatch, capsys):
    from roborun_sdk import cli, policy, robot
    from fake_policy_bits import Sway
    first, second = 'f' * 32, '1' * 32
    _stale(rig, first, pid=dead_pid())
    cfg = rig.tmp / 'robot.json'
    cfg.write_text(json.dumps(dict(rig.config, manifest=str(rig.runtime), backend='http://127.0.0.1:9')))
    monkeypatch.setattr(robot, 'CONFIG', cfg)
    assert cli._stop_this_robot(SimpleNamespace(json=False)) == 0
    assert f'closed 1 policy run whose process was gone: {first[:12]}' in capsys.readouterr().out
    assert task(rig, first)['status'] == 'failed'
    time.sleep(0.8)                                         # the station's Stop latch
    _stale(rig, second, pid=dead_pid())
    result = policy.run(Sway(), prompt='p', arms='left', seconds=0.6, **rig.kw())
    assert result.status == 'done' and task(rig, second)['status'] == 'failed'
