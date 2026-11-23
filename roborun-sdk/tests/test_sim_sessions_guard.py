"""roborun sim --sessions never writes into what the robot's history upload reads (its runtime's log/ or robot.json's data
sources): those sessions would be uploaded as real robot data. The simulation's own folder and folders outside the robot's
data stay the user's to choose."""
import json

import pytest

from test_sim_isolation import box, _sim_stubs, args  # noqa: F401  (fixtures and helpers of the isolation suite)
from roborun_sdk import mujoco_sim, robot, sim_home


def _run(monkeypatch, **kw):
    seen = dict(sock=sim_home.camera_socket()); _sim_stubs(monkeypatch, seen)
    return mujoco_sim.cmd_sim(args(**kw)), seen


def test_sessions_in_the_live_runtime_log_are_refused(box, monkeypatch):
    target = box.live / 'log' / 'video-motion' / 'sessions'
    with pytest.raises(SystemExit) as exc:
        _run(monkeypatch, sessions=str(target))
    assert "robot's own data" in str(exc.value) and 'Nothing was started' in str(exc.value)


def test_sessions_in_a_data_sync_source_are_refused(box, monkeypatch, tmp_path):
    custom = tmp_path / 'my-robot-data'; custom.mkdir()
    c = json.loads(robot.CONFIG.read_text()) if robot.CONFIG.exists() else {}
    c.setdefault('data', {})['sources'] = {'video-motion': str(custom)}
    robot.CONFIG.parent.mkdir(parents=True, exist_ok=True); robot.CONFIG.write_text(json.dumps(c))
    with pytest.raises(SystemExit):
        _run(monkeypatch, sessions=str(custom / 'sessions'))


def test_sessions_in_the_simulations_own_folder_are_fine(box, monkeypatch):
    rc, seen = _run(monkeypatch, sessions=str(sim_home.folder() / 'sessions'))
    assert rc == 0 and seen['sessions'] == str(sim_home.folder() / 'sessions')


def test_sessions_elsewhere_are_the_users_choice(box, monkeypatch, tmp_path):
    rc, seen = _run(monkeypatch, sessions=str(tmp_path / 'elsewhere'))
    assert rc == 0 and seen['sessions'] == str(tmp_path / 'elsewhere')


def test_no_sessions_flag_keeps_the_default(box, monkeypatch):
    rc, seen = _run(monkeypatch)
    assert rc == 0 and sim_home.is_copy(seen['sessions'])
