"""The suite's guard (conftest._policy_runtime_in_tmp): a test that reaches a policy runtime outside pytest's temp tree is
caught, also when the touch happens where `roborun stop` swallows errors; one under tmp_path is not."""
import json
from types import SimpleNamespace

from roborun_sdk import cli, robot
from roborun_sdk.policy import control, record


def test_a_runtime_outside_the_temp_tree_is_caught(_policy_runtime_in_tmp, tmp_path):
    outside = '/nonexistent-roborun-guard-check/runtime'
    record.close_orphans(outside)
    record.Paths(outside, 'a' * 32)
    control.running(outside)
    assert [w for w, _ in _policy_runtime_in_tmp] == ['close_orphans', 'Paths', 'close_orphans']
    _policy_runtime_in_tmp.clear()                       # seen on purpose: this test must not fail at teardown
    record.close_orphans(tmp_path / 'rt')
    record.Paths(tmp_path / 'rt', 'a' * 32)
    assert _policy_runtime_in_tmp == []


def test_roborun_stop_through_a_robot_json_without_a_runtime_is_caught_when_home_is_not_the_tests(
        _policy_runtime_in_tmp, tmp_path, monkeypatch):
    """What a Stop test that patches only robot.CONFIG does: the default runtime under $HOME, outside the temp tree."""
    monkeypatch.setenv('HOME', '/nonexistent-roborun-guard-check')
    monkeypatch.delenv('ROBORUN_HOME', raising=False)
    monkeypatch.delenv('MANIFEST_DREAMAPI', raising=False)
    cfg = tmp_path / 'robot.json'
    cfg.write_text(json.dumps(dict(backend='http://127.0.0.1:9', station='http://127.0.0.1:9')))
    monkeypatch.setattr(robot, 'CONFIG', cfg)
    monkeypatch.setattr(cli, '_post_json', lambda *a, **k: (_ for _ in ()).throw(OSError('no backend')))
    monkeypatch.setattr(cli, '_stop_station', lambda config: ('not running', None))
    cli._stop_this_robot(SimpleNamespace(json=True))
    assert _policy_runtime_in_tmp == [('close_orphans', '/nonexistent-roborun-guard-check/roborun/robot/runtime')]
    _policy_runtime_in_tmp.clear()
