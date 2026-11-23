"""An arm whose control loop stopped is never reported as holding: `down` and setup tell the person to support it."""
import json
import threading
from types import SimpleNamespace

from roborun_sdk.arms import hardware


class Arm:
    def __init__(self, error=None, alive=True):
        self._error = error
        self._thread = SimpleNamespace(is_alive=lambda: alive)

    def park(self):
        return dict(state='failed', why='ArmHardwareError: the motor loop stopped')

    def let_go(self, force=False):
        return 'it is not at its rest pose, so nothing is known to carry it'


def answer(tmp_path, monkeypatch, do, arms):
    monkeypatch.setattr(hardware, '_control', lambda name: tmp_path / name)
    (tmp_path / 'request').write_text(json.dumps(dict(id='x', do=do)))
    hardware._answer_station(arms, set())
    return json.loads((tmp_path / 'answer').read_text())['held']


def test_a_running_arm_that_refuses_is_held(tmp_path, monkeypatch):
    held = answer(tmp_path, monkeypatch, 'down', dict(left=Arm()))
    assert held['left'].startswith('it is not at its rest pose') and hardware.faulted(held) == []


def test_an_arm_whose_loop_stopped_is_faulted_not_held(tmp_path, monkeypatch):
    dead = Arm(error='RuntimeError: Motor error detected: motor id: 1, error: disabled', alive=False)
    held = answer(tmp_path, monkeypatch, 'park', dict(left=dead))
    assert hardware.faulted(held) == ['left'] and 'may be off' in held['left'] and 'motor loop stopped' in held['left']


def test_a_broken_rules_file_still_holds_the_arm(tmp_path, monkeypatch):
    held = answer(tmp_path, monkeypatch, 'park', dict(left=Arm(error='your rules file failed: RuntimeError: oops')))   # loop alive
    assert hardware.faulted(held) == [] and held['left']


def test_down_says_support_it_for_a_faulted_arm(monkeypatch, capsys):
    from roborun_sdk import robot
    monkeypatch.setattr(robot, '_state', lambda unit: 'running')
    monkeypatch.setattr(robot, '_stop', lambda unit: None)
    monkeypatch.setattr(hardware, 'ask_station', lambda what: {'left': hardware.FAULTED + ' (the motor loop stopped)'})
    assert robot.cmd_down(SimpleNamespace(park=False, release=False)) == 1
    out = capsys.readouterr().out
    assert 'support it now' in out and 'still powered and holding' not in out
