"""The simulator obeys robot.yaml's rules exactly as a real arm: refuse holds, change is obeyed, a failing rule stops the arm."""
import threading
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('mujoco')
from roborun_sdk.arms.base import ArmHardwareError      # noqa: E402
from roborun_sdk.mujoco_sim import MuJoCoArm             # noqa: E402


def world():
    arm = SimpleNamespace(limits=[(-3, 3)] * 6, n_joints=6)
    return SimpleNamespace(rig=SimpleNamespace(arms=dict(left=arm)), q=dict(left=np.full(6, .1)), jaw=dict(left=.5),
                           target=dict(left=np.full(6, .1)), jaw_target=dict(left=.5), lock=threading.Lock())


def rules(tmp_path, body):
    f = tmp_path / 'rules.py'; f.write_text('def check(side, positions_rad, gripper, state):\n' + body)
    return str(f)


def test_a_refusing_rule_holds_the_simulated_arm(tmp_path):
    w = world(); arm = MuJoCoArm(w, 'left', rules(tmp_path, '    return False\n'))
    arm.command([1.] * 6, 0.)
    assert w.target['left'].tolist() == [.1] * 6 and w.jaw_target['left'] == .5


def test_a_changing_rule_is_obeyed(tmp_path):
    w = world(); arm = MuJoCoArm(w, 'left', rules(tmp_path, '    return [min(v, .3) for v in positions_rad], gripper\n'))
    arm.command([1.] * 6, .2)
    assert w.target['left'].tolist() == [.3] * 6 and w.jaw_target['left'] == .2


def test_a_failing_rule_stops_the_simulated_arm_and_says_so(tmp_path):
    w = world(); arm = MuJoCoArm(w, 'left', rules(tmp_path, '    raise RuntimeError("oops")\n'))
    with pytest.raises(ArmHardwareError, match='your rules file failed: RuntimeError: oops'):
        arm.command([1.] * 6, 0.)
    assert w.target['left'].tolist() == [.1] * 6
    with pytest.raises(ArmHardwareError, match='rules file failed'):     # latched: no later command moves it either
        arm.command([.2] * 6, 0.)


def test_no_rules_moves_as_before():
    w = world(); MuJoCoArm(w, 'left').command([1.] * 6, 0.)
    assert w.target['left'].tolist() == [1.] * 6


def test_the_station_hears_why_a_ruled_sim_arm_stopped(tmp_path, monkeypatch):
    w = world(); arm = MuJoCoArm(w, 'left', rules(tmp_path, '    return False\n'))
    clock = [100.]
    monkeypatch.setattr('roborun_sdk.mujoco_sim.time.monotonic', lambda: clock[0])
    arm.command([1.] * 6, 0.); assert arm.rule_refusal() is None                       # one refused step may just be a trim
    clock[0] += 1.; assert arm.rule_refusal() == 'your rules file refused this motion of the left arm'
    broken = MuJoCoArm(world(), 'left', rules(tmp_path, '    raise RuntimeError("oops")\n'))
    with pytest.raises(ArmHardwareError):
        broken.command([1.] * 6, 0.)
    assert broken.health() == dict(ok=False, error='your rules file failed: RuntimeError: oops', kind='rules')
    (tmp_path / 'rules.py').write_text('def check(side, positions_rad, gripper, state):\n    return True\n')   # fixed by its owner
    broken.reopen(); assert broken.health() == dict(ok=True)                            # Reset reads it again
    broken.command([.5] * 6, 0.); assert broken.world.target['left'].tolist() == [.5] * 6
