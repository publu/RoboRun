"""The jaw effort an arm reports while gripping: a hold too weak to be seen is refused, and a reduced jaw force reads full."""
from types import SimpleNamespace

import pytest

from roborun_sdk.arms import base, motor_arm, safety


def test_a_hold_too_weak_to_be_seen_is_refused():
    with pytest.raises(ValueError, match='gripper_hold_effort must be at least'):
        base.StreamedArm(gripper_hold_effort=safety.GRIP_HOLD_MIN - .01)


@pytest.mark.parametrize('share', [1.0, .5])
def test_a_reduced_jaw_force_still_reads_full_at_its_own_limit(share):
    jaw = SimpleNamespace(name='jaw', effort_full=2.0, to_fraction=lambda p: .5)
    reading = SimpleNamespace(pos=0., vel=0., effort=2.0 * share, temp=None)
    arm = SimpleNamespace(joints=[], fixed=[], gripper=jaw, gripper_effort=share, _read_states=lambda: {'jaw': reading}, _check_faults=lambda s: None)
    assert motor_arm.MotorArm.read_hardware(arm)['gripper_effort'] == pytest.approx(1.0)
