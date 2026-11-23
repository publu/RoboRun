"""The motion every real arm shares: nothing moves at connect, motion is smooth, faults latch."""
import time

import pytest

from roborun_sdk.arms.base import ArmHardwareError, JerkLimiter, StreamedArm


class Bench(StreamedArm):
    limits = [(-3, 3)] * 6
    def __init__(self, **kw):
        super().__init__(**kw); self.q = [0.1] * 6; self.jaw = 0.5; self.writes = []; self.limped = 0; self.fail = False
    def open_hardware(self): pass
    def read_hardware(self):
        if self.fail: raise OSError('bus gone')
        return dict(positions_rad=list(self.q), gripper=self.jaw)
    def write_hardware(self, q, jaw): self.writes.append(list(q)); self.q = list(q); self.jaw = jaw
    def limp_hardware(self): self.limped += 1


def wait(fn, s=3):
    end = time.monotonic() + s
    while time.monotonic() < end and not fn(): time.sleep(.01)
    assert fn()


def test_limiter_bounds_speed_accel_and_jerk_and_arrives():
    s = JerkLimiter([0.0], speed=.6, accel=2, jerk=20); dt = .01; last_v = last_a = 0
    for _ in range(600):
        q = s.step([1.0], dt)[0]; v = s.v[0]; a = (v - last_v) / dt
        assert abs(v) <= .6 + 1e-9 and abs(a) <= 2 + 1e-6 and abs(a - last_a) / dt <= 20 + 1e-3 or q == 1.0
        last_v, last_a = v, a
    assert q == pytest.approx(1.0, abs=1e-3)


def test_connect_does_not_move_the_arm():
    arm = Bench(); arm.connect(); time.sleep(.1); arm.close()
    assert all(w == pytest.approx([.1] * 6) for w in arm.writes)


def test_a_far_command_is_approached_smoothly_and_limits_apply():
    arm = Bench(); arm.connect(); arm.command([9.0] * 6, 1.0); time.sleep(.3); arm.close()
    steps = [abs(b[0] - a[0]) for a, b in zip(arm.writes, arm.writes[1:])]
    assert 0 < max(steps) <= arm.joint_speed_rad_s / arm.rate_hz + 1e-9
    assert arm._target == [3.0] * 6


def test_silence_ends_in_a_hold():
    arm = Bench(); arm.command_timeout_s = .05; arm.connect(); arm.command([1.0] * 6, .5); time.sleep(.5)
    a = arm.q[0]; time.sleep(.3); assert arm.q[0] == pytest.approx(a, abs=1e-3) and a < .9; arm.close()


def test_a_fault_latches_and_refuses():
    arm = Bench(); arm.connect(); arm.fail = True
    wait(lambda: arm._error is not None)
    with pytest.raises(ArmHardwareError): arm.read()
    with pytest.raises(ArmHardwareError): arm.command([0] * 6, 0)
    arm.close()


def test_release_then_command_resumes_from_the_measured_pose():
    arm = Bench(); arm.connect(); arm.release(); assert arm.limped == 1
    arm.q = [.5] * 6; time.sleep(.05); n = len(arm.writes)
    arm.command([.5] * 6, .5); wait(lambda: len(arm.writes) > n); arm.close()
    assert arm.writes[n] == pytest.approx([.5] * 6, abs=1e-6)


def test_motion_never_passes_a_limit_even_when_braking_late():
    s = JerkLimiter([0.0], speed=.6, accel=2, jerk=20, limits=[(-1, .2)])
    assert max(s.step([.2], .01)[0] for _ in range(400)) <= .2


def test_an_arm_starting_outside_a_limit_does_not_jump_and_may_only_come_back():
    s = JerkLimiter([.5], speed=.6, accel=2, jerk=20, limits=[(-1, .2)])
    assert s.step([.9], .01)[0] == pytest.approx(.5)
    seen = [s.step([.2], .01)[0] for _ in range(300)]
    assert all(b <= a + 1e-12 for a, b in zip(seen, seen[1:]) if a > .2) and abs(seen[1] - seen[0]) < .01
    assert min(seen) > .2 - .005 and seen[-1] == pytest.approx(.2, abs=1e-3)


def test_a_command_of_the_wrong_length_is_refused_not_truncated():
    arm = Bench(); arm.limits = [(-3, 3)] * 7; arm.q = [.1] * 7; arm.connect()
    with pytest.raises(ArmHardwareError, match='6 joints, this arm has 7'): arm.command([0.] * 6, 0)
    arm.close()
    short = Bench(); short.limits = [(-3, 3)] * 7
    with pytest.raises(ArmHardwareError, match='reports 6 joints'): short.connect()


def _run(limiter, target, seconds, dt=.01):
    rows = []
    for _ in range(int(seconds / dt)):
        limiter.step(target, dt); rows.append((list(limiter.q), list(limiter.v), list(limiter.a)))
    return rows


@pytest.mark.parametrize('speed', [.1, .3, .6])
def test_a_stop_brakes_every_joint_to_rest_quickly_and_smoothly(speed):
    s = JerkLimiter([0.0, 1.0], speed=speed, accel=2, jerk=20)
    _run(s, [5.0, -5.0], 1.0)                                     # cruising, one joint each way
    assert abs(s.v[0]) == pytest.approx(speed, abs=1e-3)
    q, v = list(s.q), list(s.v)
    end = s.brake(4, 60)
    assert all((e - a) * b > 0 for e, a, b in zip(end, q, v))      # it stops ahead of where it was, never turns back
    rows, dt = _run(s, end, .3), .01
    speeds = [abs(r[1][0]) for r in rows]
    assert not s.braking and s.v == [0.0, 0.0] and s.q == pytest.approx(end)
    assert all(b <= a + 1e-9 for a, b in zip(speeds, speeds[1:]))  # slowing down all the way
    settled = next(i for i, r in enumerate(rows) if r[1][0] == 0.0) * dt
    assert settled <= max(1.5 * speed / 4, (6 * speed / 60) ** .5) + 2 * dt
    accels = [r[2][0] for r in rows]
    assert max(map(abs, accels)) <= 4 + 1e-6
    assert max(abs(b - a) / dt for a, b in zip(accels, accels[1:])) <= 60 * 1.05
    assert s.step([s.q[0] + 1, s.q[1]], dt) != end                 # after the stop it follows targets again


def test_a_stop_while_accelerating_has_no_step_in_speed_or_acceleration():
    s = JerkLimiter([0.0], speed=.6, accel=2, jerk=20); _run(s, [5.0], .12); assert s.a[0] > .5
    v, a = s.v[0], s.a[0]; s.brake(4, 60); s.step([5.0], .001)
    assert s.v[0] == pytest.approx(v, abs=.01) and s.a[0] == pytest.approx(a, abs=.5)


def test_a_stop_never_ends_past_a_joint_limit():
    s = JerkLimiter([0.0], speed=.6, accel=2, jerk=20, limits=[(-1, .31)]); _run(s, [.31], .45)
    assert s.brake(4, 60)[0] <= .31 and max(r[0][0] for r in _run(s, [.31], .4)) <= .31


def test_hold_is_prompt_but_the_commands_merely_ending_is_not_a_stop():
    arm = Bench(); arm.connect(); arm.command([3.0] * 6, .5); time.sleep(.4)
    arm.hold(); rest = arm.held_target(); assert arm._shaper.braking
    time.sleep(.35); assert arm.q == pytest.approx(rest, abs=1e-6) and arm._shaper.v == [0.0] * 6; arm.close()
    arm = Bench(); arm.connect(); arm.command([3.0] * 6, .5); time.sleep(.4)
    arm.settle(); assert not arm._shaper.braking and arm.held_target()[0] == pytest.approx(arm._shaper.stopping_point()[0], abs=.03); arm.close()


def test_stop_limits_can_be_set_and_never_go_below_the_motion_limits():
    arm = Bench(stop_accel_rad_s2=1.0, stop_jerk_rad_s3=5.0); arm.connect(); arm.command([3.0] * 6, .5); time.sleep(.4)
    arm.hold(); assert arm._shaper._brake['span'] >= 1.5 * max(map(abs, arm._shaper._brake['v'])) / 2.0 - 1e-9; arm.close()


def test_a_rule_that_keeps_refusing_is_reported_and_one_refusal_is_not():
    arm = Bench(); arm.rule_refused_after_s = .05; answers = [False]
    arm._rule = lambda side, q, jaw, state: answers[0]
    arm.connect(); arm.command([1.0] * 6, .5); assert arm.rule_refusal() is None
    time.sleep(.08); arm.command([1.0] * 6, .5); assert 'rules file refused' in arm.rule_refusal()
    answers[0] = True; arm.command([1.0] * 6, .5); assert arm.rule_refusal() is None; arm.close()


def test_a_stop_brakes_the_shaft_of_a_motor_that_trails_its_command():
    """A first-order motor (rate 10/s: a soft wrist joint): the shaft must be still when the curve ends, with no creeping tail."""
    rate, dt = 10.0, .002
    s = JerkLimiter([0.0], speed=.3, accel=2, jerk=20); shaft = 0.0; v = 0.0
    for _ in range(int(1.5 / dt)):
        c = s.step([5.0], dt)[0]; v = rate * (c - shaft); shaft += v * dt
    end = s.brake(4, 60, [shaft], [v], [1 / rate]); span = s._brake['span']; last = c; steps = []
    for _ in range(int(span / dt) + 1):
        c = s.step(end, dt)[0]; steps.append(c - last); last = c; shaft += rate * (c - shaft) * dt
    assert abs(rate * (c - shaft)) < .03 and abs(c - shaft) < .004                 # stopped with the curve
    assert max(map(abs, steps)) < .3 * dt * 3                                      # and the command never stepped


def test_repeated_stops_of_a_resting_arm_do_not_move_it_whatever_the_sensors_say():
    s = JerkLimiter([.5], speed=.6, accel=2, jerk=20)
    for noise in (.02, -.03, .015):
        assert s.brake(4, 60, [.5 + noise / 10], [noise], [.1]) == [.5] and not s.braking
