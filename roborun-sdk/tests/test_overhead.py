"""What `roborun up` and `roborun status` say about the overhead camera."""
import numpy as np
import pytest

from roborun_sdk import overhead


def test_describe_says_what_the_robot_said():
    assert overhead.describe(None) == 'not reported'
    line = overhead.describe(dict(state='needs_attention', message='Connect an RGB-D camera'))
    assert 'NEEDS ATTENTION' in line and 'RGB-D' in line


def test_check_waits_for_a_verdict(monkeypatch):
    states = [dict(state='calibrating', message='x'), dict(state='rough', message='y')]
    monkeypatch.setattr(overhead, 'read', lambda backend: states.pop(0))
    assert overhead.check('http://robot', sleep=lambda s: None)['state'] == 'rough'


def test_improve_moves_nothing_without_a_yes(monkeypatch):
    monkeypatch.setattr(overhead, 'check', lambda *a, **k: dict(state='rough', message='y'))
    monkeypatch.setattr(overhead.urllib.request, 'urlopen', lambda *a, **k: (_ for _ in ()).throw(AssertionError('moved')))
    assert overhead.improve('http://robot', lambda q: False, say=lambda t: None)['state'] == 'rough'


# -- the second pose ------------------------------------------------------------------------------------------------------
def _so101():
    from roborun_sdk import arm_profiles
    from roborun_sdk.arms import catalog
    slot = arm_profiles.rig('so101')['arms']['left']
    return slot, catalog()['so101']['limits'], overhead._Model(arm_profiles.MODELS / 'so101' / 'so101.xml', slot['T_table_base'], 5)


@pytest.mark.parametrize('start', [[0.] * 5, [.4, -.3, .3, 0., 0.], [-1.5, 0., 0., 0., 0.]])
def test_the_second_pose_is_chosen_from_where_the_arm_is(start):
    slot, limits, model = _so101()
    need = overhead.needed_travel(slot)
    target, moved = overhead.choose_pose(start, limits, model, need)
    assert moved >= need and all(lo + .08 <= v <= hi - .08 for v, (lo, hi) in zip(target, limits))
    assert target != start and max(abs(a - b) for a, b in zip(target, start)) <= 1.1
    assert model.at(target)[0][2] >= min(model.at(start)[0][2], .08)


def test_no_second_pose_is_offered_when_the_limits_leave_no_room():
    slot, limits, model = _so101()
    tight = [(-.05, .05)] * 5
    assert overhead.choose_pose([0.] * 5, tight, model, overhead.needed_travel(slot)) is None


def test_the_other_arm_is_kept_clear():
    slot, limits, model = _so101()
    q = [0.] * 5
    near = np.array([model.at(overhead.choose_pose(q, limits, model, .16)[0])[0]])
    found = overhead.choose_pose(q, limits, model, .16, avoid=near)
    assert found is None or np.linalg.norm(model.at(found[0])[0] - near[0]) >= overhead.CLEARANCE_M


def _station_on_a_clock():
    from roborun_station import FakeYamStation
    from roborun_station.core import SimClock
    clock = SimClock()
    station = FakeYamStation(clock=clock)
    station.advance(.2)
    return station


def test_the_station_runs_one_smooth_curve_and_stops_at_the_goal():
    s = _station_on_a_clock()
    start = list(s.left.q)
    target = [start[0] + .4] + start[1:]
    seen = s.observe()
    lease = s.action(dict(observation_id=seen['observation_id'], seconds=6., reference=dict(kind='setup_pose', arm='left', target_rad=target)))['lease']['id']
    path = []
    for _ in range(int(6 * 30)):
        s.heartbeat(dict(lease_id=lease)); s.advance(1 / 30); path.append(s.left.q_cmd[0])
        if not s.lease['active']:
            break
    assert s.lease['reason'] == 'action_goal_reached' and abs(s.left.q[0] - target[0]) < .02
    steps = np.diff(path) * 30
    assert steps.max() <= .27 and (steps >= -1e-9).all() and abs(steps[0]) < .02            # slow, one way, starts from rest


@pytest.mark.parametrize('change', [lambda q: [q[0] + 1.3] + q[1:], lambda q: [99.] + q[1:], lambda q: q[:-1]])
def test_the_station_refuses_a_setup_pose_it_should_not_run(change):
    from roborun_station.yam_station import StationError
    s = _station_on_a_clock()
    seen = s.observe()
    with pytest.raises(StationError):
        s.action(dict(observation_id=seen['observation_id'], seconds=12., reference=dict(kind='setup_pose', arm='left', target_rad=change(list(s.left.q)))))
    assert s.lease is None


def test_a_setup_pose_ends_when_its_client_goes_away():
    s = _station_on_a_clock()
    start = list(s.left.q); target = [start[0] + 1.] + start[1:]
    seen = s.observe()
    s.action(dict(observation_id=seen['observation_id'], seconds=9., reference=dict(kind='setup_pose', arm='left', target_rad=target)))
    s.advance(3.)                                                                           # no heartbeat
    assert not s.lease['active'] and s.lease['reason'] == 'client_disconnected' and abs(s.left.q[0] - target[0]) > .02


def test_improve_never_moves_without_a_yes_and_reports_a_move_that_was_stopped(monkeypatch):
    calls = []
    step = dict(arm='left', q=[0.] * 5, target=[.5] + [0.] * 4, moved_m=.2, need_m=.17, model=None)
    monkeypatch.setattr(overhead, 'check', lambda *a, **k: dict(state='rough', message='y', arm='left'))
    monkeypatch.setattr(overhead, 'read', lambda backend: dict(state='rough', message='y'))
    monkeypatch.setattr(overhead, 'plan', lambda *a: step)
    monkeypatch.setattr(overhead, 'move_to', lambda station, s, sleep=None: calls.append(s) or 'operator_took_control')
    said = []
    assert overhead.improve('http://r', lambda q: False, station=object(), say=said.append)['state'] == 'rough' and not calls
    overhead.improve('http://r', lambda q: True, station=object(), say=said.append)
    assert calls == [step] and any('did not finish' in t for t in said)


def test_without_a_passed_movement_check_only_the_hands_move_it(monkeypatch):
    step = dict(arm='left', q=[0.] * 5, target=[.5] + [0.] * 4, moved_m=.2, need_m=.17, model=None)
    monkeypatch.setattr(overhead, 'check', lambda *a, **k: dict(state='rough', message='y'))
    monkeypatch.setattr(overhead, 'read', lambda backend: dict(state='rough', message='y'))
    monkeypatch.setattr(overhead, 'plan', lambda *a: step)
    monkeypatch.setattr(overhead, 'move_to', lambda *a, **k: pytest.fail('moved'))
    asked = []
    overhead.improve('http://r', lambda q: asked.append(q) or False, station=object(), ask=input, moves_allowed=False, say=lambda t: None)
    assert len(asked) == 1 and 'by hand' in asked[0]


def test_after_a_hand_move_the_arm_is_held_where_the_hands_left_it():
    s = _station_on_a_clock()
    seen = s.observe()
    lease = s.action(dict(observation_id=seen['observation_id'], seconds=60., reference=dict(kind='hand_guide', arm='left')))['lease']['id']
    assert s.left.powered is False
    s.left.q[0] += .5
    moved = list(s.left.q)
    s.supervisor_stop(dict(lease_id=lease)); s.advance(.5)
    assert s.left.powered is True and abs(s.left.q[0] - moved[0]) < .01 and abs(s.left.q_cmd[0] - moved[0]) < .01


def test_a_second_pose_is_offered_for_a_rough_or_an_ambiguous_camera():
    from roborun_sdk.overhead import second_pose_helps
    assert second_pose_helps(dict(state='rough')) and second_pose_helps(dict(state='needs_attention', code='ambiguous'))
    assert not second_pose_helps(dict(state='needs_attention', code='no_depth')) and not second_pose_helps(dict(state='calibrated'))
    assert not second_pose_helps(None)


def test_after_the_move_setup_waits_for_a_fit_still_in_progress(monkeypatch):
    from roborun_sdk import overhead
    now = [0.]
    states = iter([dict(state='calibrating')] * 60 + [dict(state='calibrated', grade='good')])   # the verdict comes at ~120 s
    monkeypatch.setattr(overhead, 'read', lambda backend: next(states))
    got = overhead._settle('b', 90, sleep=lambda s: now.__setitem__(0, now[0] + s), clock=lambda: now[0])
    assert got['state'] == 'calibrated' and now[0] < 90 + overhead.EXTRA_WAIT_S
    states = iter([dict(state='rough')] * 1000)                                               # a settled 'rough' is not waited past wait_s
    now[0] = 0.; got = overhead._settle('b', 90, sleep=lambda s: now.__setitem__(0, now[0] + s), clock=lambda: now[0])
    assert got['state'] == 'rough' and now[0] <= 92


def test_a_second_pose_that_cannot_settle_the_heading_gets_another_move_up_to_three(monkeypatch):
    from roborun_sdk import overhead
    moves, verdicts = [], iter([dict(state='rough'), dict(state='calibrated', grade='good')])
    monkeypatch.setattr(overhead, 'check', lambda backend, wait_s=30, sleep=None: dict(state='rough', arm='left'))
    monkeypatch.setattr(overhead, 'plan', lambda st, r, m, arm: dict(arm=arm, q=[0.] * 6, target=[.3] * 6, moved_m=.2, need_m=.08))
    monkeypatch.setattr(overhead, 'move_to', lambda st, step, sleep=None: moves.append(step) and None)
    monkeypatch.setattr(overhead, '_settle', lambda backend, wait_s, sleep: next(verdicts))
    said = []
    out = overhead.improve('b', lambda q: True, station=object(), say=said.append, sleep=lambda s: None)
    assert out['state'] == 'calibrated' and len(moves) == 2 and any('One more slow move' in t for t in said)
    moves.clear(); monkeypatch.setattr(overhead, '_settle', lambda backend, wait_s, sleep: dict(state='rough'))
    assert overhead.improve('b', lambda q: True, station=object(), say=said.append, sleep=lambda s: None)['state'] == 'rough'
    assert len(moves) == overhead.MAX_MOVES                                                  # never more than three
