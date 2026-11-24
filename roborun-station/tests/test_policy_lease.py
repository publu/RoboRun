"""The station's `policy` lease (roborun_station.policy): pushed chunks, checks, playback, the clip, holds, Stop, heartbeat,
jaw units. Deterministic: a SimClock station ticked by hand."""
import json
import math
import sys
import urllib.error
import urllib.request
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from roborun_station import FakeYamStation, SimClock, SimServer, StationError, metal_spec  # noqa: E402
from roborun_station import policy as pol  # noqa: E402

DT = 1 / 30


def station(**kw):
    kw.setdefault("right", "yam")
    return FakeYamStation(clock=SimClock(), lease_free_hold=True, **kw)


def lease(st, arms=("left",), seconds=30, **ref):
    oid = st.observe()["observation_id"]
    reply = st.action({"observation_id": oid, "seconds": seconds,
                       "reference": dict({"kind": "policy", "arms": list(arms), "run_id": "a" * 32}, **ref)})
    return reply["lease"]["id"], reply


def rows_from(st, side, n=48, hz=30.0, dq=0.0, j=0, frac=1.0, start=None):
    arm = st.arm(side)
    q0 = list(arm.q_cmd) if start is None else list(start)
    out = []
    for i in range(n):
        q = list(q0)
        q[j] += dq * i
        out.append(q + [frac])
    return out


def push(st, lid, rows, hz=30.0, skip_s=0.0, seq=None):
    body = {"lease_id": lid, "hz": hz, "skip_s": skip_s, "rows": rows}
    if seq is not None:
        body["seq"] = seq
    return st.policy_chunk(body)


def test_lease_starts_echoes_limits_and_respects_the_cap():
    st = station()
    lid, reply = lease(st, arms=("left", "right"), seconds=pol.POLICY_MAX_S)
    assert reply["limits"]["max_s"] == pol.POLICY_MAX_S and reply["limits"]["seam_rad"] == pol.POLICY_SEAM_RAD
    assert reply["lease"]["arms"] == ["left", "right"] and reply["lease"]["arm"] == "left"
    assert reply["lease"]["heartbeat_s"] == pol.POLICY_HEARTBEAT_S
    st.hold()
    st.advance(1.0)
    with pytest.raises(StationError, match="seconds must be between 0.5 and 600"):
        lease(st, seconds=pol.POLICY_MAX_S + 1)
    with pytest.raises(StationError, match="names its arms"):
        lease(st, arms=("middle",))


def test_lease_refused_in_the_latch_and_names_the_run_when_busy():
    st = station()
    st.hold()
    with pytest.raises(StationError, match="Stop \\(hold\\) was pressed"):
        lease(st)
    st.advance(1.0)
    lease(st)
    oid = st.observe()["observation_id"]
    with pytest.raises(StationError, match="a policy run is in progress \\(task a{32}\\); Stop it first"):
        st.action({"observation_id": oid, "seconds": 4, "reference": {"kind": "mixed_jog", "target_rad": [0] * 6}})


def test_a_chunk_plays_skips_due_rows_and_overlays():
    st = station()
    lid, _ = lease(st)
    rows = rows_from(st, "left", n=48, dq=0.01)
    out = push(st, lid, {"left": rows}, skip_s=0.2)            # 0.2 s at 30 Hz: 6 rows already due
    assert out["accepted"] and out["skipped_rows"] == 6
    st.advance(0.5)
    q = st.left.q_cmd[0]
    assert rows[0][0] + 0.06 < q < rows[0][0] + 0.06 + 0.2      # moving along the chunk, never back to its start
    rows2 = rows_from(st, "left", n=48, dq=-0.01)               # a new chunk from here replaces what was left
    push(st, lid, {"left": rows2})
    st.advance(0.5)
    assert st.left.q_cmd[0] < q


def test_a_20_hz_chunk_is_resampled_on_the_30_hz_tick():
    st = station()
    lid, _ = lease(st)
    q0 = st.left.q_cmd[0]
    push(st, lid, {"left": rows_from(st, "left", n=21, hz=20.0, dq=0.01)}, hz=20.0)   # 0.2 rad/s for 1 s
    st.advance(0.5)
    assert abs(st.left.q_cmd[0] - (q0 + 0.1)) < 0.012


@pytest.mark.parametrize("mutate, code", [
    (lambda r: [row[:-1] for row in r], "length"),
    (lambda r: [r[0][:2] + [float("nan")] + r[0][3:]] + r[1:], "not_finite"),
    (lambda r: [row[:-1] + [1.3] for row in r], "gripper"),
    (lambda r: [], "rows"),
    (lambda r: r * 20, "rows"),
])
def test_each_malformed_chunk_is_refused(mutate, code):
    st = station()
    lid, _ = lease(st)
    with pytest.raises(pol.ChunkRefused) as err:
        push(st, lid, {"left": mutate(rows_from(st, "left"))})
    assert err.value.code == code and err.value.reply["lease"]["active"]


def test_hz_late_seam_and_side_refusals():
    st = station()
    lid, _ = lease(st)
    good = rows_from(st, "left")
    for body, code in [(dict(hz=2.0), "hz"), (dict(skip_s=3.5), "late"), (dict(skip_s=2.0), "late"),
                       (dict(rows={"left": good, "right": good}), "wrong_side")]:
        with pytest.raises(pol.ChunkRefused) as err:
            st.policy_chunk(dict({"lease_id": lid, "hz": 30.0, "skip_s": 0.0, "rows": {"left": good}}, **body))
        assert err.value.code == code
    far = rows_from(st, "left", dq=0.0)
    far = [[v + (0.6 if i == 2 else 0) for i, v in enumerate(r)] for r in far]
    with pytest.raises(pol.ChunkRefused) as err:
        push(st, lid, {"left": far})
    assert err.value.code == "seam"
    lid2 = None
    st.hold(); st.advance(1.0)
    lid2, _ = lease(st, arms=("left", "right"))
    with pytest.raises(pol.ChunkRefused) as err:
        push(st, lid2, {"left": rows_from(st, "left")})
    assert err.value.code == "missing_side"


def test_five_refusals_in_a_row_end_the_lease():
    st = station()
    lid, _ = lease(st)
    for i in range(pol.POLICY_REFUSALS_END):
        with pytest.raises(pol.ChunkRefused):
            push(st, lid, {"left": [[0.0] * 3]})
    assert st.lease["active"] is False and st.lease["reason"] == "policy_error"
    assert "5 chunks refused in a row" in st.lease["stop_reason"]


def test_the_step_per_tick_is_clipped_and_counted():
    st = station()
    lid, _ = lease(st)
    q0 = st.left.q_cmd[0]
    rows = rows_from(st, "left", n=30, dq=0.0)
    rows = [rows[0]] + [[r[0] + 0.45] + r[1:] for r in rows[1:]]    # a 0.45 rad jump on the second row
    push(st, lid, {"left": rows})
    st.advance(DT)
    st.advance(DT)
    st.advance(DT)
    steps = []
    last = st.left.q_cmd[0]
    for _ in range(5):
        st.advance(DT)
        steps.append(st.left.q_cmd[0] - last)
        last = st.left.q_cmd[0]
    assert max(abs(s) for s in steps) <= pol.JOINT_STEP + 1e-9
    assert st.lease["policy"]["clipped_ticks"] > 0
    assert st.left.q_cmd[0] > q0


def test_an_empty_buffer_holds_and_a_long_dry_spell_ends_the_lease():
    st = station()
    lid, _ = lease(st, seconds=60)
    push(st, lid, {"left": rows_from(st, "left", n=10, dq=0.01)})
    st.advance(1.0)
    held = list(st.left.q_cmd)
    for _ in range(22):
        st.heartbeat({"lease_id": lid})
        st.advance(0.5)
    assert st.left.q_cmd == held                                 # dry: the last target stays
    assert st.lease["policy"]["dry_ticks"] > 0 and st.lease["active"] is False
    assert st.lease["reason"] == "policy_error" and "no rows to play" in st.lease["stop_reason"]


def test_a_missing_heartbeat_disconnects_and_holds_within_a_second():
    st = station()
    lid, _ = lease(st)
    push(st, lid, {"left": rows_from(st, "left", n=200, dq=0.005)})
    st.advance(0.9)
    assert st.lease["active"]
    st.advance(0.2)
    assert st.lease["active"] is False and st.lease["reason"] == "client_disconnected"


def test_stop_mid_chunk_ends_the_lease_and_nothing_more_plays():
    st = station()
    lid, _ = lease(st)
    push(st, lid, {"left": rows_from(st, "left", n=48, dq=0.01)})
    st.advance(0.3)
    reply = st.hold()
    assert "lease_ended" in reply["actions"] and st.lease["reason"] == "supervisor_stop"
    assert all(not b.rows for b in st.lease["_policy"]["buffers"].values())
    q = list(st.left.q_cmd)
    st.advance(0.5)
    assert st.left.q_cmd == q
    with pytest.raises(pol.ChunkRefused) as err:
        push(st, lid, {"left": rows_from(st, "left")})
    assert err.value.code == "lease" and err.value.reply["lease"]["reason"] == "supervisor_stop"
    with pytest.raises(StationError, match="Stop \\(hold\\) was pressed"):
        lease(st)


def test_policy_end_is_an_ordinary_end():
    st = station()
    lid, _ = lease(st)
    st.policy_end({"lease_id": lid, "reason": "seconds elapsed"})
    assert st.lease["reason"] == "policy_finished" and st.state == "idle"


def test_jaw_fraction_is_converted_to_metal_motor_units():
    st = FakeYamStation(clock=SimClock(), lease_free_hold=True, left_spec=metal_spec("left"))
    closed, opened = 0.1009054704, 1.9413522545
    assert st.observe()["jaw_range"]["left"] == [closed, opened]
    assert pol.fraction_from_jaw(pol.jaw_from_fraction(0.5, (closed, opened)), (closed, opened)) == pytest.approx(0.5)
    lid, _ = lease(st)
    out = push(st, lid, {"left": rows_from(st, "left", n=90, frac=0.5)})
    assert out["accepted"]
    for _ in range(4):
        st.heartbeat({"lease_id": lid})
        st.advance(0.5)
    assert st.lease["active"]
    assert st.left.jaw_cmd == pytest.approx(closed + 0.5 * (opened - closed), abs=1e-6)
    assert st.lease["policy"]["refused"] == {}


def test_jaw_step_is_applied_in_station_units():
    st = FakeYamStation(clock=SimClock(), lease_free_hold=True, left_spec=metal_spec("left"))
    lid, _ = lease(st)
    before = st.left.jaw_cmd
    push(st, lid, {"left": rows_from(st, "left", n=60, frac=0.0)})
    st.advance(DT)
    assert before - st.left.jaw_cmd <= pol.JAW_STEP + 1e-9


def test_an_arm_without_a_jaw_range_is_refused_for_a_gripper_run():
    st = FakeYamStation(clock=SimClock(), lease_free_hold=True, left_spec=metal_spec("left", jaw_open=0.5, jaw_closed=0.5))
    with pytest.raises(StationError, match="the left arm has no jaw range; policy runs need one"):
        lease(st)
    lid, _ = lease(st, gripper=False)
    assert push(st, lid, {"left": [r[:-1] for r in rows_from(st, "left")]})["accepted"]


def test_a_dying_arm_in_a_two_arm_lease_ends_it():
    st = station()
    lid, _ = lease(st, arms=("left", "right"))
    st.faults.add("usb_drop", st.clock() + 0.1, arm="right")
    st.heartbeat({"lease_id": lid})
    st.advance(0.3)
    assert st.lease["reason"] == "station_error"


def test_the_tool_speed_cap_can_only_be_lowered():
    st = station()
    _, reply = lease(st, max_tool_speed_m_s=5.0)
    assert reply["lease"]["tool_speed_m_s"] == pol.POLICY_TOOL_SPEED_M_S
    st.hold(); st.advance(1.0)
    _, reply = lease(st, max_tool_speed_m_s=0.1)
    assert reply["lease"]["tool_speed_m_s"] == 0.1


def test_the_old_kinds_are_unchanged():
    st = station(right=None)
    session = st.register_session(seconds=6)
    oid = st.observe()["observation_id"]
    out = st.action({"observation_id": oid, "seconds": 6, "reference": {"kind": "video_motion", "session_id": session}})
    assert "arms" not in out["lease"] and "limits" not in out
    with pytest.raises(StationError, match="seconds must be between 0.5 and 12"):
        oid = st.observe()["observation_id"]
        st.hold(); st.advance(1.0)
        st.action({"observation_id": st.observe()["observation_id"], "seconds": 13, "reference": {"kind": "mixed_jog"}})


def test_pose_reports_the_policy_buffer():
    st = station()
    lid, _ = lease(st)
    push(st, lid, {"left": rows_from(st, "left", n=48)})
    st.advance(0.2)
    pose = st.pose()
    assert pose["execution_lease"]["kind"] == "policy"
    p = pose["execution_lease"]["policy"]
    assert 1.0 < p["buffer_s"] < 1.6 and p["chunks"] == 1 and "tool_speed_m_s" in p


def test_over_http_a_refusal_answers_409_with_its_reason():
    st = FakeYamStation(lease_free_hold=True, right="yam")
    with SimServer(st) as srv:
        def post(path, body):
            req = urllib.request.Request(srv.url + path, data=json.dumps(body).encode(), method="POST",
                                         headers={"Content-Type": "application/json", "X-Manifest-Supervisor": "1"})
            try:
                with urllib.request.urlopen(req, timeout=5) as r:
                    return r.status, json.load(r)
            except urllib.error.HTTPError as exc:
                return exc.code, json.load(exc)
        oid = post("/supervisor/observe", {})[1]["observation_id"]
        code, out = post("/supervisor/action", {"observation_id": oid, "seconds": 10,
                                                "reference": {"kind": "policy", "arms": ["left"]}})
        assert code == 200
        lid = out["lease"]["id"]
        code, out = post("/supervisor/policy/chunk", {"lease_id": lid, "hz": 30, "skip_s": 0, "rows": {"left": [[0.0]]}})
        assert code == 409 and out["code"] == "length" and out["lease"]["active"] is True
        code, out = post("/supervisor/policy/end", {"lease_id": lid})
        assert code == 200 and out["lease"]["reason"] == "policy_finished"


def test_a_policy_lease_refuses_when_a_stop_came_after_its_run_started():
    st = station()
    seen = st.observe()["status"]["hold_generation"]
    assert st.pose()["hold_generation"] == seen
    st.hold()
    st.advance(1.0)                                    # past the latch: only the Stop counter remembers
    assert st.pose()["hold_generation"] == seen + 1
    with pytest.raises(StationError, match="Stop was pressed after this run started; nothing moved; run again"):
        lease(st, since_hold_generation=seen)
    assert st.lease is None
    with pytest.raises(StationError, match="restarted"):
        lease(st, since_hold_generation=seen + 5)
    with pytest.raises(StationError, match="must be an integer"):
        lease(st, since_hold_generation="1")
    lid, reply = lease(st, since_hold_generation=seen + 1)
    assert reply["lease"]["active"]


def test_a_per_task_stop_ends_that_run_s_lease_like_a_task_s_own_stop():
    st = station()
    lid, _ = lease(st)
    push(st, lid, {"left": rows_from(st, "left", n=48, dq=0.01)})
    st.advance(0.3)
    gen = st.hold_generation
    reply = st.policy_stop({"run_id": "a" * 32})
    assert reply["lease_ended"] is True and reply["lease"]["reason"] == "supervisor_stop"
    assert st.lease["active"] is False and st.lease["end_note"] == "per-task Stop"
    assert all(not b.rows for b in st.lease["_policy"]["buffers"].values())
    assert st.hold_generation == gen                   # one run's Stop: not the robot-wide hold
    q = list(st.left.q_cmd)
    st.advance(0.5)
    assert st.left.q_cmd == q                          # nothing more plays
    with pytest.raises(pol.ChunkRefused) as err:
        push(st, lid, {"left": rows_from(st, "left")})
    assert err.value.reply["lease"]["reason"] == "supervisor_stop"
    assert "a" * 32 in st.pose()["stopped_runs"] and "a" * 32 in st.status()["stopped_runs"]
    again = st.policy_stop({"run_id": "a" * 32})       # stopping an ended run is fine
    assert again["lease_ended"] is False


def test_a_per_task_stop_before_the_lease_refuses_that_run_only():
    st = station()
    reply = st.policy_stop({"run_id": "a" * 32})
    assert reply["lease_ended"] is False and reply["lease"] is None
    with pytest.raises(StationError, match="Stop was pressed for this run \\(task a{32}\\); nothing moved"):
        lease(st)
    assert st.lease is None
    lid, other = lease(st, run_id="b" * 32)            # another run is not affected, and no latch applies
    assert other["lease"]["active"]
    assert st.policy_stop({"run_id": "c" * 32})["lease_ended"] is False
    assert st.lease["active"] is True                  # a stop for another run leaves this lease alone


def test_a_per_task_stop_needs_a_run_id_and_keeps_a_bounded_list():
    st = station()
    for bad in ({}, {"run_id": ""}, {"run_id": 5}, {"run_id": "x" * 65}):
        with pytest.raises(StationError, match="run_id"):
            st.policy_stop(bad)
    for i in range(100):
        st.policy_stop({"run_id": f"{i:032x}"})
    assert len(st.stopped_runs) == 32 and st.stopped_runs[-1] == f"{99:032x}"


def test_over_http_the_per_task_stop_needs_the_supervisor_header():
    st = FakeYamStation(lease_free_hold=True, right="yam")
    with SimServer(st) as srv:
        def post(path, body, headers):
            req = urllib.request.Request(srv.url + path, data=json.dumps(body).encode(), method="POST",
                                         headers=dict({"Content-Type": "application/json"}, **headers))
            try:
                with urllib.request.urlopen(req, timeout=5) as r:
                    return r.status, json.load(r)
            except urllib.error.HTTPError as exc:
                return exc.code, json.load(exc)
        assert post("/supervisor/policy/stop", {"run_id": "a" * 32}, {})[0] == 403
        code, out = post("/supervisor/policy/stop", {"run_id": "a" * 32}, {"X-Manifest-Supervisor": "1"})
        assert code == 200 and out["lease_ended"] is False and "a" * 32 in out["status"]["stopped_runs"]
