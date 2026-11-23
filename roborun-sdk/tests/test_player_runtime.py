"""The player rules on the runtime: executed time, ramp, fresh-pose hold budget, triggers, end gates, continuations."""
import numpy as np

from conftest import Rig, example


def test_rows30_executed_time_and_completion(rig):
    seg = rig.segment(seq=1, sid="m", delta=0.03, seconds=1.0)
    assert rig.rt.submit(seg)["result"] == "accepted"
    rig.run_to_end()
    pb = rig.rt.playback
    assert pb.outcome == "complete" and pb.writes == 30 and pb.clamps == 0
    assert abs(rig.left.q[0] - (seg["valid_if"]["start_q"][0] + 0.03)) < 0.002
    reports = rig.events("gate_report")
    assert reports and reports[-1]["outcome"] == "complete" and reports[-1]["evidence"]["writes"] == 30


def test_ramp_blends_the_start_offset(rig):
    q = rig.left.q.copy()
    rows = np.tile(np.r_[q, rig.left.jaw], (31, 1))
    rows[:, 2] += 0.008                                    # rows start 8 mrad off the measured pose (inside tol)
    rig.rt.submit(rig.segment(rows, seq=1, sid="ramp"))
    rig.drive(1 / 30)
    first = rig.left.writes[0][1]
    assert abs(first[2] - q[2]) < 0.002                    # first write is near the measured pose, not a jump
    rig.run_to_end()
    assert abs(rig.left.writes[-1][1][2] - (q[2] + 0.008)) < 1e-6


def test_stale_samples_freeze_tau_then_end_after_budget(rig):
    rig.rt.submit(rig.segment(seq=1, sid="f", seconds=2.0))
    rig.drive(0.3)
    tau = rig.rt.playback.tau
    writes = len(rig.left.writes)
    rig.right.inject(frozen_stamps=True)                   # the other arm's samples stop: fresh-pose hold
    rig.drive(0.3)
    assert rig.rt.playback.tau <= tau + rig.rt.dt * 6      # a hold never skips ahead (<= the ~150 ms age bound)
    assert len(rig.left.writes) <= writes + 6
    rig.drive(1.0)
    assert rig.rt.playback.outcome == "fresh_pose_budget_exhausted"
    kinds = [e.get("kind") for e in rig.events()]
    assert "fresh_pose_hold" in kinds


def test_fresh_hold_resumes_where_it_stopped(rig):
    rig.rt.submit(rig.segment(seq=1, sid="r", seconds=2.0))
    rig.drive(0.3)
    rig.right.inject(frozen_stamps=True)
    rig.drive(0.25)
    tau_hold = rig.rt.playback.tau
    rig.right.faults.clear()
    rig.drive(1 / 30 * 2)
    assert rig.rt.playback.state != "ended"
    assert rig.rt.playback.tau <= tau_hold + 2 * rig.rt.dt + 1e-9      # resumes from where it stopped
    rig.run_to_end()
    assert rig.rt.playback.outcome == "complete"


def test_path_rows_tracking_pause(rig):
    seg = example("sim-reach.path-rows")
    r = Rig(rig.pack, rig.clock, left_q=seg["valid_if"]["start_q"], left_jaw=seg["valid_if"]["jaw"])
    r.rt.register_observation("obs_30c7", r.clock.now())
    r.left.tau_s = 0.5                                     # a sluggish arm: the target would run away
    assert r.rt.submit(seg)["result"] == "accepted"
    r.drive(0.5)
    pb = r.rt.playback
    lead = np.max(np.abs(pb.last_target[:6] - r.left.q))
    assert lead <= 0.085 + 0.02                            # track_limit (+ one step)
    r.run_to_end()
    assert r.rt.playback.outcome == "complete"


def test_trigger_hold_ends_and_reports(rig):
    trig = [{"id": "jaw_low", "when": {"sig": "jaw", "op": "<", "v": 0.99}, "do": "hold", "then": "report"}]
    rig.rt.submit(rig.segment(seq=1, sid="t", seconds=2.0, triggers=trig))
    rig.drive(0.2)
    rig.left.jaw = 0.9                                     # the measured jaw drops (contact)
    rig.drive(0.1)
    assert rig.rt.playback.outcome == "trigger:jaw_low"
    ev = [e for e in rig.events() if e.get("kind") == "trigger"]
    assert ev and ev[0]["id"] == "jaw_low"


def test_pause_waits_for_decide(rig):
    trig = [{"id": "bias", "when": {"sig": "track_err", "op": ">", "v": 0.05}, "do": {"pause_s": 0.5}, "then": "end"}]
    rig.rt.submit(rig.segment(seq=1, sid="p", seconds=2.0, triggers=trig))
    rig.drive(0.2)
    rig.left.tau_s = 10.0                                         # pushed and held off its target
    rig.left.q = rig.left.q + np.array([0.06, 0, 0, 0, 0, 0])    # tracking error 0.06 > 0.05 (< the 0.085 pause)
    rig.drive(1 / 30)
    assert rig.rt.playback.state == "paused"
    rig.drive(0.6)
    assert rig.rt.playback.outcome == "trigger:bias"            # no decide arrived: then=end (hold)


def test_continuation_after_complete(rig):
    first = rig.segment(seq=1, sid="one", delta=0.02)
    assert rig.rt.submit(first)["result"] == "accepted"
    rig.run_to_end()
    q = np.asarray(rig.rt.finished["one"]["end_q"])
    rows = np.tile(np.r_[q, rig.left.jaw], (31, 1))
    rows[:, 1] += np.linspace(0, 0.02, 31)
    cont = rig.segment(rows, seq=2, sid="two", after={"segment": "one", "outcome": "complete"})
    assert rig.rt.submit(cont)["result"] == "accepted"
    rig.run_to_end()
    bad = rig.segment(rows, seq=3, sid="three", after={"segment": "nope", "outcome": "complete"})
    assert rig.rt.submit(bad)["reason"] == "continuation_unknown"


def test_gc_is_disabled_only_while_a_segment_runs(rig):
    import gc
    gc.enable()
    rig.rt.submit(rig.segment(seq=1, sid="gc"))
    assert not gc.isenabled()
    rig.run_to_end()
    assert gc.isenabled()


def test_tick_watchdog(rig):
    rig.rt.submit(rig.segment(seq=1, sid="w", seconds=2.0))
    rig.drive(0.2)
    rig.rt.next_tick_t = rig.clock.now() - 0.03
    rig.rt.tick()
    assert any(e.get("kind") == "tick_late" for e in rig.events())
    rig.rt.next_tick_t = rig.clock.now() - 0.06
    rig.rt.tick()
    assert rig.rt.playback.outcome == "tick_overrun"
    assert "tick_overrun" in rig.left.holds and "tick_overrun" in rig.right.holds
    rig.rt.next_tick_t = None
