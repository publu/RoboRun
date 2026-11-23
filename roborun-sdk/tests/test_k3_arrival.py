"""A segment K3 slows still reaches its end (a K3-clamped segment ends arrived).

Seen in sim: with the stream7d.v1 fixed step 0.034 rad at a 0.10 m/s cap, K3 clamped 120 of 120 writes of the reach;
the player advanced executed time by the full proposed step anyway, ran out of rows, and the end gate never wrote the
last row: `not_arrived`, 0.566 rad short. Now a slowed write advances executed time only by the fraction K3 let through
(the command stays on the rows), and the end gate keeps commanding the last row through K3/K4 until the command holds
it before it times stillness and arrival.
"""
import numpy as np
import pytest

from roborun_sdk.rt import player as player_mod
from roborun_sdk.rt import runtime as runtime_mod

from conftest import Rig


def reach_rows(rig, n=19, j2=0.35, j3=0.30):
    """A long elbow-and-shoulder reach as path rows: the stream7d.v1 0.034 rad step at a 0.10 m/s cap is far too fast
    for K3's per-point bound at this lever, so every write is clamped (the case the earlier behaviour got wrong)."""
    q0 = np.r_[rig.left.q, rig.left.jaw]
    rows = np.tile(q0, (n, 1))
    s = np.linspace(0.0, 1.0, n)
    rows[:, 1] += j2 * s
    rows[:, 2] += j3 * s
    rows[:, 0] += 0.25 * np.sin(np.pi * s)            # a bend, so cutting corners would show
    return rows


def polyline_distance(rows, q):
    """Max-norm distance (rad) from joint vector q to the piecewise-linear rows (joints only)."""
    best = np.inf
    for a, b in zip(rows[:-1, :6], rows[1:, :6]):
        d = b - a
        t = 0.0 if not np.any(d) else float(np.clip(np.dot(q - a, d) / np.dot(d, d), 0.0, 1.0))
        best = min(best, float(np.max(np.abs(q - (a + t * d)))))
    return best


def run(rig, seg, seconds=60.0):
    ack = rig.rt.submit(seg)
    assert ack["result"] == "accepted", ack
    rig.run_to_end(seconds)
    return rig.rt.playback


def test_k3_clamped_path_rows_reach_ends_arrived(rig):
    rows = reach_rows(rig)
    seg = rig.segment(rows, fmt="path-rows.v1", sid="reach_clamped", limits={"ee_speed_m_s": 0.10})
    pb = run(rig, seg)
    assert pb.clamps > 0.8 * pb.writes                       # K3 slowed (nearly) every write: the planner-bug case
    clamp_events = [e for e in rig.events("clamp")]
    assert clamp_events and clamp_events[0]["planner_bug"] is True
    assert pb.outcome == "complete" and pb.gate_evidence["arrived"] is True
    assert np.max(np.abs(rig.left.q - rows[-1, :6])) < 0.01
    assert pb.settled and pb.command_at_end()
    # executed time: the command followed the rows (never cut a corner), every write on the polyline
    worst = max(polyline_distance(rows, np.asarray(w[1][:6])) for w in rig.left.writes)
    assert worst < 2e-3, worst


def at_cap_rows30(rig, n=31, offset=0.008, ee=0.10):
    """rows30 timed at 98 % of K3's per-point bound, starting 8 mrad ahead of the measured pose (inside tol_rad): the
    ramp that blends that offset away adds motion on top of rows already at the bound, so K3 clamps the first writes."""
    from roborun_sdk.rt.kernel.limits import ee_cap, point_travel
    ap = rig.pack.arm("left")
    cap = min(ap.point_step_m_per_15hz * 0.5, ee_cap(ap, {"ee_speed_m_s": ee}, None, "local") / 30)
    q0 = np.r_[rig.left.q, rig.left.jaw]
    lo, hi = 0.0, 0.05
    for _ in range(40):
        mid = (lo + hi) / 2
        b = q0[:6].copy()
        b[0] += mid
        lo, hi = (lo, mid) if point_travel(ap, q0[:6], b) > 0.98 * cap else (mid, hi)
    rows = np.tile(q0, (n, 1))
    rows[:, 0] += offset + lo * np.arange(n)
    return rows


def test_k3_clamped_rows30_segment_ends_arrived(rig):
    rows = at_cap_rows30(rig)
    seg = rig.segment(rows, sid="rows30_clamped", limits={"ee_speed_m_s": 0.10}, start_q=list(rig.left.q))
    pb = run(rig, seg)
    assert pb.clamps > 0
    assert pb.outcome == "complete" and pb.gate_evidence["arrived"] is True
    assert np.max(np.abs(rig.left.q - rows[-1, :6])) < 0.003


def set_jaw_noise(arm, noise):
    """The sim jaw reading's jitter (connectors.sim.JAW_READ_NOISE by default: one recorded YAM quantum)."""
    arm.jaw_noise = noise
    arm._jaw_read_noise = 0.0


@pytest.mark.parametrize("noise", [0.0, None, 5e-4])
def test_jaw_that_lags_its_rows_is_settled_by_the_gate(rig, noise):
    """A close at the jaw-step bound on a sluggish jaw: the 0.03 lead bound holds the command back, so the rows run out
    first. The gate keeps commanding the last row's jaw until the command holds it (it was once left 0.22 short). A
    jittering jaw reading does not end that early: the jaw still gains ~0.03 per rest window, far above the noise band."""
    if noise is not None:
        set_jaw_noise(rig.left, noise)
    rig.left.tau_s = 0.3
    q0 = np.r_[rig.left.q, rig.left.jaw]
    rows = np.tile(q0, (16, 1))
    rows[:, 6] = q0[6] - 0.0195 * np.arange(16)
    seg = rig.segment(rows, sid="close_slow", label="close")
    pb = run(rig, seg)
    assert pb.outcome == "complete" and pb.settled
    assert abs(pb.last_target[6] - rows[-1, 6]) < 1e-6        # the last write is the last row's jaw


def blocked_close(rig, sid="close_blocked"):
    rig.left.inject(jaw_block=0.9)
    q0 = np.r_[rig.left.q, rig.left.jaw]
    rows = np.tile(q0, (16, 1))
    rows[:, 6] = q0[6] - 0.0195 * np.arange(16)
    seg = rig.segment(rows, sid=sid, label="close")
    t0 = rig.clock.now()
    pb = run(rig, seg)
    rows_end = pb.gate_opened
    return dict(pb=pb, rows=rows, rows_s=rows_end - t0, end_after_rows_s=rig.clock.now() - rows_end,
                settle_writes=sum(1 for t, _ in rig.left.writes if t > rows_end + 1e-9))


@pytest.mark.parametrize("noise", [0.0, None, 1e-4, 5e-4])
def test_a_blocked_jaw_does_not_hold_up_completion(rig, noise):
    """An object in the jaw stops the measured jaw; the lead bound stops the command 0.03 past it. Nothing more can
    be commanded, so the gate settles and completes on the joints, as before -- also when the jaw reading jitters
    (with +-1e-4 of jitter the lead bound moved every tick, the exact-equality settle test never passed, and
    the close ended 3.53 s after its rows with 90 extra writes). Noise-free it settles at once; with jitter within one
    rest window (0.3 s) of no progress beyond the rest band."""
    if noise is not None:
        set_jaw_noise(rig.left, noise)
    r = blocked_close(rig)
    pb = r["pb"]
    assert pb.outcome == "complete" and pb.settled and pb.gate_evidence["arrived"] is True
    assert r["rows"][-1, 6] + 0.1 < pb.last_target[6]          # the command never went past the lead bound
    assert abs(pb.last_target[6] - (0.9 - rig.pack.arm("left").jaw.lead_max)) < 2e-3
    if noise == 0.0:
        assert r["settle_writes"] == 0 and r["end_after_rows_s"] < 0.6
    else:
        rest = rig.pack.rest
        assert r["end_after_rows_s"] < 1.2, r                  # was 3.53 s: the whole complete.timeout_s
        assert r["settle_writes"] <= round(rest["window_s"] * 30) + 2, r


def test_the_exact_equality_settle_is_caught(rig, monkeypatch):
    """Mutation check: without the noise-band rule (settle only on an unchanged command) a jittering blocked close sits
    in the settle phase for the whole complete.timeout_s, so the test above bites."""
    monkeypatch.setattr(player_mod.Playback, "settle_idle_s", lambda self, *a, **k: 0.0)
    set_jaw_noise(rig.left, 1e-4)
    r = blocked_close(rig, sid="close_blocked_exact")
    assert r["pb"].outcome == "complete" and r["end_after_rows_s"] > 3.0 and r["settle_writes"] > 60


def test_the_earlier_player_is_caught(rig, monkeypatch):
    """Mutation check: the earlier behaviour (full tau per clamped write, the gate times stillness on whatever the
    last proposal was) ends this reach short of its rows, so the tests above bite."""
    def old_commit(self, target, tau, *, clamped, scale=1.0):
        self.last_target = np.asarray(target, float).copy()
        self.tau = tau
        self.writes += 1
        self.clamps += int(bool(clamped))

    monkeypatch.setattr(player_mod.Playback, "commit", old_commit)
    monkeypatch.setattr(player_mod.Playback, "command_at_end", lambda self, tol=1e-9: True)
    rows = reach_rows(rig)
    seg = rig.segment(rows, fmt="path-rows.v1", sid="reach_example", limits={"ee_speed_m_s": 0.10})
    pb = run(rig, seg, seconds=20.0)
    assert pb.outcome == "not_arrived"
    assert np.max(np.abs(rig.left.q - rows[-1, :6])) > 0.1
    r2 = Rig(rig.pack, rig.clock)
    rows30 = at_cap_rows30(r2)
    pb = run(r2, r2.segment(rows30, sid="rows30_example", limits={"ee_speed_m_s": 0.10}, start_q=list(r2.left.q)))
    assert pb.outcome == "not_arrived" and pb.gate_evidence["arrived"] is False


def test_unclamped_segments_are_unchanged(rig):
    """No K3 clamp: one write per row step, the gate opens on the last row (settled at once), as before."""
    seg = rig.segment(seq=1, sid="plain", delta=0.03, seconds=1.0)
    pb = run(rig, seg)
    assert pb.outcome == "complete" and pb.writes == 30 and pb.clamps == 0 and pb.settled


def test_settle_idle_counts_progress_beyond_the_noise_band_only(rig):
    """The settle rule's bookkeeping: progress is a gap that shrinks by more than the band since the best so far (so
    slow progress adds up), or a step K3's speed clamps cut (the joints still travel at the cap); jitter is not."""
    from roborun_sdk.rt.segment import parse_segment
    pb = player_mod.Playback(parse_segment(rig.segment(sid="idle")), start_q7=np.r_[rig.left.q, rig.left.jaw],
                             n_joints=6, now=0.0)
    end = pb.seg.rows[-1]
    at = lambda dj=0.0, dw=0.0: np.r_[end[:6] + dj, end[6] + dw]
    kw = dict(joint_eps=0.0015, jaw_eps=0.003)
    assert pb.settle_idle_s(at(dw=0.03), 0.0, **kw) == 0.0
    assert pb.settle_idle_s(at(dw=0.0302), 0.1, **kw) == pytest.approx(0.1)        # jitter: not progress
    assert pb.settle_idle_s(at(dw=0.0298), 0.2, **kw) == pytest.approx(0.2)
    assert pb.settle_idle_s(at(dw=0.0285), 0.3, **kw) == pytest.approx(0.3)        # 1.5e-3 closer: still in the band
    assert pb.settle_idle_s(at(dw=0.0265), 0.4, **kw) == 0.0                       # 3.5e-3 closer in total: progress
    assert pb.settle_idle_s(at(dw=0.0265), 0.7, **kw) == pytest.approx(0.3)
    assert pb.settle_idle_s(at(dw=0.0265), 0.8, moving=True, **kw) == 0.0          # speed-clamped step: progress
    assert pb.settle_idle_s(at(dj=-0.002, dw=0.0265), 0.9, **kw) == pytest.approx(0.1)   # joints moved AWAY: no
    assert pb.settle_idle_s(at(dj=0.0, dw=0.0265), 1.0, **kw) == pytest.approx(0.2)


@pytest.mark.parametrize("scale", [0.0, 0.25, 1.0, float("nan")])
def test_commit_advances_by_the_written_fraction(rig, scale):
    seg = rig.segment(sid="frac")
    from roborun_sdk.rt.segment import parse_segment
    pb = player_mod.Playback(parse_segment(seg), start_q7=np.r_[rig.left.q, rig.left.jaw], n_joints=6, now=0.0)
    target, tau = pb.proposal()
    pb.commit(target, tau, clamped=scale < 1.0 if scale == scale else True, scale=scale)
    expected = 0.0 if scale != scale else tau * scale
    assert pb.tau == pytest.approx(expected)
    assert runtime_mod.Playback is player_mod.Playback
