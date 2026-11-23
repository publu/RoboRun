"""Every acceptance rule, as a typed refusal that leaves the arm holding."""
import copy

import numpy as np
import pytest

from roborun_sdk import dev
from roborun_sdk.rt.datatypes import OperatorConfirm

from conftest import Rig, example


def reason(ack):
    return ack.get("reason") if ack.get("result") == "refused" else ack.get("result")


def test_signed_insert_is_accepted_and_completes(pack, clock):
    seg = example("sim-insert.rows30")
    r = Rig(pack, clock, left_q=seg["valid_if"]["start_q"], left_jaw=seg["valid_if"]["jaw"],
            trust_unknown_plan_basis=False)
    r.rt.register_observation("obs_30c7", clock.now())
    ack = r.rt.submit(seg)
    assert ack["result"] == "accepted", ack
    assert ack["rest"]["joint_range_rad"] <= 0.0015 and ack["proximity"]["ok"]
    r.run_to_end()
    assert r.rt.playback.outcome == "complete"
    assert r.rt.finished["seg_sim_insert"]["outcome"] == "complete"


def test_protocol_rules(rig):
    seg = rig.segment(seq=5, sid="a")
    unsigned = rig.segment(seq=6, sid="b", key=None)
    assert reason(rig.rt.submit(unsigned)) == "bad_signature"
    wrong_wire = rig.segment(seq=7, sid="c", extra={"wire": 2}, key=None)    # not even a valid wire message
    assert reason(rig.rt.submit(wrong_wire)) == "wire_mismatch"
    assert reason(rig.rt.submit(seg)) == "accepted"
    dup = rig.rt.submit(seg)
    assert dup["result"] == "accepted" and dup["duplicate"] is True          # re-acked, never re-run
    rig.run_to_end()
    old = rig.segment(seq=4, sid="d")
    assert reason(rig.rt.submit(old)) == "stale_seq"


def test_generation_and_expiry(rig):
    assert reason(rig.rt.submit(rig.segment(seq=10, sid="g5", generation=5, delta=0.01))) == "accepted"
    rig.run_to_end()
    assert reason(rig.rt.submit(rig.segment(seq=11, sid="g4", generation=4, delta=0.01))) == "old_generation"
    late = rig.segment(seq=12, sid="late", generation=6)
    ack = rig.rt.submit(late, received_t=rig.clock.now() - 11.0)
    assert reason(ack) == "expired"


def test_context_rules(rig, pack):
    assert reason(rig.rt.submit(rig.segment(seq=1, sid="p", pack_digest="sha256:" + "0" * 64))) == "pack_mismatch"
    rig.rt.calibration["id"] = "cal_current"
    assert reason(rig.rt.submit(rig.segment(seq=2, sid="c1", calibration="cal_other"))) == "calibration_mismatch"
    rig.rt.calibration["suspect"] = True
    assert reason(rig.rt.submit(rig.segment(seq=3, sid="c2", calibration="cal_current"))) == "calibration_suspect"
    rig.rt.calibration.update(id=None, suspect=False)
    # plan basis: unknown observation refused unless trusted; a scene change makes older plans stale
    assert reason(rig.rt.submit(rig.segment(seq=4, sid="u", plan_basis_obs="obs_unknown"))) == "plan_basis_unknown"
    rig.rt.scene_change("operator_action")
    assert reason(rig.rt.submit(rig.segment(seq=5, sid="s"))) == "plan_basis_stale"
    rig.drive(0.05)
    rig.rt.register_observation("obs_new", rig.clock.now())
    assert reason(rig.rt.submit(rig.segment(seq=6, sid="ok", plan_basis_obs="obs_new"))) == "accepted"


def test_start_rules(rig):
    q = rig.left.q.copy()
    off = q.copy()
    off[3] += 0.02
    n = 31
    rows = np.tile(np.r_[off, rig.left.jaw], (n, 1))
    ack = rig.rt.submit(rig.segment(rows, seq=1, sid="off"))
    assert reason(ack) == "start_state_mismatch" and ack["detail"]["joint"] == "j4"
    assert reason(rig.rt.submit(rig.segment(seq=2, sid="jaw", jaw=0.5))) == "jaw_mismatch"
    # a moving arm is not at rest by the position window (velocity readings never decide it)
    rig.left.q_cmd = rig.left.q + 0.003
    rig.drive(0.1)
    rows = np.tile(np.r_[rig.left.q, rig.left.jaw], (n, 1))
    assert reason(rig.rt.submit(rig.segment(rows, seq=3, sid="moving"))) == "not_at_rest"


def test_stale_samples_and_offline_other_arm(rig):
    rig.right.inject(frozen_stamps=True)
    rig.drive(0.2)
    assert reason(rig.rt.submit(rig.segment(seq=1, sid="o"))) == "other_arm_unknown"
    rig.right.faults.clear()
    rig.drive(0.5)
    rig.left.inject(frozen_stamps=True)
    rig.drive(0.1)
    assert reason(rig.rt.submit(rig.segment(seq=2, sid="s"))) == "sample_stale"


def test_interlocks_health_and_arming(rig):
    rig.rt.armed_local = False
    assert reason(rig.rt.submit(rig.segment(seq=1, sid="a"))) == "not_armed"
    rig.rt.armed_local = True
    rig.rt.interlocks.set_pending_release("left", evidence="test", source="test")
    assert reason(rig.rt.submit(rig.segment(seq=2, sid="pr"))) == "pending_release"
    rig.rt.interlocks.clear("left", reason="operator_attested", confirm=OperatorConfirm("op", "gripper empty", 0.0))
    rig.left.temps = [72.0] * 7
    rig.drive(0.1)
    assert reason(rig.rt.submit(rig.segment(seq=3, sid="hot"))) == "too_hot"
    rig.left.temps = [40.0] * 7
    rig.drive(0.1)
    rig.rt.stop("test")
    assert reason(rig.rt.submit(rig.segment(seq=4, sid="st"))) == "stopped"


def test_trajectory_rules(rig, pack):
    q, jaw = rig.left.q.copy(), rig.left.jaw
    n = 31
    opening = np.tile(np.r_[q, jaw], (n, 1))
    rig.left.jaw_cmd = rig.left.jaw = 0.5
    rig.drive(0.5)
    jaw = rig.left.jaw
    opening = np.tile(np.r_[q, jaw], (n, 1))
    opening[:, -1] = np.linspace(jaw, jaw + 0.2, n)
    assert reason(rig.rt.submit(rig.segment(opening, seq=1, sid="open"))) == "jaw_open_not_release"
    fast = np.tile(np.r_[q, jaw], (n, 1))
    fast[:, 1] = q[1] + np.linspace(0, 0.9, n)                      # 0.03 rad per tick > 0.02
    assert reason(rig.rt.submit(rig.segment(fast, seq=2, sid="fast"))) == "step_limit"
    hi = pack.arm("left").upper[3] - 0.005                          # inside the hard limit, outside the -0.01 margin
    far = np.tile(np.r_[q, jaw], (n, 1))
    far[:, 3] = np.linspace(q[3], hi, n)
    ack = rig.rt.submit(rig.segment(far, seq=3, sid="limit"))
    assert reason(ack) in ("joint_limit", "step_limit")
    loose = rig.segment(seq=4, sid="loose", limits={"ee_speed_m_s": 0.5})
    assert reason(rig.rt.submit(loose)) == "loosening_limits"
    nan = rig.segment(seq=5, sid="nan")
    nan["trajectory"]["rows"] = np.full((31, 7), np.nan, "<f4").tobytes()
    assert reason(rig.rt.submit(nan)) in ("nonfinite", "bad_signature")


def test_unsupported_shapes_are_typed(rig):
    from roborun_sdk.dev import STREAM7D_MOVE
    loaded = dict(STREAM7D_MOVE, loaded=True)
    cases = ((rig.segment(seq=1, sid="rel", extra={"kind": "release"}), "unsupported_kind"),
             (rig.segment(seq=1, sid="br", extra={"branches": {"lift": {}}}, key=None), "unsupported_branches"),
             (rig.segment(seq=1, sid="ld", fmt="path-rows.v1", player=loaded), "unsupported_player_feature"),
             (rig.segment(seq=1, sid="sc", key=None,
                          extra={"scale": [{"sig": "bias_j1_4", "from": 0.065, "to": 0.085, "min": 0.25}]}),
              "unsupported_format"))
    for seg, want in cases:
        assert reason(rig.rt.submit(seg)) == want


def test_one_moving_arm_per_rig(rig):
    assert reason(rig.rt.submit(rig.segment(seq=1, sid="l", seconds=2.0))) == "accepted"
    rig.drive(0.2)
    right = rig.segment(arm="right", seq=2, sid="r", delta=0.005, limits={"ee_speed_m_s": 0.04})
    assert reason(rig.rt.submit(right)) == "another_arm_moving"


def test_swept_proximity_refuses_a_path_through_an_obstacle(pack, clock):
    """Put a pack obstacle on the midpoint of the path's tool sweep: acceptance must refuse 'proximity' (capsule
    kernel), while the same path without the obstacle is accepted."""
    import copy
    from roborun_sdk.rt.pack import parse_pack
    r = Rig(pack, clock, left_q=[1.5432, 1.7448, 0.2768, 1.5111, -0.0572, 0.0184])
    q = r.left.q.copy()
    rows = np.tile(np.r_[q, r.left.jaw], (61, 1))
    rows[:, 0] = q[0] + np.linspace(0, 0.3, 61) * np.linspace(0, 1, 61)  # a smooth joint-1 sweep (<= 0.01 rad/tick)
    seg = r.segment(rows, seq=1, sid="sweep", limits={"ee_speed_m_s": 0.20})
    assert reason(r.rt.submit(seg)) == "accepted"                    # clear without the obstacle
    geom = pack.arm("left").geometry
    P0, P1, _ = geom.world(rows[30, :6])
    mid = P1[-1]                                                     # the tool end at mid-sweep
    raw = copy.deepcopy(dev.sim_pack_raw())
    raw.pop("sig")
    raw["scene"]["obstacles"].append({"name": "test_post", "capsule": {"p0": [float(mid[0]), float(mid[1]), 0.0],
                                      "p1": [float(mid[0]), float(mid[1]), float(mid[2]) + 0.05], "r": 0.02},
                                      "min_gap_m": 0.02})
    blocked = parse_pack(dev.sign_pack(raw), signed=True)
    r2 = Rig(blocked, clock, left_q=list(q))
    seg2 = r2.segment(rows, seq=1, sid="sweep2", limits={"ee_speed_m_s": 0.20})
    ack = r2.rt.submit(seg2)
    assert reason(ack) == "proximity" and ack["detail"]["checker"] == "capsule", ack


def test_home_rest_below_a_soft_limit_may_hold_but_not_go_further_out(pack, clock):
    """The YAM rests at Home with joint 2 at -0.0013 rad (soft lower limit 0.01): a segment may keep it there or move
    it inward; one that pushes it further out is refused (inward-only past a limit)."""
    home = list(pack.arm("left").home_q)
    r = Rig(pack, clock, left_q=home)
    n = 31
    keep = np.tile(np.r_[home, r.left.jaw], (n, 1))
    keep[:, 0] += np.linspace(0, 0.02, n)
    assert reason(r.rt.submit(r.segment(keep, seq=1, sid="keep"))) == "accepted"
    r.run_to_end()
    q = np.asarray(r.rt.samples["left"].q)
    out = np.tile(np.r_[q, r.left.jaw], (n, 1))
    out[:, 1] = q[1] - np.linspace(0, 0.03, n)
    assert reason(r.rt.submit(r.segment(out, seq=2, sid="out", plan_basis_obs="obs_sim"))) == "joint_limit"
