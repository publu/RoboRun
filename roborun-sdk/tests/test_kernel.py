"""K2 link watchdog, K3 limits, K6 health, K8 interlocks."""
import json
import time

import numpy as np
import pytest

from roborun_sdk.rt.datatypes import ArmSample, OperatorConfirm, Refused
from roborun_sdk.rt.kernel import limits as k3
from roborun_sdk.rt.kernel.interlocks import Interlocks

from conftest import Rig


# ---------------------------------------------------------------- K2 link loss

def test_link_late_then_lost_holds_and_never_resumes(rig):
    rig.rt.submit(rig.segment(seq=1, sid="l", seconds=3.0))
    rig.drive(0.2)
    rig.drive(0.55, feed_link=False)                   # local transport: late at 0.5 s
    assert rig.rt.link.state == "LATE"
    assert rig.rt.playback.state == "running"          # the running segment continues under every local check
    assert rig.rt.submit(rig.segment(seq=2, sid="new"))["reason"] in ("link_late", "another_arm_moving")
    rig.drive(0.6, feed_link=False)                    # lost at 1.0 s
    assert rig.rt.link.state == "HELD" and rig.rt.link.latched_lost
    assert rig.rt.playback.outcome == "link_lost"
    assert all(g is None for g in rig.rt.grants.values())
    n = len(rig.left.writes)
    rig.drive(0.5)                                     # heartbeats again: still nothing moves, latch stays
    assert len(rig.left.writes) == n and rig.rt.link.latched_lost
    assert rig.rt.submit(rig.segment(seq=3, sid="x"))["reason"] == "link_lost"
    rig.rt.session_started("session-2")                # BACK: a new session clears the latch ...
    assert not rig.rt.link.latched_lost
    assert rig.rt.submit(rig.segment(seq=4, sid="y"))["reason"] == "plan_basis_stale"   # ... old plans stay refused
    assert any(e.get("kind") == "link_back" for e in rig.events())


def test_admit_may_only_tighten_the_threshold(rig):
    rig.rt.session_started("s", transport="wss", link_lost_ms=5000)
    assert rig.rt.link.lost_after_s == pytest.approx(1.0)
    rig.rt.session_started("s2", transport="wss", link_lost_ms=600)
    assert rig.rt.link.lost_after_s == pytest.approx(0.6)


# ---------------------------------------------------------------- K3 limits

def _sample(q, jaw=0.5):
    return ArmSample("left", "s", 0.0, 0.0, tuple(q), tuple(q), None, jaw, jaw)


def test_k3_joint_limit_inward_only(pack):
    ap = pack.arm("left")
    q = ap.upper.copy() - 0.05
    q[3] = ap.upper[3] + 0.02                          # joint 4 already past its limit
    target = np.r_[q, 0.5]
    target[3] = q[3] + 0.005                           # further out: clamped to where it is
    chk = k3.check_target(ap, target, last7=np.r_[q, 0.5], measured=_sample(q), dt=1 / 30)
    assert chk.target[3] == pytest.approx(q[3]) and "joint_limit" in chk.clamps
    target[3] = q[3] - 0.005                           # inward: allowed
    chk = k3.check_target(ap, target, last7=np.r_[q, 0.5], measured=_sample(q), dt=1 / 30)
    assert chk.target[3] == pytest.approx(q[3] - 0.005)


def test_k3_step_and_point_clamps(pack):
    ap = pack.arm("left")
    q = ap.home_q.copy()
    target = np.r_[q, 0.5]
    target[0] += 0.05                                  # 0.05 rad in one 1/30 s tick
    chk = k3.check_target(ap, target, last7=np.r_[q, 0.5], measured=_sample(q), dt=1 / 30,
                          seg_limits={"ee_speed_m_s": 0.20})
    assert "joint_step" in chk.clamps and abs(chk.target[0] - q[0]) <= 0.02 + 1e-9
    travel = ap.geometry.max_point_displacement(q, chk.target[:6])
    assert travel <= 0.20 / 30 + 1e-9                  # EE / per-point bound on every capsule endpoint
    # without a chain the conservative reach bound applies
    from dataclasses import replace
    blind = replace(ap, geometry=None, point_reach_m=np.array([0.65, 0.6, 0.35, 0.2, 0.15, 0.12]))
    chk2 = k3.check_target(blind, target, last7=np.r_[q, 0.5], measured=_sample(q), dt=1 / 30,
                           seg_limits={"ee_speed_m_s": 0.20})
    assert np.sum(blind.point_reach_m * np.abs(chk2.target[:6] - q)) <= 0.20 / 30 + 1e-9


def test_k3_jaw_lead_and_ceiling(pack):
    ap = pack.arm("left")
    q = ap.home_q.copy()
    target = np.r_[q, 0.2]
    chk = k3.check_target(ap, target, last7=np.r_[q, 0.5], measured=_sample(q, 0.5), dt=1 / 30)
    assert chk.target[6] >= 0.5 - 0.03 - 1e-9 and "jaw" in chk.clamps
    target = np.r_[q, 0.9]
    chk = k3.check_target(ap, target, last7=np.r_[q, 0.5], measured=_sample(q, 0.5), dt=1 / 30, jaw_ref=0.5,
                          jaw_allow=0.005)
    assert chk.target[6] <= 0.505 + 1e-9


def test_k3_without_any_point_bound_refuses(pack):
    from dataclasses import replace
    ap = replace(pack.arm("left"), geometry=None, point_reach_m=None)
    q = pack.arm("left").home_q
    t = np.r_[q, 0.5]
    t[0] += 0.01
    assert k3.check_target(ap, t, last7=np.r_[q, 0.5], measured=_sample(q), dt=1 / 30).refused == "no_point_bound"


def test_k3_nonfinite_refused(pack):
    ap = pack.arm("left")
    q = ap.home_q.copy()
    chk = k3.check_target(ap, np.r_[q, np.nan], last7=None, measured=_sample(q), dt=1 / 30)
    assert chk.refused == "nonfinite"


# ---------------------------------------------------------------- K6 health

def test_fault_latches_disarms_holds_and_refuses(rig):
    rig.rt.submit(rig.segment(seq=1, sid="f", seconds=2.0))
    rig.drive(0.2)
    rig.left.inject(offline=True, cause="motor_comm_lost:4")
    rig.drive(1 / 30)
    assert rig.rt.playback.outcome == "fault"
    assert not rig.rt.armed and rig.rt.health.faults() == {"left": "motor_comm_lost:4"}
    ev = [e for e in rig.events() if e.get("kind") == "arm_offline"]
    assert ev and ev[0]["latched"] and ev[0]["cause"] == "motor_comm_lost:4"
    assert any(h.startswith("fault") for h in rig.right.holds)       # the rig holds
    rig.left.faults.clear()
    rig.drive(0.5)
    assert rig.rt.submit(rig.segment(seq=2, sid="g"))["reason"] == "fault_latched"
    rig.rt.health.clear("left", OperatorConfirm("op", "recovered", 0.0), rig.clock.now())
    rig.rt.armed_local = True
    rig.drive(0.5)
    rig.rt.register_observation("obs_after", rig.clock.now())
    assert rig.rt.submit(rig.segment(seq=3, sid="h", plan_basis_obs="obs_after"))["result"] == "accepted"


def test_thermal_stop_work_ends_segment(rig):
    rig.rt.submit(rig.segment(seq=1, sid="t", seconds=2.0))
    rig.drive(0.2)
    rig.left.temps = [76.0] * 7
    rig.drive(0.1)
    assert rig.rt.playback.outcome == "thermal"


# ---------------------------------------------------------------- K8 interlocks

def test_interlocks_persist_expire_and_seed(tmp_path):
    path = tmp_path / "interlocks.json"
    wall = [1000.0]
    il = Interlocks(path, ("left", "right"), wall=lambda: wall[0])
    il.set_held("left", jaw=0.8, tool_z_m=0.21, evidence="close ended on contact", source="kernel")
    again = Interlocks(path, ("left", "right"), wall=lambda: wall[0] + 3600)
    assert again.held("left")["jaw"] == 0.8
    stale = Interlocks(path, ("left", "right"), wall=lambda: wall[0] + 13 * 3600)
    assert stale.held("left") is None                                   # kept at most 12 h
    with pytest.raises(Refused):
        again.clear("left", reason="verified_release", measured_jaw=0.8, open_value=1.0)
    again.clear("left", reason="verified_release", measured_jaw=0.99, open_value=1.0)
    assert again.held("left") is None
    memory = tmp_path / "jaw-memory.json"
    memory.write_text(json.dumps({"right": {"jaw": 0.803, "t": wall[0] - 60, "z": 0.2, "name": "orange cup",
                                            "release_pending": {"why": "rim"}}}))
    seeded = Interlocks(tmp_path / "b.json", ("left", "right"), wall=lambda: wall[0])
    assert sorted(seeded.seed_from_jaw_memory(memory)) == ["right: held", "right: pending_release"]
    assert memory.read_text().startswith('{"right"')                     # read-only: never rewritten


def test_held_blocks_jaw_opening_home(rig):
    rig.left.jaw_cmd = rig.left.jaw = 0.5
    rig.drive(0.5)
    rig.rt.interlocks.set_held("left", jaw=rig.left.jaw, tool_z_m=None, evidence="test", source="test")
    q = rig.left.q.copy()
    rows = np.tile(np.r_[q, rig.left.jaw], (31, 1))
    rows[:, 0] += np.linspace(0, 0.02, 31)
    opening = rows.copy()
    opening[:, -1] = np.linspace(0.5, 0.6, 31)
    ack = rig.rt.submit(rig.segment(opening, seq=1, sid="home_open", label="home", limits={"ee_speed_m_s": 0.08}))
    assert ack["result"] == "refused" and ack["reason"] == "held_interlock", ack
    ack = rig.rt.submit(rig.segment(rows, seq=2, sid="home", label="home", limits={"ee_speed_m_s": 0.08}))
    assert ack["result"] == "accepted", ack                            # a non-opening home is fine while held


# ---------------------------------------------------------------- K2 pack admission + signed controls

def test_offer_pack_activates_tighter_and_stages_loosening(rig):
    import copy
    from roborun_sdk import dev
    rig.rt.root_keys = dev.dev_root_keys()
    raw = copy.deepcopy(dev.sim_pack_raw())
    raw.pop("sig")
    raw["arms"]["left"]["limits"]["vel_rad_s"] = 0.5
    rec = rig.rt.offer_pack(dev.sign_pack(raw))
    assert rec["result"] == "activated" and rig.rt.pack.arm("left").vel_rad_s == 0.5
    raw2 = copy.deepcopy(raw)
    raw2["arms"]["left"]["limits"]["vel_rad_s"] = 0.9
    signed = dev.sign_pack(raw2)
    rec = rig.rt.offer_pack(signed)
    assert rec["result"] == "staged" and rig.rt.pack.arm("left").vel_rad_s == 0.5
    rec = rig.rt.offer_pack(signed, confirm=OperatorConfirm("operator", "approve loosening", 0.0))
    assert rec["result"] == "activated" and rig.rt.pack.arm("left").vel_rad_s == 0.9
    forged = copy.deepcopy(signed)
    forged["arms"]["left"]["limits"]["vel_rad_s"] = 5.0
    with pytest.raises(Refused):
        rig.rt.offer_pack(forged)


def test_signed_set_interlock_control(rig):
    from roborun_sdk import dev
    msg = {"type": "control", "seq": 4191, "op": "set_interlock", "arm": "left", "held": True,
           "evidence": "gate:seg_30c7_05:rise_82mm"}
    with pytest.raises(Refused):
        rig.rt.control(msg)                                   # unsigned: refused
    out = rig.rt.control(dev.DEV_SEGMENTS.sign(msg, "control"))
    assert out["held"]["evidence"].startswith("gate:") and rig.rt.interlocks.held("left")
    hold = rig.rt.control({"type": "control", "seq": 1, "op": "hold"})   # hold only tightens: obeyed unsigned
    assert set(hold["arms"]) == {"left", "right"}
