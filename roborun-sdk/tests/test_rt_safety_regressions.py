"""Safety regressions of the runtime: one section per rule."""
import copy
import datetime
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from roborun_sdk import cli, dev
from roborun_sdk.connectors.sim import JAW_READ_NOISE
from roborun_sdk.link import wire_compat
from roborun_sdk.link.pack_cache import PackCache
from roborun_sdk.link.rtclient import InProcessRt, RtClient
from roborun_sdk.rt import signing
from roborun_sdk.rt.clock import Clock, SimClock
from roborun_sdk.rt.datatypes import ArmSample, OperatorConfirm, Refused
from roborun_sdk.rt.kernel import limits as k3
from roborun_sdk.rt.kernel.proximity import CapsuleProximity
from roborun_sdk.rt.pack import PackStore, compare, parse_pack
from roborun_sdk.rt.server import RtServer, stop_message

from conftest import EXAMPLES, Rig
from fake_owners import FakeMetalOwner, FakeYamStation


def reason(ack):
    return ack.get("reason") if ack.get("result") == "refused" else ack.get("result")


def resign(seg, **valid_if):
    seg = dict(seg)
    seg.pop("sig")
    seg["valid_if"] = dict(seg["valid_if"], **valid_if)
    return dev.DEV_SEGMENTS.sign(seg, "segment")


def variant(mutate):
    raw = copy.deepcopy(dev.sim_pack_raw())
    raw.pop("sig")
    mutate(raw)
    return parse_pack(dev.sign_pack(raw), signed=True)


def flaky(owner, path, failures):
    """Make the fake owner answer ``path`` with 503 for the next ``failures`` GETs (a transient glitch)."""
    real_get = owner.get
    left = {"n": failures}

    def get(p):
        if p == path and left["n"] > 0:
            left["n"] -= 1
            return 503, {"error": "busy"}
        return real_get(p)
    owner.get = get
    return left


# ---------------------------------------------------------------- rt --live never auto-POSTs on its own

@pytest.fixture
def owners():
    yam = FakeYamStation(lease={"id": "L" * 32, "active": True, "kind": "video_motion"}, state="running")
    metal = FakeMetalOwner(state="moving", motion={"kind": "motion"})
    yield yam, metal
    yam.close()
    metal.close()


def _live_rt(yam, metal, pack):
    clock = Clock()
    conns = cli.live_connectors(clock, allow_post=True, yam_url=yam.url, metal_url=metal.url,
                                sysroot=Path("/nonexistent"))
    rt = cli.live_runtime(conns, pack, clock=clock, proximity=CapsuleProximity(pack))
    return rt, conns


def test_live_rt_wiring_sends_no_post_on_startup_glitches_or_overruns(owners, pack):
    yam, metal = owners
    rt, conns = _live_rt(yam, metal, pack)
    stop = threading.Event()
    try:
        rt.tick()                                              # the first tick right after start: first polls answered
        assert rt.health.faults() == {}
        th = threading.Thread(target=rt.run, args=(stop,), daemon=True)
        th.start()
        time.sleep(0.4)
        flaky(yam, "/supervisor/pose", 1)                      # ONE failed pose GET
        flaky(metal, "/status", 3)                             # three failed status GETs (~100 ms)
        time.sleep(0.6)
        assert rt.health.faults() == {}                        # transient: STALE at most, never latched OFFLINE
        rt.next_tick_t = rt.clock.now() - 0.06                 # a tick overrun while rt commands nothing
        stop.set()
        th.join(2.0)
        rt.tick()
        assert any(e.get("kind") == "tick_overrun" and e.get("rig_hold") is False for e in rt.drain_events())
        assert yam.posts() == [] and metal.posts() == []
        assert yam.lease["active"] is True and metal.motion == {"kind": "motion"}   # the owner's own motion untouched
    finally:
        stop.set()
        for c in conns.values():
            c.close()


def test_live_rt_sustained_outage_latches_but_still_never_posts(owners, pack):
    yam, metal = owners
    rt, conns = _live_rt(yam, metal, pack)
    stop = threading.Event()
    try:
        th = threading.Thread(target=rt.run, args=(stop,), daemon=True)
        th.start()
        time.sleep(0.3)
        flaky(yam, "/supervisor/pose", 10_000)                 # the station stops answering
        deadline = time.time() + 3.0
        while not rt.health.faults() and time.time() < deadline:
            time.sleep(0.05)
        assert rt.health.faults() == {"left": "owner_unreachable"} and not rt.armed
        time.sleep(0.2)
        assert yam.posts() == [] and metal.posts() == []      # latched, refusing, disarmed: but it commands nothing
        report = rt.stop("lan_udp")                           # an explicit Stop reaches every owner
        assert "/stop" in metal.posts() and report["arms"]["right"]["holding"] is True
        assert set(yam.posts()) <= {"/supervisor/hold", "/supervisor/stop", "/stop"}
    finally:
        stop.set()
        for c in conns.values():
            c.close()


def test_live_rt_refuses_dev_signatures(owners, pack):
    yam, metal = owners
    rt, conns = _live_rt(yam, metal, pack)
    try:
        assert set(dev.DEV_KIDS) <= rt.refused_kids and rt.trusted_keys == {}
        rt.armed_local = rt.armed_server = True
        rt.session_started("s")
        q = list(conns["left"].sample().q)
        seg = dev.make_segment([[*q, 0.999]] * 31, semantics=pack.arm("left").semantics, pack_digest=pack.digest)
        assert reason(rt.submit(seg)) == "bad_signature"
        ctl = dev.DEV_SEGMENTS.sign({"type": "control", "seq": 7, "op": "set_interlock", "arm": "left", "held": False,
                                     "evidence": "forged"}, "control")
        with pytest.raises(Refused) as exc:
            rt.control(ctl)
        assert exc.value.reason == "bad_signature"
        rt.root_keys = dev.dev_root_keys()
        with pytest.raises(Refused) as exc:
            rt.offer_pack(dev.sim_pack_raw())
        assert exc.value.reason == "no_root_keys"
    finally:
        for c in conns.values():
            c.close()


def test_rt_live_process_against_fakes_sends_no_post(owners, tmp_path):
    """`roborun rt --live` itself (the real cmd_rt wiring) against fake owners: a startup race, one failed pose GET
    and a short Metal outage cause no POST; the running line names the refused DEV kids and the console token."""
    yam, metal = owners
    flaky(yam, "/supervisor/pose", 0)
    src = str(EXAMPLES.parent / "src")
    env = dict(os.environ, PYTHONPATH=src, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
    proc = subprocess.Popen([sys.executable, "-m", "roborun_sdk.cli", "rt", "--live", "--dev-keys-ok", "--geometry",
                             "capsule", "--yam-url", yam.url, "--metal-url", metal.url, "--sysroot", "/nonexistent",
                             "--seconds", "2.5", "--stop-port", "0", "--socket", str(tmp_path / "rt.sock")],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
    try:
        first = json.loads(proc.stdout.readline())
        assert first["mode"] == "LIVE" and set(dev.DEV_KIDS) <= set(first["refused_kids"]) and first["console_token"]
        time.sleep(0.5)
        flaky(yam, "/supervisor/pose", 1)
        flaky(metal, "/status", 4)
        out, err = proc.communicate(timeout=15)
        assert proc.returncode == 0, err[-2000:]
    finally:
        if proc.poll() is None:
            proc.kill()
    assert yam.posts() == [] and metal.posts() == []
    assert yam.lease["active"] is True and metal.motion == {"kind": "motion"}


# ---------------------------------------------------------------- K3 limits never widen to planned rows

def test_chained_segments_cannot_ratchet_a_past_limit_joint(pack, clock):
    home = list(pack.arm("left").home_q)                       # j2 -0.0013, below the 0.01 soft and 0.0 hard limit
    r = Rig(pack, clock, left_q=home)
    q = np.asarray(r.rt.samples["left"].q)
    rows = np.tile(np.r_[q, r.left.jaw], (31, 1))
    rows[:, 1] = q[1] - 0.0095                                 # within tol_rad of the measured start, further out
    rows[:, 0] = q[0] + np.linspace(0, 0.002, 31)
    ack = r.rt.submit(r.segment(rows, seq=1, sid="out", start_q=list(q)))
    assert reason(ack) == "joint_limit", ack
    wide = r.segment(rows * [1, 1, 1, 1, 1, 1, 1] + [0, -0.0855, 0, 0, 0, 0, 0], seq=2, sid="tol01", start_q=list(q))
    assert reason(r.rt.submit(resign(wide, tol_rad=0.1))) == "joint_limit"
    keep = np.tile(np.r_[home, r.left.jaw], (31, 1))           # holding j2 at its Home value is fine (noise-proof)
    keep[:, 0] += np.linspace(0, 0.02, 31)
    assert reason(r.rt.submit(r.segment(keep, seq=3, sid="keep"))) == "accepted"
    r.run_to_end()
    assert r.rt.playback.outcome == "complete" and r.left.q[1] >= home[1] - 0.0005


def test_k3_per_write_bounds_ignore_the_planned_start(pack):
    ap = pack.arm("left")
    q = ap.home_q.copy()
    q[1] = -0.0011                                             # measured a little inward of Home
    target = np.r_[q, 0.5]
    target[1] = -0.02                                          # further out than both measured and Home
    s = ArmSample("left", "s", 0.0, 0.0, tuple(q), tuple(q), None, 0.5, 0.5)
    chk = k3.check_target(ap, target, last7=np.r_[q, 0.5], measured=s, dt=1 / 30)
    assert chk.target[1] == pytest.approx(ap.home_q[1]) and "joint_limit" in chk.clamps


# ---------------------------------------------------------------- the jaw never opens in a move

def test_jaw_tol_is_bounded_by_the_pack(rig):
    seg = rig.segment(seq=1, sid="jt")
    assert reason(rig.rt.submit(resign(seg, jaw_tol=0.05))) == "loosening_limits"
    assert reason(rig.rt.submit(resign(rig.segment(seq=2, sid="jt2"), jaw_tol=1.0))) == "loosening_limits"


def test_held_blocks_jaw_opening_for_any_label(pack, clock):
    r = Rig(pack, clock, left_jaw=0.5)
    r.rt.interlocks.set_held("left", jaw=0.5, tool_z_m=None, evidence="cup", source="test")
    q = np.asarray(r.rt.samples["left"].q)
    rows = np.tile(np.r_[q, 0.5], (61, 1))
    rows[:, -1] = 0.5 + np.linspace(0, 0.45, 61)
    rows[:, 0] = q[0] + np.linspace(0, 0.01, 61)
    for i, label in enumerate(("carry", "move", "home", "park")):
        ack = r.rt.submit(r.segment(rows, seq=i + 1, sid=f"open_{label}", label=label, limits={"ee_speed_m_s": 0.08}))
        assert reason(ack) == "held_interlock", (label, ack)
    assert r.left.jaw == pytest.approx(0.5)


def test_a_move_cannot_release_a_squeeze(pack, clock):
    """An object in the jaw: commanded 0.45 (squeeze), measured 0.50 (the object's width). Commanding the measured
    width would release the squeeze: refused; holding the command plays without ever opening past it."""
    r = Rig(pack, clock, left_jaw=0.45)
    r.left.inject(jaw_block=0.5)
    r.drive(0.6)
    r.observe()
    s = r.rt.samples["left"]
    # the jaw reading jitters by about one encoder quantum (connectors.sim.JAW_READ_NOISE), as on the real YAM
    assert s.jaw == pytest.approx(0.5, abs=3 * JAW_READ_NOISE) and s.jaw_cmd == pytest.approx(0.45)
    q = np.asarray(s.q)
    width = np.tile(np.r_[q, 0.5], (31, 1))
    width[:, 0] += np.linspace(0, 0.01, 31)
    ack = r.rt.submit(r.segment(width, seq=1, sid="release", jaw=0.5))
    assert reason(ack) in ("jaw_mismatch", "jaw_open_not_release"), ack
    keep = width.copy()
    keep[:, -1] = 0.45
    assert reason(r.rt.submit(r.segment(keep, seq=2, sid="keep", jaw=0.5))) == "accepted"
    r.run_to_end()
    assert r.rt.playback.outcome == "complete"
    assert max(w[1][-1] for w in r.left.writes) <= 0.45 + 0.003 + 1e-9


def test_missing_jaw_reading_refuses_the_start_and_the_write(rig):
    real_sample = rig.left.sample

    def no_jaw():
        s = real_sample()
        return ArmSample(s.arm, s.sample_id, s.t, s.clock_uncertainty_s, s.q, s.q_cmd, s.dq, None, None,
                         temps_c=s.temps_c, status=s.status, source=s.source, raw=s.raw)
    rig.left.sample = no_jaw
    rig.drive(0.1)
    assert reason(rig.rt.submit(rig.segment(seq=1, sid="nj"))) == "jaw_unknown"
    rig.left.sample = real_sample
    rig.drive(0.4)
    assert reason(rig.rt.submit(rig.segment(seq=2, sid="ok", seconds=2.0))) == "accepted"
    rig.drive(0.2)
    rig.left.sample = no_jaw
    rig.drive(0.1)
    assert rig.rt.playback.outcome == "limit:jaw_unknown"
    assert any(h == "segment_limit:jaw_unknown" for h in rig.left.holds)


# ---------------------------------------------------------------- a relayed Stop latches whatever its signature

BAD_SIGS = ({"alg": "Ed25519", "kid": ["k"], "value": "AAAA"}, {"alg": "Ed25519", "kid": {"a": 1}, "value": "AAAA"},
            "garbage", ["x"], {"alg": ["Ed25519"], "kid": "k"}, {"alg": "Ed25519", "kid": dev.DEV_SEGMENTS.kid,
                                                                  "value": ["AAAA"]},
            {"alg": "Ed25519", "kid": dev.DEV_SEGMENTS.kid, "value": "not base64 !!"}, 7, None)


@pytest.mark.parametrize("sig", BAD_SIGS)
def test_malformed_stop_signatures_never_block_the_latch(pack, clock, sig):
    msg = {"type": "stop", "id": "stp_x", "source": "hosted_ui", "sig": sig}
    r = Rig(pack, clock)
    report = stop_message(r.rt, msg)
    assert r.rt.stop_latch.latched and report["verified"] is False and report["not_confirmed"] == []
    r2 = Rig(pack, SimClock())
    InProcessRt(r2.rt).stop(msg)
    assert r2.rt.stop_latch.latched


def test_stop_latches_before_its_signature_is_checked(rig, monkeypatch):
    seen = {}
    real = signing.verify

    def spy(msg, kind, keys):
        seen["latched_at_verify"] = rig.rt.stop_latch.latched
        return real(msg, kind, keys)
    monkeypatch.setattr(signing, "verify", spy)
    signed = dev.DEV_SEGMENTS.sign({"type": "stop", "id": "stp_ok", "source": "hosted_ui", "scope": "rig"}, "stop")
    report = stop_message(rig.rt, signed)
    assert seen["latched_at_verify"] is True and report["verified"] is True
    assert stop_message(rig.rt, ["not", "a", "map"])["source"] == "api:relay"     # not even a map: still a Stop


def test_verify_is_total():
    for msg in (None, [], {"sig": {"alg": "Ed25519", "kid": [1]}}, {"sig": {"alg": "Ed25519", "kid": "k", "value": 5}},
                {"type": "segment", "sig": {"alg": "Ed25519", "kid": "k", "value": "AAAA"}}):
        ok, why = signing.verify(msg, "segment", {"k": bytes(32)})
        assert ok is False and isinstance(why, str)


def test_stop_over_the_socket_with_a_malformed_sig(rig, tmp_path):
    server = RtServer(rig.rt, str(tmp_path / "rt.sock")).start()
    try:
        client = RtClient(str(tmp_path / "rt.sock"))
        rep = client.stop({"type": "stop", "id": "stp_s", "sig": {"alg": "Ed25519", "kid": ["k"]}})
        assert rig.rt.stop_latch.latched and rep["verified"] is False
        client.close()
    finally:
        server.close()


# ---------------------------------------------------------------- a segment may only tighten the rest test

def test_rest_spec_can_only_tighten(rig):
    rig.left.q_cmd = rig.left.q + 0.01
    rig.drive(0.1)                                            # the arm is moving
    q = np.asarray(rig.rt.samples["left"].q)
    rows = np.tile(np.r_[q, rig.left.jaw], (31, 1))
    assert reason(rig.rt.submit(rig.segment(rows, seq=1, sid="d"))) == "not_at_rest"
    short = resign(rig.segment(rows, seq=2, sid="short"), rest={"window_s": 0.001, "range_rad": 0.0015})
    assert reason(rig.rt.submit(short)) == "loosening_limits"
    wide = resign(rig.segment(rows, seq=3, sid="wide"), rest={"window_s": 0.3, "range_rad": 5.0})
    assert reason(rig.rt.submit(wide)) == "loosening_limits"
    rig.drive(1.0)                                            # settled again
    rig.observe()
    q = np.asarray(rig.rt.samples["left"].q)
    rows = np.tile(np.r_[q, rig.left.jaw], (31, 1))
    rows[:, 0] += np.linspace(0, 0.01, 31)
    tight = resign(rig.segment(rows, seq=4, sid="tight"), rest={"window_s": 0.5, "range_rad": 0.001})
    assert reason(rig.rt.submit(tight)) == "accepted"


# ---------------------------------------------------------------- end gates mean what they say

def test_verify_gate_is_evaluated(rig):
    lift = rig.segment(seq=1, sid="lift", delta=0.01, end_gate={"kind": "verify", "hold": True, "require":
                       {"sig": "tool_z", "stat": "delta_since_start", "op": ">=", "v": 0.5}})
    assert reason(rig.rt.submit(lift)) == "accepted"
    rig.run_to_end()
    report = [e for e in rig.rt.drain_events() if e.get("type") == "gate_report"][-1]
    assert report["gate"] == "verify" and report["outcome"] == "verify_failed"
    assert report["evidence"]["require"]["passed"] is False
    assert wire_compat.gate_report(report, 1)["outcome"] == "failed"
    rig.drive(0.5)
    rig.observe()
    ok = rig.segment(seq=2, sid="ok", delta=0.01, end_gate={"kind": "verify", "hold": True, "require":
                     {"sig": "jaw", "op": ">=", "v": 0.9}})
    assert reason(rig.rt.submit(ok)) == "accepted"
    rig.run_to_end()
    assert rig.rt.playback.outcome == "complete"


def test_unevaluable_verify_gates_are_refused(rig):
    for i, req in enumerate(({"any": [{"sig": "jaw", "op": ">", "v": 0.5}]},
                             {"sig": "jaw", "stat": "range", "w": 0.4, "op": "<", "v": 0.03},
                             {"sig": "press", "op": ">", "v": 1}, None)):
        seg = rig.segment(seq=i + 1, sid=f"v{i}", end_gate={"kind": "verify", "hold": True, "require": req}, key=None)
        assert reason(rig.rt.submit(seg)) == "unsupported_gate", req          # parsed before the signature counts


def test_complete_requires_arrival(rig):
    q0 = rig.left.q.copy()
    assert reason(rig.rt.submit(rig.segment(seq=1, sid="stall", delta=0.04, seconds=1.0))) == "accepted"
    rig.drive(0.5)
    rig.left.tau_s = 1e9                                       # blocked: the arm stops, commands keep coming
    rig.run_to_end()
    report = [e for e in rig.rt.drain_events() if e.get("type") == "gate_report"][-1]
    assert report["outcome"] == "not_arrived" and report["evidence"]["arrival_err_rad"] > 0.01
    assert report["evidence"]["arrived"] is False and float(rig.left.q[0] - q0[0]) < 0.03
    assert wire_compat.gate_report(report, 1)["outcome"] == "failed"
    assert any(h == "segment_not_arrived" for h in rig.left.holds)


def test_metal_completes_within_its_arrival_rad(pack, clock):
    # the bundled sim pack's keep-in box is built from the rig, so it includes the Metal's own Home (its link2 capsule
    # reaches y = -0.323) and the bundled pack plays it as is
    r = Rig(pack, clock)
    seg = r.segment(arm="right", seq=1, sid="m", delta=0.02, seconds=4.0, limits={"ee_speed_m_s": 0.04})
    assert reason(r.rt.submit(seg)) == "accepted"
    r.run_to_end()
    report = [e for e in r.rt.drain_events() if e.get("type") == "gate_report"][-1]
    assert report["outcome"] == "complete" and report["evidence"]["arrival_tol_rad"][4] == pytest.approx(0.0262)


# ---------------------------------------------------------------- pack compare uses enforced values

def test_added_loose_keys_are_loosenings(pack):
    fast = variant(lambda r: r["arms"]["right"].__setitem__("tcp_m_s", {"base": 1.0, "free_max": 2.0, "loaded_max": 2.0}))
    d = compare(pack, fast)
    assert not d.tighter and "arms.right.tcp_m_s.free_max" in d.loosened
    assert PackStore(pack).offer(fast, at_rest=True, now=1.0)["result"] == "staged"
    slow = variant(lambda r: r["arms"]["right"].__setitem__("tcp_m_s", {"base": 0.03, "free_max": 0.03,
                                                                        "loaded_max": 0.03}))
    assert compare(pack, slow).tighter
    hot_yam = variant(lambda r: r["arms"]["left"]["thermal_c"].update({"refuse_start": 95, "stop_work": [98, 99]}))
    d = compare(pack, hot_yam)
    assert not d.tighter and "arms.left.thermal.refuse_start" in d.loosened
    bare = variant(lambda r: r["arms"]["right"].pop("thermal_c"))       # defaults 70 / 75 apply
    hot = variant(lambda r: r["arms"]["right"].__setitem__("thermal_c", {"refuse_start": 95, "stop_work": [98, 99]}))
    assert not compare(bare, hot).tighter
    capped = variant(lambda r: r["arms"]["right"]["thermal_c"].update({"refuse_start": 95, "stop_work": [98, 99]}))
    assert compare(pack, capped).tighter            # the Metal's native holds (65 / 55 C) still cap what is enforced
    assert capped.arm("right").thermal["stop_work"] == 65 and capped.arm("right").thermal["jaw_stop_work"] == 55
    raw = copy.deepcopy(dev.sim_pack_raw())                 # late_after_s is SDK-local (not in the wire's Link)
    raw["link"]["late_after_s"] = {"wss": 0.9}
    assert "link.late_after_s.wss" in compare(pack, parse_pack(raw)).loosened
    far_home = variant(lambda r: r["arms"]["left"]["home"]["q"].__setitem__(1, -0.05))
    assert "arms.left.home_limit_lower" in compare(pack, far_home).loosened
    with pytest.raises(Refused):
        variant(lambda r: r["arms"]["left"]["home"]["q"].__setitem__(1, -0.5))   # beyond the 5 degree slack


# ---------------------------------------------------------------- K6 follows pack activation (+ at rest after a segment)

def test_health_follows_an_activated_pack_even_after_a_segment(rig):
    assert reason(rig.rt.submit(rig.segment(seq=1, sid="first"))) == "accepted"
    rig.run_to_end()
    assert rig.rt.at_rest_all()                               # an ended playback is at rest
    rig.rt.root_keys = dev.dev_root_keys()
    raw = copy.deepcopy(dev.sim_pack_raw())
    raw.pop("sig")
    raw["arms"]["left"]["thermal_c"].update({"refuse_start": 50, "stop_work": [55, 60]})
    rec = rig.rt.offer_pack(dev.sign_pack(raw))
    assert rec["result"] == "activated"
    rig.left.temps = [52.0] * 7
    rig.drive(0.2)
    assert rig.rt.health.arms["left"].thermal == "refuse_start"
    rig.observe()
    assert reason(rig.rt.submit(rig.segment(seq=2, sid="warm", pack_digest=rig.rt.pack.digest))) == "too_hot"


# ---------------------------------------------------------------- the loaded EE ceiling applies while held

def test_loaded_ee_ceiling_while_held(rig, pack):
    ap = pack.arm("left")
    assert k3.ee_cap(ap, {"ee_speed_m_s": 0.20}, None, "local") == pytest.approx(0.20)
    assert k3.ee_cap(ap, {"ee_speed_m_s": 0.20}, None, "local", loaded=True) == pytest.approx(0.08)
    rig.rt.interlocks.set_held("left", jaw=rig.left.jaw, tool_z_m=None, evidence="cup", source="test")
    assert reason(rig.rt.submit(rig.segment(seq=1, sid="fast", limits={"ee_speed_m_s": 0.10}))) == "loosening_limits"
    assert reason(rig.rt.submit(rig.segment(seq=2, sid="ok", limits={"ee_speed_m_s": 0.08}))) == "accepted"
    assert rig.rt.grants["left"].speed_cap_m_s == pytest.approx(0.08)


# ---------------------------------------------------------------- goaway is an orderly link loss

def test_goaway_holds_the_running_segment_and_latches(rig):
    assert reason(rig.rt.submit(rig.segment(seq=1, sid="long", seconds=4.0, delta=0.06))) == "accepted"
    rig.drive(0.2)
    rig.rt.control(dev.DEV_SEGMENTS.sign({"type": "control", "seq": 5, "op": "goaway"}, "control"))
    assert rig.rt.playback.outcome == "link_lost" and rig.rt.link.latched_lost
    n = len(rig.left.writes)
    rig.drive(2.0, feed_link=False)
    assert len(rig.left.writes) == n and "segment_link_lost" in rig.left.holds
    assert reason(rig.rt.submit(rig.segment(seq=6, sid="after"))) == "link_lost"


# ---------------------------------------------------------------- an unpowered arm is not held

def test_unpowered_arm_is_reported_not_holding(rig):
    from roborun_sdk.rt.datatypes import HoldAck
    rig.right.hold = lambda reason: HoldAck("right", False, None, 0.001, False, {"state": "observing"})
    report = rig.rt.stop("cli")
    right = report["arms"]["right"]
    assert right["confirmed"] is False and "nothing holds it" in right["console"] and "right" in report["not_confirmed"]
    wire = wire_compat.stop_answer(dict(report, sdk_t=1.0), 3)
    assert wire["kind"] == "stop_not_confirmed" and wire["arms"]["right"]["holding"] is False
    assert "support the arm" in wire["arms"]["right"]["detail"]


# ---------------------------------------------------------------- the pack cache uses the segments' address

def test_pack_cache_is_addressed_by_the_pack_digest(tmp_path, pack):
    cache = PackCache(tmp_path, dev.dev_root_keys())
    assert cache.put(dev.sim_pack_raw()) == pack.digest
    assert cache.get(pack.digest)["pack"] == dev.sim_pack_raw()["pack"]


# ---------------------------------------------------------------- rt does not take the link's word

def test_observation_times_are_clamped_and_never_refreshed(rig):
    with pytest.raises(Refused):
        rig.rt.register_observation("obs_nan", float("nan"))
    rig.rt.register_observation("obs_future", rig.clock.now() + 100.0)
    assert rig.rt.observations["obs_future"] <= rig.clock.now()
    rig.rt.stop("cli")
    rig.rt.clear_stop(OperatorConfirm("op", "clear", rig.clock.now(), "test"))
    rig.drive(0.1)
    rig.rt.register_observation("obs_sim", rig.clock.now())   # re-registering an old id never makes it newer
    assert reason(rig.rt.submit(rig.segment(seq=1, sid="old", plan_basis_obs="obs_sim"))) == "plan_basis_stale"


def test_received_t_never_extends_expiry(rig, monkeypatch):
    from roborun_sdk.rt import acceptance
    seen = {}
    real = acceptance.check

    def spy(seg, ctx):
        seen["received_t"], seen["now"] = ctx.received_t, ctx.now
        return real(seg, ctx)
    monkeypatch.setattr(acceptance, "check", spy)
    rig.rt.submit(rig.segment(seq=1, sid="f"), received_t=rig.clock.now() + 50.0)
    assert seen["received_t"] <= seen["now"]


def test_signed_controls_are_ordered_and_never_replayed(rig):
    held = dev.DEV_SEGMENTS.sign({"type": "control", "seq": 10, "op": "set_interlock", "arm": "left", "held": True,
                                  "evidence": "gate:rise_82mm"}, "control")
    rig.rt.control(held)
    with pytest.raises(Refused) as exc:
        rig.rt.control(held)                                  # a replay
    assert exc.value.reason == "stale_seq"
    clear = dev.DEV_SEGMENTS.sign({"type": "control", "seq": 9, "op": "set_interlock", "arm": "left", "held": False,
                                   "evidence": "old"}, "control")
    with pytest.raises(Refused):
        rig.rt.control(clear)                                 # older than the last
    assert rig.rt.interlocks.held("left")
    assert reason(rig.rt.submit(rig.segment(seq=10, sid="s10", limits={"ee_speed_m_s": 0.08}))) == "stale_seq"


def test_socket_clear_stop_needs_the_console_token(rig, tmp_path):
    server = RtServer(rig.rt, str(tmp_path / "rt.sock"), console_token="tok-123").start()
    try:
        client = RtClient(str(tmp_path / "rt.sock"))
        rig.rt.stop("cli")
        for token in (None, "", "wrong"):
            with pytest.raises(Refused):
                client.clear_stop("operator", token)
        assert rig.rt.stop_latch.latched
        with pytest.raises(Refused):
            client._call("clear_stop", operator="link")      # the old form, with no token
        assert client.clear_stop("operator", "tok-123")["cleared"] is True and not rig.rt.stop_latch.latched
        client.close()
    finally:
        server.close()


def test_expired_segment_keys_never_verify(rig, monkeypatch):
    key = signing.SigningKey("seg-old", b"\x07" * 32)
    raw = copy.deepcopy(dev.sim_pack_raw())
    raw.pop("sig")
    raw["trusted_keys"] = [key.trusted_key(not_after="2020-01-01")]
    expired = parse_pack(dev.sign_pack(raw), signed=True)
    assert expired.trusted_keys == {}
    raw["trusted_keys"] = [key.trusted_key(not_after="2030-01-01")]
    pack = parse_pack(dev.sign_pack(raw), signed=True)
    r = Rig(pack, SimClock())
    seg = r.segment(seq=1, sid="k", key=key)
    monkeypatch.setattr(signing, "today_utc", lambda: datetime.date(2031, 1, 1))   # rt outlives the key's last day
    assert reason(r.rt.submit(seg)) == "bad_signature"
    monkeypatch.setattr(signing, "today_utc", lambda: datetime.date(2029, 1, 1))
    assert reason(r.rt.submit(r.segment(seq=2, sid="k2", key=key))) == "accepted"


# ---------------------------------------------------------------- real pack fields are mapped, not ignored

def test_real_pack_fields_are_mapped(rig, pack):
    right, left = pack.arm("right"), pack.arm("left")
    assert right.jaw.force_cap == pytest.approx(0.45)           # Metal trip_nm (effort_nm 0.30 stays native)
    assert right.thermal == {"refuse_start": 65, "stop_work": 65, "jaw_refuse_start": 55, "jaw_stop_work": 55}
    assert right.arrival_rad[4] == pytest.approx(0.0262)
    assert left.jaw.force_cap == pytest.approx(0.5) and left.thermal["refuse_start"] == 70
    assert left.thermal["stop_work"] == 75
    rig.right.temps = [40.0] * 6 + [56.0]                     # only the Metal's jaw motor is warm: past its 55 C hold
    rig.drive(0.1)
    assert rig.rt.health.arms["right"].thermal == "stop_work"
    rig.left.temps = [40.0] * 6 + [56.0]                      # the YAM's jaw motor at 56 C is fine (70 / 75)
    rig.drive(0.1)
    assert rig.rt.health.arms["left"].thermal == "ok"
