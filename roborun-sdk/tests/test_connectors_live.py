"""connector-yam / connector-metal against in-process fakes of their HTTP routes (loopback inside unshare -rn)."""
import time

import pytest

from roborun_sdk import dev
from roborun_sdk.connectors.metal import MetalOwnerConnector
from roborun_sdk.connectors.yam import YamStationConnector
from roborun_sdk.rt.clock import Clock
from roborun_sdk.rt.datatypes import NotWired, OperatorConfirm, ReadOnlyConnector, Refused
from roborun_sdk.rt.runtime import Runtime

from fake_owners import FakeMetalOwner, FakeYamStation

NEVER = ("/home", "/reset", "/start", "/supervisor/action", "/connect", "/prepare", "/move", "/lease/action", "/reload_owner")


@pytest.fixture
def yam():
    f = FakeYamStation()
    yield f
    f.close()


@pytest.fixture
def metal():
    f = FakeMetalOwner()
    yield f
    f.close()


def test_samples_parse_with_driver_stamps(yam, metal):
    y = YamStationConnector(yam.url, sysroot=__import__("pathlib").Path("/nonexistent"))
    s = y.poll_once()
    assert s.status == "holding" and s.source == "driver_state_cycle" and len(s.q) == 6 and s.jaw == 0.999
    assert y.sample() is s and y.health().state == "ok"
    m = MetalOwnerConnector(metal.url)
    s = m.poll_once()
    assert s.status == "holding" and s.effort_fresh and s.jaw == 1.9 and s.dq[0] == pytest.approx(0.0024)
    assert m.health().state == "ok"
    assert yam.posts() == [] and metal.posts() == []


def test_read_only_connectors_never_post(yam, metal):
    y = YamStationConnector(yam.url, allow_post=False)
    m = MetalOwnerConnector(metal.url, allow_post=False)
    ack = m.hold("x")
    assert ack.holding is None and "read-only" in ack.detail["error"]
    with pytest.raises(ReadOnlyConnector):
        y.http.post("/stop", {})
    y.hold("x")
    assert yam.posts() == [] and metal.posts() == []


def test_motion_is_not_wired(yam, metal):
    y = YamStationConnector(yam.url, allow_post=True)
    m = MetalOwnerConnector(metal.url, allow_post=True)
    for c in (y, m):
        with pytest.raises(NotWired):
            c.acquire(None)
        with pytest.raises(NotWired):
            c.write([0.0] * 7, 0.0)
    assert yam.posts() == [] and metal.posts() == []


def test_yam_hold_falls_back_to_lease_stop_and_acks_from_measured_state():
    f = FakeYamStation(state="running", lease={"id": "a" * 32, "active": True, "kind": "video_motion"})
    try:
        y = YamStationConnector(f.url, allow_post=True)
        ack = y.hold("stop:cli")
        assert ack.holding is True and ack.sample is not None
        assert f.posts() == ["/supervisor/hold", "/supervisor/stop"]
        stop_body = [b for m, p, _, b in f.requests if p == "/supervisor/stop"][0]
        assert stop_body == {"lease_id": "a" * 32}
        assert all(h.get("X-Manifest-Supervisor") == "1" for m, p, h, _ in f.requests if m == "POST")
    finally:
        f.close()


def test_yam_hold_prefers_the_lease_free_route():
    f = FakeYamStation(hold_route=True, state="running", lease={"id": "b" * 32, "active": True})
    try:
        ack = YamStationConnector(f.url, allow_post=True).hold("stop:ui")
        assert ack.holding is True and f.posts() == ["/supervisor/hold"]
    finally:
        f.close()


def test_yam_homing_is_reported_not_holding_never_assumed():
    f = FakeYamStation(state="homing")
    try:
        ack = YamStationConnector(f.url, allow_post=True).hold("stop:cli")
        assert ack.holding is False
        assert any("refuses Stop" in s.get("note", "") for s in ack.detail["steps"])
        assert "/stop" not in f.posts()                     # the vendor would refuse it while HOMING
    finally:
        f.close()


def test_yam_unreachable_is_not_confirmed():
    y = YamStationConnector("http://127.0.0.1:9", allow_post=True, hold_budget_s=0.3)
    ack = y.hold("stop:cli")
    assert ack.holding is None
    # one failed poll is not a fault (the sample ages into STALE); OFFLINE only after a sustained outage
    assert y.poll_once() is None and y.health().state == "degraded" and y.health().cause == "owner_poll_error"
    probe = YamStationConnector("http://127.0.0.1:9", allow_post=False, offline_after_s=0.0)   # doctor's one-shot probe
    assert probe.poll_once() is None and probe.health().state == "offline"
    assert probe.health().cause == "never_polled"


def test_metal_stop_and_unpowered(metal):
    m = MetalOwnerConnector(metal.url, allow_post=True)
    metal.state, metal.motion = "moving", {"kind": "home"}
    ack = m.hold("stop:cli")
    assert ack.holding is True and metal.posts() == ["/stop"]
    hdr = [h for meth, p, h, _ in metal.requests if meth == "POST"][0]
    assert hdr["X-Manifest-Metal-Control"] == "1" and "Origin" not in hdr
    metal.state, metal.connected = "observing", False
    ack = m.hold("stop:cli")
    assert ack.holding is False and ack.powered is False     # unpowered, brakeless: not held (support the arm)


def test_metal_adapter_identity_and_operator_verbs(metal):
    metal.serial = "WRONG"
    m = MetalOwnerConnector(metal.url, allow_post=True)
    m.poll_once()
    assert m.health().cause == "adapter_identity_mismatch"
    with pytest.raises(Refused):
        m.jaw_set(0.5, OperatorConfirm("op", "ok", 0.0))
    m.jaw_set(0.5, OperatorConfirm("op", "I am supporting the object", 0.0))
    with pytest.raises(Refused):
        m.recover(None)
    m.recover(OperatorConfirm("op", "recover", 0.0))
    assert metal.posts() == ["/jaw/set", "/recover_hold"]


def test_runtime_over_live_connectors_accepts_but_cannot_stream(yam, metal, pack):
    clock = Clock()
    conns = {"left": YamStationConnector(yam.url, clock=clock, sysroot=__import__("pathlib").Path("/nonexistent")),
             "right": MetalOwnerConnector(metal.url, clock=clock)}
    rt = Runtime(conns, pack, clock=clock, armed_local=True, armed_server=True, trust_unknown_plan_basis=True)
    rt.session_started("live-fake")
    t_end = time.monotonic() + 0.6
    while time.monotonic() < t_end:
        for c in conns.values():
            c.poll_once()
        rt.link_heartbeat()
        rt.tick()
        time.sleep(1 / 30)
    for c in conns.values():
        c.poll_once()                                 # submit judges the freshest cached samples
    q = list(conns["left"].sample().q)
    n = 31
    rows = [[*q, 0.999] for _ in range(n)]
    for i, r in enumerate(rows):
        r[0] += 0.02 * i / (n - 1)
    seg = dev.make_segment(rows, semantics=pack.arm("left").semantics, pack_digest=pack.digest, jaw=0.999,
                           limits={"ee_speed_m_s": 0.10})
    ack = rt.submit(seg)
    assert ack["result"] == "refused" and ack["reason"] == "connector_not_wired" and ack["acceptance"] == "passed"
    for name in NEVER:
        assert name not in yam.posts() + metal.posts()
    assert yam.posts() == [] and metal.posts() == []
