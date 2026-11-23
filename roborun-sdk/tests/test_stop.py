"""K1 Stop: latch first, every owner's hold in parallel with a 300 ms budget, cancel, report from measured state.
The sim half of the Stop drill matrix: sources x states."""
import os
import socket
import threading
import time
import urllib.request

import pytest

from roborun_sdk.connectors.hid import HidPedal, event_bytes
from roborun_sdk.rt.datatypes import OperatorConfirm, Refused
from roborun_sdk.rt.stop_listeners import StopListeners, send_udp_stop

from conftest import Rig


def test_stop_idle_reports_both_arms_holding(rig):
    report = rig.rt.stop("cli")
    assert rig.rt.stop_latch.latched
    assert report["not_confirmed"] == []
    for arm in ("left", "right"):
        assert report["arms"][arm]["holding"] is True and "q" in report["arms"][arm]
    assert any(e.get("kind") == "stopped" for e in rig.events())


def test_stop_during_segment_ends_it_and_no_write_follows(rig):
    rig.rt.submit(rig.segment(seq=1, sid="s", seconds=2.0))
    rig.drive(0.3)
    n = len(rig.left.writes)
    report = rig.rt.stop("pedal")
    assert report["cancelled"]["segment"] == "s"
    rig.drive(0.5)
    assert len(rig.left.writes) == n
    assert rig.rt.playback.outcome == "stopped"
    assert rig.rt.submit(rig.segment(seq=2, sid="after"))["reason"] == "stopped"


def test_hold_that_hangs_is_not_confirmed_within_budget(rig):
    rig.right.inject(hold_hang_s=0.8)
    t0 = time.perf_counter()
    report = rig.rt.stop("lan")
    elapsed = time.perf_counter() - t0
    assert elapsed < 0.6
    assert report["arms"]["left"]["holding"] is True
    assert report["arms"]["right"]["holding"] is None
    assert report["arms"]["right"]["console"] == "Stop not confirmed on right: support the arm or cut power"
    assert report["not_confirmed"] == ["right"]
    # a repeat while the first hold is still in flight shares it (no pile-up) and is harmless
    report2 = rig.rt.stop("lan")
    assert report2["not_confirmed"] == ["right"]


def test_refused_hold_is_never_reported_holding(rig):
    rig.left.inject(hold_refuse=True)
    report = rig.rt.stop("ui")
    assert report["arms"]["left"]["holding"] is False and not report["arms"]["left"]["confirmed"]


def test_unverified_relayed_stop_is_obeyed(rig):
    from roborun_sdk.rt.server import stop_message
    report = stop_message(rig.rt, {"type": "stop", "id": "stp_1", "source": "hosted_ui", "user_role": "operator"})
    assert rig.rt.stop_latch.latched and report["verified"] is False and report["ref"] == "stp_1"


def test_clearing_needs_an_operator_and_starts_nothing(rig):
    rig.rt.submit(rig.segment(seq=1, sid="a", seconds=2.0))
    rig.drive(0.2)
    rig.rt.stop("cli")
    with pytest.raises(Refused):
        rig.rt.stop_latch.clear(None)
    rig.rt.clear_stop(OperatorConfirm("operator", "clear stop", rig.clock.now(), "cli"))
    n = len(rig.left.writes)
    rig.drive(0.5)
    assert len(rig.left.writes) == n                                  # nothing resumed
    assert rig.rt.submit(rig.segment(seq=2, sid="old"))["reason"] == "plan_basis_stale"


def test_write_then_recheck_lets_a_racing_stop_win(rig):
    rig.rt.submit(rig.segment(seq=1, sid="race", seconds=2.0))
    rig.drive(0.1)
    real_write = rig.left.write

    def racing_write(target, t):
        real_write(target, t)
        rig.rt.stop_latch.latch()                                     # a Stop lands during the write
    rig.left.write = racing_write
    rig.drive(1 / 30)
    assert "stop_after_write" in rig.left.holds


def test_listener_refuses_web_pages_and_takes_local_stops(rig):
    listeners = StopListeners(rig.rt, udp_port=None, http_port=0).start()
    try:
        url = f"http://127.0.0.1:{listeners.http_port}/stop"
        for headers in ({"Origin": "http://evil.test"}, {"Origin": "http://127.0.0.1:9011"}, {"Sec-Fetch-Site": "cross-site"},
                        {"Host": f"rebind.test:{listeners.http_port}"}):
            with pytest.raises(urllib.error.HTTPError) as err:
                urllib.request.urlopen(urllib.request.Request(url, data=b"{}", method="POST", headers=headers), timeout=2)
            assert err.value.code == 403
        assert listeners.received == []
        with urllib.request.urlopen(urllib.request.Request(url, data=b"{}", method="POST"), timeout=2) as resp:
            assert resp.status == 200
        assert listeners.received == ["lan_http"]
    finally:
        listeners.close()


def test_listeners_udp_http_and_sigusr1(rig):
    listeners = StopListeners(rig.rt, udp_port=0, http_port=0, sigusr1=True).start()
    try:
        send_udp_stop("127.0.0.1", listeners.udp_port, "stp_udp", repeats=1)
        deadline = time.time() + 2
        while "lan_udp" not in listeners.received and time.time() < deadline:
            time.sleep(0.01)
        assert "lan_udp" in listeners.received
        req = urllib.request.Request(f"http://127.0.0.1:{listeners.http_port}/stop", data=b"{}", method="POST")
        with urllib.request.urlopen(req, timeout=2) as resp:
            assert resp.status == 200
        os.kill(os.getpid(), __import__("signal").SIGUSR1)
        deadline = time.time() + 2
        while "sigusr1" not in listeners.received and time.time() < deadline:
            time.sleep(0.01)
        assert {"lan_udp", "lan_http", "sigusr1"} <= set(listeners.received)
    finally:
        listeners.close()
        import signal
        signal.signal(signal.SIGUSR1, signal.SIG_DFL)


def test_hid_pedal_press_calls_stop(rig):
    r, w = os.pipe()
    pedal = HidPedal.from_fd(r)
    pressed = []
    pedal.on_press(lambda src: pressed.append(rig.rt.stop(src)))
    pedal.start()
    os.write(w, event_bytes(value=0) + event_bytes(value=1))         # a release, then a press
    deadline = time.time() + 2
    while not pressed and time.time() < deadline:
        time.sleep(0.01)
    os.close(w)
    pedal.close()
    os.close(r)
    assert pressed and pressed[0]["source"] == "pedal" and rig.rt.stop_latch.latched
