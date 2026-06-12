"""OpenClaw bridge tests — routing and payload shape. No network."""
import pytest

from roborun import openclaw


@pytest.fixture
def gateway(monkeypatch):
    """Configured bridge with a captured-POST gateway."""
    sent = []
    monkeypatch.setenv("OPENCLAW_HOOKS_URL", "http://127.0.0.1:1/hooks")
    monkeypatch.setattr(openclaw, "_post",
                        lambda path, payload: sent.append((path, payload)) or True)
    return sent


def _ev(etype, source="sentry", title="hello", detail=None):
    return {"type": etype, "source": source, "title": title,
            "detail": detail or {}}


def test_disabled_without_url(monkeypatch):
    monkeypatch.delenv("OPENCLAW_HOOKS_URL", raising=False)
    assert not openclaw.configured()


def test_notify_goes_to_agent_hook_with_delivery(gateway, monkeypatch):
    monkeypatch.setenv("OPENCLAW_CHANNEL", "whatsapp")
    monkeypatch.setenv("OPENCLAW_TO", "+15555550123")
    assert openclaw.handle_event(_ev("notify", title="person spotted"))
    path, payload = gateway[0]
    assert path == "/agent"
    assert "person spotted" in payload["message"]
    assert payload["deliver"] is True
    assert payload["channel"] == "whatsapp"
    assert payload["to"] == "+15555550123"


def test_channel_omitted_when_unset(gateway, monkeypatch):
    monkeypatch.delenv("OPENCLAW_CHANNEL", raising=False)
    monkeypatch.delenv("OPENCLAW_TO", raising=False)
    openclaw.handle_event(_ev("notify"))
    _, payload = gateway[0]
    assert "channel" not in payload and "to" not in payload


def test_frames_stripped_from_detail(gateway):
    openclaw.handle_event(_ev("notify", detail={"frame": "x" * 9999, "count": 2}))
    _, payload = gateway[0]
    assert "xxxx" not in payload["message"]
    assert '"count": 2' in payload["message"]


def test_every_notify_sends(gateway):
    assert openclaw.handle_event(_ev("notify"))
    assert openclaw.handle_event(_ev("notify"))
    assert len(gateway) == 2


def test_other_event_types_stay_local(gateway):
    assert not openclaw.handle_event(_ev("detection"))
    assert not openclaw.handle_event(_ev("system"))
    assert not gateway


def test_own_events_never_loop_back(gateway):
    assert not openclaw.handle_event(_ev("notify", source="openclaw"))
    assert not gateway


def test_robot_notify_emits_notify_event(monkeypatch):
    from roborun.behaviors import Robot
    seen = []
    monkeypatch.setattr("roborun.behaviors.emit",
                        lambda *a, **k: seen.append(a) or {})
    Robot("sentry").notify("lap complete", laps=1)
    assert seen and seen[0][0] == "notify" and seen[0][2] == "lap complete"
