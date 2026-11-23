"""Incidents — flag a moment in a run to revisit (data-flywheel primitive)."""
from __future__ import annotations

import pytest


@pytest.fixture(autouse=True)
def _isolated(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))


def test_flag_and_list():
    from roborun.incidents import flag, list_incidents
    r = flag("run_123", ts=42.0, note="robot clipped a pallet", tag="near-miss",
             robot_id="go2")
    assert r["run_id"] == "run_123" and r["note"]
    rows = list_incidents()
    assert len(rows) == 1 and rows[0]["tag"] == "near-miss"
    assert list_incidents(run_id="run_123")[0]["ts"] == 42.0
    assert list_incidents(run_id="other") == []
    assert list_incidents(tag="near-miss") and list_incidents(tag="nope") == []


def test_flag_emits_timeline_event():
    from roborun.incidents import flag
    from roborun.events import recent
    flag("run_x", note="check this")
    titles = [e.get("title", "") for e in recent(20)]
    assert any("flagged" in t for t in titles)
