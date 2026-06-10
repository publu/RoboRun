"""Beacon tests — signed cross-robot awareness over a shared prefix."""
import json
import time
from pathlib import Path

import pytest

from roborun import beacons


@pytest.fixture
def beacon_dir(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_BEACON_DIR", str(tmp_path))
    return tmp_path


def test_emit_and_poll(beacon_dir):
    b = beacons.emit_beacon("go2-01", "person", x=1.0, y=2.0,
                            run_id="run_x", frame_topic="/camera/cam",
                            frame_log_time=123)
    assert b["published"]
    got = beacons.poll_beacons()
    assert len(got) == 1
    assert got[0]["label"] == "person"
    assert got[0]["run_ref"]["run_id"] == "run_x"


def test_signature_roundtrip(beacon_dir):
    b = beacons.emit_beacon("go2-01", "cone")
    if b["signature"] is None:
        pytest.skip("cryptography not installed")
    assert beacons.verify_beacon(b) is True


def test_forged_beacon_dropped(beacon_dir):
    b = beacons.emit_beacon("go2-01", "person", x=1.0, y=2.0)
    if b["signature"] is None:
        pytest.skip("cryptography not installed")
    path = Path(b["published"])
    data = json.loads(path.read_text())
    data["pose"]["x"] = 99.0  # move the claim
    path.write_text(json.dumps(data))
    assert beacons.poll_beacons() == []


def test_exclude_self_and_since(beacon_dir):
    beacons.emit_beacon("me", "thing")
    beacons.emit_beacon("them", "thing")
    got = beacons.poll_beacons(exclude_robot="me")
    assert [b["robot_id"] for b in got] == ["them"]
    assert beacons.poll_beacons(since=time.time() + 10) == []
