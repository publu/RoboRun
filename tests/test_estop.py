"""Emergency stop — the always-allowed safety primitive, REST + CLI, recorded."""
from __future__ import annotations

import json, threading, urllib.request
from contextlib import closing
import pytest


@pytest.fixture()
def server(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))
    from roborun import server as srv
    from http.server import ThreadingHTTPServer
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), srv.Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{httpd.server_address[1]}"
    httpd.shutdown()


def test_estop_route_halts_and_records(server, monkeypatch):
    monkeypatch.setenv("ROBORUN_PORT", server.rsplit(":", 1)[1])
    req = urllib.request.Request(server + "/api/estop", data=b"{}",
                                 headers={"Content-Type": "application/json"})
    with closing(urllib.request.urlopen(req, timeout=8)) as r:
        d = json.loads(r.read())
    assert d["ok"]
    # the stop is recorded as evidence
    from roborun.events import recent
    assert any("STOP" in e.get("title", "").upper() for e in recent(50))


def test_estop_disables_running_behaviors(server, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    runner_dir = tmp_path / "behaviors"; runner_dir.mkdir()
    (runner_dir / "spin.py").write_text(
        "from roborun.behaviors import behavior\n@behavior(hz=5)\ndef spin(robot):\n    robot.stop()\n")
    from roborun.behaviors import BehaviorRunner
    r = BehaviorRunner.get(); r.dirs = [runner_dir]; r._mtimes.clear()
    for loops in list(r._loops.values()):
        for lp in loops: lp.halt()
    r._loops.clear(); r.start()
    import time; time.sleep(1.2)
    assert any(s["enabled"] for s in r.statuses())   # spin is running
    req = urllib.request.Request(server + "/api/estop", data=b"{}",
                                 headers={"Content-Type": "application/json"})
    with closing(urllib.request.urlopen(req, timeout=8)): pass
    time.sleep(0.3)
    assert all(not s["enabled"] for s in r.statuses())  # all disabled after estop
