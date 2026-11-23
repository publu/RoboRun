"""The /scenarios board: API routes + page serve + run trigger."""
from __future__ import annotations

import json
import threading
import urllib.request
from contextlib import closing

import pytest


@pytest.fixture()
def server(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))
    monkeypatch.setenv("ROBORUN_PORT", "0")
    # seed a scored run + a runnable scenario before the server imports settle
    from roborun import scenario as S, scenario_defs as D
    monkeypatch.setattr(D, "_REGISTRY", {})
    with S.scenario("lobby_0", suite="lobby", tags=["nav"]) as r:
        r.metric("path_length_m", 18.4); r.evaluate("safety", near_misses=0); r.passed()

    @D.scenario_def("quick", suite="lobby")
    def quick(ctx):
        ctx.run.passed()

    from roborun import server as srv
    from http.server import ThreadingHTTPServer
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), srv.Handler)
    t = threading.Thread(target=httpd.serve_forever, daemon=True); t.start()
    base = f"http://127.0.0.1:{httpd.server_address[1]}"
    yield base
    httpd.shutdown()


def _get(base, path):
    with closing(urllib.request.urlopen(base + path, timeout=5)) as r:
        return r.status, r.read()


def _post(base, path, body):
    req = urllib.request.Request(base + path, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with closing(urllib.request.urlopen(req, timeout=10)) as r:
        return r.status, json.loads(r.read())


def test_page_serves(server):
    status, body = _get(server, "/scenarios")
    assert status == 200 and b"SCENARIOS" in body


def test_suites_and_results_api(server):
    _, body = _get(server, "/api/scenarios/suites")
    suites = json.loads(body)["suites"]
    assert any(s["suite"] == "lobby" for s in suites)

    _, body = _get(server, "/api/scenarios?suite=lobby&outcome=passed")
    d = json.loads(body)
    assert d["ok"] and d["results"][0]["name"] == "lobby_0"


def test_defs_and_run(server):
    _, body = _get(server, "/api/scenarios/defs")
    assert "quick" in [d["name"] for d in json.loads(body)["defs"]]

    status, d = _post(server, "/api/scenarios/run", {"scenario": "quick", "seed": 7})
    assert status == 200 and d["result"]["outcome"] == "passed" and d["result"]["seed"] == 7
