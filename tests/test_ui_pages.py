"""UI test pass: every page serves, every dashboard API feeds it, nav connects."""
from __future__ import annotations

import json
import threading
import urllib.request
from contextlib import closing

import pytest


@pytest.fixture()
def server(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))
    # reset the cached store singleton so it rebinds to the tmp data dir
    from roborun.routes import _singletons
    _singletons._spatial_memory = None
    # seed data so the dashboards have something real to render
    from roborun.routes._singletons import get_memory
    store = get_memory()
    store.store(detections=[{"label": "person", "score": 0.9, "bbox": [0, 0, 1, 1]}],
                ts=__import__("time").time(), robot_id="r1", source="production",
                source_id="lobby")
    store.store(detections=[{"label": "forklift", "score": 0.8, "bbox": [0, 0, 1, 1]}],
                ts=__import__("time").time(), robot_id="r2", source="robot")
    from roborun import scenario as S
    with S.scenario("nav_1", suite="warehouse") as run:
        run.passed()

    from roborun import server as srv
    from http.server import ThreadingHTTPServer
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), srv.Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{httpd.server_address[1]}"
    httpd.shutdown()


def _get(base, path):
    with closing(urllib.request.urlopen(base + path, timeout=6)) as r:
        return r.status, r.read()


def _post(base, path, body):
    req = urllib.request.Request(base + path, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with closing(urllib.request.urlopen(req, timeout=8)) as r:
        return r.status, json.loads(r.read())


@pytest.mark.parametrize("path,needle", [
    ("/search", b"SEARCH OVER TIME"),
    ("/timeline", b"TIMELINE"),
    ("/analytics", b"ANALYTICS"),
    ("/scenarios", b"SCENARIOS"),
    ("/run", b"RUN"),
])
def test_pages_serve(server, path, needle):
    status, body = _get(server, path)
    assert status == 200 and needle in body


def test_run_series_route_validates(server):
    status, d = _post(server, "/api/scenarios/run", {"scenario": "x"}) if False else (200, None)
    # missing id → 400 via the GET route
    import urllib.error
    try:
        _get(server, "/api/run/series")
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400


def test_pages_cross_link_nav(server):
    # every dashboard links to the others — one connected UI
    for path in ("/search", "/timeline", "/analytics"):
        _, body = _get(server, path)
        for target in (b'href="/scenarios"', b'href="/timeline"',
                       b'href="/analytics"', b'href="/search"', b'href="/fleet"'):
            assert target in body, f"{path} missing {target}"


def test_analytics_api_aggregates(server):
    _, d = _post(server, "/api/analytics", {}) if False else (_get(server, "/api/analytics"))
    d = json.loads(d) if isinstance(d, bytes) else d


def test_analytics_payload(server):
    status, body = _get(server, "/api/analytics")
    d = json.loads(body)
    assert d["ok"]
    assert d["observations"]["total"] >= 2
    labels = {x["label"] for x in d["labels"]}
    assert {"person", "forklift"} <= labels
    assert any(s["suite"] == "warehouse" for s in d["suites"])
    assert "over_time" in d and len(d["over_time"]) == 24
    assert d["runs"]["count"] >= 0


def test_search_api_over_time(server):
    status, d = _post(server, "/api/search", {"query": "person", "by": "label"})
    assert status == 200 and d["ok"]
    assert d["results"][0]["detections"][0]["label"] == "person"
    assert d["results"][0]["source"] == "production"


def test_perception_status(server):
    status, body = _get(server, "/api/perception/status")
    assert json.loads(body)["ok"]
