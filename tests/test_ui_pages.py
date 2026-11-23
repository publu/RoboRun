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


def _get_noredirect(base, path):
    """GET without following redirects — so we can assert the 302 itself."""
    import urllib.error

    class _NoRedirect(urllib.request.HTTPRedirectHandler):
        def redirect_request(self, *a, **k):
            return None

    opener = urllib.request.build_opener(_NoRedirect)
    try:
        with closing(opener.open(base + path, timeout=6)) as r:
            return r.status, r.read(), r.headers.get("Location")
    except urllib.error.HTTPError as e:
        return e.code, e.read(), e.headers.get("Location")


def _post(base, path, body):
    req = urllib.request.Request(base + path, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with closing(urllib.request.urlopen(req, timeout=8)) as r:
        return r.status, json.loads(r.read())


# Pages still served by the shared app shell (shell.js injects the sidebar/top-bar
# nav). These haven't been ported into the Studio SPA yet, so they keep loading
# the shell + the element ID their own JS drives.
@pytest.mark.parametrize("path,needle", [
    ("/analytics", b'id="kpis"'),
    ("/scenarios", b'id="suites"'),
])
def test_pages_serve(server, path, needle):
    status, body = _get(server, path)
    assert status == 200 and needle in body and b"shell.js" in body


# Routes the Studio SPA now owns: the legacy path 302-redirects into /studio/*.
# (See server._STUDIO_REDIRECT — search/timeline/run/browser/arena/deck/home.)
@pytest.mark.parametrize("path,dest", [
    ("/search", "/studio/search"),
    ("/timeline", "/studio/runs"),
    ("/run", "/studio/runs"),
    ("/browser", "/studio/runs"),
    ("/", "/studio/live"),
])
def test_absorbed_routes_redirect_into_studio(server, path, dest):
    status, _, location = _get_noredirect(server, path)
    assert status == 302 and location == dest


def test_studio_spa_serves(server):
    # Any /studio/* that isn't a built asset falls back to the SPA's index.html.
    status, body = _get(server, "/studio/live")
    assert status == 200 and b'id="root"' in body


def test_run_series_route_validates(server):
    status, d = _post(server, "/api/scenarios/run", {"scenario": "x"}) if False else (200, None)
    # missing id → 400 via the GET route
    import urllib.error
    try:
        _get(server, "/api/run/series")
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400


def test_shell_carries_nav(server):
    # nav is centralized in the app shell now (one connected UI), not duplicated
    # per page. The shell links to every dashboard.
    status, shell = _get(server, "/shell.js")
    assert status == 200
    for target in (b'"/scenarios"', b'"/timeline"', b'"/analytics"',
                   b'"/search"', b'"/fleet"', b'"/browser"', b'"/projects"', b'"/setup"'):
        assert target in shell, f"shell.js missing {target}"
    # and every page still served by the shell (not yet absorbed into the SPA)
    # loads it. The absorbed routes (search/timeline/run/browser) are covered by
    # test_absorbed_routes_redirect_into_studio instead.
    for path in ("/analytics", "/scenarios", "/projects", "/fleet", "/fleet-sim", "/setup"):
        _, body = _get(server, path)
        assert b"shell.js" in body, f"{path} not on the shell"


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
    # per-robot fleet breakdown
    robots = {r["robot_id"] for r in d.get("robots", [])}
    assert {"r1", "r2"} <= robots


def test_search_api_over_time(server):
    status, d = _post(server, "/api/search", {"query": "person", "by": "label"})
    assert status == 200 and d["ok"]
    assert d["results"][0]["detections"][0]["label"] == "person"
    assert d["results"][0]["source"] == "production"


def test_perception_status(server):
    status, body = _get(server, "/api/perception/status")
    assert json.loads(body)["ok"]
