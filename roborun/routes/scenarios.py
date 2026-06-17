"""Scenario registry routes — the /scenarios board (Antioch-style suites).

Read-only over the scenario JSON registry + a run trigger. Pure files, no infra.
"""
from __future__ import annotations

from roborun.routes import get, post, send_json, ApiError

# Registering built-in runnable scenarios so the board is live out of the box.
import roborun.demo_scenarios  # noqa: F401


@get("/api/scenarios")
def list_scenarios(h):
    """Scored runs, newest-first, with optional ?suite=&outcome=&name= filters."""
    from urllib.parse import parse_qs, urlparse
    from roborun.scenario import list_results
    q = parse_qs(urlparse(h.path).query)
    one = lambda k: (q.get(k) or [None])[0]
    rows = list_results(limit=int((q.get("limit") or [100])[0]),
                        suite=one("suite"), outcome=one("outcome"), name=one("name"),
                        tag=one("tag"))
    send_json(h, 200, {"ok": True, "results": rows, "total": len(rows)})


@get("/api/scenarios/suites")
def list_suites(h):
    """Suite cards: pass-rate, run count, latest — the Antioch Suites view."""
    from roborun.scenario import list_suites as _suites
    send_json(h, 200, {"ok": True, "suites": _suites()})


@get("/api/scenarios/defs")
def list_defs(h):
    """The catalog of *runnable* scenarios (not past results)."""
    from roborun.scenario_defs import list_defs as _defs, suites_defined
    send_json(h, 200, {"ok": True, "defs": _defs(), "suites": suites_defined()})


@post("/api/scenarios/run")
def run(h, payload):
    """Run a registered scenario or a whole suite. Body: {scenario} or {suite}."""
    from roborun.scenario_defs import run_scenario, run_suite
    suite = str(payload.get("suite", "")).strip()
    name = str(payload.get("scenario", "")).strip()
    seed = payload.get("seed")
    try:
        if suite:
            send_json(h, 200, {"ok": True, "summary": run_suite(suite)})
        elif name:
            send_json(h, 200, {"ok": True, "result": run_scenario(
                name, seed=int(seed) if seed is not None else None)})
        else:
            raise ApiError(400, "provide 'scenario' or 'suite'")
    except KeyError as exc:
        raise ApiError(404, str(exc))
