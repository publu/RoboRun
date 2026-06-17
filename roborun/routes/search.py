"""Search-over-time + perception session routes — the production surface.

`/api/search` finds anything/anyone across all recorded history (semantic + label +
time window), regardless of which mode produced it. `/api/perception/*` runs the
unified capture loop in a chosen mode.
"""
from __future__ import annotations

from roborun.routes import get, post, send_json, ApiError

_session = None


@post("/api/search")
def search_history(h, payload):
    """Body: {query, by?: clip|label|near, k?, since?, until?, source_id?}.
    Searches all runs/modes; since/until are unix seconds."""
    from roborun.routes._singletons import get_memory  # SpatialMemoryStore singleton
    from roborun.session import search
    query = payload.get("query")
    if query is None and payload.get("by") not in ("time",):
        raise ApiError(400, "query required")
    try:
        store = get_memory()
    except Exception:
        from roborun.spatial_memory import SpatialMemoryStore
        store = SpatialMemoryStore()
    rows = search(store, query, by=str(payload.get("by", "clip")),
                  k=int(payload.get("k", 10)),
                  since=payload.get("since"), until=payload.get("until"),
                  source_id=payload.get("source_id"))
    send_json(h, 200, {"ok": True, "results": rows, "total": len(rows)})


@post("/api/perception/start")
def perception_start(h, payload):
    """Start the unified capture loop in a mode (sim|robot|production)."""
    global _session
    from roborun.session import PerceptionSession, MODES
    mode = str(payload.get("mode", "production"))
    if mode not in MODES:
        raise ApiError(400, f"mode must be one of {MODES}")
    if _session is not None:
        _session.stop()
    try:
        store = None
        from roborun.routes._singletons import get_memory
        store = get_memory()
    except Exception:
        store = None
    _session = PerceptionSession.for_mode(
        mode, store=store, source_id=str(payload.get("source_id", "cam")))
    _session.start()
    send_json(h, 200, {"ok": True, "mode": mode, "source_id": _session.source_id})


@post("/api/perception/stop")
def perception_stop(h, payload):
    global _session
    if _session is not None:
        _session.stop()
        _session = None
    send_json(h, 200, {"ok": True})


@get("/api/perception/status")
def perception_status(h):
    send_json(h, 200, {"ok": True, "running": _session is not None,
                       "mode": getattr(_session, "mode", None),
                       "indexed": getattr(_session, "indexed", 0)})
