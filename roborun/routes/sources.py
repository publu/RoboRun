"""Source routes — what can see/move right now, and what's on the network."""
from __future__ import annotations

from roborun.routes import get, post, send_json


@get("/api/sources")
def sources(h):
    from roborun.sources import inventory
    send_json(h, 200, inventory())


@post("/api/sources/scan")
def sources_scan(h, payload):
    from roborun.sources import network_scan
    send_json(h, 200, {"ok": True, **network_scan(force=True)})
