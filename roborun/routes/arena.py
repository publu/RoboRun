"""Arena routes — the browser sim's wire to behaviors and the timeline.

GET  /api/arena/cmd     latest robot.move() command (browser polls ~20 Hz)
POST /api/arena/state   {detections, pose, level} from the browser (~10 Hz)
POST /api/arena/event   chamber events (sightings, room entered, win) → timeline
"""
from __future__ import annotations

from roborun.arena import get_arena
from roborun.routes import get, post, send_json
from roborun import events as bus


@get("/api/arena/cmd")
def arena_cmd(h):
    a = get_arena()
    send_json(h, 200, {"ok": True, "cmd": a.cmd(), "answer": a.answer()})


_last_level: list[str] = [""]


@post("/api/arena/state")
def arena_state(h, payload):
    payload = payload or {}
    get_arena().update(payload)
    from roborun import sightings
    level = (payload.get("level") or {}).get("name", "")
    if level and level != _last_level[0]:
        _last_level[0] = level
        sightings.reset()
    sightings.observe(payload.get("detections") or [],
                      pose=payload.get("pose"), source="arena")
    send_json(h, 200, {"ok": True})


@post("/api/arena/event")
def arena_event(h, payload):
    title = str(payload.get("title", "")).strip()
    if title:
        bus.emit(str(payload.get("type", "arena")), "arena", title,
                 payload.get("detail") or {})
    send_json(h, 200, {"ok": True})


@get("/api/sightings")
def sightings_summary(h):
    """The run's automatic object memory — labels, counts, deduped world
    locations. The arena map plots these; any client can read them."""
    from roborun.sightings import summary
    send_json(h, 200, {"ok": True, "sightings": summary()})
