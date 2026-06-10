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
    send_json(h, 200, {"ok": True, "cmd": get_arena().cmd()})


@post("/api/arena/state")
def arena_state(h, payload):
    get_arena().update(payload or {})
    send_json(h, 200, {"ok": True})


@post("/api/arena/event")
def arena_event(h, payload):
    title = str(payload.get("title", "")).strip()
    if title:
        bus.emit(str(payload.get("type", "arena")), "arena", title,
                 payload.get("detail") or {})
    send_json(h, 200, {"ok": True})
