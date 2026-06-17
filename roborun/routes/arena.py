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
    send_json(h, 200, {"ok": True, "cmd": a.cmd(), "answer": a.answer(),
                       "intent": a.intent()})


_last_level: list[str] = [""]
_last_index: list[float] = [0.0]   # throttle live store-indexing


def _index_live(payload: dict) -> None:
    """Doing stuff in rapier should GENERATE real, searchable data — not seeds.
    Index the sim's detections into the spatial store (scoped to the active
    project/environment), throttled, so /search, /api/spatial and /analytics
    fill from real play. Best-effort; never blocks the sim."""
    import time
    dets = payload.get("detections") or []
    if not dets:
        return
    now = time.time()
    if now - _last_index[0] < 1.0:      # ~1 Hz is plenty for a searchable index
        return
    _last_index[0] = now
    try:
        from roborun.routes._singletons import get_memory
        pose = payload.get("pose") or {}
        robot = (payload.get("level") or {}).get("robot") or "sim"
        get_memory().store(detections=dets, ts=now,
                           x=pose.get("x", 0.0), y=-pose.get("z", 0.0),
                           robot_id=f"{robot}-sim", source="sim", source_id="arena")
    except Exception:
        pass


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
    _index_live(payload)   # real rapier play → real searchable data
    # the black box gets the full ROS-shaped view: /pose, /detections, /lidar
    try:
        from roborun import recorder as rec_mod
        rec = rec_mod.active_recorder()
        if rec is not None:
            pose = payload.get("pose") or {}
            fx, fy = pose.get("x", 0.0), -pose.get("z", 0.0)
            hd = pose.get("heading", 0.0)   # not `h` — that's the HTTP handler
            alt = (pose.get("y", 0.0)
                   if (payload.get("level") or {}).get("robot") == "drone" else 0.0)
            rec.write_pose(fx, fy, alt, heading=hd)
            dets = payload.get("detections") or []
            if dets:
                rec.write_detections(dets, name="arena")
                rec.write_detection_scene(dets, fx, fy, hd)
            lidar = payload.get("lidar") or []
            if lidar:
                rec.write_scan(lidar, fx, fy, hd)
    except Exception:
        pass
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
