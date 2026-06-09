"""Webcam pipeline routes."""
from __future__ import annotations

from roborun.routes import get, post, send_json
from roborun.routes._singletons import get_webcam
from roborun.routes.tasks import log_event


@get("/api/webcam/state")
def state(h):
    send_json(h, 200, get_webcam().get_state())


@get("/api/webcam/detections")
def detections(h):
    send_json(h, 200, {"ok": True, "detections": get_webcam().get_detections()})


@post("/api/webcam/start")
def start(h, payload):
    source = payload.get("source")  # video file path or stream URL
    cam = str(source) if source else int(payload.get("camera", 0))
    models = payload.get("models", ["yolo"])
    result = get_webcam().start(camera_index=cam, models=models)
    if result.get("ok"):
        label = f"Video source: {source}" if source else f"Webcam started (cam {cam})"
        log_event("webcam_started", label)
    send_json(h, 200, result)


@post("/api/webcam/stop")
def stop(h, payload):
    result = get_webcam().stop()
    log_event("webcam_stopped", "Webcam stopped")
    send_json(h, 200, result)


@post("/api/webcam/models")
def models(h, payload):
    result = get_webcam().set_models(payload.get("models", []))
    send_json(h, 200, result)


@post("/api/webcam/clip_query")
def clip_query(h, payload):
    query = str(payload.get("query", "")).strip()
    result = get_webcam().set_clip_query(query)
    if query:
        from roborun.events import emit
        emit("task", "operator", f"track: {query}", {"query": query})
    send_json(h, 200, result)
