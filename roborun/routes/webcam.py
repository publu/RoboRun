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
    cam = int(payload.get("camera", 0))
    models = payload.get("models", ["yolo"])
    result = get_webcam().start(camera_index=cam, models=models)
    if result.get("ok"):
        log_event("webcam_started", f"Webcam started (cam {cam})")
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
    result = get_webcam().set_clip_query(str(payload.get("query", "")))
    send_json(h, 200, result)
