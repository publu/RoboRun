"""Dataset recording routes."""
from __future__ import annotations

from roborun.routes import get, post, send_json
from roborun.routes._singletons import get_dataset
from roborun.routes.tasks import log_event


@get("/api/dataset/status")
def status(h):
    send_json(h, 200, {"ok": True, **get_dataset().get_status()})


@get("/api/dataset/list")
def list_datasets(h):
    send_json(h, 200, get_dataset().list_datasets())


@post("/api/dataset/start")
def start(h, payload):
    result = get_dataset().start_recording(str(payload.get("name", "default")).strip())
    if result.get("ok"):
        log_event("recording_started", f"Recording: {payload.get('name')}")
    send_json(h, 200, result)


@post("/api/dataset/stop")
def stop(h, payload):
    result = get_dataset().stop_recording()
    if result.get("ok"):
        log_event("recording_stopped", f"Saved {result.get('frames', 0)} frames")
    send_json(h, 200, result)
