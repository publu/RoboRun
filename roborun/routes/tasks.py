"""Task CRUD and event routes."""
from __future__ import annotations

import json
import threading
import time
import uuid
from pathlib import Path

from roborun.routes import get, post, send_json, ApiError

ROOT = Path(__file__).resolve().parent.parent.parent
STATE_ROOT = ROOT / ".roborun"
TASKS_PATH = STATE_ROOT / "tasks.json"
EVENTS_PATH = STATE_ROOT / "events.json"
MAX_EVENTS = 200

_tasks_lock = threading.Lock()
_events_lock = threading.Lock()

VALID_SCHEDULES = {"5m": 300, "15m": 900, "30m": 1800, "1h": 3600, "6h": 21600, "12h": 43200, "24h": 86400}
VALID_ACTIONS = {"navigate_gps", "navigate_map", "explore", "query"}


# ── Events ───────────────────────────────────────────────────────────────────

def log_event(event_type: str, message: str, data: dict | None = None, level: str = "info") -> dict:
    from roborun.events import emit
    emit("system", event_type, message, data or {})
    event = {"id": uuid.uuid4().hex[:8], "type": event_type, "level": level,
             "message": message, "data": data or {},
             "ts": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    with _events_lock:
        STATE_ROOT.mkdir(parents=True, exist_ok=True)
        events = []
        if EVENTS_PATH.exists():
            try:
                events = json.loads(EVENTS_PATH.read_text())
            except Exception:
                events = []
        events.insert(0, event)
        events = events[:MAX_EVENTS]
        EVENTS_PATH.write_text(json.dumps(events, indent=2))
    return event


@get("/api/events(?:\\?.*)?")
def get_events(h):
    limit = 50
    unread_since = None
    if "?" in h.path:
        for part in h.path.split("?", 1)[1].split("&"):
            if part.startswith("limit="):
                limit = min(int(part[6:]), MAX_EVENTS)
            if part.startswith("since="):
                unread_since = part[6:]
    with _events_lock:
        if not EVENTS_PATH.exists():
            send_json(h, 200, {"ok": True, "events": [], "total": 0})
            return
        try:
            events = json.loads(EVENTS_PATH.read_text())
        except Exception:
            events = []
    events = events[:limit]
    unread = sum(1 for e in events if e.get("ts", "") > (unread_since or "")) if unread_since else 0
    send_json(h, 200, {"ok": True, "events": events, "total": len(events), "unread": unread})


# ── Tasks ────────────────────────────────────────────────────────────────────

def _load_tasks() -> list[dict]:
    if TASKS_PATH.exists():
        try:
            data = json.loads(TASKS_PATH.read_text())
            if isinstance(data, list):
                return data
        except Exception:
            pass
    return []


def _save_tasks(tasks: list[dict]) -> None:
    STATE_ROOT.mkdir(parents=True, exist_ok=True)
    TASKS_PATH.write_text(json.dumps(tasks, indent=2))


@get("/api/tasks(?:\\?.*)?")
def list_tasks(h):
    sf = None
    if "?" in h.path:
        for part in h.path.split("?", 1)[1].split("&"):
            if part.startswith("status="):
                sf = part[7:]
    with _tasks_lock:
        tasks = _load_tasks()
    if sf and sf != "all":
        tasks = [t for t in tasks if t.get("status") == sf]
    send_json(h, 200, {"ok": True, "tasks": tasks, "total": len(tasks)})


@post("/api/tasks/create")
def create_task(h, payload):
    name = str(payload.get("name", "")).strip()
    if not name:
        raise ApiError(400, "Task name required")
    action = str(payload.get("action", "explore")).strip()
    if action not in VALID_ACTIONS:
        raise ApiError(400, f"Action must be one of: {', '.join(VALID_ACTIONS)}")
    task_type = payload.get("type", "one_off")
    schedule = str(payload.get("schedule", "1h")).strip()
    params: dict = {}
    if action == "navigate_gps":
        params = {"lat": float(payload.get("lat", 0)), "lon": float(payload.get("lon", 0))}
    elif action == "navigate_map":
        params = {"x": float(payload.get("x", 0)), "y": float(payload.get("y", 0))}
    elif action == "query":
        params = {"text": str(payload.get("text", "")).strip()}
    task = {
        "id": str(uuid.uuid4()), "name": name,
        "description": str(payload.get("description", "")).strip(),
        "type": task_type, "action": action, "params": params,
        "schedule": schedule if task_type == "recurring" else None,
        "enabled": True, "status": "scheduled", "run_count": 0,
        "last_run": None, "last_result": None,
        "source": str(payload.get("source", "dashboard")),
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    with _tasks_lock:
        tasks = _load_tasks()
        tasks.append(task)
        _save_tasks(tasks)
    log_event("task_created", f"Task: {name} ({action})", {"taskId": task["id"]})
    send_json(h, 200, {"ok": True, "task": task})


@post("/api/tasks/update")
def update_task(h, payload):
    task_id = str(payload.get("id", "")).strip()
    if not task_id:
        raise ApiError(400, "Task id required")
    with _tasks_lock:
        tasks = _load_tasks()
        for t in tasks:
            if str(t.get("id", "")).startswith(task_id):
                for f in ("name", "description"):
                    if f in payload:
                        t[f] = str(payload[f]).strip()
                if "status" in payload:
                    t["status"] = payload["status"]
                if "enabled" in payload:
                    t["enabled"] = bool(payload["enabled"])
                    t["status"] = "scheduled" if t["enabled"] else "disabled"
                if "schedule" in payload and payload["schedule"] in VALID_SCHEDULES:
                    t["schedule"] = payload["schedule"]
                _save_tasks(tasks)
                send_json(h, 200, {"ok": True, "task": t})
                return
    raise ApiError(404, "Task not found")


@post("/api/tasks/delete")
def delete_task(h, payload):
    task_id = str(payload.get("id", "")).strip()
    with _tasks_lock:
        tasks = _load_tasks()
        original = len(tasks)
        tasks = [t for t in tasks if not str(t.get("id", "")).startswith(task_id)]
        if len(tasks) == original:
            raise ApiError(404, "Task not found")
        _save_tasks(tasks)
    send_json(h, 200, {"ok": True, "deleted": task_id})


@post("/api/tasks/run")
def run_task(h, payload):
    task_id = str(payload.get("id", "")).strip()
    with _tasks_lock:
        tasks = _load_tasks()
        task = next((t for t in tasks if str(t.get("id", "")).startswith(task_id)), None)
        if not task:
            raise ApiError(404, "Task not found")
    log_event("task_dispatched", f"Task run: {task.get('name', '?')}", {"taskId": task_id})
    send_json(h, 200, {"ok": True, "taskId": task_id})
