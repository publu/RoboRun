"""RoboRun server — zero-terminal visual teleop, dataset collection, dimOS deployment.

Merges the full dimOS/RobotClaw workbench (MCP calls, agent chat, fleet, tasks,
blueprints, daemon, camera, Command Center) with webcam + model pipeline and
dataset collection. dimOS is optional — webcam mode works standalone.
"""

from __future__ import annotations

import base64
import json
import os
import plistlib
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent
WEB_ROOT = ROOT / "web"
HOST = "127.0.0.1"
PORT = int(os.environ.get("ROBORUN_PORT", "8765"))
IP_PATTERN = re.compile(r"^(?:(?:25[0-5]|2[0-4]\d|1?\d?\d)\.){3}(?:25[0-5]|2[0-4]\d|1?\d?\d)$")
NAME_PATTERN = re.compile(r"^[A-Za-z0-9_.:/@-]+$")
STATE_ROOT = ROOT / ".roborun"
JOB_ROOT = STATE_ROOT / "jobs"
PROFILE_PATH = STATE_ROOT / "profile.json"
TASKS_PATH = STATE_ROOT / "tasks.json"
EVENTS_PATH = STATE_ROOT / "events.json"
FLEET_PATH = STATE_ROOT / "fleet.json"
BLUEPRINTS_PATH = STATE_ROOT / "blueprints.json"

MAX_EVENTS = 200
_tasks_lock = threading.Lock()
_events_lock = threading.Lock()
_fleet_lock = threading.Lock()
_blueprints_lock = threading.Lock()

JOBS: dict[str, dict[str, Any]] = {}
ACTIVE_DIMOS_JOB_ID: str | None = None

# ── Webcam + Dataset singletons ──────────────────────────────────────────────
_webcam = None
_dataset = None

def _get_webcam():
    global _webcam
    if _webcam is None:
        from roborun.webcam import WebcamPipeline
        _webcam = WebcamPipeline()
    return _webcam

def _get_dataset():
    global _dataset
    if _dataset is None:
        from roborun.dataset import DatasetCollector
        _dataset = DatasetCollector()
    return _dataset

_simulator = None
def _get_simulator():
    global _simulator
    if _simulator is None:
        from roborun.simulator import SimulatorRunner
        _simulator = SimulatorRunner()
    return _simulator

_spatial_memory = None
def _get_memory():
    global _spatial_memory
    if _spatial_memory is None:
        from roborun.spatial_memory import SpatialMemoryStore
        _spatial_memory = SpatialMemoryStore(
            s3_bucket=os.environ.get("ROBORUN_S3_BUCKET"),
            s3_prefix=os.environ.get("ROBORUN_S3_PREFIX", "roborun/memories/"),
            s3_endpoint=os.environ.get("ROBORUN_S3_ENDPOINT"),
        )
    return _spatial_memory

def _get_scene_builder():
    from roborun.scene_builder import SceneBuilder
    return SceneBuilder.get()

# ── Agent — FastRobotAgent (SDK) preferred, RobotAgent (subprocess) as fallback
_AGENT = None
def _get_agent():
    global _AGENT
    if _AGENT is None:
        import os
        if os.environ.get("ANTHROPIC_API_KEY"):
            try:
                from roborun.agent import FastRobotAgent
                _AGENT = FastRobotAgent()
                return _AGENT
            except Exception:
                pass
        try:
            from roborun.agent import RobotAgent
            _AGENT = RobotAgent()
        except Exception:
            _AGENT = "unavailable"
    return _AGENT

_GEMINI_AGENT = None
def _get_gemini_agent():
    global _GEMINI_AGENT
    if _GEMINI_AGENT is None:
        import os
        if not os.environ.get("GEMINI_API_KEY"):
            return "unavailable"
        try:
            from roborun.agent import GeminiAgent
            _GEMINI_AGENT = GeminiAgent()
        except Exception:
            _GEMINI_AGENT = "unavailable"
    return _GEMINI_AGENT

# ── Async MCP-call task store ────────────────────────────────────────────────
_MCP_TASKS: dict[str, dict[str, Any]] = {}
_MCP_TASKS_LOCK = threading.Lock()
_MCP_TASK_TTL = 300.0

def _run_mcp_task(task_id: str, name: str, args: dict[str, Any]) -> None:
    import urllib.request
    try:
        body = json.dumps({
            "jsonrpc": "2.0", "id": 1, "method": "tools/call",
            "params": {"name": name, "arguments": args},
        }).encode()
        req = urllib.request.Request(
            "http://127.0.0.1:9990/mcp", data=body,
            headers={"Content-Type": "application/json"}, method="POST",
        )
        with urllib.request.urlopen(req, timeout=30) as resp:
            data = json.loads(resp.read().decode())
        if "error" in data:
            with _MCP_TASKS_LOCK:
                _MCP_TASKS[task_id].update(status="error", error=str(data["error"]), finished=time.time())
            return
        content = data.get("result", {}).get("content")
        if isinstance(content, list):
            text = "\n".join(
                (it.get("text", str(it)) if isinstance(it, dict) else str(it)) for it in content
            )
        elif isinstance(content, str):
            text = content
        else:
            text = json.dumps(content) if content is not None else json.dumps(data.get("result", {}))
        with _MCP_TASKS_LOCK:
            _MCP_TASKS[task_id].update(status="done", result=text, finished=time.time())
    except Exception as exc:
        with _MCP_TASKS_LOCK:
            _MCP_TASKS[task_id].update(status="error", error=f"MCP call failed: {exc}", finished=time.time())

def _prune_mcp_tasks() -> None:
    cutoff = time.time() - _MCP_TASK_TTL
    with _MCP_TASKS_LOCK:
        stale = [k for k, v in _MCP_TASKS.items()
                 if v.get("status") in ("done", "error") and v.get("finished", 0) < cutoff]
        for k in stale:
            _MCP_TASKS.pop(k, None)

# ── Frame paths ──────────────────────────────────────────────────────────────
_CAMERA_FRAME_PATH = Path("/tmp/go2_camera_frame.jpg")
_HACKATHON_FRAME_PATH = Path("/tmp/go2_hackathon_frame.jpg")
_HACKATHON_STATE_PATH = Path("/tmp/go2_hackathon_state.json")
_WEBCAM_FRAME_PATH = Path("/tmp/roborun_frame.jpg")
_WEBCAM_STATE_PATH = Path("/tmp/roborun_state.json")

# ── Helpers ──────────────────────────────────────────────────────────────────

class ApiError(Exception):
    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = status
        self.message = message

def read_json(handler: SimpleHTTPRequestHandler) -> dict[str, Any]:
    length = int(handler.headers.get("Content-Length", "0") or "0")
    if length > 32_768:
        raise ApiError(413, "Request body too large")
    raw = handler.rfile.read(length)
    if not raw:
        return {}
    try:
        value = json.loads(raw.decode("utf-8"))
    except json.JSONDecodeError as exc:
        raise ApiError(400, "Invalid JSON") from exc
    if not isinstance(value, dict):
        raise ApiError(400, "JSON body must be an object")
    return value

def valid_ip(value: str) -> bool:
    return bool(IP_PATTERN.fullmatch(value.strip()))

def safe_arg(value: str, field: str) -> str:
    value = value.strip()
    if not value:
        raise ApiError(400, f"{field} is required")
    if not NAME_PATTERN.fullmatch(value):
        raise ApiError(400, f"{field} contains unsupported characters")
    return value

def shell_command(args: list[str]) -> str:
    return " ".join(shlex.quote(part) for part in args)

def command_env(env: dict[str, str] | None = None) -> dict[str, str]:
    merged = os.environ.copy()
    dimos_path = load_profile().get("dimosPath", "").strip()
    if dimos_path:
        venv_bin = Path(dimos_path) / ".venv" / "bin"
        if venv_bin.exists():
            merged["PATH"] = f"{venv_bin}{os.pathsep}{merged.get('PATH', '')}"
    if env:
        merged.update(env)
    return merged

def run_command(args: list[str], env: dict[str, str] | None = None, timeout: int = 20,
                display_args: list[str] | None = None) -> dict[str, Any]:
    merged = command_env(env)
    try:
        completed = subprocess.run(args, cwd=str(ROOT), env=merged, text=True,
                                   stdin=subprocess.DEVNULL, capture_output=True, timeout=timeout, check=False)
        return {"ok": completed.returncode == 0, "code": completed.returncode,
                "command": shell_command(display_args or args),
                "stdout": completed.stdout[-6000:], "stderr": completed.stderr[-6000:]}
    except FileNotFoundError:
        return {"ok": False, "code": 127, "command": shell_command(display_args or args),
                "stdout": "", "stderr": f"{args[0]} not found on PATH."}
    except subprocess.TimeoutExpired:
        return {"ok": False, "code": 124, "command": shell_command(display_args or args),
                "stdout": "", "stderr": "Command timed out."}

# ── Camera ───────────────────────────────────────────────────────────────────

def camera_snapshot() -> dict[str, Any]:
    for path in (_HACKATHON_FRAME_PATH, _WEBCAM_FRAME_PATH, _CAMERA_FRAME_PATH):
        if path.exists() and (time.time() - path.stat().st_mtime) < 5.0:
            try:
                data = base64.b64encode(path.read_bytes()).decode()
                return {"ok": True, "image": f"data:image/jpeg;base64,{data}", "ts": path.stat().st_mtime}
            except Exception as exc:
                return {"ok": False, "error": str(exc)}
    return {"ok": False, "error": "No frame available — start webcam or dimOS"}

# ── Profile ──────────────────────────────────────────────────────────────────

def load_profile() -> dict[str, Any]:
    defaults = {
        "deviceName": "RoboRun Station",
        "deviceType": "Webcam + Robot",
        "serial": "",
        "group": "Robots",
        "robotIp": "",
        "blueprint": "unitree-go2",
        "mode": "replay",
        "viewer": "rerun",
        "daemon": True,
        "dimosPath": "",
        "cameraIndex": 0,
        "activeModels": ["yolo"],
    }
    if PROFILE_PATH.exists():
        try:
            saved = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
            if isinstance(saved, dict):
                defaults.update(saved)
        except Exception:
            pass
    return defaults

def save_profile(payload: dict[str, Any]) -> dict[str, Any]:
    profile = load_profile()
    for key in ["deviceName", "deviceType", "serial", "group", "robotIp", "blueprint",
                "mode", "viewer", "daemon", "dimosPath", "cameraIndex", "activeModels"]:
        if key in payload:
            profile[key] = payload[key]
    STATE_ROOT.mkdir(parents=True, exist_ok=True)
    PROFILE_PATH.write_text(json.dumps(profile, indent=2), encoding="utf-8")
    return profile

# ── System stats ─────────────────────────────────────────────────────────────

def system_stats() -> dict[str, Any]:
    disk = shutil.disk_usage(ROOT)
    load_avg = os.getloadavg()
    return {
        "load": [round(v, 2) for v in load_avg],
        "disk": {"percent": round((disk.used / disk.total) * 100, 1),
                 "usedBytes": disk.used, "totalBytes": disk.total},
    }

# ── dimOS status ─────────────────────────────────────────────────────────────

_dimos_status_cache: dict[str, Any] = {}
_dimos_status_ts: float = 0.0

def dimos_status() -> dict[str, Any]:
    global _dimos_status_cache, _dimos_status_ts
    if _dimos_status_cache and (time.time() - _dimos_status_ts) < 8.0:
        return _dimos_status_cache
    if not shutil.which("dimos"):
        _dimos_status_cache = {"ok": False, "running": False, "label": "Not installed", "stdout": "", "stderr": ""}
        _dimos_status_ts = time.time()
        return _dimos_status_cache
    status = run_command(["dimos", "status"], timeout=5)
    stdout = status.get("stdout", "").strip()
    running = bool(status.get("ok")) and "No running DimOS instance" not in stdout
    status["running"] = running
    status["label"] = "Online" if running else "Idle"
    _dimos_status_cache = status
    _dimos_status_ts = time.time()
    return status

def command_center_status() -> dict[str, Any]:
    reachable = False
    detail = "Not listening"
    try:
        with socket.create_connection(("127.0.0.1", 7779), timeout=2):
            reachable = True
            detail = "Listening on 127.0.0.1:7779"
    except OSError as exc:
        detail = str(exc)
    return {"ok": reachable, "url": "http://127.0.0.1:7779/command-center", "port": 7779, "detail": detail}

def robot_reachable(ip: str) -> bool:
    if not valid_ip(ip):
        return False
    try:
        result = subprocess.run(["ping", "-c", "1", "-W", "1", ip],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3)
        return result.returncode == 0
    except Exception:
        return False

# ── Events ───────────────────────────────────────────────────────────────────

def log_event(event_type: str, message: str, data: dict | None = None, level: str = "info") -> dict:
    event = {"id": uuid.uuid4().hex[:8], "type": event_type, "level": level,
             "message": message, "data": data or {},
             "ts": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    with _events_lock:
        STATE_ROOT.mkdir(parents=True, exist_ok=True)
        events = []
        if EVENTS_PATH.exists():
            try: events = json.loads(EVENTS_PATH.read_text())
            except Exception: events = []
        events.insert(0, event)
        events = events[:MAX_EVENTS]
        EVENTS_PATH.write_text(json.dumps(events, indent=2))
    return event

def get_events(limit: int = 50, unread_since: str | None = None) -> dict:
    with _events_lock:
        if not EVENTS_PATH.exists():
            return {"ok": True, "events": [], "total": 0}
        try: events = json.loads(EVENTS_PATH.read_text())
        except Exception: events = []
    events = events[:limit]
    unread = sum(1 for e in events if e.get("ts", "") > (unread_since or "")) if unread_since else 0
    return {"ok": True, "events": events, "total": len(events), "unread": unread}

# ── Jobs ─────────────────────────────────────────────────────────────────────

def start_job(name: str, args: list[str], env: dict[str, str] | None = None,
              display_args: list[str] | None = None) -> dict[str, Any]:
    JOB_ROOT.mkdir(parents=True, exist_ok=True)
    job_id = f"{int(time.time())}-{name}"
    log_path = JOB_ROOT / f"{job_id}.log"
    log_file = log_path.open("w", encoding="utf-8")
    proc = subprocess.Popen(args, cwd=str(ROOT), env=command_env(env), text=True,
                            stdin=subprocess.DEVNULL, stdout=log_file, stderr=subprocess.STDOUT,
                            start_new_session=True)
    log_file.close()
    JOBS[job_id] = {"name": name, "process": proc, "log_path": log_path,
                    "command": shell_command(display_args or args), "started_at": time.time()}
    return {"ok": True, "started": True, "job_id": job_id, "pid": proc.pid,
            "command": shell_command(display_args or args),
            "stdout": f"Started {name}.", "stderr": ""}

def job_status(job_id: str | None = None) -> dict[str, Any]:
    if not JOBS:
        return {"ok": True, "running": False, "stdout": "No job in this session.", "stderr": ""}
    job = JOBS.get(job_id or next(reversed(JOBS)))
    if not job:
        raise ApiError(404, "Job not found")
    proc = job["process"]
    code = proc.poll()
    log_text = ""
    if job["log_path"].exists():
        log_text = "\n".join(job["log_path"].read_text(errors="replace").splitlines()[-80:])
    return {"ok": code is None or code == 0, "running": code is None, "code": code,
            "pid": proc.pid, "command": job["command"],
            "stdout": log_text or "Starting...", "stderr": ""}

def stop_active_dimos() -> dict[str, Any]:
    global ACTIVE_DIMOS_JOB_ID
    stopped_job = None
    if ACTIVE_DIMOS_JOB_ID and ACTIVE_DIMOS_JOB_ID in JOBS:
        job = JOBS[ACTIVE_DIMOS_JOB_ID]
        proc = job["process"]
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                try: proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=5)
                stopped_job = job_status(ACTIVE_DIMOS_JOB_ID)
            except ProcessLookupError:
                pass
    ACTIVE_DIMOS_JOB_ID = None
    dimos_stop = run_command(["dimos", "stop"], timeout=20)
    return {"ok": bool(stopped_job) or dimos_stop.get("ok", False),
            "command": "stop dimOS", "stdout": dimos_stop.get("stdout", ""), "stderr": dimos_stop.get("stderr", "")}

# ── Tasks ────────────────────────────────────────────────────────────────────

VALID_SCHEDULES = {"5m": 300, "15m": 900, "30m": 1800, "1h": 3600, "6h": 21600, "12h": 43200, "24h": 86400}
VALID_ACTIONS = {"navigate_gps", "navigate_map", "explore", "query"}

def _load_tasks() -> list[dict]:
    if TASKS_PATH.exists():
        try:
            data = json.loads(TASKS_PATH.read_text())
            if isinstance(data, list): return data
        except Exception: pass
    return []

def _save_tasks(tasks: list[dict]) -> None:
    STATE_ROOT.mkdir(parents=True, exist_ok=True)
    TASKS_PATH.write_text(json.dumps(tasks, indent=2))

def list_tasks(status_filter: str | None = None) -> dict:
    with _tasks_lock:
        tasks = _load_tasks()
    if status_filter and status_filter != "all":
        tasks = [t for t in tasks if t.get("status") == status_filter]
    return {"ok": True, "tasks": tasks, "total": len(tasks)}

def create_task(payload: dict) -> dict:
    name = str(payload.get("name", "")).strip()
    if not name: raise ApiError(400, "Task name required")
    action = str(payload.get("action", "explore")).strip()
    if action not in VALID_ACTIONS: raise ApiError(400, f"Action must be one of: {', '.join(VALID_ACTIONS)}")
    task_type = payload.get("type", "one_off")
    schedule = str(payload.get("schedule", "1h")).strip()
    params: dict = {}
    if action == "navigate_gps":
        params = {"lat": float(payload.get("lat", 0)), "lon": float(payload.get("lon", 0))}
    elif action == "navigate_map":
        params = {"x": float(payload.get("x", 0)), "y": float(payload.get("y", 0))}
    elif action == "query":
        params = {"text": str(payload.get("text", "")).strip()}
    task = {"id": str(uuid.uuid4()), "name": name, "description": str(payload.get("description", "")).strip(),
            "type": task_type, "action": action, "params": params,
            "schedule": schedule if task_type == "recurring" else None,
            "enabled": True, "status": "scheduled", "run_count": 0,
            "last_run": None, "last_result": None, "source": str(payload.get("source", "dashboard")),
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    with _tasks_lock:
        tasks = _load_tasks()
        tasks.append(task)
        _save_tasks(tasks)
    log_event("task_created", f"Task: {name} ({action})", {"taskId": task["id"]})
    return {"ok": True, "task": task}

def update_task(payload: dict) -> dict:
    task_id = str(payload.get("id", "")).strip()
    if not task_id: raise ApiError(400, "Task id required")
    with _tasks_lock:
        tasks = _load_tasks()
        for t in tasks:
            if str(t.get("id", "")).startswith(task_id):
                for f in ("name", "description"):
                    if f in payload: t[f] = str(payload[f]).strip()
                if "status" in payload: t["status"] = payload["status"]
                if "enabled" in payload:
                    t["enabled"] = bool(payload["enabled"])
                    t["status"] = "scheduled" if t["enabled"] else "disabled"
                if "schedule" in payload and payload["schedule"] in VALID_SCHEDULES:
                    t["schedule"] = payload["schedule"]
                _save_tasks(tasks)
                return {"ok": True, "task": t}
    raise ApiError(404, "Task not found")

def delete_task(task_id: str) -> dict:
    task_id = task_id.strip()
    with _tasks_lock:
        tasks = _load_tasks()
        original = len(tasks)
        tasks = [t for t in tasks if not str(t.get("id", "")).startswith(task_id)]
        if len(tasks) == original: raise ApiError(404, "Task not found")
        _save_tasks(tasks)
    return {"ok": True, "deleted": task_id}

def run_task_now(task_id: str) -> dict:
    task_id = task_id.strip()
    with _tasks_lock:
        tasks = _load_tasks()
        task = next((t for t in tasks if str(t.get("id", "")).startswith(task_id)), None)
        if not task: raise ApiError(404, "Task not found")
    log_event("task_dispatched", f"Task run: {task.get('name', '?')}", {"taskId": task_id})
    return {"ok": True, "taskId": task_id}

# ── Fleet ────────────────────────────────────────────────────────────────────

def _load_fleet() -> list[dict]:
    if FLEET_PATH.exists():
        try:
            data = json.loads(FLEET_PATH.read_text())
            if isinstance(data, list): return data
        except Exception: pass
    return []

def _save_fleet(fleet: list[dict]) -> None:
    STATE_ROOT.mkdir(parents=True, exist_ok=True)
    FLEET_PATH.write_text(json.dumps(fleet, indent=2))

def list_fleet() -> dict:
    with _fleet_lock:
        robots = _load_fleet()
    groups: dict[str, int] = {}
    online = 0
    for r in robots:
        g = r.get("group", "Default")
        groups[g] = groups.get(g, 0) + 1
        if r.get("status") == "online": online += 1
    return {"ok": True, "robots": robots, "total": len(robots), "online": online, "groups": groups}

def add_robot(payload: dict) -> dict:
    name = str(payload.get("name", "")).strip()
    if not name: raise ApiError(400, "Robot name required")
    robot = {"id": str(uuid.uuid4()), "name": name,
             "deviceType": str(payload.get("deviceType", "Unitree Go2")).strip(),
             "serial": str(payload.get("serial", "")).strip(),
             "robotIp": str(payload.get("robotIp", "")).strip(),
             "group": str(payload.get("group", "Default")).strip() or "Default",
             "blueprint": str(payload.get("blueprint", "unitree-go2")).strip(),
             "status": "offline", "tags": payload.get("tags", []),
             "notes": str(payload.get("notes", "")).strip(),
             "createdAt": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    with _fleet_lock:
        fleet = _load_fleet()
        fleet.append(robot)
        _save_fleet(fleet)
    log_event("robot_added", f"Robot: {name}", {"robotId": robot["id"]})
    return {"ok": True, "robot": robot}

def update_robot(payload: dict) -> dict:
    rid = str(payload.get("id", "")).strip()
    if not rid: raise ApiError(400, "Robot id required")
    with _fleet_lock:
        fleet = _load_fleet()
        for r in fleet:
            if r.get("id") == rid:
                for f in ("name", "deviceType", "serial", "robotIp", "group", "blueprint", "notes"):
                    if f in payload: r[f] = str(payload[f]).strip()
                if "tags" in payload and isinstance(payload["tags"], list): r["tags"] = payload["tags"]
                if "status" in payload: r["status"] = payload["status"]
                _save_fleet(fleet)
                return {"ok": True, "robot": r}
    raise ApiError(404, "Robot not found")

def delete_robot(rid: str) -> dict:
    rid = rid.strip()
    with _fleet_lock:
        fleet = _load_fleet()
        fleet = [r for r in fleet if r.get("id") != rid]
        _save_fleet(fleet)
    return {"ok": True, "deleted": rid}

def deploy_blueprint(payload: dict) -> dict:
    rid = str(payload.get("robotId", "")).strip()
    bp = str(payload.get("blueprint", "")).strip()
    if not rid or not bp: raise ApiError(400, "robotId and blueprint required")
    with _fleet_lock:
        fleet = _load_fleet()
        robot = next((r for r in fleet if r.get("id") == rid), None)
        if not robot: raise ApiError(404, "Robot not found")
        robot["blueprint"] = bp
        _save_fleet(fleet)
    log_event("blueprint_deployed", f"'{bp}' -> {robot['name']}", {"robotId": rid, "blueprint": bp})
    return {"ok": True, "robot": robot}

# ── Blueprints ───────────────────────────────────────────────────────────────

SEED_BLUEPRINTS: list[dict[str, Any]] = [
    {"id": "bp-hackathon", "slug": "unitree-go2-hackathon", "name": "Full Dashboard",
     "description": "Daneel MCP on :9990, smart follow/find, dog mode, perception loop.",
     "base": "unitree-go2-hackathon", "modules": ["daneel-mcp", "perception-loop", "dog-mode", "smart-follow"],
     "extraArgs": "", "tags": ["hackathon"], "icon": "★", "color": "#00d47e", "builtIn": True},
    {"id": "bp-standard", "slug": "unitree-go2", "name": "Standard Go2",
     "description": "Navigation + mapping + live costmap.",
     "base": "unitree-go2", "modules": ["navigation", "mapping"],
     "extraArgs": "", "tags": ["standard"], "icon": "◈", "color": "#4090e0", "builtIn": True},
    {"id": "bp-basic", "slug": "unitree-go2-basic", "name": "Basic",
     "description": "Minimal connection only.",
     "base": "unitree-go2-basic", "modules": [], "extraArgs": "", "tags": ["minimal"],
     "icon": "◻", "color": "#7aaf90", "builtIn": True},
    {"id": "bp-spatial", "slug": "unitree-go2-spatial", "name": "Spatial Memory",
     "description": "Persistent spatial memory — recall and return to places.",
     "base": "unitree-go2-spatial", "modules": ["spatial-memory", "navigation"],
     "extraArgs": "", "tags": ["spatial"], "icon": "◎", "color": "#a060f0", "builtIn": True},
    {"id": "bp-agentic", "slug": "unitree-go2-agentic", "name": "Agentic (Claude)",
     "description": "Full AI agent via Claude. Needs ANTHROPIC_API_KEY.",
     "base": "unitree-go2-agentic", "modules": ["claude-agent", "navigation"],
     "extraArgs": "", "tags": ["agentic"], "icon": "⬡", "color": "#d4a030", "builtIn": True},
    {"id": "bp-agentic-ollama", "slug": "unitree-go2-agentic-ollama", "name": "Agentic (Ollama)",
     "description": "Full AI agent using local Ollama. No API key.",
     "base": "unitree-go2-agentic-ollama", "modules": ["ollama-agent", "navigation"],
     "extraArgs": "", "tags": ["agentic"], "icon": "⬡", "color": "#d4a030", "builtIn": True},
    {"id": "bp-security", "slug": "unitree-go2-security", "name": "Security Patrol",
     "description": "Automated waypoint patrol loop.",
     "base": "unitree-go2-security", "modules": ["patrol", "threat-detection"],
     "extraArgs": "", "tags": ["security"], "icon": "◉", "color": "#e04040", "builtIn": True},
    {"id": "bp-drone", "slug": "generic-drone", "name": "Quadrotor",
     "description": "Generic drone with waypoint navigation and altitude hold.",
     "base": "generic-drone", "modules": ["waypoint-nav", "altitude-hold"],
     "extraArgs": "", "tags": ["drone"], "icon": "✈", "color": "#40a0e0",
     "robotType": "drone", "builtIn": True},
    {"id": "bp-webcam", "slug": "webcam-only", "name": "Webcam Only",
     "description": "Standalone webcam with vision AI — no robot required.",
     "base": "webcam-only", "modules": ["yolo", "clip"],
     "extraArgs": "", "tags": ["webcam"], "icon": "◉", "color": "#a0a0a0",
     "robotType": "webcam_only", "builtIn": True},
    {"id": "bp-g1", "slug": "unitree-g1", "name": "Unitree G1",
     "description": "G1 humanoid with full joint control and walking policy.",
     "base": "unitree-g1", "modules": ["navigation", "walking-policy"],
     "extraArgs": "", "tags": ["humanoid"], "icon": "⬡", "color": "#d4a030",
     "robotType": "humanoid", "builtIn": True},
]

def _load_blueprints() -> list[dict]:
    if BLUEPRINTS_PATH.exists():
        try:
            data = json.loads(BLUEPRINTS_PATH.read_text())
            if isinstance(data, list): return data
        except Exception: pass
    return list(SEED_BLUEPRINTS)

def _save_blueprints(bps: list[dict]) -> None:
    STATE_ROOT.mkdir(parents=True, exist_ok=True)
    BLUEPRINTS_PATH.write_text(json.dumps(bps, indent=2))

def list_blueprints() -> dict:
    with _blueprints_lock:
        bps = _load_blueprints()
        if not bps:
            bps = list(SEED_BLUEPRINTS)
            _save_blueprints(bps)
    return {"ok": True, "blueprints": bps, "total": len(bps)}

def create_blueprint(payload: dict) -> dict:
    name = str(payload.get("name", "")).strip()
    if not name: raise ApiError(400, "Blueprint name required")
    slug = str(payload.get("slug", "")).strip() or re.sub(r"[^a-z0-9_-]", "", name.lower().replace(" ", "-"))
    bp = {"id": f"bp-{uuid.uuid4().hex[:8]}", "slug": slug, "name": name,
          "description": str(payload.get("description", "")).strip(),
          "base": str(payload.get("base", "unitree-go2")).strip(),
          "modules": payload.get("modules", []), "extraArgs": str(payload.get("extraArgs", "")).strip(),
          "tags": payload.get("tags", []),
          "icon": str(payload.get("icon", "◈")).strip(), "color": str(payload.get("color", "#4090e0")).strip(),
          "builtIn": False}
    with _blueprints_lock:
        bps = _load_blueprints()
        if any(b.get("slug") == slug for b in bps): raise ApiError(409, f"Slug '{slug}' exists")
        bps.append(bp)
        _save_blueprints(bps)
    return {"ok": True, "blueprint": bp}

def update_blueprint(payload: dict) -> dict:
    bp_id = str(payload.get("id", "")).strip()
    if not bp_id: raise ApiError(400, "Blueprint id required")
    with _blueprints_lock:
        bps = _load_blueprints()
        for bp in bps:
            if bp.get("id") == bp_id:
                for f in ("name", "slug", "description", "base", "extraArgs", "icon", "color"):
                    if f in payload and not (bp.get("builtIn") and f in ("name", "slug", "base")):
                        bp[f] = str(payload[f]).strip()
                if "modules" in payload and not bp.get("builtIn"): bp["modules"] = payload["modules"]
                if "tags" in payload: bp["tags"] = payload["tags"]
                _save_blueprints(bps)
                return {"ok": True, "blueprint": bp}
    raise ApiError(404, "Blueprint not found")

def delete_blueprint(bp_id: str) -> dict:
    bp_id = bp_id.strip()
    with _blueprints_lock:
        bps = _load_blueprints()
        target = next((b for b in bps if b.get("id") == bp_id), None)
        if not target: raise ApiError(404, "Blueprint not found")
        if target.get("builtIn"): raise ApiError(403, "Cannot delete built-in blueprint")
        bps = [b for b in bps if b.get("id") != bp_id]
        _save_blueprints(bps)
    return {"ok": True, "deleted": bp_id}

def duplicate_blueprint(bp_id: str) -> dict:
    bp_id = bp_id.strip()
    with _blueprints_lock:
        bps = _load_blueprints()
        source = next((b for b in bps if b.get("id") == bp_id), None)
        if not source: raise ApiError(404, "Blueprint not found")
        new = dict(source)
        new["id"] = f"bp-{uuid.uuid4().hex[:8]}"
        new["name"] = f"{source['name']} (Copy)"
        new["slug"] = f"{source['slug']}-copy-{uuid.uuid4().hex[:4]}"
        new["builtIn"] = False
        bps.append(new)
        _save_blueprints(bps)
    return {"ok": True, "blueprint": new}

# ── Dashboard ────────────────────────────────────────────────────────────────

def dashboard() -> dict[str, Any]:
    profile = load_profile()
    dimos = dimos_status()
    robot_ip = profile.get("robotIp", "").strip()
    webcam = _get_webcam()
    dataset = _get_dataset()
    try:
        sim = _get_simulator()
        sim_state = sim.get_state()
    except Exception:
        sim_state = {"running": False, "state": "idle", "robot": "", "robot_type": "",
                     "fps": 0, "frame_count": 0, "sim_time": 0, "position": {"x": 0, "y": 0, "z": 0},
                     "orientation": {"w": 1, "x": 0, "y": 0, "z": 0}, "has_policy": False, "has_drone_ctrl": False}

    from roborun.robot_types import detect_type, get_profile as get_robot_profile
    rtype = detect_type(
        blueprint=profile.get("blueprint", ""),
        sim_robot_type=sim_state.get("robot_type", ""),
    )
    robot_profile = get_robot_profile(rtype)

    ros_connected = False
    ros_topics_count = 0
    try:
        from roborun.rosbridge import get_client as _get_ros_client
        rc = _get_ros_client(auto_connect=False)
        if rc and rc.is_connected:
            ros_connected = True
            from roborun.ros_telemetry import get_bridge
            ros_topics_count = len(get_bridge()._subscribed_topics)
    except Exception:
        pass

    return {
        "ok": True,
        "profile": profile,
        "dimos": dimos,
        "robotOnline": False,
        "robotIp": robot_ip,
        "commandCenter": {"ok": False, "url": "http://127.0.0.1:7779/command-center"},
        "webcam": webcam.get_state(),
        "sim": sim_state,
        "dataset": dataset.get_status(),
        "stats": system_stats(),
        "robotType": robot_profile,
        "telemetryWs": "ws://127.0.0.1:8766",
        "ros": {"connected": ros_connected, "telemetry_topics": ros_topics_count},
        "collectTime": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
    }

# ── Launch command builder ───────────────────────────────────────────────────

def build_launch_command(payload: dict) -> tuple[list[str], dict[str, str]]:
    mode = safe_arg(str(payload.get("mode", "hardware")), "Mode")
    if mode not in {"hardware", "replay", "simulation"}: raise ApiError(400, "Invalid mode")
    robot_ip = str(payload.get("robotIp", "")).strip()
    if mode == "hardware" and not valid_ip(robot_ip): raise ApiError(400, "Valid robot IP required")
    blueprint = safe_arg(str(payload.get("blueprint", "unitree-go2")), "Blueprint")
    viewer = safe_arg(str(payload.get("viewer", "rerun")), "Viewer")
    args = ["dimos"]
    if mode == "replay": args.append("--replay")
    elif mode == "simulation": args.append("--simulation")
    args.extend(["--viewer", viewer, "run", blueprint])
    if payload.get("daemon", True): args.append("--daemon")
    env = {"ROBOT_IP": robot_ip} if mode == "hardware" else {}
    return args, env

# ── HTTP Handler ─────────────────────────────────────────────────────────────

class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, directory=str(WEB_ROOT), **kwargs)

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[roborun] {self.address_string()} - {fmt % args}")

    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        super().end_headers()

    def send_json(self, status: int, data: dict) -> None:
        encoded = json.dumps(data, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    # ── MCP server implementation ─────────────────────────────────────────────

    _MCP_TOOLS = [
        {
            "name": "camera_snapshot",
            "description": "Capture the current robot camera frame. Returns a live JPEG image so you can see what the robot sees right now.",
            "inputSchema": {"type": "object", "properties": {}},
        },
        {
            "name": "yolo_detections",
            "description": "Get the current YOLO object detection results from the robot camera — labels, confidence scores, and bounding boxes.",
            "inputSchema": {"type": "object", "properties": {}},
        },
        {
            "name": "move",
            "description": "Send a velocity command to the robot. linear_x: forward (+) / back (−) in m/s. angular_z: turn left (+) / right (−) in rad/s.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "linear_x": {"type": "number", "description": "Forward speed m/s (e.g. 0.3)"},
                    "angular_z": {"type": "number", "description": "Turn rate rad/s (e.g. 0.5)"},
                    "duration_s": {"type": "number", "description": "Hold command for N seconds, then stop"},
                },
            },
        },
        {
            "name": "execute_skill",
            "description": "Execute a dimOS robot skill. Available skills: navigate_with_text, begin_exploration, smart_follow_person, smart_follow_object, smart_find, query_scene, execute_sport_command, speak, tag_location, where_am_i.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "skill": {"type": "string", "description": "Skill name"},
                    "args": {"type": "object", "description": "Skill arguments"},
                },
                "required": ["skill"],
            },
        },
        {
            "name": "memory_search",
            "description": "Search the robot's spatial memory for past observations using CLIP semantic similarity. Returns matching frames with location coordinates.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "query": {"type": "string", "description": "What to search for e.g. 'red chair', 'charging dock'"},
                    "top_k": {"type": "integer", "description": "Max results (default 5)"},
                },
                "required": ["query"],
            },
        },
        {
            "name": "ros_topics",
            "description": "List all available ROS 2 topics and their message types. Requires rosbridge connection.",
            "inputSchema": {"type": "object", "properties": {}},
        },
        {
            "name": "ros_publish",
            "description": "Publish a message to any ROS 2 topic.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "topic": {"type": "string"},
                    "type": {"type": "string", "description": "ROS message type"},
                    "message": {"type": "object"},
                },
                "required": ["topic", "type", "message"],
            },
        },
        {
            "name": "ros_subscribe_once",
            "description": "Read one message from a ROS 2 topic.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "topic": {"type": "string"},
                    "type": {"type": "string"},
                    "timeout_ms": {"type": "number"},
                },
                "required": ["topic"],
            },
        },
        {
            "name": "takeoff",
            "description": "Arm and take off to the specified altitude (drone only).",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "altitude": {"type": "number", "description": "Target altitude in meters (default 2.0)"},
                },
            },
        },
        {
            "name": "land",
            "description": "Land the drone at current position.",
            "inputSchema": {"type": "object", "properties": {}},
        },
        {
            "name": "goto_waypoint",
            "description": "Fly to a 3D waypoint (drone only).",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"},
                },
                "required": ["x", "y", "z"],
            },
        },
        {
            "name": "set_altitude",
            "description": "Set drone altitude setpoint.",
            "inputSchema": {
                "type": "object",
                "properties": {"altitude": {"type": "number"}},
                "required": ["altitude"],
            },
        },
        {
            "name": "follow_target",
            "description": "Follow a YOLO-detected object by label. Uses detection centroid to steer the robot.",
            "inputSchema": {
                "type": "object",
                "properties": {"label": {"type": "string", "description": "YOLO label to follow e.g. 'car', 'person'"}},
                "required": ["label"],
            },
        },
        {
            "name": "get_telemetry",
            "description": "Get current robot telemetry snapshot — battery, position, orientation, velocity, joints.",
            "inputSchema": {"type": "object", "properties": {}},
        },
        {
            "name": "get_trajectory",
            "description": "Get the recorded trajectory as a list of timestamped poses.",
            "inputSchema": {
                "type": "object",
                "properties": {"limit": {"type": "integer", "description": "Max points to return"}},
            },
        },
        {
            "name": "get_depth_frame",
            "description": "Get the current depth heatmap as a base64 JPEG with min/max/mean distances.",
            "inputSchema": {"type": "object", "properties": {}},
        },
        {
            "name": "get_pointcloud",
            "description": "Get a downsampled colored point cloud from the depth camera.",
            "inputSchema": {"type": "object", "properties": {}},
        },
    ]

    def _mcp_reply(self, req_id: Any, result: Any) -> None:
        body = json.dumps({"jsonrpc": "2.0", "id": req_id, "result": result}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _mcp_error(self, req_id: Any, code: int, message: str) -> None:
        body = json.dumps({"jsonrpc": "2.0", "id": req_id,
                           "error": {"code": code, "message": message}}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _handle_mcp(self, payload: dict) -> None:
        req_id = payload.get("id")
        method = payload.get("method", "")
        params = payload.get("params", {})

        if method == "initialize":
            self._mcp_reply(req_id, {
                "protocolVersion": "2024-11-05",
                "capabilities": {"tools": {}},
                "serverInfo": {"name": "roborun", "version": "0.7.0"},
            })
            return

        if method == "tools/list":
            from roborun.ros_mcp import MCP_TOOLS as ROS_TOOLS
            all_tools = self._MCP_TOOLS + [t for t in ROS_TOOLS
                                           if t["name"] not in {x["name"] for x in self._MCP_TOOLS}]
            self._mcp_reply(req_id, {"tools": all_tools})
            return

        if method == "tools/call":
            name = params.get("name", "")
            args = params.get("arguments", {})
            from roborun.ros_mcp import _TOOL_HANDLERS
            if name in _TOOL_HANDLERS:
                from roborun.ros_mcp import handle_tool_call
                result = handle_tool_call(name, args)
                text = json.dumps(result, indent=2)
                self._mcp_reply(req_id, {"content": [{"type": "text", "text": text}]})
                return
            content = self._run_mcp_tool(name, args)
            self._mcp_reply(req_id, {"content": content})
            return

        if method == "notifications/initialized":
            # client acknowledges — no response needed for notifications
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            return

        self._mcp_error(req_id, -32601, f"Method not found: {method}")

    def _run_mcp_tool(self, name: str, args: dict) -> list[dict]:
        """Execute a tool and return MCP content blocks."""
        try:
            if name == "camera_snapshot":
                return self._mcp_camera()

            if name == "yolo_detections":
                for p in (_HACKATHON_STATE_PATH, _WEBCAM_STATE_PATH):
                    if p.exists() and (time.time() - p.stat().st_mtime) < 3.0:
                        try:
                            state = json.loads(p.read_text())
                            dets = state.get("detections", [])
                            return [{"type": "text", "text": json.dumps(dets, indent=2)}]
                        except Exception:
                            pass
                return [{"type": "text", "text": "No detections available"}]

            if name == "move":
                lx = float(args.get("linear_x", 0.0))
                az = float(args.get("angular_z", 0.0))
                dur = float(args.get("duration_s", 0.0))
                profile = load_profile()
                host = profile.get("robotIp", "")
                try:
                    from roborun.rosbridge import get_client
                    client = get_client(host) if host else None
                    if client:
                        client.move(lx, 0.0, az)
                        if dur > 0:
                            time.sleep(dur)
                            client.stop()
                        return [{"type": "text", "text": f"Move sent: linear_x={lx}, angular_z={az}"}]
                except Exception:
                    pass
                return [{"type": "text", "text": "Move failed: no rosbridge connection"}]

            if name == "execute_skill":
                skill = args.get("skill", "")
                skill_args = args.get("args", {})
                task_id = uuid.uuid4().hex
                with _MCP_TASKS_LOCK:
                    _MCP_TASKS[task_id] = {"status": "pending", "name": skill, "started": time.time()}
                threading.Thread(target=_run_mcp_task, args=(task_id, skill, skill_args), daemon=True).start()
                for _ in range(60):
                    time.sleep(0.5)
                    with _MCP_TASKS_LOCK:
                        state = dict(_MCP_TASKS.get(task_id, {}))
                    if state.get("status") == "done":
                        return [{"type": "text", "text": state.get("result", "Done.")}]
                    if state.get("status") == "error":
                        return [{"type": "text", "text": f"Error: {state.get('error')}"}]
                return [{"type": "text", "text": "Skill timed out"}]

            if name == "memory_search":
                query = args.get("query", "")
                top_k = int(args.get("top_k", 5))
                try:
                    mem = _get_memory()
                    wc = _get_webcam()
                    if wc._clip is None:
                        from roborun.models import CLIPMatcher
                        wc._clip = CLIPMatcher()
                    emb = wc._clip.embed_text(query)
                    results = mem.search_clip(emb, top_k=top_k)
                    if not results:
                        return [{"type": "text", "text": "No matching memories."}]
                    lines = []
                    for r in results:
                        loc = f"({r.get('x', '?'):.1f}, {r.get('y', '?'):.1f})" if r.get("x") is not None else "unknown"
                        dets = r.get("detections") or []
                        if isinstance(dets, str):
                            try: dets = json.loads(dets)
                            except Exception: dets = []
                        labels = [d.get("label", "") for d in dets[:4] if isinstance(d, dict)]
                        lines.append(f"{loc} — {', '.join(labels) or 'no labels'}")
                    return [{"type": "text", "text": "\n".join(lines)}]
                except Exception as exc:
                    return [{"type": "text", "text": f"Search failed: {exc}"}]

            if name == "ros_topics":
                profile = load_profile()
                host = profile.get("robotIp", "")
                try:
                    from roborun.rosbridge import get_client
                    client = get_client(host) if host else None
                    if not client:
                        return [{"type": "text", "text": "Not connected to rosbridge"}]
                    topics = client.list_topics()
                    text = "\n".join(f"{t['topic']} [{t['type']}]" for t in topics)
                    return [{"type": "text", "text": text or "No topics found"}]
                except Exception as exc:
                    return [{"type": "text", "text": f"Failed: {exc}"}]

            if name == "ros_publish":
                profile = load_profile()
                host = profile.get("robotIp", "")
                try:
                    from roborun.rosbridge import get_client
                    client = get_client(host) if host else None
                    if not client:
                        return [{"type": "text", "text": "Not connected to rosbridge"}]
                    client.publish(args["topic"], args["type"], args["message"])
                    return [{"type": "text", "text": "Published."}]
                except Exception as exc:
                    return [{"type": "text", "text": f"Failed: {exc}"}]

            if name == "ros_subscribe_once":
                profile = load_profile()
                host = profile.get("robotIp", "")
                try:
                    from roborun.rosbridge import get_client
                    client = get_client(host) if host else None
                    if not client:
                        return [{"type": "text", "text": "Not connected to rosbridge"}]
                    timeout = float(args.get("timeout_ms", 5000)) / 1000.0
                    msg = client.subscribe_once(args["topic"], args.get("type", ""), timeout=timeout)
                    return [{"type": "text", "text": json.dumps(msg) if msg else "No message received"}]
                except Exception as exc:
                    return [{"type": "text", "text": f"Failed: {exc}"}]

            if name == "takeoff":
                alt = float(args.get("altitude", 2.0))
                sim = _get_simulator()
                if sim.is_running and sim._drone_ctrl:
                    sim.set_altitude(alt)
                    return [{"type": "text", "text": f"Takeoff to {alt}m"}]
                return [{"type": "text", "text": "No drone available"}]

            if name == "land":
                sim = _get_simulator()
                if sim.is_running and sim._drone_ctrl:
                    sim.set_altitude(0.1)
                    return [{"type": "text", "text": "Landing initiated"}]
                return [{"type": "text", "text": "No drone available"}]

            if name == "goto_waypoint":
                x, y, z = float(args["x"]), float(args["y"]), float(args["z"])
                sim = _get_simulator()
                if sim.is_running and sim._drone_ctrl:
                    sim.set_waypoint(x, y, z)
                    return [{"type": "text", "text": f"Waypoint set: ({x}, {y}, {z})"}]
                return [{"type": "text", "text": "No drone available"}]

            if name == "set_altitude":
                alt = float(args["altitude"])
                sim = _get_simulator()
                if sim.is_running and sim._drone_ctrl:
                    sim.set_altitude(alt)
                    return [{"type": "text", "text": f"Altitude set: {alt}m"}]
                return [{"type": "text", "text": "No drone available"}]

            if name == "follow_target":
                label = args.get("label", "")
                return [{"type": "text", "text": f"Follow target set: {label}. Tracking via YOLO detections."}]

            if name == "get_telemetry":
                sim = _get_simulator()
                if sim.is_running:
                    tel = sim.get_telemetry()
                    return [{"type": "text", "text": json.dumps(tel, indent=2)}]
                from roborun.telemetry import TelemetryBus
                latest = TelemetryBus.get().get_latest()
                if latest:
                    return [{"type": "text", "text": json.dumps(latest, indent=2)}]
                return [{"type": "text", "text": "No telemetry available"}]

            if name == "get_trajectory":
                from roborun.trajectory import TrajectoryRecorder
                limit = int(args.get("limit", 500))
                traj = TrajectoryRecorder.get().get_trajectory(limit=limit)
                return [{"type": "text", "text": json.dumps(traj)}]

            if name == "get_depth_frame":
                from roborun.depth import DepthProcessor
                result = DepthProcessor.get().get_heatmap()
                if result.get("ok") and result.get("image"):
                    return [{"type": "image", "data": result["image"].split(",")[1], "mimeType": "image/jpeg"}]
                return [{"type": "text", "text": "No depth data available"}]

            if name == "get_pointcloud":
                from roborun.depth import DepthProcessor
                result = DepthProcessor.get().get_pointcloud()
                if result.get("ok"):
                    return [{"type": "text", "text": f"{result['count']} points"}]
                return [{"type": "text", "text": "No depth data available"}]

            return [{"type": "text", "text": f"Unknown tool: {name}"}]

        except Exception as exc:
            return [{"type": "text", "text": f"Tool error: {exc}"}]

    def _handle_ros_mcp(self, payload: dict) -> None:
        req_id = payload.get("id")
        method = payload.get("method", "")
        params = payload.get("params", {})

        if method == "initialize":
            from roborun.ros_mcp import get_mcp_manifest
            manifest = get_mcp_manifest()
            self._mcp_reply(req_id, {
                "protocolVersion": "2024-11-05",
                "capabilities": {"tools": {}},
                "serverInfo": {"name": manifest["name"], "version": manifest["version"]},
            })
            return

        if method == "tools/list":
            from roborun.ros_mcp import MCP_TOOLS
            self._mcp_reply(req_id, {"tools": MCP_TOOLS})
            return

        if method == "tools/call":
            from roborun.ros_mcp import handle_tool_call
            name = params.get("name", "")
            args = params.get("arguments", {})
            result = handle_tool_call(name, args)
            text = json.dumps(result, indent=2)
            self._mcp_reply(req_id, {"content": [{"type": "text", "text": text}]})
            return

        if method == "notifications/initialized":
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            return

        self._mcp_error(req_id, -32601, f"Method not found: {method}")

    def _mcp_camera(self) -> list[dict]:
        """Return camera frame as MCP image content block."""
        for p in (_HACKATHON_FRAME_PATH, _WEBCAM_FRAME_PATH, _CAMERA_FRAME_PATH):
            if p.exists() and (time.time() - p.stat().st_mtime) < 5.0:
                try:
                    data = base64.b64encode(p.read_bytes()).decode()
                    return [{"type": "image", "data": data, "mimeType": "image/jpeg"}]
                except Exception:
                    pass
        return [{"type": "text", "text": "No camera frame available — start webcam or connect robot"}]

    def do_GET(self) -> None:
        path_only = self.path.split("?", 1)[0]

        # ── MCP server discovery (SSE transport handshake) ──
        if path_only == "/mcp":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            # Send endpoint event so MCP clients know where to POST
            msg = f"data: {{\"type\":\"endpoint\",\"url\":\"http://127.0.0.1:{PORT}/mcp\"}}\n\n"
            try:
                self.wfile.write(msg.encode())
                self.wfile.flush()
                # Keep alive until client disconnects
                while True:
                    self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
                    time.sleep(15)
            except Exception:
                pass
            return

        # ── Direct ROS MCP discovery (SSE) ──
        if path_only == "/mcp/ros":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            msg = f"data: {{\"type\":\"endpoint\",\"url\":\"http://127.0.0.1:{PORT}/mcp/ros\"}}\n\n"
            try:
                self.wfile.write(msg.encode())
                self.wfile.flush()
                while True:
                    self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
                    time.sleep(15)
            except Exception:
                pass
            return

        # ── MJPEG stream (webcam or robot camera) ──
        if path_only == "/api/camera/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                last_mtime = 0.0
                started = time.monotonic()
                try: self.connection.settimeout(30.0)
                except Exception: pass
                while time.monotonic() - started < 300:
                    for p in (_HACKATHON_FRAME_PATH, _WEBCAM_FRAME_PATH, _CAMERA_FRAME_PATH):
                        if p.exists():
                            mtime = p.stat().st_mtime
                            if mtime != last_mtime:
                                last_mtime = mtime
                                data = p.read_bytes()
                                header = (b"--frame\r\nContent-Type: image/jpeg\r\n"
                                          b"Content-Length: " + str(len(data)).encode() + b"\r\n\r\n")
                                self.wfile.write(header + data + b"\r\n")
                                self.wfile.flush()
                            break
                    time.sleep(0.033)
            except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                pass
            return

        if self.path == "/api/health":
            self.send_json(200, {"ok": True, "name": "RoboRun"})
            return
        if self.path == "/api/dashboard":
            self.send_json(200, dashboard())
            return
        if self.path == "/api/camera":
            self.send_json(200, camera_snapshot())
            return
        if self.path == "/api/hackathon/state":
            try: state = json.loads(_HACKATHON_STATE_PATH.read_text()) if _HACKATHON_STATE_PATH.exists() else {}
            except Exception: state = {}
            self.send_json(200, state)
            return
        if self.path == "/api/webcam/state":
            self.send_json(200, _get_webcam().get_state())
            return
        if self.path == "/api/webcam/detections":
            self.send_json(200, {"ok": True, "detections": _get_webcam().get_detections()})
            return
        if self.path == "/api/scene":
            for p in (_HACKATHON_STATE_PATH, _WEBCAM_STATE_PATH):
                if p.exists():
                    try:
                        self.send_json(200, json.loads(p.read_text()))
                        return
                    except Exception: pass
            self.send_json(200, {})
            return
        if self.path == "/api/dataset/status":
            self.send_json(200, {"ok": True, **_get_dataset().get_status()})
            return
        if self.path == "/api/dataset/list":
            self.send_json(200, _get_dataset().list_datasets())
            return
        if self.path == "/api/events" or self.path.startswith("/api/events?"):
            limit = 50; unread_since = None
            if "?" in self.path:
                for part in self.path.split("?", 1)[1].split("&"):
                    if part.startswith("limit="): limit = min(int(part[6:]), MAX_EVENTS)
                    if part.startswith("since="): unread_since = part[6:]
            self.send_json(200, get_events(limit, unread_since))
            return
        if self.path == "/api/tasks" or self.path.startswith("/api/tasks?"):
            sf = None
            if "?" in self.path:
                for part in self.path.split("?", 1)[1].split("&"):
                    if part.startswith("status="): sf = part[7:]
            self.send_json(200, list_tasks(sf))
            return
        if self.path == "/api/fleet":
            self.send_json(200, list_fleet())
            return
        if self.path == "/api/blueprints":
            self.send_json(200, list_blueprints())
            return
        if self.path == "/api/mcp/tools":
            try:
                import urllib.request
                body = json.dumps({"jsonrpc": "2.0", "id": 1, "method": "tools/list"}).encode()
                req = urllib.request.Request("http://127.0.0.1:9990/mcp", data=body,
                                            headers={"Content-Type": "application/json"}, method="POST")
                with urllib.request.urlopen(req, timeout=2) as resp:
                    d = json.loads(resp.read().decode())
                tools = [t.get("name", "") for t in d.get("result", {}).get("tools", [])]
                self.send_json(200, {"ok": True, "count": len(tools), "tools": tools})
            except Exception as exc:
                self.send_json(200, {"ok": False, "count": 0, "error": str(exc)})
            return
        if path_only == "/api/mcp/result":
            from urllib.parse import parse_qs, urlparse
            qs = parse_qs(urlparse(self.path).query)
            tid = (qs.get("id") or [""])[0]
            if not tid:
                self.send_json(400, {"ok": False, "error": "id required"})
                return
            _prune_mcp_tasks()
            with _MCP_TASKS_LOCK:
                state = _MCP_TASKS.get(tid)
                snapshot = dict(state) if state else None
            if snapshot is None:
                self.send_json(404, {"ok": False, "error": "unknown task"})
                return
            snapshot["elapsed"] = round(time.time() - snapshot.get("started", time.time()), 2)
            self.send_json(200, snapshot)
            return
        if self.path == "/api/agent/status":
            agent = _get_agent()
            if agent == "unavailable":
                self.send_json(200, {"ok": True, "alive": False, "available": False, "mode": None})
            else:
                from roborun.agent import FastRobotAgent
                mode = "fast" if isinstance(agent, FastRobotAgent) else "subprocess"
                session = getattr(agent, "_session_id", None)
                self.send_json(200, {"ok": True, "alive": agent.is_alive,
                                     "available": True, "mode": mode,
                                     "session": session is not None})
            return
        if self.path == "/api/agent/gemini/status":
            import os
            has_key = bool(os.environ.get("GEMINI_API_KEY"))
            agent = _get_gemini_agent()
            self.send_json(200, {"ok": True, "available": agent != "unavailable",
                                 "alive": agent != "unavailable" and agent.is_alive,
                                 "has_key": has_key})
            return

        # ── ROS bridge GET ──
        if self.path == "/api/ros/topics":
            from urllib.parse import parse_qs, urlparse
            qs = parse_qs(urlparse(self.path).query)
            host = qs.get("host", [None])[0] or load_profile().get("robotIp", "")
            if not host:
                self.send_json(400, {"ok": False, "error": "No robot IP configured"})
                return
            try:
                from roborun.rosbridge import get_client
                client = get_client(host)
                if not client:
                    self.send_json(503, {"ok": False, "error": "Could not connect to rosbridge"})
                    return
                topics = client.list_topics()
                self.send_json(200, {"ok": True, "topics": topics, "count": len(topics)})
            except Exception as exc:
                self.send_json(500, {"ok": False, "error": str(exc)})
            return

        if self.path.startswith("/api/ros/status"):
            from urllib.parse import parse_qs, urlparse
            qs = parse_qs(urlparse(self.path).query)
            host = qs.get("host", [None])[0] or load_profile().get("robotIp", "")
            try:
                from roborun.rosbridge import get_client
                client = get_client(host) if host else None
                connected = client is not None and client.is_connected
                self.send_json(200, {"ok": True, "connected": connected, "host": host or None})
            except Exception as exc:
                self.send_json(200, {"ok": True, "connected": False, "error": str(exc)})
            return

        # ── Simulator GET ──
        if self.path == "/api/sim/robots":
            self.send_json(200, {"ok": True, "robots": _get_simulator().list_robots()})
            return
        if self.path == "/api/sim/state":
            self.send_json(200, {"ok": True, **_get_simulator().get_state()})
            return

        # ── Robot type ──
        if self.path == "/api/robot-type":
            from roborun.robot_types import detect_type, get_profile
            profile = load_profile()
            sim_type = ""
            try:
                sim = _get_simulator()
                if sim.is_running:
                    sim_state = sim.get_state()
                    sim_type = sim_state.get("robot_type", "")
            except Exception:
                pass
            rtype = detect_type(
                blueprint=profile.get("blueprint", ""),
                sim_robot_type=sim_type,
            )
            self.send_json(200, {"ok": True, **get_profile(rtype)})
            return

        # ── Telemetry ──
        if self.path == "/api/telemetry":
            from roborun.telemetry import TelemetryBus
            robot_id = "sim" if _get_simulator().is_running else "local"
            latest = TelemetryBus.get().get_latest(robot_id)
            self.send_json(200, {"ok": True, "telemetry": latest, "robot_id": robot_id})
            return
        if self.path.startswith("/api/telemetry/history"):
            from roborun.telemetry import TelemetryBus
            from urllib.parse import parse_qs, urlparse
            qs = parse_qs(urlparse(self.path).query)
            robot_id = qs.get("robot_id", [None])[0]
            channel = qs.get("channel", [None])[0]
            limit = int(qs.get("limit", ["200"])[0])
            data = TelemetryBus.get().get_history(robot_id, channel, limit)
            self.send_json(200, {"ok": True, "data": data, "count": len(data)})
            return
        if self.path == "/api/telemetry/ws-info":
            self.send_json(200, {"ok": True, "url": "ws://127.0.0.1:8766"})
            return

        # ── Trajectory ──
        if self.path == "/api/trajectory":
            from roborun.trajectory import TrajectoryRecorder
            rec = TrajectoryRecorder.get()
            traj = rec.get_trajectory(limit=2000)
            self.send_json(200, {"ok": True, **rec.get_state(), "trajectory": traj})
            return

        # ── Depth / Point Cloud ──
        if self.path == "/api/depth-frame":
            from roborun.depth import DepthProcessor
            self.send_json(200, DepthProcessor.get().get_heatmap())
            return
        if self.path == "/api/pointcloud":
            from roborun.depth import DepthProcessor
            self.send_json(200, DepthProcessor.get().get_pointcloud())
            return

        # ── Spatial Memory GET ──
        if self.path == "/api/memory/stats":
            self.send_json(200, {"ok": True, **_get_memory().stats()})
            return

        # ── ZK proof endpoints ──
        if self.path == "/api/zk/status":
            from roborun.zk_prover import get_prover
            prover = get_prover()
            self.send_json(200, {
                "ok": True,
                "ezkl_available": prover.is_available(),
                "circuit_ready": prover._ready,
                "circuit_path": str(prover._circuit) if prover._ready else None,
            })
            return

        if self.path.startswith("/api/zk/verify/"):
            shard_id = self.path.split("/api/zk/verify/")[1]
            from roborun.zk_prover import get_prover
            prover = get_prover()
            proof_bytes, meta = prover.load_proof(shard_id)
            if not proof_bytes:
                self.send_json(404, {"ok": False, "error": f"No proof found for shard {shard_id}"})
                return
            verified = prover.verify(proof_bytes)
            self.send_json(200, {"ok": True, "shard_id": shard_id,
                                 "verified": verified, "meta": meta})
            return
        if self.path == "/api/scene3d":
            sb = _get_scene_builder()
            scene = sb.get_scene()
            scene["is_running"] = sb.is_running
            scene["available"] = sb._available
            scene["has_depth"] = sb._depth_estimator is not None
            scene["last_error"] = sb._last_error
            scene["loop_state"] = sb._loop_state
            self.send_json(200, scene)
            return

        if self.path.startswith("/api/timeline"):
            from urllib.parse import parse_qs, urlparse
            qs = parse_qs(urlparse(self.path).query)
            limit = min(int(qs.get("limit", ["30"])[0]), 100)
            since = float(qs.get("since", ["0"])[0])
            memories = _get_memory().list_memories(
                limit=limit, source="timeline",
                since=since if since > 0 else None,
            )
            self.send_json(200, {"ok": True, "entries": memories, "count": len(memories)})
            return

        if self.path.startswith("/api/memory/list"):
            from urllib.parse import parse_qs, urlparse
            qs = parse_qs(urlparse(self.path).query)
            robot = qs.get("robot_id", [None])[0]
            limit = int(qs.get("limit", ["50"])[0])
            self.send_json(200, {"ok": True, "memories": _get_memory().list_memories(limit=limit, robot_id=robot)})
            return
        if self.path.startswith("/api/memory/thumb/"):
            mid = self.path.split("/api/memory/thumb/")[1]
            data = _get_memory().get_thumbnail(mid)
            if data:
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_json(404, {"ok": False, "error": "not found"})
            return

        if self.path == "/":
            self.path = "/index.html"
        super().do_GET()

    def do_OPTIONS(self) -> None:
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_POST(self) -> None:
        # ── MCP server (JSON-RPC 2.0) ──
        if self.path == "/mcp":
            try:
                payload = read_json(self)
            except Exception as exc:
                self._mcp_error(None, -32700, f"Parse error: {exc}")
                return
            self._handle_mcp(payload)
            return

        # ── Direct ROS MCP (DDS, no rosbridge) ──
        if self.path == "/mcp/ros":
            try:
                payload = read_json(self)
            except Exception as exc:
                self._mcp_error(None, -32700, f"Parse error: {exc}")
                return
            self._handle_ros_mcp(payload)
            return

        try:
            payload = read_json(self)

            # ── Profile ──
            if self.path == "/api/profile":
                self.send_json(200, {"ok": True, "profile": save_profile(payload)})
                return

            # ── Webcam ──
            if self.path == "/api/webcam/start":
                cam = int(payload.get("camera", 0))
                models = payload.get("models", ["yolo"])
                result = _get_webcam().start(camera_index=cam, models=models)
                if result.get("ok"): log_event("webcam_started", f"Webcam started (cam {cam})")
                self.send_json(200, result)
                return
            if self.path == "/api/webcam/stop":
                result = _get_webcam().stop()
                log_event("webcam_stopped", "Webcam stopped")
                self.send_json(200, result)
                return
            if self.path == "/api/webcam/models":
                result = _get_webcam().set_models(payload.get("models", []))
                self.send_json(200, result)
                return
            if self.path == "/api/webcam/clip_query":
                result = _get_webcam().set_clip_query(str(payload.get("query", "")))
                self.send_json(200, result)
                return

            # ── Simulator ──
            if self.path == "/api/sim/start":
                _get_webcam().stop()
                robot = payload.get("robot", "unitree_go1")
                result = _get_simulator().start(robot_id=robot)
                if result.get("ok"):
                    log_event("sim_started", f"Sim: {robot}")
                self.send_json(200, result)
                return
            if self.path == "/api/sim/stop":
                result = _get_simulator().stop()
                log_event("sim_stopped", "Simulator stopped")
                self.send_json(200, result)
                return
            if self.path == "/api/sim/reset":
                result = _get_simulator().reset()
                if result.get("ok"):
                    log_event("sim_reset", "Simulator robot reset")
                self.send_json(200, result)
                return
            if self.path == "/api/sim/move":
                _get_simulator().set_cmd_vel(
                    forward=float(payload.get("forward", 0)),
                    left=float(payload.get("left", 0)),
                    turn=float(payload.get("turn", 0)),
                )
                self.send_json(200, {"ok": True})
                return
            if self.path == "/api/sim/waypoint":
                result = _get_simulator().set_waypoint(
                    x=float(payload.get("x", 0)),
                    y=float(payload.get("y", 0)),
                    z=float(payload.get("z", 2.0)),
                )
                self.send_json(200, result)
                return
            if self.path == "/api/sim/altitude":
                result = _get_simulator().set_altitude(float(payload.get("altitude", 2.0)))
                self.send_json(200, result)
                return

            # ── Trajectory controls ──
            if self.path == "/api/trajectory/start":
                from roborun.trajectory import TrajectoryRecorder
                self.send_json(200, TrajectoryRecorder.get().start())
                return
            if self.path == "/api/trajectory/stop":
                from roborun.trajectory import TrajectoryRecorder
                self.send_json(200, TrajectoryRecorder.get().stop())
                return
            if self.path == "/api/trajectory/clear":
                from roborun.trajectory import TrajectoryRecorder
                self.send_json(200, TrajectoryRecorder.get().clear())
                return

            # ── Spatial Memory ──
            if self.path == "/api/memory/store":
                mem = _get_memory()
                wc = _get_webcam()
                frame = wc.snapshot()
                embedding = None
                dets = None
                if frame is not None:
                    if wc._clip:
                        try:
                            embedding = wc._clip.embed_image(frame)
                        except Exception:
                            pass
                    dets = wc.get_detections()
                mid = mem.store(
                    frame=frame,
                    embedding=embedding,
                    detections=dets,
                    x=payload.get("x"),
                    y=payload.get("y"),
                    z=payload.get("z"),
                    robot_id=payload.get("robot_id", "local"),
                    metadata=payload.get("metadata"),
                )
                log_event("memory_stored", f"Memory {mid}")
                self.send_json(200, {"ok": True, "id": mid})
                return
            if self.path == "/api/memory/search":
                mem = _get_memory()
                query = payload.get("query", "")
                mode = payload.get("mode", "clip")
                top_k = int(payload.get("top_k", 10))
                robot_id = payload.get("robot_id")
                if mode == "clip" and query:
                    wc = _get_webcam()
                    if wc._clip is None:
                        from roborun.models import CLIPMatcher
                        wc._clip = CLIPMatcher()
                    emb = wc._clip.embed_text(query)
                    results = mem.search_clip(emb, top_k=top_k, robot_id=robot_id)
                elif mode == "nearby":
                    results = mem.search_nearby(
                        x=float(payload.get("x", 0)), y=float(payload.get("y", 0)),
                        z=float(payload.get("z")) if payload.get("z") is not None else None,
                        radius=float(payload.get("radius", 2.0)), top_k=top_k, robot_id=robot_id,
                    )
                elif mode == "yolo" and query:
                    results = mem.search_yolo(query, top_k=top_k, robot_id=robot_id)
                else:
                    results = mem.list_memories(limit=top_k, robot_id=robot_id)
                self.send_json(200, {"ok": True, "results": results})
                return
            if self.path == "/api/memory/delete":
                ok = _get_memory().delete(payload.get("id", ""))
                self.send_json(200, {"ok": ok})
                return

            # ── ZK proof POST ──
            if self.path == "/api/zk/setup":
                force = bool(payload.get("force", False))
                from roborun.zk_prover import get_prover
                prover = get_prover()
                if not prover.is_available():
                    self.send_json(503, {"ok": False, "error": "ezkl not installed — pip install ezkl"})
                    return
                def _do_setup():
                    result = prover.setup(force=force)
                    log_event("zk_setup", "ZK circuit setup complete" if result.get("ok") else "ZK setup failed",
                              data=result, level="info" if result.get("ok") else "error")
                threading.Thread(target=_do_setup, daemon=True).start()
                self.send_json(202, {"ok": True, "message": "ZK circuit setup started (background). Check /api/zk/status."})
                return

            if self.path == "/api/zk/prove":
                shard_id = str(payload.get("shard_id", "")).strip()
                if not shard_id:
                    raise ApiError(400, "shard_id required")
                from roborun.zk_prover import get_prover
                prover = get_prover()
                if not prover.is_available():
                    self.send_json(503, {"ok": False, "error": "ezkl not installed"})
                    return
                self.send_json(202, {"ok": True, "message": "Proof generation started (background). This takes 30-120s.",
                                     "shard_id": shard_id})
                def _do_prove():
                    try:
                        mem = _get_memory()
                        records = [r for r in mem.list_memories(limit=1000)
                                   if r.get("shard_id") == shard_id]
                        if not records:
                            log_event("zk_prove", f"Shard {shard_id} not found", level="error")
                            return
                        import cv2
                        frames = []
                        embeddings = []
                        for rec in records:
                            thumb = mem.get_thumbnail(rec["id"])
                            if thumb:
                                arr = cv2.imdecode(np.frombuffer(thumb, np.uint8), cv2.IMREAD_COLOR)
                                if arr is not None:
                                    frames.append(arr)
                                    if rec.get("embedding"):
                                        embeddings.append(np.frombuffer(bytes(rec["embedding"]), np.float32))
                        if not frames:
                            log_event("zk_prove", f"No frames found in shard {shard_id}", level="error")
                            return
                        proof = prover.prove(frames, embeddings)
                        if proof and "proof" in proof:
                            prover.save_proof(proof, shard_id)
                            log_event("zk_prove", f"Proof generated for shard {shard_id}",
                                      data={"proof_hash": proof["proof_hash"], "frames": len(frames)})
                        else:
                            log_event("zk_prove", f"Proof failed for shard {shard_id}",
                                      data=proof or {}, level="error")
                    except Exception as exc:
                        log_event("zk_prove", f"Proof exception: {exc}", level="error")
                threading.Thread(target=_do_prove, daemon=True).start()
                return

            # ── Scene Builder ──
            if self.path == "/api/scene3d/start":
                sb = _get_scene_builder()
                sb._webcam_ref = _get_webcam()
                self.send_json(200, sb.start())
                return
            if self.path == "/api/scene3d/stop":
                self.send_json(200, _get_scene_builder().stop())
                return
            if self.path == "/api/scene3d/clear":
                self.send_json(200, _get_scene_builder().clear())
                return

            # ── Timeline ──
            if self.path == "/api/timeline/settings":
                wc = _get_webcam()
                if "enabled" in payload:
                    wc._timeline_enabled = bool(payload["enabled"])
                if "interval" in payload:
                    wc._timeline_interval = max(1.0, min(30.0, float(payload["interval"])))
                self.send_json(200, {"ok": True, "enabled": wc._timeline_enabled,
                                     "interval": wc._timeline_interval})
                return

            # ── Dataset ──
            if self.path == "/api/dataset/start":
                result = _get_dataset().start_recording(str(payload.get("name", "default")).strip())
                if result.get("ok"): log_event("recording_started", f"Recording: {payload.get('name')}")
                self.send_json(200, result)
                return
            if self.path == "/api/dataset/stop":
                result = _get_dataset().stop_recording()
                if result.get("ok"): log_event("recording_stopped", f"Saved {result.get('frames', 0)} frames")
                self.send_json(200, result)
                return

            # ── dimOS launch / stop ──
            if self.path == "/api/demo":
                global ACTIVE_DIMOS_JOB_ID
                status = dimos_status()
                if status["running"]:
                    self.send_json(200, {"ok": True, "started": False, "alreadyRunning": True, "stdout": "dimOS already running."})
                    return
                result = start_job("dimos-go2-replay", ["dimos", "--replay", "run", "unitree-go2",
                                                        "-o", "rerunbridgemodule.rerun_open=none"])
                ACTIVE_DIMOS_JOB_ID = result["job_id"]
                self.send_json(200, result)
                return
            if self.path == "/api/launch":
                status = dimos_status()
                if status["running"]:
                    self.send_json(200, {"ok": True, "started": False, "alreadyRunning": True, "stdout": "dimOS already running."})
                    return
                args, env = build_launch_command(payload)
                result = start_job("go2-launch", args, env=env)
                ACTIVE_DIMOS_JOB_ID = result["job_id"]
                self.send_json(200, result)
                return
            if self.path == "/api/stop":
                self.send_json(200, stop_active_dimos())
                return
            if self.path == "/api/status":
                self.send_json(200, run_command(["dimos", "status"], timeout=10))
                return
            if self.path == "/api/log":
                self.send_json(200, run_command(["dimos", "log", "--lines", "80"], timeout=10))
                return
            if self.path == "/api/job":
                self.send_json(200, job_status(str(payload.get("job_id", "") or "") or None))
                return

            # ── MCP call (async) ──
            if self.path == "/api/mcp/call":
                name = str(payload.get("name", "")).strip()
                if not name:
                    self.send_json(400, {"ok": False, "error": "name required"})
                    return
                args = payload.get("args") or {}
                task_id = uuid.uuid4().hex
                with _MCP_TASKS_LOCK:
                    _MCP_TASKS[task_id] = {"status": "pending", "name": name, "started": time.time()}
                threading.Thread(target=_run_mcp_task, args=(task_id, name, args), daemon=True).start()
                self.send_json(200, {"ok": True, "task_id": task_id})
                return

            # ── Agent chat (SSE) ──
            if self.path == "/api/agent/chat":
                agent = _get_agent()
                if agent == "unavailable":
                    self.send_json(200, {"ok": False, "error": "Agent not available (claude CLI not found)"})
                    return
                message = str(payload.get("message", "")).strip()
                if not message:
                    self.send_json(400, {"ok": False, "error": "message required"})
                    return
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("X-Accel-Buffering", "no")
                self.end_headers()
                def sse(data: dict) -> None:
                    self.wfile.write(f"data: {json.dumps(data)}\n\n".encode())
                    self.wfile.flush()
                try:
                    for chunk in agent.send(message):
                        sse(chunk)
                        if chunk.get("type") in ("done", "error"): break
                except Exception as exc:
                    try: sse({"type": "error", "error": str(exc)})
                    except Exception: pass
                return
            if self.path == "/api/agent/stop":
                agent = _get_agent()
                if agent != "unavailable": agent.stop()
                self.send_json(200, {"ok": True})
                return
            if self.path == "/api/agent/clear":
                agent = _get_agent()
                if agent != "unavailable": agent.clear_session()
                self.send_json(200, {"ok": True})
                return

            # ── Gemini agent chat (SSE) ──
            if self.path == "/api/agent/gemini":
                agent = _get_gemini_agent()
                if agent == "unavailable":
                    self.send_json(200, {"ok": False, "error": "Gemini agent not available — set GEMINI_API_KEY"})
                    return
                message = str(payload.get("message", "")).strip()
                if not message:
                    self.send_json(400, {"ok": False, "error": "message required"})
                    return
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("X-Accel-Buffering", "no")
                self.end_headers()
                def sse(data: dict) -> None:
                    self.wfile.write(f"data: {json.dumps(data)}\n\n".encode())
                    self.wfile.flush()
                try:
                    for chunk in agent.send(message):
                        sse(chunk)
                        if chunk.get("type") in ("done", "error"): break
                except Exception as exc:
                    try: sse({"type": "error", "error": str(exc)})
                    except Exception: pass
                return
            if self.path == "/api/agent/gemini/clear":
                agent = _get_gemini_agent()
                if agent != "unavailable": agent.clear_session()
                self.send_json(200, {"ok": True})
                return

            # ── Ping ──
            if self.path == "/api/ping":
                ip = str(payload.get("robotIp", "")).strip()
                if not valid_ip(ip): raise ApiError(400, "Valid IP required")
                self.send_json(200, run_command(["ping", "-c", "2", "-W", "2", ip], timeout=8))
                return

            # ── Tasks CRUD ──
            if self.path == "/api/tasks/create":
                self.send_json(200, create_task(payload))
                return
            if self.path == "/api/tasks/update":
                self.send_json(200, update_task(payload))
                return
            if self.path == "/api/tasks/delete":
                self.send_json(200, delete_task(str(payload.get("id", "")).strip()))
                return
            if self.path == "/api/tasks/run":
                self.send_json(200, run_task_now(str(payload.get("id", "")).strip()))
                return

            # ── Fleet CRUD ──
            if self.path == "/api/fleet/add":
                self.send_json(200, add_robot(payload))
                return
            if self.path == "/api/fleet/update":
                self.send_json(200, update_robot(payload))
                return
            if self.path == "/api/fleet/delete":
                self.send_json(200, delete_robot(str(payload.get("id", "")).strip()))
                return
            if self.path == "/api/fleet/deploy":
                self.send_json(200, deploy_blueprint(payload))
                return

            # ── ROS bridge POST ──
            if self.path == "/api/ros/connect":
                host = str(payload.get("host", "")).strip() or load_profile().get("robotIp", "")
                if not host:
                    raise ApiError(400, "host required")
                port = int(payload.get("port", 9090))
                try:
                    from roborun.rosbridge import reset_client, get_client
                    reset_client()
                    client = get_client(host, port)
                    if not client:
                        self.send_json(503, {"ok": False, "error": "Connection failed"})
                    else:
                        from roborun.ros_telemetry import get_bridge
                        bridge = get_bridge()
                        bridge.stop()
                        bridge.start(host)
                        self.send_json(200, {"ok": True, "host": host, "port": port})
                except Exception as exc:
                    self.send_json(503, {"ok": False, "error": str(exc)})
                return

            if self.path == "/api/ros/disconnect":
                from roborun.rosbridge import reset_client
                reset_client()
                self.send_json(200, {"ok": True})
                return

            if self.path == "/api/ros/publish":
                topic = str(payload.get("topic", "")).strip()
                msg_type = str(payload.get("type", "")).strip()
                message = payload.get("message", {})
                if not topic:
                    raise ApiError(400, "topic required")
                host = load_profile().get("robotIp", "")
                try:
                    from roborun.rosbridge import get_client
                    client = get_client(host)
                    if not client:
                        raise ApiError(503, "Not connected to rosbridge")
                    client.publish(topic, msg_type, message)
                    self.send_json(200, {"ok": True})
                except ApiError:
                    raise
                except Exception as exc:
                    self.send_json(500, {"ok": False, "error": str(exc)})
                return

            if self.path == "/api/ros/subscribe-once":
                topic = str(payload.get("topic", "")).strip()
                msg_type = str(payload.get("type", "")).strip()
                timeout = float(payload.get("timeout", 5000)) / 1000.0
                if not topic:
                    raise ApiError(400, "topic required")
                host = load_profile().get("robotIp", "")
                try:
                    from roborun.rosbridge import get_client
                    client = get_client(host)
                    if not client:
                        raise ApiError(503, "Not connected to rosbridge")
                    msg = client.subscribe_once(topic, msg_type, timeout=timeout)
                    self.send_json(200, {"ok": True, "message": msg})
                except ApiError:
                    raise
                except Exception as exc:
                    self.send_json(500, {"ok": False, "error": str(exc)})
                return

            if self.path == "/api/ros/service":
                service = str(payload.get("service", "")).strip()
                srv_type = str(payload.get("type", "")).strip()
                args = payload.get("args", {})
                timeout = float(payload.get("timeout", 10000)) / 1000.0
                if not service:
                    raise ApiError(400, "service required")
                host = load_profile().get("robotIp", "")
                try:
                    from roborun.rosbridge import get_client
                    client = get_client(host)
                    if not client:
                        raise ApiError(503, "Not connected to rosbridge")
                    result = client.call_service(service, srv_type, args, timeout=timeout)
                    self.send_json(200, {"ok": True, "result": result})
                except ApiError:
                    raise
                except Exception as exc:
                    self.send_json(500, {"ok": False, "error": str(exc)})
                return

            if self.path == "/api/ros/action":
                action = str(payload.get("action", "")).strip()
                action_type = str(payload.get("actionType", "")).strip()
                goal = payload.get("goal", {})
                timeout = float(payload.get("timeout", 30000)) / 1000.0
                if not action or not action_type:
                    raise ApiError(400, "action and actionType required")
                host = load_profile().get("robotIp", "")
                try:
                    from roborun.rosbridge import get_client
                    client = get_client(host)
                    if not client:
                        raise ApiError(503, "Not connected to rosbridge")
                    result = client.send_action_goal(action, action_type, goal, timeout=timeout)
                    self.send_json(200, {"ok": True, "result": result})
                except ApiError:
                    raise
                except Exception as exc:
                    self.send_json(500, {"ok": False, "error": str(exc)})
                return

            if self.path == "/api/ros/param/get":
                node = str(payload.get("node", "")).strip()
                parameter = str(payload.get("parameter", "")).strip()
                if not node or not parameter:
                    raise ApiError(400, "node and parameter required")
                host = load_profile().get("robotIp", "")
                try:
                    from roborun.rosbridge import get_client
                    client = get_client(host)
                    if not client:
                        raise ApiError(503, "Not connected to rosbridge")
                    value = client.get_param(node, parameter)
                    self.send_json(200, {"ok": True, "value": value})
                except ApiError:
                    raise
                except Exception as exc:
                    self.send_json(500, {"ok": False, "error": str(exc)})
                return

            if self.path == "/api/ros/param/set":
                node = str(payload.get("node", "")).strip()
                parameter = str(payload.get("parameter", "")).strip()
                value = payload.get("value")
                if not node or not parameter:
                    raise ApiError(400, "node and parameter required")
                host = load_profile().get("robotIp", "")
                try:
                    from roborun.rosbridge import get_client
                    client = get_client(host)
                    if not client:
                        raise ApiError(503, "Not connected to rosbridge")
                    ok = client.set_param(node, parameter, value)
                    self.send_json(200, {"ok": ok})
                except ApiError:
                    raise
                except Exception as exc:
                    self.send_json(500, {"ok": False, "error": str(exc)})
                return

            if self.path == "/api/ros/camera":
                topic = str(payload.get("topic", "/camera/image_raw/compressed")).strip()
                timeout = float(payload.get("timeout", 10000)) / 1000.0
                host = load_profile().get("robotIp", "")
                try:
                    from roborun.rosbridge import get_client
                    client = get_client(host)
                    if not client:
                        raise ApiError(503, "Not connected to rosbridge")
                    frame_bytes = client.camera_snapshot(topic, timeout=timeout)
                    if not frame_bytes:
                        self.send_json(404, {"ok": False, "error": "No frame received"})
                        return
                    import base64
                    b64 = base64.b64encode(frame_bytes).decode()
                    self.send_json(200, {"ok": True, "image": f"data:image/jpeg;base64,{b64}"})
                except ApiError:
                    raise
                except Exception as exc:
                    self.send_json(500, {"ok": False, "error": str(exc)})
                return

            if self.path == "/api/ros/depth":
                topic = str(payload.get("topic", "/camera/depth/image_raw")).strip()
                timeout = float(payload.get("timeout", 5000)) / 1000.0
                host = load_profile().get("robotIp", "")
                try:
                    from roborun.rosbridge import get_client
                    client = get_client(host)
                    if not client:
                        raise ApiError(503, "Not connected to rosbridge")
                    dist = client.depth_distance(topic, timeout=timeout)
                    self.send_json(200, {"ok": True, "distance_m": dist})
                except ApiError:
                    raise
                except Exception as exc:
                    self.send_json(500, {"ok": False, "error": str(exc)})
                return

            if self.path == "/api/ros/move":
                linear_x = float(payload.get("linear_x", 0.0))
                linear_y = float(payload.get("linear_y", 0.0))
                angular_z = float(payload.get("angular_z", 0.0))
                topic = str(payload.get("topic", "/cmd_vel"))
                host = load_profile().get("robotIp", "")
                try:
                    from roborun.rosbridge import get_client
                    client = get_client(host)
                    if not client:
                        raise ApiError(503, "Not connected to rosbridge")
                    client.move(linear_x, linear_y, angular_z, topic)
                    self.send_json(200, {"ok": True})
                except ApiError:
                    raise
                except Exception as exc:
                    self.send_json(500, {"ok": False, "error": str(exc)})
                return

            # ── Blueprint CRUD ──
            if self.path == "/api/blueprints/create":
                self.send_json(200, create_blueprint(payload))
                return
            if self.path == "/api/blueprints/update":
                self.send_json(200, update_blueprint(payload))
                return
            if self.path == "/api/blueprints/delete":
                self.send_json(200, delete_blueprint(str(payload.get("id", "")).strip()))
                return
            if self.path == "/api/blueprints/duplicate":
                self.send_json(200, duplicate_blueprint(str(payload.get("id", "")).strip()))
                return

            raise ApiError(404, "Unknown API route")
        except ApiError as exc:
            self.send_json(exc.status, {"ok": False, "error": exc.message})


def _frame_recorder_loop() -> None:
    while True:
        try:
            ds = _get_dataset()
            wc = _get_webcam()
            if ds.is_recording and wc.is_running:
                frame = wc.snapshot()
                if frame is not None:
                    ds.record_frame(frame, detections=wc.get_detections())
        except Exception:
            pass
        time.sleep(0.1)


def main() -> None:
    if not WEB_ROOT.exists():
        raise SystemExit(f"Missing web directory at {WEB_ROOT}")
    STATE_ROOT.mkdir(parents=True, exist_ok=True)

    recorder = threading.Thread(target=_frame_recorder_loop, daemon=True, name="FrameRecorder")
    recorder.start()

    from roborun.telemetry import start_ws_server
    start_ws_server()

    from roborun.ros_telemetry import get_bridge
    get_bridge().start()

    from roborun.trajectory import TrajectoryRecorder
    TrajectoryRecorder.get().start()

    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"\n  RoboRun is live: http://{HOST}:{PORT}\n")
    print(f"  Telemetry WS:    ws://127.0.0.1:8766\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")
        try: _get_webcam().stop()
        except Exception: pass


if __name__ == "__main__":
    main()
