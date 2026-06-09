"""Fleet and blueprint CRUD routes."""
from __future__ import annotations

import json
import re
import threading
import time
import uuid
from pathlib import Path

from roborun.routes import get, post, send_json, ApiError

ROOT = Path(__file__).resolve().parent.parent.parent
STATE_ROOT = ROOT / ".roborun"
FLEET_PATH = STATE_ROOT / "fleet.json"
BLUEPRINTS_PATH = STATE_ROOT / "blueprints.json"

_fleet_lock = threading.Lock()
_blueprints_lock = threading.Lock()


# ── Fleet ────────────────────────────────────────────────────────────────────

def _load_fleet() -> list[dict]:
    if FLEET_PATH.exists():
        try:
            data = json.loads(FLEET_PATH.read_text())
            if isinstance(data, list):
                return data
        except Exception:
            pass
    return []


def _save_fleet(fleet: list[dict]) -> None:
    STATE_ROOT.mkdir(parents=True, exist_ok=True)
    FLEET_PATH.write_text(json.dumps(fleet, indent=2))


@get("/api/fleet")
def list_fleet(h):
    with _fleet_lock:
        robots = _load_fleet()
    groups: dict[str, int] = {}
    online = 0
    for r in robots:
        g = r.get("group", "Default")
        groups[g] = groups.get(g, 0) + 1
        if r.get("status") == "online":
            online += 1
    send_json(h, 200, {"ok": True, "robots": robots, "total": len(robots),
                        "online": online, "groups": groups})


@post("/api/fleet/add")
def add_robot(h, payload):
    name = str(payload.get("name", "")).strip()
    if not name:
        raise ApiError(400, "Robot name required")
    robot = {
        "id": str(uuid.uuid4()), "name": name,
        "deviceType": str(payload.get("deviceType", "Robot")).strip(),
        "serial": str(payload.get("serial", "")).strip(),
        "robotIp": str(payload.get("robotIp", "")).strip(),
        "group": str(payload.get("group", "Default")).strip() or "Default",
        "blueprint": str(payload.get("blueprint", "generic-robot")).strip(),
        "status": "offline", "tags": payload.get("tags", []),
        "notes": str(payload.get("notes", "")).strip(),
        "createdAt": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    with _fleet_lock:
        fleet = _load_fleet()
        fleet.append(robot)
        _save_fleet(fleet)
    send_json(h, 200, {"ok": True, "robot": robot})


@post("/api/fleet/update")
def update_robot(h, payload):
    rid = str(payload.get("id", "")).strip()
    if not rid:
        raise ApiError(400, "Robot id required")
    with _fleet_lock:
        fleet = _load_fleet()
        for r in fleet:
            if r.get("id") == rid:
                for f in ("name", "deviceType", "serial", "robotIp", "group", "blueprint", "notes"):
                    if f in payload:
                        r[f] = str(payload[f]).strip()
                if "tags" in payload and isinstance(payload["tags"], list):
                    r["tags"] = payload["tags"]
                if "status" in payload:
                    r["status"] = payload["status"]
                _save_fleet(fleet)
                send_json(h, 200, {"ok": True, "robot": r})
                return
    raise ApiError(404, "Robot not found")


@post("/api/fleet/delete")
def delete_robot(h, payload):
    rid = str(payload.get("id", "")).strip()
    with _fleet_lock:
        fleet = _load_fleet()
        fleet = [r for r in fleet if r.get("id") != rid]
        _save_fleet(fleet)
    send_json(h, 200, {"ok": True, "deleted": rid})


@post("/api/fleet/deploy")
def deploy_blueprint(h, payload):
    rid = str(payload.get("robotId", "")).strip()
    bp = str(payload.get("blueprint", "")).strip()
    if not rid or not bp:
        raise ApiError(400, "robotId and blueprint required")
    with _fleet_lock:
        fleet = _load_fleet()
        robot = next((r for r in fleet if r.get("id") == rid), None)
        if not robot:
            raise ApiError(404, "Robot not found")
        robot["blueprint"] = bp
        _save_fleet(fleet)
    send_json(h, 200, {"ok": True, "robot": robot})


# ── Blueprints ───────────────────────────────────────────────────────────────

SEED_BLUEPRINTS: list[dict] = [
    {"id": "bp-generic", "slug": "generic-robot", "name": "Generic Robot",
     "description": "Any ROS 2 robot — rosbridge or DDS. Full MCP tools, skills, and fleet support.",
     "base": "generic-robot", "modules": ["navigation", "mapping"],
     "extraArgs": "", "tags": ["standard"], "icon": "◈", "color": "#00d47e", "builtIn": True},
    {"id": "bp-basic", "slug": "basic", "name": "Basic",
     "description": "Minimal connection only — topic pub/sub and service calls.",
     "base": "basic", "modules": [], "extraArgs": "", "tags": ["minimal"],
     "icon": "◻", "color": "#7aaf90", "builtIn": True},
    {"id": "bp-agentic", "slug": "agentic-claude", "name": "Agentic (Claude)",
     "description": "Full AI agent via Claude with vision, memory, and autonomous behaviors. Needs ANTHROPIC_API_KEY.",
     "base": "agentic-claude", "modules": ["claude-agent", "navigation", "vision"],
     "extraArgs": "", "tags": ["agentic"], "icon": "⬡", "color": "#d4a030", "builtIn": True},
    {"id": "bp-spatial", "slug": "spatial-memory", "name": "Spatial Memory",
     "description": "Persistent spatial memory — recall and return to places. CLIP + YOLO.",
     "base": "spatial-memory", "modules": ["spatial-memory", "navigation"],
     "extraArgs": "", "tags": ["spatial"], "icon": "◎", "color": "#a060f0", "builtIn": True},
    {"id": "bp-patrol", "slug": "security-patrol", "name": "Security Patrol",
     "description": "Automated waypoint patrol loop with object detection.",
     "base": "security-patrol", "modules": ["patrol", "yolo"],
     "extraArgs": "", "tags": ["security"], "icon": "◉", "color": "#e04040", "builtIn": True},
    {"id": "bp-turtlebot", "slug": "turtlebot3", "name": "TurtleBot3",
     "description": "TurtleBot3 with navigation, mapping, and SLAM.",
     "base": "turtlebot3", "modules": ["navigation", "mapping"],
     "extraArgs": "", "tags": ["turtlebot"], "icon": "◈", "color": "#4090e0", "builtIn": True},
    {"id": "bp-go2", "slug": "unitree-go2", "name": "Unitree Go2",
     "description": "Unitree Go2 quadruped with sport commands and navigation.",
     "base": "unitree-go2", "modules": ["navigation", "sport-commands"],
     "extraArgs": "", "tags": ["quadruped"], "icon": "★", "color": "#00d47e", "builtIn": True},
    {"id": "bp-drone", "slug": "generic-drone", "name": "Quadrotor",
     "description": "Generic drone with waypoint navigation and altitude hold.",
     "base": "generic-drone", "modules": ["waypoint-nav", "altitude-hold"],
     "extraArgs": "", "tags": ["drone"], "icon": "✈", "color": "#40a0e0",
     "robotType": "drone", "builtIn": True},
    {"id": "bp-g1", "slug": "unitree-g1", "name": "Unitree G1",
     "description": "G1 humanoid with full joint control and walking policy.",
     "base": "unitree-g1", "modules": ["navigation", "walking-policy"],
     "extraArgs": "", "tags": ["humanoid"], "icon": "⬡", "color": "#d4a030",
     "robotType": "humanoid", "builtIn": True},
    {"id": "bp-webcam", "slug": "webcam-only", "name": "Webcam Only",
     "description": "Standalone webcam with vision AI — no robot required.",
     "base": "webcam-only", "modules": ["yolo", "clip"],
     "extraArgs": "", "tags": ["webcam"], "icon": "◉", "color": "#a0a0a0",
     "robotType": "webcam_only", "builtIn": True},
]


def _load_blueprints() -> list[dict]:
    if BLUEPRINTS_PATH.exists():
        try:
            data = json.loads(BLUEPRINTS_PATH.read_text())
            if isinstance(data, list):
                return data
        except Exception:
            pass
    return list(SEED_BLUEPRINTS)


def _save_blueprints(bps: list[dict]) -> None:
    STATE_ROOT.mkdir(parents=True, exist_ok=True)
    BLUEPRINTS_PATH.write_text(json.dumps(bps, indent=2))


@get("/api/blueprints")
def list_blueprints(h):
    with _blueprints_lock:
        bps = _load_blueprints()
        if not bps:
            bps = list(SEED_BLUEPRINTS)
            _save_blueprints(bps)
    send_json(h, 200, {"ok": True, "blueprints": bps, "total": len(bps)})


@post("/api/blueprints/create")
def create_blueprint(h, payload):
    name = str(payload.get("name", "")).strip()
    if not name:
        raise ApiError(400, "Blueprint name required")
    slug = str(payload.get("slug", "")).strip() or re.sub(r"[^a-z0-9_-]", "", name.lower().replace(" ", "-"))
    bp = {
        "id": f"bp-{uuid.uuid4().hex[:8]}", "slug": slug, "name": name,
        "description": str(payload.get("description", "")).strip(),
        "base": str(payload.get("base", "unitree-go2")).strip(),
        "modules": payload.get("modules", []),
        "extraArgs": str(payload.get("extraArgs", "")).strip(),
        "tags": payload.get("tags", []),
        "icon": str(payload.get("icon", "◈")).strip(),
        "color": str(payload.get("color", "#4090e0")).strip(),
        "builtIn": False,
    }
    with _blueprints_lock:
        bps = _load_blueprints()
        if any(b.get("slug") == slug for b in bps):
            raise ApiError(409, f"Slug '{slug}' exists")
        bps.append(bp)
        _save_blueprints(bps)
    send_json(h, 200, {"ok": True, "blueprint": bp})


@post("/api/blueprints/update")
def update_blueprint(h, payload):
    bp_id = str(payload.get("id", "")).strip()
    if not bp_id:
        raise ApiError(400, "Blueprint id required")
    with _blueprints_lock:
        bps = _load_blueprints()
        for bp in bps:
            if bp.get("id") == bp_id:
                for f in ("name", "slug", "description", "base", "extraArgs", "icon", "color"):
                    if f in payload and not (bp.get("builtIn") and f in ("name", "slug", "base")):
                        bp[f] = str(payload[f]).strip()
                if "modules" in payload and not bp.get("builtIn"):
                    bp["modules"] = payload["modules"]
                if "tags" in payload:
                    bp["tags"] = payload["tags"]
                _save_blueprints(bps)
                send_json(h, 200, {"ok": True, "blueprint": bp})
                return
    raise ApiError(404, "Blueprint not found")


@post("/api/blueprints/delete")
def delete_blueprint(h, payload):
    bp_id = str(payload.get("id", "")).strip()
    with _blueprints_lock:
        bps = _load_blueprints()
        target = next((b for b in bps if b.get("id") == bp_id), None)
        if not target:
            raise ApiError(404, "Blueprint not found")
        if target.get("builtIn"):
            raise ApiError(403, "Cannot delete built-in blueprint")
        bps = [b for b in bps if b.get("id") != bp_id]
        _save_blueprints(bps)
    send_json(h, 200, {"ok": True, "deleted": bp_id})


@post("/api/blueprints/duplicate")
def duplicate_blueprint(h, payload):
    bp_id = str(payload.get("id", "")).strip()
    with _blueprints_lock:
        bps = _load_blueprints()
        source = next((b for b in bps if b.get("id") == bp_id), None)
        if not source:
            raise ApiError(404, "Blueprint not found")
        new = dict(source)
        new["id"] = f"bp-{uuid.uuid4().hex[:8]}"
        new["name"] = f"{source['name']} (Copy)"
        new["slug"] = f"{source['slug']}-copy-{uuid.uuid4().hex[:4]}"
        new["builtIn"] = False
        bps.append(new)
        _save_blueprints(bps)
    send_json(h, 200, {"ok": True, "blueprint": new})
