"""Patrol skill — autonomous waypoint loop with configurable dwell time.

The robot visits a list of waypoints in sequence, pausing at each one
for a configurable duration. Useful for security, monitoring, or
inspection tasks. Waypoints can be added dynamically via the
patrol_add_waypoint tool.
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
from pathlib import Path

SKILL_ID = "patrol"
SKILL_NAME = "Patrol"
SKILL_VERSION = "1.0.0"

log = logging.getLogger(__name__)

_active = False
_thread: threading.Thread | None = None
_waypoints: list[dict] = []
_current_idx = 0
_dwell_s = float(os.environ.get("ROBORUN_PATROL_DWELL", "5.0"))
_speed = float(os.environ.get("ROBORUN_PATROL_SPEED", "0.3"))
_state_file = Path.cwd() / ".roborun" / "patrol_waypoints.json"


def _call_api(path: str, payload: dict) -> dict:
    import urllib.request
    port = int(os.environ.get("ROBORUN_PORT", "8765"))
    url = f"http://127.0.0.1:{port}{path}"
    body = json.dumps(payload).encode()
    req = urllib.request.Request(url, data=body,
                                headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            return json.loads(resp.read().decode())
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _load_waypoints() -> list[dict]:
    if _state_file.exists():
        try:
            return json.loads(_state_file.read_text())
        except Exception:
            pass
    return []


def _save_waypoints() -> None:
    _state_file.parent.mkdir(parents=True, exist_ok=True)
    _state_file.write_text(json.dumps(_waypoints, indent=2))


def _patrol_loop() -> None:
    global _active, _current_idx
    while _active and _waypoints:
        wp = _waypoints[_current_idx % len(_waypoints)]
        log.info("Patrol: heading to waypoint %d (%s)", _current_idx, wp.get("name", "unnamed"))

        result = _call_api("/api/ros/move", {
            "linear_x": _speed,
            "angular_z": wp.get("angular_z", 0.0),
        })
        time.sleep(wp.get("travel_s", 3.0))
        _call_api("/api/ros/move", {"linear_x": 0, "angular_z": 0})

        dwell = wp.get("dwell_s", _dwell_s)
        log.info("Patrol: dwelling at waypoint %d for %.1fs", _current_idx, dwell)
        time.sleep(dwell)

        _current_idx = (_current_idx + 1) % len(_waypoints)

    _call_api("/api/ros/move", {"linear_x": 0, "angular_z": 0})
    _active = False


def _start(args: dict) -> dict:
    global _active, _thread, _waypoints
    if _active:
        return {"ok": True, "status": "already_patrolling"}
    _waypoints = _load_waypoints()
    if not _waypoints:
        return {"ok": False, "error": "No waypoints defined. Use patrol_add_waypoint first."}
    _active = True
    _thread = threading.Thread(target=_patrol_loop, daemon=True)
    _thread.start()
    return {"ok": True, "status": "patrolling", "waypoints": len(_waypoints)}


def _stop(args: dict) -> dict:
    global _active
    _active = False
    _call_api("/api/ros/move", {"linear_x": 0, "angular_z": 0})
    return {"ok": True, "status": "stopped"}


def _add_waypoint(args: dict) -> dict:
    global _waypoints
    wp = {
        "name": args.get("name", f"wp-{len(_waypoints)}"),
        "angular_z": float(args.get("angular_z", 0.0)),
        "travel_s": float(args.get("travel_s", 3.0)),
        "dwell_s": float(args.get("dwell_s", _dwell_s)),
    }
    _waypoints.append(wp)
    _save_waypoints()
    return {"ok": True, "waypoint": wp, "total": len(_waypoints)}


def _clear_waypoints(args: dict) -> dict:
    global _waypoints
    _waypoints = []
    _save_waypoints()
    return {"ok": True, "cleared": True}


def _status(args: dict) -> dict:
    return {
        "ok": True,
        "active": _active,
        "current_waypoint": _current_idx,
        "total_waypoints": len(_waypoints),
        "waypoints": _waypoints,
    }


def register(registry) -> None:
    registry.add_tool(
        name="patrol_start",
        description="Start patrolling through saved waypoints in a loop. Add waypoints first with patrol_add_waypoint.",
        input_schema={"type": "object", "properties": {}},
        handler=_start,
        skill_id=SKILL_ID,
    )
    registry.add_tool(
        name="patrol_stop",
        description="Stop the patrol loop.",
        input_schema={"type": "object", "properties": {}},
        handler=_stop,
        skill_id=SKILL_ID,
    )
    registry.add_tool(
        name="patrol_add_waypoint",
        description="Add a waypoint to the patrol route. Specify travel time and dwell time.",
        input_schema={
            "type": "object",
            "properties": {
                "name": {"type": "string", "description": "Waypoint name"},
                "angular_z": {"type": "number", "description": "Turn angle to face this waypoint (rad/s briefly)"},
                "travel_s": {"type": "number", "description": "Seconds to drive forward toward this waypoint"},
                "dwell_s": {"type": "number", "description": "Seconds to pause at this waypoint"},
            },
        },
        handler=_add_waypoint,
        skill_id=SKILL_ID,
    )
    registry.add_tool(
        name="patrol_clear_waypoints",
        description="Clear all patrol waypoints.",
        input_schema={"type": "object", "properties": {}},
        handler=_clear_waypoints,
        skill_id=SKILL_ID,
    )
    registry.add_tool(
        name="patrol_status",
        description="Get current patrol status and waypoint list.",
        input_schema={"type": "object", "properties": {}},
        handler=_status,
        skill_id=SKILL_ID,
    )
    registry.add_behavior(
        name="patrol",
        description="Autonomous waypoint patrol loop",
        handler=_start,
        skill_id=SKILL_ID,
    )
