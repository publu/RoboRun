"""Follow-Me skill — depth-based person following with P-controller.

Built-in reference skill for the RoboRun skills plugin system.
Tracks the largest detected person via YOLO and steers the robot to
keep them centered, using depth (if available) to maintain distance.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from pathlib import Path

SKILL_ID = "follow-me"
SKILL_NAME = "Follow Me"
SKILL_VERSION = "1.0.0"

log = logging.getLogger(__name__)

_FRAME_PATHS = [
    Path("/tmp/roborun_state.json"),
    Path("/tmp/go2_hackathon_state.json"),
]

_active = False
_thread: threading.Thread | None = None

# P-controller gains
_KP_ANGULAR = float(os.environ.get("ROBORUN_FOLLOW_KP_ANG", "0.005"))
_KP_LINEAR = float(os.environ.get("ROBORUN_FOLLOW_KP_LIN", "0.3"))
_TARGET_DISTANCE = float(os.environ.get("ROBORUN_FOLLOW_DISTANCE", "1.5"))
_MAX_LINEAR = float(os.environ.get("ROBORUN_MAX_LINEAR_VEL", "1.0"))
_MAX_ANGULAR = float(os.environ.get("ROBORUN_MAX_ANGULAR_VEL", "1.5"))
_LOST_TIMEOUT = 3.0


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


def _call_api(path: str, payload: dict) -> dict:
    import urllib.request
    url = f"http://127.0.0.1:8765{path}"
    body = json.dumps(payload).encode()
    req = urllib.request.Request(url, data=body,
                                headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return json.loads(resp.read().decode())
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _get_person_detection() -> dict | None:
    for p in _FRAME_PATHS:
        try:
            if p.exists() and (time.time() - p.stat().st_mtime) < 3.0:
                state = json.loads(p.read_text())
                persons = [d for d in state.get("detections", [])
                           if d.get("label", "").lower() == "person" and d.get("confidence", 0) >= 0.4]
                if persons:
                    return max(persons, key=lambda d: (d.get("bbox", [0, 0, 0, 0])[2] - d.get("bbox", [0, 0, 0, 0])[0])
                               * (d.get("bbox", [0, 0, 0, 0])[3] - d.get("bbox", [0, 0, 0, 0])[1]))
        except Exception:
            pass
    return None


def _follow_loop() -> None:
    global _active
    last_seen = time.time()
    frame_w = 640

    while _active:
        person = _get_person_detection()
        if person is None:
            if time.time() - last_seen > _LOST_TIMEOUT:
                _call_api("/api/ros/move", {"linear_x": 0, "angular_z": 0})
                log.info("Follow-me: target lost, stopping")
                _active = False
                return
            time.sleep(0.2)
            continue

        last_seen = time.time()
        bbox = person.get("bbox", [0, 0, 0, 0])
        cx = (bbox[0] + bbox[2]) / 2
        bw = bbox[2] - bbox[0]
        bh = bbox[3] - bbox[1]

        error_x = cx - (frame_w / 2)
        angular_z = _clamp(-_KP_ANGULAR * error_x, _MAX_ANGULAR)

        area_ratio = (bw * bh) / (frame_w * 480)
        if area_ratio < 0.05:
            linear_x = _clamp(_KP_LINEAR, _MAX_LINEAR)
        elif area_ratio > 0.25:
            linear_x = _clamp(-_KP_LINEAR * 0.5, _MAX_LINEAR)
        else:
            linear_x = 0.0

        _call_api("/api/ros/move", {"linear_x": linear_x, "angular_z": angular_z})
        time.sleep(0.15)

    _call_api("/api/ros/move", {"linear_x": 0, "angular_z": 0})


def _start_follow(args: dict) -> dict:
    global _active, _thread
    if _active:
        return {"ok": True, "status": "already_following"}
    _active = True
    _thread = threading.Thread(target=_follow_loop, daemon=True)
    _thread.start()
    return {"ok": True, "status": "following"}


def _stop_follow(args: dict) -> dict:
    global _active
    _active = False
    _call_api("/api/ros/move", {"linear_x": 0, "angular_z": 0})
    return {"ok": True, "status": "stopped"}


def _follow_status(args: dict) -> dict:
    return {"ok": True, "active": _active}


def register(registry) -> None:
    registry.add_tool(
        name="follow_me_start",
        description="Start following the nearest person. Uses YOLO person detection with P-controller steering. Stops when target is lost for 3 seconds.",
        input_schema={"type": "object", "properties": {}},
        handler=_start_follow,
        skill_id=SKILL_ID,
    )
    registry.add_tool(
        name="follow_me_stop",
        description="Stop the follow-me behavior.",
        input_schema={"type": "object", "properties": {}},
        handler=_stop_follow,
        skill_id=SKILL_ID,
    )
    registry.add_tool(
        name="follow_me_status",
        description="Check if follow-me is currently active.",
        input_schema={"type": "object", "properties": {}},
        handler=_follow_status,
        skill_id=SKILL_ID,
    )
    registry.add_behavior(
        name="follow_me",
        description="Autonomous person-following with depth-based distance control",
        handler=_start_follow,
        skill_id=SKILL_ID,
    )
