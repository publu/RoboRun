"""Scan & Detect skill — YOLO-based autonomous object scanning.

Rotates the robot in increments, checking YOLO detections at each step.
Reports all objects found with their positions relative to the starting
orientation. Can search for a specific label or do a full-sweep inventory.
"""
from __future__ import annotations

import json
import logging
import os
import time
from pathlib import Path

SKILL_ID = "scan-detect"
SKILL_NAME = "Scan & Detect"
SKILL_VERSION = "1.0.0"

log = logging.getLogger(__name__)

_MAX_ANGULAR = float(os.environ.get("ROBORUN_MAX_ANGULAR_VEL", "1.5"))
_STEP_DEG = float(os.environ.get("ROBORUN_SCAN_STEP_DEG", "30"))
_FRAME_PATHS = [
    Path("/tmp/roborun_state.json"),
]


def _call_api(path: str, payload: dict) -> dict:
    import urllib.request
    port = int(os.environ.get("ROBORUN_PORT", "8765"))
    url = f"http://127.0.0.1:{port}{path}"
    body = json.dumps(payload).encode()
    req = urllib.request.Request(url, data=body,
                                headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return json.loads(resp.read().decode())
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _get_detections(min_confidence: float = 0.3) -> list[dict]:
    for p in _FRAME_PATHS:
        try:
            if p.exists() and (time.time() - p.stat().st_mtime) < 3.0:
                state = json.loads(p.read_text())
                return [d for d in state.get("detections", [])
                        if d.get("confidence", 0) >= min_confidence]
        except Exception:
            pass
    return []


def _rotate_step(degrees: float) -> None:
    import math
    rad = math.radians(degrees)
    angular_z = min(_MAX_ANGULAR, max(-_MAX_ANGULAR, rad / 1.0))
    _call_api("/api/ros/move", {"linear_x": 0, "angular_z": angular_z})
    time.sleep(1.0)
    _call_api("/api/ros/move", {"linear_x": 0, "angular_z": 0})
    time.sleep(0.5)


def _find_object(args: dict) -> dict:
    label = str(args.get("label", "")).strip().lower()
    max_rotation = float(args.get("max_rotation_deg", 360))
    confidence = float(args.get("min_confidence", 0.4))

    if not label:
        return {"ok": False, "error": "label required"}

    rotated = 0.0
    while rotated < max_rotation:
        dets = _get_detections(confidence)
        matches = [d for d in dets if d.get("label", "").lower() == label]
        if matches:
            best = max(matches, key=lambda d: d.get("confidence", 0))
            return {
                "ok": True,
                "found": True,
                "label": label,
                "confidence": best.get("confidence", 0),
                "bbox": best.get("bbox"),
                "rotation_deg": rotated,
            }
        _rotate_step(_STEP_DEG)
        rotated += _STEP_DEG

    return {"ok": True, "found": False, "label": label,
            "searched_deg": rotated}


def _scan_all(args: dict) -> dict:
    max_rotation = float(args.get("max_rotation_deg", 360))
    confidence = float(args.get("min_confidence", 0.3))

    all_objects: dict[str, dict] = {}
    rotated = 0.0

    while rotated < max_rotation:
        dets = _get_detections(confidence)
        for d in dets:
            lbl = d.get("label", "unknown")
            conf = d.get("confidence", 0)
            if lbl not in all_objects or conf > all_objects[lbl]["confidence"]:
                all_objects[lbl] = {
                    "label": lbl,
                    "confidence": conf,
                    "bbox": d.get("bbox"),
                    "found_at_deg": rotated,
                }
        _rotate_step(_STEP_DEG)
        rotated += _STEP_DEG

    return {
        "ok": True,
        "objects": list(all_objects.values()),
        "total_unique": len(all_objects),
        "scanned_deg": rotated,
    }


def _detect_now(args: dict) -> dict:
    confidence = float(args.get("min_confidence", 0.3))
    label_filter = str(args.get("label", "")).strip().lower()
    dets = _get_detections(confidence)
    if label_filter:
        dets = [d for d in dets if d.get("label", "").lower() == label_filter]
    return {
        "ok": True,
        "detections": dets,
        "count": len(dets),
    }


def register(registry) -> None:
    registry.add_tool(
        name="find_object",
        description="Search for a specific object by rotating and checking YOLO detections. Returns position when found.",
        input_schema={
            "type": "object",
            "properties": {
                "label": {"type": "string", "description": "YOLO class label to find (e.g. 'person', 'chair', 'cup')"},
                "max_rotation_deg": {"type": "number", "description": "Max degrees to rotate (default 360)"},
                "min_confidence": {"type": "number", "description": "Min detection confidence (default 0.4)"},
            },
            "required": ["label"],
        },
        handler=_find_object,
        skill_id=SKILL_ID,
    )
    registry.add_tool(
        name="scan_surroundings",
        description="Do a full 360-degree scan and inventory all visible objects using YOLO detection.",
        input_schema={
            "type": "object",
            "properties": {
                "max_rotation_deg": {"type": "number", "description": "Degrees to scan (default 360)"},
                "min_confidence": {"type": "number", "description": "Min detection confidence (default 0.3)"},
            },
        },
        handler=_scan_all,
        skill_id=SKILL_ID,
    )
    registry.add_tool(
        name="detect_now",
        description="Get current YOLO detections without rotating. Optionally filter by label.",
        input_schema={
            "type": "object",
            "properties": {
                "label": {"type": "string", "description": "Filter to this label only"},
                "min_confidence": {"type": "number", "description": "Min confidence (default 0.3)"},
            },
        },
        handler=_detect_now,
        skill_id=SKILL_ID,
    )
    registry.add_behavior(
        name="find_object",
        description="Autonomous rotate-and-scan to find a specific object",
        handler=_find_object,
        skill_id=SKILL_ID,
    )
    registry.add_behavior(
        name="scan_surroundings",
        description="Full environment scan with object inventory",
        handler=_scan_all,
        skill_id=SKILL_ID,
    )
