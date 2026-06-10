"""Dashboard, health, camera, system routes."""
from __future__ import annotations

import base64
import json
import os
import shutil
import time
from pathlib import Path

from roborun.routes import get, post, send_json, ApiError

ROOT = Path(__file__).resolve().parent.parent.parent
STATE_ROOT = ROOT / ".roborun"
PROFILE_PATH = STATE_ROOT / "profile.json"

_FRAME_PATHS = [
    Path("/tmp/roborun_frame.jpg"),
    Path("/tmp/roborun_camera.jpg"),
]
_STATE_PATHS = [
    Path("/tmp/roborun_state.json"),
]


def load_profile() -> dict:
    defaults = {
        "deviceName": "ros-agent Station", "deviceType": "Robot",
        "serial": "", "group": "Robots", "robotIp": "",
        "blueprint": "generic-robot", "mode": "hardware", "viewer": "rerun",
        "daemon": True, "cameraIndex": 0,
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


def save_profile(payload: dict) -> dict:
    profile = load_profile()
    for key in ["deviceName", "deviceType", "serial", "group", "robotIp", "blueprint",
                "mode", "viewer", "daemon", "cameraIndex", "activeModels"]:
        if key in payload:
            profile[key] = payload[key]
    STATE_ROOT.mkdir(parents=True, exist_ok=True)
    PROFILE_PATH.write_text(json.dumps(profile, indent=2), encoding="utf-8")
    return profile


def camera_snapshot() -> dict:
    for path in _FRAME_PATHS:
        if path.exists() and (time.time() - path.stat().st_mtime) < 5.0:
            try:
                data = base64.b64encode(path.read_bytes()).decode()
                return {"ok": True, "image": f"data:image/jpeg;base64,{data}", "ts": path.stat().st_mtime}
            except Exception as exc:
                return {"ok": False, "error": str(exc)}
    return {"ok": False, "error": "No frame available — start webcam or connect a robot camera"}


def system_stats() -> dict:
    disk = shutil.disk_usage(ROOT)
    load_avg = os.getloadavg()
    return {
        "load": [round(v, 2) for v in load_avg],
        "disk": {"percent": round((disk.used / disk.total) * 100, 1),
                 "usedBytes": disk.used, "totalBytes": disk.total},
    }


@get("/api/health")
def health(h):
    send_json(h, 200, {"ok": True, "name": "ros-agent"})


@get("/api/camera")
def camera(h):
    send_json(h, 200, camera_snapshot())


@get("/api/scene")
def scene(h):
    for p in _STATE_PATHS:
        if p.exists():
            try:
                send_json(h, 200, json.loads(p.read_text()))
                return
            except Exception:
                pass
    send_json(h, 200, {})


@get("/api/state")
def robot_state(h):
    for p in _STATE_PATHS:
        try:
            if p.exists():
                send_json(h, 200, json.loads(p.read_text()))
                return
        except Exception:
            pass
    send_json(h, 200, {})


@post("/api/profile")
def profile_update(h, payload):
    send_json(h, 200, {"ok": True, "profile": save_profile(payload)})


@get("/api/dashboard")
def dashboard(h):
    from roborun.routes._singletons import get_webcam, get_simulator, get_memory
    profile = load_profile()
    webcam = get_webcam()
    try:
        sim = get_simulator()
        sim_state = sim.get_state()
    except Exception:
        sim_state = {"running": False, "state": "idle", "robot": "", "robot_type": "",
                     "fps": 0, "frame_count": 0, "sim_time": 0,
                     "position": {"x": 0, "y": 0, "z": 0},
                     "orientation": {"w": 1, "x": 0, "y": 0, "z": 0},
                     "has_policy": False, "has_drone_ctrl": False}

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

    send_json(h, 200, {
        "ok": True,
        "profile": profile,
        "stack": {"ok": False, "running": False, "label": "Not installed"},
        "robotOnline": False,
        "robotIp": profile.get("robotIp", "").strip(),
        "commandCenter": {"ok": False, "url": "http://127.0.0.1:7779/command-center"},
        "webcam": webcam.get_state(),
        "sim": sim_state,
        "stats": system_stats(),
        "robotType": robot_profile,
        "telemetryWs": "ws://127.0.0.1:8766",
        "ros": {"connected": ros_connected, "telemetry_topics": ros_topics_count},
        "collectTime": time.strftime("%Y-%m-%d %H:%M:%S %Z"),
    })
