"""Source routes — what can see/move right now, and what's on the network."""
from __future__ import annotations

from roborun.routes import get, post, send_json


@get("/api/sources")
def sources(h):
    from roborun.sources import inventory
    send_json(h, 200, inventory())


@post("/api/sources/scan")
def sources_scan(h, payload):
    from roborun.sources import network_scan
    send_json(h, 200, {"ok": True, **network_scan(force=True)})


@get("/api/robot/detections")
def robot_detections(h):
    """Normalized YOLO boxes from the robot camera, for the cockpit overlay."""
    from roborun.ros_camera import get_ros_camera
    send_json(h, 200, {"ok": True, **get_ros_camera().detections_normalized()})
