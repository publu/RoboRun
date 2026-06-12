"""Source inventory: what can see and what can move, right now.

One place that answers "what's available?" — the webcam, a connected
robot's camera, and rosbridge servers discovered on the local network —
so UIs offer sources instead of making the user remember ports. The LAN
scan is a plain TCP probe of :9090 across the machine's /24: cheap,
parallel, and exactly what `roborun connect <ip>` would need anyway.
"""
from __future__ import annotations

import socket
import threading
import time
from typing import Any

ROSBRIDGE_PORT = 9090
_SCAN_TTL = 60.0          # results stay fresh this long
_scan_lock = threading.Lock()
_scan: dict[str, Any] = {"scanning": False, "found": [], "scanned_at": 0.0}


def _local_subnet() -> list[str]:
    """Hosts on this machine's /24 (best effort, no extra deps)."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))          # no traffic sent; just routes
        ip = s.getsockname()[0]
        s.close()
    except Exception:
        return []
    base = ip.rsplit(".", 1)[0]
    return [f"{base}.{i}" for i in range(1, 255) if f"{base}.{i}" != ip]


def _probe(host: str, port: int = ROSBRIDGE_PORT, timeout: float = 0.6) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False


def _scan_worker() -> None:
    found: list[dict] = []
    if _probe("127.0.0.1"):
        found.append({"host": "127.0.0.1", "port": ROSBRIDGE_PORT, "local": True})
    hosts = _local_subnet()
    sem = threading.Semaphore(64)
    lock = threading.Lock()

    def check(h: str) -> None:
        with sem:
            if _probe(h):
                with lock:
                    found.append({"host": h, "port": ROSBRIDGE_PORT, "local": False})

    threads = [threading.Thread(target=check, args=(h,), daemon=True) for h in hosts]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=5.0)
    with _scan_lock:
        _scan.update(scanning=False, found=found, scanned_at=time.time())


def network_scan(force: bool = False) -> dict[str, Any]:
    """Cached LAN rosbridge inventory; kicks a background rescan when stale."""
    with _scan_lock:
        stale = time.time() - _scan["scanned_at"] > _SCAN_TTL
        if (force or stale) and not _scan["scanning"]:
            _scan["scanning"] = True
            threading.Thread(target=_scan_worker, daemon=True).start()
        return dict(_scan)


def inventory() -> dict[str, Any]:
    """Everything a source picker needs, without side effects: peeking
    must never start a pipeline or open the camera."""
    from roborun.routes import _singletons

    wc = _singletons._webcam              # peek — do not instantiate
    webcam = {"available": True, "on": bool(wc and wc.is_running)}

    robot: dict[str, Any] = {"connected": False}
    try:
        from roborun.rosbridge import get_client
        client = get_client(auto_connect=False)
        if client and client.is_connected:
            from roborun.ros_camera import get_ros_camera
            from roborun.ros_telemetry import get_bridge
            cam = get_ros_camera().state()
            b = get_bridge()
            robot = {
                "connected": True,
                "host": client.health.get("host"),
                "type": b.robot_type.value if b.robot_type else None,
                "camera_topic": cam.get("topic"),
                "camera_active": cam.get("active", False),
            }
    except Exception:
        pass

    return {"ok": True, "webcam": webcam, "robot": robot,
            "network": network_scan()}
