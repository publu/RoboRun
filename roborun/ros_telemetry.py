"""Auto-subscribe to standard ROS/ROS2 topics and feed into TelemetryBus.

When a rosbridge connection is active, this module subscribes to common
telemetry topics (battery, odometry, IMU, joint states, diagnostics) and
pushes data through the unified TelemetryBus so the dashboard charts and
WebSocket clients see live robot data automatically.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any


STANDARD_TOPICS = [
    ("/battery_state", "sensor_msgs/BatteryState"),
    ("/battery_status", "sensor_msgs/BatteryState"),
    ("/odom", "nav_msgs/Odometry"),
    ("/imu/data", "sensor_msgs/Imu"),
    ("/imu", "sensor_msgs/Imu"),
    ("/joint_states", "sensor_msgs/JointState"),
    ("/diagnostics", "diagnostic_msgs/DiagnosticArray"),
    ("/cmd_vel", "geometry_msgs/Twist"),
    ("/scan", "sensor_msgs/LaserScan"),
    ("/tf", "tf2_msgs/TFMessage"),
]

_instance: RosTelemetryBridge | None = None
_lock = threading.Lock()


def get_bridge() -> RosTelemetryBridge:
    global _instance
    if _instance is None:
        with _lock:
            if _instance is None:
                _instance = RosTelemetryBridge()
    return _instance


def _quaternion_to_euler(x: float, y: float, z: float, w: float) -> dict[str, float]:
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)

    return {"roll": roll, "pitch": pitch, "yaw": yaw}


class RosTelemetryBridge:
    def __init__(self) -> None:
        self._running = False
        self._thread: threading.Thread | None = None
        self._subscribed_topics: set[str] = set()
        self._last_host: str | None = None

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self, host: str | None = None) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, args=(host,), daemon=True, name="RosTelemetry")
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        self._subscribed_topics.clear()
        self._last_host = None

    def _loop(self, initial_host: str | None) -> None:
        from roborun.telemetry import TelemetryBus
        bus = TelemetryBus.get()

        while self._running:
            try:
                self._try_subscribe(bus, initial_host)
            except Exception:
                pass
            time.sleep(5.0)

    def _try_subscribe(self, bus: Any, fallback_host: str | None) -> None:
        from roborun.rosbridge import get_client

        host = fallback_host
        if not host:
            try:
                from roborun.routes.dashboard import load_profile
                host = load_profile().get("robotIp", "")
            except Exception:
                pass

        if not host:
            return

        client = get_client(host, auto_connect=False)
        if not client or not client.is_connected:
            self._subscribed_topics.clear()
            return

        if self._last_host != host:
            self._subscribed_topics.clear()
            self._last_host = host

        try:
            available = client.list_topics(timeout=3.0)
        except Exception:
            return

        available_names = {t["topic"] for t in available}

        for topic, msg_type in STANDARD_TOPICS:
            if topic in self._subscribed_topics:
                continue
            if topic not in available_names:
                continue

            handler = self._make_handler(topic, msg_type, bus)
            if handler is None:
                continue

            try:
                client.subscribe(topic, msg_type, handler, throttle_rate=500)
                self._subscribed_topics.add(topic)
            except Exception:
                pass

    def _make_handler(self, topic: str, msg_type: str, bus: Any) -> Any:
        robot_id = "ros"

        if "BatteryState" in msg_type:
            def on_battery(msg: dict) -> None:
                pct = msg.get("percentage", 0)
                if isinstance(pct, (int, float)) and pct <= 1.0:
                    pct *= 100
                bus.push(robot_id, "battery", {
                    "percent": round(pct, 1),
                    "voltage": msg.get("voltage", 0),
                    "current": msg.get("current", 0),
                    "temperature": msg.get("temperature", 0),
                })
            return on_battery

        if "Odometry" in msg_type:
            def on_odom(msg: dict) -> None:
                pose = msg.get("pose", {}).get("pose", {})
                pos = pose.get("position", {})
                orient = pose.get("orientation", {})
                twist = msg.get("twist", {}).get("twist", {})
                lin = twist.get("linear", {})
                ang = twist.get("angular", {})

                bus.push(robot_id, "position", {
                    "x": pos.get("x", 0),
                    "y": pos.get("y", 0),
                    "z": pos.get("z", 0),
                })

                euler = _quaternion_to_euler(
                    orient.get("x", 0), orient.get("y", 0),
                    orient.get("z", 0), orient.get("w", 1),
                )
                bus.push(robot_id, "orientation", euler)

                bus.push(robot_id, "velocity", {
                    "x": lin.get("x", 0),
                    "y": lin.get("y", 0),
                    "z": lin.get("z", 0),
                    "angular_z": ang.get("z", 0),
                })
            return on_odom

        if "Imu" in msg_type:
            def on_imu(msg: dict) -> None:
                orient = msg.get("orientation", {})
                euler = _quaternion_to_euler(
                    orient.get("x", 0), orient.get("y", 0),
                    orient.get("z", 0), orient.get("w", 1),
                )
                bus.push(robot_id, "orientation", euler)

                accel = msg.get("linear_acceleration", {})
                bus.push(robot_id, "imu_accel", {
                    "x": accel.get("x", 0),
                    "y": accel.get("y", 0),
                    "z": accel.get("z", 0),
                })

                gyro = msg.get("angular_velocity", {})
                bus.push(robot_id, "imu_gyro", {
                    "x": gyro.get("x", 0),
                    "y": gyro.get("y", 0),
                    "z": gyro.get("z", 0),
                })
            return on_imu

        if "JointState" in msg_type:
            def on_joints(msg: dict) -> None:
                names = msg.get("name", [])
                positions = msg.get("position", [])
                velocities = msg.get("velocity", [])
                efforts = msg.get("effort", [])
                joints = {}
                for i, name in enumerate(names):
                    joints[name] = {
                        "position": positions[i] if i < len(positions) else 0,
                        "velocity": velocities[i] if i < len(velocities) else 0,
                        "effort": efforts[i] if i < len(efforts) else 0,
                    }
                bus.push(robot_id, "joints", {"joints": joints, "count": len(names)})
            return on_joints

        if "DiagnosticArray" in msg_type:
            def on_diag(msg: dict) -> None:
                statuses = msg.get("status", [])
                items = []
                for s in statuses[:10]:
                    items.append({
                        "name": s.get("name", ""),
                        "level": s.get("level", 0),
                        "message": s.get("message", ""),
                    })
                if items:
                    bus.push(robot_id, "diagnostics", {"items": items})
            return on_diag

        if "LaserScan" in msg_type:
            def on_scan(msg: dict) -> None:
                ranges = msg.get("ranges", [])
                if not ranges:
                    return
                valid = [r for r in ranges if isinstance(r, (int, float)) and 0.01 < r < 100]
                if not valid:
                    return
                bus.push(robot_id, "lidar", {
                    "min_range": round(min(valid), 2),
                    "max_range": round(max(valid), 2),
                    "mean_range": round(sum(valid) / len(valid), 2),
                    "points": len(ranges),
                })
            return on_scan

        return None
