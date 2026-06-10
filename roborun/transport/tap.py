"""Tap mode — the bridge from the ROS graph to the black box (spec §1.2.4).

A passive subscriber that taps a configured set of topics straight into the
MCAP recorder, with no LLM in the loop. The tap runs at full topic rate; the
agent runs at its own pace; both land in the same sealed run. This is the
literal meaning of ros_tap: tap the ROS graph into the recorder.

Routing by message type:
    sensor_msgs/CompressedImage → /camera/<name>      (native Foxglove replay)
    nav_msgs/Odometry, PoseStamped, TFMessage → /pose (+ raw copy)
    everything else             → recorded verbatim on its own topic

Usage:
    tap = Tap(transport, recorder, topics=["/odom", "/camera/image/compressed"])
    tap.start()
    ...
    tap.stop()    # recorder seals separately
"""
from __future__ import annotations

import base64
import fnmatch
import threading
import time
from typing import Any

from roborun.transport import Transport
from roborun.transport.schemas import normalize_type


class Tap:

    def __init__(self, transport: Transport, recorder,
                 topics: list[str] | None = None) -> None:
        """`topics` accepts exact names or fnmatch patterns ('/camera/*').
        None taps every topic with a known type."""
        self._transport = transport
        self._recorder = recorder
        self._patterns = topics
        self._subscribed: dict[str, str] = {}   # topic -> msg_type
        self._counts: dict[str, int] = {}
        self._errors: dict[str, str] = {}
        self._lock = threading.Lock()
        self._running = False

    def _matches(self, topic: str) -> bool:
        if self._patterns is None:
            return True
        return any(fnmatch.fnmatch(topic, p) or topic == p for p in self._patterns)

    def start(self) -> dict[str, Any]:
        graph = self._transport.topics()
        for topic, msg_type in graph.items():
            if not self._matches(topic) or topic in self._subscribed:
                continue
            try:
                self._transport.subscribe(
                    topic, self._make_callback(topic, msg_type), msg_type)
                self._subscribed[topic] = msg_type
            except Exception as exc:
                self._errors[topic] = str(exc)  # surfaced, not swallowed
        self._running = True
        return self.status()

    def refresh(self) -> dict[str, Any]:
        """Pick up topics that appeared after start (discovery is ongoing)."""
        return self.start()

    def stop(self) -> dict[str, Any]:
        for topic in list(self._subscribed):
            try:
                self._transport.unsubscribe(topic)
            except Exception:
                pass
        self._running = False
        return self.status()

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "running": self._running,
                "topics": dict(self._subscribed),
                "messages": dict(self._counts),
                "not_tappable": dict(self._errors),
            }

    # ── routing ──────────────────────────────────────────────────────────

    def _make_callback(self, topic: str, msg_type: str):
        kind = normalize_type(msg_type)
        name = topic.strip("/").replace("/", "_") or "root"

        def cb(msg: dict) -> None:
            ts = _msg_time(msg) or time.time()
            try:
                if kind == "sensor_msgs/CompressedImage":
                    data = msg.get("data", b"")
                    if isinstance(data, list):
                        data = bytes(data)
                    elif isinstance(data, str):
                        data = base64.b64decode(data)
                    self._recorder.write_camera(data, name=name, ts=ts,
                                                frame_id=_frame_id(msg))
                elif kind in ("nav_msgs/Odometry", "geometry_msgs/PoseStamped"):
                    pose = msg.get("pose") or {}
                    if "pose" in pose:  # Odometry nests pose.pose
                        pose = pose["pose"]
                    pos = pose.get("position") or {}
                    self._recorder.write_pose(
                        pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0),
                        orientation=pose.get("orientation"),
                        frame_id=_frame_id(msg), ts=ts)
                    self._recorder.write_json(topic, "roborun.Json", msg, ts)
                else:
                    self._recorder.write_json(topic, "roborun.Json", msg, ts)
                with self._lock:
                    self._counts[topic] = self._counts.get(topic, 0) + 1
            except Exception:
                pass  # a bad message must never kill the tap thread

        return cb


def _msg_time(msg: dict) -> float | None:
    stamp = (msg.get("header") or {}).get("stamp") or {}
    sec = stamp.get("sec")
    if sec:
        return sec + stamp.get("nanosec", stamp.get("nsec", 0)) / 1e9
    return None


def _frame_id(msg: dict) -> str:
    return (msg.get("header") or {}).get("frame_id", "") or "robot"
