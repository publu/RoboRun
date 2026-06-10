"""Robot-camera perception: the robot's eyes follow the robot's body.

Subscribes to the connected robot's CompressedImage topic over rosbridge,
runs the same YOLO the webcam pipeline uses, and serves detections through
the same cache `robot.see()` reads. With this, see() means "what is in
front of the *robot*" on every backend: arena → arena ground truth,
connected robot → this pipeline, neither → local webcam.

Frames also land at /tmp/roborun_frame.jpg, so the deck's camera panel and
robot.ask(image=True) show the robot's view, not your desk.
"""
from __future__ import annotations

import base64
import threading
import time
from pathlib import Path
from typing import Any

FRAME_PATH = Path("/tmp/roborun_frame.jpg")
_FRESH = 2.0
_DETECT_HZ = 5.0


class RosCameraPipeline:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._topic: str | None = None
        self._detections: list[dict] = []
        self._frame = None              # np.ndarray (BGR)
        self._frame_ts = 0.0
        self._last_detect = 0.0
        self._detector = None
        self._frames = 0

    # ── lifecycle ────────────────────────────────────────────────────────

    def start(self, topic: str | None = None) -> dict[str, Any]:
        from roborun.rosbridge import get_client
        client = get_client(auto_connect=False)
        if client is None or not client.is_connected:
            return {"ok": False, "error": "no robot connected (roborun connect <ip>)"}
        topic = topic or self._find_camera_topic(client)
        if not topic:
            return {"ok": False, "error": "no CompressedImage topic found on the robot"}
        if self._topic:
            self.stop()
        client.subscribe(topic, "sensor_msgs/CompressedImage", self._on_msg,
                         throttle_rate=120)  # ms → ~8 fps over the wire
        self._topic = topic
        return {"ok": True, "topic": topic}

    def stop(self) -> dict[str, Any]:
        if self._topic:
            try:
                from roborun.rosbridge import get_client
                client = get_client(auto_connect=False)
                if client:
                    client.unsubscribe(self._topic)
            except Exception:
                pass
        topic, self._topic = self._topic, None
        return {"ok": True, "stopped": topic}

    @staticmethod
    def _find_camera_topic(client) -> str | None:
        try:
            topics = client.list_topics(timeout=4.0)
        except Exception:
            return None
        names = [t.get("topic", "") for t in topics
                 if "CompressedImage" in t.get("type", "")]
        # prefer color cams over depth
        names.sort(key=lambda n: ("depth" in n.lower(), len(n)))
        return names[0] if names else None

    # ── frame path ───────────────────────────────────────────────────────

    def _on_msg(self, msg: dict) -> None:
        data = msg.get("data", "")
        try:
            raw = base64.b64decode(data) if isinstance(data, str) else bytes(data)
        except Exception:
            return
        self.ingest_jpeg(raw)

    def ingest_jpeg(self, raw: bytes) -> None:
        """Decode + detect. Separated from transport so tests can feed frames."""
        try:
            import cv2
            import numpy as np
            frame = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
        except Exception:
            return
        if frame is None:
            return
        now = time.monotonic()
        dets = None
        if now - self._last_detect >= 1.0 / _DETECT_HZ:
            self._last_detect = now
            try:
                if self._detector is None:
                    from roborun.models import YOLODetector
                    self._detector = YOLODetector()
                dets = [d.to_dict() for d in self._detector.detect(frame)]
            except Exception:
                dets = []
        with self._lock:
            self._frame = frame
            self._frame_ts = now
            self._frames += 1
            if dets is not None:
                self._detections = dets
        if dets:
            try:
                from roborun.sightings import observe
                observe(dets, source="robot-camera")
            except Exception:
                pass
        try:
            FRAME_PATH.write_bytes(raw)
        except Exception:
            pass

    # ── the cache robot.see() reads ──────────────────────────────────────

    def is_active(self) -> bool:
        with self._lock:
            return time.monotonic() - self._frame_ts < _FRESH

    def get_detections(self) -> list[dict]:
        with self._lock:
            return list(self._detections)

    def snapshot(self):
        with self._lock:
            return self._frame

    def state(self) -> dict[str, Any]:
        with self._lock:
            return {"topic": self._topic, "active": self.is_active(),
                    "frames": self._frames, "detections": len(self._detections)}


_pipeline: RosCameraPipeline | None = None
_pipeline_lock = threading.Lock()


def get_ros_camera() -> RosCameraPipeline:
    global _pipeline
    with _pipeline_lock:
        if _pipeline is None:
            _pipeline = RosCameraPipeline()
        return _pipeline
