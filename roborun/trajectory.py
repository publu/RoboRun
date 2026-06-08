"""Trajectory recorder — stores timestamped robot poses for 3D path visualization."""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any

MAX_POINTS = 10000


class TrajectoryRecorder:
    _instance: TrajectoryRecorder | None = None
    _lock = threading.Lock()

    def __init__(self) -> None:
        self._buffer: deque[dict[str, Any]] = deque(maxlen=MAX_POINTS)
        self._recording = False
        self._buf_lock = threading.Lock()

    @classmethod
    def get(cls) -> TrajectoryRecorder:
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    @property
    def is_recording(self) -> bool:
        return self._recording

    def start(self) -> dict[str, Any]:
        self._recording = True
        return {"ok": True, "recording": True}

    def stop(self) -> dict[str, Any]:
        self._recording = False
        with self._buf_lock:
            count = len(self._buffer)
        return {"ok": True, "recording": False, "points": count}

    def clear(self) -> dict[str, Any]:
        with self._buf_lock:
            self._buffer.clear()
        return {"ok": True, "cleared": True}

    def add_pose(self, x: float, y: float, z: float,
                 qw: float = 1.0, qx: float = 0.0,
                 qy: float = 0.0, qz: float = 0.0,
                 robot_id: str = "local") -> None:
        if not self._recording:
            return
        entry = {
            "t": time.time(),
            "x": x, "y": y, "z": z,
            "qw": qw, "qx": qx, "qy": qy, "qz": qz,
            "robot_id": robot_id,
        }
        with self._buf_lock:
            self._buffer.append(entry)

    def get_trajectory(self, limit: int = 0) -> list[dict[str, Any]]:
        with self._buf_lock:
            data = list(self._buffer)
        if limit > 0:
            data = data[-limit:]
        return data

    def get_state(self) -> dict[str, Any]:
        with self._buf_lock:
            count = len(self._buffer)
        return {"recording": self._recording, "points": count}
