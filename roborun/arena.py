"""Arena bridge — the browser sim is an actuator/sensor backend.

The arena (web/arena.html) renders and steps the world in the browser;
this module is the meeting point with the Python side:

    behaviors  robot.move()  ──▶  set_cmd()      ──▶  GET /api/arena/cmd  (browser polls)
    behaviors  robot.see()   ◀──  detections()   ◀──  POST /api/arena/state (browser pushes)

The arena is "active" while the browser has pushed state within the last
2 seconds; behaviors then drive the arena dog instead of MuJoCo/rosbridge.
Detections arrive in the same shape the webcam pipeline produces
({label, confidence, bbox}) on a virtual 1280x720 frame, so robot.see()
and the recorder treat the sim exactly like a camera.
"""
from __future__ import annotations

import threading
import time
from typing import Any

_FRESH = 2.0  # seconds of browser silence before the arena counts as gone


class ArenaState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cmd: dict[str, float] = {"forward": 0.0, "strafe": 0.0, "turn": 0.0}
        self._cmd_ts = 0.0
        self._state: dict[str, Any] = {}
        self._state_ts = 0.0

    # ── Python side (behaviors) ──────────────────────────────────────────

    def set_cmd(self, forward: float, strafe: float, turn: float) -> None:
        with self._lock:
            self._cmd = {"forward": forward, "strafe": strafe, "turn": turn}
            self._cmd_ts = time.monotonic()

    def is_active(self) -> bool:
        with self._lock:
            return time.monotonic() - self._state_ts < _FRESH

    def detections(self) -> list[dict]:
        with self._lock:
            return list(self._state.get("detections", []))

    def lidar(self) -> list:
        with self._lock:
            return list(self._state.get("lidar", []))

    def pose(self) -> dict | None:
        with self._lock:
            return self._state.get("pose")

    def level(self) -> dict | None:
        with self._lock:
            return self._state.get("level")

    # ── browser side (routes) ────────────────────────────────────────────

    def cmd(self) -> dict:
        with self._lock:
            # Stale commands decay to stop: a crashed behavior must not
            # leave the dog walking into a wall forever (L0 thinking).
            if time.monotonic() - self._cmd_ts > 1.0:
                return {"forward": 0.0, "strafe": 0.0, "turn": 0.0}
            return dict(self._cmd)

    def update(self, state: dict) -> None:
        with self._lock:
            self._state = state
            self._state_ts = time.monotonic()


_arena: ArenaState | None = None
_arena_lock = threading.Lock()


def get_arena() -> ArenaState:
    global _arena
    with _arena_lock:
        if _arena is None:
            _arena = ArenaState()
        return _arena
