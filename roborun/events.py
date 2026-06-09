"""In-memory event bus with SSE subscriber support.

Any subsystem (MCP tool calls, vision detections, ROS events, agent actions)
can emit events. The frontend connects via SSE to get a unified live feed.
"""
from __future__ import annotations

import queue
import threading
import time
from collections import deque
from typing import Any

_lock = threading.Lock()
_events: deque[dict] = deque(maxlen=500)
_subscribers: list[queue.Queue] = []
_counter = 0

EVENT_TYPES = ("mcp_tool", "detection", "ros", "agent", "system", "task")
ICONS = {
    "mcp_tool": "⚙",
    "detection": "◉",
    "ros": "⬡",
    "agent": "✦",
    "system": "◆",
    "task": "▶",
}


def emit(event_type: str, source: str, title: str,
         detail: dict[str, Any] | None = None) -> dict:
    global _counter
    with _lock:
        _counter += 1
        event = {
            "id": f"evt_{_counter}",
            "type": event_type,
            "source": source,
            "title": title,
            "detail": detail or {},
            "ts": time.time(),
        }
        _events.append(event)
        dead: list[queue.Queue] = []
        for q in _subscribers:
            try:
                q.put_nowait(event)
            except queue.Full:
                dead.append(q)
        for q in dead:
            _subscribers.remove(q)
    return event


def subscribe() -> queue.Queue:
    q: queue.Queue = queue.Queue(maxsize=200)
    with _lock:
        _subscribers.append(q)
    return q


def unsubscribe(q: queue.Queue) -> None:
    with _lock:
        try:
            _subscribers.remove(q)
        except ValueError:
            pass


def recent(limit: int = 100) -> list[dict]:
    with _lock:
        items = list(_events)
    return items[-limit:]
