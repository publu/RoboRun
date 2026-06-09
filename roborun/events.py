"""Event bus: live SSE fan-out + an append-only, hash-chained journal.

Any subsystem (MCP tool calls, vision detections, ROS events, agent actions)
emits events here. Every event:

  1. Fans out to SSE subscribers (the live UI feed).
  2. Appends to the current run's journal (run.jsonl) with `prev` set to the
     SHA-256 of the previous event — a hash chain, so the log is tamper-evident
     *while it is being written*, not only after sealing.

Sealing (roborun.integrity via routes/run.py) closes the current journal and
starts a new one whose manifest records the sealed run's Merkle root — runs
chain end-to-end like blocks.

Journals live in ~/.roborun/runs (override: ROBORUN_STATE_DIR).
"""
from __future__ import annotations

import json
import os
import queue
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, IO

from roborun.integrity import GENESIS, hash_event

_lock = threading.Lock()
_events: deque[dict] = deque(maxlen=500)
_subscribers: list[queue.Queue] = []
_counter = 0

# Journal state (all guarded by _lock)
_journal: IO[str] | None = None
_journal_dir: Path | None = None
_prev_hash = GENESIS
_last_sealed: dict[str, str] | None = None  # {"run": ..., "merkle_root": ...}

EVENT_TYPES = ("mcp_tool", "detection", "ros", "agent", "system", "task", "frame")
ICONS = {
    "mcp_tool": "⚙",
    "detection": "◉",
    "ros": "⬡",
    "agent": "✦",
    "system": "◆",
    "task": "▶",
    "frame": "▣",
}


def runs_root() -> Path:
    base = os.environ.get("ROBORUN_STATE_DIR")
    root = Path(base) if base else Path.home() / ".roborun"
    return root / "runs"


def _start_journal_locked() -> None:
    """Open a fresh run directory + journal. Caller holds _lock."""
    global _journal, _journal_dir, _prev_hash
    run_id = time.strftime("run_%Y%m%d_%H%M%S", time.gmtime())
    d = runs_root() / run_id
    # Same-second restart collision: suffix until free
    n = 1
    while d.exists():
        d = runs_root() / f"{run_id}_{n}"
        n += 1
    d.mkdir(parents=True)
    manifest: dict[str, Any] = {
        "run": d.name,
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "format": "chained-jsonl-v1",
    }
    if _last_sealed:
        manifest["prev_run"] = _last_sealed
    (d / "manifest.json").write_text(json.dumps(manifest, indent=2))
    _journal = open(d / "run.jsonl", "a", buffering=1)  # line-buffered
    _journal_dir = d
    _prev_hash = GENESIS


def current_run() -> dict[str, Any] | None:
    with _lock:
        if _journal_dir is None:
            return None
        return {"run": _journal_dir.name, "events": _counter, "head": _prev_hash}


def close_journal() -> Path | None:
    """Close the live journal for sealing. Returns its dir, or None if empty."""
    global _journal, _journal_dir
    with _lock:
        if _journal is None or _journal_dir is None:
            return None
        _journal.close()
        d = _journal_dir
        _journal = None
        _journal_dir = None
        return d


def record_sealed(run_name: str, merkle_root: str) -> None:
    """Remember the last sealed run so the next journal's manifest links to it."""
    global _last_sealed
    with _lock:
        _last_sealed = {"run": run_name, "merkle_root": merkle_root}


def emit(event_type: str, source: str, title: str,
         detail: dict[str, Any] | None = None) -> dict:
    global _counter, _prev_hash
    with _lock:
        _counter += 1
        event = {
            "id": f"evt_{_counter}",
            "type": event_type,
            "source": source,
            "title": title,
            "detail": detail or {},
            "ts": time.time(),
            "prev": _prev_hash,
        }
        try:
            if _journal is None:
                _start_journal_locked()
                event["prev"] = _prev_hash  # fresh journal ⇒ genesis
            _journal.write(json.dumps(event, default=str) + "\n")
            _prev_hash = hash_event(event)
        except OSError:
            pass  # journal unavailable — live feed still works
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
