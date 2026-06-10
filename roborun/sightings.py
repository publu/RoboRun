"""Automatic sighting memory — the system keeps the ledger, not the policy.

Every perception source feeds this as a side effect of seeing: the webcam
pipeline's YOLO, the robot-camera pipeline, the arena's ground-truth
detections. Policies and agents *query* it (`robot.seen("red door")`, the
`seen` MCP tool) instead of bookkeeping their own sets — recon questions
("how many red doors?") are answered from what the system observed, and
the same observations are already in the timeline/MCAP, so a sealed run
carries the evidence for every answer.

Counting is by *episode*: a label seen continuously counts once; seeing it
again after EPISODE_GAP seconds of absence counts as a new sighting.
"""
from __future__ import annotations

import threading
import time
from typing import Any

EPISODE_GAP = 3.0
_MAX_POSES = 50

_lock = threading.Lock()
_log: dict[str, dict[str, Any]] = {}


def observe(detections: list[dict], pose: dict | None = None,
            source: str = "") -> None:
    """Called by perception pipelines on every detection batch. Cheap."""
    if not detections:
        return
    now = time.time()
    with _lock:
        for d in detections:
            label = d.get("label")
            if not label:
                continue
            ent = _log.setdefault(label, {
                "label": label, "count": 0, "first_ts": now,
                "last_ts": 0.0, "source": source, "poses": []})
            if now - ent["last_ts"] > EPISODE_GAP:
                ent["count"] += 1
                if pose is not None and len(ent["poses"]) < _MAX_POSES:
                    ent["poses"].append({k: pose.get(k) for k in ("x", "z")})
            ent["last_ts"] = now
            ent["source"] = source or ent["source"]


def summary(label: str | None = None) -> list[dict]:
    with _lock:
        rows = [dict(e) for e in _log.values()
                if label is None or e["label"] == label]
    return sorted(rows, key=lambda e: -e["count"])


def reset() -> None:
    """New run / new level — the slate is clean (the old run's evidence
    lives on in its sealed recording, not here)."""
    with _lock:
        _log.clear()
