"""Incidents — flag a moment in a run to revisit (the data-flywheel primitive).

The winning physical-AI loop turns production data into curated "incidents to
revisit" (Foxglove's framing). An incident is just a `{run_id, ts, note, tag}`
bookmark — stored as a small JSONL sidecar next to the runs, emitted onto the
event timeline, and surfaced on the /run player. Tiny, composes existing infra,
no new store. Search/curation already cover the heavy lifting.
"""
from __future__ import annotations

import json
import time
import uuid
from typing import Any

from roborun.events import emit, runs_root


def _path():
    p = runs_root() / "incidents.jsonl"
    p.parent.mkdir(parents=True, exist_ok=True)
    return p


def flag(run_id: str, ts: float | None = None, note: str = "",
         tag: str = "incident", robot_id: str | None = None) -> dict[str, Any]:
    """Bookmark a moment. `ts` defaults to now (live flagging). Returns the record."""
    rec = {"id": uuid.uuid4().hex[:12], "run_id": run_id,
           "ts": ts if ts is not None else time.time(),
           "note": note, "tag": tag, "robot_id": robot_id,
           "flagged_at": time.time()}
    with open(_path(), "a") as fh:
        fh.write(json.dumps(rec) + "\n")
    emit("incident", robot_id or "operator",
         f"flagged · {tag}" + (f" · {note}" if note else ""),
         {"run_id": run_id, "ts": rec["ts"], "incident_id": rec["id"]})
    return rec


def list_incidents(run_id: str | None = None, tag: str | None = None,
                   limit: int = 200) -> list[dict[str, Any]]:
    p = _path()
    if not p.exists():
        return []
    out: list[dict] = []
    for line in p.read_text().splitlines():
        try:
            r = json.loads(line)
        except Exception:
            continue
        if run_id and r.get("run_id") != run_id:
            continue
        if tag and r.get("tag") != tag:
            continue
        out.append(r)
    out.sort(key=lambda r: r.get("flagged_at", 0), reverse=True)
    return out[:limit]
