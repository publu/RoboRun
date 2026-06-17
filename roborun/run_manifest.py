"""Run manifest (platform spec 01 P2) — a run's table of contents.

A tiny JSON written beside each MCAP so consumers (telemetry browser, search)
can list a run and know its project/environment/backend/cameras without
cracking the tape. Best-effort: never let manifest IO break a recording.

    <run>.run.json  next to  <run>.mcap
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any


def manifest_path(mcap_path: str | Path) -> Path:
    return Path(mcap_path).with_suffix(".run.json")


def write_start(mcap_path: str | Path, run_id: str, robot_id: str,
                backend: str | None = None) -> dict[str, Any]:
    try:
        from roborun import projects, environments
        a = projects.active() or {}
        env = environments.active_meta() or {}
        m = {
            "run_id": run_id, "robot_id": robot_id,
            "project": a.get("project"), "environment": a.get("environment"),
            "backend": backend or env.get("backend"),
            "cameras": env.get("cameras", []),
            "frame": env.get("frame", "world"),
            "started": time.time(), "ended": None,
            "channels": [], "seal": None,
        }
        manifest_path(mcap_path).write_text(json.dumps(m, indent=2))
        return m
    except Exception:
        return {}


def finalize(mcap_path: str | Path, seal: dict | None = None,
             channels: list[str] | None = None) -> dict[str, Any]:
    try:
        p = manifest_path(mcap_path)
        m = json.loads(p.read_text()) if p.exists() else {"run_id": Path(mcap_path).stem}
        m["ended"] = time.time()
        if seal:
            m["seal"] = {"merkle_root": seal.get("merkle_root"),
                         "anchor": (seal.get("anchor") or {}).get("status")}
        if channels:
            m["channels"] = channels
        p.write_text(json.dumps(m, indent=2))
        return m
    except Exception:
        return {}


def read(mcap_path: str | Path) -> dict[str, Any] | None:
    p = manifest_path(mcap_path)
    if not p.exists():
        return None
    try:
        return json.loads(p.read_text())
    except Exception:
        return None
