"""Projects — the first-class container that scopes data (platform spec 07).

A Project is a named body of work that owns a data root and can span
Environments (spec 08). The **active** project/environment is small runtime
state the server + runner read; when one is selected, the recorder + event
journal write under ``<state>/projects/<project>/<env>/`` instead of the flat
legacy ``<state>/runs``. With nothing selected, everything behaves exactly as
before — so this layer is purely additive and back-compatible.

    <state>/projects/<project_id>/project.json
    <state>/projects/<project_id>/<env_id>/env.json
    <state>/active.json            # {project, environment}

``<state>`` is ``ROBORUN_STATE_DIR`` or ``~/.roborun`` (matches recorder.py).
"""
from __future__ import annotations

import json
import os
import re
import time
from pathlib import Path
from typing import Any

SCRATCH = "scratch"          # the zero-friction default project (spec 08)
MODES = ("scratch", "test", "production")


def state_dir() -> Path:
    base = os.environ.get("ROBORUN_STATE_DIR")
    return Path(base) if base else Path.home() / ".roborun"


def projects_root() -> Path:
    return state_dir() / "projects"


def slug(name: str) -> str:
    s = re.sub(r"[^a-z0-9]+", "-", (name or "").lower()).strip("-")
    return s or "untitled"


# ── active context ────────────────────────────────────────────────────────
def _active_file() -> Path:
    return state_dir() / "active.json"


def active() -> dict[str, str] | None:
    """The active {project, environment}, or None (→ legacy flat layout).

    Env vars win so a runner can be pinned without touching disk state."""
    penv = os.environ.get("ROBORUN_PROJECT")
    if penv:
        return {"project": slug(penv),
                "environment": slug(os.environ.get("ROBORUN_ENV") or "default")}
    f = _active_file()
    if f.exists():
        try:
            d = json.loads(f.read_text())
            if d.get("project"):
                return {"project": d["project"],
                        "environment": d.get("environment") or "default"}
        except Exception:
            return None
    return None


def set_active(project: str, environment: str = "default") -> dict[str, str]:
    pid, eid = slug(project), slug(environment)
    state_dir().mkdir(parents=True, exist_ok=True)
    ctx = {"project": pid, "environment": eid}
    _active_file().write_text(json.dumps(ctx))
    return ctx


def clear_active() -> None:
    f = _active_file()
    if f.exists():
        f.unlink()


def data_root() -> Path | None:
    """``<state>/projects/<project>/<env>`` when a project is active, else None
    (callers fall back to the legacy ``<state>/runs``)."""
    a = active()
    if not a:
        return None
    return projects_root() / a["project"] / a["environment"]


# ── project CRUD ──────────────────────────────────────────────────────────
def _meta_path(pid: str) -> Path:
    return projects_root() / pid / "project.json"


def create(name: str, mode: str = "scratch") -> dict[str, Any]:
    pid = slug(name)
    p = _meta_path(pid)
    p.parent.mkdir(parents=True, exist_ok=True)
    if p.exists():
        return json.loads(p.read_text())
    meta = {"id": pid, "name": name, "created": time.time(),
            "mode_default": mode if mode in MODES else "scratch",
            "environments": []}
    p.write_text(json.dumps(meta, indent=2))
    return meta


def get(pid: str) -> dict[str, Any] | None:
    p = _meta_path(slug(pid))
    if not p.exists():
        return None
    try:
        return json.loads(p.read_text())
    except Exception:
        return None


def list_projects() -> list[dict[str, Any]]:
    root = projects_root()
    if not root.exists():
        return []
    out = []
    for d in sorted(root.iterdir()):
        m = d / "project.json"
        if m.exists():
            try:
                meta = json.loads(m.read_text())
            except Exception:
                continue
            # cheap activity rollup: count runs across the project's envs
            runs = len(list(d.glob("*/runs/*/*.mcap"))) + len(list(d.glob("*/runs/*.mcap")))
            meta["runs"] = runs
            out.append(meta)
    out.sort(key=lambda m: m.get("created", 0), reverse=True)
    return out


def ensure_scratch() -> dict[str, Any]:
    """The default project always exists so 'just playing' has a home."""
    return create("scratch", mode="scratch")
