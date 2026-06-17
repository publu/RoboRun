"""Environments — a registered space inside a project (platform specs 08 + 09).

An Environment binds a coordinate frame + map + tagged camera placements to a
backend (rapier/gazebo/isaac/real) and a mode (scratch/test/production). Data
for a session lands under the active project/environment (see projects.py).

    <state>/projects/<project>/<env>/env.json
        { id, name, backend, mode, world_ref, map_ref, cameras[] }

`cameras[]` entries carry placement/extrinsics so every sensor maps into one
spatial frame (spec 09):  {source_id, kind: robot|fixed, placement:{x,y,z,
roll,pitch,yaw}, intrinsics?}
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

from roborun import projects
from roborun.projects import slug, MODES

BACKENDS = ("rapier", "mujoco", "mjx", "gazebo", "isaac", "real")


def _env_dir(project: str, env: str) -> Path:
    return projects.projects_root() / slug(project) / slug(env)


def _meta_path(project: str, env: str) -> Path:
    return _env_dir(project, env) / "env.json"


def create(project: str, name: str, backend: str = "rapier",
           mode: str = "scratch", world_ref: str | None = None) -> dict[str, Any]:
    projects.create(project)                      # ensure the project exists
    eid = slug(name)
    p = _meta_path(project, eid)
    p.parent.mkdir(parents=True, exist_ok=True)
    if p.exists():
        return json.loads(p.read_text())
    meta = {
        "id": eid, "name": name,
        "backend": backend if backend in BACKENDS else "rapier",
        "mode": mode if mode in MODES else "scratch",
        "world_ref": world_ref,
        "map_ref": None,          # populated when a map is registered (spec 09)
        "cameras": [],            # tagged placements (spec 09)
        "frame": "world",
        "created": time.time(),
    }
    p.write_text(json.dumps(meta, indent=2))
    _link_to_project(project, eid)
    return meta


def _link_to_project(project: str, env: str) -> None:
    meta = projects.get(project)
    if meta is None:
        return
    envs = set(meta.get("environments", []))
    if env not in envs:
        meta.setdefault("environments", []).append(env)
        projects._meta_path(slug(project)).write_text(json.dumps(meta, indent=2))


def get(project: str, env: str) -> dict[str, Any] | None:
    p = _meta_path(project, env)
    if not p.exists():
        return None
    try:
        return json.loads(p.read_text())
    except Exception:
        return None


def list_envs(project: str) -> list[dict[str, Any]]:
    root = projects.projects_root() / slug(project)
    if not root.exists():
        return []
    out = []
    for d in sorted(root.iterdir()):
        if not d.is_dir():
            continue
        m = d / "env.json"
        if m.exists():
            try:
                meta = json.loads(m.read_text())
            except Exception:
                continue
            meta["runs"] = len(list((d / "runs").glob("*/*.mcap"))) + \
                len(list((d / "runs").glob("*.mcap"))) if (d / "runs").exists() else 0
            out.append(meta)
    return out


def update(project: str, env: str, **fields: Any) -> dict[str, Any] | None:
    meta = get(project, env)
    if meta is None:
        return None
    meta.update({k: v for k, v in fields.items() if v is not None})
    _meta_path(project, env).write_text(json.dumps(meta, indent=2))
    return meta


def register_camera(project: str, env: str, source_id: str,
                    placement: dict | None = None, kind: str = "robot",
                    intrinsics: dict | None = None) -> dict[str, Any] | None:
    """Tag a camera's placement in the environment frame (spec 09)."""
    meta = get(project, env)
    if meta is None:
        return None
    cams = [c for c in meta.get("cameras", []) if c.get("source_id") != source_id]
    cams.append({"source_id": source_id, "kind": kind,
                 "placement": placement or {"x": 0, "y": 0, "z": 0,
                                            "roll": 0, "pitch": 0, "yaw": 0},
                 "intrinsics": intrinsics})
    meta["cameras"] = cams
    _meta_path(project, env).write_text(json.dumps(meta, indent=2))
    return meta


def active_meta() -> dict[str, Any] | None:
    a = projects.active()
    if not a:
        return None
    return get(a["project"], a["environment"])
