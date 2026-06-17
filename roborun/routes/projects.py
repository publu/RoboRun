"""Project + Environment routes (platform specs 07/08).

Read-only listing + create + active-context switch. Pure files, no infra.
Selecting a project/environment re-scopes the recorder + event journal + the
project-aware APIs (timeline/scenarios) under it.
"""
from __future__ import annotations

from urllib.parse import parse_qs, urlparse

from roborun.routes import get, post, send_json, ApiError
from roborun import projects, environments


@get("/api/projects")
def list_projects(h):
    send_json(h, 200, {"ok": True, "projects": projects.list_projects(),
                       "active": projects.active()})


@post("/api/projects")
def create_project(h, payload):
    name = str(payload.get("name", "")).strip()
    if not name:
        raise ApiError(400, "name required")
    meta = projects.create(name, mode=str(payload.get("mode", "scratch")))
    # a project is only useful with at least one environment
    backend = str(payload.get("backend", "rapier"))
    if not environments.list_envs(meta["id"]):
        environments.create(meta["id"], "default", backend=backend,
                             mode=meta.get("mode_default", "scratch"))
    send_json(h, 200, {"ok": True, "project": meta})


@get("/api/projects/active")
def get_active(h):
    a = projects.active()
    env = environments.active_meta() if a else None
    send_json(h, 200, {"ok": True, "active": a, "environment": env})


@post("/api/projects/active")
def set_active(h, payload):
    project = str(payload.get("project", "")).strip()
    if not project:
        raise ApiError(400, "project required")
    env = str(payload.get("environment", "default")).strip() or "default"
    projects.create(project)                       # idempotent
    if not environments.get(project, env):
        environments.create(project, env,
                            backend=str(payload.get("backend", "rapier")))
    ctx = projects.set_active(project, env)
    send_json(h, 200, {"ok": True, "active": ctx})


@post("/api/projects/active/clear")
def clear_active(h, payload):
    projects.clear_active()
    send_json(h, 200, {"ok": True, "active": None})


@get("/api/environments")
def list_environments(h):
    q = parse_qs(urlparse(h.path).query)
    project = (q.get("project") or [None])[0] or (projects.active() or {}).get("project")
    if not project:
        send_json(h, 200, {"ok": True, "environments": [], "project": None})
        return
    send_json(h, 200, {"ok": True, "project": project,
                       "environments": environments.list_envs(project)})


@post("/api/environments")
def create_environment(h, payload):
    project = str(payload.get("project", "")).strip() or (projects.active() or {}).get("project")
    name = str(payload.get("name", "")).strip()
    if not project or not name:
        raise ApiError(400, "project and name required")
    meta = environments.create(project, name,
                               backend=str(payload.get("backend", "rapier")),
                               mode=str(payload.get("mode", "scratch")),
                               world_ref=payload.get("world_ref"))
    send_json(h, 200, {"ok": True, "environment": meta})


@post("/api/environments/camera")
def register_camera(h, payload):
    """Tag a camera's placement in the environment frame (spec 09)."""
    project = str(payload.get("project", "")).strip() or (projects.active() or {}).get("project")
    env = str(payload.get("environment", "")).strip() or (projects.active() or {}).get("environment")
    source_id = str(payload.get("source_id", "")).strip()
    if not project or not env or not source_id:
        raise ApiError(400, "project, environment, source_id required")
    meta = environments.register_camera(
        project, env, source_id,
        placement=payload.get("placement"),
        kind=str(payload.get("kind", "robot")),
        intrinsics=payload.get("intrinsics"))
    if meta is None:
        raise ApiError(404, "environment not found")
    send_json(h, 200, {"ok": True, "environment": meta})


@get("/api/backends")
def list_backends(h):
    """Backend registry + live capability matrix (spec 03)."""
    from roborun import backends
    send_json(h, 200, {"ok": True, "backends": backends.list_backends()})


@get("/api/runs")
def list_runs(h):
    """MCAP runs in the active project/environment, with their manifests —
    the telemetry browser's index (spec 02)."""
    from roborun import recorder as rec_mod, run_manifest
    runs = rec_mod.list_runs()
    for r in runs:
        m = run_manifest.read(r.get("mcap", "")) if r.get("mcap") else None
        if m:
            r["manifest"] = {k: m.get(k) for k in
                             ("project", "environment", "backend", "started",
                              "ended", "cameras")}
    send_json(h, 200, {"ok": True, "runs": runs, "active": projects.active()})
