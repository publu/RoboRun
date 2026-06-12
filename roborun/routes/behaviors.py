"""Behavior runtime routes — list, enable, disable the vibecode loops."""
from __future__ import annotations

from roborun.routes import get, post, send_json
from roborun.behaviors import BehaviorRunner


@get("/api/behaviors")
def list_behaviors(h):
    runner = BehaviorRunner.get()
    send_json(h, 200, {"ok": True,
                       "dirs": [str(d) for d in runner.dirs],
                       "behaviors": runner.statuses()})


@post("/api/behaviors/enable")
def enable(h, payload):
    name = str(payload.get("name", "")).strip()
    ok = BehaviorRunner.get().set_enabled(name, True)
    send_json(h, 200 if ok else 404,
              {"ok": ok} if ok else {"ok": False, "error": f"no behavior named {name!r}"})


@post("/api/behaviors/disable")
def disable(h, payload):
    name = str(payload.get("name", "")).strip()
    ok = BehaviorRunner.get().set_enabled(name, False)
    send_json(h, 200 if ok else 404,
              {"ok": ok} if ok else {"ok": False, "error": f"no behavior named {name!r}"})


@post("/api/behaviors/read")
def read_source(h, payload):
    """Editor loads a running behavior's source. Stem only — no paths."""
    from pathlib import Path
    name = str(payload.get("name", "")).strip()
    if not name or "/" in name or "\\" in name or ".." in name:
        send_json(h, 400, {"ok": False, "error": "bad name"})
        return
    p = Path("behaviors") / f"{name}.py"
    if not p.is_file():
        send_json(h, 404, {"ok": False, "error": f"{name} not found"})
        return
    send_json(h, 200, {"ok": True, "name": name, "source": p.read_text()})


@post("/api/behaviors/write")
def write(h, payload):
    """The arena code panel writes through here — same validation as MCP."""
    from roborun.behaviors import write_behavior_file
    result = write_behavior_file(str(payload.get("name", "")),
                                 str(payload.get("source", "")))
    send_json(h, 200 if result["ok"] else 400, result)


@post("/api/behaviors/compile")
def compile_mission(h, payload):
    """Plain language -> policy source (the editor's words-first path)."""
    from roborun.mission import compile_mission as cm
    result = cm(str(payload.get("mission", "")), str(payload.get("context", "")))
    send_json(h, 200 if result["ok"] else 400, result)
