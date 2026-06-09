"""Skills registry API routes."""
from __future__ import annotations

from roborun.routes import get, send_json


@get("/api/skills")
def list_skills(h):
    from roborun.skills import get_registry
    reg = get_registry()
    skills = []
    for sid, info in reg.skills.items():
        tools = [t["name"] for t in reg.tools.values() if t.get("skill_id") == sid]
        behaviors = [b["name"] for b in reg.behaviors.values() if b.get("skill_id") == sid]
        skills.append({
            "id": info["id"],
            "name": info["name"],
            "version": info["version"],
            "description": info.get("description", ""),
            "tools": tools,
            "behaviors": behaviors,
        })
    send_json(h, 200, {
        "ok": True,
        "skills": skills,
        "total_skills": len(skills),
        "total_tools": len(reg.tools),
        "total_behaviors": len(reg.behaviors),
    })


@get("/api/skills/behaviors")
def list_behaviors(h):
    from roborun.skills import get_registry
    reg = get_registry()
    behaviors = [
        {"name": b["name"], "description": b["description"], "skill_id": b.get("skill_id", "")}
        for b in reg.behaviors.values()
    ]
    send_json(h, 200, {"ok": True, "behaviors": behaviors, "total": len(behaviors)})
