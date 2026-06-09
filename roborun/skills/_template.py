"""Skill template — copy this file to create a new RoboRun skill.

Rename it, update the constants below, and implement your tools/behaviors.
Drop it in roborun/skills/ for built-in loading, or point to it via:
  - ROBORUN_SKILL_PATHS=/path/to/your_skill.py
  - .roborun/skills.yaml → paths: ["/path/to/your_skill.py"]

A skill registers tools (callable from MCP / agent / UI) and optional
behaviors (long-running autonomous loops like follow-me or patrol).
"""
from __future__ import annotations

# ── Required metadata ────────────────────────────────────────────────────────

SKILL_ID = "my-skill"        # unique slug, used in registry
SKILL_NAME = "My Skill"      # display name
SKILL_VERSION = "0.1.0"      # semver


# ── Tool handlers ────────────────────────────────────────────────────────────
# Each handler takes a dict of args and returns a dict result.
# The result MUST include an "ok" key (True/False).

def _my_tool(args: dict) -> dict:
    name = args.get("name", "world")
    return {"ok": True, "message": f"Hello, {name}!"}


# ── Registration ─────────────────────────────────────────────────────────────
# Called once at startup. Register all your tools and behaviors here.

def register(registry) -> None:
    registry.add_tool(
        name="my_skill_greet",           # tool name (must be unique across all skills)
        description="Say hello.",         # shown to LLM — be specific about what it does
        input_schema={                    # JSON Schema for the tool arguments
            "type": "object",
            "properties": {
                "name": {"type": "string", "description": "Who to greet"},
            },
        },
        handler=_my_tool,
        skill_id=SKILL_ID,
    )

    # Optional: register a behavior (long-running autonomous action)
    # registry.add_behavior(
    #     name="my_behavior",
    #     description="Does something continuously",
    #     handler=_my_behavior_handler,
    #     skill_id=SKILL_ID,
    # )
