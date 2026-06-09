"""Compose skill — chain tools and behaviors into reusable workflows.

The killer feature: run_sequence executes a list of tool calls in order,
passing results between steps. save_workflow persists sequences for
one-click replay. This is what makes RoboRun composable.

Example sequence:
  [
    {"tool": "scan_surroundings"},
    {"tool": "find_object", "args": {"label": "person"}},
    {"tool": "follow_me_start"}
  ]
"""
from __future__ import annotations

import json
import logging
import time
from pathlib import Path

SKILL_ID = "compose"
SKILL_NAME = "Compose"
SKILL_VERSION = "1.0.0"

log = logging.getLogger(__name__)

_WORKFLOWS_FILE = Path.cwd() / ".roborun" / "workflows.json"


def _load_workflows() -> dict[str, list]:
    if _WORKFLOWS_FILE.exists():
        try:
            return json.loads(_WORKFLOWS_FILE.read_text())
        except Exception:
            pass
    return {}


def _save_workflows(workflows: dict) -> None:
    _WORKFLOWS_FILE.parent.mkdir(parents=True, exist_ok=True)
    _WORKFLOWS_FILE.write_text(json.dumps(workflows, indent=2))


def _execute_tool(name: str, args: dict) -> dict:
    from roborun.ros_mcp import handle_tool_call
    return handle_tool_call(name, args)


def _run_sequence(args: dict) -> dict:
    steps = args.get("steps", [])
    if not steps:
        return {"ok": False, "error": "steps required — list of {tool, args} objects"}

    stop_on_error = args.get("stop_on_error", True)
    delay_s = float(args.get("delay_between_s", 0.5))
    results = []

    for i, step in enumerate(steps):
        tool_name = step.get("tool", "")
        tool_args = step.get("args", {})

        if not tool_name:
            results.append({"step": i, "error": "missing tool name"})
            if stop_on_error:
                break
            continue

        result = _execute_tool(tool_name, tool_args)
        results.append({"step": i, "tool": tool_name, "result": result})

        if not result.get("ok", True) and stop_on_error:
            break

        if i < len(steps) - 1 and delay_s > 0:
            time.sleep(delay_s)

    succeeded = sum(1 for r in results if r.get("result", {}).get("ok", False))
    return {
        "ok": succeeded == len(results),
        "steps_run": len(results),
        "steps_total": len(steps),
        "succeeded": succeeded,
        "results": results,
    }


def _save_workflow(args: dict) -> dict:
    name = str(args.get("name", "")).strip()
    steps = args.get("steps", [])
    if not name:
        return {"ok": False, "error": "name required"}
    if not steps:
        return {"ok": False, "error": "steps required"}

    workflows = _load_workflows()
    workflows[name] = {
        "steps": steps,
        "description": args.get("description", ""),
        "created": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    _save_workflows(workflows)
    return {"ok": True, "name": name, "steps": len(steps)}


def _run_workflow(args: dict) -> dict:
    name = str(args.get("name", "")).strip()
    if not name:
        return {"ok": False, "error": "name required"}

    workflows = _load_workflows()
    wf = workflows.get(name)
    if not wf:
        available = list(workflows.keys())
        return {"ok": False, "error": f"Workflow '{name}' not found",
                "available": available}

    return _run_sequence({
        "steps": wf["steps"],
        "stop_on_error": args.get("stop_on_error", True),
        "delay_between_s": args.get("delay_between_s", 0.5),
    })


def _list_workflows(args: dict) -> dict:
    workflows = _load_workflows()
    items = []
    for name, wf in workflows.items():
        items.append({
            "name": name,
            "description": wf.get("description", ""),
            "steps": len(wf.get("steps", [])),
            "created": wf.get("created", ""),
        })
    return {"ok": True, "workflows": items, "total": len(items)}


def _delete_workflow(args: dict) -> dict:
    name = str(args.get("name", "")).strip()
    workflows = _load_workflows()
    if name in workflows:
        del workflows[name]
        _save_workflows(workflows)
        return {"ok": True, "deleted": name}
    return {"ok": False, "error": f"Workflow '{name}' not found"}


def register(registry) -> None:
    registry.add_tool(
        name="run_sequence",
        description=(
            "Execute a sequence of tools in order. Each step is {tool, args}. "
            "Results from each step are collected. Stops on first error by default. "
            "Example: [{\"tool\": \"scan_surroundings\"}, {\"tool\": \"find_object\", \"args\": {\"label\": \"cup\"}}]"
        ),
        input_schema={
            "type": "object",
            "properties": {
                "steps": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "tool": {"type": "string", "description": "Tool name to call"},
                            "args": {"type": "object", "description": "Tool arguments"},
                        },
                        "required": ["tool"],
                    },
                    "description": "Ordered list of tool calls",
                },
                "stop_on_error": {"type": "boolean", "description": "Stop on first failure (default true)"},
                "delay_between_s": {"type": "number", "description": "Delay between steps in seconds (default 0.5)"},
            },
            "required": ["steps"],
        },
        handler=_run_sequence,
        skill_id=SKILL_ID,
    )
    registry.add_tool(
        name="save_workflow",
        description="Save a sequence of steps as a named workflow for later replay.",
        input_schema={
            "type": "object",
            "properties": {
                "name": {"type": "string", "description": "Workflow name"},
                "description": {"type": "string", "description": "What this workflow does"},
                "steps": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "tool": {"type": "string"},
                            "args": {"type": "object"},
                        },
                        "required": ["tool"],
                    },
                },
            },
            "required": ["name", "steps"],
        },
        handler=_save_workflow,
        skill_id=SKILL_ID,
    )
    registry.add_tool(
        name="run_workflow",
        description="Run a previously saved workflow by name.",
        input_schema={
            "type": "object",
            "properties": {
                "name": {"type": "string", "description": "Workflow name"},
                "stop_on_error": {"type": "boolean"},
                "delay_between_s": {"type": "number"},
            },
            "required": ["name"],
        },
        handler=_run_workflow,
        skill_id=SKILL_ID,
    )
    registry.add_tool(
        name="list_workflows",
        description="List all saved workflows.",
        input_schema={"type": "object", "properties": {}},
        handler=_list_workflows,
        skill_id=SKILL_ID,
    )
    registry.add_tool(
        name="delete_workflow",
        description="Delete a saved workflow.",
        input_schema={
            "type": "object",
            "properties": {
                "name": {"type": "string", "description": "Workflow name to delete"},
            },
            "required": ["name"],
        },
        handler=_delete_workflow,
        skill_id=SKILL_ID,
    )
