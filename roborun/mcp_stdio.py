"""Stdio MCP transport for Claude Desktop and other MCP clients.

Full MCP 2024-11-05 implementation with:
  - 41+ tools (ROS + skills)
  - Resources (live ROS graph, robot specs, skill catalog)
  - Prompts (guided workflows for common robot tasks)
  - Image auto-extraction from tool results

Usage:
  ros-agent-mcp                  # run as stdio MCP server
  echo '{"jsonrpc":"2.0","id":1,"method":"tools/list"}' | ros-agent-mcp

Claude Desktop config:
  {"mcpServers": {"ros-agent": {"command": "ros-agent-mcp"}}}
"""

from __future__ import annotations

import json
import sys
import time
from typing import Any

from roborun.ros_mcp import get_all_tools, get_mcp_manifest, handle_tool_call

SERVER_INFO = {
    "name": "ros-agent",
    "version": "0.8.0",
}

CAPABILITIES = {
    "tools": {},
    "resources": {},
    "prompts": {},
    "logging": {},
}

RESOURCE_TEMPLATES = [
    {
        "uriTemplate": "ros-agent://topic/{topic_path}",
        "name": "Live ROS Topic",
        "description": "Subscribe once to any ROS topic and read its latest message. Use topic path without leading slash (e.g. cmd_vel, camera/image_raw).",
        "mimeType": "application/json",
    },
]

# ── MCP Prompts ──────────────────────────────────────────────────────────────
# Guided workflows that help LLMs interact with robots effectively.

MCP_PROMPTS = [
    {
        "name": "explore-robot",
        "description": "Discover what's connected and what you can do. Scans for robots, lists topics, identifies the robot type, and shows available skills.",
        "arguments": [],
    },
    {
        "name": "safety-check",
        "description": "Pre-flight safety check before moving a robot. Verifies connection, checks battery, confirms estop works, and validates velocity limits.",
        "arguments": [
            {"name": "robot_ip", "description": "Robot IP address", "required": False},
        ],
    },
    {
        "name": "environment-scan",
        "description": "Survey the robot's surroundings. Does a 360-degree YOLO scan, inventories objects, captures camera frames, and reports what's around.",
        "arguments": [],
    },
    {
        "name": "teach-waypoints",
        "description": "Interactive waypoint teaching. Drive the robot to positions, name them, and save as a patrol route.",
        "arguments": [],
    },
    {
        "name": "debug-topic",
        "description": "Debug a specific ROS topic. Shows type, publishers, subscribers, and samples recent messages.",
        "arguments": [
            {"name": "topic", "description": "Topic name to debug", "required": True},
        ],
    },
    {
        "name": "quick-start",
        "description": "Zero-to-moving in 60 seconds. Connect, verify safety, take a photo, do a test move. Perfect for first-time users.",
        "arguments": [
            {"name": "robot_ip", "description": "Robot IP (default: auto-discover)", "required": False},
        ],
    },
    {
        "name": "fleet-sweep",
        "description": "Multi-robot fleet check. Discover all robots on the network, check each one's status, and report which are ready.",
        "arguments": [],
    },
    {
        "name": "build-workflow",
        "description": "Interactive workflow builder. Describe what you want the robot to do and build a reusable sequence of tool calls step by step.",
        "arguments": [
            {"name": "goal", "description": "What should the workflow accomplish?", "required": True},
        ],
    },
]

PROMPT_MESSAGES = {
    "explore-robot": [
        {"role": "user", "content": {"type": "text", "text": (
            "I just connected to a robot. Help me understand what's available.\n\n"
            "1. Run scan_robots to discover what's on the network\n"
            "2. Use get_robot_info to identify the robot type\n"
            "3. List all topics and categorize them (sensors, actuators, camera, etc.)\n"
            "4. Check which skills are available\n"
            "5. Give me a summary: what this robot is, what it can do, and what tools I should use"
        )}},
    ],
    "safety-check": [
        {"role": "user", "content": {"type": "text", "text": (
            "Run a pre-flight safety check before I move this robot:\n\n"
            "1. Verify rosbridge or DDS connection is live\n"
            "2. Read battery state (subscribe_once to battery topic)\n"
            "3. Test estop — send zero velocity and confirm it works\n"
            "4. Check if /cmd_vel topic exists and has subscribers\n"
            "5. Report: connection status, battery %, estop verified, ready to move?"
        )}},
    ],
    "environment-scan": [
        {"role": "user", "content": {"type": "text", "text": (
            "Survey the environment around the robot:\n\n"
            "1. Take a camera snapshot to see what's in front\n"
            "2. Run detect_now to get current YOLO detections\n"
            "3. Run scan_surroundings to do a full 360° sweep\n"
            "4. Report: what objects are visible, where they are relative to the robot, "
            "and any notable features of the environment"
        )}},
    ],
    "teach-waypoints": [
        {"role": "user", "content": {"type": "text", "text": (
            "Help me teach waypoints for a patrol route:\n\n"
            "1. First clear any existing waypoints with patrol_clear_waypoints\n"
            "2. I'll tell you directions (forward, turn left, etc.) — use move to drive there\n"
            "3. At each stop, add a waypoint with patrol_add_waypoint\n"
            "4. After all waypoints are set, show the full route with patrol_status\n"
            "5. Ask if I want to start the patrol\n\n"
            "Let's start — where should the first waypoint be?"
        )}},
    ],
    "debug-topic": [
        {"role": "user", "content": {"type": "text", "text": (
            "Debug a ROS topic for me:\n\n"
            "1. Get the topic type with get_topic_type\n"
            "2. Get publisher/subscriber details with get_topic_details\n"
            "3. Get the message field definitions with get_message_details\n"
            "4. Subscribe once to see a live message\n"
            "5. Report: is this topic active? What's publishing to it? What does the data look like?"
        )}},
    ],
    "quick-start": [
        {"role": "user", "content": {"type": "text", "text": (
            "Get me from zero to controlling this robot as fast as possible:\n\n"
            "1. Run robot_brief to see what's connected and what we can do\n"
            "2. If no robot found, try scan_robots then connect to the first one\n"
            "3. Take a camera_snapshot so I can see what the robot sees\n"
            "4. Send a tiny test move (0.1 m/s forward for 0.5s) to confirm movement works\n"
            "5. Run estop to stop, then report: what robot, what it can do, ready to go\n\n"
            "Keep it fast — I want to be driving in under a minute."
        )}},
    ],
    "fleet-sweep": [
        {"role": "user", "content": {"type": "text", "text": (
            "Do a full sweep of all robots on my network:\n\n"
            "1. Run scan_robots to discover everything\n"
            "2. For each robot found, try to connect and run robot_brief\n"
            "3. Check battery status on each (subscribe_once to battery topics)\n"
            "4. Build a fleet status table: robot name/IP, type, battery %, topics, ready?\n"
            "5. Recommend which robots are ready for tasking and which need attention"
        )}},
    ],
    "build-workflow": [
        {"role": "user", "content": {"type": "text", "text": (
            "Help me build a reusable workflow for: {goal}\n\n"
            "1. Break the goal into a sequence of tool calls\n"
            "2. For each step, explain what it does and what args it needs\n"
            "3. Test each step individually to make sure it works\n"
            "4. Chain them together with run_sequence to verify the full workflow\n"
            "5. Save it with save_workflow so I can replay it anytime with run_workflow\n\n"
            "Start by listing the steps you think we need."
        )}},
    ],
}

# ── MCP Resources ────────────────────────────────────────────────────────────

def _get_resources() -> list[dict]:
    resources = [
        {
            "uri": "ros-agent://server-info",
            "name": "Server Info",
            "description": "ros-agent server version, capabilities, and total tool count",
            "mimeType": "application/json",
        },
        {
            "uri": "ros-agent://skills",
            "name": "Installed Skills",
            "description": "All loaded skills with their tools and behaviors",
            "mimeType": "application/json",
        },
        {
            "uri": "ros-agent://ros-graph",
            "name": "Live ROS Graph",
            "description": "Current ROS nodes, topics, services — live from DDS/rosbridge",
            "mimeType": "application/json",
        },
        {
            "uri": "ros-agent://workflows",
            "name": "Saved Workflows",
            "description": "All saved tool-chain workflows that can be replayed with run_workflow",
            "mimeType": "application/json",
        },
        {
            "uri": "ros-agent://prompts-catalog",
            "name": "Prompts Catalog",
            "description": "All available guided workflow prompts with descriptions and arguments",
            "mimeType": "application/json",
        },
        {
            "uri": "ros-agent://soul",
            "name": "Agent Identity (SOUL.md)",
            "description": "Behavioral guidelines for the robot agent — safety rules, interaction style, personality",
            "mimeType": "text/markdown",
        },
    ]
    return resources


def _read_resource(uri: str) -> dict | None:
    if uri == "ros-agent://server-info":
        manifest = get_mcp_manifest()
        manifest["version"] = SERVER_INFO["version"]
        return {"uri": uri, "mimeType": "application/json",
                "text": json.dumps(manifest, default=str)}

    if uri == "ros-agent://skills":
        from roborun.skills import get_registry
        reg = get_registry()
        skills = []
        for sid, info in reg.skills.items():
            skills.append({
                "id": info["id"],
                "name": info["name"],
                "version": info["version"],
                "tools": [t["name"] for t in reg.tools.values() if t.get("skill_id") == sid],
                "behaviors": [b["name"] for b in reg.behaviors.values() if b.get("skill_id") == sid],
            })
        data = {"skills": skills, "total_tools": len(reg.tools),
                "total_behaviors": len(reg.behaviors)}
        return {"uri": uri, "mimeType": "application/json",
                "text": json.dumps(data, indent=2)}

    if uri == "ros-agent://ros-graph":
        from roborun.ros_mcp import _discover
        graph = _discover()
        graph.pop("ts", None)
        return {"uri": uri, "mimeType": "application/json",
                "text": json.dumps(graph, indent=2, default=str)}

    if uri == "ros-agent://workflows":
        from roborun.skills.compose import _load_workflows
        workflows = _load_workflows()
        return {"uri": uri, "mimeType": "application/json",
                "text": json.dumps(workflows, indent=2)}

    if uri == "ros-agent://prompts-catalog":
        catalog = []
        for p in MCP_PROMPTS:
            catalog.append({
                "name": p["name"],
                "description": p["description"],
                "arguments": p.get("arguments", []),
            })
        return {"uri": uri, "mimeType": "application/json",
                "text": json.dumps(catalog, indent=2)}

    if uri == "ros-agent://soul":
        from pathlib import Path
        soul_path = Path.cwd() / ".roborun" / "SOUL.md"
        if soul_path.exists():
            return {"uri": uri, "mimeType": "text/markdown",
                    "text": soul_path.read_text()}
        return {"uri": uri, "mimeType": "text/markdown",
                "text": "# No SOUL.md found\nCreate .roborun/SOUL.md to define agent behavioral guidelines."}

    if uri.startswith("ros-agent://topic/"):
        topic_name = "/" + uri[len("ros-agent://topic/"):]
        result = handle_tool_call("subscribe_once", {"topic": topic_name, "timeout_ms": 3000})
        return {"uri": uri, "mimeType": "application/json",
                "text": json.dumps(result, default=str, indent=2)}

    return None


# ── JSON-RPC transport ───────────────────────────────────────────────────────

def _write(msg: dict) -> None:
    line = json.dumps(msg, separators=(",", ":"))
    sys.stdout.write(line + "\n")
    sys.stdout.flush()


_log_level = "info"

def _emit_log(level: str, data: Any, logger: str = "ros-agent") -> None:
    levels = ["debug", "info", "notice", "warning", "error", "critical", "alert", "emergency"]
    if levels.index(level) < levels.index(_log_level):
        return
    _write({
        "jsonrpc": "2.0",
        "method": "notifications/message",
        "params": {"level": level, "logger": logger, "data": data},
    })


def _error_response(req_id: Any, code: int, message: str) -> dict:
    return {"jsonrpc": "2.0", "id": req_id, "error": {"code": code, "message": message}}


def _result_response(req_id: Any, result: Any) -> dict:
    return {"jsonrpc": "2.0", "id": req_id, "result": result}


def _handle_request(method: str, params: dict | None, req_id: Any) -> dict | None:
    params = params or {}

    if method == "initialize":
        return _result_response(req_id, {
            "protocolVersion": "2024-11-05",
            "serverInfo": SERVER_INFO,
            "capabilities": CAPABILITIES,
        })

    if method == "notifications/initialized":
        return None

    if method == "ping":
        return _result_response(req_id, {})

    if method == "logging/setLevel":
        global _log_level
        _log_level = params.get("level", "info")
        return _result_response(req_id, {})

    # ── Tools ────────────────────────────────────────────────────────────────

    if method == "tools/list":
        tools = []
        for t in get_all_tools():
            tools.append({
                "name": t["name"],
                "description": t["description"],
                "inputSchema": t["inputSchema"],
            })
        return _result_response(req_id, {"tools": tools})

    if method == "tools/call":
        tool_name = params.get("name", "")
        arguments = params.get("arguments", {})
        _emit_log("info", f"Calling tool: {tool_name}", "ros-agent.tools")
        result = handle_tool_call(tool_name, arguments)

        is_error = not result.get("ok", True)
        if is_error:
            _emit_log("warning", f"Tool {tool_name} failed: {result.get('error', 'unknown')}", "ros-agent.tools")
        content = [{"type": "text", "text": json.dumps(result, default=str)}]

        if "image" in result and isinstance(result["image"], str) and result["image"].startswith("data:image/"):
            parts = result["image"].split(",", 1)
            if len(parts) == 2:
                mime = parts[0].replace("data:", "").replace(";base64", "")
                content.append({
                    "type": "image",
                    "data": parts[1],
                    "mimeType": mime,
                })

        return _result_response(req_id, {"content": content, "isError": is_error})

    # ── Resources ────────────────────────────────────────────────────────────

    if method == "resources/list":
        return _result_response(req_id, {"resources": _get_resources()})

    if method == "resources/templates/list":
        return _result_response(req_id, {"resourceTemplates": RESOURCE_TEMPLATES})

    if method == "resources/read":
        uri = params.get("uri", "")
        content = _read_resource(uri)
        if content:
            return _result_response(req_id, {"contents": [content]})
        return _error_response(req_id, -32602, f"Unknown resource: {uri}")

    # ── Prompts ──────────────────────────────────────────────────────────────

    if method == "prompts/list":
        return _result_response(req_id, {"prompts": MCP_PROMPTS})

    if method == "prompts/get":
        name = params.get("name", "")
        messages = PROMPT_MESSAGES.get(name)
        if messages is None:
            return _error_response(req_id, -32602, f"Unknown prompt: {name}")

        # Inject any provided arguments into the message text
        prompt_args = params.get("arguments", {})
        resolved = []
        for msg in messages:
            content = msg["content"]
            text = content["text"]
            for key, value in prompt_args.items():
                text = text.replace(f"{{{key}}}", str(value))
            resolved.append({"role": msg["role"], "content": {"type": "text", "text": text}})

        return _result_response(req_id, {"messages": resolved})

    return _error_response(req_id, -32601, f"Method not found: {method}")


def main() -> None:
    try:
        from roborun.skills import load_skills
        count = load_skills()
        if count:
            sys.stderr.write(f"Loaded {count} skill(s)\n")
    except Exception:
        pass

    tools = get_all_tools()
    sys.stderr.write(f"ros-agent-mcp v{SERVER_INFO['version']} — "
                     f"{len(tools)} tools, {len(MCP_PROMPTS)} prompts, "
                     f"{len(_get_resources())} resources\n")
    sys.stderr.flush()

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue

        try:
            msg = json.loads(line)
        except json.JSONDecodeError as e:
            _write(_error_response(None, -32700, f"Parse error: {e}"))
            continue

        if not isinstance(msg, dict) or msg.get("jsonrpc") != "2.0":
            _write(_error_response(msg.get("id") if isinstance(msg, dict) else None,
                                   -32600, "Invalid JSON-RPC request"))
            continue

        method = msg.get("method", "")
        params = msg.get("params")
        req_id = msg.get("id")

        response = _handle_request(method, params, req_id)
        if response is not None:
            _write(response)


if __name__ == "__main__":
    main()
