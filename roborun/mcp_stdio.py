"""Stdio MCP transport for Claude Desktop and other MCP clients.

Full MCP 2024-11-05 implementation with:
  - 41+ tools (ROS + skills)
  - Resources (live ROS graph, robot specs, skill catalog)
  - Prompts (guided workflows for common robot tasks)
  - Image auto-extraction from tool results

Usage:
  roborun-mcp                  # run as stdio MCP server
  echo '{"jsonrpc":"2.0","id":1,"method":"tools/list"}' | roborun-mcp

Claude Desktop config:
  {"mcpServers": {"roborun": {"command": "roborun-mcp"}}}
"""

from __future__ import annotations

import json
import sys
import time
from typing import Any

from roborun.ros_mcp import get_all_tools, get_mcp_manifest, handle_tool_call

SERVER_INFO = {
    "name": "roborun",
    "version": "0.8.0",
}

CAPABILITIES = {
    "tools": {},
    "resources": {},
    "prompts": {},
}

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
}

# ── MCP Resources ────────────────────────────────────────────────────────────

def _get_resources() -> list[dict]:
    resources = [
        {
            "uri": "roborun://server-info",
            "name": "Server Info",
            "description": "RoboRun server version, capabilities, and total tool count",
            "mimeType": "application/json",
        },
        {
            "uri": "roborun://skills",
            "name": "Installed Skills",
            "description": "All loaded skills with their tools and behaviors",
            "mimeType": "application/json",
        },
        {
            "uri": "roborun://ros-graph",
            "name": "Live ROS Graph",
            "description": "Current ROS nodes, topics, services — live from DDS/rosbridge",
            "mimeType": "application/json",
        },
    ]
    return resources


def _read_resource(uri: str) -> dict | None:
    if uri == "roborun://server-info":
        manifest = get_mcp_manifest()
        manifest["version"] = SERVER_INFO["version"]
        return {"uri": uri, "mimeType": "application/json",
                "text": json.dumps(manifest, default=str)}

    if uri == "roborun://skills":
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

    if uri == "roborun://ros-graph":
        from roborun.ros_mcp import _discover
        graph = _discover()
        graph.pop("ts", None)
        return {"uri": uri, "mimeType": "application/json",
                "text": json.dumps(graph, indent=2, default=str)}

    return None


# ── JSON-RPC transport ───────────────────────────────────────────────────────

def _write(msg: dict) -> None:
    line = json.dumps(msg, separators=(",", ":"))
    sys.stdout.write(line + "\n")
    sys.stdout.flush()


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
        result = handle_tool_call(tool_name, arguments)

        is_error = not result.get("ok", True)
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
    sys.stderr.write(f"roborun-mcp v{SERVER_INFO['version']} — "
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
