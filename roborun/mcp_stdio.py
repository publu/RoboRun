"""Stdio MCP transport for Claude Desktop and other MCP clients.

Reads JSON-RPC from stdin, writes to stdout. Wraps the existing
MCP_TOOLS and handle_tool_call from ros_mcp.py.

Usage:
  roborun-mcp                  # run as stdio MCP server
  echo '{"jsonrpc":"2.0","id":1,"method":"tools/list"}' | roborun-mcp

Claude Desktop config:
  {"mcpServers": {"roborun": {"command": "roborun-mcp"}}}
"""

from __future__ import annotations

import json
import sys
from typing import Any

from roborun.ros_mcp import get_all_tools, get_mcp_manifest, handle_tool_call

SERVER_INFO = {
    "name": "roborun-ros",
    "version": "0.7.0",
}

CAPABILITIES = {
    "tools": {},
}


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

    if method == "resources/list":
        manifest = get_mcp_manifest()
        return _result_response(req_id, {"resources": [
            {
                "uri": "roborun://server-info",
                "name": "RoboRun Server Info",
                "description": manifest["description"],
                "mimeType": "application/json",
            },
        ]})

    if method == "resources/read":
        uri = params.get("uri", "")
        if uri == "roborun://server-info":
            return _result_response(req_id, {"contents": [
                {"uri": uri, "mimeType": "application/json",
                 "text": json.dumps(get_mcp_manifest(), default=str)},
            ]})
        return _error_response(req_id, -32602, f"Unknown resource: {uri}")

    return _error_response(req_id, -32601, f"Method not found: {method}")


def main() -> None:
    try:
        from roborun.skills import load_skills
        count = load_skills()
        if count:
            sys.stderr.write(f"Loaded {count} skill(s)\n")
    except Exception:
        pass

    sys.stderr.write(f"roborun-mcp v{SERVER_INFO['version']} — stdio MCP server ready\n")
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
