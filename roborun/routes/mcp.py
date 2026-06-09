"""MCP server routes — unified HTTP + SSE transport.

Full MCP protocol: tools + resources + prompts. Shared implementation
with mcp_stdio.py — both use the same prompts and resources.
"""
from __future__ import annotations

import json
import os
import time
from typing import Any

from roborun.routes import get, post, send_json

PORT = int(os.environ.get("ROBORUN_PORT", "8765"))


def _mcp_reply(h, req_id: Any, result: Any) -> None:
    body = json.dumps({"jsonrpc": "2.0", "id": req_id, "result": result}).encode()
    h.send_response(200)
    h.send_header("Content-Type", "application/json")
    h.send_header("Content-Length", str(len(body)))
    h.send_header("Access-Control-Allow-Origin", "*")
    h.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
    h.end_headers()
    h.wfile.write(body)


def _mcp_error(h, req_id: Any, code: int, message: str) -> None:
    body = json.dumps({"jsonrpc": "2.0", "id": req_id,
                       "error": {"code": code, "message": message}}).encode()
    h.send_response(200)
    h.send_header("Content-Type", "application/json")
    h.send_header("Content-Length", str(len(body)))
    h.send_header("Access-Control-Allow-Origin", "*")
    h.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
    h.end_headers()
    h.wfile.write(body)


def handle_mcp_request(h, payload: dict) -> None:
    from roborun.mcp_stdio import (
        MCP_PROMPTS, PROMPT_MESSAGES, CAPABILITIES, SERVER_INFO,
        RESOURCE_TEMPLATES, _get_resources, _read_resource,
    )

    req_id = payload.get("id")
    method = payload.get("method", "")
    params = payload.get("params", {})

    if method == "initialize":
        _mcp_reply(h, req_id, {
            "protocolVersion": "2024-11-05",
            "capabilities": CAPABILITIES,
            "serverInfo": SERVER_INFO,
        })
        return

    if method == "notifications/initialized":
        h.send_response(204)
        h.send_header("Access-Control-Allow-Origin", "*")
        h.end_headers()
        return

    if method == "ping":
        _mcp_reply(h, req_id, {})
        return

    if method == "tools/list":
        from roborun.ros_mcp import get_all_tools
        tools = [{"name": t["name"], "description": t["description"],
                  "inputSchema": t["inputSchema"]} for t in get_all_tools()]
        _mcp_reply(h, req_id, {"tools": tools})
        return

    if method == "tools/call":
        name = params.get("name", "")
        args = params.get("arguments", {})
        from roborun.ros_mcp import handle_tool_call
        result = handle_tool_call(name, args)
        is_error = not result.get("ok", True)
        content = [{"type": "text", "text": json.dumps(result, default=str)}]
        if "image" in result and isinstance(result["image"], str) and result["image"].startswith("data:image/"):
            parts = result["image"].split(",", 1)
            if len(parts) == 2:
                mime = parts[0].replace("data:", "").replace(";base64", "")
                content.append({"type": "image", "data": parts[1], "mimeType": mime})
        _mcp_reply(h, req_id, {"content": content, "isError": is_error})
        return

    if method == "resources/list":
        _mcp_reply(h, req_id, {"resources": _get_resources()})
        return

    if method == "resources/templates/list":
        _mcp_reply(h, req_id, {"resourceTemplates": RESOURCE_TEMPLATES})
        return

    if method == "resources/read":
        uri = params.get("uri", "")
        content = _read_resource(uri)
        if content:
            _mcp_reply(h, req_id, {"contents": [content]})
            return
        _mcp_error(h, req_id, -32602, f"Unknown resource: {uri}")
        return

    if method == "prompts/list":
        _mcp_reply(h, req_id, {"prompts": MCP_PROMPTS})
        return

    if method == "prompts/get":
        name = params.get("name", "")
        messages = PROMPT_MESSAGES.get(name)
        if messages is None:
            _mcp_error(h, req_id, -32602, f"Unknown prompt: {name}")
            return
        prompt_args = params.get("arguments", {})
        resolved = []
        for msg in messages:
            text = msg["content"]["text"]
            for key, value in prompt_args.items():
                text = text.replace(f"{{{key}}}", str(value))
            resolved.append({"role": msg["role"], "content": {"type": "text", "text": text}})
        _mcp_reply(h, req_id, {"messages": resolved})
        return

    _mcp_error(h, req_id, -32601, f"Method not found: {method}")


def handle_mcp_sse(h) -> None:
    h.send_response(200)
    h.send_header("Content-Type", "text/event-stream")
    h.send_header("Cache-Control", "no-cache")
    h.send_header("Access-Control-Allow-Origin", "*")
    h.end_headers()
    msg = f'data: {{"type":"endpoint","url":"http://127.0.0.1:{PORT}/mcp"}}\n\n'
    try:
        h.wfile.write(msg.encode())
        h.wfile.flush()
        while True:
            h.wfile.write(b": ping\n\n")
            h.wfile.flush()
            time.sleep(15)
    except Exception:
        pass
