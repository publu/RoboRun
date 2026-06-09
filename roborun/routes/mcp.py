"""MCP server routes — unified HTTP + SSE transport.

Replaces the old duplicated tool definitions. All tools come from
ros_mcp.get_all_tools() which includes skills.
"""
from __future__ import annotations

import base64
import json
import os
import threading
import time
import uuid
from pathlib import Path
from typing import Any

from roborun.routes import get, post, send_json

PORT = int(os.environ.get("ROBORUN_PORT", "8765"))

_CAMERA_FRAME_PATH = Path("/tmp/go2_camera_frame.jpg")
_HACKATHON_FRAME_PATH = Path("/tmp/go2_hackathon_frame.jpg")
_WEBCAM_FRAME_PATH = Path("/tmp/roborun_frame.jpg")


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


def _camera_content() -> list[dict]:
    for p in (_HACKATHON_FRAME_PATH, _WEBCAM_FRAME_PATH, _CAMERA_FRAME_PATH):
        if p.exists() and (time.time() - p.stat().st_mtime) < 5.0:
            try:
                data = base64.b64encode(p.read_bytes()).decode()
                return [{"type": "image", "data": data, "mimeType": "image/jpeg"}]
            except Exception:
                pass
    return [{"type": "text", "text": "No camera frame available"}]


def handle_mcp_request(h, payload: dict) -> None:
    req_id = payload.get("id")
    method = payload.get("method", "")
    params = payload.get("params", {})

    if method == "initialize":
        _mcp_reply(h, req_id, {
            "protocolVersion": "2024-11-05",
            "capabilities": {"tools": {}},
            "serverInfo": {"name": "roborun", "version": "0.7.0"},
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
        from roborun.ros_mcp import get_mcp_manifest
        manifest = get_mcp_manifest()
        _mcp_reply(h, req_id, {"resources": [{
            "uri": "roborun://server-info",
            "name": "RoboRun Server Info",
            "description": manifest["description"],
            "mimeType": "application/json",
        }]})
        return

    if method == "resources/read":
        uri = params.get("uri", "")
        if uri == "roborun://server-info":
            from roborun.ros_mcp import get_mcp_manifest
            _mcp_reply(h, req_id, {"contents": [{
                "uri": uri, "mimeType": "application/json",
                "text": json.dumps(get_mcp_manifest(), default=str),
            }]})
            return
        _mcp_error(h, req_id, -32602, f"Unknown resource: {uri}")
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
