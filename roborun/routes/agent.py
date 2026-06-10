"""Agent chat routes — the flight deck command bar, SSE streaming."""
from __future__ import annotations

import json

from roborun.routes import get, post, send_json
from roborun.routes._singletons import get_agent


@get("/api/agent/status")
def status(h):
    agent = get_agent()
    if agent == "unavailable":
        send_json(h, 200, {"ok": True, "alive": False, "available": False})
    else:
        send_json(h, 200, {"ok": True, "alive": agent.is_alive, "available": True})


def _sse_stream(h, agent, message: str) -> None:
    h.send_response(200)
    h.send_header("Content-Type", "text/event-stream")
    h.send_header("Cache-Control", "no-cache")
    h.send_header("X-Accel-Buffering", "no")
    h.end_headers()

    def sse(data: dict) -> None:
        h.wfile.write(f"data: {json.dumps(data)}\n\n".encode())
        h.wfile.flush()

    try:
        for chunk in agent.send(message):
            sse(chunk)
            if chunk.get("type") in ("done", "error"):
                break
    except Exception as exc:
        try:
            sse({"type": "error", "error": str(exc)})
        except Exception:
            pass


@post("/api/agent/chat")
def chat(h, payload):
    agent = get_agent()
    if agent == "unavailable":
        send_json(h, 200, {"ok": False,
                           "error": "in-process agent needs ANTHROPIC_API_KEY "
                                    "(or drive the robot over MCP)"})
        return
    message = str(payload.get("message", "")).strip()
    if not message:
        send_json(h, 400, {"ok": False, "error": "message required"})
        return
    from roborun.events import emit
    emit("task", "operator", message, {})
    _sse_stream(h, agent, message)


@post("/api/agent/stop")
def stop(h, payload):
    agent = get_agent()
    if agent != "unavailable":
        agent.stop()
    send_json(h, 200, {"ok": True})


@post("/api/agent/clear")
def clear(h, payload):
    agent = get_agent()
    if agent != "unavailable":
        agent.clear_session()
    send_json(h, 200, {"ok": True})
