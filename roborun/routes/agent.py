"""Agent chat routes — Claude + Gemini SSE streaming."""
from __future__ import annotations

import json
import os

from roborun.routes import get, post, send_json
from roborun.routes._singletons import get_agent, get_gemini_agent


@get("/api/agent/status")
def status(h):
    agent = get_agent()
    if agent == "unavailable":
        send_json(h, 200, {"ok": True, "alive": False, "available": False, "mode": None})
    else:
        from roborun.agent import FastRobotAgent
        mode = "fast" if isinstance(agent, FastRobotAgent) else "subprocess"
        session = getattr(agent, "_session_id", None)
        send_json(h, 200, {"ok": True, "alive": agent.is_alive,
                            "available": True, "mode": mode,
                            "session": session is not None})


@get("/api/agent/gemini/status")
def gemini_status(h):
    has_key = bool(os.environ.get("GEMINI_API_KEY"))
    agent = get_gemini_agent()
    send_json(h, 200, {"ok": True, "available": agent != "unavailable",
                        "alive": agent != "unavailable" and agent.is_alive,
                        "has_key": has_key})


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
        send_json(h, 200, {"ok": False, "error": "Agent not available (claude CLI not found)"})
        return
    message = str(payload.get("message", "")).strip()
    if not message:
        send_json(h, 400, {"ok": False, "error": "message required"})
        return
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


@post("/api/agent/gemini")
def gemini_chat(h, payload):
    agent = get_gemini_agent()
    if agent == "unavailable":
        send_json(h, 200, {"ok": False, "error": "Gemini agent not available — set GEMINI_API_KEY"})
        return
    message = str(payload.get("message", "")).strip()
    if not message:
        send_json(h, 400, {"ok": False, "error": "message required"})
        return
    _sse_stream(h, agent, message)


@post("/api/agent/gemini/clear")
def gemini_clear(h, payload):
    agent = get_gemini_agent()
    if agent != "unavailable":
        agent.clear_session()
    send_json(h, 200, {"ok": True})
