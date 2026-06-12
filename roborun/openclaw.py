"""OpenClaw bridge: the robot's reach-a-human channel.

MCP is the command channel — Claude (or any MCP client) connects to /mcp
and drives the robot. But MCP is pull-shaped: nothing on that channel can
ring your phone while the robot patrols and your laptop sleeps. This module
is the push half. It subscribes to the event bus and forwards "notify"
events to an OpenClaw gateway (https://openclaw.ai), whose assistant relays
them to WhatsApp, Telegram, or whatever channel it is wired to.

Only "notify" events cross — robot.notify() in behaviors and the `notify`
MCP tool emit them. The filtering judgment ("is this worth a human's
attention?") belongs in the behavior, where the author has state and
context, not in a chat session sifting raw events. Notifies are never
throttled here for the same reason: the behavior already decided.

Env:
  OPENCLAW_HOOKS_URL  e.g. http://127.0.0.1:18789/hooks (unset = off)
  OPENCLAW_TOKEN      hooks.token from the gateway config
  OPENCLAW_CHANNEL    delivery channel (whatsapp, telegram, ...)
  OPENCLAW_TO         delivery address; gateway's default peer when unset
"""
from __future__ import annotations

import json
import os
import threading
import urllib.request

from roborun import events

_started = False
_ok = True  # last delivery state — failures are logged edge-triggered


def configured() -> bool:
    return bool(os.environ.get("OPENCLAW_HOOKS_URL"))


def _post(path: str, payload: dict) -> bool:
    url = os.environ["OPENCLAW_HOOKS_URL"].rstrip("/") + path
    headers = {"Content-Type": "application/json"}
    token = os.environ.get("OPENCLAW_TOKEN", "")
    if token:
        headers["Authorization"] = f"Bearer {token}"
    req = urllib.request.Request(url, data=json.dumps(payload).encode(),
                                 headers=headers, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            return 200 <= resp.status < 300
    except Exception:
        return False


def agent_payload(event: dict) -> dict:
    """An isolated agent turn, delivered to the configured chat channel."""
    detail = {k: v for k, v in (event.get("detail") or {}).items()
              if k != "frame"}  # frames are big and binary; link, don't inline
    message = (f"RoboRun notification from {event.get('source', '?')!r}: "
               f"{event.get('title', '')}")
    if detail:
        message += f"\nDetails: {json.dumps(detail, default=str)}"
    message += ("\nRelay this to the user as one short message. "
                "Do not ask questions back.")
    payload: dict = {"message": message, "name": "roborun", "deliver": True}
    if os.environ.get("OPENCLAW_CHANNEL"):
        payload["channel"] = os.environ["OPENCLAW_CHANNEL"]
    if os.environ.get("OPENCLAW_TO"):
        payload["to"] = os.environ["OPENCLAW_TO"]
    return payload


def handle_event(event: dict) -> bool:
    """Route one event. Returns True if it was posted to the gateway."""
    if event.get("type") != "notify" or event.get("source") == "openclaw":
        return False
    return _send("/agent", agent_payload(event))


def _send(path: str, payload: dict) -> bool:
    global _ok
    ok = _post(path, payload)
    if ok != _ok:
        events.emit("system", "openclaw",
                    "delivery restored" if ok else f"delivery failed: POST {path}",
                    {})
    _ok = ok
    return ok


def _run() -> None:
    q = events.subscribe()
    while True:
        handle_event(q.get())


def start_bridge() -> bool:
    """Start the forwarding thread when a gateway is configured. Idempotent."""
    global _started
    if not configured() or _started:
        return _started
    _started = True
    threading.Thread(target=_run, daemon=True, name="OpenClawBridge").start()
    events.emit("system", "openclaw", "bridge up — notify() reaches the assistant",
                {"url": os.environ["OPENCLAW_HOOKS_URL"]})
    return True
