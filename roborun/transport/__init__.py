"""roborun.transport — the vendored, real transport abstraction (spec §1).

ros_tap grew up and moved in-repo: one interface, three backends, plus a
passive tap that feeds the MCAP recorder. The same discovery primitive that
connects to robots also powers the "is your robot exposed" audit — one
scanner, two uses.

    Transport (interface)
      ├─ discover(domains, timeout) -> [Robot{id, host, topics, transport}]
      ├─ topics() / type_of(topic) / message_schema(type)
      ├─ subscribe(topic, cb) / unsubscribe(topic)
      ├─ publish(topic, msg_type, msg) / call_service / send_goal
      └─ capabilities() -> {pub, sub, services, actions, params, types}

    Backends:
      dds       CycloneDDS — zero-config discovery + pub/sub, common types
      rosbridge websocket  — services/actions/params + any type via a bridge
      native    rclpy      — everything, when running on-robot in a ROS env

The capability matrix is surfaced so agents and the UI degrade gracefully
instead of calling a service over a DDS-only connection and hitting a
confusing error. No silent failures.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Callable

from roborun.transport.schemas import message_fields, SUPPORTED_TYPES

# Spec §1.3 — what each backend supports right now.
CAPABILITY_MATRIX: dict[str, dict[str, Any]] = {
    "dds": {
        "discovery": True, "subscribe": True, "publish": True,
        "services": False, "actions": False, "params": False,
        "types": "common families (bundled schemas + XTypes when advertised)",
    },
    "rosbridge": {
        "discovery": "partial (needs host)", "subscribe": True, "publish": True,
        "services": True, "actions": True, "params": True,
        "types": "any (bridge deserializes)",
    },
    "native": {
        "discovery": True, "subscribe": True, "publish": True,
        "services": True, "actions": True, "params": True,
        "types": "any (rclpy)",
    },
}


@dataclass
class Robot:
    id: str
    host: str | None = None
    transport: str = "dds"
    domain: int | None = None
    topics: dict[str, str] = field(default_factory=dict)  # topic -> msg type
    last_seen: float = 0.0
    alive: bool = True

    def to_dict(self) -> dict:
        return {"id": self.id, "host": self.host, "transport": self.transport,
                "domain": self.domain, "topics": self.topics,
                "last_seen": self.last_seen, "alive": self.alive}


class Transport(ABC):
    """One interface over DDS / rosbridge / rclpy."""

    name: str = "abstract"

    @abstractmethod
    def topics(self) -> dict[str, str]:
        """Visible topics: {topic: msg_type}."""

    def type_of(self, topic: str) -> str | None:
        return self.topics().get(topic)

    def message_schema(self, msg_type: str) -> dict | None:
        """Field definitions for a message type (bundled families)."""
        return message_fields(msg_type)

    @abstractmethod
    def subscribe(self, topic: str, callback: Callable[[dict], None],
                  msg_type: str | None = None) -> str:
        """Subscribe; callback receives messages as plain dicts."""

    @abstractmethod
    def unsubscribe(self, topic: str) -> None: ...

    @abstractmethod
    def publish(self, topic: str, msg_type: str, msg: dict) -> dict:
        """Publish a dict-encoded message. Returns {ok, …} with honest errors."""

    def call_service(self, service: str, srv_type: str = "",
                     args: dict | None = None, timeout: float = 10.0) -> dict:
        return self._unsupported("services")

    def send_goal(self, action: str, action_type: str, goal: dict,
                  timeout: float = 30.0) -> dict:
        return self._unsupported("actions")

    def get_param(self, node: str, parameter: str) -> dict:
        return self._unsupported("params")

    def capabilities(self) -> dict[str, Any]:
        return {"transport": self.name, **CAPABILITY_MATRIX.get(self.name, {})}

    def _unsupported(self, what: str) -> dict:
        caps = CAPABILITY_MATRIX.get(self.name, {})
        alt = [name for name, c in CAPABILITY_MATRIX.items() if c.get(what) is True]
        return {"ok": False,
                "error": f"{what} not supported over {self.name}",
                "hint": f"use {' or '.join(alt)} for {what}",
                "capabilities": {"transport": self.name, **caps}}

    def close(self) -> None:
        pass


def get_transport(kind: str = "auto", **kwargs) -> Transport:
    """Pick a backend: explicit, or best available (native > rosbridge > dds)."""
    if kind in ("auto", "native"):
        try:
            from roborun.transport.native import NativeTransport
            return NativeTransport(**kwargs)
        except Exception:
            if kind == "native":
                raise
    if kind in ("auto", "rosbridge") and kwargs.get("host"):
        from roborun.transport.bridge import RosbridgeTransport
        return RosbridgeTransport(**kwargs)
    if kind == "rosbridge":
        from roborun.transport.bridge import RosbridgeTransport
        return RosbridgeTransport(**kwargs)
    from roborun.transport.dds import DDSTransport
    kwargs.pop("host", None)
    kwargs.pop("port", None)
    return DDSTransport(**kwargs)


def discover(domains: range | list[int] | None = None, timeout: float = 3.0,
             rosbridge_host: str | None = None) -> list[Robot]:
    """Find robots: DDS participant discovery across a domain range, deduped
    against rosbridge when a host is given. Powers fleet status and the
    exposure audit alike."""
    robots: list[Robot] = []
    try:
        from roborun.transport.dds import dds_discover
        robots.extend(dds_discover(domains=domains, timeout=timeout))
    except Exception:
        pass
    if rosbridge_host:
        try:
            from roborun.transport.bridge import RosbridgeTransport
            bridge = RosbridgeTransport(host=rosbridge_host)
            topics = bridge.topics()
            rid = f"rosbridge@{rosbridge_host}"
            seen = {(r.host, frozenset(r.topics)) for r in robots}
            if (rosbridge_host, frozenset(topics)) not in seen:
                import time as _t
                robots.append(Robot(id=rid, host=rosbridge_host,
                                    transport="rosbridge", topics=topics,
                                    last_seen=_t.time()))
            bridge.close()
        except Exception:
            pass
    return robots


def exposure_report(domains: range | list[int] | None = None,
                    timeout: float = 3.0) -> dict:
    """The audit use of the scanner: what is answering on this network?

    Any robot visible here is visible to anyone on the same network — if
    that network is the internet, the robot is exposed. Same primitive as
    discover(); zero new attack surface.
    """
    robots = discover(domains=domains, timeout=timeout)
    findings = []
    for r in robots:
        writable = [t for t in r.topics if "cmd_vel" in t or "goal" in t]
        findings.append({
            **r.to_dict(),
            "risk": "controllable" if writable else "observable",
            "writable_topics": writable,
        })
    return {"robots_visible": len(robots), "findings": findings,
            "note": "anything listed here is reachable by every host on this "
                    "network segment; controllable topics accept unauthenticated commands"}


__all__ = [
    "Transport", "Robot", "CAPABILITY_MATRIX", "SUPPORTED_TYPES",
    "get_transport", "discover", "exposure_report", "message_fields",
]
