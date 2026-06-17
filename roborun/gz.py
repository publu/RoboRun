"""gz-sim discovery + lockstep driver (GAZEBO_RUNNER_SPEC).

gz-sim publishes standard ROS 2 topics, so sensing/actuation already flow through
`ros_telemetry` + the existing transports — this module adds only the three things
gz needs that a real robot doesn't: (1) recognizing a live gz world, (2) owning the
clock for determinism, (3) spawning a RoboRun level as SDF entities.

ROS 2 / gz aren't importable in every environment; everything here degrades to
`None`/no-op when the graph isn't reachable, and the pure level→entity mapping is
unit-testable without a running world.
"""
from __future__ import annotations

from typing import Any

# Capability row the agent/UI reads (mirrors transport.CAPABILITY_MATRIX shape).
CAPABILITIES = {
    "discovery": True, "subscribe": True, "publish": True,
    "services": True, "actions": True, "clock_control": True,
}


def detect_world(transport: Any = None) -> dict | None:
    """Return {world, sim_time} if a gz world is up (a `/clock` + `/world/*`
    surface), else None. Uses whatever transport is connected."""
    try:
        tp = transport or _default_transport()
        if tp is None:
            return None
        topics = tp.topics()
        if "/clock" not in topics:
            return None
        worlds = [t for t in topics if t.startswith("/world/")]
        name = worlds[0].split("/")[2] if worlds else "default"
        return {"world": name, "clock": True}
    except Exception:
        return None


def _default_transport():
    try:
        from roborun.rosbridge import get_client
        c = get_client(auto_connect=False)
        return c if c and c.health.get("connected") else None
    except Exception:
        return None


# ── level → SDF entities (pure mapping; testable with no world) ─────────────

def level_to_spawns(level: dict, seed: int = 0) -> list[dict]:
    """Translate a RoboRun level (walls/props/robot/zones) into gz SpawnEntity
    requests. Deterministic given `seed`. Returns a list of
    {service, name, type, pose} dicts the driver POSTs to `/world/<w>/create`."""
    spawns: list[dict] = []
    spawn = level.get("spawn", {"x": 0, "z": 0, "heading": 0})
    robot_model = {"dog": "go2", "wheeled": "turtlebot3", "drone": "x500",
                   "biped": "g1"}.get(level.get("robot", "wheeled"), "turtlebot3")
    spawns.append({"service": "create", "name": "robot", "type": robot_model,
                   "pose": {"x": spawn.get("x", 0), "y": -spawn.get("z", 0),
                            "yaw": spawn.get("heading", 0)}})
    for i, p in enumerate(level.get("props", [])):
        spawns.append({"service": "create", "name": f"prop_{i}",
                       "type": p.get("kind", "box"),
                       "pose": {"x": p.get("x", 0), "y": -p.get("z", 0), "yaw": 0}})
    for i, w in enumerate(level.get("walls", [])):
        spawns.append({"service": "create", "name": f"wall_{i}", "type": "wall",
                       "pose": {"a": w}})
    return spawns


class GzClock:
    """Lockstep clock: pause the world, step N at a time, seed physics — the
    determinism contract gz lacks by default. No-op when no world/transport."""

    def __init__(self, world: str, transport: Any = None) -> None:
        self.world = world
        self._tp = transport or _default_transport()

    def _call(self, service: str, args: dict) -> dict | None:
        if self._tp is None:
            return None
        try:
            return self._tp.call_service(f"/world/{self.world}/{service}",
                                         args=args, timeout=5.0)
        except Exception:
            return None

    def pause(self) -> dict | None:
        return self._call("control", {"pause": True})

    def step(self, n: int = 1) -> dict | None:
        return self._call("control", {"pause": True, "multi_step": int(n)})

    def reset(self, seed: int = 0) -> dict | None:
        return self._call("control", {"reset": {"all": True}, "seed": int(seed)})
