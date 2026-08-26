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


class GzRunner:
    """Orchestrates a deterministic gz episode: detect the world, spawn a level,
    own the clock. Degrades to a dry-run plan (no calls) when no world/transport,
    so the flow is testable without ROS/gz."""

    def __init__(self, transport: Any = None) -> None:
        self.transport = transport
        self.world = None
        self.clock: GzClock | None = None

    def attach(self) -> bool:
        info = detect_world(self.transport)
        if info is None:
            return False
        self.world = info["world"]
        self.clock = GzClock(self.world, self.transport)
        return True

    def plan_level(self, level: dict, seed: int = 0) -> list[dict]:
        """The spawn plan for a level (pure; what attach() would POST)."""
        return level_to_spawns(level, seed)

    def run_level(self, level: dict, seed: int = 0, steps: int = 100) -> dict:
        """Spawn + step. Returns a report; status 'no-world' when offline."""
        if not self.attach():
            return {"status": "no-world", "plan": self.plan_level(level, seed)}
        self.clock.reset(seed)
        for sp in self.plan_level(level, seed):
            self.transport.call_service(f"/world/{self.world}/{sp['service']}",
                                        args=sp, timeout=5.0)
        for _ in range(steps):
            self.clock.step(1)
        return {"status": "ran", "world": self.world, "steps": steps, "seed": seed}
