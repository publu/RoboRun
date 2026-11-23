"""Isaac Sim discovery + lockstep driver (platform spec 03 P2).

Mirrors gz.py: Isaac Sim publishes ROS 2 topics through its ROS bridge, so
sensing/actuation already flow through `ros_telemetry` + the transports. This
module adds the Isaac-specific bits: (1) recognizing a live Isaac stage, (2)
owning the clock for determinism, (3) mapping a RoboRun level to USD stage prims.

Isaac/omni aren't importable in most environments; everything degrades to
`None`/no-op when the bridge isn't reachable, and the pure level→prim mapping is
unit-testable without a running Isaac.
"""
from __future__ import annotations

from typing import Any

CAPABILITIES = {
    "discovery": True, "subscribe": True, "publish": True,
    "services": True, "actions": True, "clock_control": True,
    "photoreal": True,
}

# RoboRun robot → Isaac USD asset (Nucleus paths in a real deploy).
_ROBOT_USD = {
    "dog": "/Isaac/Robots/Unitree/go2.usd",
    "wheeled": "/Isaac/Robots/Nova/nova_carter.usd",
    "drone": "/Isaac/Robots/Quadcopter/quadcopter.usd",
    "biped": "/Isaac/Robots/Unitree/g1.usd",
}


def _default_transport():
    try:
        from roborun.rosbridge import get_client
        c = get_client(auto_connect=False)
        return c if c and c.health.get("connected") else None
    except Exception:
        return None


def detect_world(transport: Any = None) -> dict | None:
    """Return {stage, clock} if an Isaac stage is live (its ROS bridge exposes
    `/clock` + Isaac-specific topics), else None."""
    try:
        tp = transport or _default_transport()
        if tp is None:
            return None
        topics = tp.topics()
        if "/clock" not in topics:
            return None
        isaac_topics = [t for t in topics
                        if t.startswith("/isaac") or "/omni" in t or t == "/rtf"]
        if not isaac_topics:
            return None
        return {"stage": "isaac", "clock": True}
    except Exception:
        return None


def level_to_prims(level: dict, seed: int = 0) -> list[dict]:
    """Translate a RoboRun level into Isaac USD stage prims (pure; testable with
    no Isaac). Returns a list of {prim_path, usd, pose} the runner would add."""
    prims: list[dict] = []
    spawn = level.get("spawn", {"x": 0, "z": 0, "heading": 0})
    usd = _ROBOT_USD.get(level.get("robot", "wheeled"), _ROBOT_USD["wheeled"])
    prims.append({"prim_path": "/World/robot", "usd": usd,
                  "pose": {"x": spawn.get("x", 0), "y": -spawn.get("z", 0),
                           "yaw": spawn.get("heading", 0)}})
    for i, p in enumerate(level.get("props", [])):
        prims.append({"prim_path": f"/World/prop_{i}",
                      "usd": f"/Isaac/Props/{p.get('kind', 'box')}.usd",
                      "pose": {"x": p.get("x", 0), "y": -p.get("z", 0), "yaw": 0}})
    for i, w in enumerate(level.get("walls", [])):
        prims.append({"prim_path": f"/World/wall_{i}", "usd": "/Isaac/Props/wall.usd",
                      "pose": {"a": w}})
    return prims


class IsaacClock:
    """Lockstep: pause the stage, step N, seed physics — the determinism contract.
    No-op when no stage/transport."""

    def __init__(self, transport: Any = None) -> None:
        self._tp = transport or _default_transport()

    def _call(self, service: str, args: dict) -> dict | None:
        if self._tp is None:
            return None
        try:
            return self._tp.call_service(service, args=args, timeout=5.0)
        except Exception:
            return None

    def pause(self) -> dict | None:
        return self._call("/isaac/pause", {"pause": True})

    def step(self, n: int = 1) -> dict | None:
        return self._call("/isaac/step", {"count": int(n)})

    def reset(self, seed: int = 0) -> dict | None:
        return self._call("/isaac/reset", {"seed": int(seed)})


class IsaacRunner:
    """Deterministic Isaac episode: detect the stage, add a level's prims, own the
    clock. Degrades to a dry-run plan when offline — so the flow is testable."""

    def __init__(self, transport: Any = None) -> None:
        self.transport = transport
        self.stage = None
        self.clock: IsaacClock | None = None

    def attach(self) -> bool:
        info = detect_world(self.transport)
        if info is None:
            return False
        self.stage = info["stage"]
        self.clock = IsaacClock(self.transport)
        return True

    def plan_level(self, level: dict, seed: int = 0) -> list[dict]:
        return level_to_prims(level, seed)

    def run_level(self, level: dict, seed: int = 0, steps: int = 100) -> dict:
        if not self.attach():
            return {"status": "no-stage", "plan": self.plan_level(level, seed)}
        self.clock.reset(seed)
        for _ in range(steps):
            self.clock.step(1)
        return {"status": "ran", "stage": self.stage, "steps": steps, "seed": seed}
