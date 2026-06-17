"""Backend registry + capability matrix (platform spec 03).

One declarative place that answers "which embodiments can I run, and what can
each do" — rapier / mujoco / mjx / gazebo / isaac / real. The UI reads this to
offer/grey-out backends when binding an environment (spec 08). The runtime
`Backend` protocol is the contract every backend implements; `SimBackend`
(`sim_backend.py`) is the reference impl.
"""
from __future__ import annotations

from typing import Any, Protocol, runtime_checkable


@runtime_checkable
class Backend(Protocol):
    """The uniform handle every backend exposes (spec 03 P1)."""
    def capabilities(self) -> dict[str, Any]: ...
    def reset(self, seed: int | None = None) -> None: ...
    def pose(self) -> dict[str, float] | None: ...
    def lidar(self, max_range: float = 10.0) -> list[float]: ...
    def see(self, label: str | None = None) -> list: ...
    def move(self, forward: float = 0.0, strafe: float = 0.0,
             turn: float = 0.0, climb: float = 0.0) -> None: ...
    def clock(self) -> float: ...


# Static description of each backend. `status` is refined at runtime by probing.
_REGISTRY: dict[str, dict[str, Any]] = {
    "rapier": {
        "name": "Rapier (browser)", "where": "web/physics.js",
        "kind": "sim", "base_status": "ready",
        "caps": {"physics": True, "camera": True, "lidar": True,
                 "fleet": True, "determinism": "seeded", "headless": False},
    },
    "mujoco": {
        "name": "MuJoCo (headless)", "where": "sim_backend.py:23",
        "kind": "sim", "base_status": "ready",
        "caps": {"physics": True, "camera": True, "lidar": True,
                 "fleet": False, "determinism": "seeded", "headless": True},
    },
    "mjx": {
        "name": "MJX (vectorized)", "where": "mjx_env.py",
        "kind": "sim", "base_status": "optional",
        "caps": {"physics": True, "camera": False, "lidar": True,
                 "fleet": True, "determinism": "seeded", "headless": True,
                 "vectorized": True},
    },
    "gazebo": {
        "name": "Gazebo (gz-sim)", "where": "gz.py:23",
        "kind": "sim", "base_status": "available",
        "caps": {"physics": True, "camera": True, "lidar": True,
                 "fleet": True, "determinism": "lockstep", "headless": True},
    },
    "isaac": {
        "name": "Isaac Sim", "where": "isaac.py (planned, spec 03 P2)",
        "kind": "sim", "base_status": "planned",
        "caps": {"physics": True, "camera": True, "lidar": True,
                 "fleet": True, "determinism": "lockstep", "headless": True},
    },
    "real": {
        "name": "Real robot (ROS)", "where": "transport/, ros_telemetry.py",
        "kind": "real", "base_status": "available",
        "caps": {"physics": True, "camera": True, "lidar": True,
                 "fleet": True, "determinism": "none", "headless": True},
    },
}


def _status(bid: str, meta: dict) -> str:
    """Refine base status by actually probing what's installed/live."""
    base = meta["base_status"]
    try:
        if bid == "mjx":
            from roborun import mjx_env
            return "ready" if mjx_env.installed() else "optional"
        if bid == "gazebo":
            from roborun import gz
            return "ready" if gz.detect_world() else "available"
    except Exception:
        pass
    return base


def list_backends() -> list[dict[str, Any]]:
    out = []
    for bid, meta in _REGISTRY.items():
        out.append({"id": bid, "name": meta["name"], "where": meta["where"],
                    "kind": meta["kind"], "status": _status(bid, meta),
                    "caps": meta["caps"]})
    return out


def get(bid: str) -> dict[str, Any] | None:
    meta = _REGISTRY.get(bid)
    if not meta:
        return None
    return {"id": bid, **meta, "status": _status(bid, meta)}
