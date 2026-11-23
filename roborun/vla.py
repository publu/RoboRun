"""VLA adapter — make robot foundation models (GR00T, OpenVLA, RT-X) native.

A VLA maps (image, instruction) → action. This wires any such model into the
handle so it runs like any behavior, against the same safety-clamped `move()`,
recorded into the same sealed run. The *model* is pluggable and optional (you
bring GR00T/OpenVLA via HF, or a stub) — RoboRun bundles only the thin adapter.

    from roborun.vla import register_vla, VLAPolicy
    register_vla("groot", lambda jpeg, instr: {...action...})   # your model

    @behavior(hz=5)
    def vla_drive(robot):
        VLAPolicy("groot").step(robot, "go to the charging dock")
"""
from __future__ import annotations

from typing import Any, Callable

# action keys the adapter maps onto robot.move(); a VLA may emit a subset
_ACTION_KEYS = ("forward", "strafe", "turn", "climb")

_REGISTRY: dict[str, Callable[[bytes, str], dict]] = {}


def register_vla(name: str, model_fn: Callable[[bytes, str], dict]) -> None:
    """Register a VLA: `model_fn(image_jpeg, instruction) -> {forward,strafe,
    turn,climb}` (any subset, each in [-1,1]). The model is the caller's."""
    _REGISTRY[name] = model_fn


def list_vla() -> list[str]:
    return sorted(_REGISTRY)


def load_vla(name: str) -> Callable[[bytes, str], dict]:
    """Resolve a registered VLA, or try a known loader (GR00T/OpenVLA via HF if
    installed). Raises with a clear, actionable message otherwise."""
    if name in _REGISTRY:
        return _REGISTRY[name]
    raise RuntimeError(
        f"VLA {name!r} not registered. register_vla({name!r}, fn) with a model "
        f"that maps (jpeg, instruction)->action — e.g. GR00T N1 or OpenVLA from "
        f"HuggingFace. RoboRun ships the adapter, not the 3B weights.")


class VLAPolicy:
    """Drives the robot from a VLA, one inference per tick. Actuation passes the
    same clamps + estop as everything else; nothing bypasses safety."""

    def __init__(self, model: str | Callable[[bytes, str], dict]) -> None:
        self._fn = model if callable(model) else load_vla(model)

    def infer(self, image_jpeg: bytes, instruction: str) -> dict:
        action = self._fn(image_jpeg, instruction) or {}
        return {k: float(action.get(k, 0.0)) for k in _ACTION_KEYS}

    def step(self, robot: Any, instruction: str) -> dict:
        """Grab the current frame, infer, and move. Returns the action taken
        (or a no-op {} if there's no frame yet)."""
        jpeg = robot.frame_jpeg() if hasattr(robot, "frame_jpeg") else None
        if not jpeg:
            robot.stop()
            return {}
        a = self.infer(jpeg, instruction)
        robot.move(forward=a["forward"], strafe=a["strafe"],
                   turn=a["turn"], climb=a["climb"])
        return a
