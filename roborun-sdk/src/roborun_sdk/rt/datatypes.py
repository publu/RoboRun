"""Shared value types for roborun-rt and the connectors."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Literal

ArmStatus = Literal["idle", "moving", "holding", "parked", "fault_hold", "offline"]
HealthState = Literal["ok", "degraded", "fault", "offline"]


class Refused(ValueError):
    """A typed refusal: ``reason`` is a stable machine code, ``detail`` a small JSON-able dict."""

    def __init__(self, reason: str, message: str = "", **detail):
        self.reason = reason
        self.detail = detail
        super().__init__(f"{reason}: {message}" if message else reason)
        self.message = message or reason

    def as_dict(self) -> dict:
        out = {"reason": self.reason, "message": self.message}
        if self.detail:
            out["detail"] = self.detail
        return out


class ConnectorError(RuntimeError):
    """The owner answered with an error, or could not be reached."""


class NotWired(ConnectorError):
    """The operation has no native route yet. Never emulated with another motion route."""


class ReadOnlyConnector(ConnectorError):
    """A connector built read-only (doctor, tests, ``run --live`` acceptance) was asked to POST."""


@dataclass(frozen=True)
class ArmDescriptor:
    name: str                    # rig-local name: "left", "right"
    embodiment: str              # embodiment id: "yam-linear-4310", "makermods-metal"
    semantics: str               # joint order + units, e.g. "makermods-metal.rad6.jaw-motor-rad.v1"
    joints: tuple[str, ...]      # native order, jaw last
    control_hz: float
    command_hz: float
    formats: frozenset[str]
    on_command_gap: Literal["holds_last_target", "float", "disable", "unknown"]
    drops_when_limp: bool
    adapter: dict
    jaw_unit: Literal["open_fraction", "motor_rad"]

    def public(self) -> dict:
        return {"semantics": self.semantics, "embodiment": self.embodiment, "joints": list(self.joints),
                "connector": {"control_hz": self.control_hz, "command_hz": self.command_hz,
                              "formats": sorted(self.formats), "hold": "position",
                              "on_command_gap": self.on_command_gap},
                "adapter": dict(self.adapter), "jaw_unit": self.jaw_unit}


@dataclass(frozen=True)
class ArmSample:
    """One measured sample, stamped at the driver cycle on the rig's monotonic clock."""
    arm: str
    sample_id: str
    t: float
    clock_uncertainty_s: float
    q: tuple[float, ...]
    q_cmd: tuple[float, ...] | None
    dq: tuple[float, ...] | None
    jaw: float | None
    jaw_cmd: float | None
    effort: tuple[float, ...] = ()
    effort_fresh: bool = False
    temps_c: tuple[float, ...] = ()
    status: ArmStatus = "idle"
    source: str = ""
    raw: dict | None = field(default=None, compare=False, repr=False)   # the owner's row, for the connector's own guards

    def q7(self):
        """Joints plus jaw (jaw measured, or NaN when unknown)."""
        return tuple(self.q) + (self.jaw if self.jaw is not None else math.nan,)

    def public(self, now: float | None = None) -> dict:
        out = {"sample": self.sample_id, "src_t": self.t, "q": list(self.q),
               "q_cmd": None if self.q_cmd is None else list(self.q_cmd),
               "dq": None if self.dq is None else list(self.dq), "jaw": self.jaw, "jaw_cmd": self.jaw_cmd,
               "temp_max_c": max(self.temps_c) if self.temps_c else None, "status": self.status}
        if now is not None:
            out["age_ms"] = round((now - self.t + self.clock_uncertainty_s) * 1000, 1)
        return out


@dataclass(frozen=True)
class Health:
    state: HealthState
    cause: str | None
    since: float
    detail: dict = field(default_factory=dict, compare=False)


@dataclass(frozen=True)
class HoldAck:
    """Per-arm Stop/Hold answer. ``holding`` comes from the owner's measured state, never assumed.

    True: confirmed holding. False: the owner answered but is not holding (or refused), including an unpowered arm
    (``powered`` False: nothing drives it, nothing holds it; these arms have no brakes). None: no answer within the
    budget.
    """
    arm: str
    holding: bool | None
    sample: ArmSample | None
    latency_s: float
    powered: bool | None = True
    detail: dict = field(default_factory=dict)

    def public(self) -> dict:
        out = {"holding": self.holding, "time_to_hold_ms": round(self.latency_s * 1000, 1)}
        if self.sample is not None:
            out["q"] = [round(v, 4) for v in self.sample.q]
            if self.sample.jaw is not None:
                out["jaw"] = round(self.sample.jaw, 4)
        if self.powered is False:
            out["powered"] = False
        if self.detail:
            out["detail"] = self.detail
        return out


@dataclass(frozen=True)
class Grant:
    """Motion authority for one arm, bound to (segment, generation, pack) (K2)."""
    arm: str
    segment: str
    generation: int
    pack: str
    issued_t: float
    speed_cap_m_s: float


@dataclass(frozen=True)
class OperatorConfirm:
    """A local operator action (console switch, CLI prompt, key switch). Required to clear a Stop, confirm a
    loosening pack, attest an empty gripper or start a Metal recover-hold."""
    operator: str
    statement: str
    t: float
    source: Literal["console", "cli", "key_switch", "test"] = "console"


@dataclass(frozen=True)
class Event:
    kind: str
    t: float
    data: dict[str, Any] = field(default_factory=dict)

    def public(self, seq: int) -> dict:
        if self.kind == "gate_report":                  # its own wire message type
            out = {"type": "gate_report", "seq": seq, "sdk_t": round(self.t, 6)}
        else:
            out = {"type": "event", "seq": seq, "sdk_t": round(self.t, 6), "kind": self.kind}
        out.update({k: v for k, v in self.data.items() if k != "type"})
        return out
