"""Connector interface: one implementation per arm owner (the local service that drives an arm).

What each call does on the two owners:

| call      | YAM station :8091                                 | Metal owner :8093                         |
|-----------|---------------------------------------------------|-------------------------------------------|
| sample    | GET /supervisor/pose (driver_state_cycle stamps)  | GET /status (CAN query round-trip stamps) |
| health    | pose arms state + GET /status + adapter in sysfs  | status state/error/adapter                |
| acquire   | not supported (NotWired)                          | not supported (NotWired)                  |
| write     | not supported (NotWired)                          | not supported (NotWired)                  |
| hold      | POST /supervisor/hold if present, else            | POST /stop (lease-free, any state)        |
|           | /supervisor/stop per active lease (+ vendor /stop)|                                           |
| release   | no-op (nothing acquired)                          | no-op                                     |
| recover   | refused: the vendor Reset lets both arms go       | POST /recover_hold (operator-confirmed)   |

Optional: ``refresh(timeout_s) -> ArmSample | None``, one synchronous read (the polled owners implement it; roborun-rt
calls it outside its lock when a segment arrives, so the start sample is fresh). A connector
without it is read through ``sample()`` alone (the in-process sim arms are always current).

These connectors do not command motion: they refuse acquire/write with ``NotWired`` and never emulate streaming with
another motion route.
"""
from __future__ import annotations

from typing import Protocol, Sequence, runtime_checkable

from ..rt.datatypes import (ArmDescriptor, ArmSample, ConnectorError, Grant, Health, HoldAck, NotWired,  # noqa: F401
                            OperatorConfirm, ReadOnlyConnector)


@runtime_checkable
class ArmConnector(Protocol):
    descriptor: ArmDescriptor

    def sample(self) -> ArmSample: ...                       # newest cached sample; never blocks > 1 ms
    def health(self) -> Health: ...
    def acquire(self, grant: Grant) -> None: ...             # station lease / Metal session
    def write(self, target: Sequence[float], t: float) -> None: ...
    def hold(self, reason: str) -> HoldAck: ...              # any state, no lease id, idempotent; never disables or opens a jaw
    def release(self) -> None: ...
    def recover(self, confirm: OperatorConfirm) -> dict: ...


@runtime_checkable
class CameraConnector(Protocol):
    def identity(self) -> dict: ...
    def latest(self, after_id: int | None = None) -> dict: ...


@runtime_checkable
class StopInput(Protocol):
    def on_press(self, callback) -> None: ...


NOT_WIRED_STREAM = ("these connectors do not command motion; the robot runtime drives the arms")
