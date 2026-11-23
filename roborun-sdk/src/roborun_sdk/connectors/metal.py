"""connector-metal: the MakerMods Metal owner on :8093 (the runtime's Metal controller).

Wrapped routes (all existing; nothing invented):
- GET  /status         manifest-metal-owner/1: state, measured.right (CAN query round-trip stamps), adapter, error
- POST /stop           lease-free and state-free; preempts the owner's own Home through an epoch bump
                       The persistent 30 Hz tick keeps sending position holds.
- POST /recover_hold   operator-started resync of already-enabled motors (never a takeover or re-enable)
- POST /jaw/set        {opening}: operator-confirmed jaw verb only ("I am supporting the object")
POST requires loopback, no browser Origin, ``X-Manifest-Metal-Control: 1`` and a JSON content type.
Never called: /home, /connect, /prepare + /move (manual jogs), /lease/action (streams from the robot runtime),
/reload_owner (a maintenance checkpoint).
"""
from __future__ import annotations

import math
import os
import time

from ..rt.clock import Clock
from ..rt.datatypes import (ArmDescriptor, ArmSample, ConnectorError, Health, HoldAck, NotWired, OperatorConfirm,
                            Refused)
from ._polled import OFFLINE_AFTER_S, PolledOwner
from .base import NOT_WIRED_STREAM
from .local_http import LocalJSON

SEMANTICS = "makermods-metal.rad6.jaw-motor-rad.v1"
ADAPTER_SERIAL = os.environ.get("ROBORUN_METAL_ADAPTER_SERIAL", "SIMMETAL0001")    # your candleLight adapter's USB serial
HEADERS = {"X-Manifest-Metal-Control": "1"}
STATUS = {"holding": "holding", "moving": "moving", "fault_hold": "fault_hold", "observing": "idle", "offline": "offline"}


def descriptor(name: str = "right") -> ArmDescriptor:
    return ArmDescriptor(name=name, embodiment="makermods-metal", semantics=SEMANTICS,
                         joints=("j1", "j2", "j3", "j4", "j5", "j6", "jaw"), control_hz=30.0, command_hz=30.0,
                         formats=frozenset({"path-rows.v1", "rows30.v1"}),
                         on_command_gap="unknown",      # the owner holds its target in software; what the motors do when it or the adapter goes away is unmeasured
                         drops_when_limp=True,
                         adapter={"kind": "candlelight", "channel": "can0", "serial": ADAPTER_SERIAL, "mode": "normal"},
                         jaw_unit="motor_rad")


def _floats(value, n):
    if (isinstance(value, list) and len(value) == n
            and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in value)):
        return tuple(float(v) for v in value)
    return None


def parse_status(status: dict, arm: str = "right") -> ArmSample | None:
    if not isinstance(status, dict) or status.get("schema") != "manifest-metal-owner/1":
        raise ConnectorError("not a manifest-metal-owner/1 answer")
    row = (status.get("measured") or {}).get("right")
    if not isinstance(row, dict):
        return None
    if row.get("body") != "metal" or row.get("timestamp_source") not in ("can_query_roundtrip", "metal_controller_cycle"):
        raise ConnectorError("right row is not a Metal CAN sample")
    q = _floats(row.get("positions_rad"), 6)
    stamp, unc = row.get("source_monotonic_s"), row.get("clock_uncertainty_s")
    if q is None or not isinstance(stamp, (int, float)) or not isinstance(unc, (int, float)):
        raise ConnectorError("right row is incomplete")
    jaw = row.get("gripper_motor_rad")
    jaw_cmd = row.get("commanded_gripper_motor_rad")
    efforts = _floats(row.get("motor_efforts_nm"), 7)
    temps = _floats(row.get("motor_temps_c"), 7)
    return ArmSample(arm=arm, sample_id=str(row.get("sample_id")), t=float(stamp), clock_uncertainty_s=float(unc),
                     q=q, q_cmd=_floats(row.get("commanded_rad"), 6), dq=_floats(row.get("velocities_rad_s"), 6),
                     jaw=float(jaw) if isinstance(jaw, (int, float)) else None,
                     jaw_cmd=float(jaw_cmd) if isinstance(jaw_cmd, (int, float)) else None,
                     effort=efforts or (), effort_fresh=efforts is not None, temps_c=temps or (),
                     status=STATUS.get(status.get("state"), "offline"), source=str(row.get("timestamp_source")), raw=row)


def hold_answer(answer: dict) -> tuple[bool | None, bool | None]:
    """(holding, powered) from the owner's own answer: holding only when it says holding (or fault_hold) and no motion."""
    state = answer.get("state")
    if state in ("holding", "fault_hold") and not answer.get("motion"):
        return True, True
    if (state in ("observing", "offline") and not answer.get("motion") and answer.get("connected") is False
            and answer.get("jaw_connected") is False):
        # nothing on that arm is powered: nothing drives it, but nothing holds it either (no brakes): not holding
        return False, False
    return False, True


class MetalOwnerConnector(PolledOwner):
    def __init__(self, base_url: str = "http://127.0.0.1:8093", *, arm: str = "right", allow_post: bool = False,
                 clock: Clock | None = None, hold_timeout_s: float = 1.5, adapter_serial: str = ADAPTER_SERIAL,
                 offline_after_s: float = OFFLINE_AFTER_S):
        super().__init__(clock or Clock(), offline_after_s=offline_after_s)
        self.descriptor = descriptor(arm)
        self.arm = arm
        self.http = LocalJSON(base_url, allow_post=allow_post, headers=HEADERS)
        self.hold_timeout_s = hold_timeout_s
        self.adapter_serial = adapter_serial

    def poll_once(self, timeout_s: float = 0.15, *, status: bool = True) -> ArmSample | None:
        # ``status`` (side reads) does not apply here: the Metal's /status answer is the sample itself
        try:
            answer = self.http.get("/status", timeout_s=timeout_s)
            sample = parse_status(answer, self.arm)
        except ConnectorError as exc:
            self._store(None, None, str(exc)[:200])
            return None
        self._store(sample, answer, None)
        return sample

    def health(self) -> Health:
        down = self._unreachable()
        if down is not None:
            return down
        now = self.clock.now()
        st = self.last_status() or {}
        adapter = st.get("adapter") or {}
        if adapter.get("usb_serial") not in (None, self.adapter_serial) or adapter.get("channel") not in (None, "can0"):
            return Health("fault", "adapter_identity_mismatch", now, {"adapter": adapter})
        state = st.get("state")
        if state == "offline":
            return Health("offline", "owner_offline", now, {"error": st.get("error")})
        if state == "fault_hold":
            return Health("fault", f"owner_fault: {st.get('error')}"[:160], now)
        if st.get("maintenance"):
            return Health("degraded", "maintenance_hold", now)
        return Health("ok", None, now)

    def acquire(self, grant) -> None:
        raise NotWired(NOT_WIRED_STREAM)

    def write(self, target, t) -> None:
        raise NotWired(NOT_WIRED_STREAM)

    def install(self, rows):
        raise NotWired("timed spans are not supported by this owner")

    def release(self) -> None:
        return None

    def hold(self, reason: str) -> HoldAck:
        started = time.perf_counter()
        try:
            answer = self.http.post("/stop", {}, timeout_s=self.hold_timeout_s)
        except ConnectorError as exc:
            return HoldAck(self.arm, None, None, time.perf_counter() - started, None, {"error": str(exc)[:200]})
        holding, powered = hold_answer(answer)
        try:
            sample = parse_status(answer, self.arm)
        except ConnectorError:
            sample = None
        detail = {"state": answer.get("state"), "motion": bool(answer.get("motion")),
                  "last_error": answer.get("error")}
        return HoldAck(self.arm, holding, sample, time.perf_counter() - started, powered, detail)

    def recover(self, confirm: OperatorConfirm) -> dict:
        if not isinstance(confirm, OperatorConfirm) or not confirm.operator:
            raise Refused("operator_confirmation_required", "recover-hold is operator-started")
        return self.http.post("/recover_hold", {}, timeout_s=3.0)

    def jaw_set(self, opening: float, confirm: OperatorConfirm) -> dict:
        """Operator-confirmed only: never called by the player or the kernel."""
        if not isinstance(confirm, OperatorConfirm) or "support" not in confirm.statement.lower():
            raise Refused("operator_confirmation_required", "jaw moves need 'I am supporting the object'")
        if isinstance(opening, bool) or not isinstance(opening, (int, float)) or not 0 <= opening <= 1:
            raise Refused("malformed", "opening is a fraction 0..1 of the observed range")
        return self.http.post("/jaw/set", {"opening": float(opening)}, timeout_s=3.0)
