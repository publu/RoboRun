"""connector-yam: the native YAM station on :8091 (the runtime's supervisor and pose routes over the vendor station API).

Wrapped routes (all existing; nothing invented):
- GET  /supervisor/pose      manifest-native-pose/1: measured.left (driver_state_cycle stamps), leases, controller_state
- GET  /status               vendor station status: arms alive, state, last_error
- POST /supervisor/hold      lease-free hold, *if* the station has it:
                             replies manifest-station-hold/1 with each arm's MEASURED state (holding / moving /
                             fault / resetting / stale). GET /supervisor/hold says whether it is installed.
- POST /supervisor/stop      {lease_id}: ends a supervised lease; the stock writer holds (needs the lease id)
- POST /stop                 vendor Stop: cancels a stock run and waits until the arms hold (refused while HOMING,
                             RESETTING or STOPPING)
Never called: /home (both-jaw Home), /reset (lets both arms go), /start, /supervisor/action (would start a lease that
streams from the robot runtime, not from this SDK).
"""
from __future__ import annotations

import math
import os
import time
from pathlib import Path

from ..rt.clock import Clock
from ..rt.datatypes import (ArmDescriptor, ArmSample, ConnectorError, Health, HoldAck, NotWired, OperatorConfirm)
from ._polled import OFFLINE_AFTER_S, PolledOwner, usb_serial_present
from .base import NOT_WIRED_STREAM
from .local_http import LocalJSON, OwnerError

SEMANTICS = "yam-linear-4310.rad6.jaw-open-fraction.v1"
ADAPTER_SERIAL = os.environ.get("ROBORUN_YAM_ADAPTER_SERIAL", "SIMYAMLEFT01")      # your slcan adapter's USB serial
HEADERS = {"X-Manifest-Supervisor": "1"}
MOVING_STATES = ("starting", "running", "stopping", "homing", "resetting")
UNSTOPPABLE_STATES = ("homing", "resetting", "stopping")


def descriptor(name: str = "left") -> ArmDescriptor:
    return ArmDescriptor(name=name, embodiment="yam-linear-4310", semantics=SEMANTICS,
                         joints=("j1", "j2", "j3", "j4", "j5", "j6", "jaw"), control_hz=30.0, command_hz=100.0,
                         formats=frozenset({"path-rows.v1", "rows30.v1"}),
                         on_command_gap="disable",      # the motors' CAN timeout ends position control (i2rt)
                         drops_when_limp=True, adapter={"kind": "slcan", "serial": ADAPTER_SERIAL, "mode": "normal"},
                         jaw_unit="open_fraction")


def _finite_list(value, n):
    return (isinstance(value, list) and len(value) == n
            and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in value))


def active_leases(pose: dict) -> list[str]:
    out = []
    for key, lease in pose.items():
        if "lease" in key and isinstance(lease, dict) and lease.get("active", True) and isinstance(lease.get("id"), str):
            if lease["id"] not in out:
                out.append(lease["id"])
    return out


def parse_pose(pose: dict, arm: str = "left") -> ArmSample | None:
    if not isinstance(pose, dict) or pose.get("schema") != "manifest-native-pose/1":
        raise ConnectorError("not a manifest-native-pose/1 answer")
    row = (pose.get("measured") or {}).get(arm)
    arm_state = ((pose.get("arms") or {}).get(arm) or {}).get("state")
    if not isinstance(row, dict):
        return None
    if row.get("body") != "yam" or row.get("timestamp_source") != "driver_state_cycle":
        raise ConnectorError("left row is not a YAM driver-cycle sample")
    q, stamp, unc = row.get("positions_rad"), row.get("source_monotonic_s"), row.get("clock_uncertainty_s")
    if not _finite_list(q, 6) or not isinstance(stamp, (int, float)) or not isinstance(unc, (int, float)):
        raise ConnectorError("left row is incomplete")
    cmd = row.get("commanded_rad")
    state = pose.get("controller_state")
    if arm_state == "parked":
        status = "parked"
    elif arm_state == "unavailable":
        status = "offline"
    elif state == "error":
        status = "fault_hold"
    elif state in MOVING_STATES or active_leases(pose):
        status = "moving"
    else:
        status = "holding"
    temps = row.get("motor_temps_c")
    jaw, jaw_cmd = row.get("gripper"), row.get("commanded_gripper")
    return ArmSample(arm=arm, sample_id=str(row.get("sample_id")), t=float(stamp), clock_uncertainty_s=float(unc),
                     q=tuple(float(v) for v in q), q_cmd=tuple(float(v) for v in cmd) if _finite_list(cmd, 6) else None,
                     dq=None, jaw=float(jaw) if isinstance(jaw, (int, float)) else None,
                     jaw_cmd=float(jaw_cmd) if isinstance(jaw_cmd, (int, float)) else None,
                     temps_c=tuple(float(t) for t in temps) if _finite_list(temps, 7) else (),
                     status=status, source="driver_state_cycle", raw=row)


class YamStationConnector(PolledOwner):
    def __init__(self, base_url: str = "http://127.0.0.1:8091", *, arm: str = "left", allow_post: bool = False,
                 clock: Clock | None = None, hold_budget_s: float = 1.5, status_every_s: float = 1.0,
                 adapter_serial: str = ADAPTER_SERIAL, sysroot: Path = Path("/sys"),
                 offline_after_s: float = OFFLINE_AFTER_S):
        super().__init__(clock or Clock(), offline_after_s=offline_after_s)
        self.descriptor = descriptor(arm)
        self.arm = arm
        self.http = LocalJSON(base_url, allow_post=allow_post, headers=HEADERS)
        self.hold_budget_s = hold_budget_s
        self.status_every_s = status_every_s
        self.adapter_serial = adapter_serial
        self.sysroot = sysroot
        self._vendor: dict | None = None
        self._vendor_t = -math.inf
        self._adapter: tuple[float, bool | None] = (-math.inf, None)

    # ------------------------------------------------------------------ reading
    def poll_once(self, timeout_s: float = 0.15, *, status: bool = True) -> ArmSample | None:
        try:
            pose = self.http.get("/supervisor/pose", timeout_s=timeout_s)
            sample = parse_pose(pose, self.arm)
        except ConnectorError as exc:
            self._store(None, None, str(exc)[:200])
            return None
        self._store(sample, pose, None)
        now = self.clock.now()
        if status and now - self._vendor_t >= self.status_every_s:
            self._vendor_t = now
            try:
                self._vendor = self.http.get("/status", timeout_s=0.3)
            except ConnectorError as exc:
                self._vendor = {"_error": str(exc)[:200]}
        return sample

    def _adapter_present(self) -> bool | None:
        t, present = self._adapter
        now = self.clock.now()
        if now - t >= 2.0:
            present = usb_serial_present(self.adapter_serial, self.sysroot)
            self._adapter = (now, present)
        return present

    def health(self) -> Health:
        down = self._unreachable()
        if down is not None:
            return down
        now = self.clock.now()
        pose = self.last_status() or {}
        arm_state = ((pose.get("arms") or {}).get(self.arm) or {})
        if self._adapter_present() is False:
            return Health("fault", "adapter_missing", now, {"serial": self.adapter_serial})
        if arm_state.get("state") == "unavailable":
            return Health("offline", "driver_unavailable", now, {"error": arm_state.get("error")})
        vendor = self._vendor or {}
        if (vendor.get("arms") or {}).get(self.arm) is False:
            return Health("fault", "motor_chain_dead", now, {"last_error": vendor.get("last_error")})
        if pose.get("controller_state") == "error":
            return Health("fault", "station_error", now, {"last_error": vendor.get("last_error")})
        if arm_state.get("state") == "parked":
            return Health("degraded", "parked", now)
        return Health("ok", None, now)

    # ------------------------------------------------------------------ motion (not supported: NotWired)
    def acquire(self, grant) -> None:
        raise NotWired(NOT_WIRED_STREAM)

    def write(self, target, t) -> None:
        raise NotWired(NOT_WIRED_STREAM)

    def install(self, rows):
        raise NotWired("timed spans are not supported by this owner")

    def release(self) -> None:
        return None

    def recover(self, confirm: OperatorConfirm) -> dict:
        raise NotWired("the vendor Reset lets both arms go; per-arm reopen is not supported by this owner. "
                       "Use the station's own Reset with both arms supported, never with an adapter missing")

    # ------------------------------------------------------------------ hold (K1)
    def hold(self, reason: str) -> HoldAck:
        started = time.perf_counter()
        deadline = started + self.hold_budget_s
        def left():
            return max(0.05, deadline - time.perf_counter())
        steps: list = []
        try:
            answer = self.http.post("/supervisor/hold", {"source": "roborun-sdk", "reason": reason[:200]},
                                    timeout_s=left())
            steps.append({"route": "/supervisor/hold", "ok": True, "actions": answer.get("actions"),
                          "pending": answer.get("pending")})
            row = (answer.get("arms") or {}).get(self.arm)
            if isinstance(row, dict) and isinstance(row.get("holding"), bool):
                # the station measured each arm after its hold: use that, never an assumption
                q = row.get("positions_rad") or row.get("q")
                sample = None
                if isinstance(q, list) and len(q) == 6:
                    jaw = row.get("gripper", row.get("jaw"))
                    sample = ArmSample(arm=self.arm, sample_id=f"hold:{(answer.get('hold') or {}).get('id', '')}",
                                       t=float(answer.get("measured_monotonic_s") or answer.get("held_at") or 0.0),
                                       clock_uncertainty_s=0.0, q=tuple(float(v) for v in q), q_cmd=None, dq=None,
                                       jaw=float(jaw) if isinstance(jaw, (int, float)) else None, jaw_cmd=None,
                                       status="holding" if row["holding"] else "moving", source="station_hold")
                detail = {"steps": steps, "state": row.get("state"), "cause": row.get("cause"),
                          "controller_state": answer.get("controller_state")}
                powered = False if row.get("state") == "resetting" else True
                return HoldAck(self.arm, row["holding"], sample, time.perf_counter() - started, powered, detail)
            if self.arm in (answer.get("parked_arms") or []):
                return HoldAck(self.arm, None, None, time.perf_counter() - started, None,
                               {"steps": steps, "state": "parked"})
            fallback = False
        except OwnerError as exc:
            unknown = exc.status == 404 or "Unknown supervisor route" in exc.message
            steps.append({"route": "/supervisor/hold", "ok": False, "status": exc.status,
                          "error": "route_absent" if unknown else exc.message[:160]})
            fallback = True
        except ConnectorError as exc:
            steps.append({"route": "/supervisor/hold", "ok": False, "error": str(exc)[:160]})
            fallback = True
        if fallback:
            try:
                pose = self.http.get("/supervisor/pose", timeout_s=left())
            except ConnectorError as exc:
                steps.append({"route": "GET /supervisor/pose", "ok": False, "error": str(exc)[:160]})
                return HoldAck(self.arm, None, None, time.perf_counter() - started, None, {"steps": steps})
            for lease in active_leases(pose):
                try:
                    self.http.post("/supervisor/stop", {"lease_id": lease}, timeout_s=left())
                    steps.append({"route": "/supervisor/stop", "lease": lease, "ok": True})
                except ConnectorError as exc:
                    steps.append({"route": "/supervisor/stop", "lease": lease, "ok": False, "error": str(exc)[:160]})
            state = pose.get("controller_state")
            if state in ("starting", "running") and not active_leases(pose):
                try:
                    self.http.post("/stop", {}, timeout_s=left())
                    steps.append({"route": "/stop", "ok": True})
                except ConnectorError as exc:
                    steps.append({"route": "/stop", "ok": False, "error": str(exc)[:160]})
            if state in UNSTOPPABLE_STATES:
                steps.append({"note": f"station is {state}: the vendor refuses Stop until it ends"})
        # the ack comes from measured state, never assumed
        try:
            pose = self.http.get("/supervisor/pose", timeout_s=left())
            sample = parse_pose(pose, self.arm)
        except ConnectorError as exc:
            steps.append({"route": "GET /supervisor/pose", "ok": False, "error": str(exc)[:160]})
            return HoldAck(self.arm, None, None, time.perf_counter() - started, None, {"steps": steps})
        state = pose.get("controller_state")
        arm_state = ((pose.get("arms") or {}).get(self.arm) or {}).get("state")
        fresh = sample is not None and bool(sample.raw.get("fresh", True))
        holding = state == "idle" and not active_leases(pose) and arm_state in ("fresh", "stale") and fresh
        detail = {"steps": steps, "controller_state": state, "arm_state": arm_state, "leases": active_leases(pose)}
        if arm_state == "parked":
            return HoldAck(self.arm, None, sample, time.perf_counter() - started, None, detail)
        return HoldAck(self.arm, bool(holding), sample, time.perf_counter() - started, True, detail)
