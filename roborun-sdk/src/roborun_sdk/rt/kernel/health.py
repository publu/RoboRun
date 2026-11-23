"""K6 device health and thermal.

- Liveness: no new driver stamp for 3 cycles, or a sample older than 150 ms, marks the arm STALE (not latched). An
  owner that misses a poll is STALE, not OFFLINE: the live connectors report OFFLINE only after a sustained outage
  (``connectors._polled.OFFLINE_AFTER_S``).
- A connector reporting fault/offline (adapter missing or wrong identity, motor comm lost, CAN down, owner crash)
  latches FAULT: the runtime refuses segments, disarms, reports ``arm_offline{cause}`` and holds what it commands.
- Thermal, per motor, from the active pack's effective thresholds (``ArmPack.thermal``): starts refused at >= 70 C,
  work moves stopped at 75 C, both capped by a native hold (Metal: 65 C joints, 55 C jaw); the native 95 C retreat
  stays native. The thresholds follow pack activation (``set_packs``).
Clearing a latched fault is an operator action after recovery.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from ..datatypes import ArmSample, Event, Health, OperatorConfirm, Refused


@dataclass
class ArmHealth:
    state: str = "unknown"          # ok | stale | fault | offline | unknown
    cause: str | None = None
    since: float = 0.0
    latched: bool = False
    last_id: str | None = None
    last_t: float | None = None
    missed: int = 0
    thermal: str = "ok"             # ok | refuse_start | stop_work
    temp_max_c: float | None = None
    history: list = field(default_factory=list)


class HealthMonitor:
    def __init__(self, arms: dict, *, control_hz: dict, max_age_s: float = 0.150, stale_cycles: int = 3):
        self.arms = {name: ArmHealth() for name in arms}
        self.packs = dict(arms)
        self.control_hz = control_hz
        self.max_age_s = max_age_s
        self.stale_cycles = stale_cycles

    def set_packs(self, arms: dict, *, max_age_s: float) -> None:
        """A new pack was activated: its thermal and liveness limits apply from the next observation."""
        self.packs.update(arms)
        self.max_age_s = float(max_age_s)

    def thermal_level(self, arm: str, temps) -> str:
        pack = self.packs[arm]
        th = pack.thermal
        joints = [t for t in list(temps)[:pack.n]]
        jaw = [t for t in list(temps)[pack.n:pack.n + 1]]
        jmax = max(joints) if joints else None
        wmax = max(jaw) if jaw else None
        if (jmax is not None and jmax >= th["stop_work"]) or (wmax is not None and wmax >= th["jaw_stop_work"]):
            return "stop_work"
        if (jmax is not None and jmax >= th["refuse_start"]) or (wmax is not None and wmax >= th["jaw_refuse_start"]):
            return "refuse_start"
        return "ok"

    def observe(self, arm: str, sample: ArmSample | None, health: Health | None, now: float) -> list[Event]:
        h = self.arms[arm]
        events = []
        if health is not None and health.state in ("fault", "offline") and not h.latched:
            h.latched, h.state, h.cause, h.since = True, health.state, health.cause or health.state, now
            events.append(Event("arm_offline", now, {"arm": arm, "latched": True, "cause": h.cause,
                                                     "health": health.state, "detail": dict(health.detail)}))
        if sample is not None and sample.sample_id != h.last_id:
            h.last_id, h.last_t, h.missed = sample.sample_id, sample.t, 0
        else:
            h.missed += 1
        age = None if h.last_t is None else now - h.last_t + (sample.clock_uncertainty_s if sample else 0.0)
        period = 1.0 / max(1.0, float(self.control_hz.get(arm, 30.0)))
        if not h.latched:
            stale = h.last_t is None or age > self.max_age_s or (h.missed >= self.stale_cycles and age > self.stale_cycles * period)
            new_state = "stale" if stale else "ok"
            if new_state != h.state:
                if new_state == "stale" and h.state == "ok":
                    events.append(Event("arm_stale", now, {"arm": arm, "age_ms": None if age is None else round(age * 1000, 1)}))
                h.state, h.since = new_state, now
        if sample is not None and sample.temps_c:
            h.temp_max_c = max(sample.temps_c)
            level = self.thermal_level(arm, sample.temps_c)
            if level != h.thermal:
                events.append(Event("thermal", now, {"arm": arm, "level": level, "temp_max_c": h.temp_max_c}))
                h.thermal = level
        return events

    def faults(self) -> dict:
        return {arm: h.cause for arm, h in self.arms.items() if h.latched}

    def fresh(self, arm: str) -> bool:
        return self.arms[arm].state == "ok"

    def clear(self, arm: str, confirm: OperatorConfirm, now: float) -> dict:
        if not isinstance(confirm, OperatorConfirm) or not confirm.operator:
            raise Refused("operator_confirmation_required", "a latched fault clears only by an operator action")
        h = self.arms[arm]
        was = h.cause
        h.latched, h.cause, h.state, h.since = False, None, "unknown", now
        return {"arm": arm, "cleared": was, "operator": confirm.operator}

    def public(self) -> dict:
        return {arm: {"state": h.state, "cause": h.cause, "latched": h.latched, "thermal": h.thermal,
                      "temp_max_c": h.temp_max_c} for arm, h in self.arms.items()}
