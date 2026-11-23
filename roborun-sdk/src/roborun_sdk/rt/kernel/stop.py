"""K1 Stop.

Order of effects: (1) latch STOPPED, (2) every arm's ``hold()`` in parallel with a 300 ms budget each, (3) cancel the
segments and the task, (4) report ``stopped`` with per-arm holding state, measured pose and time to hold.
No auth, no lease id, no state precondition; repeats are harmless; an unverified Stop is obeyed and logged (its
signature is checked only after the latch, while the holds are in flight, to fill the ``verified`` field); Stop
never cuts power (brakeless arms). An arm whose hold is not confirmed within the budget is reported with
"Stop not confirmed on <arm>: support the arm or cut power". An unpowered arm is not held either (no brakes): it is
reported not holding, with "nothing holds it: support the arm". Clearing the latch takes an operator action and
starts nothing by itself.
"""
from __future__ import annotations

import threading
import time
from collections import deque

from ..datatypes import Event, HoldAck, OperatorConfirm, Refused

BUDGET_S = 0.300


def not_confirmed_text(arm: str) -> str:
    return f"Stop not confirmed on {arm}: support the arm or cut power"


def unpowered_text(arm: str) -> str:
    return f"Stop: {arm} is unpowered (motors off) and nothing holds it: support the arm"


class _Inflight:
    def __init__(self):
        self.done = threading.Event()
        self.ack: HoldAck | None = None
        self.error: str | None = None
        self.started = time.perf_counter()


class StopLatch:
    def __init__(self, clock, *, budget_s: float = BUDGET_S):
        self.clock = clock
        self.budget_s = budget_s
        self._latched = threading.Event()
        self._lock = threading.Lock()
        self._inflight: dict[str, _Inflight] = {}
        self.records: deque = deque(maxlen=128)
        self.last_report: dict | None = None
        self.count = 0

    @property
    def latched(self) -> bool:
        return self._latched.is_set()

    def latch(self) -> None:
        """Step 1 alone (signal handlers, tick watchdog). Writes are refused from here on."""
        self._latched.set()

    def _start_hold(self, arm: str, connector, reason: str) -> _Inflight:
        with self._lock:
            current = self._inflight.get(arm)
            if current is not None and not current.done.is_set():
                return current          # a hold for this arm is still in flight: share it, never pile threads up
            job = _Inflight()
            self._inflight[arm] = job

        def run():
            try:
                job.ack = connector.hold(reason)
            except Exception as exc:     # an unreachable owner is "not confirmed", never "holding"
                job.error = f"{type(exc).__name__}: {exc}"[:300]
            finally:
                job.done.set()

        threading.Thread(target=run, name=f"manifest-stop-{arm}", daemon=True).start()
        return job

    def hold_all(self, connectors: dict, reason: str, *, during=None) -> dict:
        """Hold every arm in parallel; wait at most the budget for each. Returns {arm: public ack}. ``during`` runs
        after every hold is in flight and before the wait (its exceptions are swallowed; it shares the budget)."""
        jobs = {arm: self._start_hold(arm, c, reason) for arm, c in connectors.items()}
        deadline = time.perf_counter() + self.budget_s
        if during is not None:
            try:
                during()
            except Exception:
                pass
        out = {}
        for arm, job in jobs.items():
            job.done.wait(max(0.0, deadline - time.perf_counter()))
            if not job.done.is_set():
                out[arm] = {"holding": None, "confirmed": False, "console": not_confirmed_text(arm),
                            "detail": {"why": "no_ack_within_budget", "budget_ms": round(self.budget_s * 1000)}}
            elif job.error is not None:
                out[arm] = {"holding": None, "confirmed": False, "console": not_confirmed_text(arm),
                            "detail": {"error": job.error}}
            else:
                ack = job.ack
                pub = ack.public()
                confirmed = ack.holding is True            # an unpowered brakeless arm is not evidence of a hold
                pub["confirmed"] = confirmed
                if not confirmed:
                    pub["console"] = unpowered_text(arm) if ack.powered is False else not_confirmed_text(arm)
                out[arm] = pub
        return out

    def trigger(self, source: str, *, connectors: dict, stop_id: str | None = None, verified: bool = True,
                cancel=None, emit=None, user_role: str | None = None, verify=None) -> dict:
        """``verify``: optional check of a relayed Stop's signature. It runs after the latch, while the holds are in
        flight, and only fills ``verified``; nothing it does (or raises) can delay or prevent the Stop."""
        self._latched.set()                                    # 1. latch, before anything else can fail
        t0 = self.clock.now()
        wall0 = time.perf_counter()
        self.count += 1
        checked = {"verified": bool(verified) if verify is None else False}

        def check():
            checked["verified"] = verify() is True
        acks = self.hold_all(connectors, f"stop:{source}",       # 2. owners first, in parallel
                             during=check if verify is not None else None)
        verified = checked["verified"]
        cancelled = None
        if cancel is not None:                                 # 3. segments and task
            try:
                cancelled = cancel("stopped")
            except Exception as exc:
                cancelled = {"error": str(exc)[:200]}
        report = {"ref": stop_id, "source": source, "verified": verified, "arms": acks,   # 4. report
                  "not_confirmed": sorted(a for a, v in acks.items() if not v.get("confirmed")),
                  "elapsed_ms": round((time.perf_counter() - wall0) * 1000, 1), "cancelled": cancelled}
        if user_role:
            report["user_role"] = user_role
        self.last_report = report
        self.records.append({"t": t0, **report})
        if emit is not None:
            emit(Event("stopped", t0, report))
        return report

    def clear(self, confirm: OperatorConfirm, *, role: str = "operator") -> dict:
        if role not in ("operator", "owner"):
            raise Refused("operator_role_required", "clearing a Stop needs the operator role")
        if not isinstance(confirm, OperatorConfirm) or not confirm.operator:
            raise Refused("operator_confirmation_required", "clearing a Stop takes an operator action")
        was = self.latched
        self._latched.clear()
        rec = {"cleared": was, "operator": confirm.operator, "source": confirm.source, "t": confirm.t}
        self.records.append({"t": confirm.t, "clear": rec})
        return rec
