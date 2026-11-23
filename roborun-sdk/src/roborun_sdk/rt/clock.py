"""K7 time authority: one monotonic clock for the rig.

The SDK monotonic clock is authoritative. The two arm owners stamp samples with the host's CLOCK_MONOTONIC
(``source_monotonic_s``: the YAM driver cycle, the Metal CAN query round trip), so they already share this clock.
A sample whose clock uncertainty exceeds 5 ms is refused (the robot runtime's own check requires the same). Only the kernel
computes ages.
"""
from __future__ import annotations

import math
import threading
import time
from pathlib import Path

MAX_CLOCK_UNCERTAINTY_S = 0.005


def _boot_id() -> str:
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text().strip().replace("-", "")
    except OSError:
        return "unknown"


class Clock:
    """The host monotonic clock."""

    source = "CLOCK_MONOTONIC"
    virtual = False

    def __init__(self):
        self.boot_id = _boot_id()
        self.id = "mono-" + self.boot_id[:6]

    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        if seconds > 0:
            time.sleep(seconds)

    def describe(self) -> dict:
        return {"id": self.id, "source": self.source, "boot_id": self.boot_id, "t": self.now()}


class SimClock(Clock):
    """Virtual time for deterministic tests and sim runs. ``sleep`` advances it; nothing waits."""

    source = "SIM"
    virtual = True

    def __init__(self, start: float = 1000.0):
        self.boot_id = "sim"
        self.id = "sim-clock"
        self._t = float(start)
        self._lock = threading.Lock()

    def now(self) -> float:
        with self._lock:
            return self._t

    def advance(self, seconds: float) -> float:
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("a virtual clock only moves forward")
        with self._lock:
            self._t += seconds
            return self._t

    def sleep(self, seconds: float) -> None:
        if seconds > 0:
            self.advance(seconds)


def sample_age(now: float, stamp: float, uncertainty: float) -> float:
    """Age charged to a sample: elapsed since its driver stamp plus its stamp uncertainty."""
    return now - stamp + uncertainty


def stamp_ok(now: float, stamp, uncertainty) -> str | None:
    """None if the stamp is usable, else a typed reason."""
    if not isinstance(stamp, (int, float)) or isinstance(stamp, bool) or not math.isfinite(stamp):
        return "stamp_missing"
    if not isinstance(uncertainty, (int, float)) or isinstance(uncertainty, bool) or not math.isfinite(uncertainty):
        return "stamp_uncertainty_missing"
    if not 0 <= uncertainty <= MAX_CLOCK_UNCERTAINTY_S:
        return "stamp_uncertainty_over_5ms"
    if stamp - now > 1e-6:
        return "stamp_in_future"
    return None
