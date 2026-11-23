"""Shared plumbing for the live owner connectors: a poller thread keeps the newest sample so ``sample()`` never
blocks (``sample()`` p99 < 1 ms).

Liveness (K6): a failed or slow poll is not a fault. The sample simply ages, and the kernel marks the arm STALE (3
cycles / 150 ms: fresh-pose hold during a segment, refusals at acceptance). The owner is reported OFFLINE (which the
kernel latches) only after ``offline_after_s`` without one successful poll. Before the first poll has answered the
connector says ``not_polled_yet`` (degraded, never a fault); ``wait_first_poll`` lets a caller start ticking only
after each owner was asked once.

Fresh start samples. The YAM station stamps its pose rows at its 30 Hz driver cycle, so a
cached sample from a 30 Hz poller is up to ~67 ms old at a random instant (45 % of starts beyond acceptance's 50 ms in
sim). ``refresh()`` reads one sample synchronously in the caller's thread (roborun-rt calls it when a segment
arrives, outside its lock): its age is then the owner's own stamp phase plus one local GET.
"""
from __future__ import annotations

import threading
import time
from pathlib import Path

from ..rt.datatypes import ConnectorError, Health

OFFLINE_AFTER_S = 1.0          # sustained outage before OFFLINE (the local link-loss threshold); STALE comes first
REFRESH_TIMEOUT_S = 0.05       # a synchronous read never waits longer than this (an in-flight poll, then one GET)


class PolledOwner:
    poll_hz = 30.0

    def __init__(self, clock, *, offline_after_s: float = OFFLINE_AFTER_S):
        self.clock = clock
        self.offline_after_s = float(offline_after_s)
        self._lock = threading.Lock()
        self._sample = None
        self._status: dict | None = None
        self._poll_error: str | None = None
        self._poll_ok_t: float | None = None
        self._first_poll_t: float | None = None
        self._polled = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._poll_lock = threading.Lock()   # one poll at a time: the poller thread or a synchronous refresh()

    # subclasses: poll_once(timeout_s=..., status=True) -> ArmSample | None (sets self._status)

    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._loop, name=f"manifest-poll-{self.descriptor.name}", daemon=True)
            self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(1.0)

    def wait_first_poll(self, timeout_s: float) -> bool:
        """True once a first poll has answered (success or error) within ``timeout_s``."""
        return self._polled.wait(max(0.0, timeout_s))

    def _loop(self) -> None:
        period = 1.0 / self.poll_hz
        while not self._stop.is_set():
            started = time.monotonic()
            with self._poll_lock:
                try:
                    self.poll_once()
                except Exception as exc:
                    self._store(None, None, f"poll: {type(exc).__name__}: {exc}"[:200])
            self._stop.wait(max(0.0, period - (time.monotonic() - started)))

    def refresh(self, timeout_s: float = REFRESH_TIMEOUT_S):
        """One synchronous read now: the newest sample after it, or None. Waits for an in-flight poll rather
        than starting a second one beside it, never longer than ``timeout_s`` in all; never raises. Status-only
        side reads (the vendor /status) are left to the poller."""
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        if self._poll_lock.acquire(timeout=max(0.0, float(timeout_s))):
            try:
                left = deadline - time.monotonic()
                if left > 0.002:
                    self.poll_once(timeout_s=left, status=False)
            except Exception as exc:
                self._store(None, None, f"refresh: {type(exc).__name__}: {exc}"[:200])
            finally:
                self._poll_lock.release()
        with self._lock:
            return self._sample

    def _store(self, sample, status: dict | None, error: str | None) -> None:
        now = self.clock.now()
        with self._lock:
            if sample is not None and (self._sample is None or sample.t >= self._sample.t):
                self._sample = sample
            if status is not None:
                self._status = status
            self._poll_error = error
            if self._first_poll_t is None:
                self._first_poll_t = now
            if error is None:
                self._poll_ok_t = now
        self._polled.set()

    def sample(self):
        with self._lock:
            s = self._sample
        if s is None:
            raise ConnectorError(f"{self.descriptor.name}: no sample yet")
        return s

    def last_status(self) -> dict | None:
        with self._lock:
            return self._status

    def _unreachable(self) -> Health | None:
        """None when the last poll answered. A poll error is ``degraded`` (the sample ages into STALE) until the owner
        has been unreachable for ``offline_after_s``; then ``offline``, which the kernel latches."""
        with self._lock:
            err, ok_t, first_t = self._poll_error, self._poll_ok_t, self._first_poll_t
        now = self.clock.now()
        if first_t is None:
            return Health("degraded", "not_polled_yet", now)
        if err is None:
            return None
        since = ok_t if ok_t is not None else first_t
        down_s = now - since
        detail = {"error": err, "down_s": round(down_s, 3)}
        if down_s >= self.offline_after_s:
            return Health("offline", "owner_unreachable" if ok_t is not None else "never_polled", now, detail)
        return Health("degraded", "owner_poll_error", now, detail)


def usb_serial_present(serial: str, sysroot: Path = Path("/sys")) -> bool | None:
    """True/False from sysfs (read-only); None when sysfs is unreadable (not Linux, sandboxed)."""
    devices = sysroot / "bus/usb/devices"
    try:
        entries = list(devices.iterdir())
    except OSError:
        return None
    for dev in entries:
        try:
            if (dev / "serial").read_text().strip() == serial:
                return True
        except OSError:
            continue
    return False
