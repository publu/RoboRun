"""Minimal in-process sim arms for tests and ``roborun run`` (sim is the default).

``roborun-station`` is the shared fake with fault injection; these lighter fakes implement the connector protocol: first-order tracking of the commanded
target, quantised velocity readings, jaw readings that jitter by about one encoder quantum (JAW_READ_NOISE), driver-cycle
stamps on the given clock, and injectable faults (offline, stale
stamps, hold that hangs or refuses, heat, an object blocking the jaw).
No network, no hardware.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

from ..rt.datatypes import ArmDescriptor, ArmSample, ConnectorError, Grant, Health, HoldAck, OperatorConfirm
from . import metal as metal_conn
from . import yam as yam_conn

VEL_QUANTUM = 0.00244   # rad/s, the YAM's velocity quantisation
# The jaw reading jitters too: in 404 recorded at-rest windows the YAM jaw's range over the window is 0 in 13 %, one quantum (7.2e-5) in 78 % and two quanta in
# 8 %. A sim that read the jaw exactly hid an exact-equality bug in the settle phase.
JAW_READ_NOISE = 7.3e-5


class SimArm:
    def __init__(self, descriptor: ArmDescriptor, q0, jaw0: float, clock, *, tau_s: float = 0.05,
                 temps_c: float = 40.0, body: str = "yam", noise_rad: float = 0.0002, seed: int = 0,
                 jaw_noise: float = JAW_READ_NOISE):
        self.descriptor = descriptor
        self.clock = clock
        self.tau_s = tau_s
        self.body = body
        self.q = np.asarray(q0, float).copy()
        self.q_cmd = self.q.copy()
        self.dq = np.zeros_like(self.q)
        self.jaw = float(jaw0)
        self.jaw_cmd = float(jaw0)
        self.temps = [float(temps_c)] * 7
        self.lease: Grant | None = None
        self.status = "holding"
        self.stamp = clock.now()
        self.counter = 0
        self.faults: dict = {}
        self.noise_rad = noise_rad
        self._rng = np.random.default_rng(seed)
        self._noise = np.zeros_like(self.q)
        self.jaw_noise = jaw_noise                     # +- jaw reading jitter (its own stream: joint noise unchanged)
        self._jaw_rng = np.random.default_rng([seed, 1])
        self._jaw_read_noise = 0.0
        self.writes: list = []
        self.holds: list = []
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ physics
    def step(self, dt: float) -> None:
        with self._lock:
            if self.faults.get("offline"):
                return
            a = 1.0 - math.exp(-dt / self.tau_s)
            new_q = self.q + (self.q_cmd - self.q) * a
            self.dq = np.round((new_q - self.q) / dt / VEL_QUANTUM) * VEL_QUANTUM
            self.q = new_q
            self.jaw += (self.jaw_cmd - self.jaw) * a
            block = self.faults.get("jaw_block")    # an object in the jaw: the measured jaw stops at its width while
            if block is not None:                   # the command squeezes past it (both arms close toward smaller values)
                self.jaw = max(self.jaw, float(block))
            if not self.faults.get("frozen_stamps"):
                self.stamp = self.clock.now()
                self.counter += 1
                if self.noise_rad:
                    self._noise = self._rng.uniform(-self.noise_rad, self.noise_rad, size=self.q.shape)
                if self.jaw_noise:
                    self._jaw_read_noise = float(self._jaw_rng.uniform(-self.jaw_noise, self.jaw_noise))

    # ------------------------------------------------------------------ connector protocol
    def sample(self) -> ArmSample:
        with self._lock:
            if self.faults.get("offline"):
                raise ConnectorError(f"{self.descriptor.name}: sim owner offline")
            source = "driver_state_cycle" if self.body == "yam" else "can_query_roundtrip"
            jaw_read = self.jaw + self._jaw_read_noise     # the jaw reading jitters as well (JAW_READ_NOISE)
            jaw_frac = jaw_read if self.body == "yam" else None
            q_read = self.q + self._noise                  # encoder noise at rest, as on the real arms
            raw = {"body": self.body, "positions_rad": q_read.tolist(), "commanded_rad": self.q_cmd.tolist(),
                   "source_monotonic_s": self.stamp, "clock_uncertainty_s": 0.0005 if self.body == "yam" else 0.0,
                   "timestamp_source": source, "sample_id": f"sim:{self.descriptor.name}:{self.counter}",
                   "gripper": jaw_frac, "motor_temps_c": list(self.temps)}
            return ArmSample(arm=self.descriptor.name, sample_id=raw["sample_id"], t=self.stamp,
                             clock_uncertainty_s=raw["clock_uncertainty_s"], q=tuple(q_read.tolist()),
                             q_cmd=tuple(self.q_cmd.tolist()), dq=tuple(self.dq.tolist()), jaw=jaw_read,
                             jaw_cmd=self.jaw_cmd, effort=(), effort_fresh=False, temps_c=tuple(self.temps),
                             status=self.status, source=source, raw=raw)

    def health(self) -> Health:
        now = self.clock.now()
        if self.faults.get("offline"):
            return Health("offline", self.faults.get("cause", "motor_comm_lost"), now)
        if self.faults.get("fault"):
            return Health("fault", self.faults["fault"], now)
        return Health("ok", None, now)

    def acquire(self, grant: Grant) -> None:
        if self.faults.get("acquire_refused"):
            raise ConnectorError("sim owner refused the lease")
        with self._lock:
            self.lease = grant
            self.status = "moving"

    def write(self, target, t: float) -> None:
        target = np.asarray(target, float)
        if target.shape != (len(self.q) + 1,) or not np.isfinite(target).all():
            raise ConnectorError("bad target")
        with self._lock:
            if self.lease is None:
                raise ConnectorError("no lease: the owner ignores writes without a grant")
            self.q_cmd = target[:-1].copy()
            self.jaw_cmd = float(target[-1])
            self.writes.append((t, target.copy()))

    def hold(self, reason: str) -> HoldAck:
        started = time.perf_counter()
        hang = self.faults.get("hold_hang_s")
        if hang:
            time.sleep(hang)                       # wall time: the K1 budget is wall time
        with self._lock:
            self.holds.append(reason)
            if self.faults.get("hold_refuse"):
                return HoldAck(self.descriptor.name, False, None, time.perf_counter() - started, True,
                               {"error": "sim owner refused"})
            self.lease = None
            self.status = "holding"                # the owner got a hold: it keeps its last target while it runs
        return HoldAck(self.descriptor.name, True, self.sample() if not self.faults.get("offline") else None,
                       time.perf_counter() - started, True, {"reason": reason})

    def release(self) -> None:
        with self._lock:
            self.lease = None
            self.status = "holding"

    def recover(self, confirm: OperatorConfirm) -> dict:
        if not isinstance(confirm, OperatorConfirm):
            raise ConnectorError("operator confirmation required")
        self.faults.clear()
        return {"recovered": True}

    def inject(self, **faults) -> None:
        self.faults.update(faults)


class SimWorld:
    def __init__(self, arms: dict, clock):
        self.arms = arms
        self.clock = clock

    def step(self, dt: float) -> None:
        for arm in self.arms.values():
            arm.step(dt)

    def settle(self, seconds: float, dt: float = 1 / 30) -> None:
        """Advance the virtual clock and the arms together (history for the rest test)."""
        for _ in range(int(round(seconds / dt))):
            self.clock.advance(dt)
            self.step(dt)

    @classmethod
    def yam_metal(cls, clock, *, left_q=None, left_jaw: float = 0.999, right_q=None, right_jaw: float = 1.9) -> "SimWorld":
        """Left YAM at the sample insert's start; right Metal at its Home (hardware.json)."""
        left_q = left_q if left_q is not None else [1.5432, 1.7448, 0.2768, 1.5111, -0.0572, 0.0184]
        right_q = right_q if right_q is not None else [1.3669, -0.0005, -0.0256, 0.1507, -0.0563, 0.1022]
        return cls({"left": SimArm(yam_conn.descriptor("left"), left_q, left_jaw, clock, body="yam"),
                    "right": SimArm(metal_conn.descriptor("right"), right_q, right_jaw, clock, body="metal")}, clock)


def drive(runtime, world: SimWorld, seconds: float, *, feed_link: bool = True, until=None) -> int:
    """Run the runtime in virtual time: per tick advance the clock, step the arms, (heartbeat), tick."""
    dt = runtime.dt
    ticks = int(round(seconds / dt))
    for i in range(ticks):
        world.clock.advance(dt)
        world.step(dt)
        if feed_link:
            runtime.link_heartbeat()
        runtime.tick()
        if until is not None and until(runtime):
            return i + 1
    return ticks
