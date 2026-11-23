"""Fresh start samples.

Seen in sim: a 30 Hz poller against the station's 30 Hz pose stamps kept a cached sample p50 48 / p95 64
/ max 67 ms old at random instants, and 45 % of starts were refused `sample_stale` (acceptance's bound is 50 ms). Now
roborun-rt reads one sample synchronously when a segment arrives (`refresh`, outside its lock), so the start is judged
on a sample taken now.
"""
import random
import statistics
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path

import pytest

from roborun_sdk.connectors.yam import YamStationConnector
from roborun_sdk.rt.clock import Clock

from conftest import Rig

SIM_SRC = Path(__file__).resolve().parents[2] / "roborun-station/src"
if SIM_SRC.is_dir() and str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))


class StaleCache:
    """A sim arm seen through a poller whose cached sample is 60 ms old; refresh() reads the arm now and caches that
    (as PolledOwner does: the cache holds the newest sample seen)."""

    def __init__(self, arm, lock_probe=None, refresh=True):
        self.arm = arm
        self.descriptor = arm.descriptor
        self.refreshes = 0
        self.lock_probe = lock_probe
        self._last = None
        if not refresh:
            self.refresh = None

    def sample(self):
        s = self.arm.sample()
        stale = replace(s, t=s.t - 0.060, sample_id=f"{s.sample_id}:cached")    # an older driver sample
        return self._last if self._last is not None and self._last.t > stale.t else stale

    def refresh(self, timeout_s):
        self.refreshes += 1
        if self.lock_probe is not None:
            self.lock_probe()
        self._last = self.arm.sample()
        return self._last

    def __getattr__(self, name):                 # health, hold, acquire, write, release, recover: the arm's own
        return getattr(self.arm, name)


def _free_elsewhere(lock) -> bool:
    """True when another thread could take the runtime lock right now (refresh runs outside it)."""
    out = []
    t = threading.Thread(target=lambda: out.append(lock.acquire(timeout=0.2) and (lock.release() or True)))
    t.start()
    t.join(1.0)
    return bool(out and out[0])


def test_a_stale_cache_is_refreshed_before_acceptance(pack, clock):
    rig = Rig(pack, clock, settle=False)
    free = []
    cached = StaleCache(rig.world.arms["left"], lock_probe=lambda: free.append(_free_elsewhere(rig.rt._lock)))
    rig.rt.connectors["left"] = cached
    rig.drive(0.5)
    rig.rt.register_observation("obs_fresh", clock.now())
    ack = rig.rt.submit(rig.segment(sid="fresh", plan_basis_obs="obs_fresh"))
    assert ack["result"] == "accepted", ack
    assert cached.refreshes == 1
    assert ack["start_measured"]["left"]["age_ms"] < 50
    assert free == [True]                        # the read ran while the runtime lock was free (the tick never waits)


def test_without_a_fresh_read_the_same_start_is_refused(pack, clock):
    """The earlier behaviour, as a control: the cached sample alone is too old to start from."""
    rig = Rig(pack, clock, settle=False)
    rig.rt.connectors["left"] = StaleCache(rig.world.arms["left"], refresh=False)
    rig.drive(0.5)
    rig.rt.register_observation("obs_stale", clock.now())
    ack = rig.rt.submit(rig.segment(sid="stale", plan_basis_obs="obs_stale"))
    assert ack["result"] == "refused" and ack["reason"] == "sample_stale", ack


def test_refresh_against_a_30hz_station_keeps_the_start_sample_under_50ms():
    """Real time, real HTTP, the shared fake station (30 Hz driver-cycle stamps), the shipped 30 Hz poller. A cached
    sample's age at a random instant is the poll phase plus the stamp phase (up to ~67 ms, depending on how the two
    30 Hz loops happen to line up: 45 % beyond 50 ms in that run); a synchronous read leaves only the stamp phase and
    one local GET, whatever the poller's phase."""
    sim = pytest.importorskip("roborun_station")
    station = sim.FakeYamStation(lease_free_hold=True)
    rng = random.Random(7)
    clock = Clock()
    cached, fresh = [], []
    with sim.SimServer(station) as srv:
        conn = YamStationConnector(srv.url, sysroot=Path("/nonexistent"), clock=clock)
        conn.start()
        try:
            assert conn.wait_first_poll(2.0)
            time.sleep(0.2)
            for _ in range(60):                    # the cache alone (informational: depends on the loops' phase)
                time.sleep(rng.uniform(0.0, 1 / 30))
                s = conn.sample()
                cached.append(clock.now() - s.t + s.clock_uncertainty_s)
            for _ in range(60):                    # a synchronous read at each instant
                time.sleep(rng.uniform(0.0, 1 / 30))
                s = conn.refresh()
                fresh.append(clock.now() - s.t + s.clock_uncertainty_s)
        finally:
            conn.close()
    fresh.sort()
    cached.sort()
    p95 = fresh[int(0.95 * (len(fresh) - 1))]
    info = {"fresh_ms": [round(v * 1000, 1) for v in fresh[-5:]], "cached_ms": [round(v * 1000, 1) for v in cached[-5:]]}
    assert statistics.median(fresh) < 0.030 and p95 < 0.045, info
    assert fresh[-1] < 0.100, info                # one scheduler hiccup on shared CPUs at most, never a stale cache
    assert conn.http.posts() == []                # a fresh read is a GET, never a POST
