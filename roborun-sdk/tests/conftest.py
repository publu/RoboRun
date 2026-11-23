"""Shared fixtures. Every suite runs under ``unshare -rn`` (no network); fake owners bind 127.0.0.1 inside it."""
import atexit
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[1] / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

# The simulated station beside this package, for suites that run it in-process. Appended, so the SDK's own source wins.
_STATION = Path(__file__).resolve().parents[2] / "roborun-station" / "src"
if _STATION.is_dir() and str(_STATION) not in sys.path:
    sys.path.append(str(_STATION))

# No test may reach this computer's systemd user manager. `roborun up` / `down` / `station` manage units with fixed names
# (roborun-station, roborun-robot, ...), so a test that ran them for real could stop, and park, a robot already running
# under this user, even with its own HOME. Every test, and every process it starts, sees no session bus, a private
# XDG_RUNTIME_DIR and systemctl / systemd-run stand-ins that print nothing on stdout and fail, so Roborun falls back to
# plain background processes (as on macOS).
_NO_SYSTEMD = Path(tempfile.mkdtemp(prefix="roborun-tests-"))
atexit.register(shutil.rmtree, _NO_SYSTEMD, True)
(_NO_SYSTEMD / "bin").mkdir()
(_NO_SYSTEMD / "run").mkdir(mode=0o700)
for _tool in ("systemctl", "systemd-run"):
    (_NO_SYSTEMD / "bin" / _tool).write_text('#!/bin/sh\necho "roborun tests: systemd is off" >&2\nexit 1\n')
    (_NO_SYSTEMD / "bin" / _tool).chmod(0o755)
os.environ.pop("DBUS_SESSION_BUS_ADDRESS", None)
os.environ["XDG_RUNTIME_DIR"] = str(_NO_SYSTEMD / "run")
os.environ["PATH"] = str(_NO_SYSTEMD / "bin") + os.pathsep + os.environ.get("PATH", "")
# Machine-wide state lives in the temp folder: one owner per CAN bus (arms/canbus.py) and the arms-in-use file
# (arms/safety.py). The fake adapters reuse the same serials, so test processes running side by side (pytest -n auto) would
# see each other's claims. Each test process gets its own temp folder; its children inherit it.
# Short, because Unix socket paths under it (pytest's tmp_path included) must stay below 104 bytes: macOS's own temp folder
# (/var/folders/...) is too long for that.
_TMP = tempfile.mkdtemp(prefix="rr-", dir="/tmp" if sys.platform == "darwin" else None)
atexit.register(shutil.rmtree, _TMP, True)
os.environ["TMPDIR"] = _TMP
tempfile.tempdir = _TMP

from roborun_sdk import dev  # noqa: E402
from roborun_sdk.connectors.sim import SimWorld, drive  # noqa: E402
from roborun_sdk.rt.canonical import from_jsonable  # noqa: E402
from roborun_sdk.rt.clock import SimClock  # noqa: E402
from roborun_sdk.rt.runtime import Runtime  # noqa: E402

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"
FIXTURES = Path(__file__).resolve().parent / "data"      # signed sim segments (DEV keys)


@pytest.fixture(autouse=True)
def _no_real_owner_urls(monkeypatch):
    """Belt and braces: even inside unshare, a test that forgets its fake base_url fails instead of reaching :8091/:8093."""
    import roborun_sdk.connectors.local_http as lh
    real = lh.LocalJSON.__init__

    def guarded(self, base_url, **kw):
        if base_url.rstrip("/").endswith((":8091", ":8093", ":8100")):
            raise AssertionError(f"test tried to reach a real owner: {base_url}")
        real(self, base_url, **kw)

    monkeypatch.setattr(lh.LocalJSON, "__init__", guarded)


@pytest.fixture(autouse=True)
def _gc_restored():
    """A runtime abandoned mid-segment in a test leaves automatic gc off (rt disables it while a segment runs)."""
    import gc
    yield
    gc.enable()


@pytest.fixture
def pack():
    return dev.sim_pack()


@pytest.fixture
def clock():
    return SimClock()


def example(name: str) -> dict:
    return from_jsonable(json.loads((FIXTURES / f"{name}.segment.json").read_text()))


class Rig:
    """A sim world + runtime, linked, armed and settled (rest window filled)."""

    def __init__(self, pack, clock, *, left_q=None, left_jaw=0.999, right_q=None, settle=True, **rt_kw):
        self.pack, self.clock = pack, clock
        self.world = SimWorld.yam_metal(clock, left_q=left_q, left_jaw=left_jaw, right_q=right_q)
        kw = dict(armed_local=True, armed_server=True)
        kw.update(rt_kw)
        self.rt = Runtime(self.world.arms, pack, clock=clock, **kw)
        self.rt.session_started("test")
        if settle:
            self.drive(0.5)
        self.basis = "obs_sim"                       # the plan basis rig.segment() names by default
        self._obs_n = 0
        self.rt.register_observation("obs_sim", clock.now())
        self.drive(0.04)

    def observe(self, name=None):
        """A fresh observation (ids are unique per capture: re-registering an old id never makes it newer), used as
        the default plan basis from now on."""
        self._obs_n += 1
        self.basis = name or f"obs_sim_{self._obs_n}"
        self.rt.register_observation(self.basis, self.clock.now())
        return self.basis

    @property
    def left(self):
        return self.world.arms["left"]

    @property
    def right(self):
        return self.world.arms["right"]

    def drive(self, seconds, **kw):
        return drive(self.rt, self.world, seconds, **kw)

    def run_to_end(self, seconds=20.0):
        return self.drive(seconds, until=lambda r: r.playback is not None and r.playback.state == "ended")

    def segment(self, rows=None, *, arm="left", delta=0.03, seconds=1.0, **kw):
        if rows is None:
            a = self.world.arms[arm]
            n = int(round(seconds * 30)) + 1
            s = np.linspace(0, 1, n)
            rows = np.tile(np.r_[a.q, a.jaw], (n, 1))
            rows[:, 0] += delta * s * s * (3 - 2 * s)
        ap = self.pack.arm(arm)
        kw.setdefault("pack_digest", self.pack.digest)
        kw.setdefault("plan_basis_obs", self.basis)
        kw.setdefault("limits", {"ee_speed_m_s": 0.10})
        kw.setdefault("jaw", float(rows[0][-1]))
        return dev.make_segment(rows, arm=arm, semantics=ap.semantics, **kw)

    def events(self, kind=None):
        evs = self.rt.drain_events()
        return [e for e in evs if kind is None or e.get("kind") == kind or e.get("type") == kind]


@pytest.fixture
def rig(pack, clock):
    return Rig(pack, clock)
