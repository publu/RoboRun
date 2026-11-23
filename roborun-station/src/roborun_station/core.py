"""Clock, fault schedule and a joint-space arm model shared by the fake YAM station, Metal owner and cameras.

No hardware, no network, no numpy: everything is deterministic under `SimClock` (tests step time by hand) and also
runs in real time under `RealClock` (the HTTP front ends). Numbers that matter for the SDK match the real arms: 30 Hz
driver cycles, velocity reported in 0.00244 rad/s quanta with at-rest noise of several quanta (the reason the rest
test is a position window), sample ages, the station's 0.04 rad / 0.081 jaw per-answer bounds.

Rigs: `yam_left_spec` / `yam_right_spec` are the dual-YAM pair (the yam profile's Homes; the right
YAM's CAN adapter is pinned as can_right by its USB serial); `metal_spec(arm=...)` is a MakerMods Metal on either
side. `metal_spec(live=True)` (and `FakeMetalOwner(physics="live")`) behaves like the live Metal instead of an ideal follower:
5 deg/s, joints 2/3 stall 0.29-2.1 deg short of their command from sag (folded .. extended), joint 5 stops 0.28 deg short
(its arrival bias), and a jaw that stalls on the cup wall (`jaw_floor`). The ideal
follower stays the default: the SDK's K1 drills judge a hold at 1 mrad and must not change under them.
"""
from __future__ import annotations

import math
import random
import threading
import time
from dataclasses import dataclass, field

VELOCITY_QUANTUM = 0.0024420024420024333        # rad/s per count, as both arms report it


class SimClock:
    """A monotonic clock that only moves when told to (deterministic tests)."""

    def __init__(self, t0: float = 434760.0):
        self._t = float(t0)
        self._lock = threading.Lock()

    def __call__(self) -> float:
        with self._lock:
            return self._t

    def advance(self, dt: float) -> float:
        if dt < 0:
            raise ValueError("a monotonic clock never goes back")
        with self._lock:
            self._t += dt
            return self._t


class RealClock:
    def __call__(self) -> float:
        return time.monotonic()


@dataclass
class Window:
    start: float
    end: float
    params: dict = field(default_factory=dict)

    def active(self, now: float) -> bool:
        return self.start <= now < self.end


class Faults:
    """A schedule of injected faults, each a time window on the owner's clock.

    kinds and their parameters:
      stale(arm, extra_s)          samples are stamped `extra_s` in the past (they arrive old)
      freeze(arm)                  the driver cycle stops: the newest sample just ages
      gc_pause()                   the owner process is paused: no ticks, replies wait until the pause ends
      link_loss()                  the owner's HTTP port refuses / resets every request
      stall(arm)                   the joints stop following the command (blocked by contact)
      usb_drop(arm)                the adapter vanished: health fault, no samples, commands dropped
      dfu(arm)                     the adapter re-enumerates in DFU (bootloader) mode: identity mismatch, no samples
      slow_reply(route, delay_s)   one route answers late
      camera_lost(camera)          a camera stops delivering frames
    `forever` is a window that never ends; `clear(kind)` removes a fault.
    """
    forever = 1e18

    def __init__(self):
        self._items: list[tuple[str, dict, Window]] = []
        self._lock = threading.Lock()

    def add(self, kind: str, start: float, duration: float | None = None, **params) -> Window:
        win = Window(start, self.forever if duration is None else start + duration, params)
        with self._lock:
            self._items.append((kind, params, win))
        return win

    def clear(self, kind: str | None = None):
        with self._lock:
            self._items = [i for i in self._items if kind is not None and i[0] != kind]

    def active(self, kind: str, now: float, **match) -> dict | None:
        with self._lock:
            for k, params, win in self._items:
                if k == kind and win.active(now) and all(params.get(a) == b for a, b in match.items()):
                    return dict(params, until=win.end)
        return None

    def pause_until(self, now: float) -> float | None:
        hit = self.active("gc_pause", now)
        return None if hit is None else hit["until"]


@dataclass
class ArmSpec:
    name: str                                     # rig-local: "left" / "right"
    body: str                                     # "yam" / "metal"
    home: tuple[float, ...]
    limits: tuple[tuple[float, float], ...]
    joint_speed_rad_s: float                      # how fast the simulated joints follow their command
    jaw_open: float = 1.0
    jaw_closed: float = 0.0
    jaw_speed: float = 1.0                        # jaw units per second
    control_hz: float = 30.0
    noise_rad: float = 0.0003                     # at-rest position noise (encoder + PD dither)
    velocity_noise_counts: int = 8                # at-rest |dq| reads up to this many quanta
    clock_uncertainty_s: float = 6.9e-7
    timestamp_source: str = "driver_state_cycle"
    adapter: str = ""
    drops_when_limp: bool = True
    slump_rad_s: float = 0.25                     # how fast a limp arm falls toward `slump`
    slump: tuple[float | None, ...] | None = None   # where a let-go joint falls to (None: it stays)
    temp_c: float = 35.0
    stop_decel_rad_s2: float = 6.0                # a stopped joint coasts v^2/2a: 30 mrad from 0.6 rad/s, 2 cycles
    # live-arm behaviour (all off by default): `shortfall(q, joint) -> rad` stops a joint that far short of its command (sag /
    # stiction stalls); `jaw_floor() -> jaw value or None` is where an object between the pads stops the closing jaw.
    shortfall: object = None
    jaw_floor: object = None
    jaw_stall_effort: float = 0.30                # what the jaw motor reads while it presses on an object (Nm on the Metal)


def yam_left_spec(**kw) -> ArmSpec:
    base = dict(name="left", body="yam", home=(1.5967, -0.0013, 0.004, -0.0196, 0.0269, 0.2462),
                limits=((-2.61799, 3.05433), (0.0, 3.65), (0.0, 3.14159), (-1.5708, 1.5708), (-1.5708, 1.5708),
                        (-2.0944, 2.0944)),
                joint_speed_rad_s=0.6, jaw_speed=0.6, adapter="slcan://SIMYAMLEFT01",
                slump=(None, 0.35, 0.55, -0.3, None, None))          # a let-go arm sags at the elbow and wrist
    base.update(kw)
    return ArmSpec(**base)


def yam_right_spec(**kw) -> ArmSpec:
    """The dual-YAM rig's right arm (the yam profile's Home; its CAN adapter pinned as can_right by USB serial)."""
    base = dict(name="right", body="yam", home=(1.6016, -0.0002, -0.001, -0.0887, 0.0284, -0.0273),
                limits=((-2.61799, 3.05433), (0.0, 3.65), (0.0, 3.14159), (-1.5708, 1.5708), (-1.5708, 1.5708),
                        (-2.0944, 2.0944)),
                joint_speed_rad_s=0.6, jaw_speed=0.6, adapter="can_right:SIMYAMRIGHT1",
                slump=(None, 0.35, 0.55, -0.3, None, None))
    base.update(kw)
    return ArmSpec(**base)


METAL_HOME = (1.3668709630013174, -0.0004791332875564436, -0.025585717555504317, 0.1507353322652012, -0.05634607461661689,
              0.10224704356450687)                                  # profiles/yam_metal/hardware.json right.home.positions_rad
METAL_SERIAL = "SIMMETAL0001"                                       # the simulated right Metal's candleLight adapter
METAL_SPEED_RAD_S = math.radians(5)                                 # metal_controller.SPEED: the Metal moves at 5 deg/s
METAL_SAG_SHORT_DEG = (0.29, 2.1)                                   # joints 2/3 stall this short: folded .. fully extended
METAL_SAG_JOINTS = (1, 2)                                           # 0-based: joints 2 and 3 carry the arm's weight
METAL_J5_ARRIVAL_DEG = 0.28                                         # joint 5 stops short of its command (stiction)
METAL_JAW_ON_CUP = 0.045                                            # normalized opening where a rim pinch stops the jaw
METAL_START_SLACK_DEG = 5.0                                         # metal_controller.START_SLACK beyond the URDF limits


def metal_extension(q) -> float:
    """How far out the Metal reaches, 0 at its folded Home (joints 2 and 3 on their 0-deg limits) to 1 at a full reach-down:
    (|q2| + |q3|) / 120 deg. The sag torque, and so the stall, grows with it."""
    return max(0.0, min(1.0, (abs(q[1]) + abs(q[2])) / math.radians(120)))


def metal_shortfall_rad(q, joint, *, sag_deg=METAL_SAG_SHORT_DEG, j5_deg=METAL_J5_ARRIVAL_DEG) -> float:
    """How far short of its command joint `joint` (0-based) of a live Metal stops: joints 2/3 0.29-2.1 deg with extension (the
    live jog stalls), joint 5 0.28 deg (its arrival bias), the rest 0."""
    if joint in METAL_SAG_JOINTS:
        lo, hi = sag_deg
        return math.radians(lo + (hi - lo) * metal_extension(q))
    if joint == 4:
        return math.radians(j5_deg)
    return 0.0


def metal_spec(arm: str = "right", *, live: bool = False, serial: str = METAL_SERIAL, channel: str = "can0",
               jaw_floor=None, **kw) -> ArmSpec:
    """A MakerMods Metal on either side of a rig. live=True: the live arm's sag / joint-5 stalls and a jaw that stops on the
    object `jaw_floor()` reports (None: nothing between the pads)."""
    d = math.radians
    slack = METAL_START_SLACK_DEG     # the owner's verified envelope: URDF limits + START_SLACK (its Home j3 sits at -1.47 deg)
    base = dict(name=arm, body="metal", home=METAL_HOME,
                limits=((d(-160 - slack), d(160 + slack)), (d(-180 - slack), d(0 + slack)), (d(0 - slack), d(180 + slack)),
                        (d(-123 - slack), d(81 + slack)), (d(-85 - slack), d(85 + slack)), (d(-145 - slack), d(145 + slack))),
                joint_speed_rad_s=METAL_SPEED_RAD_S, jaw_open=1.9413522545, jaw_closed=0.1009054704, jaw_speed=0.20,
                timestamp_source="can_query_roundtrip", clock_uncertainty_s=0.0,
                adapter=f"{channel}:{serial}", drops_when_limp=True)
    if live:
        base.update(shortfall=metal_shortfall_rad, jaw_floor=jaw_floor)
    base.update(kw)
    return ArmSpec(**base)


def metal_right_spec(**kw) -> ArmSpec:
    return metal_spec("right", **kw)


class SimArm:
    """Joints and jaw that follow their commands at a bounded speed, sampled once per driver cycle."""

    def __init__(self, spec: ArmSpec, clock, faults: Faults, *, seed: int = 0):
        self.spec, self.clock, self.faults = spec, clock, faults
        self.rng = random.Random(seed)
        self.q = list(spec.home)
        self.q_cmd = list(spec.home)
        self.jaw = spec.jaw_open
        self.jaw_cmd = spec.jaw_open
        self.dq = [0.0]*len(spec.home)
        self.jaw_effort = 0.0
        self.jaw_blocked = False
        self.stalled: list[int] = []                  # joints stopped short of their command (live arms)
        self.temps = [spec.temp_c]*7
        self.powered = True                           # torque on; False = let go (limp)
        self.seq = 0
        self.last_row: dict | None = None
        self.last_step: float | None = None
        self.lock = threading.RLock()
        self._sample(self.clock())

    # -- physics ------------------------------------------------------------------------------------------------
    def step(self, now: float):
        with self.lock:
            dt = 0.0 if self.last_step is None else max(0.0, now - self.last_step)
            self.last_step = now
            if dt <= 0:
                return
            dropped = (self.faults.active("usb_drop", now, arm=self.spec.name) is not None
                       or self.faults.active("dfu", now, arm=self.spec.name) is not None)
            moved = [0.0]*len(self.q)
            if not self.powered and self.spec.drops_when_limp and self.spec.slump is not None:
                target = [q if s is None else s for q, s in zip(self.q, self.spec.slump)]
                speed = self.spec.slump_rad_s
            elif self.faults.active("stall", now, arm=self.spec.name) is not None:
                target, speed = self.q, 0.0
            else:
                target, speed = self.q_cmd, self.spec.joint_speed_rad_s
            short = self.spec.shortfall if target is self.q_cmd else None
            stalled = []
            # A dropped adapter stops the bus: the motors keep their last target (power stays on), so they hold.
            for i, (a, b) in enumerate(zip(self.q, target)):
                if short is not None:
                    s = float(short(self.q, i))
                    if s > 0.0:
                        if abs(b - a) <= s + 1e-12:     # sag / stiction: the joint cannot close the last `s` of its error
                            if abs(b - a) > 1e-9:
                                stalled.append(i)
                            b = a
                        else:
                            b = b - math.copysign(s, b - a)
                step = max(-speed*dt, min(speed*dt, b - a))
                self.q[i] = a + step
                moved[i] = step/dt
            self.stalled = stalled
            if self.powered:
                jaw_target = self.jaw_cmd
                floor = self.spec.jaw_floor() if callable(self.spec.jaw_floor) else None
                blocked = floor is not None and jaw_target < floor <= self.jaw + 1e-9
                if blocked:
                    jaw_target = floor                   # the pads stop on the object's wall
                jstep = max(-self.spec.jaw_speed*dt, min(self.spec.jaw_speed*dt, jaw_target - self.jaw))
                self.jaw += jstep
                self.jaw_blocked = bool(blocked and abs(self.jaw - floor) < 1e-6)
                self.jaw_effort = self.spec.jaw_stall_effort if self.jaw_blocked else 0.0
            self.dq = moved
            if not dropped and self.faults.active("freeze", now, arm=self.spec.name) is None:
                self._sample(now)

    def _sample(self, now: float):
        stale = self.faults.active("stale", now, arm=self.spec.name)
        stamp = now - (stale["extra_s"] if stale else 0.0)
        noise = [self.rng.uniform(-self.spec.noise_rad, self.spec.noise_rad) for _ in self.q]
        counts = self.spec.velocity_noise_counts
        dq = [round((v + self.rng.randint(-counts, counts)*VELOCITY_QUANTUM*(abs(v) < 1e-9)) / VELOCITY_QUANTUM)
              * VELOCITY_QUANTUM for v in self.dq]
        self.seq += 1
        sample_id = (f"{1794730574.0 + stamp:.9f}" if self.spec.body == "yam"
                     else f"metal:{self.spec.adapter.split(':')[-1]}:{stamp:.9f}")
        self.last_row = {
            "body": self.spec.body, "positions_rad": [a+b for a, b in zip(self.q, noise)], "gripper": self.jaw,
            "commanded_rad": list(self.q_cmd), "commanded_gripper": self.jaw_cmd, "velocities_rad_s": dq,
            "motor_temps_c": list(self.temps), "source_monotonic_s": stamp, "sample_id": sample_id,
            "timestamp_source": self.spec.timestamp_source, "clock_uncertainty_s": self.spec.clock_uncertainty_s,
            "seq": self.seq}
        if self.spec.jaw_floor is not None or self.spec.shortfall is not None:
            self.last_row.update(jaw_effort=self.jaw_effort, jaw_blocked=self.jaw_blocked, stalled_joints=list(self.stalled))

    # -- reads and writes ---------------------------------------------------------------------------------------
    def row(self, now: float) -> dict | None:
        with self.lock:
            if self.last_row is None:
                return None
            r = dict(self.last_row)
            age = now - r["source_monotonic_s"]
            r.update(age_ms=round(age*1000, 3), fresh=0 <= age + r["clock_uncertainty_s"] < 0.050)
            return r

    def command(self, q=None, jaw=None):
        with self.lock:
            if q is not None:
                self.q_cmd = [max(lo, min(hi, float(v))) for v, (lo, hi) in zip(q, self.spec.limits)]
            if jaw is not None:
                lo, hi = sorted((self.spec.jaw_closed, self.spec.jaw_open))
                self.jaw_cmd = max(lo, min(hi, float(jaw)))

    def hold_here(self, urgent=True, keep_jaw=False):
        """An owner's stop: the command becomes the current position plus the distance each joint coasts while it
        decelerates (v^2 / 2a in its direction of travel), so a moving arm takes a few driver cycles to settle, as a
        real one does under PD; a still arm holds exactly where it is. The jaw holds where it is.
        urgent=False: the commands simply ended, nobody asked for a stop (arms that can tell the two apart run out more gently).
        keep_jaw: the jaw finishes its last command: a slower jaw motor is still on its way when the motion's last row has been sent."""
        with self.lock:
            a = self.spec.stop_decel_rad_s2
            coast = [v*abs(v)/(2*a) if a > 0 else 0.0 for v in self.dq]
            self.q_cmd = [max(lo, min(hi, q + c)) for q, c, (lo, hi) in zip(self.q, coast, self.spec.limits)]
            if not keep_jaw:
                self.jaw_cmd = self.jaw

    def at(self, q, tol=1e-3) -> bool:
        return max(abs(a-b) for a, b in zip(self.q, q)) <= tol

    def moving(self, tol=1e-4) -> bool:
        return max(abs(v) for v in self.dq) > tol


def finite(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
