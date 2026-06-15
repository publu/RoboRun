"""The fleet communication model — the same assumptions the browser sandbox
runs (roborun/web/fleet.js), as plain, runnable Python.

Three things make multi-robot coordination hard, and all three are knobs here:

* **radio range**   — two robots can exchange a message only while within
                      ``range`` metres. Past it, they're on their own.
* **airtime**       — a robot may send at most ``airtime`` messages per second
                      (its slice of a shared band); the rest wait.
* **onboard memory**— a robot remembers at most ``memory`` mapped cells and a
                      backlog of at most ``inbox`` unread messages; overflow is
                      dropped, not magically kept.

And the robot can chase **one goal at a time** — it commits to a cell and only
re-tasks once it arrives. A strategy is just a policy over this model; see
``strategies.py``. This module is deliberately framework-free so it runs
headless (``python -m roborun.swarm``) and in CI.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

WORLD = 56.0           # metres across the square field
CG = 28                # coverage cells per side
CELL = WORLD / CG
SENSE = 3.2            # metres mapped around a robot as it walks
NCELL = CG * CG


def cell_xy(c: int) -> tuple[float, float]:
    return (c % CG + 0.5) * CELL, (c // CG + 0.5) * CELL


@dataclass
class Message:
    kind: str               # "cells" | "claim" | "request" | "assign"
    sender: int
    payload: dict = field(default_factory=dict)


@dataclass
class Robot:
    id: int
    x: float
    y: float
    color: str = "#00d47e"
    speed: float = 3.0
    heading: float = math.pi / 2
    goal: int = -1                                   # cell index, -1 = free
    known: set = field(default_factory=set)          # cells I believe are mapped
    order: list = field(default_factory=list)        # eviction order for memory
    fresh: list = field(default_factory=list)        # mapped since last broadcast
    inbox: list = field(default_factory=list)
    budget: float = 0.0
    is_leader: bool = False
    assigned: int = -1
    sent: int = 0
    payload: set = field(default_factory=set)        # data points carried, not yet at base

    # ── the robot's view of itself, used by the strategies ──────────────
    def arrived(self) -> bool:
        if self.goal < 0:
            return True
        gx, gy = cell_xy(self.goal)
        return math.hypot(gx - self.x, gy - self.y) < CELL * 0.6

    def just_mapped(self) -> list:
        f, self.fresh = self.fresh, []
        return f

    def note_mapped(self, cells) -> None:
        for c in (cells if hasattr(cells, "__iter__") else [cells]):
            self._remember(c)

    def _remember(self, c: int, fresh: bool = False) -> None:
        if c in self.known:
            return
        self.known.add(c)
        self.order.append(c)
        if fresh:
            self.fresh.append(c)


class Fleet:
    """Owns the field, the robots, and the imperfect radio between them."""

    def __init__(self, count=6, range_m=14.0, airtime=4.0, memory=60,
                 inbox=8, reliability=0.85, seed=0, targets=8, base=True):
        self.range = float(range_m)
        self.airtime = float(airtime)
        self.memory = int(memory)
        self.inbox_cap = int(inbox)
        self.reliability = float(reliability)
        self.rng = random.Random(seed)
        self.covered_by = [0] * NCELL          # bitmask of robots that mapped cell
        self.claims: dict[int, tuple[int, float]] = {}   # cell -> (owner, dist)
        self.metrics = dict(sent=0, recv=0, drop=0, redundant=0,
                            data_found=0, data_delivered=0)
        self.t = 0.0
        # the base station (the "main server") sits at one edge; robots deploy
        # beside it and must relay discovered data back to it, hop by hop
        self.base = bool(base)
        self.base_xy = (WORLD * 0.5, WORLD * 0.055)
        self.base_delivered: set = set()
        self.targets = []                       # data points to find: [x, y, found]
        for _ in range(int(targets)):
            self.targets.append([4 + self.rng.random() * (WORLD - 8),
                                 WORLD * 0.25 + self.rng.random() * (WORLD * 0.7), False])
        self.robots = []
        for i in range(count):
            x = WORLD * 0.5 + (self.rng.random() - 0.5) * 8
            y = WORLD * 0.12 + (self.rng.random() - 0.5) * 6
            self.robots.append(Robot(id=i, x=x, y=y))
        self._sense()

    # ── coverage / memory ──────────────────────────────────────────────
    def _sense(self) -> None:
        rad2 = SENSE * SENSE
        for r in self.robots:
            for c in self._cells_near(r.x, r.y, SENSE):
                bit = 1 << r.id
                if not (self.covered_by[c] & bit):
                    if self.covered_by[c]:
                        self.metrics["redundant"] += 1   # someone already had it
                    self.covered_by[c] |= bit
                r._remember(c, fresh=True)
            for tid, tg in enumerate(self.targets):   # discover data points in range
                if (tg[0] - r.x) ** 2 + (tg[1] - r.y) ** 2 <= rad2:
                    if not tg[2]:
                        tg[2] = True
                        self.metrics["data_found"] += 1
                    if self.base and tid not in self.base_delivered:
                        r.payload.add(tid)
            self._evict(r)

    def _evict(self, r: Robot) -> None:
        while len(r.order) > self.memory:           # finite memory forgets ground
            old = r.order.pop(0)
            r.known.discard(old)

    @staticmethod
    def _cells_near(x, y, rad):
        r2 = rad * rad
        gx0, gx1 = max(0, int((x - rad) / CELL)), min(CG - 1, int((x + rad) / CELL))
        gy0, gy1 = max(0, int((y - rad) / CELL)), min(CG - 1, int((y + rad) / CELL))
        out = []
        for gy in range(gy0, gy1 + 1):
            for gx in range(gx0, gx1 + 1):
                c = gy * CG + gx
                cxx, cyy = cell_xy(c)
                if (cxx - x) ** 2 + (cyy - y) ** 2 <= r2:
                    out.append(c)
        return out

    # ── radio ──────────────────────────────────────────────────────────
    def neighbors(self, r: Robot):
        out = []
        for o in self.robots:
            if o is r:
                continue
            d = math.hypot(o.x - r.x, o.y - r.y)
            if d <= self.range:
                out.append((o, d))
        return out

    def _deliver(self, frm: Robot, to: Robot, msg: Message, d: float) -> None:
        p = self.reliability * (1 - (d / self.range) ** 2)   # lossy toward the edge
        if self.rng.random() >= p:
            self.metrics["drop"] += 1
            return
        if len(to.inbox) >= self.inbox_cap:                  # inbox overflow
            self.metrics["drop"] += 1
            return
        to.inbox.append(msg)
        self.metrics["recv"] += 1

    def broadcast(self, r: Robot, kind: str, **payload) -> bool:
        if r.budget < 1:
            return False
        r.budget -= 1
        r.sent += 1
        self.metrics["sent"] += 1
        for o, d in self.neighbors(r):
            self._deliver(r, o, Message(kind, r.id, payload), d)
        return True

    def unicast(self, r: Robot, to: Robot, kind: str, **payload) -> bool:
        if r.budget < 1:
            return False
        r.budget -= 1
        r.sent += 1
        self.metrics["sent"] += 1
        d = math.hypot(to.x - r.x, to.y - r.y)
        if d <= self.range:
            self._deliver(r, to, Message(kind, r.id, payload), d)
        return True

    def recv(self, r: Robot, kind: str | None = None):
        """Drain the inbox (limited per tick by airtime — backlog overflows)."""
        proc = max(2, math.ceil(self.airtime))
        out, keep = [], []
        for m in r.inbox[:proc]:
            (out if (kind is None or m.kind == kind) else keep).append(m)
        r.inbox = keep + r.inbox[proc:]
        return out

    # ── goal helpers used by the strategies ────────────────────────────
    def nearest_unknown(self, r: Robot, avoid_claims=False) -> int:
        best, bd = -1, 1e9
        for c in range(NCELL):
            if c in r.known:
                continue
            cxx, cyy = cell_xy(c)
            d = math.hypot(cxx - r.x, cyy - r.y)
            if avoid_claims:
                cl = self.claims.get(c)
                if cl and cl[0] != r.id and cl[1] <= d:
                    continue
            if d < bd:
                best, bd = c, d
        if best < 0 and r.order:
            best = r.order[self.rng.randrange(len(r.order))]
        return best

    def reserve(self, cell: int, owner: int, dist: float) -> None:
        cur = self.claims.get(cell)
        if cur is None or dist < cur[1]:
            self.claims[cell] = (owner, dist)

    def goto(self, r: Robot, cell: int) -> None:
        r.goal = cell

    def leader_in_range(self, r: Robot):
        for o, _ in self.neighbors(r):
            if o.is_leader:
                return o
        return None

    # ── physics-free motion + leader election ──────────────────────────
    def _elect_leaders(self) -> None:
        parent = list(range(len(self.robots)))

        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a

        for r in self.robots:
            for o in self.robots:
                if o.id <= r.id:
                    continue
                if math.hypot(o.x - r.x, o.y - r.y) <= self.range:
                    ra, rb = find(r.id), find(o.id)
                    if ra != rb:
                        parent[max(ra, rb)] = min(ra, rb)
        for r in self.robots:
            r.is_leader = find(r.id) == r.id

    def _move(self, dt: float) -> None:
        for r in self.robots:
            if r.goal < 0:
                continue
            gx, gy = cell_xy(r.goal)
            dx, dy = gx - r.x, gy - r.y
            d = math.hypot(dx, dy)
            if d < 1e-3:
                continue
            want = math.atan2(dy, dx)
            dh = (want - r.heading + math.pi) % (2 * math.pi) - math.pi
            r.heading += max(-3 * dt, min(3 * dt, dh))
            step = min(d, r.speed * dt * max(0.2, math.cos(dh)))
            r.x = min(WORLD - 0.4, max(0.4, r.x + math.cos(r.heading) * step))
            r.y = min(WORLD - 0.4, max(0.4, r.y + math.sin(r.heading) * step))

    def relay_data(self) -> None:
        """Greedy geographic routing: each robot holding data hands it to an
        in-range neighbour closer to base, delivers straight to base if it's in
        range, or carries it like a data mule until a downhill peer appears."""
        bx, by = self.base_xy

        def dB(n) -> float:
            return math.hypot(getattr(n, "x", bx) - bx, getattr(n, "y", by) - by)

        for r in self.robots:
            if not r.payload or r.budget < 1:
                continue
            if dB(r) <= self.range:                         # base in reach: upload
                r.budget -= 1
                r.sent += 1
                self.metrics["sent"] += 1
                for tid in list(r.payload):
                    if tid not in self.base_delivered:
                        self.base_delivered.add(tid)
                        self.metrics["data_delivered"] += 1
                r.payload.clear()
                continue
            best, best_d = None, dB(r)                      # else hand off downhill
            for o, _ in self.neighbors(r):
                if dB(o) < best_d:
                    best, best_d = o, dB(o)
            if best is not None:
                r.budget -= 1
                r.sent += 1
                self.metrics["sent"] += 1
                best.payload |= r.payload
                r.payload.clear()

    def coverage(self) -> float:
        return sum(1 for m in self.covered_by if m) / NCELL

    def step(self, dt: float, policy) -> None:
        """Advance the world one tick: refill airtime, run the policy on every
        robot, relay data toward base, move, then sense. ``policy(fleet, robot)``
        is a strategy."""
        for r in self.robots:
            r.budget = min(self.airtime, r.budget + self.airtime * dt)
        if getattr(policy, "needs_leaders", False):
            self._elect_leaders()
        for r in self.robots:
            policy(self, r)
        if self.base:
            self.relay_data()
        self._move(dt)
        self._sense()
        self.t += dt
