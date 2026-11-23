"""The four fleet coordination strategies, one function each.

A strategy is a policy over the comms model in ``comms.py`` — exactly what the
browser sandbox runs, and the shape a real on-robot behaviour would take once a
``robot.radio`` is wired. Each is called once per robot per tick. They differ
only in *what they say on the radio* and *how they choose the next cell*; the
range / airtime / memory limits do the rest.

These are the JS twins of the strategy cards in roborun/web/fleet.js — keep the
two in sync if you change the coordination logic.
"""
from __future__ import annotations

from .comms import Fleet, Robot, cell_xy
import math


def lone_wolves(fleet: Fleet, r: Robot) -> None:
    """No radio at all. Map the nearest patch *you* haven't seen. Robust, but
    two robots cheerfully re-walk the same ground — watch the overlap count."""
    if r.arrived():
        fleet.goto(r, fleet.nearest_unknown(r))


def gossip(fleet: Fleet, r: Robot) -> None:
    """Tell neighbours what you just mapped; believe what they tell you. Map
    knowledge floods hop-by-hop, so robots stop chasing ground a neighbour
    already covered. Bounded by airtime and range."""
    for m in fleet.recv(r, "cells"):
        r.note_mapped(m.payload["cells"])
    fresh = r.just_mapped()
    if fresh:
        fleet.broadcast(r, "cells", cells=fresh[:12])
    if r.arrived():
        fleet.goto(r, fleet.nearest_unknown(r))


def claim_and_yield(fleet: Fleet, r: Robot) -> None:
    """A tiny market. Before committing to a cell, announce a claim with your
    distance; if a closer robot already claimed it, yield and pick another. No
    central server — just neighbours settling overlaps. Tightest coverage while
    everyone stays linked."""
    for m in fleet.recv(r):
        if m.kind == "cells":
            r.note_mapped(m.payload["cells"])
        elif m.kind == "claim":
            fleet.reserve(m.payload["cell"], m.sender, m.payload["dist"])
    fresh = r.just_mapped()
    if fresh:
        fleet.broadcast(r, "cells", cells=fresh[:12])
    if r.arrived():
        c = fleet.nearest_unknown(r, avoid_claims=True)
        if c >= 0:
            cx, cy = cell_xy(c)
            dist = math.hypot(cx - r.x, cy - r.y)
            fleet.reserve(c, r.id, dist)
            fleet.broadcast(r, "claim", cell=c, dist=dist)
        fleet.goto(r, c)


def one_commander(fleet: Fleet, r: Robot) -> None:
    """The lowest-id robot in radio reach is the leader and hands out
    non-overlapping targets. Centralised and tidy — but a robot that drifts out
    of the leader's range gets no orders and falls back to lone-wolf."""
    for m in fleet.recv(r):
        if m.kind == "cells":
            r.note_mapped(m.payload["cells"])
        elif m.kind == "assign":
            r.assigned = m.payload["cell"]
        elif m.kind == "request" and r.is_leader:
            c = _leader_pick(fleet, fleet.robots[m.sender])
            if c >= 0:
                fleet.reserve(c, m.sender, 0.0)
            fleet.unicast(r, fleet.robots[m.sender], "assign", cell=c)
    fresh = r.just_mapped()
    if fresh:
        fleet.broadcast(r, "cells", cells=fresh[:12])
    if r.arrived():
        if r.is_leader:
            fleet.goto(r, _leader_pick(fleet, r))
        else:
            leader = fleet.leader_in_range(r)
            if leader:
                fleet.unicast(r, leader, "request")
                tgt = r.assigned if (r.assigned >= 0 and r.assigned not in r.known) \
                    else fleet.nearest_unknown(r, avoid_claims=True)
                fleet.goto(r, tgt)
            else:
                fleet.goto(r, fleet.nearest_unknown(r))   # orphaned → on my own


one_commander.needs_leaders = True   # tells Fleet.step to run leader election


def _leader_pick(fleet: Fleet, requester: Robot) -> int:
    best, bd = -1, 1e9
    for c in range(len(fleet.covered_by)):
        if c in requester.known:
            continue
        cl = fleet.claims.get(c)
        if cl and cl[0] != requester.id:
            continue
        cx, cy = cell_xy(c)
        d = math.hypot(cx - requester.x, cy - requester.y)
        if d < bd:
            best, bd = c, d
    return best


STRATEGIES = {
    "independent": lone_wolves,
    "gossip": gossip,
    "auction": claim_and_yield,
    "leader": one_commander,
}
