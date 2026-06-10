"""The shipped demo policy must beat its chamber — without privileged knowledge.

This loads chamber-01's demo source OUT OF arena.js (so test and shipped
code can't diverge) and runs it closed-loop against the level geometry
with the arena's kinematics, lidar, and wall-slide. The policy gets only
what the handle gives: pose, lidar, move, state. If someone hardcodes the
answer key into a future level's demo, this harness is where the shame
gets institutionalized: change the walls and a cheating demo fails.
"""
import math
import re
from pathlib import Path

import pytest

ARENA_JS = Path(__file__).resolve().parent.parent / "roborun" / "web" / "arena.js"

WALLS = [(-8, -8, 8, -8), (-8, 8, 8, 8), (-8, -8, -8, 8), (8, -8, 8, 8),
         (-8, -1, -3.2, -1), (-1.8, -1, 1.8, -1), (3.2, -1, 8, -1),
         (-8, 1, -3.2, 1), (-1.8, 1, 1.8, 1), (3.2, 1, 8, 1),
         (-1, -8, -1, -5.4), (-1, -4, -1, -1), (1, -8, 1, -5.4), (1, -4, 1, -1),
         (-1, 1, -1, 4), (-1, 5.4, -1, 8), (1, 1, 1, 4), (1, 5.4, 1, 8)]
ROOMS = {"NW": (-8, -8, -1, -1), "NE": (1, -8, 8, -1),
         "SW": (-8, 1, -1, 8), "SE": (1, 1, 8, 8)}
DOORS = [("red door", -2.5, -1), ("blue door", 2.5, -1), ("red door", -2.5, 1),
         ("green door", 2.5, 1), ("red door", -1, -4.7), ("blue door", 1, 4.7)]
W = 0.15
AABBS = [(min(x1, x2) - W, min(z1, z2) - W, max(x1, x2) + W, max(z1, z2) + W)
         for x1, z1, x2, z2 in WALLS]


def load_demo_source(level_name: str) -> str:
    js = ARENA_JS.read_text()
    block = js[js.index(f'name: "{level_name}"'):]
    m = re.search(r"demo: `(.*?)`,", block, re.S)
    assert m, f"no demo found for {level_name}"
    return m.group(1)


class SimRobot:
    """The handle, backed by the same physics the arena runs."""

    def __init__(self):
        self.x, self.z, self.heading = 0.0, 0.0, 0.0
        self.state = {}
        self.cmd = (0.0, 0.0, 0.0)

    # senses
    def pose(self):
        return {"x": self.x, "z": self.z, "heading": self.heading}

    def lidar(self, rays=36, rng=8.0):
        out = []
        for i in range(rays):
            a = self.heading + (i / rays) * 2 * math.pi
            dx, dz = math.cos(a), -math.sin(a)
            best = rng
            for lx, lz, hx, hz in AABBS:
                # slab method: exact ray/AABB intersection
                tmin, tmax = 0.0, best
                ok = True
                for o, d, lo, hi in ((self.x, dx, lx, hx), (self.z, dz, lz, hz)):
                    if abs(d) < 1e-9:
                        if o < lo or o > hi:
                            ok = False
                            break
                    else:
                        t1, t2 = (lo - o) / d, (hi - o) / d
                        if t1 > t2:
                            t1, t2 = t2, t1
                        tmin, tmax = max(tmin, t1), min(tmax, t2)
                        if tmin > tmax:
                            ok = False
                            break
                if ok and tmin < best:
                    best = max(tmin, 0.0)
            out.append(best)
        return out

    def see(self, label=None, min_conf=0.4):
        """Door sensing with the arena's exact encoding: cone, occlusion,
        cx from bearing, dist in meters."""
        out = []
        for name, dx_, dz_ in DOORS:
            if label and name != label:
                continue
            tx, tz = dx_ - self.x, dz_ - self.z
            dist = math.hypot(tx, tz)
            if dist > 7 or dist < 0.05:
                continue
            phi = math.atan2(-tz, tx)
            bearing = (phi - self.heading + math.pi * 3) % (math.pi * 2) - math.pi
            if abs(bearing) > 0.62:
                continue
            # occlusion: march the ray, stop if a wall AABB is hit first
            ux, uz = tx / dist, tz / dist
            blocked = False
            t = 0.1
            while t < dist - 0.4:
                px, pz = self.x + ux * t, self.z + uz * t
                if any(lx <= px <= hx and lz <= pz <= hz
                       for lx, lz, hx, hz in AABBS):
                    blocked = True
                    break
                t += 0.1
            if blocked:
                continue
            cx_px = 640 - (bearing / 0.62) * 600

            class T:
                pass
            th = T()
            th.label, th.conf, th.dist = name, 0.95, round(dist, 2)
            th.cx, th.cy = cx_px / 1280, 0.5
            th.w = th.h = 0.1
            out.append(th)
        return out

    def answer(self, text):
        self.answers = getattr(self, "answers", [])
        self.answers.append(str(text))

    def seen(self, label=None):
        return []

    # action
    def move(self, forward=0.0, strafe=0.0, turn=0.0):
        self.cmd = (forward, strafe, turn)

    def stop(self):
        self.cmd = (0.0, 0.0, 0.0)

    def say(self, *_a, **_k):
        pass

    # the real handle's navigation primitive, verbatim (it only touches
    # pose/move/stop, which this stub provides)
    from roborun.behaviors import Robot as _R
    goto = _R.goto
    explore = _R.explore
    locate = _R.locate
    approach = _R.approach
    FOV = _R.FOV

    def log(self, *_a, **_k):
        pass

    # physics step (matches arena.js updateDog)
    def step(self, dt=0.1, r=0.32):
        f, _s, t = self.cmd
        t = max(-1.5, min(1.5, t))
        f = max(-1.0, min(1.0, f))
        self.heading += t * dt
        nx = self.x + f * math.cos(self.heading) * dt
        nz = self.z - f * math.sin(self.heading) * dt

        def blocked(px, pz):
            return any(lx - r <= px <= hx + r and lz - r <= pz <= hz + r
                       for lx, lz, hx, hz in AABBS)
        if blocked(nx, nz):
            if not blocked(nx, self.z):
                nz = self.z
            elif not blocked(self.x, nz):
                nx = self.x
            else:
                nx, nz = self.x, self.z
        self.x, self.z = nx, nz


def run_policy(source: str, budget_ticks: int):
    ns = {"behavior": lambda **kw: (lambda fn: fn)}
    code = source.replace("from roborun.behaviors import behavior\n", "")
    exec(compile(code, "demo.py", "exec"), ns)
    policy = ns["player_policy"]
    robot = SimRobot()
    visited = set()
    for _ in range(budget_ticks):
        policy(robot)
        robot.step()
        for name, (a, b, c, d) in ROOMS.items():
            if a < robot.x < c and b < robot.z < d:
                visited.add(name)
        if getattr(robot, "answers", None):
            break
    return visited, robot


def test_census_demo_explores_all_rooms_without_a_map():
    source = load_demo_source("dog-census")
    assert "TOUR" not in source, "hardcoded waypoints = cheating the benchmark"
    visited, robot = run_policy(source, budget_ticks=3600)  # 6 sim-minutes
    assert visited == {"NW", "NE", "SW", "SE"}, f"only explored {sorted(visited)}"
    # the census itself: perception-counted, deduped, correct
    assert getattr(robot, "answers", []) == ["6"], \
        f"answered {getattr(robot, 'answers', [])!r}, there are 6 doors"
