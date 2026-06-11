"""The arm chamber's shipped demo must sort all four blocks — physically.

Same contract as test_demo_policy: the demo source is read OUT of
arena.js and run closed-loop against the level's actual mechanics. The
arena's arm is physical, and so is this harness:

  * the wrist has a real vertical axis (climb, 0.9 m/s, clamped)
  * the eye-in-hand camera rides 0.42 m below the wrist, looking
    radially outward; detection distances are 3D
  * grasping is edge-triggered closure, not a magnet: it succeeds only
    if a block is centered between the pads (planar < 0.16 m) AND the
    fingers are at block height (|block_y - (wrist - 0.65)| < 0.14)
  * pose() reports y and holding (gripper feedback)
  * releasing drops the block under gravity; it lands on the floor or
    on another block; sorted = at REST inside the matching bin

If the demo cheats (teleports, grabs from cruise height, ignores the
keep-out), this is where it fails.
"""
import math

from test_demo_policy import load_demo_source

FOV = 1.323
SPEED = 1.2
CLIMB_RATE = 0.9
DT = 0.1
CAM_DROP = 0.42            # camera sits this far below the wrist
FINGER_DROP = 0.65         # finger centers, below the wrist
BLOCKS = [("red block", -1.5, -2.0), ("blue block", 1.8, -1.4),
          ("red block", 0.6, 2.1), ("blue block", -2.2, 1.6)]
BINS = {"red": (-3.0, 0.0), "blue": (3.0, 0.0)}


class Thing:
    def __init__(self, label, cx, dist):
        self.label, self.cx, self.dist, self.conf = label, cx, dist, 0.95


class SimArm:
    """The effector handle, backed by the arena's physical arm mechanics."""

    def __init__(self):
        self.x, self.z, self.y = 1.6, 0.0, 1.1     # spawn: home pose, cruise height
        self.state = {}
        self.cmd = (0.0, 0.0, 0.0)                 # forward, strafe, climb
        self.grip = False
        self._grip_was = False
        self.carrying = None
        self.blocks = [{"label": lb, "x": x, "z": z, "y": 0.15, "vy": 0.0,
                        "carried": False, "sorted": False}
                       for lb, x, z in BLOCKS]

    @property
    def heading(self):
        return math.atan2(-self.z, self.x)

    def pose(self):
        return {"x": self.x, "z": self.z, "y": self.y, "heading": self.heading,
                "holding": self.carrying["label"] if self.carrying else None}

    def see(self, label=None, min_conf=0.4):
        eye_y = max(0.3, self.y - CAM_DROP)
        out = []
        for b in self.blocks:
            if b["carried"] or b["sorted"]:
                continue
            dx, dz = b["x"] - self.x, b["z"] - self.z
            dist = math.hypot(math.hypot(dx, dz), eye_y - b["y"])
            phi = math.atan2(-dz, dx)
            bearing = math.atan2(math.sin(phi - self.heading),
                                 math.cos(phi - self.heading))
            if abs(bearing) > FOV / 2 or dist > 8:
                continue
            if label and b["label"] != label:
                continue
            out.append(Thing(b["label"], 0.5 - bearing / FOV, dist))
        return sorted(out, key=lambda t: t.dist)

    def move(self, forward=0.0, strafe=0.0, turn=0.0, climb=0.0):
        self.cmd = (max(-1, min(1, forward)), max(-1, min(1, strafe)),
                    max(-1, min(1, climb)))

    def stop(self):
        self.cmd = (0.0, 0.0, 0.0)

    def grasp(self, closed):
        self.grip = bool(closed)

    def _floor_under(self, b):
        floor = 0.15
        for o in self.blocks:
            if o is b or o["carried"]:
                continue
            if abs(b["x"] - o["x"]) < 0.28 and abs(b["z"] - o["z"]) < 0.28 \
                    and o["y"] + 0.225 <= b["y"]:
                floor = max(floor, o["y"] + 0.3)
        return floor

    def step(self):
        f, st, cl = self.cmd
        # vertical axis
        self.y = max(0.55, min(1.6, self.y + cl * CLIMB_RATE * DT))
        # planar motion in the annulus workspace
        nx, nz = self.x + f * SPEED * DT, self.z + st * SPEED * DT
        d_now, d_new = math.hypot(self.x, self.z), math.hypot(nx, nz)
        if d_new < 4.0 * 0.95:
            if d_new > 0.7 or d_new > d_now:
                self.x, self.z = nx, nz
            else:
                a = math.atan2(nz, nx)
                cur = math.atan2(self.z, self.x)
                if abs(math.atan2(math.sin(a - cur), math.cos(a - cur))) < 0.02:
                    a = cur + 0.05
                self.x, self.z = math.cos(a) * 0.72, math.sin(a) * 0.72
        # grasp: closure edge only, fingers must straddle a block
        if self.grip and not self._grip_was and self.carrying is None:
            fy = self.y - FINGER_DROP
            for b in self.blocks:
                if b["carried"]:
                    continue
                if abs(b["y"] - fy) < 0.14 and \
                        math.hypot(b["x"] - self.x, b["z"] - self.z) < 0.16:
                    self.carrying = b
                    b["carried"] = True
                    b["vy"] = 0.0
                    break
        if not self.grip and self.carrying is not None:
            b = self.carrying
            self.carrying = None
            b["carried"] = False
            b["vy"] = 0.0                          # gravity takes it from here
        self._grip_was = self.grip
        # block physics: carried rides the fingers, free blocks fall/stack
        for b in self.blocks:
            if b["carried"]:
                b["x"], b["z"] = self.x, self.z
                b["y"] = max(0.15, self.y - FINGER_DROP)
                continue
            b["vy"] -= 9.8 * DT
            b["y"] += b["vy"] * DT
            floor = self._floor_under(b)
            if b["y"] <= floor:
                b["y"], b["vy"] = floor, 0.0

    def sort_check(self):
        for b in self.blocks:
            if b["sorted"] or b["carried"] or b["vy"] != 0.0:
                continue
            color = b["label"].split()[0]
            bx, bz = BINS[color]
            if math.hypot(b["x"] - bx, b["z"] - bz) < 0.8:
                b["sorted"] = True

    # unused handle surface the demo may touch
    def say(self, *_): pass
    def log(self, *_): pass


def test_arm_demo_physically_sorts_all_four_blocks():
    src = load_demo_source("arm-sort")
    ns = {}
    exec(compile(src, "arm_demo.py", "exec"), ns)
    fns = [v for v in ns.values() if callable(v) and hasattr(v, "_behavior")]
    assert fns, "demo has no @behavior policy"
    policy = fns[0]

    robot = SimArm()
    tick = 0
    for tick in range(6000):                       # 600 sim-seconds at 10 Hz
        policy(robot)
        robot.step()
        robot.sort_check()
        if all(b["sorted"] for b in robot.blocks):
            break
    n = sum(b["sorted"] for b in robot.blocks)
    assert n == 4, (
        f"demo sorted {n}/4 in {tick} ticks; state: "
        f"{[(b['label'], round(b['x'], 2), round(b['z'], 2), round(b['y'], 2), b['sorted']) for b in robot.blocks]}; "
        f"hand at ({robot.x:.2f}, {robot.z:.2f}, y={robot.y:.2f}) "
        f"holding={robot.carrying['label'] if robot.carrying else None}")
    assert tick < 5500, f"sorted all four but took {tick} ticks"


def test_grasp_fails_from_cruise_height():
    """The magnet is dead: closing the gripper at cruise height above a
    block must NOT acquire it — the fingers are nowhere near it."""
    robot = SimArm()
    b = robot.blocks[1]                            # blue block at (1.8, -1.4)
    robot.x, robot.z, robot.y = b["x"], b["z"], 1.2
    robot.grasp(True)
    robot.step()
    assert robot.carrying is None, "grasped from 0.55 m above the block — magnet behavior"
    # descend to grasp height and re-close (edge): now it must work
    robot.grasp(False); robot.step()
    robot.y = 0.9
    robot.grasp(True); robot.step()
    assert robot.carrying is b, "fingers straddle the block at the right height — should grasp"


def test_release_drops_under_gravity():
    robot = SimArm()
    b = robot.blocks[0]
    robot.x, robot.z, robot.y = b["x"], b["z"], 0.9
    robot.grasp(True); robot.step()
    assert robot.carrying is b
    robot.y = 1.4                                  # lift
    robot.step()
    assert b["y"] > 0.6, "carried block should ride the fingers up"
    robot.grasp(False)
    settled = 0
    for _ in range(30):                            # 3 s is plenty to fall 0.6 m
        robot.step()
        if b["y"] <= 0.151 and b["vy"] == 0.0:
            settled += 1
    assert settled > 0, "released block never landed — gravity missing"
