"""The arm chamber's shipped demo must sort all four blocks — eye-in-hand.

Same contract as test_demo_policy: the demo source is read OUT of
arena.js and run closed-loop against the level's actual mechanics —
world-axis effector motion, the annulus workspace (outer reach, inner
keep-out with rim slide), the radially-outward wrist camera with the
arena's FOV and 3D distances (camera 0.7 m above block tops), grasp
within 0.4 m, bins of radius 0.8. No privileged knowledge: the policy
only sees what the hand camera sees.
"""
import math

from test_demo_policy import load_demo_source

FOV = 1.323
SPEED = 1.2
DT = 0.1
EYE_H = 0.85
BLOCK_TOP = 0.15
BLOCKS = [("red block", -1.5, -2.0), ("blue block", 1.8, -1.4),
          ("red block", 0.6, 2.1), ("blue block", -2.2, 1.6)]
BINS = {"red": (-3.0, 0.0), "blue": (3.0, 0.0)}


class Thing:
    def __init__(self, label, cx, dist):
        self.label, self.cx, self.dist, self.conf = label, cx, dist, 0.95


class SimArm:
    """The effector handle, backed by the arena's arm mechanics."""

    def __init__(self):
        self.x, self.z = 1.6, 0.0          # the level's spawn: home, out front
        self.state = {}
        self.cmd = (0.0, 0.0)
        self.grip = False
        self.blocks = [{"label": lb, "x": x, "z": z, "carried": False,
                        "sorted": False} for lb, x, z in BLOCKS]

    @property
    def heading(self):
        return math.atan2(-self.z, self.x)

    def pose(self):
        return {"x": self.x, "z": self.z, "heading": self.heading}

    def see(self, label=None, min_conf=0.4):
        out = []
        for b in self.blocks:
            if b["carried"] or b["sorted"]:
                continue
            dx, dz = b["x"] - self.x, b["z"] - self.z
            dist = math.hypot(math.hypot(dx, dz), EYE_H - BLOCK_TOP)
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
        self.cmd = (max(-1, min(1, forward)), max(-1, min(1, strafe)))

    def stop(self):
        self.cmd = (0.0, 0.0)

    def grasp(self, closed):
        if closed and not self.grip:
            for b in self.blocks:
                if not b["carried"] and not b["sorted"] and \
                        math.hypot(b["x"] - self.x, b["z"] - self.z) < 0.4:
                    b["carried"] = True
                    break
        if not closed and self.grip:
            for b in self.blocks:
                if b["carried"]:
                    b["carried"] = False
                    color = b["label"].split()[0]
                    bx, bz = BINS[color]
                    if math.hypot(b["x"] - bx, b["z"] - bz) < 0.8:
                        b["sorted"] = True
        self.grip = closed

    def step(self):
        f, st = self.cmd
        nx, nz = self.x + f * SPEED * DT, self.z + st * SPEED * DT
        d_now, d_new = math.hypot(self.x, self.z), math.hypot(nx, nz)
        if d_new < 4.0 * 0.95:                      # the workspace annulus
            if d_new > 0.7 or d_new > d_now:
                self.x, self.z = nx, nz
            else:                                   # slide around the rim
                a = math.atan2(nz, nx)
                cur = math.atan2(self.z, self.x)
                # purely radial push: nudge CCW past the antipode deadlock
                if abs(math.atan2(math.sin(a - cur), math.cos(a - cur))) < 0.02:
                    a = cur + 0.05
                self.x, self.z = math.cos(a) * 0.72, math.sin(a) * 0.72
        for b in self.blocks:
            if b["carried"]:
                b["x"], b["z"] = self.x, self.z

    # unused handle surface the demo may touch
    def say(self, *_): pass
    def log(self, *_): pass


def test_arm_demo_sorts_all_four_blocks():
    src = load_demo_source("arm-sort")
    ns = {}
    exec(compile(src, "arm_demo.py", "exec"), ns)
    fns = [v for v in ns.values() if callable(v) and hasattr(v, "_behavior")]
    assert fns, "demo has no @behavior policy"
    policy = fns[0]

    robot = SimArm()
    for tick in range(4000):                        # 400 sim-seconds at 10 Hz
        policy(robot)
        robot.step()
        if all(b["sorted"] for b in robot.blocks):
            break
    sorted_n = sum(b["sorted"] for b in robot.blocks)
    assert sorted_n == 4, (
        f"demo sorted {sorted_n}/4 blocks in {tick} ticks; "
        f"state: {[(b['label'], round(b['x'], 1), round(b['z'], 1), b['sorted']) for b in robot.blocks]}")
    assert tick < 3500, f"sorted all four but took {tick} ticks"
