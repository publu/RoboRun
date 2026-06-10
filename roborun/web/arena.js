/* RoboRun Arena.
   Seven chambers, four robot types (dog, humanoid, arm, drone) — all in
   the browser, all driven by the same policy handle that drives real
   ROS robots. The robot's senses: forward camera cone + 360° lidar
   (+ pose). The spectator gets more. Briefs are policy specs; WASD is
   debug only (practice). Attempts auto-record; wins seal and show the
   run-hash. */

import * as THREE from "three";

/* ════════════════ levels ════════════════ */
const COLORS = { red: 0xd84a4a, blue: 0x4a7ad8, green: 0x44b86a,
                 yellow: 0xd8b54a, purple: 0x9a5ad8 };

const LEVELS = [
  /* ── DOG 1 · DOOR CENSUS — explore, count with perception, answer ── */
  {
    name: "dog-census", robot: "dog",
    title: "DOG 01 — DOOR CENSUS",
    brief: "QUESTION: how many doors does this building have? Explore, count "
         + "them with your own perception (project see() hits into world "
         + "coords and dedupe — the same door seen twice is one door), then "
         + "robot.answer(n). Wrong answers are logged. You have pose(), "
         + "lidar(), see(), goto().",
    bounds: 16, spawn: { x: 0, z: 0, heading: 0 },
    rooms: [
      { id: "north-west", rect: [-8, -8, -1, -1] },
      { id: "north-east", rect: [1, -8, 8, -1] },
      { id: "south-west", rect: [-8, 1, -1, 8] },
      { id: "south-east", rect: [1, 1, 8, 8] },
    ],
    walls: [
      [-8, -8, 8, -8], [-8, 8, 8, 8], [-8, -8, -8, 8], [8, -8, 8, 8],
      [-8, -1, -3.2, -1], [-1.8, -1, 1.8, -1], [3.2, -1, 8, -1],
      [-8, 1, -3.2, 1], [-1.8, 1, 1.8, 1], [3.2, 1, 8, 1],
      [-1, -8, -1, -5.4], [-1, -4, -1, -1], [1, -8, 1, -5.4], [1, -4, 1, -1],
      [-1, 1, -1, 4], [-1, 5.4, -1, 8], [1, 1, 1, 4], [1, 5.4, 1, 8],
    ],
    props: [
      { kind: "door", color: "red", x: -2.5, z: -1 },
      { kind: "door", color: "blue", x: 2.5, z: -1 },
      { kind: "door", color: "red", x: -2.5, z: 1 },
      { kind: "door", color: "green", x: 2.5, z: 1 },
      { kind: "door", color: "red", x: -1, z: -4.7 },
      { kind: "door", color: "blue", x: 1, z: 4.7 },
    ],
    win: { type: "answer", value: "6", question: "how many doors?" },
    demo: `from roborun.behaviors import behavior
from math import hypot, pi, cos, sin

# Explore (frontier search), project every door sighting into world
# coordinates, dedupe by position, answer when nothing is left unknown.
CELL = 0.25
FOV = 1.323     # see(): bearing = (0.5 - cx) * FOV  (radians)

def integrate(grid, pose, scan):
    n = len(scan)
    for i, r in enumerate(scan):
        a = pose["heading"] + i / n * 2 * pi
        d = CELL * 0.5
        while d < r:
            c = (round((pose["x"] + cos(a) * d) / CELL),
                 round((pose["z"] - sin(a) * d) / CELL))
            if grid.get(c) != 2:
                grid[c] = 1
            d += CELL * 0.5
        if r < 7.5:
            grid[(round((pose["x"] + cos(a) * r) / CELL),
                  round((pose["z"] - sin(a) * r) / CELL))] = 2

def clear(grid, c):
    if grid.get(c) != 1:
        return False
    for dx in (-1, 0, 1):
        for dz in (-1, 0, 1):
            if grid.get((c[0] + dx, c[1] + dz)) == 2:
                return False
    return True

def plan(grid, pose):
    start = (round(pose["x"] / CELL), round(pose["z"] / CELL))
    prev, queue, seen = {}, [start], {start}
    while queue:
        c = queue.pop(0)
        for dx, dz in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (c[0] + dx, c[1] + dz)
            if n in seen:
                continue
            if grid.get(n) is None:
                path = [c]
                while path[-1] != start:
                    path.append(prev[path[-1]])
                return path[::-1]
            if clear(grid, n):
                seen.add(n); prev[n] = c; queue.append(n)
    return None

@behavior(hz=10)
def player_policy(robot):
    pose, scan = robot.pose(), robot.lidar()
    if not pose or not scan:
        return robot.stop()
    st = robot.state
    grid = st.setdefault("grid", {})
    integrate(grid, pose, scan)

    # perception-based door census: project each sighting to a world
    # point, snap to a 1.5 m cell — one cell, one door, however often seen
    doors = st.setdefault("doors", set())
    for d in robot.see():
        if "door" in d.label and d.dist:
            a = pose["heading"] + (0.5 - d.cx) * FOV
            wx = pose["x"] + cos(a) * d.dist
            wz = pose["z"] - sin(a) * d.dist
            doors.add((round(wx / 1.5), round(wz / 1.5)))

    st["tick"] = st.get("tick", 0) + 1
    if st["tick"] % 10 == 1 or not st.get("path"):
        st["path"] = plan(grid, pose)
    path = st.get("path")
    if not path:
        if not st.get("done"):           # fully explored: commit the count
            st["done"] = True
            robot.answer(str(len(doors)))
            robot.say("counted " + str(len(doors)) + " doors")
        return robot.stop()

    while path and hypot(path[0][0] * CELL - pose["x"],
                         path[0][1] * CELL - pose["z"]) < 0.45:
        path.pop(0)
    if not path:
        return
    tx, tz = path[min(4, len(path) - 1)]
    ahead = min(scan[:2] + scan[-2:])
    if ahead < 0.6:
        left, right = sum(scan[6:12]), sum(scan[-12:-6])
        robot.move(turn=1.0 if left > right else -1.0)
    else:
        robot.goto(tx * CELL, tz * CELL)
`,
  },

  /* ── DOG 2 · RED DOOR — random spawn, find the red one ── */
  {
    name: "dog-reddoor", robot: "dog",
    title: "DOG 02 — FIND THE RED DOOR",
    brief: "Five rooms off a corridor, each with a colored door. Colors and "
         + "your spawn are random. Stand at the RED door (within 1.6 m) and "
         + "hold 1.5 s. No map given.",
    bounds: 16, spawn: { random: "corridor" },
    rooms: [],
    walls: [
      [-16, -8, 16, -8], [-16, 7, 16, 7], [-16, -8, -16, 7], [16, -8, 16, 7],
      [-16, -1, -12.7, -1], [-11.3, -1, -6.7, -1], [-5.3, -1, -0.7, -1],
      [0.7, -1, 5.3, -1], [6.7, -1, 11.3, -1], [12.7, -1, 16, -1],
      [-9, -8, -9, -1], [-3, -8, -3, -1], [3, -8, 3, -1], [9, -8, 9, -1],
    ],
    props: "reddoor-random",
    win: { type: "near", label: "red door", dist: 1.6, hold: 1.5 },
    demo: `from roborun.behaviors import behavior
from math import cos, sin

FOV = 1.323

@behavior(hz=10)
def player_policy(robot):
    pose, scan = robot.pose(), robot.lidar()
    if not pose or not scan:
        return robot.stop()
    st = robot.state

    red = robot.see("red door")
    if red and red[0].dist:
        a = pose["heading"] + (0.5 - red[0].cx) * FOV
        st["target"] = (pose["x"] + cos(a) * red[0].dist,
                        pose["z"] - sin(a) * red[0].dist)
    t = st.get("target")
    if t:
        robot.goto(t[0], t[1], tol=1.0)   # the chamber checks the hold
        return

    # sweep the corridor until red shows up
    ahead = min(scan[:2] + scan[-2:])
    if ahead < 1.0:
        st["dir"] = -st.get("dir", 1)
        robot.move(turn=1.2)
    else:
        robot.move(forward=0.8 * st.get("dir", 1), turn=0.25)
`,
  },

  /* ── DOG 3 · PATROL — laps with a moving hazard ── */
  {
    name: "dog-patrol", robot: "dog",
    title: "DOG 03 — PATROL",
    brief: "Patrol route: the four corners at (±6, ±6), in order, two full "
         + "laps. A service rover circles the middle — it will not dodge "
         + "you, so you dodge it.",
    bounds: 16, spawn: { x: -6, z: 6, heading: 0 },
    rooms: [],
    walls: [[-8, -8, 8, -8], [-8, 8, 8, 8], [-8, -8, -8, 8], [8, -8, 8, 8],
            [-2, -2, 2, -2], [-2, 2, 2, 2]],
    props: [
      { kind: "checkpoint", color: "green", x: -6, z: -6, id: 0 },
      { kind: "checkpoint", color: "green", x: 6, z: -6, id: 1 },
      { kind: "checkpoint", color: "green", x: 6, z: 6, id: 2 },
      { kind: "checkpoint", color: "green", x: -6, z: 6, id: 3 },
    ],
    movers: [{ kind: "rover", r: 4.2, speed: 0.55, size: 0.9 }],
    win: { type: "checkpoints", order: [0, 1, 2, 3], laps: 2, dist: 1.2 },
    demo: `from roborun.behaviors import behavior

CORNERS = [(-6, -6), (6, -6), (6, 6), (-6, 6)]   # given in the brief

@behavior(hz=10)
def player_policy(robot):
    pose, scan = robot.pose(), robot.lidar()
    if not pose or not scan:
        return robot.stop()
    st = robot.state
    i = st.setdefault("i", 0)
    tx, tz = CORNERS[i % 4]

    ahead = min(scan[:3] + scan[-3:])
    if ahead < 1.1:                       # the rover, or a wall: yield
        left, right = sum(scan[6:12]), sum(scan[-12:-6])
        robot.move(turn=1.2 if left > right else -1.2)
        return
    if robot.goto(tx, tz, tol=1.0):
        st["i"] = i + 1
`,
  },

  /* ── HUMANOID 1 · BUTTONS — press in order ── */
  {
    name: "biped-buttons", robot: "biped",
    title: "HUMANOID 01 — PRESS THE BUTTONS",
    brief: "Three floor buttons: press RED, then GREEN, then BLUE — stand "
         + "on each pad until it locks (0.8 s). Out-of-order presses do "
         + "nothing. Find them with see('red button') etc.",
    bounds: 12, spawn: { x: 0, z: 4, heading: 1.57 },
    rooms: [],
    walls: [[-6, -6, 6, -6], [-6, 6, 6, 6], [-6, -6, -6, 6], [6, -6, 6, 6],
            [-2, -1, 2, -1]],
    props: [
      { kind: "button", color: "red", x: -4, z: -4, id: 0 },
      { kind: "button", color: "green", x: 4, z: -4, id: 1 },
      { kind: "button", color: "blue", x: 4, z: 4, id: 2 },
    ],
    win: { type: "buttons", order: [0, 1, 2], dist: 0.8, hold: 0.8 },
    demo: `from roborun.behaviors import behavior
from math import cos, sin

FOV = 1.323
ORDER = ["red button", "green button", "blue button"]

@behavior(hz=10)
def player_policy(robot):
    pose = robot.pose()
    if not pose:
        return robot.stop()
    st = robot.state
    i = st.setdefault("i", 0)
    if i >= len(ORDER):
        return robot.stop()

    hits = robot.see(ORDER[i])
    if hits and hits[0].dist:
        a = pose["heading"] + (0.5 - hits[0].cx) * FOV
        st["target"] = (pose["x"] + cos(a) * hits[0].dist,
                        pose["z"] - sin(a) * hits[0].dist)
    t = st.get("target")
    if not t:
        return robot.move(turn=0.9)       # scan for it
    if robot.goto(t[0], t[1], tol=0.35):
        st["hold"] = st.get("hold", 0) + 1
        if st["hold"] > 12:               # pressed — next button
            st["i"], st["target"], st["hold"] = i + 1, None, 0
`,
  },

  /* ── HUMANOID 2 · CARRY — crates to the drop zone ── */
  {
    name: "biped-carry", robot: "biped",
    title: "HUMANOID 02 — CARRY",
    brief: "Two crates, one glowing drop zone. Walk into a crate to pick it "
         + "up (auto), walk into the zone to set it down (auto). Deliver "
         + "both. see('crate') / see('drop zone').",
    bounds: 12, spawn: { x: 0, z: 0, heading: 0 },
    rooms: [],
    walls: [[-6, -6, 6, -6], [-6, 6, 6, 6], [-6, -6, -6, 6], [6, -6, 6, 6],
            [0, -3, 0, -0.5], [-3, 2, -0.5, 2]],
    props: [
      { kind: "crate", color: "yellow", x: -4.5, z: -4.5, id: 0 },
      { kind: "crate", color: "yellow", x: 4.5, z: -4.5, id: 1 },
      { kind: "zone", color: "green", x: 4.5, z: 4.5, r: 1.2, label: "drop zone" },
    ],
    win: { type: "carry", crates: 2, pickup: 0.7, zone: 1.2 },
    demo: `from roborun.behaviors import behavior
from math import cos, sin

FOV = 1.323

@behavior(hz=10)
def player_policy(robot):
    pose = robot.pose()
    if not pose:
        return robot.stop()
    st = robot.state
    want = "drop zone" if st.get("carrying") else "crate"
    hits = robot.see(want)
    if hits and hits[0].dist:
        a = pose["heading"] + (0.5 - hits[0].cx) * FOV
        st["target"] = (pose["x"] + cos(a) * hits[0].dist,
                        pose["z"] - sin(a) * hits[0].dist)
    t = st.get("target")
    if not t:
        return robot.move(turn=0.9)
    if robot.goto(t[0], t[1], tol=0.4):
        st["carrying"] = not st.get("carrying")   # auto pick/drop happened
        st["target"] = None
`,
  },

  /* ── ARM · SORT — blocks into matching bins ── */
  {
    name: "arm-sort", robot: "arm",
    title: "ARM 01 — SORT THE BLOCKS",
    brief: "A fixed arm over a table. Four blocks (red/blue), two bins: RED "
         + "bin at (-3, 0), BLUE bin at (3, 0). Controls differ: move() "
         + "drives the end-effector — forward=+x, strafe=+z; grasp(True) "
         + "closes near a block, grasp(False) releases. pose() is the "
         + "effector. Sort all four.",
    bounds: 8, spawn: { x: 0, z: 0, heading: 0 },
    rooms: [], walls: [],
    props: [
      { kind: "block", color: "red", x: -1.5, z: -2, id: 0 },
      { kind: "block", color: "blue", x: 1.8, z: -1.4, id: 1 },
      { kind: "block", color: "red", x: 0.6, z: 2.1, id: 2 },
      { kind: "block", color: "blue", x: -2.2, z: 1.6, id: 3 },
      { kind: "bin", color: "red", x: -3, z: 0, r: 0.8, label: "red bin" },
      { kind: "bin", color: "blue", x: 3, z: 0, r: 0.8, label: "blue bin" },
    ],
    win: { type: "sort", blocks: 4, grabDist: 0.4 },
    demo: `from roborun.behaviors import behavior
from math import cos, sin

FOV = 1.323
BINS = {"red": (-3, 0), "blue": (3, 0)}   # given in the brief

def toward(robot, pose, tx, tz, tol=0.25):
    dx, dz = tx - pose["x"], tz - pose["z"]
    if abs(dx) < tol and abs(dz) < tol:
        return True
    robot.move(forward=max(-0.8, min(0.8, 2 * dx)),
               strafe=max(-0.8, min(0.8, 2 * dz)))
    return False

@behavior(hz=10)
def player_policy(robot):
    pose = robot.pose()
    if not pose:
        return robot.stop()
    st = robot.state

    if st.get("carrying"):                 # take it to the matching bin
        bx, bz = BINS[st["carrying"]]
        if toward(robot, pose, bx, bz):
            robot.grasp(False)
            st["carrying"] = None
        return

    blocks = robot.see("red block") + robot.see("blue block")
    if not blocks:
        return robot.stop()                # table clear — done
    b = min(blocks, key=lambda d: d.dist or 9)
    a = (0.5 - b.cx) * FOV                 # effector camera looks along +x
    tx = pose["x"] + cos(a) * (b.dist or 0)
    tz = pose["z"] - sin(a) * (b.dist or 0)
    if toward(robot, pose, tx, tz):
        robot.grasp(True)
        st["carrying"] = b.label.split()[0]
`,
  },

  /* ── DRONE · RINGS — fly the course ── */
  {
    name: "drone-rings", robot: "drone",
    title: "DRONE 01 — RING RUN",
    brief: "Race course: four rings, in order — (-5,-5) alt 1.2 · (5,-5) "
         + "alt 2.4 · (5,5) alt 1.0 · (-5,5) alt 2.8 (given: it's a "
         + "course). move(climb=) is your vertical; pose() includes y.",
    bounds: 16, spawn: { x: 0, z: 0, heading: 0 },
    rooms: [],
    walls: [[-8, -8, 8, -8], [-8, 8, 8, 8], [-8, -8, -8, 8], [8, -8, 8, 8]],
    props: [
      { kind: "ring", color: "green", x: -5, z: -5, y: 1.2, id: 0 },
      { kind: "ring", color: "green", x: 5, z: -5, y: 2.4, id: 1 },
      { kind: "ring", color: "green", x: 5, z: 5, y: 1.0, id: 2 },
      { kind: "ring", color: "green", x: -5, z: 5, y: 2.8, id: 3 },
    ],
    win: { type: "rings", order: [0, 1, 2, 3], dist: 0.9, alt: 0.5 },
    demo: `from roborun.behaviors import behavior

RINGS = [(-5, -5, 1.2), (5, -5, 2.4), (5, 5, 1.0), (-5, 5, 2.8)]

@behavior(hz=10)
def player_policy(robot):
    pose = robot.pose()
    if not pose:
        return robot.stop()
    st = robot.state
    i = st.setdefault("i", 0)
    if i >= len(RINGS):
        return robot.stop()
    tx, tz, ty = RINGS[i]
    dy = ty - pose.get("y", 1)
    arrived = robot.goto(tx, tz, tol=0.6)
    robot.move(climb=max(-0.8, min(0.8, 2 * dy)))   # climb stacks on goto
    if arrived and abs(dy) < 0.4:
        st["i"] = i + 1
`,
  },
];

/* ════════════════ scene shell ════════════════ */
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0e11);
scene.fog = new THREE.Fog(0x0b0e11, 18, 34);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.autoClear = false;
renderer.domElement.className = "webgl";
document.body.appendChild(renderer.domElement);

scene.add(new THREE.HemisphereLight(0xcfe8ff, 0x223038, 0.85));
const sun = new THREE.DirectionalLight(0xffffff, 1.4);
sun.position.set(8, 14, 6);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.camera.left = sun.shadow.camera.bottom = -18;
sun.shadow.camera.right = sun.shadow.camera.top = 18;
scene.add(sun);

const specCam = new THREE.PerspectiveCamera(60, innerWidth / innerHeight, 0.1, 100);
const povCam = new THREE.PerspectiveCamera(70, 16 / 9, 0.08, 60);
const topCam = new THREE.OrthographicCamera(-9, 9, 9, -9, 1, 50);
topCam.position.set(0, 30, 0);
topCam.lookAt(0, 0, 0);
const chaseCam = new THREE.PerspectiveCamera(60, 16 / 9, 0.1, 100);
const orbitCam = new THREE.PerspectiveCamera(60, 16 / 9, 0.1, 100);
const CAM_POOL = { pov: povCam, top: topCam, chase: chaseCam, orbit: orbitCam };
let camMode = 0;
const CAM_MODES = ["chase", "orbit", "top"];
addEventListener("resize", () => {
  renderer.setSize(innerWidth, innerHeight);
  specCam.aspect = innerWidth / innerHeight;
  specCam.updateProjectionMatrix();
});

/* ════════════════ level building ════════════════ */
const W = 0.15;
let LV = null, levelGroup = null;
let wallMeshes = [], propObjs = [], movers = [];

function makeProp(p) {
  const color = COLORS[p.color] || 0x9fb0bd;
  const mat = new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.35 });
  let mesh, y = 0.8;
  const label = p.label || `${p.color} ${p.kind}`;
  if (p.kind === "door") {
    mesh = new THREE.Mesh(new THREE.TorusGeometry(0.75, 0.09, 8, 24, Math.PI), mat);
    mesh.position.set(p.x, 0.05, p.z);
  } else if (["checkpoint", "zone", "bin", "button"].includes(p.kind)) {
    const r = p.r || (p.kind === "button" ? 0.6 : 0.9);
    mesh = new THREE.Mesh(new THREE.CylinderGeometry(r, r, 0.05, 28),
      new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.45,
                                       transparent: true, opacity: 0.55 }));
    mesh.position.set(p.x, 0.03, p.z);
    y = 0.4;
  } else if (p.kind === "crate" || p.kind === "block") {
    const s = p.kind === "crate" ? 0.5 : 0.3;
    mesh = new THREE.Mesh(new THREE.BoxGeometry(s, s, s), mat);
    mesh.position.set(p.x, s / 2, p.z);
    y = s / 2;
  } else if (p.kind === "ring") {
    mesh = new THREE.Mesh(new THREE.TorusGeometry(0.9, 0.08, 8, 28), mat);
    mesh.position.set(p.x, p.y, p.z);
    y = p.y;
  }
  mesh.castShadow = true;
  levelGroup.add(mesh);
  return { ...p, label, mesh, pos: new THREE.Vector3(p.x, y, p.z), seen: false,
           pressed: false, carried: false, delivered: false, passed: false,
           sorted: false };
}

function buildLevel(def) {
  if (levelGroup) {
    scene.remove(levelGroup);
    levelGroup.traverse((o) => { o.geometry?.dispose(); o.material?.dispose?.(); });
  }
  LV = def;
  levelGroup = new THREE.Group();
  wallMeshes = []; propObjs = []; movers = [];

  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(def.bounds * 2, def.bounds * 2),
    new THREE.MeshStandardMaterial({ color: 0x18202a, roughness: 0.92 }));
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  levelGroup.add(floor);
  const grid = new THREE.GridHelper(def.bounds * 2, def.bounds * 2, 0x24303c, 0x1b2530);
  grid.position.y = 0.002;
  levelGroup.add(grid);

  const wallMat = new THREE.MeshStandardMaterial({ color: 0x2e3c4a, roughness: 0.8 });
  for (const [x1, z1, x2, z2] of def.walls) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(
      Math.abs(x2 - x1) || W * 2, 1.6, Math.abs(z2 - z1) || W * 2), wallMat);
    m.position.set((x1 + x2) / 2, 0.8, (z1 + z2) / 2);
    m.castShadow = m.receiveShadow = true;
    m.userData.aabb = new THREE.Box3().setFromObject(m);
    levelGroup.add(m);
    wallMeshes.push(m);
  }

  let props = def.props;
  if (props === "reddoor-random") {
    const colors = ["red", "blue", "green", "yellow", "purple"]
      .sort(() => Math.random() - 0.5);
    props = [-12, -6, 0, 6, 12].map((x, i) => ({ kind: "door", color: colors[i], x, z: -1 }));
  }
  for (const p of props || []) propObjs.push(makeProp(p));

  for (const mv of def.movers || []) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(mv.size, 0.8, mv.size),
      new THREE.MeshStandardMaterial({ color: 0xb86a3a, roughness: 0.6 }));
    m.castShadow = true;
    m.userData.aabb = new THREE.Box3();
    levelGroup.add(m);
    wallMeshes.push(m);              // lidar sees it, collision respects it
    movers.push({ ...mv, mesh: m, phase: Math.random() * 6.28 });
  }
  scene.add(levelGroup);
}

function updateMovers(dt) {
  for (const mv of movers) {
    mv.phase += dt * mv.speed;
    mv.mesh.position.set(Math.cos(mv.phase) * mv.r, 0.4, Math.sin(mv.phase) * mv.r);
    mv.mesh.userData.aabb.setFromObject(mv.mesh);
  }
}

/* ════════════════ robot bodies ════════════════ */
const bot = { type: "dog", pos: new THREE.Vector3(), heading: 0, alt: 1,
              group: null, legs: [], phase: 0, grip: false, carrying: null,
              rotors: [], armParts: null };

const bodyMat = new THREE.MeshStandardMaterial({ color: 0xc8cdd4, roughness: 0.5, metalness: 0.35 });
const darkMat = new THREE.MeshStandardMaterial({ color: 0x23282e, roughness: 0.6 });

function legSet(group, hips, L1, L2) {
  const legs = [];
  for (const hip of hips) {
    const thigh = new THREE.Mesh(new THREE.BoxGeometry(0.07, L1, 0.05), darkMat);
    const shin = new THREE.Mesh(new THREE.BoxGeometry(0.05, L2, 0.04), bodyMat);
    thigh.castShadow = shin.castShadow = true;
    thigh.geometry.translate(0, -L1 / 2, 0);
    shin.geometry.translate(0, -L2 / 2, 0);
    const hipPivot = new THREE.Group();
    hipPivot.position.set(hip.x, hip.y, hip.z);
    const kneePivot = new THREE.Group();
    kneePivot.position.set(0, -L1, 0);
    hipPivot.add(thigh); hipPivot.add(kneePivot); kneePivot.add(shin);
    group.add(hipPivot);
    legs.push({ hip, hipPivot, kneePivot, foot: new THREE.Vector3(),
                swing: 0, from: new THREE.Vector3(), to: new THREE.Vector3(),
                L1, L2 });
  }
  return legs;
}

const BODIES = {
  dog(group) {
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.62, 0.18, 0.3), bodyMat);
    body.castShadow = true;
    group.add(body);
    const head = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.14, 0.18), darkMat);
    head.position.set(0.41, 0.06, 0);
    group.add(head);
    bot.legs = legSet(group, [
      { x: 0.25, y: -0.06, z: -0.15, phase: 0 }, { x: 0.25, y: -0.06, z: 0.15, phase: 1 },
      { x: -0.25, y: -0.06, z: -0.15, phase: 1 }, { x: -0.25, y: -0.06, z: 0.15, phase: 0 },
    ], 0.26, 0.26);
    return { standH: 0.42, stepTime: 0.28, eyeH: 0.45, speed: 1.0 };
  },
  biped(group) {
    const torso = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.55, 0.38), bodyMat);
    torso.position.y = 0.25;
    torso.castShadow = true;
    group.add(torso);
    const head = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.2, 0.18), darkMat);
    head.position.y = 0.68;
    head.castShadow = true;
    group.add(head);
    bot.legs = legSet(group, [
      { x: 0, y: -0.03, z: -0.11, phase: 0 }, { x: 0, y: -0.03, z: 0.11, phase: 1 },
    ], 0.42, 0.42);
    return { standH: 0.85, stepTime: 0.42, eyeH: 1.45, speed: 0.6 };
  },
  arm(group) {
    const base = new THREE.Mesh(new THREE.CylinderGeometry(0.35, 0.45, 0.4, 20), darkMat);
    base.position.set(0, 0.2, 0);
    base.castShadow = true;
    group.add(base);
    const seg1 = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.14, 1), bodyMat);
    seg1.geometry.translate(0, 0, 0.5);
    const seg2 = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.1, 1), darkMat);
    seg2.geometry.translate(0, 0, 0.5);
    seg1.castShadow = seg2.castShadow = true;
    const j1 = new THREE.Group(); j1.position.set(0, 0.45, 0);
    const j2 = new THREE.Group(); j2.position.set(0, 0, 1);
    j1.add(seg1); j1.add(j2); j2.add(seg2);
    group.add(j1);
    const eff = new THREE.Mesh(new THREE.SphereGeometry(0.09, 12, 12),
      new THREE.MeshStandardMaterial({ color: 0x00d47e, emissive: 0x00d47e, emissiveIntensity: 0.5 }));
    eff.castShadow = true;
    group.add(eff);
    bot.armParts = { j1, j2, eff, reach: 4.0 };
    return { standH: 0, stepTime: 1, eyeH: 3.0, speed: 1.2 };
  },
  drone(group) {
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.4, 0.12, 0.4), bodyMat);
    body.castShadow = true;
    group.add(body);
    bot.rotors = [];
    for (const [dx, dz] of [[0.26, 0.26], [0.26, -0.26], [-0.26, 0.26], [-0.26, -0.26]]) {
      const rotor = new THREE.Mesh(new THREE.CylinderGeometry(0.16, 0.16, 0.02, 16),
        new THREE.MeshStandardMaterial({ color: 0x4a7ad8, transparent: true, opacity: 0.5 }));
      rotor.position.set(dx, 0.1, dz);
      group.add(rotor);
      bot.rotors.push(rotor);
    }
    return { standH: 1.0, stepTime: 1, eyeH: 0, speed: 1.3 };
  },
};

let bodySpec = null;
function buildBody(type) {
  if (bot.group) {
    scene.remove(bot.group);
    bot.group.traverse((o) => { o.geometry?.dispose(); });
  }
  bot.type = type;
  bot.group = new THREE.Group();
  bot.legs = []; bot.rotors = []; bot.armParts = null;
  bodySpec = BODIES[type](bot.group);
  scene.add(bot.group);
}

function bodyToWorld(bx, bz, out) {
  const c = Math.cos(bot.heading), s = Math.sin(bot.heading);
  return out.set(bot.pos.x + bx * c + bz * s, 0, bot.pos.z - bx * s + bz * c);
}
function worldToBodyX(wx, wz) {
  const c = Math.cos(bot.heading), s = Math.sin(bot.heading);
  return (wx - bot.pos.x) * c - (wz - bot.pos.z) * s;
}
function homeFoot(leg, out) { return bodyToWorld(leg.hip.x, leg.hip.z, out); }

function slideCollide(next, r) {
  for (const m of wallMeshes) {
    const b = m.userData.aabb;
    if (next.x > b.min.x - r && next.x < b.max.x + r &&
        next.z > b.min.z - r && next.z < b.max.z + r) {
      const keepX = bot.pos.clone(); keepX.x = next.x;
      const keepZ = bot.pos.clone(); keepZ.z = next.z;
      const okX = !(keepX.x > b.min.x - r && keepX.x < b.max.x + r &&
                    keepX.z > b.min.z - r && keepX.z < b.max.z + r);
      const okZ = !(keepZ.x > b.min.x - r && keepZ.x < b.max.x + r &&
                    keepZ.z > b.min.z - r && keepZ.z < b.max.z + r);
      next.copy(okX ? keepX : okZ ? keepZ : bot.pos);
    }
  }
  return next;
}

function gaitUpdate(dt, dx, dz, moving) {
  const stepT = bodySpec.stepTime;
  if (moving) bot.phase = (bot.phase + dt / (stepT * 2)) % 1;
  for (const leg of bot.legs) {
    const legPhase = (bot.phase + leg.hip.phase * 0.5) % 1;
    const inSwing = moving && legPhase < 0.5;
    if (inSwing) {
      if (leg.swing === 0) {
        leg.from.copy(leg.foot);
        homeFoot(leg, leg.to);
        leg.to.x += dx / dt * stepT * 1.6;
        leg.to.z += dz / dt * stepT * 1.6;
      }
      leg.swing = legPhase / 0.5;
      leg.foot.lerpVectors(leg.from, leg.to, leg.swing);
      leg.foot.y = Math.sin(leg.swing * Math.PI) * 0.09;
    } else {
      leg.swing = 0;
      leg.foot.y = 0;
    }
  }
  const bob = moving ? Math.sin(bot.phase * Math.PI * 4) * 0.012 : 0;
  bot.group.position.set(bot.pos.x, bodySpec.standH + bob, bot.pos.z);
  bot.group.rotation.y = bot.heading;
  for (const leg of bot.legs) {
    const dxp = worldToBodyX(leg.foot.x, leg.foot.z) - leg.hip.x;
    const dyp = (bodySpec.standH + bob + leg.hip.y) - leg.foot.y;
    const { L1, L2 } = leg;
    const reach = Math.min(Math.hypot(dxp, dyp), L1 + L2 - 0.01);
    const a1 = Math.atan2(dxp, dyp);
    const a2 = Math.acos(THREE.MathUtils.clamp(
      (L1 * L1 + reach * reach - L2 * L2) / (2 * L1 * reach), -1, 1));
    const interior = Math.acos(THREE.MathUtils.clamp(
      (L1 * L1 + L2 * L2 - reach * reach) / (2 * L1 * L2), -1, 1));
    leg.hipPivot.rotation.z = a1 + a2;
    leg.kneePivot.rotation.z = -(Math.PI - interior);
  }
}

function updateBody(dt, cmd) {
  const f = THREE.MathUtils.clamp(cmd.forward || 0, -1, 1) * bodySpec.speed;
  const st = THREE.MathUtils.clamp(cmd.strafe || 0, -1, 1) * bodySpec.speed;
  const yaw = THREE.MathUtils.clamp(cmd.turn || 0, -1.5, 1.5);
  bot.grip = (cmd.grip || 0) > 0.5;

  if (bot.type === "arm") {
    const p = bot.armParts;
    const nx = bot.pos.x + f * dt, nz = bot.pos.z + st * dt;
    if (Math.hypot(nx, nz) < p.reach * 0.95) { bot.pos.x = nx; bot.pos.z = nz; }
    const r = Math.max(0.2, Math.hypot(bot.pos.x, bot.pos.z));
    const az = Math.atan2(bot.pos.x, bot.pos.z);
    const L = p.reach / 2;
    const aa = Math.acos(THREE.MathUtils.clamp(r / (2 * L), -1, 1));
    p.j1.rotation.y = az + aa;
    p.j2.rotation.y = -2 * aa;
    p.eff.position.set(bot.pos.x, 0.45, bot.pos.z);
    if (bot.carrying) bot.carrying.mesh.position.set(bot.pos.x, 0.3, bot.pos.z);
    return;
  }

  bot.heading += yaw * dt;
  const c = Math.cos(bot.heading), s = Math.sin(bot.heading);
  const dx = (f * c + st * s) * dt, dz = (-f * s + st * c) * dt;
  const next = bot.pos.clone(); next.x += dx; next.z += dz;
  slideCollide(next, 0.32);
  bot.pos.copy(next);
  const moving = (Math.hypot(dx, dz) / dt + Math.abs(yaw) * 0.4) > 0.02;

  if (bot.type === "drone") {
    bot.alt = THREE.MathUtils.clamp(bot.alt + (cmd.climb || 0) * dt * 1.2, 0.4, 3.4);
    bot.group.position.set(bot.pos.x, bot.alt, bot.pos.z);
    bot.group.rotation.set(-f * 0.15, bot.heading, st * 0.15, "YXZ");
    for (const r of bot.rotors) r.rotation.y += dt * 40;
    return;
  }
  gaitUpdate(dt, dx || 1e-9, dz || 1e-9, moving);
  if (bot.carrying)
    bot.carrying.mesh.position.set(bot.pos.x, bodySpec.standH + 0.45, bot.pos.z);
}

/* ════════════════ senses ════════════════ */
const raycaster = new THREE.Raycaster();
const LIDAR_RAYS = 36, LIDAR_RANGE = 8;
function eyePos() {
  if (bot.type === "arm") return new THREE.Vector3(bot.pos.x, 0.6, bot.pos.z);
  return new THREE.Vector3(bot.pos.x,
    bot.type === "drone" ? bot.alt : bodySpec.eyeH, bot.pos.z);
}
function fwdVec() { return new THREE.Vector3(Math.cos(bot.heading), 0, -Math.sin(bot.heading)); }

let currentDets = [];   // [{p, dist}] — what the robot knows right now
function senseDetections() {
  const out = [];
  const dets = [];
  const eye = eyePos(), fwd = fwdVec();
  for (const p of propObjs) {
    if (p.carried || p.delivered || p.sorted) continue;
    const ppos = p.kind === "crate" || p.kind === "block" ? p.mesh.position : p.pos;
    const to = ppos.clone().sub(eye);
    const dist = to.length();
    if (dist > (bot.type === "arm" ? 6 : 8)) continue;
    let bearing;
    if (bot.type === "arm") {
      bearing = Math.atan2(-(ppos.z - eye.z), ppos.x - eye.x);  // camera looks along +x
      if (Math.abs(bearing) > Math.PI) continue;
    } else {
      // natural convention: world angle of the target minus heading, so a
      // policy recovers the world direction with heading + bearing
      const phi = Math.atan2(-to.z, to.x);
      bearing = ((phi - bot.heading + Math.PI * 3) % (Math.PI * 2)) - Math.PI;
      if (Math.abs(bearing) > 0.62) continue;
      raycaster.set(eye, to.clone().normalize());
      const hit = raycaster.intersectObjects(wallMeshes, false)[0];
      if (hit && hit.distance < dist - 0.4) continue;
    }
    const cx = 640 - (bearing / 0.62) * 600;
    const size = Math.min(420, 2200 / Math.max(dist, 0.4));
    out.push({ label: p.label, confidence: 0.95,
               bbox: [cx - size / 4, 360 - size / 2, cx + size / 4, 360 + size / 2],
               distance: +dist.toFixed(2) });
    dets.push({ p, dist, ppos });
    if (!p.seen) { p.seen = true; postEvent("detection", `sighted: ${p.label}`, {}); }
  }
  currentDets = dets;
  if (bot.type === "dog" || bot.type === "biped") {
    raycaster.set(eyePos(), fwdVec());
    const ahead = raycaster.intersectObjects(wallMeshes, false)[0];
    if (ahead && ahead.distance < 1.6) {
      const size = 700 / ahead.distance;
      out.push({ label: "obstacle", confidence: 1.0,
                 bbox: [640 - size / 2, 360 - size / 2, 640 + size / 2, 360 + size / 2],
                 distance: +ahead.distance.toFixed(2) });
    }
  }
  return out;
}

function senseLidar() {
  if (bot.type === "arm") return [];
  const eye = new THREE.Vector3(bot.pos.x, 0.45, bot.pos.z);
  const ranges = [];
  for (let i = 0; i < LIDAR_RAYS; i++) {
    const a = bot.heading + (i / LIDAR_RAYS) * Math.PI * 2;
    raycaster.set(eye, new THREE.Vector3(Math.cos(a), 0, -Math.sin(a)));
    raycaster.far = LIDAR_RANGE;
    const hit = raycaster.intersectObjects(wallMeshes, false)[0];
    ranges.push(hit ? +hit.distance.toFixed(2) : LIDAR_RANGE);
  }
  raycaster.far = Infinity;
  return ranges;
}

/* ════════════════ lidar cloud + robot map ════════════════ */
const CLOUD_MAX = 80000;
const cloudPos = new Float32Array(CLOUD_MAX * 3);
const cloudCol = new Float32Array(CLOUD_MAX * 3);
let cloudCount = 0, cloudHead = 0;
const cloudGeo = new THREE.BufferGeometry();
cloudGeo.setAttribute("position", new THREE.BufferAttribute(cloudPos, 3).setUsage(THREE.DynamicDrawUsage));
cloudGeo.setAttribute("color", new THREE.BufferAttribute(cloudCol, 3).setUsage(THREE.DynamicDrawUsage));
const cloud = new THREE.Points(cloudGeo,
  new THREE.PointsMaterial({ size: 0.035, vertexColors: true, sizeAttenuation: true }));
cloud.frustumCulled = false;
let cloudOn = true;
scene.add(cloud);
const _tmpColor = new THREE.Color();
function cloudAdd(x, y, z, range) {
  const i = cloudHead * 3;
  cloudPos[i] = x; cloudPos[i + 1] = y; cloudPos[i + 2] = z;
  // range-colored like a real lidar viz: near = warm, far = cool
  _tmpColor.setHSL(0.66 * Math.min(range / LIDAR_RANGE, 1), 0.9, 0.5);
  cloudCol[i] = _tmpColor.r; cloudCol[i + 1] = _tmpColor.g; cloudCol[i + 2] = _tmpColor.b;
  cloudHead = (cloudHead + 1) % CLOUD_MAX;
  cloudCount = Math.min(cloudCount + 1, CLOUD_MAX);
}
function cloudCommit() {
  cloudGeo.attributes.position.needsUpdate = true;
  cloudGeo.attributes.color.needsUpdate = true;
  cloudGeo.setDrawRange(0, cloudCount);
}
function cloudReset() { cloudCount = 0; cloudHead = 0; cloudGeo.setDrawRange(0, 0); }

const GRID = 96;
let CELL = 32 / GRID;
let occ = new Uint8Array(GRID * GRID);
const mapCanvas = document.getElementById("map");
const mapCtx = mapCanvas.getContext("2d");
function cellOf(x, z) {
  return [Math.floor((x + LV.bounds) / CELL), Math.floor((z + LV.bounds) / CELL)];
}
function integrateLidar(ranges) {
  for (let i = 0; i < ranges.length; i++) {
    const a = bot.heading + (i / ranges.length) * Math.PI * 2;
    const dx = Math.cos(a), dz = -Math.sin(a);
    for (let r = 0.2; r < ranges[i]; r += CELL * 0.8) {
      const [cx, cz] = cellOf(bot.pos.x + dx * r, bot.pos.z + dz * r);
      if (cx >= 0 && cx < GRID && cz >= 0 && cz < GRID && occ[cz * GRID + cx] !== 2)
        occ[cz * GRID + cx] = 1;
    }
    if (ranges[i] < LIDAR_RANGE) {
      const hx = bot.pos.x + dx * ranges[i], hz = bot.pos.z + dz * ranges[i];
      const [cx, cz] = cellOf(hx, hz);
      if (cx >= 0 && cx < GRID && cz >= 0 && cz < GRID) occ[cz * GRID + cx] = 2;
      for (let k = 0; k < 5; k++)
        cloudAdd(hx + (Math.random() - 0.5) * 0.05, Math.random() * 1.55,
                 hz + (Math.random() - 0.5) * 0.05, ranges[i]);
    }
  }
}
function drawMap() {
  const img = mapCtx.createImageData(GRID, GRID);
  for (let i = 0; i < occ.length; i++) {
    const v = occ[i];
    const [r, g, b] = v === 2 ? [120, 170, 200] : v === 1 ? [22, 34, 30] : [8, 10, 12];
    img.data[i * 4] = r; img.data[i * 4 + 1] = g; img.data[i * 4 + 2] = b;
    img.data[i * 4 + 3] = 255;
  }
  mapCtx.putImageData(img, 0, 0);
  const [cx, cz] = cellOf(bot.pos.x, bot.pos.z);
  mapCtx.fillStyle = "#00d47e";
  mapCtx.fillRect(cx - 1, cz - 1, 3, 3);
}

/* ════════════════ chamber engine ════════════════ */
const briefTitle = document.getElementById("briefTitle");
const briefText = document.getElementById("briefText");
const roomsEl = document.getElementById("rooms");
const levelSel = document.getElementById("levelSel");
for (let i = 0; i < LEVELS.length; i++) {
  const o = document.createElement("option");
  o.value = i; o.textContent = LEVELS[i].title;
  levelSel.appendChild(o);
}
let visited = new Set(), won = false, t0 = performance.now();
let usedManual = false, levelIndex = 0;
let chips = [];
let winState = {};
let lastAnswerTs = 0;

function setChips(list) {
  chips = list;
  roomsEl.innerHTML = "";
  for (const c of chips) {
    const el = document.createElement("span");
    el.className = "room-chip" + (c.done ? " seen" : "");
    el.id = `chip-${c.id}`;
    el.textContent = c.label;
    roomsEl.appendChild(el);
  }
}
function markChip(id) {
  const c = chips.find((c) => c.id === id);
  if (c && !c.done) {
    c.done = true;
    document.getElementById(`chip-${id}`)?.classList.add("seen");
  }
}

function loadLevel(i) {
  levelIndex = ((i % LEVELS.length) + LEVELS.length) % LEVELS.length;
  buildLevel(LEVELS[levelIndex]);
  buildBody(LV.robot);
  levelSel.value = levelIndex;
  briefTitle.textContent = LV.title;
  briefText.textContent = LV.brief;
  const sp = LV.spawn.random === "corridor"
    ? { x: Math.random() * 26 - 13, z: 3 + Math.random() * 3, heading: Math.random() * 6.28 }
    : LV.spawn;
  bot.pos.set(sp.x, 0, sp.z);
  bot.heading = sp.heading || 0;
  bot.alt = 1.0;
  bot.carrying = null;
  for (const leg of bot.legs) homeFoot(leg, leg.foot);
  visited = new Set(); won = false; usedManual = false;
  winState = { idx: 0, lap: 0, hold: 0, delivered: 0, sorted: 0 };
  t0 = performance.now();
  occ = new Uint8Array(GRID * GRID);
  CELL = (LV.bounds * 2) / GRID;
  cloudReset();
  odo = 0;
  document.getElementById("win").classList.remove("show");
  setCode(LV.demo || "");
  policyStatus("demo policy loaded — press RUN, or rewrite it", "");

  const w = LV.win;
  if (LV.rooms?.length) setChips(LV.rooms.map((r) => ({ id: r.id, label: r.id, done: false })));
  else chips = [], roomsEl.innerHTML = "";
  if (w.type === "answer") setChips([{ id: "q", label: w.question, done: false },
    ...(LV.rooms || []).map((r) => ({ id: r.id, label: r.id, done: false }))]);
  else if (w.type === "near") setChips([{ id: "near", label: `reach the ${w.label}`, done: false }]);
  else if (w.type === "checkpoints") setChips(
    Array.from({ length: w.laps }, (_, l) => ({ id: `lap${l}`, label: `lap ${l + 1}`, done: false })));
  else if (w.type === "buttons") setChips(
    w.order.map((id) => ({ id: `b${id}`, label: propObjs.find((p) => p.id === id)?.label || id, done: false })));
  else if (w.type === "carry") setChips(
    Array.from({ length: w.crates }, (_, i) => ({ id: `c${i}`, label: `crate ${i + 1}`, done: false })));
  else if (w.type === "sort") setChips(
    Array.from({ length: w.blocks }, (_, i) => ({ id: `s${i}`, label: `block ${i + 1}`, done: false })));
  else if (w.type === "rings") setChips(
    w.order.map((id) => ({ id: `r${id}`, label: `ring ${id + 1}`, done: false })));
  postEvent("arena", `level loaded: ${LV.name}`, { robot: LV.robot });
}
levelSel.addEventListener("change", () => loadLevel(+levelSel.value));

function winChamber(detail) {
  won = true;
  const secs = ((performance.now() - t0) / 1000).toFixed(1);
  document.getElementById("winTitle").textContent =
    usedManual ? "PRACTICE RUN — MANUAL DRIVE" : "CHAMBER COMPLETE — AUTONOMOUS";
  document.getElementById("winDetail").textContent = `${detail} · ${secs}s` +
    (usedManual ? " · write a policy to make it count" : "");
  document.getElementById("win").classList.add("show");
  document.querySelector("#win .card").classList.toggle("practice", usedManual);
  postEvent("task", `${LV.title} ${usedManual ? "cleared MANUALLY (practice)" : "COMPLETE — autonomous"} · ${secs}s`,
            { time_s: +secs, level: LV.name, mode: usedManual ? "manual" : "autonomous" });
  sealAttempt();
}

function dist2d(a, bx, bz) { return Math.hypot(a.x - bx, a.z - bz); }

function tickChamber(dt, answer) {
  if (won || !LV) return;
  const w = LV.win;

  for (const r of LV.rooms || []) {
    const [x1, z1, x2, z2] = r.rect;
    if (!visited.has(r.id) && bot.pos.x > x1 && bot.pos.x < x2 && bot.pos.z > z1 && bot.pos.z < z2) {
      visited.add(r.id);
      markChip(r.id);
      postEvent("arena", `room explored: ${r.id}`, {});
    }
  }

  if (w.type === "answer" && answer && answer.ts > lastAnswerTs) {
    lastAnswerTs = answer.ts;
    if (String(answer.text).trim() === w.value) {
      markChip("q");
      winChamber(`correct: ${w.value} doors`);
    } else {
      postEvent("arena", `wrong answer: ${answer.text} (keep looking)`, {});
    }
  }

  if (w.type === "near") {
    const target = propObjs.find((p) => p.label === w.label);
    if (target) {
      const close = dist2d(bot.pos, target.pos.x, target.pos.z) < w.dist;
      winState.hold = close ? winState.hold + dt : 0;
      if (winState.hold >= w.hold) { markChip("near"); winChamber(`found the ${w.label}`); }
    }
  }

  if (w.type === "checkpoints") {
    const want = propObjs.find((p) => p.id === w.order[winState.idx % w.order.length]);
    if (want && dist2d(bot.pos, want.pos.x, want.pos.z) < w.dist) {
      winState.idx += 1;
      postEvent("arena", `checkpoint ${want.id + 1}`, {});
      if (winState.idx % w.order.length === 0) {
        markChip(`lap${winState.lap}`);
        winState.lap += 1;
        if (winState.lap >= w.laps) winChamber(`${w.laps} laps patrolled`);
      }
    }
  }

  if (w.type === "buttons") {
    const want = propObjs.find((p) => p.id === w.order[winState.idx]);
    if (want) {
      const on = dist2d(bot.pos, want.pos.x, want.pos.z) < w.dist;
      winState.hold = on ? winState.hold + dt : 0;
      if (winState.hold >= w.hold) {
        want.pressed = true;
        want.mesh.material.opacity = 1;
        markChip(`b${want.id}`);
        postEvent("arena", `button pressed: ${want.label}`, {});
        winState.idx += 1; winState.hold = 0;
        if (winState.idx >= w.order.length) winChamber("sequence complete");
      }
    }
  }

  if (w.type === "carry") {
    const zone = propObjs.find((p) => p.kind === "zone");
    if (!bot.carrying) {
      const crate = propObjs.find((p) => p.kind === "crate" && !p.delivered &&
        dist2d(bot.pos, p.mesh.position.x, p.mesh.position.z) < w.pickup);
      if (crate) { bot.carrying = crate; crate.carried = true;
                   postEvent("arena", "crate picked up", {}); }
    } else if (zone && dist2d(bot.pos, zone.pos.x, zone.pos.z) < w.zone) {
      bot.carrying.delivered = true;
      bot.carrying.carried = false;
      bot.carrying.mesh.position.set(zone.pos.x + (winState.delivered - 0.5), 0.25, zone.pos.z);
      bot.carrying = null;
      markChip(`c${winState.delivered}`);
      winState.delivered += 1;
      postEvent("arena", `crate delivered (${winState.delivered}/${w.crates})`, {});
      if (winState.delivered >= w.crates) winChamber("all crates delivered");
    }
  }

  if (w.type === "sort") {
    if (bot.grip && !bot.carrying) {
      const block = propObjs.find((p) => p.kind === "block" && !p.sorted &&
        dist2d(bot.pos, p.mesh.position.x, p.mesh.position.z) < w.grabDist);
      if (block) { bot.carrying = block; block.carried = true; }
    }
    if (!bot.grip && bot.carrying) {
      const block = bot.carrying;
      bot.carrying = null; block.carried = false;
      const bin = propObjs.find((p) => p.kind === "bin" &&
        dist2d({ x: block.mesh.position.x, z: block.mesh.position.z }, p.pos.x, p.pos.z) < p.r);
      if (bin && bin.color === block.color) {
        block.sorted = true;
        block.mesh.position.set(bin.pos.x, 0.18 + winState.sorted * 0.12, bin.pos.z);
        markChip(`s${winState.sorted}`);
        winState.sorted += 1;
        postEvent("arena", `sorted ${block.label} (${winState.sorted}/${w.blocks})`, {});
        if (winState.sorted >= w.blocks) winChamber("table sorted");
      }
    }
  }

  if (w.type === "rings") {
    const want = propObjs.find((p) => p.id === w.order[winState.idx]);
    if (want && dist2d(bot.pos, want.pos.x, want.pos.z) < w.dist &&
        Math.abs(bot.alt - want.pos.y) < w.alt) {
      want.passed = true;
      want.mesh.material.emissiveIntensity = 1.2;
      markChip(`r${want.id}`);
      postEvent("arena", `ring ${want.id + 1}`, {});
      winState.idx += 1;
      if (winState.idx >= w.order.length) winChamber("course complete");
    }
  }
}

/* ════════════════ recording ════════════════ */
let recording = false;
async function startAttemptRecording() {
  try {
    const r = await api("/api/run/record/start", { robot_id: "arena" });
    recording = !!r.ok;
    document.getElementById("rec").textContent = recording ? "● REC" : "";
  } catch {}
}
async function sealAttempt() {
  const hashEl = document.getElementById("winHash");
  try {
    const r = await api("/api/run/record/stop", {});
    const root = r?.seal?.merkle_root;
    if (root) {
      hashEl.innerHTML = `<span class="k">run-hash</span>0x${root}`;
      document.getElementById("rec").textContent = "";
      recording = false;
      return;
    }
    hashEl.innerHTML = `<span class="k">run-hash</span>unrecorded — server wasn't running`;
  } catch {
    hashEl.innerHTML = `<span class="k">run-hash</span>unavailable`;
  }
}

/* ════════════════ the wire ════════════════ */
async function api(path, body) {
  const r = await fetch(path, { method: "POST",
    headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) });
  return r.json();
}
let serverCmd = { forward: 0, strafe: 0, turn: 0, climb: 0, grip: 0 };
let serverAnswer = null;
let linked = false, lastLidar = [];
async function pollCmd() {
  try {
    const r = await (await fetch("/api/arena/cmd")).json();
    serverCmd = r.cmd;
    serverAnswer = r.answer;
    if (!linked) { linked = true; startAttemptRecording(); }
  } catch { linked = false; }
  const el = document.getElementById("link");
  el.textContent = linked ? "behaviors: linked" : "behaviors: no server (WASD only)";
  el.className = `link ${linked ? "on" : "off"}`;
  setTimeout(pollCmd, 50);
}
async function pushState() {
  if (linked) {
    try {
      await api("/api/arena/state", {
        detections: senseDetections(),
        lidar: lastLidar,
        pose: { x: +bot.pos.x.toFixed(2), z: +bot.pos.z.toFixed(2),
                y: +bot.alt.toFixed(2), heading: +bot.heading.toFixed(3) },
        level: { name: LV.name, robot: LV.robot, room: currentRoom(),
                 rooms_visited: [...visited], odometer_m: +odo.toFixed(1), won },
      });
    } catch {}
  }
  setTimeout(pushState, 100);
}
function postEvent(type, title, detail) {
  if (!linked) return;
  api("/api/arena/event", { type, title, detail }).catch(() => {});
}

/* ════════════════ policy editor ════════════════ */
const codeEl = document.getElementById("code");
const statusEl = document.getElementById("policyStatus");
function policyStatus(msg, cls) { statusEl.textContent = msg; statusEl.className = cls; }

let cm = null;
function getCode() { return cm ? cm.state.doc.toString() : codeEl.value; }
function setCode(text) {
  if (cm) cm.dispatch({ changes: { from: 0, to: cm.state.doc.length, insert: text } });
  else codeEl.value = text;
}
(async () => {
  try {
    const [{ basicSetup, EditorView }, { python }, { oneDark }] = await Promise.all([
      import("https://esm.sh/codemirror@6.0.1"),
      import("https://esm.sh/@codemirror/lang-python@6.1.6"),
      import("https://esm.sh/@codemirror/theme-one-dark@6.1.2"),
    ]);
    cm = new EditorView({
      doc: codeEl.value,
      extensions: [basicSetup, python(), oneDark],
      parent: document.getElementById("editorHost"),
    });
    codeEl.style.display = "none";
    document.getElementById("editorHost").style.display = "block";
    policyStatus("editor ready — ⌘⏎ runs", "");
  } catch {
    policyStatus("plain editor (CDN offline) — ⌘⏎ runs", "");
  }
})();
document.getElementById("p-policy").addEventListener("keydown", (e) => {
  if ((e.metaKey || e.ctrlKey) && e.key === "Enter") {
    e.preventDefault();
    document.getElementById("btnRun").click();
  }
  e.stopPropagation();
});
document.getElementById("p-policy").addEventListener("keyup", (e) => e.stopPropagation());
codeEl.addEventListener("keydown", (e) => {
  if (e.key === "Tab") {
    e.preventDefault();
    const { selectionStart: a, selectionEnd: b, value: v } = codeEl;
    codeEl.value = v.slice(0, a) + "    " + v.slice(b);
    codeEl.selectionStart = codeEl.selectionEnd = a + 4;
  }
});

const connectEl = document.getElementById("connect");
document.getElementById("btnConnect").addEventListener("click", () => connectEl.classList.add("show"));
document.getElementById("connectClose").addEventListener("click", () => connectEl.classList.remove("show"));
connectEl.addEventListener("click", (e) => { if (e.target === connectEl) connectEl.classList.remove("show"); });
for (const pre of connectEl.querySelectorAll("pre")) {
  pre.title = "click to copy";
  pre.addEventListener("click", async () => {
    try { await navigator.clipboard.writeText(pre.textContent.trim());
          pre.style.borderColor = "#00d47e";
          setTimeout(() => pre.style.borderColor = "", 600); } catch {}
  });
}

document.getElementById("btnRun").addEventListener("click", async () => {
  let source = getCode();
  try {
    if (!source.includes("@behavior")) {
      // words, not code: compile the mission into a policy first
      policyStatus("✨ compiling mission via LLM… (~10s)", "");
      const c = await api("/api/behaviors/compile",
                          { mission: source, context: LV.title + ": " + LV.brief });
      if (!c.ok) { policyStatus(c.error, "err"); return; }
      source = c.source;
      setCode(source);                 // language in, code out — inspect/edit it
    }
    policyStatus("saving…", "");
    const w = await api("/api/behaviors/write", { name: "player_policy", source });
    if (!w.ok) { policyStatus(w.error, "err"); return; }
    await api("/api/behaviors/enable", { name: "player_policy" });
    policyStatus("running — hot reload applies edits on every RUN", "ok");
  } catch { policyStatus("no server — run `roborun` first", "err"); }
});
document.getElementById("btnStop").addEventListener("click", async () => {
  try {
    await api("/api/behaviors/disable", { name: "player_policy" });
    policyStatus("stopped", "");
  } catch {}
});

/* ════════════════ telemetry ════════════════ */
let odo = 0;
const prevPos = new THREE.Vector3();
function currentRoom() {
  for (const r of LV.rooms || []) {
    const [x1, z1, x2, z2] = r.rect;
    if (bot.pos.x > x1 && bot.pos.x < x2 && bot.pos.z > z1 && bot.pos.z < z2) return r.id;
  }
  return "field";
}
function updateTelemetry() {
  document.getElementById("teleRoom").textContent = `room ${currentRoom()}`;
  document.getElementById("teleOdo").textContent = `odometer ${odo.toFixed(1)} m`;
  document.getElementById("telePose").textContent = bot.type === "drone"
    ? `x ${bot.pos.x.toFixed(1)} · z ${bot.pos.z.toFixed(1)} · alt ${bot.alt.toFixed(1)}`
    : `x ${bot.pos.x.toFixed(1)} · z ${bot.pos.z.toFixed(1)} · θ ${bot.heading.toFixed(2)}`;
}

/* ════════════════ panels ════════════════ */
const LAYOUT_KEY = "arena-layout-v2";
const PANEL_IDS = ["p-brief", "p-policy", "p-status", "p-map", "p-view1", "p-view2"];
let zTop = 100;
function defaultLayout() {
  const w = innerWidth, h = innerHeight;
  return {
    "p-brief":  { l: 14, t: 52, w: 360, h: 190, hidden: false },
    "p-policy": { l: 14, t: 252, w: 470, h: Math.min(430, h - 270), hidden: false },
    "p-status": { l: w - 230, t: 52, w: 216, h: 150, hidden: false },
    "p-map":    { l: w - 230, t: 212, w: 216, h: 240, hidden: false },
    "p-view1":  { l: w - 340, t: h - 230, w: 326, h: 216, hidden: false },
    "p-view2":  { l: w - 680, t: h - 230, w: 326, h: 216, hidden: false },
  };
}
function loadLayout() {
  try { return { ...defaultLayout(), ...JSON.parse(localStorage.getItem(LAYOUT_KEY) || "{}") }; }
  catch { return defaultLayout(); }
}
let layout = loadLayout();
function saveLayout() { localStorage.setItem(LAYOUT_KEY, JSON.stringify(layout)); }
function applyLayout() {
  for (const id of PANEL_IDS) {
    const el = document.getElementById(id), st = layout[id];
    if (!el || !st) continue;
    el.style.left = `${Math.max(0, Math.min(st.l, innerWidth - 60))}px`;
    el.style.top = `${Math.max(0, Math.min(st.t, innerHeight - 40))}px`;
    el.style.width = `${st.w}px`;
    el.style.height = `${st.h}px`;
    el.classList.toggle("hidden", !!st.hidden);
    document.querySelector(`#toolbar [data-panel="${id}"]`)
      ?.classList.toggle("on", !st.hidden);
  }
}
function initPanels() {
  for (const id of PANEL_IDS) {
    const el = document.getElementById(id);
    const head = el.querySelector(".p-head");
    el.addEventListener("pointerdown", () => { el.style.zIndex = ++zTop; });
    head.addEventListener("pointerdown", (e) => {
      if (e.target.tagName === "SELECT" || e.target.classList.contains("x")) return;
      e.preventDefault();
      const sx = e.clientX - el.offsetLeft, sy = e.clientY - el.offsetTop;
      function move(ev) {
        layout[id].l = ev.clientX - sx; layout[id].t = ev.clientY - sy;
        el.style.left = `${layout[id].l}px`; el.style.top = `${layout[id].t}px`;
      }
      function up() {
        removeEventListener("pointermove", move); removeEventListener("pointerup", up);
        saveLayout();
      }
      addEventListener("pointermove", move); addEventListener("pointerup", up);
    });
    head.querySelector(".x").addEventListener("click", () => {
      layout[id].hidden = true; saveLayout(); applyLayout();
    });
    new ResizeObserver(() => {
      if (el.classList.contains("hidden")) return;
      layout[id].w = el.offsetWidth; layout[id].h = el.offsetHeight; saveLayout();
    }).observe(el);
  }
  for (const btn of document.querySelectorAll("#toolbar [data-panel]")) {
    btn.addEventListener("click", () => {
      const id = btn.dataset.panel;
      layout[id].hidden = !layout[id].hidden;
      saveLayout(); applyLayout();
    });
  }
  document.getElementById("btnReset").addEventListener("click", () => {
    layout = defaultLayout(); saveLayout(); applyLayout();
  });
  applyLayout();
}
initPanels();

/* ════════════════ input ════════════════ */
const keys = {};
addEventListener("keydown", (e) => {
  if (e.target.closest?.("#p-policy") ||
      ["SELECT", "TEXTAREA", "INPUT"].includes(e.target.tagName)) return;
  keys[e.key.toLowerCase()] = true;
  if (e.key.toLowerCase() === "c") camMode = (camMode + 1) % CAM_MODES.length;
  if (e.key.toLowerCase() === "n") loadLevel(levelIndex + 1);
  if (e.key.toLowerCase() === "l") { cloudOn = !cloudOn; cloud.visible = cloudOn; }
  if (e.key === "Escape") connectEl.classList.remove("show");
});
addEventListener("keyup", (e) => keys[e.key.toLowerCase()] = false);
function keyboardCmd() {
  const f = (keys.w ? 1 : 0) - (keys.s ? 1 : 0);
  const t = (keys.a ? 1 : 0) - (keys.d ? 1 : 0);
  const cl = (keys.e ? 1 : 0) - (keys.q ? 1 : 0);
  if (f || t || cl) {
    usedManual = true;
    return { forward: f * 0.9, strafe: 0, turn: t * 1.2, climb: cl * 0.8,
             grip: keys[" "] ? 1 : 0 };
  }
  return null;
}

/* ════════════════ multi-view render ════════════════ */
function renderViews() {
  const w = innerWidth, h = innerHeight;
  renderer.setScissorTest(true);
  renderer.setViewport(0, 0, w, h);
  renderer.setScissor(0, 0, w, h);
  renderer.clear();
  renderer.render(scene, camMode === 2 ? topCam : specCam);
  for (const panel of document.querySelectorAll(".view-panel")) {
    if (panel.classList.contains("hidden")) continue;
    const vp = panel.querySelector(".viewport").getBoundingClientRect();
    if (vp.width < 40 || vp.height < 40) continue;
    const cam = CAM_POOL[panel.querySelector(".camSel").value] || povCam;
    if (cam.isPerspectiveCamera) {
      cam.aspect = vp.width / vp.height;
      cam.updateProjectionMatrix();
    }
    const x = Math.round(vp.left), y = Math.round(h - vp.bottom);
    renderer.setViewport(x, y, Math.round(vp.width), Math.round(vp.height));
    renderer.setScissor(x, y, Math.round(vp.width), Math.round(vp.height));
    renderer.clear(true, true, false);
    renderer.render(scene, cam);
    drawDetOverlay(panel, cam === povCam, vp);
  }
  renderer.setScissorTest(false);
}

const _proj = new THREE.Vector3();
function drawDetOverlay(panel, isPov, vp) {
  let cv = panel.querySelector(".det-overlay");
  if (!cv) {
    cv = document.createElement("canvas");
    cv.className = "det-overlay";
    panel.querySelector(".p-body").appendChild(cv);
  }
  const w = Math.round(vp.width), h = Math.round(vp.height);
  if (cv.width !== w || cv.height !== h) { cv.width = w; cv.height = h; }
  const g = cv.getContext("2d");
  g.clearRect(0, 0, w, h);
  if (!isPov) return;                      // perception overlay = robot cam only
  g.font = "10px ui-monospace, Menlo, monospace";
  g.lineWidth = 1.5;
  for (const d of currentDets) {
    _proj.copy(d.ppos).project(povCam);
    if (_proj.z > 1 || Math.abs(_proj.x) > 1.1 || Math.abs(_proj.y) > 1.1) continue;
    const px = (_proj.x + 1) / 2 * w, py = (1 - _proj.y) / 2 * h;
    const sz = Math.max(14, Math.min(120, 160 / d.dist));
    const color = "#" + new THREE.Color(COLORS[d.p.color] || 0x00d47e).getHexString();
    g.strokeStyle = color;
    g.strokeRect(px - sz / 2, py - sz / 2, sz, sz);
    g.fillStyle = color;
    g.fillText(d.p.label + " " + d.dist.toFixed(1) + "m", px - sz / 2, py - sz / 2 - 3);
  }
}

/* ════════════════ main loop ════════════════ */
const clockEl = document.getElementById("clock"), cmdEl = document.getElementById("cmdline");
let last = performance.now(), senseTick = 0, orbitAngle = 0;
function frame(now) {
  const dt = Math.min((now - last) / 1000, 0.05);
  last = now;
  const cmd = keyboardCmd() || serverCmd;
  updateMovers(dt);
  updateBody(dt, cmd);
  tickChamber(dt, serverAnswer);

  odo += prevPos.distanceTo(bot.pos);
  prevPos.copy(bot.pos);
  senseTick += dt;
  if (senseTick > 0.12) {
    senseTick = 0;
    lastLidar = senseLidar();
    integrateLidar(lastLidar);
    cloudCommit();
    drawMap();
    updateTelemetry();
  }

  const focusY = bot.type === "drone" ? bot.alt : 0.5;
  if (camMode === 0) {
    specCam.position.lerp(new THREE.Vector3(
      bot.pos.x - Math.cos(bot.heading) * 3.4, focusY + 2,
      bot.pos.z + Math.sin(bot.heading) * 3.4), 0.06);
    specCam.lookAt(bot.pos.x, focusY, bot.pos.z);
  } else if (camMode === 1) {
    specCam.position.copy(orbitCam.position);
    specCam.quaternion.copy(orbitCam.quaternion);
  }
  const fwd = fwdVec();
  const eye = eyePos();
  povCam.position.set(eye.x + fwd.x * 0.35, eye.y, eye.z + fwd.z * 0.35);
  povCam.lookAt(eye.x + fwd.x * 5, bot.type === "arm" ? 0 : eye.y - 0.05, eye.z + fwd.z * 5);
  chaseCam.position.lerp(new THREE.Vector3(
    bot.pos.x - Math.cos(bot.heading) * 3.4, focusY + 2,
    bot.pos.z + Math.sin(bot.heading) * 3.4), 0.08);
  chaseCam.lookAt(bot.pos.x, focusY, bot.pos.z);
  orbitCam.position.set(bot.pos.x + Math.cos(orbitAngle) * 6, focusY + 3.6,
                        bot.pos.z + Math.sin(orbitAngle) * 6);
  orbitCam.lookAt(bot.pos.x, focusY, bot.pos.z);
  orbitAngle += dt * 0.25;

  if (!won) clockEl.textContent = `${((now - t0) / 1000).toFixed(1)}s`;
  cmdEl.textContent = `cmd f=${(cmd.forward || 0).toFixed(2)} t=${(cmd.turn || 0).toFixed(2)} · cam ${CAM_MODES[camMode]}`;
  renderViews();
  requestAnimationFrame(frame);
}

loadLevel(0);
pollCmd(); pushState();
requestAnimationFrame(frame);
