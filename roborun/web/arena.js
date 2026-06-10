/* RoboRun Arena.
   Self-contained browser sim: procedural-gait quadruped (planted feet),
   robot senses = forward camera cone + 360° lidar, nothing else.
   Levels are data; switch with the HUD selector or N. Behaviors drive
   via GET /api/arena/cmd; attempts auto-record; wins seal and show the
   run-hash. */

import * as THREE from "three";

/* ---------- levels ---------- */
const LEVELS = [
  {
    name: "chamber-01",
    title: "CHAMBER 01 — RECON",
    brief: "POLICY GOAL: visit all four rooms, autonomously. The robot has "
         + "see() — doors, obstacles — and move(). Drop a file in behaviors/ "
         + "or let your agent write it over MCP (write_behavior). "
         + "WASD is debug-drive only: manual runs count as practice.",
    bounds: 16,
    spawn: { x: 0, z: 0, heading: 0 },
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
    doors: [
      { id: "d1", x: -2.5, z: -1, color: "red" },
      { id: "d2", x: 2.5, z: -1, color: "blue" },
      { id: "d3", x: -2.5, z: 1, color: "red" },
      { id: "d4", x: 2.5, z: 1, color: "green" },
      { id: "d5", x: -1, z: -4.7, color: "red" },
      { id: "d6", x: 1, z: 4.7, color: "blue" },
    ],
    demo: `from roborun.behaviors import behavior

@behavior(hz=10)
def player_policy(robot):
    # L1 reflex — runs at 10 Hz, never waits on anything
    scan = robot.lidar()               # 36 ranges (m), [0] = ahead
    ahead = min(scan[0:3] + scan[-3:]) if scan else 8
    if ahead < 1.2:
        robot.move(turn=1.1)
    else:
        robot.move(forward=0.8, turn=0.15)

    # keep a ledger of what we saw (recon levels ask questions later)
    for d in robot.see("red door"):
        robot.state.setdefault("red_doors", set()).add(round(d.cx, 1))

    # escalate when stuck: async — the loop keeps driving while the
    # LLM thinks, and it has tools (it may even rewrite this file).
    st = robot.state
    st["blocked"] = st.get("blocked", 0) + 1 if ahead < 1.2 else 0
    if st["blocked"] > 60 and not robot.thinking("fix"):
        robot.delegate(
            "I'm a wall-following policy stuck spinning in the arena. "
            "Check arena_status and see, then write_behavior a smarter "
            "player_policy (hz=10, use robot.lidar()).", key="fix")
    report = robot.thought("fix")
    if report:
        robot.say(report)
`,
  },
  {
    name: "chamber-02",
    title: "CHAMBER 02 — SPRINT",
    brief: "POLICY GOAL: reach the beacon and hold it 1.5s. see('beacon') "
         + "gives bearing via .cx; see('obstacle') warns of walls. "
         + "The pillars do not move. Your policy might.",
    bounds: 16,
    spawn: { x: -6.5, z: -6.5, heading: -0.7 },
    rooms: [],
    walls: [
      [-8, -8, 8, -8], [-8, 8, 8, 8], [-8, -8, -8, 8], [8, -8, 8, 8],
      [-4, -5, -4, -1], [0, -3, 0, 2], [-2, 4, 3, 4],
      [4, -6, 4, -2], [4, 1, 4, 5], [-6, 0, -6, 4], [-2, -8, -2, -6.5],
    ],
    doors: [{ id: "d1", x: 0, z: -3, color: "red" }],
    goal: { x: 6.5, z: 6.5, r: 1.1, hold: 1.5 },
    demo: `from roborun.behaviors import behavior

@behavior(hz=10)
def player_policy(robot):
    # demo: beacon seeker. robot.see("beacon")[0].cx is bearing (0.5 = centered)
    beacon = robot.see("beacon")
    scan = robot.lidar()
    ahead = min(scan[0:3] + scan[-3:]) if scan else 8
    if ahead < 1.0:
        robot.move(turn=1.2)           # dodge the pillar
    elif beacon:
        robot.move(forward=0.9, turn=-1.5 * (beacon[0].cx - 0.5))
    else:
        robot.move(forward=0.5, turn=0.6)   # search pattern
`,
  },
];

/* ---------- renderer / scene shell ---------- */
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
let camMode = 0;
const CAM_MODES = ["chase", "orbit", "top"];
const chaseCam = new THREE.PerspectiveCamera(60, 16 / 9, 0.1, 100);
const orbitCam = new THREE.PerspectiveCamera(60, 16 / 9, 0.1, 100);
const CAM_POOL = { pov: povCam, top: topCam, chase: chaseCam, orbit: orbitCam };
addEventListener("resize", () => {
  renderer.setSize(innerWidth, innerHeight);
  specCam.aspect = innerWidth / innerHeight;
  specCam.updateProjectionMatrix();
});

/* ---------- level building (rebuildable) ---------- */
const DOOR_COLORS = { red: 0xd84a4a, blue: 0x4a7ad8, green: 0x44b86a };
const W = 0.15;
let LV = null;                 // active level def
let levelGroup = null;         // all level meshes, swapped on switch
let wallMeshes = [], doorObjs = [], goalMesh = null;

function buildLevel(def) {
  if (levelGroup) {
    scene.remove(levelGroup);
    levelGroup.traverse((o) => { o.geometry?.dispose(); o.material?.dispose?.(); });
  }
  LV = def;
  levelGroup = new THREE.Group();
  wallMeshes = []; doorObjs = []; goalMesh = null;

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
  for (const d of def.doors || []) {
    const frame = new THREE.Mesh(new THREE.TorusGeometry(0.75, 0.09, 8, 24, Math.PI),
      new THREE.MeshStandardMaterial({ color: DOOR_COLORS[d.color],
        emissive: DOOR_COLORS[d.color], emissiveIntensity: 0.35 }));
    frame.position.set(d.x, 0.05, d.z);
    frame.castShadow = true;
    levelGroup.add(frame);
    doorObjs.push({ ...d, pos: new THREE.Vector3(d.x, 0.8, d.z), seen: false });
  }
  if (def.goal) {
    goalMesh = new THREE.Mesh(new THREE.CylinderGeometry(def.goal.r, def.goal.r, 0.05, 32),
      new THREE.MeshStandardMaterial({ color: 0x44b86a, emissive: 0x44b86a,
        emissiveIntensity: 0.5, transparent: true, opacity: 0.55 }));
    goalMesh.position.set(def.goal.x, 0.03, def.goal.z);
    levelGroup.add(goalMesh);
  }
  scene.add(levelGroup);
}

/* ---------- the dog ---------- */
const BODY_LEN = 0.62, BODY_W = 0.3, STAND_H = 0.42;
const HIPS = [
  { x: BODY_LEN / 2 - 0.06, z: -BODY_W / 2 }, { x: BODY_LEN / 2 - 0.06, z: BODY_W / 2 },
  { x: -BODY_LEN / 2 + 0.06, z: -BODY_W / 2 }, { x: -BODY_LEN / 2 + 0.06, z: BODY_W / 2 },
];
const TROT_PAIRS = [0, 1, 1, 0];
const L1 = 0.26, L2 = 0.26;
const dog = { pos: new THREE.Vector3(), heading: 0, group: new THREE.Group(),
              legs: [], phase: 0 };
{
  const bodyMat = new THREE.MeshStandardMaterial({ color: 0xc8cdd4, roughness: 0.5, metalness: 0.35 });
  const darkMat = new THREE.MeshStandardMaterial({ color: 0x23282e, roughness: 0.6 });
  const body = new THREE.Mesh(new THREE.BoxGeometry(BODY_LEN, 0.18, BODY_W), bodyMat);
  body.castShadow = true;
  dog.group.add(body);
  const head = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.14, 0.18), darkMat);
  head.position.set(BODY_LEN / 2 + 0.1, 0.06, 0);
  head.castShadow = true;
  dog.group.add(head);
  for (const hip of HIPS) {
    const thigh = new THREE.Mesh(new THREE.BoxGeometry(0.07, L1, 0.05), darkMat);
    const shin = new THREE.Mesh(new THREE.BoxGeometry(0.05, L2, 0.04), bodyMat);
    thigh.castShadow = shin.castShadow = true;
    thigh.geometry.translate(0, -L1 / 2, 0);
    shin.geometry.translate(0, -L2 / 2, 0);
    const hipPivot = new THREE.Group();
    hipPivot.position.set(hip.x, -0.06, hip.z);
    const kneePivot = new THREE.Group();
    kneePivot.position.set(0, -L1, 0);
    hipPivot.add(thigh); hipPivot.add(kneePivot); kneePivot.add(shin);
    dog.group.add(hipPivot);
    dog.legs.push({ hip, hipPivot, kneePivot, foot: new THREE.Vector3(),
                    swing: 0, from: new THREE.Vector3(), to: new THREE.Vector3() });
  }
  scene.add(dog.group);
}
function bodyToWorld(bx, bz, out) {
  const c = Math.cos(dog.heading), s = Math.sin(dog.heading);
  return out.set(dog.pos.x + bx * c + bz * s, 0, dog.pos.z - bx * s + bz * c);
}
function worldToBodyX(wx, wz) {
  const c = Math.cos(dog.heading), s = Math.sin(dog.heading);
  return (wx - dog.pos.x) * c - (wz - dog.pos.z) * s;
}
function homeFoot(leg, out) { return bodyToWorld(leg.hip.x, leg.hip.z, out); }

const STEP_TIME = 0.28, STEP_H = 0.09;
function updateDog(dt, cmd) {
  const f = THREE.MathUtils.clamp(cmd.forward, -1, 1);
  const st = THREE.MathUtils.clamp(cmd.strafe, -1, 1);
  const yaw = THREE.MathUtils.clamp(cmd.turn, -1.5, 1.5);
  dog.heading += yaw * dt;
  const c = Math.cos(dog.heading), s = Math.sin(dog.heading);
  const dx = (f * c + st * s) * dt, dz = (-f * s + st * c) * dt;
  const next = dog.pos.clone(); next.x += dx; next.z += dz;
  const r = 0.32;
  for (const m of wallMeshes) {
    const b = m.userData.aabb;
    if (next.x > b.min.x - r && next.x < b.max.x + r &&
        next.z > b.min.z - r && next.z < b.max.z + r) {
      const keepX = dog.pos.clone(); keepX.x = next.x;
      const keepZ = dog.pos.clone(); keepZ.z = next.z;
      const okX = !(keepX.x > b.min.x - r && keepX.x < b.max.x + r &&
                    keepX.z > b.min.z - r && keepX.z < b.max.z + r);
      const okZ = !(keepZ.x > b.min.x - r && keepZ.x < b.max.x + r &&
                    keepZ.z > b.min.z - r && keepZ.z < b.max.z + r);
      next.copy(okX ? keepX : okZ ? keepZ : dog.pos);
    }
  }
  dog.pos.copy(next);

  const speed = Math.hypot(dx, dz) / dt + Math.abs(yaw) * 0.4;
  const moving = speed > 0.02;
  if (moving) dog.phase = (dog.phase + dt / (STEP_TIME * 2)) % 1;

  for (let i = 0; i < 4; i++) {
    const leg = dog.legs[i];
    const legPhase = (dog.phase + TROT_PAIRS[i] * 0.5) % 1;
    const inSwing = moving && legPhase < 0.5;
    if (inSwing) {
      if (leg.swing === 0) {
        leg.from.copy(leg.foot);
        homeFoot(leg, leg.to);
        leg.to.x += dx / dt * STEP_TIME * 1.6;
        leg.to.z += dz / dt * STEP_TIME * 1.6;
      }
      leg.swing = legPhase / 0.5;
      leg.foot.lerpVectors(leg.from, leg.to, leg.swing);
      leg.foot.y = Math.sin(leg.swing * Math.PI) * STEP_H;
    } else {
      leg.swing = 0;
      leg.foot.y = 0;
    }
  }

  const bob = moving ? Math.sin(dog.phase * Math.PI * 4) * 0.012 : 0;
  dog.group.position.set(dog.pos.x, STAND_H + bob, dog.pos.z);
  dog.group.rotation.y = dog.heading;

  for (const leg of dog.legs) {
    const dxp = worldToBodyX(leg.foot.x, leg.foot.z) - leg.hip.x;
    const dyp = (STAND_H + bob - 0.06) - leg.foot.y;
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

/* ---------- robot senses ---------- */
const raycaster = new THREE.Raycaster();
const LIDAR_RAYS = 36, LIDAR_RANGE = 8;
function eyePos() { return new THREE.Vector3(dog.pos.x, 0.45, dog.pos.z); }
function fwdVec() { return new THREE.Vector3(Math.cos(dog.heading), 0, -Math.sin(dog.heading)); }

function senseDetections() {
  const out = [];
  const eye = eyePos(), fwd = fwdVec();
  for (const d of doorObjs) {
    const to = d.pos.clone().sub(eye);
    const dist = to.length();
    if (dist > 7) continue;
    const bearing = Math.atan2(fwd.x * to.z - fwd.z * to.x, fwd.x * to.x + fwd.z * to.z);
    if (Math.abs(bearing) > 0.62) continue;
    raycaster.set(eye, to.normalize());
    const hit = raycaster.intersectObjects(wallMeshes, false)[0];
    if (hit && hit.distance < dist - 0.4) continue;
    const cx = 640 - (bearing / 0.62) * 600;
    const size = Math.min(420, 2200 / dist);
    out.push({ label: `${d.color} door`, confidence: 0.95,
               bbox: [cx - size / 4, 360 - size / 2, cx + size / 4, 360 + size / 2],
               door_id: d.id, distance: +dist.toFixed(2) });
    if (!d.seen) { d.seen = true; postEvent("detection", `sighted: ${d.color} door`, { door: d.id }); }
  }
  if (LV.goal) {
    const gp = new THREE.Vector3(LV.goal.x, 0.4, LV.goal.z);
    const to = gp.clone().sub(eye);
    const dist = to.length();
    const bearing = Math.atan2(fwd.x * to.z - fwd.z * to.x, fwd.x * to.x + fwd.z * to.z);
    if (dist < 12 && Math.abs(bearing) <= 0.62) {
      raycaster.set(eye, to.normalize());
      const hit = raycaster.intersectObjects(wallMeshes, false)[0];
      if (!hit || hit.distance > dist - 0.4) {
        const cx = 640 - (bearing / 0.62) * 600;
        const size = Math.min(420, 2600 / dist);
        out.push({ label: "beacon", confidence: 0.98,
                   bbox: [cx - size / 3, 360 - size / 2, cx + size / 3, 360 + size / 2],
                   distance: +dist.toFixed(2) });
      }
    }
  }
  raycaster.set(eyePos(), fwd);
  const ahead = raycaster.intersectObjects(wallMeshes, false)[0];
  if (ahead && ahead.distance < 1.6) {
    const size = 700 / ahead.distance;
    out.push({ label: "obstacle", confidence: 1.0,
               bbox: [640 - size / 2, 360 - size / 2, 640 + size / 2, 360 + size / 2],
               distance: +ahead.distance.toFixed(2) });
  }
  return out;
}

function senseLidar() {
  const eye = eyePos();
  const ranges = [];
  for (let i = 0; i < LIDAR_RAYS; i++) {
    const a = dog.heading + (i / LIDAR_RAYS) * Math.PI * 2;
    raycaster.set(eye, new THREE.Vector3(Math.cos(a), 0, -Math.sin(a)));
    raycaster.far = LIDAR_RANGE;
    const hit = raycaster.intersectObjects(wallMeshes, false)[0];
    ranges.push(hit ? +hit.distance.toFixed(2) : LIDAR_RANGE);
  }
  raycaster.far = Infinity;
  return ranges;
}

/* ---------- lidar point cloud (the world as the robot has sensed it) ---------- */
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
function cloudAdd(x, y, z) {
  const i = cloudHead * 3;
  cloudPos[i] = x; cloudPos[i + 1] = y; cloudPos[i + 2] = z;
  _tmpColor.setHSL(0.66 - (y / 1.7) * 0.55, 0.9, 0.45 + (y / 1.7) * 0.2);
  cloudCol[i] = _tmpColor.r; cloudCol[i + 1] = _tmpColor.g; cloudCol[i + 2] = _tmpColor.b;
  cloudHead = (cloudHead + 1) % CLOUD_MAX;
  cloudCount = Math.min(cloudCount + 1, CLOUD_MAX);
}
function cloudCommit() {
  cloudGeo.attributes.position.needsUpdate = true;
  cloudGeo.attributes.color.needsUpdate = true;
  cloudGeo.setDrawRange(0, cloudCount);
}
function cloudReset() {
  cloudCount = 0; cloudHead = 0;
  cloudGeo.setDrawRange(0, 0);
}

/* ---------- robot-built map ---------- */
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
    const a = dog.heading + (i / ranges.length) * Math.PI * 2;
    const dx = Math.cos(a), dz = -Math.sin(a);
    for (let r = 0.2; r < ranges[i]; r += CELL * 0.8) {
      const [cx, cz] = cellOf(dog.pos.x + dx * r, dog.pos.z + dz * r);
      if (cx >= 0 && cx < GRID && cz >= 0 && cz < GRID && occ[cz * GRID + cx] !== 2)
        occ[cz * GRID + cx] = 1;
    }
    if (ranges[i] < LIDAR_RANGE) {
      const hx = dog.pos.x + dx * ranges[i], hz = dog.pos.z + dz * ranges[i];
      const [cx, cz] = cellOf(hx, hz);
      if (cx >= 0 && cx < GRID && cz >= 0 && cz < GRID) occ[cz * GRID + cx] = 2;
      for (let k = 0; k < 5; k++)   // vertical spread up the wall + noise
        cloudAdd(hx + (Math.random() - 0.5) * 0.05,
                 Math.random() * 1.55,
                 hz + (Math.random() - 0.5) * 0.05);
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
  const [cx, cz] = cellOf(dog.pos.x, dog.pos.z);
  mapCtx.fillStyle = "#00d47e";
  mapCtx.fillRect(cx - 1, cz - 1, 3, 3);
  mapCtx.fillRect(Math.round(cx + Math.cos(dog.heading) * 3),
                  Math.round(cz - Math.sin(dog.heading) * 3), 1, 1);
}

/* ---------- chamber state, switching, win ---------- */
const briefTitle = document.getElementById("briefTitle");
const briefText = document.getElementById("briefText");
const roomsEl = document.getElementById("rooms");
const levelSel = document.getElementById("levelSel");
for (let i = 0; i < LEVELS.length; i++) {
  const o = document.createElement("option");
  o.value = i; o.textContent = LEVELS[i].title;
  levelSel.appendChild(o);
}
let visited = new Set(), won = false, t0 = performance.now(), goalHeld = 0;
let usedManual = false;
let levelIndex = 0;

function loadLevel(i) {
  levelIndex = ((i % LEVELS.length) + LEVELS.length) % LEVELS.length;
  buildLevel(LEVELS[levelIndex]);
  levelSel.value = levelIndex;
  briefTitle.textContent = LV.title;
  briefText.textContent = LV.brief;
  dog.pos.set(LV.spawn.x, 0, LV.spawn.z);
  dog.heading = LV.spawn.heading;
  for (const leg of dog.legs) homeFoot(leg, leg.foot);
  visited = new Set(); won = false; goalHeld = 0; t0 = performance.now();
  usedManual = false;
  occ = new Uint8Array(GRID * GRID);
  CELL = (LV.bounds * 2) / GRID;
  cloudReset();
  odo = 0;
  setCode(LV.demo || "");
  policyStatus("demo policy loaded — press RUN, or rewrite it", "");
  document.getElementById("win").classList.remove("show");
  roomsEl.innerHTML = "";
  for (const r of LV.rooms || []) {
    const chip = document.createElement("span");
    chip.className = "room-chip"; chip.id = `room-${r.id}`; chip.textContent = r.id;
    roomsEl.appendChild(chip);
  }
  if (LV.goal) {
    const chip = document.createElement("span");
    chip.className = "room-chip"; chip.id = "goal-chip"; chip.textContent = "reach the beacon";
    roomsEl.appendChild(chip);
  }
  postEvent("arena", `level loaded: ${LV.name}`, {});
}
levelSel.addEventListener("change", () => loadLevel(+levelSel.value));

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

function winChamber(detail) {
  won = true;
  const secs = ((performance.now() - t0) / 1000).toFixed(1);
  const mode = usedManual ? "manual" : "autonomous";
  document.getElementById("winTitle").textContent =
    usedManual ? "PRACTICE RUN — MANUAL DRIVE" : "CHAMBER COMPLETE — AUTONOMOUS";
  document.getElementById("winDetail").textContent = `${detail} · ${secs}s` +
    (usedManual ? " · write a policy to make it count" : "");
  document.getElementById("win").classList.add("show");
  document.querySelector("#win .card").classList.toggle("practice", usedManual);
  postEvent("task", `${LV.title} ${usedManual ? "cleared MANUALLY (practice)" : "COMPLETE — autonomous"} · ${secs}s`,
            { time_s: +secs, level: LV.name, mode });
  sealAttempt();
}

function tickChamber(dt) {
  if (won) return;
  for (const r of LV.rooms || []) {
    const [x1, z1, x2, z2] = r.rect;
    if (!visited.has(r.id) && dog.pos.x > x1 && dog.pos.x < x2 && dog.pos.z > z1 && dog.pos.z < z2) {
      visited.add(r.id);
      document.getElementById(`room-${r.id}`)?.classList.add("seen");
      postEvent("arena", `room explored: ${r.id}`, { rooms: visited.size });
    }
  }
  if ((LV.rooms || []).length && visited.size === LV.rooms.length) {
    const seen = doorObjs.filter(d => d.seen);
    winChamber(`all ${LV.rooms.length} rooms · doors sighted ${seen.length}/${doorObjs.length}`);
  }
  if (LV.goal) {
    const inGoal = Math.hypot(dog.pos.x - LV.goal.x, dog.pos.z - LV.goal.z) < LV.goal.r;
    goalHeld = inGoal ? goalHeld + dt : 0;
    const chip = document.getElementById("goal-chip");
    if (chip) {
      chip.textContent = inGoal ? `holding… ${Math.max(0, LV.goal.hold - goalHeld).toFixed(1)}s` : "reach the beacon";
      chip.classList.toggle("seen", inGoal);
    }
    if (goalHeld >= LV.goal.hold) winChamber("beacon held");
  }
}

/* ---------- the wire ---------- */
async function api(path, body) {
  const r = await fetch(path, { method: "POST",
    headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) });
  return r.json();
}
let serverCmd = { forward: 0, strafe: 0, turn: 0 };
let linked = false, lastLidar = [];
async function pollCmd() {
  try {
    const r = await (await fetch("/api/arena/cmd")).json();
    serverCmd = r.cmd;
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
        pose: { x: +dog.pos.x.toFixed(2), z: +dog.pos.z.toFixed(2),
                heading: +dog.heading.toFixed(3) },
        level: { name: LV.name, room: currentRoom(), rooms_visited: [...visited],
                 odometer_m: +odo.toFixed(1), won },
      });
    } catch {}
  }
  setTimeout(pushState, 100);
}
function postEvent(type, title, detail) {
  if (!linked) return;
  api("/api/arena/event", { type, title, detail }).catch(() => {});
}

/* ---------- the policy editor: this is how you play ---------- */
const codeEl = document.getElementById("code");
const statusEl = document.getElementById("policyStatus");
function policyStatus(msg, cls) { statusEl.textContent = msg; statusEl.className = cls; }

let cm = null;   // CodeMirror view when the CDN loads; textarea otherwise
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
  e.stopPropagation();          // typing must not drive the dog
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
document.getElementById("btnRun").addEventListener("click", async () => {
  policyStatus("saving…", "");
  try {
    const w = await api("/api/behaviors/write", { name: "player_policy", source: getCode() });
    if (!w.ok) { policyStatus(w.error, "err"); return; }
    await api("/api/behaviors/enable", { name: "player_policy" });
    policyStatus("running — hot reload applies edits on every RUN", "ok");
  } catch { policyStatus("no server — run `roborun` first", "err"); }
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

document.getElementById("btnStop").addEventListener("click", async () => {
  try {
    await api("/api/behaviors/disable", { name: "player_policy" });
    policyStatus("stopped", "");
  } catch {}
});

/* ---------- telemetry: room, odometer, pose ---------- */
let odo = 0;
const prevPos = new THREE.Vector3();
function currentRoom() {
  for (const r of LV.rooms || []) {
    const [x1, z1, x2, z2] = r.rect;
    if (dog.pos.x > x1 && dog.pos.x < x2 && dog.pos.z > z1 && dog.pos.z < z2) return r.id;
  }
  return LV.goal ? "field" : "corridor";
}
function updateTelemetry() {
  document.getElementById("teleRoom").textContent = `room ${currentRoom()}`;
  document.getElementById("teleOdo").textContent = `odometer ${odo.toFixed(1)} m`;
  document.getElementById("telePose").textContent =
    `x ${dog.pos.x.toFixed(1)} · z ${dog.pos.z.toFixed(1)} · θ ${dog.heading.toFixed(2)}`;
}

/* ---------- input ---------- */
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
  if (f || t) { usedManual = true; return { forward: f * 0.9, strafe: 0, turn: t * 1.2 }; }
  return null;
}

/* ---------- panel system: drag, resize, toggle, persist ---------- */
const LAYOUT_KEY = "arena-layout-v2";
const PANEL_IDS = ["p-brief", "p-policy", "p-status", "p-map", "p-view1", "p-view2"];
let zTop = 100;

function defaultLayout() {
  const w = innerWidth, h = innerHeight;
  return {
    "p-brief":  { l: 14, t: 52, w: 360, h: 170, hidden: false },
    "p-policy": { l: 14, t: 232, w: 470, h: Math.min(430, h - 250), hidden: false },
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

/* ---------- multi-view render: viewports are DOM rects ---------- */
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
  }
  renderer.setScissorTest(false);
}

/* ---------- loop ---------- */
const clockEl = document.getElementById("clock"), cmdEl = document.getElementById("cmdline");
let last = performance.now(), senseTick = 0, orbitAngle = 0;
function frame(now) {
  const dt = Math.min((now - last) / 1000, 0.05);
  last = now;
  const cmd = keyboardCmd() || serverCmd;
  updateDog(dt, cmd);
  tickChamber(dt);

  odo += prevPos.distanceTo(dog.pos);
  prevPos.copy(dog.pos);
  senseTick += dt;
  if (senseTick > 0.12) {
    senseTick = 0;
    lastLidar = senseLidar();
    integrateLidar(lastLidar);
    cloudCommit();
    drawMap();
    updateTelemetry();
  }

  if (camMode === 0) {
    specCam.position.lerp(new THREE.Vector3(
      dog.pos.x - Math.cos(dog.heading) * 3.4, 2.4,
      dog.pos.z + Math.sin(dog.heading) * 3.4), 0.06);
    specCam.lookAt(dog.pos.x, 0.5, dog.pos.z);
  } else if (camMode === 1) {
    specCam.position.copy(orbitCam.position);
    specCam.quaternion.copy(orbitCam.quaternion);
  }
  const fwd = fwdVec();
  povCam.position.set(dog.pos.x + fwd.x * 0.35, 0.45, dog.pos.z + fwd.z * 0.35);
  povCam.lookAt(dog.pos.x + fwd.x * 5, 0.4, dog.pos.z + fwd.z * 5);
  chaseCam.position.lerp(new THREE.Vector3(
    dog.pos.x - Math.cos(dog.heading) * 3.4, 2.4,
    dog.pos.z + Math.sin(dog.heading) * 3.4), 0.08);
  chaseCam.lookAt(dog.pos.x, 0.5, dog.pos.z);
  orbitCam.position.set(dog.pos.x + Math.cos(orbitAngle) * 6, 4.2,
                        dog.pos.z + Math.sin(orbitAngle) * 6);
  orbitCam.lookAt(dog.pos.x, 0.4, dog.pos.z);
  orbitAngle += dt * 0.25;

  if (!won) clockEl.textContent = `${((now - t0) / 1000).toFixed(1)}s`;
  cmdEl.textContent = `cmd f=${cmd.forward.toFixed(2)} t=${cmd.turn.toFixed(2)} · cam ${CAM_MODES[camMode]}`;
  renderViews();
  requestAnimationFrame(frame);
}

loadLevel(0);
pollCmd(); pushState();
requestAnimationFrame(frame);
