/* RoboRun Arena — Chamber 01 (recon).
   Self-contained browser sim: the body is a procedural-gait quadruped
   (planted feet, no sliding); the robot's senses are a forward camera
   cone + 360° lidar, nothing else. The spectator gets more (C cycles
   chase/orbit/top), but robot.see() only ever gets the cone, and the
   ROBOT MAP panel shows only what the lidar has actually swept.
   Behaviors drive it via GET /api/arena/cmd; every attempt records into
   the black box and the win card shows the sealed run-hash. */

import * as THREE from "three";

/* ---------- level ---------- */
const W = 0.15;
const LEVEL = {
  name: "chamber-01",
  bounds: { x: 16, z: 16 },
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
};

/* ---------- scene ---------- */
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0e11);
scene.fog = new THREE.Fog(0x0b0e11, 18, 34);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.autoClear = false;
document.body.appendChild(renderer.domElement);

scene.add(new THREE.HemisphereLight(0xcfe8ff, 0x223038, 0.85));
const sun = new THREE.DirectionalLight(0xffffff, 1.4);
sun.position.set(8, 14, 6);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.camera.left = sun.shadow.camera.bottom = -18;
sun.shadow.camera.right = sun.shadow.camera.top = 18;
scene.add(sun);

const floor = new THREE.Mesh(
  new THREE.PlaneGeometry(LEVEL.bounds.x * 2, LEVEL.bounds.z * 2),
  new THREE.MeshStandardMaterial({ color: 0x18202a, roughness: 0.92 }));
floor.rotation.x = -Math.PI / 2;
floor.receiveShadow = true;
scene.add(floor);
const grid = new THREE.GridHelper(LEVEL.bounds.x * 2, LEVEL.bounds.x * 2, 0x24303c, 0x1b2530);
grid.position.y = 0.002;
scene.add(grid);

const wallMat = new THREE.MeshStandardMaterial({ color: 0x2e3c4a, roughness: 0.8 });
const wallMeshes = [];
for (const [x1, z1, x2, z2] of LEVEL.walls) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(
    Math.abs(x2 - x1) || W * 2, 1.6, Math.abs(z2 - z1) || W * 2), wallMat);
  m.position.set((x1 + x2) / 2, 0.8, (z1 + z2) / 2);
  m.castShadow = m.receiveShadow = true;
  m.userData.aabb = new THREE.Box3().setFromObject(m);
  scene.add(m);
  wallMeshes.push(m);
}

const DOOR_COLORS = { red: 0xd84a4a, blue: 0x4a7ad8, green: 0x44b86a };
const doorObjs = [];
for (const d of LEVEL.doors) {
  const frame = new THREE.Mesh(new THREE.TorusGeometry(0.75, 0.09, 8, 24, Math.PI),
    new THREE.MeshStandardMaterial({ color: DOOR_COLORS[d.color],
      emissive: DOOR_COLORS[d.color], emissiveIntensity: 0.35 }));
  frame.position.set(d.x, 0.05, d.z);
  frame.castShadow = true;
  scene.add(frame);
  doorObjs.push({ ...d, pos: new THREE.Vector3(d.x, 0.8, d.z), seen: false });
}

/* ---------- cameras: spectator (3 modes) + robot POV + top inset ---------- */
const specCam = new THREE.PerspectiveCamera(60, innerWidth / innerHeight, 0.1, 100);
const povCam = new THREE.PerspectiveCamera(70, 16 / 9, 0.08, 60);
const topCam = new THREE.OrthographicCamera(-9, 9, 9, -9, 1, 50);
topCam.position.set(0, 30, 0);
topCam.lookAt(0, 0, 0);
let camMode = 0;                        // 0 chase · 1 orbit · 2 top
const CAM_MODES = ["chase", "orbit", "top"];
addEventListener("resize", () => {
  renderer.setSize(innerWidth, innerHeight);
  specCam.aspect = innerWidth / innerHeight;
  specCam.updateProjectionMatrix();
});

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
for (const leg of dog.legs) homeFoot(leg, leg.foot);

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
      leg.foot.y = 0;        // stance: planted — no sliding
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

/* ---------- robot senses: forward camera cone + 360° lidar ---------- */
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
  raycaster.set(eye, fwd);
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

/* ---------- the map the robot builds (fog of war from lidar) ---------- */
const GRID = 96, CELL = (LEVEL.bounds.x * 2) / GRID;   // 32 m / 96 cells
const occ = new Uint8Array(GRID * GRID);               // 0 unknown · 1 free · 2 wall
const mapCanvas = document.getElementById("map");
const mapCtx = mapCanvas.getContext("2d");
function cellOf(x, z) {
  return [Math.floor((x + LEVEL.bounds.x) / CELL), Math.floor((z + LEVEL.bounds.z) / CELL)];
}
function integrateLidar(ranges) {
  const eye = dog.pos;
  for (let i = 0; i < ranges.length; i++) {
    const a = dog.heading + (i / ranges.length) * Math.PI * 2;
    const dx = Math.cos(a), dz = -Math.sin(a);
    for (let r = 0.2; r < ranges[i]; r += CELL * 0.8) {
      const [cx, cz] = cellOf(eye.x + dx * r, eye.z + dz * r);
      if (cx >= 0 && cx < GRID && cz >= 0 && cz < GRID && occ[cz * GRID + cx] !== 2)
        occ[cz * GRID + cx] = 1;
    }
    if (ranges[i] < LIDAR_RANGE) {
      const [cx, cz] = cellOf(eye.x + dx * ranges[i], eye.z + dz * ranges[i]);
      if (cx >= 0 && cx < GRID && cz >= 0 && cz < GRID) occ[cz * GRID + cx] = 2;
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
  const [cx, cz] = cellOf(dog.pos.x, dog.pos.z);   // the dog, as a green dot + nose
  mapCtx.fillStyle = "#00d47e";
  mapCtx.fillRect(cx - 1, cz - 1, 3, 3);
  mapCtx.fillRect(Math.round(cx + Math.cos(dog.heading) * 3),
                  Math.round(cz - Math.sin(dog.heading) * 3), 1, 1);
}

/* ---------- chamber logic + auto-recorded attempts ---------- */
const roomsEl = document.getElementById("rooms");
const visited = new Set();
for (const r of LEVEL.rooms) {
  const chip = document.createElement("span");
  chip.className = "room-chip"; chip.id = `room-${r.id}`; chip.textContent = r.id;
  roomsEl.appendChild(chip);
}
let won = false, t0 = performance.now(), recording = false;

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
      return;
    }
    hashEl.innerHTML = `<span class="k">run-hash</span>unrecorded — deck wasn't running`;
  } catch {
    hashEl.innerHTML = `<span class="k">run-hash</span>unavailable`;
  }
}

function tickChamber() {
  for (const r of LEVEL.rooms) {
    const [x1, z1, x2, z2] = r.rect;
    if (!visited.has(r.id) && dog.pos.x > x1 && dog.pos.x < x2 && dog.pos.z > z1 && dog.pos.z < z2) {
      visited.add(r.id);
      document.getElementById(`room-${r.id}`).classList.add("seen");
      postEvent("arena", `room explored: ${r.id}`, { rooms: visited.size });
    }
  }
  if (!won && visited.size === LEVEL.rooms.length) {
    won = true;
    const secs = ((performance.now() - t0) / 1000).toFixed(1);
    const seen = doorObjs.filter(d => d.seen);
    document.getElementById("winDetail").textContent =
      `all ${LEVEL.rooms.length} rooms · ${secs}s · doors sighted ${seen.length}/${doorObjs.length}`;
    document.getElementById("win").classList.add("show");
    postEvent("task", `CHAMBER 01 COMPLETE · ${secs}s`,
              { time_s: +secs, doors_seen: seen.map(d => d.id) });
    sealAttempt();
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
        level: { name: LEVEL.name, rooms_visited: [...visited], won },
      });
    } catch {}
  }
  setTimeout(pushState, 100);
}
function postEvent(type, title, detail) {
  if (!linked) return;
  api("/api/arena/event", { type, title, detail }).catch(() => {});
}
pollCmd(); pushState();

/* WASD + camera key */
const keys = {};
addEventListener("keydown", (e) => {
  keys[e.key.toLowerCase()] = true;
  if (e.key.toLowerCase() === "c") camMode = (camMode + 1) % CAM_MODES.length;
});
addEventListener("keyup", (e) => keys[e.key.toLowerCase()] = false);
function keyboardCmd() {
  const f = (keys.w ? 1 : 0) - (keys.s ? 1 : 0);
  const t = (keys.a ? 1 : 0) - (keys.d ? 1 : 0);
  if (f || t) return { forward: f * 0.9, strafe: 0, turn: t * 1.2 };
  return null;
}

/* ---------- multi-view render ---------- */
function placeLabels(povRect, topRect) {
  const pl = document.getElementById("povlbl");
  pl.style.left = `${povRect.x + 6}px`;
  pl.style.top = `${innerHeight - povRect.y - povRect.h + 4}px`;
  const tl = document.getElementById("toplbl");
  tl.style.left = `${topRect.x + 6}px`;
  tl.style.top = `${innerHeight - topRect.y - topRect.h + 4}px`;
}
function renderViews() {
  const w = innerWidth, h = innerHeight;
  renderer.setScissorTest(true);
  // main spectator view
  renderer.setViewport(0, 0, w, h);
  renderer.setScissor(0, 0, w, h);
  renderer.clear();
  renderer.render(scene, camMode === 2 ? topCam : specCam);
  // robot POV inset (bottom-right) — exactly what the robot's camera sees
  const pw = Math.round(Math.min(w * 0.24, 360)), ph = Math.round(pw * 9 / 16);
  const povRect = { x: w - pw - 14, y: 14, w: pw, h: ph };
  renderer.setViewport(povRect.x, povRect.y, pw, ph);
  renderer.setScissor(povRect.x, povRect.y, pw, ph);
  renderer.clearDepth();
  renderer.clear(true, true, false);
  renderer.render(scene, povCam);
  // top view inset (left of POV) unless top is the main view
  const topRect = { x: w - pw * 2 - 24, y: 14, w: pw, h: ph };
  if (camMode !== 2) {
    renderer.setViewport(topRect.x, topRect.y, pw, ph);
    renderer.setScissor(topRect.x, topRect.y, pw, ph);
    renderer.clear(true, true, false);
    renderer.render(scene, topCam);
  }
  renderer.setScissorTest(false);
  placeLabels(povRect, camMode === 2 ? povRect : topRect);
}

/* ---------- loop ---------- */
const clockEl = document.getElementById("clock"), cmdEl = document.getElementById("cmdline");
let last = performance.now(), senseTick = 0, orbitAngle = 0;
function frame(now) {
  const dt = Math.min((now - last) / 1000, 0.05);
  last = now;
  const cmd = keyboardCmd() || serverCmd;
  updateDog(dt, cmd);
  tickChamber();

  senseTick += dt;
  if (senseTick > 0.12) {            // ~8 Hz lidar + map, like a real scanner
    senseTick = 0;
    lastLidar = senseLidar();
    integrateLidar(lastLidar);
    drawMap();
  }

  // spectator camera
  if (camMode === 0) {
    specCam.position.lerp(new THREE.Vector3(
      dog.pos.x - Math.cos(dog.heading) * 3.4, 2.4,
      dog.pos.z + Math.sin(dog.heading) * 3.4), 0.06);
    specCam.lookAt(dog.pos.x, 0.5, dog.pos.z);
  } else if (camMode === 1) {
    orbitAngle += dt * 0.25;
    specCam.position.set(dog.pos.x + Math.cos(orbitAngle) * 6, 4.2,
                         dog.pos.z + Math.sin(orbitAngle) * 6);
    specCam.lookAt(dog.pos.x, 0.4, dog.pos.z);
  }
  // robot POV: head height, looking where the robot looks — nothing more
  const fwd = fwdVec();
  povCam.position.set(dog.pos.x + fwd.x * 0.35, 0.45, dog.pos.z + fwd.z * 0.35);
  povCam.lookAt(dog.pos.x + fwd.x * 5, 0.4, dog.pos.z + fwd.z * 5);

  if (!won) clockEl.textContent = `${((now - t0) / 1000).toFixed(1)}s`;
  cmdEl.textContent = `cmd f=${cmd.forward.toFixed(2)} t=${cmd.turn.toFixed(2)} · cam ${CAM_MODES[camMode]}`;
  renderViews();
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
