/* RoboRun Arena — Chamber 01 (recon).
   The sim is self-contained in the browser: its own body (procedural-gait
   quadruped, planted feet, no sliding) and its own eyes (view-cone +
   raycast detections published to the server, where robot.see() reads
   them). Behaviors drive it through GET /api/arena/cmd; WASD works too. */

import * as THREE from "three";

/* ---------- level ---------- */
// One building, four rooms around a cross corridor. Doors are colored —
// recon questions get asked about them later.
const W = 0.15;            // wall half-thickness
const LEVEL = {
  name: "chamber-01",
  bounds: { x: 16, z: 16 },
  rooms: [
    { id: "north-west", rect: [-8, -8, -1, -1] },
    { id: "north-east", rect: [1, -8, 8, -1] },
    { id: "south-west", rect: [-8, 1, -1, 8] },
    { id: "south-east", rect: [1, 1, 8, 8] },
  ],
  // walls as [x1, z1, x2, z2] segments (axis-aligned)
  walls: [
    [-8, -8, 8, -8], [-8, 8, 8, 8], [-8, -8, -8, 8], [8, -8, 8, 8],  // outer
    [-8, -1, -3.2, -1], [-1.8, -1, 1.8, -1], [3.2, -1, 8, -1],        // north corridor wall (gaps = doorways)
    [-8, 1, -3.2, 1], [-1.8, 1, 1.8, 1], [3.2, 1, 8, 1],              // south corridor wall
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
const camera = new THREE.PerspectiveCamera(60, innerWidth / innerHeight, 0.1, 100);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
document.body.appendChild(renderer.domElement);
addEventListener("resize", () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

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
  const len = Math.hypot(x2 - x1, z2 - z1);
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
    new THREE.MeshStandardMaterial({ color: DOOR_COLORS[d.color], emissive: DOOR_COLORS[d.color], emissiveIntensity: 0.35 }));
  frame.position.set(d.x, 0.05, d.z);
  frame.castShadow = true;
  scene.add(frame);
  doorObjs.push({ ...d, pos: new THREE.Vector3(d.x, 0.8, d.z), seen: false });
}

/* ---------- the dog: procedural trot, feet plant in the world ---------- */
const BODY_LEN = 0.62, BODY_W = 0.3, STAND_H = 0.42;
const HIPS = [  // x fore/aft, z left/right (body frame)
  { x: BODY_LEN / 2 - 0.06, z: -BODY_W / 2 }, { x: BODY_LEN / 2 - 0.06, z: BODY_W / 2 },
  { x: -BODY_LEN / 2 + 0.06, z: -BODY_W / 2 }, { x: -BODY_LEN / 2 + 0.06, z: BODY_W / 2 },
];
const TROT_PAIRS = [0, 1, 1, 0];      // FL+RR phase 0 · FR+RL phase 0.5
const L1 = 0.26, L2 = 0.26;           // thigh, shin

const dog = {
  pos: new THREE.Vector3(0, 0, 0), heading: 0,
  vel: new THREE.Vector3(), yawRate: 0,
  group: new THREE.Group(),
  legs: [],
  phase: 0,
};
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
    thigh.geometry.translate(0, -L1 / 2, 0);   // rotate about hip
    shin.geometry.translate(0, -L2 / 2, 0);    // rotate about knee
    const hipPivot = new THREE.Group();
    hipPivot.position.set(hip.x, -0.06, hip.z);
    const kneePivot = new THREE.Group();
    kneePivot.position.set(0, -L1, 0);
    hipPivot.add(thigh); hipPivot.add(kneePivot); kneePivot.add(shin);
    dog.group.add(hipPivot);
    dog.legs.push({ hip, hipPivot, kneePivot,
                    foot: new THREE.Vector3(),     // planted world position
                    swing: 0, from: new THREE.Vector3(), to: new THREE.Vector3() });
  }
  scene.add(dog.group);
}
// body-frame (x fwd, z right) ↔ world, matching three.js rotation.y
function bodyToWorld(bx, bz, out) {
  const c = Math.cos(dog.heading), s = Math.sin(dog.heading);
  return out.set(dog.pos.x + bx * c + bz * s, 0, dog.pos.z - bx * s + bz * c);
}
function worldToBodyX(wx, wz) {  // body-frame x of a world point
  const c = Math.cos(dog.heading), s = Math.sin(dog.heading);
  return (wx - dog.pos.x) * c - (wz - dog.pos.z) * s;
}
function homeFoot(leg, out) { return bodyToWorld(leg.hip.x, leg.hip.z, out); }
for (const leg of dog.legs) homeFoot(leg, leg.foot);

const STEP_TIME = 0.28;     // s per half-cycle (one pair swings)
const STEP_H = 0.09;

function updateDog(dt, cmd) {
  // body motion (heading-relative command, clamped like the real handle)
  const f = THREE.MathUtils.clamp(cmd.forward, -1, 1);
  const st = THREE.MathUtils.clamp(cmd.strafe, -1, 1);
  const yaw = THREE.MathUtils.clamp(cmd.turn, -1.5, 1.5);
  dog.heading += yaw * dt;
  const c = Math.cos(dog.heading), s = Math.sin(dog.heading);
  const dx = (f * c - st * s) * dt, dz = -(f * s + st * c) * dt;
  // wall collision: slide
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

  // gait phase advances with motion (standing dogs don't march)
  const speed = Math.hypot(dx, dz) / dt + Math.abs(yaw) * 0.4;
  const moving = speed > 0.02;
  if (moving) dog.phase = (dog.phase + dt / (STEP_TIME * 2)) % 1;

  for (let i = 0; i < 4; i++) {
    const leg = dog.legs[i];
    const legPhase = (dog.phase + TROT_PAIRS[i] * 0.5) % 1;
    const inSwing = moving && legPhase < 0.5;
    if (inSwing) {
      if (leg.swing === 0) {           // lift off: pick the next foothold
        leg.from.copy(leg.foot);
        homeFoot(leg, leg.to);
        leg.to.x += (f * c - st * s) * STEP_TIME * 1.6;
        leg.to.z += -(f * s + st * c) * STEP_TIME * 1.6;
      }
      leg.swing = legPhase / 0.5;
      leg.foot.lerpVectors(leg.from, leg.to, leg.swing);
      leg.foot.y = Math.sin(leg.swing * Math.PI) * STEP_H;
    } else {
      leg.swing = 0;
      leg.foot.y = 0;                  // stance: foot stays planted — no sliding
    }
  }

  // body pose: height + a little bob, then leg IK toward each planted foot
  const bob = moving ? Math.sin(dog.phase * Math.PI * 4) * 0.012 : 0;
  dog.group.position.set(dog.pos.x, STAND_H + bob, dog.pos.z);
  dog.group.rotation.y = dog.heading;

  for (const leg of dog.legs) {
    // 2-segment IK in the leg's sagittal plane: hip → planted foot
    const dxp = worldToBodyX(leg.foot.x, leg.foot.z) - leg.hip.x;  // forward offset
    const dyp = (STAND_H + bob - 0.06) - leg.foot.y;               // vertical reach
    const reach = Math.min(Math.hypot(dxp, dyp), L1 + L2 - 0.01);
    const a1 = Math.atan2(dxp, dyp);                // hip-to-foot angle from vertical
    const a2 = Math.acos(THREE.MathUtils.clamp(     // thigh offset from that line
      (L1 * L1 + reach * reach - L2 * L2) / (2 * L1 * reach), -1, 1));
    const interior = Math.acos(THREE.MathUtils.clamp(
      (L1 * L1 + L2 * L2 - reach * reach) / (2 * L1 * L2), -1, 1));
    leg.hipPivot.rotation.z = a1 + a2;              // thigh forward of the reach line
    leg.kneePivot.rotation.z = -(Math.PI - interior);  // knee bends backward
  }
}

/* ---------- eyes: view-cone + occlusion detections ---------- */
const raycaster = new THREE.Raycaster();
function senseDetections() {
  const out = [];
  const eye = new THREE.Vector3(dog.pos.x, 0.45, dog.pos.z);
  const fwd = new THREE.Vector3(Math.cos(dog.heading), 0, -Math.sin(dog.heading));
  for (const d of doorObjs) {
    const to = d.pos.clone().sub(eye);
    const dist = to.length();
    if (dist > 7) continue;
    const bearing = Math.atan2(
      fwd.x * to.z - fwd.z * to.x, fwd.x * to.x + fwd.z * to.z);
    if (Math.abs(bearing) > 0.62) continue;            // ~70° FOV
    raycaster.set(eye, to.normalize());
    const hit = raycaster.intersectObjects(wallMeshes, false)[0];
    if (hit && hit.distance < dist - 0.4) continue;     // occluded
    // synthesize a bbox on the virtual 1280x720 frame
    const cx = 640 - (bearing / 0.62) * 600;
    const size = Math.min(420, 2200 / dist);
    out.push({ label: `${d.color} door`, confidence: 0.95,
               bbox: [cx - size / 4, 360 - size / 2, cx + size / 4, 360 + size / 2],
               door_id: d.id, distance: +dist.toFixed(2) });
    if (!d.seen) { d.seen = true; postEvent("detection", `sighted: ${d.color} door`, { door: d.id }); }
  }
  // wall-ahead as an obstacle detection (what explore behaviors steer by)
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

/* ---------- chamber logic ---------- */
const roomsEl = document.getElementById("rooms");
const visited = new Set();
for (const r of LEVEL.rooms) {
  const chip = document.createElement("span");
  chip.className = "room-chip"; chip.id = `room-${r.id}`; chip.textContent = r.id;
  roomsEl.appendChild(chip);
}
let won = false, t0 = performance.now();
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
    const reds = doorObjs.filter(d => d.seen && d.color === "red").length;
    document.getElementById("winDetail").textContent =
      `all ${LEVEL.rooms.length} rooms in ${secs}s · doors sighted: ${doorObjs.filter(d => d.seen).length}/${doorObjs.length}`;
    document.getElementById("win").classList.add("show");
    postEvent("task", `CHAMBER 01 COMPLETE · ${secs}s · red doors seen: ${reds}`,
              { time_s: +secs, doors_seen: doorObjs.filter(d => d.seen).map(d => d.id) });
  }
}

/* ---------- the wire: behaviors in, eyes out ---------- */
let serverCmd = { forward: 0, strafe: 0, turn: 0 };
let linked = false;
async function pollCmd() {
  try {
    const r = await (await fetch("/api/arena/cmd")).json();
    serverCmd = r.cmd; linked = true;
  } catch { linked = false; }
  document.getElementById("link").textContent = linked ? "behaviors: linked" : "behaviors: no server (WASD only)";
  document.getElementById("link").className = `link ${linked ? "on" : "off"}`;
  setTimeout(pollCmd, 50);
}
async function pushState() {
  if (linked) {
    try {
      await fetch("/api/arena/state", { method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          detections: senseDetections(),
          pose: { x: +dog.pos.x.toFixed(2), z: +dog.pos.z.toFixed(2), heading: +dog.heading.toFixed(3) },
          level: { name: LEVEL.name, rooms_visited: [...visited], won },
        }) });
    } catch {}
  }
  setTimeout(pushState, 100);
}
function postEvent(type, title, detail) {
  if (!linked) return;
  fetch("/api/arena/event", { method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ type, title, detail }) }).catch(() => {});
}
pollCmd(); pushState();

/* WASD for humans (overrides server while held) */
const keys = {};
addEventListener("keydown", (e) => keys[e.key.toLowerCase()] = true);
addEventListener("keyup", (e) => keys[e.key.toLowerCase()] = false);
function keyboardCmd() {
  const f = (keys.w ? 1 : 0) - (keys.s ? 1 : 0);
  const t = (keys.a ? 1 : 0) - (keys.d ? 1 : 0);
  if (f || t) return { forward: f * 0.9, strafe: 0, turn: t * 1.2 };
  return null;
}

/* ---------- loop ---------- */
const clockEl = document.getElementById("clock"), cmdEl = document.getElementById("cmdline");
let last = performance.now();
function frame(now) {
  const dt = Math.min((now - last) / 1000, 0.05);
  last = now;
  const cmd = keyboardCmd() || serverCmd;
  updateDog(dt, cmd);
  tickChamber();
  // chase camera
  const camTarget = new THREE.Vector3(
    dog.pos.x - Math.cos(dog.heading) * 3.4, 2.4, dog.pos.z + Math.sin(dog.heading) * 3.4);
  camera.position.lerp(camTarget, 0.06);
  camera.lookAt(dog.pos.x, 0.5, dog.pos.z);
  if (!won) clockEl.textContent = `${((now - t0) / 1000).toFixed(1)}s`;
  cmdEl.textContent = `cmd f=${cmd.forward.toFixed(2)} t=${cmd.turn.toFixed(2)}`;
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
