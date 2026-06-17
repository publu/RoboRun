/* rapier-fleet.js — a REAL multi-floor fleet: N robots in one Rapier physics
 * warehouse, stacked floors connected by elevators.
 *
 * One Rapier world; each floor is a ground slab + walls at its own Y level (6 m
 * apart, so floors are physically independent). N kinematic-capsule robots with
 * a character controller collide with walls AND each other, navigate room to
 * room, and ride elevators between floors. Every detection POSTs to
 * /api/fleet/observe → indexed into the active project/environment store. One
 * top-down canvas per floor renders the true physics positions.
 */
import RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import { initPhysics } from "./physics.js";

const $ = (s) => document.querySelector(s);
const COLORS = ["#00d47e", "#4090e0", "#d4a030", "#e0563f", "#a070e0", "#40c0c0", "#e090c0", "#90c040"];
const LABELS = ["pallet", "forklift", "shelf", "crate", "barrel", "agv", "worker", "bin"];
const H = 1 / 60, FLOOR_GAP = 6;
const floorY = (f) => f * FLOOR_GAP;

const S = { world: null, ctl: null, robots: [], floors: 3, size: 50, playing: false,
            speed: 1, n: 8, floorData: [], elevators: [], detected: new Set(), found: 0,
            totalItems: 0, t0: performance.now(), canvases: [], ready: false };

// ── deterministic warehouse layout per floor + elevators ───────────────────
function buildLayout() {
  const sz = S.size, cols = 4, rows = 3, cw = sz / cols, ch = sz / rows;
  S.floorData = []; S.elevators = []; S.totalItems = 0;
  let seed = 7; const rng = () => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed / 0x7fffffff; };
  for (let f = 0; f < S.floors; f++) {
    const walls = [], items = [], rooms = [];
    walls.push([0, 0, sz, 0], [sz, 0, sz, sz], [sz, sz, 0, sz], [0, sz, 0, 0]);
    for (let c = 1; c < cols; c++) { const x = c * cw; for (let r = 0; r < rows; r++) walls.push([x, r * ch + 2.2, x, (r + 1) * ch - 2.2]); }
    for (let r = 1; r < rows; r++) { const z = r * ch; for (let c = 0; c < cols; c++) walls.push([c * cw + 2.2, z, (c + 1) * cw - 2.2, z]); }
    for (let c = 0; c < cols; c++) for (let r = 0; r < rows; r++) {
      rooms.push({ x: c * cw, z: r * ch, w: cw, h: ch });
      const n = 1 + (rng() < 0.6 ? 1 : 0);
      for (let k = 0; k < n; k++) items.push({ label: LABELS[Math.floor(rng() * LABELS.length)],
        x: c * cw + 2 + rng() * (cw - 4), z: r * ch + 2 + rng() * (ch - 4) });
    }
    S.floorData.push({ walls, items, rooms }); S.totalItems += items.length;
  }
  const sx = sz / 2, sz2 = sz / 2;     // shared elevator shaft at the center
  for (let f = 0; f < S.floors - 1; f++) S.elevators.push({ x: sx, z: sz2, from: f, to: f + 1 });
}
const elevatorsOn = (f) => S.elevators.filter(e => e.from === f || e.to === f);
const SHAFT = () => ({ x: S.size / 2, z: S.size / 2 });

// ── build the stacked Rapier world ─────────────────────────────────────────
async function build() {
  S.ready = false;
  await initPhysics();
  buildLayout();
  try { S.world && S.world.free(); } catch {}
  const w = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  w.timestep = H;
  for (let f = 0; f < S.floors; f++) {
    const fy = floorY(f);
    const g = w.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    w.createCollider(RAPIER.ColliderDesc.cuboid(S.size, 0.1, S.size)
      .setTranslation(S.size / 2, fy - 0.1, S.size / 2).setFriction(0.8), g);
    for (const [x1, z1, x2, z2] of S.floorData[f].walls) {
      const cx = (x1 + x2) / 2, cz = (z1 + z2) / 2;
      const hx = Math.max(Math.abs(x2 - x1) / 2, 0.12), hz = Math.max(Math.abs(z2 - z1) / 2, 0.12);
      const b = w.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(cx, fy + 0.6, cz));
      w.createCollider(RAPIER.ColliderDesc.cuboid(hx, 0.6, hz), b);
    }
  }
  S.world = w;
  S.ctl = w.createCharacterController(0.02);
  S.ctl.setApplyImpulsesToDynamicBodies(false);
  init3D();
  buildScene();
  spawnRobots();
  S.ready = true;
}

function spawnRobots() {
  for (const rb of S.robots) { try { S.world.removeRigidBody(rb.body); } catch {} }
  S.robots = []; S.found = 0; S.detected = new Set(); S.elapsed = 0;
  const per = Math.ceil(Math.sqrt(S.n));
  for (let i = 0; i < S.n; i++) {
    const gx = i % per, gz = Math.floor(i / per);
    const x = 3 + gx * 1.4 + Math.random() * .4, z = 3 + gz * 1.4 + Math.random() * .4;
    const body = S.world.createRigidBody(
      RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(x, floorY(0) + 0.3, z));
    const col = S.world.createCollider(RAPIER.ColliderDesc.capsule(0.14, 0.3), body);
    const rb = { id: "r" + i, body, col, color: COLORS[i % COLORS.length], floor: 0,
                 heading: Math.random() * 6.28, seen: 0, trail: [], inElev: 0, dest: null, target: null };
    pickTarget(rb); S.robots.push(rb);
  }
  build3DRobots();
}

function pickTarget(rb) {
  const elevs = elevatorsOn(rb.floor);
  if (elevs.length && Math.random() < 0.25) {
    const e = elevs[Math.floor(Math.random() * elevs.length)];
    rb.target = { x: e.x, z: e.z, elevator: e };
  } else {
    const fd = S.floorData[rb.floor], rm = fd.rooms[Math.floor(Math.random() * fd.rooms.length)];
    rb.target = { x: rm.x + rm.w / 2 + (Math.random() - .5) * (rm.w - 6),
                  z: rm.z + rm.h / 2 + (Math.random() - .5) * (rm.h - 6) };
  }
}

function detect(rb) {
  const cur = rb.body.translation(), items = S.floorData[rb.floor].items;
  for (let k = 0; k < items.length; k++) {
    const it = items[k];
    if (Math.hypot(it.x - cur.x, it.z - cur.z) < 2.2) {
      const key = rb.floor + ":" + k;
      if (!S.detected.has(key)) { S.detected.add(key); S.found++; report(rb, it); }
      rb.seen++;
    }
  }
}
function report(rb, it) {
  const p = rb.body.translation();
  fetch("/api/fleet/observe", { method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ robot_id: rb.id, pose: { x: p.x, y: p.z },
      detections: [{ label: it.label, score: 0.9, bbox: [0, 0, 12, 12] }] }) }).catch(() => {});
}

function stepOnce() {
  const v = 1.6;
  for (const rb of S.robots) {
    if (rb.inElev > 0) {                                  // riding the elevator
      rb.inElev -= H;
      if (rb.inElev <= 0 && rb.dest != null) {
        rb.floor = rb.dest; rb.dest = null;
        const s = SHAFT();
        rb.body.setTranslation({ x: s.x, y: floorY(rb.floor) + 0.3, z: s.z }, true);
        rb.trail = []; pickTarget(rb);
      }
      continue;
    }
    const cur = rb.body.translation();
    let dx = rb.target.x - cur.x, dz = rb.target.z - cur.z;
    const d = Math.hypot(dx, dz);
    if (d < 1.0) {
      if (rb.target.elevator) {                           // step into the shaft → other floor
        const e = rb.target.elevator;
        rb.dest = (e.from === rb.floor) ? e.to : e.from; rb.inElev = 0.8;
      } else { detect(rb); pickTarget(rb); }
      continue;
    }
    dx /= d; dz /= d; rb.heading = Math.atan2(dz, dx);
    S.ctl.computeColliderMovement(rb.col, { x: dx * v * H, y: 0, z: dz * v * H });
    const mv = S.ctl.computedMovement();
    rb.body.setNextKinematicTranslation({ x: cur.x + mv.x, y: cur.y, z: cur.z + mv.z });
    const lp = rb.trail[rb.trail.length - 1];
    if (!lp || Math.hypot(cur.x - lp.x, cur.z - lp.z) > 1.2) {
      rb.trail.push({ x: cur.x, z: cur.z }); if (rb.trail.length > 16) rb.trail.shift();
    }
    detect(rb);
  }
  S.world.step();
}

// ── render: a real 3D warehouse (three.js), same world as the cockpit ───────
// The Rapier world is already 3D (x,z floor plane, y up, floors stacked
// FLOOR_GAP apart). We render the true physics positions in three.js with an
// orbit camera — no flat 2D top-down. Items light up green on detection.
const V = { scene: null, camera: null, renderer: null, statics: null, robots: null,
            itemMeshes: [], center: new THREE.Vector3(), theta: 0.9, phi: 1.04, r: 78,
            drag: false, px: 0, py: 0, idle: 0, inited: false };
const GREEN = 0x00d47e, DIM = 0x35563f;

function init3D() {
  if (V.inited) return;
  const cv = $("#stage"); if (!cv) return;
  V.renderer = new THREE.WebGLRenderer({ canvas: cv, antialias: true });
  V.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  V.scene = new THREE.Scene();
  V.scene.background = new THREE.Color(0x070a07);
  V.scene.fog = new THREE.Fog(0x070a07, 120, 260);
  V.camera = new THREE.PerspectiveCamera(46, 1, 0.5, 600);
  V.scene.add(new THREE.AmbientLight(0xb8d8c4, 0.7));
  V.scene.add(new THREE.HemisphereLight(0x88c0a0, 0x0a140d, 0.5));
  const key = new THREE.DirectionalLight(0xffffff, 1.0); key.position.set(40, 90, 20); V.scene.add(key);
  const fill = new THREE.DirectionalLight(0x4a90e0, 0.4); fill.position.set(-30, 40, -25); V.scene.add(fill);
  V.statics = new THREE.Group(); V.scene.add(V.statics);
  V.robots = new THREE.Group(); V.scene.add(V.robots);
  // orbit controls (manual — no addons needed)
  cv.addEventListener("pointerdown", e => { V.drag = true; V.px = e.clientX; V.py = e.clientY; cv.setPointerCapture(e.pointerId); });
  cv.addEventListener("pointerup", e => { V.drag = false; try { cv.releasePointerCapture(e.pointerId); } catch {} });
  cv.addEventListener("pointermove", e => {
    if (!V.drag) return; V.idle = 0;
    V.theta -= (e.clientX - V.px) * 0.006; V.py != null && (V.phi = Math.max(0.18, Math.min(1.45, V.phi - (e.clientY - V.py) * 0.006)));
    V.px = e.clientX; V.py = e.clientY;
  });
  cv.addEventListener("wheel", e => { e.preventDefault(); V.r = Math.max(42, Math.min(190, V.r * (1 + e.deltaY * 0.0012))); }, { passive: false });
  V.inited = true;
}

function buildScene() {
  if (!V.statics) return;
  V.statics.clear(); V.itemMeshes = [];
  const sz = S.size, c = sz / 2;
  // aim a touch above the lower floors so the whole stack sits centred in frame
  V.center.set(c, floorY(Math.max(0, S.floors - 1)) * 0.3 + 1, c);
  V.r = 64 + sz * 0.62;
  const wallMat = new THREE.MeshStandardMaterial({ color: 0x3c5a45, transparent: true, opacity: 0.72, roughness: 0.85 });
  const slabMat = new THREE.MeshStandardMaterial({ color: 0x0d160f, transparent: true, opacity: 0.55, roughness: 1 });
  for (let f = 0; f < S.floors; f++) {
    const fy = floorY(f), fd = S.floorData[f];
    const slab = new THREE.Mesh(new THREE.BoxGeometry(sz, 0.2, sz), slabMat);
    slab.position.set(c, fy - 0.1, c); V.statics.add(slab);
    const grid = new THREE.GridHelper(sz, 12, 0x244031, 0x18271d);
    grid.position.set(c, fy + 0.02, c); grid.material.transparent = true; grid.material.opacity = 0.5; V.statics.add(grid);
    for (const [x1, z1, x2, z2] of fd.walls) {
      const cx = (x1 + x2) / 2, cz = (z1 + z2) / 2;
      const hx = Math.max(Math.abs(x2 - x1), 0.24), hz = Math.max(Math.abs(z2 - z1), 0.24);
      const wall = new THREE.Mesh(new THREE.BoxGeometry(hx, 1.4, hz), wallMat);
      wall.position.set(cx, fy + 0.7, cz); V.statics.add(wall);
    }
    const itemRow = [];
    for (let k = 0; k < fd.items.length; k++) {
      const it = fd.items[k];
      const m = new THREE.Mesh(new THREE.CylinderGeometry(0.5, 0.5, 0.7, 8),
        new THREE.MeshStandardMaterial({ color: DIM, roughness: 0.8 }));
      m.position.set(it.x, fy + 0.35, it.z); V.statics.add(m); itemRow.push(m);
    }
    V.itemMeshes[f] = itemRow;
    for (const e of elevatorsOn(f)) {
      if (e.from !== f) continue;                         // draw the shaft once, spanning the two floors
      const span = FLOOR_GAP;
      const shaft = new THREE.Mesh(new THREE.BoxGeometry(3.4, span, 3.4),
        new THREE.MeshStandardMaterial({ color: 0x4a90e0, transparent: true, opacity: 0.16, roughness: 0.4 }));
      shaft.position.set(e.x, fy + span / 2, e.z); V.statics.add(shaft);
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(shaft.geometry),
        new THREE.LineBasicMaterial({ color: 0x4a90e0, transparent: true, opacity: 0.55 }));
      edges.position.copy(shaft.position); V.statics.add(edges);
    }
  }
  renderFloorTags();
}

function build3DRobots() {
  if (!V.robots) return;
  V.robots.clear();
  for (const rb of S.robots) {
    const col = new THREE.Color(rb.color);
    const g = new THREE.Group();
    const body = new THREE.Mesh(new THREE.CapsuleGeometry(0.55, 0.8, 4, 10),
      new THREE.MeshStandardMaterial({ color: col, roughness: 0.45, emissive: col, emissiveIntensity: 0.4 }));
    body.rotation.z = Math.PI / 2;                        // lie the capsule along heading
    g.add(body);
    const nose = new THREE.Mesh(new THREE.ConeGeometry(0.42, 0.95, 10),
      new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: col, emissiveIntensity: 0.7 }));
    nose.rotation.z = -Math.PI / 2; nose.position.x = 1.05; g.add(nose);
    V.robots.add(g); rb.mesh = g;
  }
}

function draw() {
  if (!V.renderer) return;
  // keep the drawing buffer matched to the element size
  const cv = V.renderer.domElement, w = cv.clientWidth || 800, h = cv.clientHeight || 560;
  if (cv.width !== Math.round(w * V.renderer.getPixelRatio()) || cv.height !== Math.round(h * V.renderer.getPixelRatio())) {
    V.renderer.setSize(w, h, false); V.camera.aspect = w / h; V.camera.updateProjectionMatrix();
  }
  // robots → true physics transforms (hidden while inside the elevator shaft)
  for (const rb of S.robots) {
    if (!rb.mesh) continue;
    const t = rb.body.translation();
    rb.mesh.position.set(t.x, t.y + 0.05, t.z);
    rb.mesh.rotation.y = -rb.heading + Math.PI / 2;
    rb.mesh.visible = rb.inElev <= 0;
  }
  // items light up green once detected
  for (let f = 0; f < S.floors; f++) {
    const row = V.itemMeshes[f] || [];
    for (let k = 0; k < row.length; k++) {
      const on = S.detected.has(f + ":" + k), mat = row[k].material;
      const want = on ? GREEN : DIM;
      if (mat.color.getHex() !== want) {
        mat.color.setHex(want); mat.emissive = new THREE.Color(on ? GREEN : 0x000000);
        mat.emissiveIntensity = on ? 0.6 : 0; row[k].scale.setScalar(on ? 1.35 : 1);
      }
    }
  }
  // orbit camera (gentle auto-rotate when the user isn't dragging)
  if (!V.drag) { V.idle++; if (V.idle > 90) V.theta += 0.0016; }
  const sp = V.phi, st = V.theta;
  V.camera.position.set(
    V.center.x + V.r * Math.sin(sp) * Math.cos(st),
    V.center.y + V.r * Math.cos(sp),
    V.center.z + V.r * Math.sin(sp) * Math.sin(st));
  V.camera.lookAt(V.center);
  V.renderer.render(V.scene, V.camera);
}

function renderFloorTags() {
  const host = $("#floorTags"); if (!host) return;
  let html = "";
  for (let f = S.floors - 1; f >= 0; f--) html += `<div class="fr">Floor <b>${f}</b> <span id="fb${f}"></span></div>`;
  host.innerHTML = html;
}
function hud() {
  const cov = S.totalItems ? Math.round(S.found / S.totalItems * 100) : 0;
  const el = S.elapsed || 0;            // sim time — only advances while playing
  const tile = (n, l) => `<div class="kpi"><div class="n">${n}</div><div class="l">${l}</div></div>`;
  $("#kpis").innerHTML = tile(S.robots.length, "robots") + tile(S.found + "/" + S.totalItems, "found") +
    tile(cov + "%", "coverage") + tile(el.toFixed(0) + "s", "elapsed");
  const cnt = new Array(S.floors).fill(0);
  for (const rb of S.robots) if (rb.inElev <= 0) cnt[rb.floor]++;
  for (let f = 0; f < S.floors; f++) { const fb = document.getElementById("fb" + f); if (fb) fb.textContent = "· " + cnt[f] + " bots"; }
}

let last = performance.now(), acc = 0;
function loop(now) {
  if (S.ready) {
    const real = Math.min(0.05, (now - last) / 1000); last = now;
    if (S.playing) {
      S.elapsed = (S.elapsed || 0) + real;     // count sim time only while running
      acc += real * S.speed; let guard = 0;
      while (acc >= H && guard++ < 240) { stepOnce(); acc -= H; }
    }
    draw(); hud();
  } else last = now;
  requestAnimationFrame(loop);
}

// ── controls + boot ────────────────────────────────────────────────────────
function wire() {
  $("#n") && ($("#n").oninput = e => { S.n = +e.target.value; $("#nlab").textContent = S.n; if (S.ready) spawnRobots(); });
  $("#fl") && ($("#fl").oninput = e => { S.floors = +e.target.value; $("#flab").textContent = S.floors; build(); });
  $("#spd") && ($("#spd").oninput = e => { S.speed = +e.target.value / 10; $("#slab").textContent = S.speed.toFixed(1) + "×"; });
  $("#play") && ($("#play").onclick = () => { S.playing = !S.playing; $("#play").textContent = S.playing ? "⏸ pause" : "▶ play"; });
  $("#reset") && ($("#reset").onclick = () => { if (S.ready) spawnRobots(); });
  fetch("/api/projects/active").then(r => r.json()).then(d => {
    const s = $("#scope"); if (!s) return;
    const where = d.active ? `<b>${d.active.project} / ${d.active.environment}</b>` : `<b>scratch</b>`;
    s.innerHTML = `<b>Layer 2: real Rapier physics.</b> ${S.floors} floors of warehouse, real bodies + collisions + elevators — robots collecting jointly into ${where}; every detection lands in its search + spatial map. ` +
      `<span style="color:var(--fg-dim)">Coordination strategies (how they decide where to search) live in the <a href="/fleet">Swarm Lab →</a></span>`;
  }).catch(() => {});
}

wire();
window.__fleet = { S, V };                 // harness hook (status/positions for tests)
build().then(() => requestAnimationFrame(loop)).catch(e => {
  const s = $("#scope"); if (s) s.innerHTML = `<span style="color:var(--bad)">Rapier failed to load: ${e}</span>`;
});
