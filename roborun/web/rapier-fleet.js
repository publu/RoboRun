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
import { initPhysics } from "./physics.js";

const $ = (s) => document.querySelector(s);
const COLORS = ["#00d47e", "#4090e0", "#d4a030", "#e0563f", "#a070e0", "#40c0c0", "#e090c0", "#90c040"];
const LABELS = ["pallet", "forklift", "shelf", "crate", "barrel", "agv", "worker", "bin"];
const H = 1 / 60, FLOOR_GAP = 6;
const floorY = (f) => f * FLOOR_GAP;

const S = { world: null, ctl: null, robots: [], floors: 3, size: 50, playing: true,
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
  buildCanvases();
  spawnRobots();
  S.ready = true;
}

function spawnRobots() {
  for (const rb of S.robots) { try { S.world.removeRigidBody(rb.body); } catch {} }
  S.robots = []; S.found = 0; S.detected = new Set(); S.t0 = performance.now();
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

// ── render: one top-down canvas per floor (true physics positions) ─────────
function buildCanvases() {
  const host = $("#floors"); if (!host) return;
  host.innerHTML = ""; S.canvases = [];
  for (let f = 0; f < S.floors; f++) {
    const div = document.createElement("div"); div.className = "floorp";
    div.innerHTML = `<h3><span>Floor ${f}</span><b id="fb${f}"></b></h3>`;
    const cv = document.createElement("canvas"); cv.width = 520; cv.height = 520;
    div.appendChild(cv); host.appendChild(div); S.canvases[f] = cv;
  }
}
function draw() {
  for (let f = 0; f < S.floors; f++) {
    const cv = S.canvases[f]; if (!cv) continue;
    const box = cv.clientWidth || 500, dpr = window.devicePixelRatio || 1;
    if (cv.width !== Math.round(box * dpr)) { cv.width = Math.round(box * dpr); cv.height = Math.round(box * dpr); }
    const g = cv.getContext("2d"); g.setTransform(dpr, 0, 0, dpr, 0, 0);
    const W = box, P = 12, sc = (W - 2 * P) / S.size, X = x => P + x * sc, Y = z => P + z * sc;
    g.clearRect(0, 0, W, W); g.fillStyle = "#080b08"; g.fillRect(0, 0, W, W);
    const fd = S.floorData[f];
    g.strokeStyle = "rgba(120,175,144,.08)"; g.lineWidth = 1;
    for (const rm of fd.rooms) g.strokeRect(X(rm.x), Y(rm.z), rm.w * sc, rm.h * sc);
    g.strokeStyle = "#2c3b2c"; g.lineWidth = 2.5; g.lineCap = "round"; g.beginPath();
    for (const [x1, z1, x2, z2] of fd.walls) { g.moveTo(X(x1), Y(z1)); g.lineTo(X(x2), Y(z2)); }
    g.stroke();
    for (let k = 0; k < fd.items.length; k++) {
      const it = fd.items[k], on = S.detected.has(f + ":" + k);
      g.fillStyle = on ? "rgba(0,212,126,.95)" : "rgba(120,175,144,.32)";
      g.beginPath(); g.arc(X(it.x), Y(it.z), on ? 4 : 2.4, 0, 6.28); g.fill();
      if (on) { g.fillStyle = "rgba(0,212,126,.7)"; g.font = "9px ui-monospace,monospace"; g.fillText(it.label, X(it.x) + 6, Y(it.z) + 3); }
    }
    for (const e of elevatorsOn(f)) {
      g.strokeStyle = "#4090e0"; g.fillStyle = "rgba(64,144,224,.14)"; g.lineWidth = 1.5;
      const s = 3.4 * sc; g.fillRect(X(e.x) - s / 2, Y(e.z) - s / 2, s, s); g.strokeRect(X(e.x) - s / 2, Y(e.z) - s / 2, s, s);
      g.fillStyle = "#4090e0"; g.font = "11px ui-monospace,monospace"; g.fillText("⇅", X(e.x) - 4, Y(e.z) + 4);
    }
    let cnt = 0;
    for (const rb of S.robots) {
      if (rb.floor !== f || rb.inElev > 0) continue; cnt++;
      if (rb.trail.length > 1) {
        g.strokeStyle = rb.color + "33"; g.lineWidth = 1.5; g.beginPath();
        rb.trail.forEach((t, i) => i ? g.lineTo(X(t.x), Y(t.z)) : g.moveTo(X(t.x), Y(t.z))); g.stroke();
      }
      const t = rb.body.translation(), x = X(t.x), y = Y(t.z), h = rb.heading;
      g.fillStyle = rb.color; g.beginPath();
      g.moveTo(x + Math.cos(h) * 7, y + Math.sin(h) * 7);
      g.lineTo(x + Math.cos(h + 2.5) * 5.5, y + Math.sin(h + 2.5) * 5.5);
      g.lineTo(x + Math.cos(h - 2.5) * 5.5, y + Math.sin(h - 2.5) * 5.5);
      g.closePath(); g.fill();
    }
    const fb = document.getElementById("fb" + f); if (fb) fb.textContent = cnt + " bots";
  }
}
function hud() {
  const cov = S.totalItems ? Math.round(S.found / S.totalItems * 100) : 0;
  const el = (performance.now() - S.t0) / 1000;
  const tile = (n, l) => `<div class="kpi"><div class="n">${n}</div><div class="l">${l}</div></div>`;
  $("#kpis").innerHTML = tile(S.robots.length, "robots") + tile(S.found + "/" + S.totalItems, "found") +
    tile(cov + "%", "coverage") + tile(el.toFixed(0) + "s", "elapsed");
}

let last = performance.now(), acc = 0;
function loop(now) {
  if (S.ready) {
    const dt = Math.min(0.05, (now - last) / 1000) * S.speed; last = now;
    if (S.playing) { acc += dt; let guard = 0; while (acc >= H && guard++ < 240) { stepOnce(); acc -= H; } }
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
    s.innerHTML = d.active
      ? `Real Rapier physics — robots across ${S.floors} floors (via elevators) collecting jointly into <b>${d.active.project} / ${d.active.environment}</b>. Every detection lands in that environment's search + spatial map.`
      : `Real Rapier physics — robots across floors via elevators, in one warehouse. Detections go to the active project (currently <b>scratch</b>).`;
  }).catch(() => {});
}

wire();
build().then(() => requestAnimationFrame(loop)).catch(e => {
  const s = $("#scope"); if (s) s.innerHTML = `<span style="color:var(--bad)">Rapier failed to load: ${e}</span>`;
});
