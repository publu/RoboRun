/* RoboRun · FLEET — a teaching sandbox for swarm coordination over an
   imperfect radio. Many quadrupeds cover a field together, discover data
   points, and relay them back to a base station — with only the radio range,
   airtime, memory and one-job-at-a-time limits a real robot has.

   Hover a robot to see exactly what *it* knows (what it sensed itself vs what
   it only heard from peers). Read the CONCEPTS tab for what delivered/dropped/
   overlap mean and how libp2p-style gossip + routing build swarm intelligence.

   No physics engine, no three.js: the comms model is the lesson, so the robots
   are kinematic and the whole thing stays readable. The four strategies mirror
   roborun/swarm/*.py. */

const cv = document.getElementById("field");
const ctx = cv.getContext("2d");
const $ = (id) => document.getElementById(id);

/* ── world + coverage grid ───────────────────────────────────────────── */
const WORLD = 56;            // metres across the (square) field
const CG = 28;               // coverage cells per side → 2 m cells
const CELL = WORLD / CG;
const SENSE = 3.2;           // metres a robot maps around itself as it walks
const NCELL = CG * CG;
const cx = (i) => (i % CG + 0.5) * CELL;          // cell index → world centre
const cy = (i) => ((i / CG | 0) + 0.5) * CELL;

/* ── config (mirrored from sliders / URL) ────────────────────────────── */
const cfg = { count: 6, range: 14, loss: 85, bw: 4, mem: 60, buf: 8,
              targets: 8, base: true, env: "open", strat: "gossip" };
(function fromURL() {
  const q = new URLSearchParams(location.search);
  for (const k of ["count", "range", "loss", "bw", "mem", "buf", "targets"])
    if (q.has(k)) cfg[k] = +q.get(k);
  if (q.has("strat")) cfg.strat = q.get("strat");
  if (q.has("env")) cfg.env = q.get("env");
})();

/* ── environments: obstacles block movement AND line-of-sight ─────────── */
const ENV = {
  open: { obstacles: [] },
  obstacles: { obstacles: [
    [10, 12, 7, 7], [34, 9, 9, 6], [20, 26, 8, 8], [40, 30, 7, 10],
    [8, 34, 8, 7], [30, 42, 9, 7] ] },
  rooms: { obstacles: [
    // a coarse building: wall segments as thin rectangles + a couple blocks
    [14, 16, 2, 22], [14, 16, 18, 2], [30, 16, 2, 14], [14, 36, 14, 2],
    [38, 14, 2, 26], [38, 38, 12, 2], [22, 30, 2, 14] ] },
};
let obstacles = [];               // [x, y, w, h] world rects
let coverable = new Uint8Array(NCELL);   // 1 where a cell is open ground
let NCOVER = NCELL;
let basePos = { x: WORLD / 2, y: WORLD * 0.055 };

function inObstacle(x, y, pad = 0) {
  for (const [ox, oy, ow, oh] of obstacles)
    if (x >= ox - pad && x <= ox + ow + pad && y >= oy - pad && y <= oy + oh + pad) return true;
  return false;
}
// segment vs axis-aligned rect (slab method) — for line-of-sight through walls
function segHitsRect(ax, ay, bx, by, ox, oy, ow, oh) {
  let t0 = 0, t1 = 1; const dx = bx - ax, dy = by - ay;
  for (const [p, q] of [[-dx, ax - ox], [dx, ox + ow - ax], [-dy, ay - oy], [dy, oy + oh - ay]]) {
    if (p === 0) { if (q < 0) return false; }
    else { const r = q / p; if (p < 0) { if (r > t1) return false; if (r > t0) t0 = r; }
           else { if (r < t0) return false; if (r < t1) t1 = r; } }
  }
  return t0 <= t1;
}
function los(ax, ay, bx, by) {
  for (const [ox, oy, ow, oh] of obstacles)
    if (segHitsRect(ax, ay, bx, by, ox, oy, ow, oh)) return false;
  return true;
}

function buildEnv(name) {
  obstacles = (ENV[name] || ENV.open).obstacles.map((r) => r.slice());
  coverable = new Uint8Array(NCELL); NCOVER = 0;
  for (let c = 0; c < NCELL; c++) {
    if (!inObstacle(cx(c), cy(c))) { coverable[c] = 1; NCOVER++; }
  }
  basePos = { x: WORLD / 2, y: WORLD * 0.055 };
}

/* ── strategies ──────────────────────────────────────────────────────── */
const STRATS = [
  { id: "independent", nm: "Lone wolves",
    ds: "Radio off. Every robot maps on its own.",
    prose: "No coordination at all — the baseline. Each robot just walks to the nearest patch it personally hasn't seen. It's robust (nothing to break) but two robots happily search the same ground because neither knows what the other did. Watch the overlap count climb." },
  { id: "gossip", nm: "Gossip",
    ds: "Tell neighbours what you mapped; skip what they've covered.",
    prose: "Each robot broadcasts what it just mapped to anyone in range, and believes what it hears — map knowledge floods hop-by-hop, libp2p gossipsub-style. Robots stop re-walking ground a neighbour already covered… but with no claims, two linked robots often pick the SAME nearest frontier and clump. Bounded by airtime and range." },
  { id: "auction", nm: "Claim & yield",
    ds: "Call dibs on your next cell; yield to a closer robot.",
    prose: "A tiny market. Before committing to a cell, a robot announces a claim with its distance; if a closer robot already claimed it, it yields and picks another. Sharing INTENT (not just the map) de-conflicts who-goes-where, so it usually finishes first — as long as everyone stays linked." },
  { id: "leader", nm: "One commander",
    ds: "A leader hands out non-overlapping targets.",
    prose: "The lowest-id robot reachable in each radio cluster becomes the leader and hands out non-overlapping targets. Centralised and tidy — but a robot that drifts out of the leader's range gets no orders and falls back to lone-wolf. A lesson in single-point-of-failure vs. range." },
  { id: "custom", nm: "✨ Your algorithm",
    ds: "Write it yourself — or have your local LLM draft one.",
    prose: "" },
];
const CUSTOM_TEMPLATE = `// Per-robot policy — runs ~5x/sec for EVERY robot.
// r = this robot, H = helpers. Edit this, then press Run.
for (const m of H.inbox(r)) {           // read what neighbours told me
  if (m.kind === 'cells') H.learn(r, m.cells);
  if (m.kind === 'claim') H.reserve(r, m.cell, m.by, m.dist);
}
H.shareMapped(r);                       // tell neighbours what I just sensed
if (H.arrived(r)) {                     // free? pick one new goal (one job at a time)
  const cell = H.nearestUnknown(r, true);   // skip ground a closer robot claimed
  if (cell >= 0) H.broadcastClaim(r, cell);
  H.goto(r, cell);
}`;
const CUSTOM_API = `r.id, r.x, r.y          this robot's id and position (metres)
r.payload              Set of data ids it is carrying toward base
H.arrived(r)           true when free to choose a new goal
H.inbox(r)             drained messages: {kind:'cells',cells} | {kind:'claim',cell,by,dist}
H.learn(r, cells)      merge map cells a peer told you about
H.reserve(r,cell,by,dist)  record a peer's claim on a cell
H.shareMapped(r)       broadcast cells you just sensed (uses 1 airtime)
H.broadcastClaim(r,cell)   announce you are taking this cell
H.claimedByOther(r,cell)   did a closer robot already claim it?
H.nearestUnknown(r, avoidClaims)  nearest unmapped cell, or -1
H.goto(r, cell)        commit to a cell (robot moves to ONE goal at a time)
H.neighborCount(r)     robots in radio range right now
H.knows(r, cell)       already mapped (sensed or heard)?`;
const CODE = {
  independent: `@behavior(hz=5)
def explore(robot):
    if robot.arrived():                 # one move at a time
        robot.goto(robot.nearest_unknown())`,
  gossip: `@behavior(hz=5)
def explore(robot):
    for cell in robot.radio.recv():     # drain inbox (capped by memory)
        robot.note_mapped(cell)         # merge — now I skip it too
    fresh = robot.just_mapped()
    if fresh:
        robot.radio.broadcast(fresh)    # one slice of airtime
    if robot.arrived():
        robot.goto(robot.nearest_unknown())`,
  auction: `@behavior(hz=5)
def explore(robot):
    for m in robot.radio.recv():
        if m.kind == "cells": robot.note_mapped(m.cells)
        if m.kind == "claim": robot.reserve(m.cell, m.by, m.dist)
    if robot.arrived():
        cell = robot.nearest_unclaimed()    # skip a closer bot's cell
        robot.radio.broadcast_claim(cell)   # call dibs
        robot.goto(cell)`,
  leader: `@behavior(hz=5)
def explore(robot):
    leader = robot.radio.leader_in_range()  # None if I drifted off
    if robot.is_leader:
        for req in robot.radio.recv("request"):
            robot.radio.assign(req.sender, robot.pick_cell_for(req.sender))
    elif leader and robot.arrived():
        robot.radio.request(leader)
        robot.goto(robot.assigned_cell())
    elif robot.arrived():
        robot.goto(robot.nearest_unknown())  # out of range → solo`,
};

/* ── state ───────────────────────────────────────────────────────────── */
const PALETTE = ["#00d47e", "#40a0e0", "#e0a030", "#d84a4a", "#a060f0", "#56b6c2",
  "#e06aa0", "#7ad07a", "#d0d040", "#6a90ff", "#ff9a5a", "#5ad0c0",
  "#c678dd", "#9fb0bd", "#40e0a0", "#e0e0e0"];
let coveredMask = new Int32Array(NCELL);   // bitmask of robots that mapped a cell
let bots = [], targets = [], base = null;
const msgs = [];            // in-flight visuals
let claims = new Map();
let metrics, t0, paused = false, lastT = 0, hoverBot = null, hoverXY = null;
let customFn = null;        // compiled user/LLM strategy: (r, H) => void
const rng = () => Math.random();

/* the stable surface a custom (or LLM-written) policy gets — same primitives
   the built-in strategies use, nothing else, so user code can't reach into the
   sim's guts. Each helper takes the robot first. */
const H = {
  arrived: (r) => arrived(r),
  inbox: (r) => recv(r),
  learn: (r, cells) => { if (cells) for (const c of cells) remember(r, c, "heard"); },
  reserve: (r, cell, by, dist) => { const cur = claims.get(cell); if (!cur || dist < cur.dist) claims.set(cell, { by, dist }); },
  shareMapped: (r) => { if (r.fresh.length) { const ok = broadcast(r, "cells", { cells: r.fresh.slice(0, 12) }); r.fresh = []; return ok; } return false; },
  broadcastClaim: (r, cell) => { if (cell < 0) return false; const dist = Math.hypot(cx(cell) - r.x, cy(cell) - r.y);
    claims.set(cell, { by: r.id, dist }); return broadcast(r, "claim", { cell, by: r.id, dist }); },
  claimedByOther: (r, cell) => { const cl = claims.get(cell); return !!(cl && cl.by !== r.id); },
  nearestUnknown: (r, avoid) => nearestUnknown(r, !!avoid),
  goto: (r, cell) => setGoal(r, cell),
  neighborCount: (r) => (r.neighbors || neighbors(r)).length,
  knows: (r, c) => knows(r, c),
};

function spawnFleet() {
  buildEnv(cfg.env);
  coveredMask = new Int32Array(NCELL);
  claims = new Map(); msgs.length = 0;
  metrics = { sent: 0, recv: 0, drop: 0, redundant: 0, dataFound: 0, dataDelivered: 0 };
  t0 = performance.now();
  base = { delivered: new Set() };
  // robots deploy from beside the base station and fan out
  bots = [];
  for (let i = 0; i < cfg.count; i++) {
    let x, y, tries = 0;
    do { x = basePos.x + (rng() - 0.5) * 12; y = basePos.y + 3 + rng() * 6; tries++; }
    while (inObstacle(x, y, 0.6) && tries < 40);
    bots.push({
      id: i, x: clamp(x), y: clamp(y), heading: Math.PI / 2,
      color: PALETTE[i % PALETTE.length], speed: 3.0 + rng() * 0.6,
      goal: null, gcell: -1,
      seen: new Set(), heard: new Set(), order: [], fresh: [],
      everSeen: new Set(), everKnown: new Set(),   // lifetime record (never evicted)
      inbox: [], budget: cfg.bw, sent: 0, isLeader: false, assigned: -1,
      payload: new Set(),       // data points carried but not yet at base
    });
  }
  // scatter data points on open ground
  targets = [];
  for (let i = 0; i < cfg.targets; i++) {
    let x, y, tries = 0;
    do { x = 4 + rng() * (WORLD - 8); y = WORLD * 0.25 + rng() * (WORLD * 0.7); tries++; }
    while (inObstacle(x, y, 1) && tries < 60);
    targets.push({ id: i, x, y, found: false });
  }
  sense(0);
}
const clamp = (v) => Math.max(0.5, Math.min(WORLD - 0.5, v));
const knows = (r, c) => r.seen.has(c) || r.heard.has(c);

/* ── sensing: map cells, discover data points, track overlap + memory ─── */
function sense() {
  for (const r of bots) {
    const rc2 = SENSE * SENSE;
    const gx0 = Math.max(0, (r.x - SENSE) / CELL | 0), gx1 = Math.min(CG - 1, (r.x + SENSE) / CELL | 0);
    const gy0 = Math.max(0, (r.y - SENSE) / CELL | 0), gy1 = Math.min(CG - 1, (r.y + SENSE) / CELL | 0);
    for (let gy = gy0; gy <= gy1; gy++) for (let gx = gx0; gx <= gx1; gx++) {
      const c = gy * CG + gx; if (!coverable[c]) continue;
      if ((cx(c) - r.x) ** 2 + (cy(c) - r.y) ** 2 > rc2) continue;
      const bit = 1 << r.id;
      if (!(coveredMask[c] & bit)) {
        if (coveredMask[c] !== 0) metrics.redundant++;
        coveredMask[c] |= bit;
      }
      remember(r, c, "seen");
    }
    // discover data points in sensing range
    for (const tg of targets) {
      if ((tg.x - r.x) ** 2 + (tg.y - r.y) ** 2 <= rc2) {
        if (!tg.found) { tg.found = true; metrics.dataFound++; }
        if (cfg.base && !base.delivered.has(tg.id)) r.payload.add(tg.id);
      }
    }
  }
}
function remember(r, c, via) {
  r.everKnown.add(c);                            // lifetime record, never forgotten
  if (via === "seen") r.everSeen.add(c);
  if (r.seen.has(c)) return;
  if (via === "seen") { r.seen.add(c); r.heard.delete(c); r.fresh.push(c); }
  else if (!r.heard.has(c)) r.heard.add(c);
  else return;
  r.order.push(c);
  while (r.order.length > cfg.mem) {            // finite memory forgets ground
    const old = r.order.shift(); r.seen.delete(old); r.heard.delete(old);
  }
}

/* ── radio ───────────────────────────────────────────────────────────── */
function neighbors(r) {
  const out = [];
  for (const o of bots) {
    if (o === r) continue;
    const d = Math.hypot(o.x - r.x, o.y - r.y);
    if (d <= cfg.range && los(r.x, r.y, o.x, o.y)) out.push({ o, d });
  }
  return out;
}
const KCOLOR = { cells: "#00d47e", claim: "#e0a030", request: "#40a0e0", assign: "#40a0e0", data: "#7ad07a" };
function visMsg(ax, ay, bx, by, kind, dropped) {
  if (msgs.length < 240) msgs.push({ ax, ay, bx, by, t: 0, life: 0.3,
    color: KCOLOR[kind] || "#9fb0bd", dropped });
}
function deliver(from, to, kind, data, d) {
  const p = (cfg.loss / 100) * (1 - (d / cfg.range) ** 2);
  const ok = rng() < p;
  visMsg(from.x, from.y, to.x ?? basePos.x, to.y ?? basePos.y, kind, !ok);
  if (!ok) { metrics.drop++; return false; }
  if (to.inbox && to.inbox.length >= cfg.buf) { metrics.drop++; return false; }
  if (to.inbox) to.inbox.push({ kind, ...data, from: from.id, d });
  metrics.recv++; return true;
}
function broadcast(r, kind, data) {
  if (r.budget < 1) return false;
  r.budget -= 1; r.sent++; metrics.sent++;
  for (const { o, d } of r.neighbors) deliver(r, o, kind, data, d);
  return true;
}
function unicast(r, to, d, kind, data) {
  if (r.budget < 1) return false;
  r.budget -= 1; r.sent++; metrics.sent++;
  return deliver(r, to, kind, data, d);
}
function recv(r) {
  const proc = Math.max(2, Math.ceil(cfg.bw));
  return r.inbox.splice(0, proc);
}

/* ── goal selection ──────────────────────────────────────────────────── */
function nearestUnknown(r, avoidClaims) {
  let best = -1, bd = 1e9;
  for (let c = 0; c < NCELL; c++) {
    if (!coverable[c] || knows(r, c)) continue;
    const d = Math.hypot(cx(c) - r.x, cy(c) - r.y);
    if (avoidClaims) { const cl = claims.get(c); if (cl && cl.by !== r.id && cl.dist <= d) continue; }
    if (d < bd) { bd = d; best = c; }
  }
  if (best < 0 && r.order.length) best = r.order[(rng() * r.order.length) | 0];
  return best;
}
function setGoal(r, c) {
  if (c < 0) { r.goal = null; r.gcell = -1; return; }
  r.gcell = c; r.goal = { x: cx(c), y: cy(c) };
}
function arrived(r) { return !r.goal || Math.hypot(r.goal.x - r.x, r.goal.y - r.y) < CELL * 0.6; }

/* ── per-strategy thinking ───────────────────────────────────────────── */
function think() {
  if (cfg.strat === "custom") {
    if (customFn) for (const r of bots) {
      r.neighbors = neighbors(r);
      try { customFn(r, H); }
      catch (e) { paused = true; $("custom-stat").textContent = "runtime error: " + e.message;
        $("custom-stat").className = "err"; $("btn-play").textContent = "▶ RESUME"; break; }
    }
    if (cfg.base) relayData();
    return;
  }
  if (cfg.strat === "leader") electLeaders();
  for (const r of bots) {
    r.neighbors = neighbors(r);
    if (cfg.strat === "independent") {
      if (arrived(r)) setGoal(r, nearestUnknown(r, false));
    } else if (cfg.strat === "gossip") {
      for (const m of recv(r)) if (m.cells) for (const c of m.cells) remember(r, c, "heard");
      if (r.fresh.length) { broadcast(r, "cells", { cells: r.fresh.slice(0, 12) }); r.fresh = []; }
      if (arrived(r)) setGoal(r, nearestUnknown(r, false));
    } else if (cfg.strat === "auction") {
      for (const m of recv(r)) {
        if (m.kind === "cells") for (const c of m.cells) remember(r, c, "heard");
        if (m.kind === "claim") { const cur = claims.get(m.cell);
          if (!cur || m.dist < cur.dist) claims.set(m.cell, { by: m.by, dist: m.dist }); }
      }
      if (r.fresh.length) { broadcast(r, "cells", { cells: r.fresh.slice(0, 12) }); r.fresh = []; }
      if (arrived(r)) {
        const c = nearestUnknown(r, true);
        if (c >= 0) { const dist = Math.hypot(cx(c) - r.x, cy(c) - r.y);
          claims.set(c, { by: r.id, dist }); broadcast(r, "claim", { cell: c, by: r.id, dist }); }
        setGoal(r, c);
      }
    } else if (cfg.strat === "leader") {
      for (const m of recv(r)) {
        if (m.kind === "cells") for (const c of m.cells) remember(r, c, "heard");
        if (m.kind === "assign") r.assigned = m.cell;
        if (m.kind === "request" && r.isLeader) {
          const c = leaderPick(r, m.from);
          if (c >= 0) claims.set(c, { by: m.from, dist: 0 });
          const tgt = bots[m.from]; if (tgt) unicast(r, tgt, m.d, "assign", { cell: c });
        }
      }
      if (r.fresh.length) { broadcast(r, "cells", { cells: r.fresh.slice(0, 12) }); r.fresh = []; }
      if (arrived(r)) {
        if (r.isLeader) setGoal(r, leaderPick(r, r.id));
        else { const leader = r.neighbors.find((nb) => nb.o.isLeader);
          if (leader) { unicast(r, leader.o, leader.d, "request", {});
            setGoal(r, r.assigned >= 0 && !knows(r, r.assigned) ? r.assigned : nearestUnknown(r, true)); }
          else setGoal(r, nearestUnknown(r, false)); }
      }
    }
  }
  if (cfg.base) relayData();
}
function electLeaders() {
  const parent = bots.map((_, i) => i);
  const find = (a) => { while (parent[a] !== a) { parent[a] = parent[parent[a]]; a = parent[a]; } return a; };
  for (const r of bots) for (const o of bots) {
    if (o.id <= r.id) continue;
    if (Math.hypot(o.x - r.x, o.y - r.y) <= cfg.range && los(r.x, r.y, o.x, o.y)) {
      const ra = find(r.id), rb = find(o.id); if (ra !== rb) parent[Math.max(ra, rb)] = Math.min(ra, rb);
    }
  }
  const leaderOf = bots.map((_, i) => find(i));
  for (const r of bots) r.isLeader = leaderOf[r.id] === r.id;
}
function leaderPick(leader, forId) {
  const tgt = bots[forId] || leader; let best = -1, bd = 1e9;
  for (let c = 0; c < NCELL; c++) {
    if (!coverable[c] || leader.seen.has(c) || leader.heard.has(c)) continue;
    const cl = claims.get(c); if (cl && cl.by !== forId) continue;
    const d = Math.hypot(cx(c) - tgt.x, cy(c) - tgt.y);
    if (d < bd) { bd = d; best = c; }
  }
  return best;
}

/* ── data relay: greedy geographic routing toward the base station ────── */
function relayData() {
  const dB = (n) => Math.hypot(n.x - basePos.x, n.y - basePos.y);
  for (const r of bots) {
    if (!r.payload.size || r.budget < 1) continue;
    const myD = dB(r);
    if (myD <= cfg.range && los(r.x, r.y, basePos.x, basePos.y)) {
      // base in reach: upload everything
      r.budget -= 1; r.sent++; metrics.sent++;
      visMsg(r.x, r.y, basePos.x, basePos.y, "data", false);
      for (const id of r.payload) if (!base.delivered.has(id)) { base.delivered.add(id); metrics.dataDelivered++; }
      r.payload.clear();
      continue;
    }
    // else hand off to the in-range neighbour strictly closer to base
    let best = null, bestD = myD;
    for (const { o } of (r.neighbors || neighbors(r))) { const od = dB(o); if (od < bestD) { bestD = od; best = o; } }
    if (best) {
      r.budget -= 1; r.sent++; metrics.sent++;
      visMsg(r.x, r.y, best.x, best.y, "data", false);
      for (const id of r.payload) best.payload.add(id);
      r.payload.clear();                       // custody transfer downhill
    }
    // else: carry it like a data mule until a downhill neighbour appears
  }
}

/* ── motion with obstacle avoidance ──────────────────────────────────── */
function move(dt) {
  for (const r of bots) {
    if (!r.goal) continue;
    const dx = r.goal.x - r.x, dy = r.goal.y - r.y, d = Math.hypot(dx, dy);
    if (d < 1e-3) continue;
    let want = Math.atan2(dy, dx);
    // steer around obstacles: if the path just ahead is blocked, veer
    const probe = 2.2;
    if (inObstacle(r.x + Math.cos(want) * probe, r.y + Math.sin(want) * probe, 0.3)) {
      const left = !inObstacle(r.x + Math.cos(want - 0.9) * probe, r.y + Math.sin(want - 0.9) * probe, 0.3);
      want += left ? -0.9 : 0.9;
    }
    let dh = (want - r.heading + Math.PI) % (2 * Math.PI) - Math.PI;
    r.heading += Math.max(-3 * dt, Math.min(3 * dt, dh));
    const step = Math.min(d, r.speed * dt * Math.max(0.2, Math.cos(dh)));
    let nx = r.x + Math.cos(r.heading) * step, ny = r.y + Math.sin(r.heading) * step;
    if (inObstacle(nx, ny, 0.3)) {               // slide along the wall it hit
      if (!inObstacle(nx, r.y, 0.3)) ny = r.y;
      else if (!inObstacle(r.x, ny, 0.3)) nx = r.x;
      else { nx = r.x; ny = r.y; r.goal = null; }   // boxed in — retask
    }
    r.x = clamp(nx); r.y = clamp(ny);
  }
}

/* ── one tick ────────────────────────────────────────────────────────── */
function step(dt) {
  for (const r of bots) r.budget = Math.min(cfg.bw, r.budget + cfg.bw * dt);
  think(); move(dt); sense();
  for (let i = msgs.length - 1; i >= 0; i--) { msgs[i].t += dt; if (msgs[i].t >= msgs[i].life) msgs.splice(i, 1); }
}

/* ── view transform ──────────────────────────────────────────────────── */
const dpr = () => Math.min(2, devicePixelRatio || 1);
let VIEW = { s: 1, ox: 0, oy: 0 };
function resize() {
  const k = dpr();
  cv.width = innerWidth * k; cv.height = innerHeight * k;
  cv.style.width = innerWidth + "px"; cv.style.height = innerHeight + "px";
  const leftPad = innerWidth > 1000 ? 310 : 254, rightPad = innerWidth > 1000 ? 368 : 274;
  const availW = Math.max(120, innerWidth - leftPad - rightPad), availH = innerHeight - 80 - 40;
  const s = Math.max(5, Math.min(availW, availH) / WORLD);
  VIEW.s = s * k;
  VIEW.ox = (leftPad + (availW - WORLD * s) / 2) * k;
  VIEW.oy = (80 + (availH - WORLD * s) / 2) * k;
}
addEventListener("resize", resize);
const wx = (x) => VIEW.ox + x * VIEW.s, wy = (y) => VIEW.oy + y * VIEW.s;
const screenToWorld = (clientX, clientY) => {
  const k = dpr();
  return { x: (clientX * k - VIEW.ox) / VIEW.s, y: (clientY * k - VIEW.oy) / VIEW.s };
};

/* ── rendering ───────────────────────────────────────────────────────── */
function draw() {
  ctx.clearRect(0, 0, cv.width, cv.height);
  const s = VIEW.s, k = dpr();
  ctx.fillStyle = "#0c1116"; ctx.fillRect(wx(0), wy(0), WORLD * s, WORLD * s);

  // coverage heat
  for (let c = 0; c < NCELL; c++) {
    const m = coveredMask[c]; if (!m) continue;
    let bits = 0, v = m; while (v) { bits += v & 1; v >>= 1; }
    ctx.fillStyle = bits > 1 ? "rgba(224,160,48,.17)" : "rgba(0,212,126,.10)";
    ctx.fillRect(wx(cx(c) - CELL / 2), wy(cy(c) - CELL / 2), CELL * s + 1, CELL * s + 1);
  }
  // hovered robot's knowledge, three tiers: faint = it once knew but has since
  // forgotten (memory full); filled = sensed firsthand & still in memory;
  // outline = heard from a peer & still in memory.
  if (hoverBot) {
    for (const c of hoverBot.everKnown) {            // forgotten footprint
      if (hoverBot.seen.has(c) || hoverBot.heard.has(c)) continue;
      ctx.fillStyle = hoverBot.color + "14";
      ctx.fillRect(wx(cx(c) - CELL / 2), wy(cy(c) - CELL / 2), CELL * s + 1, CELL * s + 1);
    }
    for (const c of hoverBot.seen) { ctx.fillStyle = hoverBot.color + "44";
      ctx.fillRect(wx(cx(c) - CELL / 2), wy(cy(c) - CELL / 2), CELL * s + 1, CELL * s + 1); }
    ctx.strokeStyle = hoverBot.color + "88"; ctx.lineWidth = 1;
    for (const c of hoverBot.heard) ctx.strokeRect(wx(cx(c) - CELL / 2) + 1, wy(cy(c) - CELL / 2) + 1, CELL * s - 1, CELL * s - 1);
  }

  // grid + obstacles + border
  ctx.strokeStyle = "rgba(31,42,51,.5)"; ctx.lineWidth = 1; ctx.beginPath();
  for (let i = 0; i <= CG; i += 2) { ctx.moveTo(wx(i * CELL), wy(0)); ctx.lineTo(wx(i * CELL), wy(WORLD));
    ctx.moveTo(wx(0), wy(i * CELL)); ctx.lineTo(wx(WORLD), wy(i * CELL)); }
  ctx.stroke();
  for (const [ox, oy, ow, oh] of obstacles) {
    ctx.fillStyle = "#2e3c4a"; ctx.fillRect(wx(ox), wy(oy), ow * s, oh * s);
    ctx.strokeStyle = "#3e4e5e"; ctx.lineWidth = 1; ctx.strokeRect(wx(ox), wy(oy), ow * s, oh * s);
  }
  ctx.strokeStyle = "rgba(42,58,70,.9)"; ctx.lineWidth = 1.5; ctx.strokeRect(wx(0), wy(0), WORLD * s, WORLD * s);

  // radio links
  if (cfg.strat !== "independent") {
    for (let i = 0; i < bots.length; i++) for (let j = i + 1; j < bots.length; j++) {
      const a = bots[i], b = bots[j], d = Math.hypot(a.x - b.x, a.y - b.y);
      if (d > cfg.range || !los(a.x, a.y, b.x, b.y)) continue;
      const edge = d / cfg.range;
      ctx.strokeStyle = edge > 0.8 ? "rgba(224,160,48,.32)" : `rgba(0,212,126,${0.26 * (1 - edge) + 0.06})`;
      ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(wx(a.x), wy(a.y)); ctx.lineTo(wx(b.x), wy(b.y)); ctx.stroke();
    }
  }

  // base station coverage ring + data points
  if (cfg.base) drawBase(s, k);
  drawTargets(s, k);

  // goal threads + sensing rings
  for (const r of bots) {
    if (r.goal) { ctx.strokeStyle = r.color + "33"; ctx.lineWidth = 1; ctx.setLineDash([3, 4]);
      ctx.beginPath(); ctx.moveTo(wx(r.x), wy(r.y)); ctx.lineTo(wx(r.goal.x), wy(r.goal.y)); ctx.stroke(); ctx.setLineDash([]); }
    ctx.strokeStyle = r.color + (hoverBot === r ? "66" : "22"); ctx.lineWidth = hoverBot === r ? 1.5 : 1;
    ctx.beginPath(); ctx.arc(wx(r.x), wy(r.y), SENSE * s, 0, 7); ctx.stroke();
  }
  for (const r of bots) drawBot(r, s, k);

  // messages in flight
  for (const m of msgs) {
    const f = m.t / m.life, x = m.ax + (m.bx - m.ax) * f, y = m.ay + (m.by - m.ay) * f;
    if (m.dropped) { ctx.fillStyle = "#d84a4a"; ctx.globalAlpha = 1 - f;
      ctx.font = `${Math.round(11 * k)}px ui-monospace`; ctx.fillText("✕", wx(m.ax) - 4, wy(m.ay) - 6 - f * 12 * k); ctx.globalAlpha = 1; }
    else { ctx.fillStyle = m.color; ctx.globalAlpha = 0.9 * (1 - f * 0.5);
      ctx.beginPath(); ctx.arc(wx(x), wy(y), 2.6 * k, 0, 7); ctx.fill(); ctx.globalAlpha = 1; }
  }
}
function drawBase(s, k) {
  const x = wx(basePos.x), y = wy(basePos.y);
  ctx.strokeStyle = "rgba(0,212,126,.18)"; ctx.lineWidth = 1; ctx.setLineDash([2, 5]);
  ctx.beginPath(); ctx.arc(x, y, cfg.range * s, 0, 7); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = "#00d47e"; const r = 6 * k;
  ctx.beginPath(); ctx.moveTo(x, y - r * 1.4); ctx.lineTo(x + r, y + r); ctx.lineTo(x - r, y + r); ctx.closePath(); ctx.fill();
  ctx.fillStyle = "#06080a"; ctx.fillRect(x - r * 0.3, y - r * 0.2, r * 0.6, r * 0.7);
  ctx.fillStyle = "rgba(0,212,126,.85)"; ctx.font = `${Math.round(9 * k)}px ui-monospace`;
  ctx.textAlign = "center"; ctx.fillText("BASE", x, y + r * 2.6); ctx.textAlign = "left";
}
function drawTargets(s, k) {
  for (const tg of targets) {
    const x = wx(tg.x), y = wy(tg.y), done = base && base.delivered.has(tg.id);
    if (done) { ctx.fillStyle = "#00d47e"; ctx.beginPath(); ctx.arc(x, y, 3.5 * k, 0, 7); ctx.fill();
      ctx.strokeStyle = "rgba(0,212,126,.35)"; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(x, y, 6 * k, 0, 7); ctx.stroke(); }
    else if (tg.found) { ctx.fillStyle = "#e0a030"; ctx.beginPath(); ctx.arc(x, y, 3.5 * k, 0, 7); ctx.fill(); }
    else { ctx.strokeStyle = "#5a6b78"; ctx.lineWidth = 1.2 * k; ctx.beginPath(); ctx.arc(x, y, 3.5 * k, 0, 7); ctx.stroke();
      ctx.fillStyle = "rgba(90,107,120,.25)"; ctx.fill(); }
  }
}
function drawBot(r, s, k) {
  const x = wx(r.x), y = wy(r.y), sz = 6.5 * k;
  ctx.save(); ctx.translate(x, y); ctx.rotate(r.heading);
  ctx.fillStyle = r.color; ctx.beginPath();
  ctx.moveTo(sz * 1.3, 0); ctx.lineTo(-sz * 0.8, sz * 0.8); ctx.lineTo(-sz * 0.4, 0); ctx.lineTo(-sz * 0.8, -sz * 0.8);
  ctx.closePath(); ctx.fill(); ctx.restore();
  if (hoverBot === r) { ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.5 * k; ctx.beginPath(); ctx.arc(x, y, sz * 1.5, 0, 7); ctx.stroke(); }
  if (r.isLeader && cfg.strat === "leader") { ctx.strokeStyle = "#ffd24a"; ctx.lineWidth = 1.5 * k;
    ctx.beginPath(); ctx.arc(x, y, sz * 1.8, 0, 7); ctx.stroke(); }
  if (r.payload.size) { ctx.fillStyle = "#7ad07a"; ctx.beginPath(); ctx.arc(x + sz, y - sz, 2.4 * k, 0, 7); ctx.fill(); }
  const fill = r.inbox.length / cfg.buf;
  if (fill > 0.01) { ctx.fillStyle = fill > 0.9 ? "#d84a4a" : "#e0a030";
    ctx.fillRect(x - sz, y - sz * 2.2, sz * 2 * Math.min(1, fill), 2 * k); }
}

/* ── hover inspector ─────────────────────────────────────────────────── */
function updateInspector() {
  const el = $("inspect");
  if (!hoverBot || !hoverXY) { el.classList.remove("on"); return; }
  const r = hoverBot, nb = (r.neighbors || neighbors(r)).length;
  const mapped = r.seen.size, heard = r.heard.size, inMem = mapped + heard;
  const everKnown = r.everKnown.size, forgotten = Math.max(0, everKnown - inMem);
  const pct = Math.round(inMem / Math.max(1, NCOVER) * 100);
  const note = nb ? `linked to ${nb} peer${nb > 1 ? "s" : ""} — sharing what it sees`
                  : "out of radio range — searching solo, no one to tell";
  el.innerHTML =
    `<div class="ih"><span class="dot" style="background:${r.color}"></span>` +
    `<span class="nm">robot ${r.id}</span>` +
    `${r.isLeader && cfg.strat === "leader" ? '<span class="role">LEADER</span>' : ''}</div>` +
    `<div class="ihd">IN MEMORY NOW · ${inMem}/${cfg.mem} cells</div>` +
    `<div class="irow"><span class="k">sensed firsthand</span><span class="v seen">${mapped}</span></div>` +
    `<div class="irow"><span class="k">heard from peers</span><span class="v heard">${heard}</span></div>` +
    `<div class="ihd">EVER KNOWN · its whole history</div>` +
    `<div class="irow"><span class="k">cells discovered</span><span class="v">${everKnown}</span></div>` +
    `<div class="irow"><span class="k">forgotten (memory full)</span><span class="v${forgotten ? " warn" : ""}">${forgotten}</span></div>` +
    `<div class="ihd">RIGHT NOW</div>` +
    `<div class="irow"><span class="k">inbox</span><span class="v">${r.inbox.length} / ${cfg.buf}</span></div>` +
    `<div class="irow"><span class="k">messages sent</span><span class="v">${r.sent}</span></div>` +
    (cfg.base ? `<div class="irow"><span class="k">data carried</span><span class="v">${r.payload.size}</span></div>` : "") +
    `<div class="note">${note}. <b style="color:${r.color}">Filled</b> = sensed firsthand, <b>outlined</b> = heard over radio, <b style="opacity:.5">faint</b> = once knew but forgot.</div>`;
  el.classList.add("on");
  const w = 232, pad = 14;
  let lx = hoverXY.x + 18, ly = hoverXY.y + 14;
  if (lx + w > innerWidth - pad) lx = hoverXY.x - w - 18;
  if (ly + 200 > innerHeight - pad) ly = innerHeight - 210;
  el.style.left = Math.max(pad, lx) + "px"; el.style.top = Math.max(70, ly) + "px";
}

/* ── HUD ─────────────────────────────────────────────────────────────── */
let tickerDone = "";
function setV(id, v, cls) { const e = $(id); e.textContent = v; e.className = "v" + (cls ? " " + cls : ""); }
function updateHUD() {
  let covered = 0; for (let c = 0; c < NCELL; c++) if (coveredMask[c]) covered++;
  const pct = Math.round(covered / Math.max(1, NCOVER) * 100);
  $("m-cover").textContent = pct + "%"; $("cover-fill").style.width = Math.min(100, pct) + "%";
  setV("m-redundant", metrics.redundant, metrics.redundant > covered * 0.6 ? "bad" : metrics.redundant > covered * 0.3 ? "warn" : "");
  let linked = 0; for (const r of bots) if (neighbors(r).length) linked++;
  $("m-linked").textContent = `${linked}/${bots.length}`;
  $("m-recv").textContent = metrics.recv;
  const lossPct = Math.round(metrics.drop / Math.max(1, metrics.drop + metrics.recv) * 100);
  setV("m-drop", `${metrics.drop} · ${lossPct}%`, lossPct > 45 ? "bad" : lossPct > 25 ? "warn" : "");
  const dataTotal = targets.length;
  const dataNum = cfg.base ? metrics.dataDelivered : metrics.dataFound;
  setV("m-data", `${dataNum}/${dataTotal}`, "");
  $("m-time").textContent = ((performance.now() - t0) / 1000).toFixed(1) + "s";
  if (pct >= 99 && !tickerDone) {
    tickerDone = `field mapped in ${((performance.now() - t0) / 1000).toFixed(1)}s · ` +
      `${metrics.redundant} overlapping re-walks · ${metrics.drop} messages dropped` +
      (cfg.base ? ` · ${metrics.dataDelivered}/${dataTotal} data home` : "");
    $("ticker").textContent = tickerDone;
  }
}

/* ── loop ────────────────────────────────────────────────────────────── */
function loop(now) {
  const dt = Math.min(0.05, (now - lastT) / 1000 || 0.016); lastT = now;
  if (!paused) { step(dt); updateHUD(); }
  draw(); updateInspector();
  requestAnimationFrame(loop);
}

/* ── teaching panel ──────────────────────────────────────────────────── */
const HL_KW = new Set(["def", "for", "in", "if", "elif", "else", "return", "from", "import",
  "None", "True", "False", "and", "or", "not", "while", "is", "with", "as"]);
const hlEsc = (s) => s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
function highlight(src) {
  let out = "", i = 0; const n = src.length;
  while (i < n) { const c = src[i];
    if (c === "#") { let j = i; while (j < n && src[j] !== "\n") j++; out += `<span class="tk-com">${hlEsc(src.slice(i, j))}</span>`; i = j; continue; }
    if (c === '"' || c === "'") { const q = c; let j = i + 1; while (j < n && src[j] !== q && src[j] !== "\n") { if (src[j] === "\\") j++; j++; }
      out += `<span class="tk-str">${hlEsc(src.slice(i, j + 1))}</span>`; i = j + 1; continue; }
    if (c === "@") { let j = i + 1; while (j < n && /[\w.]/.test(src[j])) j++; out += `<span class="tk-dec">${hlEsc(src.slice(i, j))}</span>`; i = j; continue; }
    if (/[A-Za-z_]/.test(c)) { let j = i; while (j < n && /\w/.test(src[j])) j++; const w = src.slice(i, j), prev = src.slice(0, i).trimEnd();
      const cls = HL_KW.has(w) ? "tk-kw" : prev.endsWith("def") ? "tk-def" : null; out += cls ? `<span class="${cls}">${w}</span>` : hlEsc(w); i = j; continue; }
    if (/[0-9]/.test(c)) { let j = i; while (j < n && /[\d.]/.test(src[j])) j++; out += `<span class="tk-num">${hlEsc(src.slice(i, j))}</span>`; i = j; continue; }
    out += hlEsc(c); i++; }
  return out;
}
function showStrategyDoc() {
  const custom = cfg.strat === "custom";
  $("builtin-ui").style.display = custom ? "none" : "";
  $("custom-ui").style.display = custom ? "" : "none";
  if (custom) {
    if (!$("custom-code").value.trim()) $("custom-code").value = CUSTOM_TEMPLATE;
    $("custom-api").textContent = CUSTOM_API;
    return;
  }
  const s = STRATS.find((x) => x.id === cfg.strat) || STRATS[1];
  $("s-name").textContent = s.nm; $("s-prose").textContent = s.prose;
  $("s-code").firstChild.innerHTML = highlight(CODE[cfg.strat]);
}
/* compile + run whatever is in the editor */
function runCustom() {
  const body = $("custom-code").value;
  const stat = $("custom-stat");
  try { customFn = new Function("r", "H", body); }
  catch (e) { customFn = null; stat.textContent = "compile error: " + e.message; stat.className = "err"; return; }
  cfg.strat = "custom";
  for (const c of $("strats").children) c.classList.toggle("on", c.dataset.strat === "custom");
  paused = false; $("btn-play").textContent = "⏸ PAUSE"; $("btn-play").classList.remove("hot");
  restart();
  stat.textContent = "running your algorithm"; stat.className = "ok";
}
/* ask the local runtime's LLM to draft a policy from the goal box */
async function generateCustom() {
  const goal = $("custom-goal").value.trim();
  const stat = $("custom-stat"), btn = $("custom-llm");
  if (!goal) { stat.textContent = "describe what you want first"; stat.className = "err"; return; }
  stat.textContent = "asking your local LLM…"; stat.className = ""; btn.disabled = true;
  try {
    const res = await fetch("/api/fleet/strategy", { method: "POST",
      headers: { "Content-Type": "application/json" }, body: JSON.stringify({ goal }) });
    const j = await res.json();
    if (!res.ok || !j.ok) throw new Error(j.error || res.statusText);
    $("custom-code").value = j.code;
    stat.textContent = `drafted by ${j.model || "the LLM"} — review it, then ▶ Run`; stat.className = "ok";
  } catch (e) {
    stat.textContent = "no LLM reachable — run `roborun` locally with an API key, or write it by hand";
    stat.className = "err";
  } finally { btn.disabled = false; }
}

/* ── wiring ──────────────────────────────────────────────────────────── */
function syncLabels() {
  $("v-count").textContent = cfg.count; $("v-range").textContent = cfg.range + " m";
  $("v-loss").textContent = cfg.loss + "%"; $("v-bw").textContent = cfg.bw + " / s";
  $("v-mem").textContent = cfg.mem; $("v-buf").textContent = cfg.buf; $("v-targets").textContent = cfg.targets;
}
function bindSlider(id, key, fmt, respawn) {
  const el = $(id); el.value = cfg[key];
  el.addEventListener("input", () => { cfg[key] = +el.value; syncLabels(); if (respawn) restart(); });
}
function restart() { spawnFleet(); tickerDone = ""; $("ticker").textContent =
  `running · ${(STRATS.find((s) => s.id === cfg.strat) || STRATS[1]).nm.toLowerCase()} · ${cfg.env}`; }

function buildStrats() {
  const host = $("strats"); host.innerHTML = "";
  for (const s of STRATS) {
    const el = document.createElement("button");
    el.className = "strat" + (s.id === cfg.strat ? " on" : "");
    el.dataset.strat = s.id;
    el.innerHTML = `<div class="nm">${s.nm}</div><div class="ds">${s.ds}</div>`;
    el.addEventListener("click", () => { cfg.strat = s.id;
      for (const c of host.children) c.classList.remove("on"); el.classList.add("on");
      showStrategyDoc(); switchPane("strategy");
      // custom waits for ▶ Run (so an empty/broken editor doesn't run); others restart now
      if (s.id === "custom") { if (customFn) restart(); else { $("ticker").textContent = "write or generate an algorithm, then ▶ Run"; } }
      else restart();
    });
    host.appendChild(el);
  }
}
function switchPane(name) {
  for (const t of document.querySelectorAll(".tab")) t.classList.toggle("on", t.dataset.pane === name);
  for (const p of document.querySelectorAll(".pane")) p.classList.toggle("on", p.id === "pane-" + name);
}
for (const t of document.querySelectorAll(".tab")) t.addEventListener("click", () => switchPane(t.dataset.pane));
$("teach-exp").addEventListener("click", () => {
  const t = $("teach"), on = t.classList.toggle("expanded"); $("scrim").classList.toggle("on", on);
  $("teach-exp").textContent = on ? "⤡" : "⤢";
});
$("scrim").addEventListener("click", () => { $("teach").classList.remove("expanded"); $("scrim").classList.remove("on"); $("teach-exp").textContent = "⤢"; });
$("btn-learn").addEventListener("click", () => { switchPane("concepts");
  if (!$("teach").classList.contains("expanded")) $("teach-exp").click(); });

$("btn-play").addEventListener("click", () => { paused = !paused;
  $("btn-play").textContent = paused ? "▶ RESUME" : "⏸ PAUSE"; $("btn-play").classList.toggle("hot", paused); });
$("btn-reset").addEventListener("click", restart);
$("custom-run").addEventListener("click", runCustom);
$("custom-llm").addEventListener("click", generateCustom);
$("custom-goal").addEventListener("keydown", (e) => { if (e.key === "Enter") generateCustom(); });

$("s-env").value = cfg.env;
$("s-env").addEventListener("change", () => { cfg.env = $("s-env").value; restart(); });
$("s-base").checked = cfg.base;
$("s-base").addEventListener("change", () => { cfg.base = $("s-base").checked; restart(); });
bindSlider("s-count", "count", null, true);
bindSlider("s-targets", "targets", null, true);
bindSlider("s-range", "range");
bindSlider("s-loss", "loss");
bindSlider("s-bw", "bw");
bindSlider("s-mem", "mem");
bindSlider("s-buf", "buf");

// hover the field to inspect a robot
cv.addEventListener("pointermove", (e) => {
  hoverXY = { x: e.clientX, y: e.clientY };
  const w = screenToWorld(e.clientX, e.clientY);
  let best = null, bd = 2.6;
  for (const r of bots) { const d = Math.hypot(r.x - w.x, r.y - w.y); if (d < bd) { bd = d; best = r; } }
  hoverBot = best; cv.style.cursor = best ? "pointer" : "default";
});
cv.addEventListener("pointerleave", () => { hoverBot = null; hoverXY = null; });

syncLabels(); buildStrats(); showStrategyDoc(); resize(); spawnFleet();
$("ticker").textContent = `running · ${(STRATS.find((s) => s.id === cfg.strat) || STRATS[1]).nm.toLowerCase()} · ${cfg.env}`;
requestAnimationFrame(loop);
