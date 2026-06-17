/* RoboRun · Fleet Sim (platform spec 04) — N robots in ONE large multi-floor
 * warehouse, navigating across floors via elevators, collecting detections
 * jointly into the active project's environment. 2D top-down, one shared clock.
 */
(() => {
  const $ = (s) => document.querySelector(s);
  const COLORS = ["#00d47e", "#4090e0", "#d4a030", "#e0563f", "#a070e0",
                  "#40c0c0", "#e090c0", "#90c040"];
  const S = { world: null, robots: [], playing: true, speed: 1.0,
              n: 8, floors: 3, t0: performance.now(), detections: 0,
              detectedKeys: new Set(), canvases: [] };

  async function fetchWorld() {
    const d = await (await fetch(`/api/worlds/warehouse?floors=${S.floors}&rooms=6&seed=1`)).json();
    S.world = d.world; S.totalItems = d.items;
  }

  function roomCenter(room) {
    const r = room.rect; return { x: (r[0] + r[2]) / 2, y: (r[1] + r[3]) / 2 };
  }
  function floorOf(level) { return S.world.floors[level]; }
  function elevatorsOn(level) { return S.world.elevators.filter(e => e.from === level || e.to === level); }

  function pickTarget(rb) {
    const fl = floorOf(rb.floor);
    // ~25% of the time, head to an elevator to change floors
    const elevs = elevatorsOn(rb.floor);
    if (elevs.length && Math.random() < 0.25) {
      const e = elevs[Math.floor(Math.random() * elevs.length)];
      rb.target = { x: e.x, y: e.y, elevator: e };
    } else {
      const room = fl.rooms[Math.floor(Math.random() * fl.rooms.length)];
      const c = roomCenter(room);
      rb.target = { x: c.x + (Math.random() - .5) * 4, y: c.y + (Math.random() - .5) * 4 };
    }
    rb.stuck = 0;
  }

  function spawn() {
    S.robots = []; S.detections = 0; S.detectedKeys = new Set(); S.t0 = performance.now();
    for (let i = 0; i < S.n; i++) {
      const sp = S.world.spawns[i % S.world.spawns.length];
      const rb = { id: "r" + i, x: sp.x + (Math.random() * 6 - 3), y: sp.y + (Math.random() * 6 - 3),
                   h: Math.random() * 6.28, floor: 0, color: COLORS[i % COLORS.length],
                   inElev: 0, seen: 0, stuck: 0, target: null };
      pickTarget(rb); S.robots.push(rb);
    }
  }

  function detect(rb) {
    const items = floorOf(rb.floor).items;
    for (let k = 0; k < items.length; k++) {
      const it = items[k];
      if (Math.hypot(it.x - rb.x, it.y - rb.y) < 3.0) {
        const key = rb.floor + ":" + k;
        if (!S.detectedKeys.has(key)) { S.detectedKeys.add(key); S.detections++; }
        rb.seen++;
      }
    }
  }

  function step(dt) {
    const v = 6.0;                      // m/s nominal
    for (const rb of S.robots) {
      if (rb.inElev > 0) {              // riding the elevator between floors
        rb.inElev -= dt;
        if (rb.inElev <= 0 && rb.pendingFloor != null) {
          rb.floor = rb.pendingFloor; rb.pendingFloor = null; pickTarget(rb);
        }
        continue;
      }
      const dx = rb.target.x - rb.x, dy = rb.target.y - rb.y;
      const dist = Math.hypot(dx, dy);
      if (dist < 1.2) {
        if (rb.target.elevator) {       // step into the shaft → other floor
          const e = rb.target.elevator;
          rb.pendingFloor = (e.from === rb.floor) ? e.to : e.from;
          rb.inElev = 0.7; rb.x = e.x; rb.y = e.y;
        } else { detect(rb); pickTarget(rb); }
        continue;
      }
      rb.h = Math.atan2(dy, dx);
      const nx = rb.x + Math.cos(rb.h) * v * dt;
      const ny = rb.y + Math.sin(rb.h) * v * dt;
      const sz = S.world.size;
      // keep inside the building; re-target if pinned at a wall
      if (nx > 0.5 && nx < sz - 0.5) rb.x = nx; else rb.stuck += dt;
      if (ny > 0.5 && ny < sz - 0.5) rb.y = ny; else rb.stuck += dt;
      detect(rb);
      if ((rb.stuck += dt * 0.0) > 0 && rb.stuck > 2.5) pickTarget(rb);
    }
  }

  function buildCanvases() {
    const host = $("#floors"); host.innerHTML = ""; S.canvases = [];
    for (const fl of S.world.floors) {
      const div = document.createElement("div"); div.className = "floor";
      div.innerHTML = `<h2><span>Floor ${fl.level}</span><span id="fc${fl.level}" style="color:var(--accent)"></span></h2>`;
      const cv = document.createElement("canvas"); cv.width = 520; cv.height = 520;
      div.appendChild(cv); host.appendChild(div); S.canvases[fl.level] = cv;
    }
  }

  function draw() {
    const sz = S.world.size;
    for (const fl of S.world.floors) {
      const cv = S.canvases[fl.level]; if (!cv) continue;
      const g = cv.getContext("2d"); const W = cv.width, H = cv.height, P = 10;
      const sc = (W - 2 * P) / sz; const X = x => P + x * sc, Y = y => P + y * sc;
      g.clearRect(0, 0, W, H);
      // rooms
      g.strokeStyle = "rgba(120,175,144,.10)"; g.lineWidth = 1;
      for (const r of fl.rooms) { const rc = r.rect; g.strokeRect(X(rc[0]), Y(rc[1]), (rc[2] - rc[0]) * sc, (rc[3] - rc[1]) * sc); }
      // walls
      g.strokeStyle = "#2c3b2c"; g.lineWidth = 2; g.beginPath();
      for (const w of fl.walls) { g.moveTo(X(w[0]), Y(w[1])); g.lineTo(X(w[2]), Y(w[3])); }
      g.stroke();
      // items (detected ones glow)
      for (let k = 0; k < fl.items.length; k++) {
        const it = fl.items[k], on = S.detectedKeys.has(fl.level + ":" + k);
        g.fillStyle = on ? "rgba(0,212,126,.9)" : "rgba(120,175,144,.35)";
        g.beginPath(); g.arc(X(it.x), Y(it.y), on ? 3.5 : 2.2, 0, 6.28); g.fill();
      }
      // elevators
      for (const e of elevatorsOn(fl.level)) {
        g.strokeStyle = "#4090e0"; g.fillStyle = "rgba(64,144,224,.15)"; g.lineWidth = 1.5;
        const s = 3.0 * sc; g.fillRect(X(e.x) - s / 2, Y(e.y) - s / 2, s, s); g.strokeRect(X(e.x) - s / 2, Y(e.y) - s / 2, s, s);
        g.fillStyle = "#4090e0"; g.font = "9px monospace"; g.fillText("⇅", X(e.x) - 3, Y(e.y) + 3);
      }
      // robots on this floor
      let cnt = 0;
      for (const rb of S.robots) {
        if (rb.floor !== fl.level || rb.inElev > 0) continue; cnt++;
        const x = X(rb.x), y = Y(rb.y);
        g.fillStyle = rb.color; g.beginPath();
        g.moveTo(x + Math.cos(rb.h) * 6, y + Math.sin(rb.h) * 6);
        g.lineTo(x + Math.cos(rb.h + 2.5) * 5, y + Math.sin(rb.h + 2.5) * 5);
        g.lineTo(x + Math.cos(rb.h - 2.5) * 5, y + Math.sin(rb.h - 2.5) * 5);
        g.closePath(); g.fill();
      }
      const fc = document.getElementById("fc" + fl.level); if (fc) fc.textContent = cnt + " bots";
    }
  }

  function hud() {
    const cov = S.totalItems ? Math.round(S.detections / S.totalItems * 100) : 0;
    const el = (performance.now() - S.t0) / 1000;
    const chip = (n, l) => `<span class="kpi" style="padding:5px 10px"><span class="n" style="font-size:15px">${n}</span> <span class="l" style="font-size:10px">${l}</span></span>`;
    $("#kpis").innerHTML = chip(S.robots.length, "robots") + chip(S.detections + "/" + S.totalItems, "found") +
      chip(cov + "%", "coverage") + chip(el.toFixed(0) + "s", "elapsed");
  }

  let last = performance.now();
  function loop(now) {
    const dt = Math.min(0.05, (now - last) / 1000) * S.speed; last = now;
    if (S.playing && S.world) { step(dt); }
    if (S.world) { draw(); hud(); }
    requestAnimationFrame(loop);
  }

  async function reset() {
    await fetchWorld(); buildCanvases(); spawn();
  }

  // controls
  $("#n").oninput = e => { S.n = +e.target.value; $("#nlab").textContent = S.n; spawn(); };
  $("#fl").oninput = e => { S.floors = +e.target.value; $("#flab").textContent = S.floors; reset(); };
  $("#spd").oninput = e => { S.speed = +e.target.value / 10; $("#slab").textContent = S.speed.toFixed(1) + "×"; };
  $("#play").onclick = () => { S.playing = !S.playing; $("#play").textContent = S.playing ? "⏸ pause" : "▶ play"; };
  $("#reset").onclick = reset;

  // show what env we're collecting into
  fetch("/api/projects/active").then(r => r.json()).then(d => {
    if (d.active) $("#scope").innerHTML = `Collecting jointly into <b>${d.active.project} / ${d.active.environment}</b> — ${S.n} robots, one warehouse, ${S.floors} floors connected by elevators. Each robot's detections land in the same environment.`;
  }).catch(() => {});

  reset().then(() => requestAnimationFrame(loop));
})();
