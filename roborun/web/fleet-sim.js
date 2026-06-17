/* RoboRun · Fleet Sim (platform spec 04) — N robots in ONE large multi-floor
 * warehouse, navigating across floors via elevators, collecting detections
 * jointly into the active project's environment. 2D top-down, one shared clock.
 */
(() => {
  const $ = (s) => document.querySelector(s);
  const COLORS = ["#00d47e", "#4090e0", "#d4a030", "#e0563f", "#a070e0",
                  "#40c0c0", "#e090c0", "#90c040"];
  const FONT = "ui-monospace, Menlo, monospace";
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
                   inElev: 0, seen: 0, stuck: 0, target: null, trail: [] };
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
      // breadcrumb trail (capped)
      const tr = rb.trail || (rb.trail = []);
      const lp = tr[tr.length - 1];
      if (!lp || Math.hypot(rb.x - lp.x, rb.y - lp.y) > 1.4) {
        tr.push({ x: rb.x, y: rb.y, f: rb.floor });
        if (tr.length > 18) tr.shift();
      }
      detect(rb);
      if ((rb.stuck += dt * 0.0) > 0 && rb.stuck > 2.5) pickTarget(rb);
    }
  }

  function buildCanvases() {
    const host = $("#floors"); host.innerHTML = ""; S.canvases = [];
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    for (const fl of S.world.floors) {
      const div = document.createElement("div"); div.className = "floor";
      div.innerHTML = `<h2><span>Floor ${fl.level}</span>` +
        `<span class="fc"><span><b id="fc${fl.level}">0</b> bots</span>` +
        `<span><b id="fi${fl.level}">0</b>/${fl.items.length} found</span></span></h2>`;
      const cv = document.createElement("canvas");
      const CSS = 520; cv.style.aspectRatio = "1 / 1";
      cv.width = CSS * dpr; cv.height = CSS * dpr;
      const g = cv.getContext("2d"); g.scale(dpr, dpr); cv._cw = CSS;
      div.appendChild(cv); host.appendChild(div); S.canvases[fl.level] = cv;
    }
  }

  function draw() {
    const sz = S.world.size;
    for (const fl of S.world.floors) {
      const cv = S.canvases[fl.level]; if (!cv) continue;
      const g = cv.getContext("2d"); const W = cv._cw, H = cv._cw, P = 14;
      const sc = (W - 2 * P) / sz; const X = x => P + x * sc, Y = y => P + y * sc;
      g.clearRect(0, 0, W, H);

      // building backdrop
      g.fillStyle = "#0b110d";
      g.fillRect(X(0), Y(0), sz * sc, sz * sc);

      // room fills + soft outlines (alternating tint for legibility)
      fl.rooms.forEach((r, i) => {
        const rc = r.rect, rx = X(rc[0]), ry = Y(rc[1]), rw = (rc[2] - rc[0]) * sc, rh = (rc[3] - rc[1]) * sc;
        g.fillStyle = (i % 2) ? "rgba(120,175,144,.030)" : "rgba(120,175,144,.060)";
        g.fillRect(rx, ry, rw, rh);
        g.strokeStyle = "rgba(120,175,144,.16)"; g.lineWidth = 1;
        g.strokeRect(rx + .5, ry + .5, rw - 1, rh - 1);
      });

      // exterior + interior walls (doorway gaps come from the data)
      g.strokeStyle = "#3a5340"; g.lineWidth = 2.5; g.lineCap = "round";
      g.beginPath();
      for (const w of fl.walls) { g.moveTo(X(w[0]), Y(w[1])); g.lineTo(X(w[2]), Y(w[3])); }
      g.stroke();

      // robot breadcrumb trails (under items/robots)
      for (const rb of S.robots) {
        const tr = rb.trail; if (!tr || tr.length < 2 || rb.floor !== fl.level) continue;
        g.strokeStyle = rb.color + "33"; g.lineWidth = 2; g.lineCap = "round"; g.beginPath();
        let started = false;
        for (const p of tr) {
          if (p.f !== fl.level) { started = false; continue; }
          if (!started) { g.moveTo(X(p.x), Y(p.y)); started = true; } else g.lineTo(X(p.x), Y(p.y));
        }
        g.stroke();
      }

      // items: detected glow green, undetected muted; label detected on hover-scale
      g.textAlign = "center"; g.textBaseline = "middle";
      for (let k = 0; k < fl.items.length; k++) {
        const it = fl.items[k], on = S.detectedKeys.has(fl.level + ":" + k);
        const px = X(it.x), py = Y(it.y);
        if (on) {
          g.fillStyle = "rgba(0,212,126,.22)"; g.beginPath(); g.arc(px, py, 8, 0, 6.28); g.fill();
          g.fillStyle = "#00d47e"; g.beginPath(); g.arc(px, py, 3.6, 0, 6.28); g.fill();
          g.fillStyle = "rgba(166,188,173,.7)"; g.font = "8px " + FONT;
          g.fillText(it.label, px, py - 11);
        } else {
          g.fillStyle = "rgba(120,175,144,.32)"; g.strokeStyle = "rgba(120,175,144,.5)";
          g.lineWidth = 1; g.beginPath(); g.arc(px, py, 2.6, 0, 6.28); g.fill(); g.stroke();
        }
      }

      // elevators: clearly boxed + labeled ⇅
      for (const e of elevatorsOn(fl.level)) {
        const s = Math.max(16, 3.0 * sc), ex = X(e.x), ey = Y(e.y);
        g.fillStyle = "rgba(64,144,224,.18)"; g.strokeStyle = "#4090e0"; g.lineWidth = 1.5;
        roundRect(g, ex - s / 2, ey - s / 2, s, s, 3); g.fill(); g.stroke();
        g.fillStyle = "#7fb6ec"; g.font = "bold 11px " + FONT;
        g.fillText("⇅", ex, ey + .5);
      }

      // robots on this floor — larger, outlined triangles in their unit color
      let cnt = 0;
      for (const rb of S.robots) {
        if (rb.floor !== fl.level || rb.inElev > 0) continue; cnt++;
        const x = X(rb.x), y = Y(rb.y), R = 9;
        g.beginPath();
        g.moveTo(x + Math.cos(rb.h) * R, y + Math.sin(rb.h) * R);
        g.lineTo(x + Math.cos(rb.h + 2.5) * R * .8, y + Math.sin(rb.h + 2.5) * R * .8);
        g.lineTo(x + Math.cos(rb.h - 2.5) * R * .8, y + Math.sin(rb.h - 2.5) * R * .8);
        g.closePath();
        g.fillStyle = rb.color; g.fill();
        g.strokeStyle = "rgba(7,10,8,.9)"; g.lineWidth = 1.5; g.stroke();
      }

      const fc = document.getElementById("fc" + fl.level); if (fc) fc.textContent = cnt;
      let found = 0;
      for (let k = 0; k < fl.items.length; k++) if (S.detectedKeys.has(fl.level + ":" + k)) found++;
      const fi = document.getElementById("fi" + fl.level); if (fi) fi.textContent = found;
    }
  }

  function roundRect(g, x, y, w, h, r) {
    g.beginPath();
    g.moveTo(x + r, y);
    g.arcTo(x + w, y, x + w, y + h, r);
    g.arcTo(x + w, y + h, x, y + h, r);
    g.arcTo(x, y + h, x, y, r);
    g.arcTo(x, y, x + w, y, r);
    g.closePath();
  }

  function hud() {
    const cov = S.totalItems ? Math.round(S.detections / S.totalItems * 100) : 0;
    const el = (performance.now() - S.t0) / 1000;
    const tile = (n, l) => `<div class="kpi"><div class="n">${n}</div><div class="l">${l}</div></div>`;
    $("#kpis").innerHTML =
      tile(S.robots.length, "robots") +
      tile(S.detections + "<span style='color:var(--fg-dim);font-size:14px'>/" + S.totalItems + "</span>", "found") +
      tile(cov + "<span style='color:var(--fg-dim);font-size:14px'>%</span>", "coverage") +
      tile(el.toFixed(0) + "<span style='color:var(--fg-dim);font-size:14px'>s</span>", "elapsed");
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
  function setLive() {
    const live = $("#live"), txt = $("#livetxt");
    if (live) live.classList.toggle("paused", !S.playing);
    if (txt) txt.textContent = S.playing ? "running" : "paused";
  }
  $("#play").onclick = () => {
    S.playing = !S.playing;
    $("#play").textContent = S.playing ? "⏸ pause" : "▶ play";
    setLive();
  };
  $("#reset").onclick = reset;

  // show what env we're collecting into
  fetch("/api/projects/active").then(r => r.json()).then(d => {
    if (d.active) $("#scope").innerHTML = `Collecting jointly into <b>${d.active.project} / ${d.active.environment}</b> — ${S.n} robots, one warehouse, ${S.floors} floors connected by elevators. Each robot's detections land in the same environment.`;
  }).catch(() => {});

  reset().then(() => requestAnimationFrame(loop));
})();
