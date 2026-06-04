/* RoboRun — app.js
   Full dashboard: webcam + models + dataset + dimOS robot control + agent chat.
   Adapted from dimOS/RobotClaw with webcam/model/dataset features added. */

const activityList = document.querySelector("#activityList");
const activeJob = { id: null, timer: null };

// ── Utilities ──────────────────────────────────────────────────────────────

function escapeHtml(v) {
  return String(v ?? "").replaceAll("&","&amp;").replaceAll("<","&lt;").replaceAll(">","&gt;").replaceAll('"',"&quot;");
}

async function api(path, payload) {
  const opts = payload === undefined ? {} : {
    method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify(payload)
  };
  return (await fetch(path, opts)).json();
}

function append(message, data = {}) {
  const stamp = new Date().toLocaleTimeString();
  const text = [data.stdout, data.stderr, data.error].filter(Boolean).join("\n").trim();
  const item = document.createElement("article");
  item.className = `activityItem ${data.ok === false ? "bad" : data.ok ? "good" : ""}`.trim();
  item.innerHTML = `<div class="activityTop"><strong>${escapeHtml(message)}</strong><span>${stamp}</span></div>
    ${data.command || text ? `<details class="activityDetails"><summary>Details</summary>
      ${data.command ? `<code>$ ${escapeHtml(data.command)}</code>` : ""}
      ${text ? `<pre>${escapeHtml(text)}</pre>` : ""}</details>` : ""}`;
  if (activityList) { activityList.prepend(item); while (activityList.children.length > 20) activityList.lastElementChild.remove(); }
}

function relativeTime(iso) {
  if (!iso) return "";
  const d = Math.floor((Date.now() - new Date(iso).getTime()) / 1000);
  if (d < 60) return "just now";
  if (d < 3600) return `${Math.floor(d/60)}m ago`;
  if (d < 86400) return `${Math.floor(d/3600)}h ago`;
  return `${Math.floor(d/86400)}d ago`;
}

// ── Tab switching ────────────────────────────────────────────────────────

document.querySelectorAll(".rnav").forEach(btn => {
  btn.addEventListener("click", () => {
    document.querySelectorAll(".rnav").forEach(b => b.classList.remove("active"));
    btn.classList.add("active");
    document.querySelectorAll(".ctab-pane").forEach(p => p.classList.remove("active"));
    document.querySelector(`#tab-${btn.dataset.tab}`)?.classList.add("active");
  });
});

// ── Dashboard refresh ──────────────────────────────────────────────────────

function setDot(el, state) {
  if (!el) return;
  el.className = "rs-dot " + (state === "ok" ? "ok" : state === "warn" ? "warn" : state === "bad" ? "bad" : "");
}

async function refreshDashboard(quiet = false) {
  try {
    const d = await api("/api/dashboard");
    const p = d.profile || {};
    const dimos = d.dimos || {};
    const wc = d.webcam || {};
    const ds = d.dataset || {};
    const stats = d.stats || {};
    const cc = d.commandCenter || {};

    // Check if sim is running
    const sim = d.sim || {};
    simActive = !!sim.running;

    // Auto-start stream if webcam or sim is running but stream isn't showing
    // Don't start webcam if sim is active
    if (sim.running && !streamOn) startStream();
    else if (wc.running && !sim.running && !streamOn) startStream();

    // Sync model pill buttons with backend state
    const activeModels = new Set(wc.models || []);
    document.querySelectorAll(".model-bar .model-pill").forEach(btn => {
      btn.classList.toggle("active", activeModels.has(btn.dataset.model));
    });
    if (activeModels.has("clip")) clipSearchRow.style.display = "";
    else clipSearchRow.style.display = "none";

    // Detect model drops
    if (wc.running) checkModelDrops(wc.models || []);

    // Update robot actions section
    const rah = document.querySelector("#robotActionsHint");
    if (rah) {
      const hasRobot = d.robotOnline || dimos.running || sim.running;
      rah.textContent = sim.running ? `Sim: ${sim.robot || ""}` : (hasRobot ? "Connected" : "No robot connected");
      rah.className = `rah-hint${hasRobot ? " connected" : ""}`;
      if (hasRobot && robotActionsBody?.classList.contains("collapsed")) {
        robotActionsBody.classList.remove("collapsed");
        robotActionsHeader?.classList.add("open");
      }
    }

    // Rail status
    const sourceActive = wc.running || sim.running;
    setDot(document.querySelector("#sWebcam"), sourceActive ? "ok" : "warn");
    document.querySelector("#sWebcamVal").textContent = sim.running ? `SIM ${sim.fps || 0}fps` : (wc.running ? `${wc.fps || 0} fps` : "Off");
    setDot(document.querySelector("#sdimOS"), dimos.running || sim.running ? "ok" : "warn");
    document.querySelector("#sdimOSVal").textContent = sim.running ? "Sim" : (dimos.running ? "Online" : "Idle");
    const robotConnected = d.robotOnline || sim.running;
    setDot(document.querySelector("#sRobot"), robotConnected ? "ok" : (p.robotIp ? "bad" : "warn"));
    document.querySelector("#sRobotVal").textContent = sim.running ? (sim.robot || "Sim") : (d.robotOnline ? "Connected" : (p.robotIp ? "Unreach" : "No IP"));
    setDot(document.querySelector("#sMap"), cc.ok ? "ok" : "warn");
    document.querySelector("#sMapVal").textContent = cc.ok ? "Ready" : "Closed";

    // Rail metrics
    document.querySelector("#fpsSummary").textContent = wc.running ? `${wc.fps || 0}` : "--";
    const fpsBar = document.querySelector("#fpsBar");
    if (fpsBar) fpsBar.style.width = `${Math.min(100, (wc.fps || 0) / 30 * 100)}%`;
    document.querySelector("#diskSummary").textContent = `${stats.disk?.percent || "--"}%`;
    const diskBar = document.querySelector("#diskBar");
    if (diskBar) diskBar.style.width = `${stats.disk?.percent || 0}%`;
    document.querySelector("#recSummary").textContent = ds.recording ? `${ds.frames || 0}` : "--";

    // System tab
    const el = (id) => document.querySelector(id);
    if (el("#deviceName")) el("#deviceName").textContent = p.deviceName || "RoboRun Station";
    if (el("#robotIpValue")) el("#robotIpValue").textContent = p.robotIp || "Not set";
    if (el("#blueprintValue")) el("#blueprintValue").textContent = p.blueprint || "--";
    if (el("#sourceValue")) el("#sourceValue").textContent = wc.running ? "webcam" : (dimos.running ? "robot" : "none");
    if (el("#loadValue")) el("#loadValue").textContent = `Load ${(stats.load || []).join(" / ")}`;
    if (el("#diskMeter")) el("#diskMeter").value = stats.disk?.percent || 0;
    if (el("#diskText")) el("#diskText").textContent = `${stats.disk?.percent || "--"}%`;
    if (el("#deviceNameInput")) el("#deviceNameInput").value = p.deviceName || "";
    if (el("#robotIp")) el("#robotIp").value = p.robotIp || "";
    if (el("#dimosPath")) el("#dimosPath").value = p.dimosPath || "";
    if (el("#blueprint")) el("#blueprint").value = p.blueprint || "unitree-go2";
    if (el("#onlinePill")) {
      el("#onlinePill").textContent = wc.running ? "Webcam" : (dimos.running ? "Online" : "Idle");
      el("#onlinePill").className = `pill ${wc.running || dimos.running ? "" : "idle"}`.trim();
    }

    // Recording bar
    const recDot = el("#recDot");
    const recLabel = el("#recLabel");
    const recFrames = el("#recFrames");
    if (recDot) recDot.className = `rec-indicator ${ds.recording ? "recording" : ""}`;
    if (recLabel) { recLabel.textContent = ds.recording ? `REC ${ds.dataset || ""}` : "Not recording"; recLabel.className = `rec-label ${ds.recording ? "recording" : ""}`; }
    if (recFrames) recFrames.textContent = ds.recording ? `${ds.frames || 0} frames` : "";

    // Recording badge
    const dsBadge = el("#datasetRecBadge");
    if (dsBadge) { dsBadge.textContent = ds.recording ? `REC ${ds.frames} frames` : "Not recording"; dsBadge.className = `pill ${ds.recording ? "bad" : "idle"}`; }

    if (!quiet) append("Dashboard refreshed.", { ok: true });
  } catch (e) {
    append("Dashboard refresh failed.", { ok: false, error: e.message });
  }
}

// ── Camera ──────────────────────────────────────────────────────────────────

const cameraImg = document.querySelector("#cameraImg");
const cameraStream = document.querySelector("#cameraStream");
const cameraPlaceholder = document.querySelector("#cameraPlaceholder");
const cameraStateBadge = document.querySelector("#cameraStateBadge");
const cameraStateOverlay = document.querySelector("#cameraStateOverlay");
const camRecDot = document.querySelector("#camRecDot");
const streamToggleBtn = document.querySelector("#cameraStreamToggle");
let streamOn = false;
let statePoller = null;

function startStream() {
  cameraStream.src = "/api/camera/stream?" + Date.now();
  cameraStream.style.display = "";
  cameraImg.style.display = "none";
  cameraPlaceholder.style.display = "none";
  streamOn = true;
  streamToggleBtn.textContent = "● LIVE";
  streamToggleBtn.classList.add("active");
  camRecDot.classList.add("live");
  cameraStateBadge.classList.add("live");
  statePoller = setInterval(async () => {
    try {
      const s = await (await fetch("/api/scene")).json();
      const mode = s.mode || "--";
      const state = s.state || "";
      cameraStateBadge.textContent = `${mode} · ${state}`;
      if (s.objects != null || s.persons != null) {
        cameraStateOverlay.style.display = "";
        const lines = [state];
        if (s.objects != null) lines.push(`objects: ${s.objects}`);
        if (s.persons != null) lines.push(`persons: ${s.persons}`);
        if (s.clip_query) lines.push(`CLIP: "${s.clip_query}"`);
        if (s.clip_matches) lines.push(`matches: ${s.clip_matches}`);
        const threats = s.threats || {};
        for (const [id, t] of Object.entries(threats)) {
          const pct = Math.round(t * 100);
          const lbl = t >= 0.7 ? "THREAT" : t >= 0.4 ? "cautious" : "curious";
          lines.push(`  #${id} ${lbl} ${pct}%`);
        }
        cameraStateOverlay.textContent = lines.join("\n");
      } else {
        cameraStateOverlay.style.display = "none";
      }
    } catch {}
  }, 2000);
}

function stopStream() {
  cameraStream.src = "";
  cameraStream.style.display = "none";
  streamOn = false;
  streamToggleBtn.textContent = "● STREAM";
  streamToggleBtn.classList.remove("active");
  camRecDot.classList.remove("live");
  cameraStateBadge.classList.remove("live");
  clearInterval(statePoller);
  cameraStateOverlay.style.display = "none";
  cameraStateBadge.textContent = "--";
  if (cameraImg.style.display === "none") cameraPlaceholder.style.display = "";
}

streamToggleBtn.addEventListener("click", async () => {
  if (streamOn) { stopStream(); return; }
  const st = await api("/api/webcam/state");
  if (!st.running && st.state !== "running") { await startWebcam(); return; }
  startStream();
});

document.querySelector("#cameraRefresh").addEventListener("click", async () => {
  try {
    const data = await api("/api/camera");
    if (data.ok) {
      stopStream();
      cameraImg.src = data.image;
      cameraImg.style.display = "";
      cameraPlaceholder.style.display = "none";
      cameraStateBadge.textContent = "snapshot";
    } else {
      cameraStateBadge.textContent = data.error ? data.error.slice(0, 30) : "no signal";
    }
  } catch (e) { cameraStateBadge.textContent = "error"; }
});

// ── Model toggles (control tab bar) ─────────────────────────────────────────

const clipSearchRow = document.querySelector("#clipSearchRow");

document.querySelectorAll(".model-bar .model-pill").forEach(btn => {
  btn.addEventListener("click", () => {
    btn.classList.toggle("active");
    const active = [...document.querySelectorAll(".model-bar .model-pill.active")].map(b => b.dataset.model);
    api("/api/webcam/models", { models: active });
    clipSearchRow.style.display = active.includes("clip") ? "" : "none";
  });
});

document.querySelector("#clipSearchBtn")?.addEventListener("click", () => {
  const q = document.querySelector("#clipQuery").value.trim();
  api("/api/webcam/clip_query", { query: q });
  if (q) append(`CLIP search: "${q}"`, { ok: true });
});
document.querySelector("#clipQuery")?.addEventListener("keydown", (e) => {
  if (e.key === "Enter") document.querySelector("#clipSearchBtn")?.click();
});

// ── Recording toggle (control tab bar) ──────────────────────────────────────

document.querySelector("#recToggle")?.addEventListener("click", async () => {
  const status = await api("/api/dataset/status");
  if (status.recording) {
    const r = await api("/api/dataset/stop", {});
    append(r.ok ? `Recording stopped: ${r.frames} frames` : "Stop failed.", r);
  } else {
    const name = document.querySelector("#datasetName")?.value || "default";
    const r = await api("/api/dataset/start", { name });
    append(r.ok ? `Recording started: ${name}` : "Start failed.", r);
  }
  refreshDashboard(true);
});

// ── Webcam start (from Control tab placeholder + stream button) ────────────

async function startWebcam() {
  if (simActive) { await api("/api/sim/stop", {}); simActive = false; }
  const models = [...document.querySelectorAll("[data-model].active")].map(b => b.dataset.model);
  const r = await api("/api/webcam/start", { camera: 0, models: models.length ? models : ["yolo"] });
  append(r.ok ? "Webcam started" : "Webcam start failed.", r);
  if (r.ok) { startStream(); refreshDashboard(true); }
}

document.querySelector("#camStartWebcam")?.addEventListener("click", startWebcam);

// ── Simulator (from Control tab placeholder) ───────────────────────────────

let simActive = false;

document.querySelector("#camStartSim")?.addEventListener("click", async () => {
  const picker = document.querySelector("#simPicker");
  if (picker.style.display === "none") {
    const r = await api("/api/sim/robots");
    const sel = document.querySelector("#simRobotSelect");
    sel.innerHTML = (r.robots || [])
      .filter(rb => rb.available)
      .map(rb => `<option value="${rb.id}">${rb.name} (${rb.type})</option>`)
      .join("");
    picker.style.display = "";
  } else {
    picker.style.display = "none";
  }
});

document.querySelector("#simLaunchBtn")?.addEventListener("click", async () => {
  const robot = document.querySelector("#simRobotSelect").value;
  const r = await api("/api/sim/start", { robot });
  append(r.ok ? `Sim started: ${robot}` : "Sim start failed.", r);
  if (r.ok) {
    simActive = true;
    document.querySelector("#simPicker").style.display = "none";
    startStream();
    refreshDashboard(true);
  }
});

// ── Dataset tab ─────────────────────────────────────────────────────────────

document.querySelector("#datasetStartBtn")?.addEventListener("click", async () => {
  const name = document.querySelector("#datasetName")?.value.trim() || "default";
  const r = await api("/api/dataset/start", { name });
  append(r.ok ? `Recording: ${name}` : "Start failed.", r);
  refreshDashboard(true);
  loadDatasets();
});

document.querySelector("#datasetStopBtn")?.addEventListener("click", async () => {
  const r = await api("/api/dataset/stop", {});
  append(r.ok ? `Saved ${r.frames} frames` : "Stop failed.", r);
  refreshDashboard(true);
  loadDatasets();
});

async function loadDatasets() {
  try {
    const r = await api("/api/dataset/list");
    const list = document.querySelector("#datasetList");
    const empty = document.querySelector("#datasetEmpty");
    const datasets = r.datasets || [];
    [...list.querySelectorAll(".dataset-card")].forEach(el => el.remove());
    if (!datasets.length) { empty.style.display = ""; return; }
    empty.style.display = "none";
    datasets.forEach(ds => {
      const card = document.createElement("article");
      card.className = "dataset-card";
      card.innerHTML = `<div class="dataset-card-head">
        <span class="dataset-card-name">${escapeHtml(ds.name)}</span>
        <span class="dataset-card-meta">${ds.episodes} episode(s)</span>
      </div>
      <div style="font-size:11px;color:var(--muted);font-family:var(--font-mono)">
        ${ds.details.map(ep => `${ep.episode_id}: ${ep.frames} frames (${relativeTime(ep.created_at)})`).join("<br>")}
      </div>`;
      list.appendChild(card);
    });
  } catch {}
}

// ── System tab ──────────────────────────────────────────────────────────────

document.querySelector("#saveProfile")?.addEventListener("click", async () => {
  const payload = {
    deviceName: document.querySelector("#deviceNameInput")?.value.trim() || "RoboRun Station",
    robotIp: document.querySelector("#robotIp")?.value.trim(),
    dimosPath: document.querySelector("#dimosPath")?.value.trim(),
    blueprint: document.querySelector("#blueprint")?.value,
  };
  const r = await api("/api/profile", payload);
  append(r.ok ? "Profile saved." : "Save failed.", r);
  refreshDashboard(true);
});

document.querySelector("#dimosReplay")?.addEventListener("click", async () => {
  append("Starting replay bot...");
  const r = await api("/api/demo", {});
  append(r.ok ? "Replay started." : "Start failed.", r);
  refreshDashboard(true);
});

document.querySelector("#dimosLaunch")?.addEventListener("click", async () => {
  const ip = document.querySelector("#robotIp")?.value.trim();
  if (!ip) { append("Robot IP required.", { ok: false }); return; }
  const bp = document.querySelector("#blueprint")?.value;
  const r = await api("/api/launch", { robotIp: ip, blueprint: bp, mode: "hardware", viewer: "rerun", daemon: true });
  append(r.ok ? "Robot starting..." : "Launch failed.", r);
  refreshDashboard(true);
});

document.querySelector("#dimosStatus")?.addEventListener("click", async () => {
  const r = await api("/api/status", {});
  append("Status check.", r);
});

document.querySelector("#dimosStop")?.addEventListener("click", async () => {
  const r = await api("/api/stop", {});
  append(r.ok ? "dimOS stopped." : "Stop failed.", r);
  refreshDashboard(true);
});

document.querySelector("#clearOutput")?.addEventListener("click", () => { if (activityList) activityList.innerHTML = ""; });
document.querySelector("#refresh")?.addEventListener("click", () => { refreshDashboard(); loadTasks(); loadFleet(); loadDatasets(); loadEvents(); });

// ── Events / Notifications ──────────────────────────────────────────────────

const notifDrawer = document.querySelector("#notifDrawer");
const notifOverlay = document.querySelector("#notifOverlay");
const notifList = document.querySelector("#notifList");
const notifBadge = document.querySelector("#notifBadge");
let lastSeenEventTs = localStorage.getItem("lastSeenEventTs") || "";

function renderEvents(events) {
  if (!events.length) { notifList.innerHTML = `<div class="notif-empty">No events yet.</div>`; return; }
  notifList.innerHTML = events.map(ev => {
    const isUnread = ev.ts > lastSeenEventTs;
    return `<div class="notifItem ${isUnread ? "unread" : ""} ${ev.level === "error" ? "notifError" : ""}">
      <div class="notifItemHead"><span>${escapeHtml(ev.message)}</span><span class="notifTime">${relativeTime(ev.ts)}</span></div></div>`;
  }).join("");
}

async function loadEvents(quiet = true) {
  try {
    const r = await api(`/api/events?limit=50&since=${encodeURIComponent(lastSeenEventTs)}`);
    renderEvents(r.events || []);
    const unread = r.unread || 0;
    if (unread > 0) { notifBadge.textContent = unread > 9 ? "9+" : unread; notifBadge.style.display = ""; }
    else { notifBadge.style.display = "none"; }
  } catch {}
}

document.querySelector("#notifBtn")?.addEventListener("click", () => { notifDrawer.style.display = ""; notifOverlay.style.display = ""; loadEvents(); });
document.querySelector("#notifClose")?.addEventListener("click", () => { notifDrawer.style.display = "none"; notifOverlay.style.display = "none"; });
document.querySelector("#notifOverlay")?.addEventListener("click", () => { notifDrawer.style.display = "none"; notifOverlay.style.display = "none"; });
document.querySelector("#notifMarkRead")?.addEventListener("click", () => {
  lastSeenEventTs = new Date().toISOString().replace(/\.\d{3}Z$/, "Z");
  localStorage.setItem("lastSeenEventTs", lastSeenEventTs);
  notifBadge.style.display = "none";
  loadEvents();
});

// ── Map / Command Center ────────────────────────────────────────────────────

const mapIframe = document.querySelector("#mapIframe");
const mapOffline = document.querySelector("#mapOffline");
const mapConnBadge = document.querySelector("#mapConnStatus");
let ccSocket = null, mapIframeLoaded = false;

function setMapOnline(online) {
  if (online) {
    mapOffline.style.display = "none";
    if (!mapIframeLoaded) { mapIframe.src = "http://127.0.0.1:7779/command-center"; mapIframeLoaded = true; }
    mapIframe.style.display = "";
    mapConnBadge.textContent = "● Live"; mapConnBadge.className = "map-badge online";
  } else {
    mapOffline.style.display = ""; mapIframe.style.display = "none";
    mapConnBadge.textContent = "● Disconnected"; mapConnBadge.className = "map-badge offline";
  }
}

function connectCommandCenter() {
  if (typeof io === "undefined") return;
  if (ccSocket) { ccSocket.disconnect(); ccSocket = null; }
  try {
    ccSocket = io("http://127.0.0.1:7779", { transports: ["websocket", "polling"], reconnection: true, reconnectionDelay: 4000, timeout: 3000 });
    ccSocket.on("connect", () => setMapOnline(true));
    ccSocket.on("disconnect", () => setMapOnline(false));
    ccSocket.on("connect_error", () => setMapOnline(false));
  } catch { setMapOnline(false); }
}

document.querySelector("#mapStartRobot")?.addEventListener("click", async () => {
  const ip = document.querySelector("#robotIp")?.value.trim();
  if (!ip) { append("Robot IP required.", { ok: false }); return; }
  const bp = document.querySelector("#blueprint")?.value;
  await api("/api/launch", { robotIp: ip, blueprint: bp, mode: "hardware", viewer: "rerun", daemon: true });
  setTimeout(connectCommandCenter, 8000);
});
document.querySelector("#mapStartReplay")?.addEventListener("click", async () => {
  await api("/api/demo", {});
  setTimeout(connectCommandCenter, 5000);
});

connectCommandCenter();
setInterval(() => { if (!ccSocket || !ccSocket.connected) connectCommandCenter(); }, 10000);

// ── Robot skill buttons (direct MCP) ────────────────────────────────────────

const SKILL_MCP = {
  explore:        { tool: "begin_exploration" },
  stop:           { tool: "stop_navigation" },
  navigate:       { tool: "navigate_with_text", ask: "Navigate to where?", placeholder: "the kitchen", argKey: "query" },
  dog_mode:       { tool: "start_dog_mode" },
  stop_dog:       { tool: "stop_dog_mode" },
  follow_person:  { tool: "smart_follow_person", ask: "Follow which person?", placeholder: "person in red jacket", argKey: "query" },
  patrol:         { tool: "start_patrol" },
  find:           { tool: "smart_find", ask: "Find what?", placeholder: "a backpack", argKey: "query" },
  scene:          { tool: "query_scene" },
  follow_object:  { tool: "smart_follow_object", ask: "Follow which object?", placeholder: "chair", argKey: "class_name" },
  stop_find:      { tool: "stop_find" },
  tag_location:   { tool: "tag_location", ask: "Tag location as...", placeholder: "charging dock", argKey: "location_name" },
  speak:          { tool: "speak", ask: "Robot should say...", placeholder: "Hello!", argKey: "text" },
  stand:          { tool: "execute_sport_command", fixedArgs: { command_name: "RecoveryStand" } },
  sit:            { tool: "execute_sport_command", fixedArgs: { command_name: "Sit" } },
  flip:           { tool: "execute_sport_command", fixedArgs: { command_name: "FrontFlip" } },
  jump:           { tool: "execute_sport_command", fixedArgs: { command_name: "FrontJump" } },
};

const SKILL_PROMPTS = {
  where_am_i: { msg: "Where is the robot? Use daneel_where_am_i and report GPS and nearby landmarks." },
  map_query:  { ask: "Find what place on the map?", placeholder: "nearest exit", build: v => `Use daneel_map_query to find "${v}".` },
  status:     { msg: "Full status report: check dashboard status, robot state. Report battery, position, mode, errors." },
};

const MOVEMENT_SKILLS = new Set(["begin_exploration", "smart_follow_person", "smart_follow_object", "smart_approach", "smart_find", "navigate_with_text", "start_dog_mode", "start_patrol", "execute_sport_command"]);
const STOP_FOR = { "begin_exploration": "stop_navigation", "navigate_with_text": "stop_navigation", "smart_follow_person": "stop_following", "smart_follow_object": "stop_following", "smart_approach": "stop_following", "smart_find": "stop_find", "start_dog_mode": "stop_dog_mode", "start_patrol": "stop_patrol" };
let lastMovement = null;

function preemptFor(skill) {
  if (!lastMovement || lastMovement === skill) return;
  const stop = STOP_FOR[lastMovement];
  if (!stop || stop === skill) return;
  fetch("/api/mcp/call", { method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify({ name: stop, args: {} }) }).catch(() => {});
}

async function mcpCall(name, args) {
  const a = args || {};
  if (MOVEMENT_SKILLS.has(name)) { preemptFor(name); lastMovement = name; }
  addUserMessage(`▷ ${name}${Object.keys(a).length ? "  " + JSON.stringify(a) : ""}`);
  startAgentTurn();
  const toolId = "d-" + Date.now();
  addToolCall(toolId, name, a);
  let taskId;
  try {
    const resp = await fetch("/api/mcp/call", { method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify({ name, args: a }) });
    const data = await resp.json();
    if (!data.task_id) { addToolResult(toolId, data.error || "failed", true); return; }
    taskId = data.task_id;
  } catch (e) { addToolResult(toolId, String(e), true); return; }
  let interval = 250;
  const started = Date.now();
  while (true) {
    await new Promise(r => setTimeout(r, interval));
    interval = Math.min(2000, Math.round(interval * 1.4));
    try {
      const s = await (await fetch("/api/mcp/result?id=" + encodeURIComponent(taskId))).json();
      if (s.status === "done") { addToolResult(toolId, s.result || "(ok)", false); return; }
      if (s.status === "error") { addToolResult(toolId, s.error || "MCP error", true); return; }
      const el = Math.round((Date.now() - started) / 1000);
      const card = currentTurn?.toolCards.get(toolId);
      if (card) { const st = card.querySelector(".tc-status"); if (st) st.textContent = `running... ${el}s`; }
    } catch {}
  }
}

// Skill prompt modal
const skillPrompt = document.querySelector("#skillPrompt");
const spLabel = document.querySelector("#spLabel");
const spInput = document.querySelector("#spInput");
let spBuild = null;

function openSkillPrompt(label, placeholder, build) {
  spLabel.textContent = label; spInput.value = ""; spInput.placeholder = placeholder || "";
  spBuild = build; skillPrompt.style.display = ""; setTimeout(() => spInput.focus(), 50);
}
function closeSkillPrompt() { skillPrompt.style.display = "none"; spBuild = null; }
function submitSkillPrompt() {
  const v = spInput.value.trim(); if (!v) return;
  const result = spBuild ? spBuild(v) : v;
  closeSkillPrompt();
  if (result == null) return;
  agentSendMessage(result);
}
document.querySelector("#spCancel")?.addEventListener("click", closeSkillPrompt);
document.querySelector("#spBackdrop")?.addEventListener("click", closeSkillPrompt);
document.querySelector("#spSend")?.addEventListener("click", submitSkillPrompt);
spInput?.addEventListener("keydown", (e) => { if (e.key === "Enter") submitSkillPrompt(); if (e.key === "Escape") closeSkillPrompt(); });

document.querySelectorAll(".ab").forEach(btn => {
  btn.addEventListener("click", () => {
    const skill = btn.dataset.skill;
    if (skill === "restart") {
      fetch("/api/sim/reset", { method: "POST", headers: {"Content-Type":"application/json"}, body: "{}" })
        .then(r => r.json())
        .then(d => { if (!d.ok) console.warn("Reset failed:", d.error); })
        .catch(e => console.error("Reset error:", e));
      return;
    }
    const mcp = SKILL_MCP[skill];
    if (mcp) {
      if (mcp.ask) {
        openSkillPrompt(mcp.ask, mcp.placeholder, (v) => {
          mcpCall(mcp.tool, Object.assign({}, mcp.fixedArgs || {}, mcp.argKey ? { [mcp.argKey]: v } : {}));
          return null;
        });
      } else { mcpCall(mcp.tool, mcp.fixedArgs || {}); }
      return;
    }
    const def = SKILL_PROMPTS[skill];
    if (!def) return;
    if (def.ask) { openSkillPrompt(def.ask, def.placeholder, def.build); }
    else { agentSendMessage(def.msg); }
  });
});

// ── Movement controls ───────────────────────────────────────────────────────

let moveStep = 0.5;
const moveStepSlider = document.querySelector("#moveStep");
const moveStepVal = document.querySelector("#moveStepVal");
const moveStepLabel = document.querySelector("#moveStepLabel");

moveStepSlider?.addEventListener("input", () => {
  moveStep = parseFloat(moveStepSlider.value);
  if (moveStepVal) moveStepVal.textContent = moveStep.toFixed(1) + " m";
  if (moveStepLabel) moveStepLabel.textContent = moveStep.toFixed(1) + "m";
});

function move(fwd, left, deg) {
  if (simActive) {
    api("/api/sim/move", { forward: fwd, left, turn: deg });
    return;
  }
  mcpCall("relative_move", { forward: fwd, left, degrees: deg });
}

const MOVE_BTNS = {
  "mv-fwd": () => move(moveStep, 0, 0), "mv-back": () => move(-moveStep, 0, 0),
  "mv-left": () => move(0, moveStep, 0), "mv-right": () => move(0, -moveStep, 0),
  "mv-tl": () => move(0, 0, 30), "mv-tr": () => move(0, 0, -30),
  "mv-stop": () => mcpCall("stop_navigation", {}),
};
Object.entries(MOVE_BTNS).forEach(([id, fn]) => document.querySelector(`#${id}`)?.addEventListener("click", fn));

const MOVE_KEYS = { "w": () => move(moveStep,0,0), "s": () => move(-moveStep,0,0), "a": () => move(0,moveStep,0), "d": () => move(0,-moveStep,0), "q": () => move(0,0,30), "e": () => move(0,0,-30), " ": () => mcpCall("stop_navigation", {}) };
document.addEventListener("keydown", (ev) => {
  if (new Set(["INPUT","TEXTAREA","SELECT"]).has(document.activeElement?.tagName)) return;
  const fn = MOVE_KEYS[ev.key.toLowerCase()] || MOVE_KEYS[ev.key];
  if (fn) { ev.preventDefault(); fn(); }
});

// ── Agent chat ──────────────────────────────────────────────────────────────

const agentMessages = document.querySelector("#agentMessages");
const agentInput = document.querySelector("#agentInput");
const agentSend = document.querySelector("#agentSend");
const agentStop = document.querySelector("#agentStop");
const agentStatus = document.querySelector("#agentStatus");
let agentStreaming = false;
let currentTurn = null;

function renderMarkdown(text) {
  if (typeof marked !== "undefined") { try { return marked.parse(text); } catch {} }
  return escapeHtml(text).replace(/\n/g, "<br>");
}

function scrollChat() { agentMessages.scrollTop = agentMessages.scrollHeight; }

function addUserMessage(text) {
  const div = document.createElement("div"); div.className = "msg-user";
  div.innerHTML = `<div class="msg-label">You</div><div class="msg-user-bubble"></div>`;
  div.querySelector(".msg-user-bubble").textContent = text;
  agentMessages.appendChild(div); scrollChat();
}

function startAgentTurn() {
  const container = document.createElement("div"); container.className = "msg-agent";
  const label = document.createElement("div"); label.className = "msg-label"; label.textContent = "Agent";
  const content = document.createElement("div"); content.className = "turn-content";
  container.appendChild(label); container.appendChild(content);
  agentMessages.appendChild(container);
  currentTurn = { container, content, textBlock: null, toolCards: new Map() };
  scrollChat(); return currentTurn;
}

function ensureTextBlock(streaming = true) {
  if (!currentTurn) startAgentTurn();
  const last = currentTurn.content.lastElementChild;
  if (last && last.classList.contains("text-block")) { currentTurn.textBlock = last; return last; }
  const tb = document.createElement("div");
  tb.className = "text-block" + (streaming ? " streaming" : "");
  currentTurn.content.appendChild(tb); currentTurn.textBlock = tb; return tb;
}

function sealTextBlock() { if (currentTurn?.textBlock) { currentTurn.textBlock.classList.remove("streaming"); currentTurn.textBlock = null; } }

function addThinkingBlock(thinking) {
  if (!currentTurn) startAgentTurn(); sealTextBlock();
  const card = document.createElement("div"); card.className = "thinking-block";
  card.innerHTML = `<div class="th-header"><span><span class="th-chevron">▸</span> thinking</span></div><div class="th-body"></div>`;
  card.querySelector(".th-body").textContent = thinking;
  card.querySelector(".th-header").addEventListener("click", () => card.classList.toggle("open"));
  currentTurn.content.appendChild(card); scrollChat();
}

function prettyToolName(n) { return n.replace(/^mcp__/, "").replace(/^dimos_workbench_/, "workbench·").replace(/^daneel_/, "daneel·"); }

function addToolCall(toolId, name, input) {
  if (!currentTurn) startAgentTurn(); sealTextBlock();
  const card = document.createElement("div"); card.className = "tool-call-card";
  card.innerHTML = `<div class="tc-header"><span class="tc-icon">⚙</span><span class="tc-name">${escapeHtml(prettyToolName(name))}</span><span class="tc-status">running...</span><span class="tc-chevron">▸</span></div><div class="tc-body"><pre class="tc-json"></pre></div>`;
  const hasInput = input && Object.keys(input).length > 0;
  card.querySelector(".tc-json").textContent = hasInput ? JSON.stringify(input, null, 2) : "(no arguments)";
  card.querySelector(".tc-header").addEventListener("click", () => card.classList.toggle("open"));
  currentTurn.content.appendChild(card);
  if (toolId) currentTurn.toolCards.set(toolId, card);
  scrollChat();
}

function addToolResult(toolUseId, resultText, isError) {
  if (!currentTurn) startAgentTurn();
  const callCard = toolUseId ? currentTurn.toolCards.get(toolUseId) : null;
  if (callCard) { const s = callCard.querySelector(".tc-status"); if (s) { s.textContent = isError ? "error" : "done"; s.style.color = isError ? "var(--red)" : "var(--green)"; } }
  sealTextBlock();
  const card = document.createElement("div");
  card.className = "tool-result-card open" + (isError ? " result-error" : "");
  card.innerHTML = `<div class="tr-header"><span class="tr-icon">${isError ? "✕" : "✓"}</span><span class="tr-label">${isError ? "tool error" : "tool result"}</span><span class="tr-chevron">▸</span></div><div class="tr-body"><pre></pre></div>`;
  card.querySelector("pre").textContent = resultText || "(empty)";
  card.querySelector(".tr-header").addEventListener("click", () => card.classList.toggle("open"));
  currentTurn.content.appendChild(card); scrollChat();
}

function agentSetBusy(busy) {
  agentStreaming = busy;
  agentSend.disabled = busy; agentInput.disabled = busy;
  agentStop.style.display = busy ? "" : "none"; agentSend.style.display = busy ? "none" : "";
  agentStatus.textContent = busy ? "thinking..." : "idle";
  agentStatus.className = `agent-badge ${busy ? "busy" : ""}`;
}

async function agentSendMessage(message) {
  const text = (message ?? agentInput.value).trim();
  if (!text || agentStreaming) return;
  agentInput.value = "";
  addUserMessage(text); agentSetBusy(true); startAgentTurn();
  try {
    const resp = await fetch("/api/agent/chat", { method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify({ message: text }) });
    if (!resp.ok) { ensureTextBlock(false).textContent = "Request failed: " + resp.status; agentSetBusy(false); return; }
    const reader = resp.body.getReader(); const decoder = new TextDecoder(); let buf = "";
    while (true) {
      const { done, value } = await reader.read(); if (done) break;
      buf += decoder.decode(value, { stream: true });
      const lines = buf.split("\n"); buf = lines.pop();
      for (const line of lines) {
        if (!line.startsWith("data: ")) continue;
        let chunk; try { chunk = JSON.parse(line.slice(6)); } catch { continue; }
        if (chunk.type === "text") { const tb = ensureTextBlock(true); tb.innerHTML = renderMarkdown(chunk.accumulated); tb.classList.add("streaming"); scrollChat(); }
        else if (chunk.type === "thinking") { addThinkingBlock(chunk.thinking); }
        else if (chunk.type === "tool_use") { addToolCall(chunk.tool_id, chunk.tool_name, chunk.tool_input); }
        else if (chunk.type === "tool_result") { addToolResult(chunk.tool_use_id, chunk.result, chunk.is_error); }
        else if (chunk.type === "done") { sealTextBlock(); if (chunk.cost) { const c = document.createElement("div"); c.className = "turn-cost"; c.textContent = `$${chunk.cost.toFixed(4)}`; currentTurn.container.appendChild(c); } scrollChat(); }
        else if (chunk.type === "error") { sealTextBlock(); ensureTextBlock(false).textContent = "Error: " + chunk.error; currentTurn.container.classList.add("msg-error"); }
      }
    }
  } catch (e) { sealTextBlock(); ensureTextBlock(false).textContent = "Error: " + e.message; currentTurn.container.classList.add("msg-error"); }
  sealTextBlock(); agentSetBusy(false);
}

document.querySelectorAll(".sc-btn").forEach(btn => btn.addEventListener("click", () => agentSendMessage(btn.dataset.msg)));
agentSend?.addEventListener("click", () => agentSendMessage());
agentInput?.addEventListener("keydown", (e) => { if (e.key === "Enter") agentSendMessage(); });
agentStop?.addEventListener("click", async () => {
  await fetch("/api/agent/stop", { method: "POST", headers: {"Content-Type":"application/json"}, body: "{}" });
  agentSetBusy(false);
});
document.querySelector("#agentClear")?.addEventListener("click", async () => {
  await fetch("/api/agent/clear", { method: "POST", headers: {"Content-Type":"application/json"}, body: "{}" });
  agentMessages.innerHTML = ""; currentTurn = null; agentStatus.textContent = "idle";
});

// ── Tasks ───────────────────────────────────────────────────────────────────

let currentTaskFilter = "all";
const taskList = document.querySelector("#taskList");
const taskEmpty = document.querySelector("#taskEmpty");

async function loadTasks() {
  try {
    const r = await api(`/api/tasks?status=${currentTaskFilter}`);
    const tasks = r.tasks || [];
    taskEmpty.style.display = tasks.length ? "none" : "block";
    [...taskList.querySelectorAll(".taskCard")].forEach(el => el.remove());
    tasks.forEach(task => {
      const card = document.createElement("article");
      card.className = "taskCard";
      card.innerHTML = `<div class="taskCardHead"><div class="taskCardLeft"><strong class="taskTitle">${escapeHtml(task.name)}</strong></div>
        <div class="taskCardRight"><span class="taskStatusPill">${escapeHtml(task.status)}</span></div></div>
        <div class="taskActions">
          <button class="taskActionBtn runBtn" data-id="${task.id}">▶ Run</button>
          <button class="taskActionBtn deleteBtn" data-id="${task.id}">Delete</button>
        </div>`;
      taskList.appendChild(card);
    });
    taskList.querySelectorAll(".runBtn").forEach(b => b.addEventListener("click", async () => { await api("/api/tasks/run", { id: b.dataset.id }); loadTasks(); }));
    taskList.querySelectorAll(".deleteBtn").forEach(b => b.addEventListener("click", async () => { if (confirm("Delete?")) { await api("/api/tasks/delete", { id: b.dataset.id }); loadTasks(); } }));
  } catch {}
}

document.querySelectorAll("[data-filter]").forEach(btn => {
  btn.addEventListener("click", () => {
    document.querySelectorAll("[data-filter]").forEach(b => b.classList.remove("active"));
    btn.classList.add("active"); currentTaskFilter = btn.dataset.filter; loadTasks();
  });
});

let selectedType = "one_off", selectedAction = "explore";
const addTaskForm = document.querySelector("#addTaskForm");
document.querySelector("#addTaskBtn")?.addEventListener("click", () => { addTaskForm.style.display = addTaskForm.style.display === "none" ? "block" : "none"; });
document.querySelector("#cancelTask")?.addEventListener("click", () => { addTaskForm.style.display = "none"; });
document.querySelectorAll(".typeBtn").forEach(btn => { btn.addEventListener("click", () => { document.querySelectorAll(".typeBtn").forEach(b => b.classList.remove("active")); btn.classList.add("active"); selectedType = btn.dataset.type; document.querySelector("#scheduleField").style.display = selectedType === "recurring" ? "" : "none"; }); });
const actionParamSections = { explore: "paramsExplore", navigate_gps: "paramsGps", navigate_map: "paramsMap", query: "paramsQuery" };
document.querySelectorAll(".actionBtn").forEach(btn => { btn.addEventListener("click", () => { document.querySelectorAll(".actionBtn").forEach(b => b.classList.remove("active")); btn.classList.add("active"); selectedAction = btn.dataset.action; Object.entries(actionParamSections).forEach(([a, id]) => { document.querySelector(`#${id}`).style.display = a === selectedAction ? "" : "none"; }); }); });
document.querySelector("#submitTask")?.addEventListener("click", async () => {
  const name = document.querySelector("#taskName")?.value.trim();
  if (!name) { append("Task name required.", { ok: false }); return; }
  const payload = { name, description: document.querySelector("#taskDescription")?.value.trim(), type: selectedType, action: selectedAction, schedule: document.querySelector("#taskSchedule")?.value, source: "dashboard" };
  if (selectedAction === "navigate_gps") { payload.lat = parseFloat(document.querySelector("#paramLat")?.value) || 0; payload.lon = parseFloat(document.querySelector("#paramLon")?.value) || 0; }
  else if (selectedAction === "navigate_map") { payload.x = parseFloat(document.querySelector("#paramX")?.value) || 0; payload.y = parseFloat(document.querySelector("#paramY")?.value) || 0; }
  else if (selectedAction === "query") { payload.text = document.querySelector("#paramText")?.value.trim(); }
  const r = await api("/api/tasks/create", payload);
  if (r.ok) { append(`Task: ${r.task.name}`, { ok: true }); addTaskForm.style.display = "none"; }
  loadTasks();
});

// ── Fleet ───────────────────────────────────────────────────────────────────

let fleetData = [], blueprintsData = [], currentFleetFilter = "all", editingRobotId = null, editingBlueprintId = null, deployingRobotId = null;

async function loadFleet() {
  try {
    const [fr, br] = await Promise.all([api("/api/fleet"), api("/api/blueprints")]);
    fleetData = fr.robots || []; blueprintsData = br.blueprints || [];
    // Stats
    const total = fleetData.length;
    const online = fleetData.filter(r => r.status === "online").length;
    document.querySelector("#fsTotal").textContent = total;
    document.querySelector("#fsOnline").textContent = online;
    document.querySelector("#fsOffline").textContent = total - online;
    document.querySelector("#fsGroups").textContent = new Set(fleetData.map(r => r.group || "Default")).size;
    document.querySelector("#fsBlueprintsCount").textContent = blueprintsData.length;
    document.querySelector("#fleetCountBadge").textContent = `${total} robot${total !== 1 ? "s" : ""}`;
    renderFleetGrid(); renderBlueprintGrid(); populateBlueprintSelects();
  } catch {}
}

function renderFleetGrid() {
  const grid = document.querySelector("#fleetGrid");
  const empty = document.querySelector("#fleetEmpty");
  [...grid.querySelectorAll(".robot-card")].forEach(el => el.remove());
  let robots = currentFleetFilter === "all" ? fleetData : fleetData.filter(r => r.status === currentFleetFilter);
  if (!robots.length) { empty.style.display = ""; return; }
  empty.style.display = "none";
  robots.forEach(robot => {
    const card = document.createElement("article");
    card.className = `robot-card ${robot.status === "online" ? "rc-online" : "rc-offline"}`;
    const bp = blueprintsData.find(b => b.slug === robot.blueprint);
    card.innerHTML = `<div class="rc-head"><span class="rc-name">${escapeHtml(robot.name)}</span><span class="rc-status-dot ${robot.status}"></span></div>
      <div class="rc-info"><span class="rc-key">IP</span><span class="rc-val">${escapeHtml(robot.robotIp || "Not set")}</span>
        <span class="rc-key">Group</span><span class="rc-val">${escapeHtml(robot.group || "Default")}</span></div>
      <div><span class="rc-bp-badge">${escapeHtml(bp ? bp.icon : "◈")} ${escapeHtml(bp ? bp.name : robot.blueprint)}</span></div>
      <div class="rc-actions">
        <button class="rc-deploy" data-id="${robot.id}">▷ Deploy</button>
        <button class="rc-edit" data-id="${robot.id}">Edit</button>
        <button class="rc-del" data-id="${robot.id}">✕</button>
      </div>`;
    grid.appendChild(card);
  });
  grid.querySelectorAll(".rc-deploy").forEach(b => b.addEventListener("click", () => openDeployModal(b.dataset.id)));
  grid.querySelectorAll(".rc-edit").forEach(b => b.addEventListener("click", () => openRobotModal(b.dataset.id)));
  grid.querySelectorAll(".rc-del").forEach(b => b.addEventListener("click", async () => { if (confirm("Remove?")) { await api("/api/fleet/delete", { id: b.dataset.id }); loadFleet(); } }));
}

function renderBlueprintGrid() {
  const grid = document.querySelector("#blueprintGrid"); grid.innerHTML = "";
  blueprintsData.forEach(bp => {
    const card = document.createElement("article"); card.className = "bp-card";
    card.innerHTML = `<div class="bp-card-head"><div class="bp-card-icon" style="background:${bp.color}15;color:${bp.color}">${bp.icon || "◈"}</div>
      <div class="bp-card-title"><div class="bp-card-name">${escapeHtml(bp.name)}</div><div class="bp-card-slug">${escapeHtml(bp.slug)}</div></div>
      ${bp.builtIn ? '<span class="bp-card-builtin">BUILT-IN</span>' : ""}</div>
      ${bp.description ? `<div class="bp-card-desc">${escapeHtml(bp.description)}</div>` : ""}
      <div class="bp-card-actions"><button class="bp-edit" data-id="${bp.id}">Edit</button><button class="bp-dup" data-id="${bp.id}">Clone</button>
      ${!bp.builtIn ? `<button class="bp-del" data-id="${bp.id}">Delete</button>` : ""}</div>`;
    grid.appendChild(card);
  });
  grid.querySelectorAll(".bp-edit").forEach(b => b.addEventListener("click", () => openBlueprintModal(b.dataset.id)));
  grid.querySelectorAll(".bp-dup").forEach(b => b.addEventListener("click", async () => { await api("/api/blueprints/duplicate", { id: b.dataset.id }); loadFleet(); }));
  grid.querySelectorAll(".bp-del").forEach(b => b.addEventListener("click", async () => { if (confirm("Delete?")) { await api("/api/blueprints/delete", { id: b.dataset.id }); loadFleet(); } }));
}

function populateBlueprintSelects() {
  [document.querySelector("#rmBlueprint"), document.querySelector("#deployBlueprintSelect")].forEach(sel => {
    if (!sel) return;
    sel.innerHTML = blueprintsData.map(bp => `<option value="${escapeHtml(bp.slug)}">${escapeHtml(bp.icon)} ${escapeHtml(bp.name)}</option>`).join("");
  });
}

document.querySelectorAll("[data-fleet-filter]").forEach(btn => {
  btn.addEventListener("click", () => {
    document.querySelectorAll("[data-fleet-filter]").forEach(b => b.classList.remove("active"));
    btn.classList.add("active"); currentFleetFilter = btn.dataset.fleetFilter; renderFleetGrid();
  });
});

function closeModal(id) { document.querySelector(`#${id}`).style.display = "none"; }
document.querySelectorAll("[data-close]").forEach(btn => btn.addEventListener("click", () => closeModal(btn.dataset.close)));

function openRobotModal(robotId) {
  editingRobotId = robotId || null;
  const title = document.querySelector("#robotModalTitle");
  if (robotId) {
    const r = fleetData.find(x => x.id === robotId); if (!r) return;
    title.textContent = "Edit Robot";
    document.querySelector("#rmName").value = r.name;
    document.querySelector("#rmDeviceType").value = r.deviceType;
    document.querySelector("#rmSerial").value = r.serial || "";
    document.querySelector("#rmIp").value = r.robotIp || "";
    document.querySelector("#rmGroup").value = r.group || "Default";
    document.querySelector("#rmBlueprint").value = r.blueprint || "";
    document.querySelector("#rmNotes").value = r.notes || "";
    document.querySelector("#rmTags").value = (r.tags || []).join(", ");
  } else {
    title.textContent = "Add Robot";
    document.querySelector("#rmName").value = "";
    document.querySelector("#rmDeviceType").value = "Unitree Go2";
    document.querySelector("#rmSerial").value = "";
    document.querySelector("#rmIp").value = "";
    document.querySelector("#rmGroup").value = "Default";
    document.querySelector("#rmNotes").value = "";
    document.querySelector("#rmTags").value = "";
  }
  document.querySelector("#robotModal").style.display = "";
}

document.querySelector("#addRobotBtn")?.addEventListener("click", () => openRobotModal(null));
document.querySelector("#fleetEmptyAdd")?.addEventListener("click", () => openRobotModal(null));

document.querySelector("#robotModalSave")?.addEventListener("click", async () => {
  const payload = { name: document.querySelector("#rmName").value.trim(), deviceType: document.querySelector("#rmDeviceType").value.trim(),
    serial: document.querySelector("#rmSerial").value.trim(), robotIp: document.querySelector("#rmIp").value.trim(),
    group: document.querySelector("#rmGroup").value.trim(), blueprint: document.querySelector("#rmBlueprint").value,
    notes: document.querySelector("#rmNotes").value.trim(),
    tags: document.querySelector("#rmTags").value.split(",").map(s => s.trim()).filter(Boolean) };
  if (!payload.name) { append("Name required.", { ok: false }); return; }
  let r;
  if (editingRobotId) { payload.id = editingRobotId; r = await api("/api/fleet/update", payload); }
  else { r = await api("/api/fleet/add", payload); }
  if (r.ok) { closeModal("robotModal"); loadFleet(); }
});

function openBlueprintModal(bpId) {
  editingBlueprintId = bpId || null;
  const title = document.querySelector("#bpModalTitle");
  if (bpId) {
    const bp = blueprintsData.find(b => b.id === bpId); if (!bp) return;
    title.textContent = bp.builtIn ? "Edit Blueprint (limited)" : "Edit Blueprint";
    document.querySelector("#bpName").value = bp.name; document.querySelector("#bpName").disabled = !!bp.builtIn;
    document.querySelector("#bpSlug").value = bp.slug; document.querySelector("#bpSlug").disabled = !!bp.builtIn;
    document.querySelector("#bpDescription").value = bp.description || "";
    document.querySelector("#bpBase").value = bp.base || "unitree-go2"; document.querySelector("#bpBase").disabled = !!bp.builtIn;
    document.querySelector("#bpIcon").value = bp.icon || "◈";
    document.querySelector("#bpColor").value = bp.color || "#4090e0";
    document.querySelector("#bpExtraArgs").value = bp.extraArgs || "";
    document.querySelector("#bpModules").value = (bp.modules || []).join(", "); document.querySelector("#bpModules").disabled = !!bp.builtIn;
    document.querySelector("#bpTags").value = (bp.tags || []).join(", ");
  } else {
    title.textContent = "Create Blueprint";
    ["#bpName","#bpSlug","#bpBase","#bpModules"].forEach(s => { const e = document.querySelector(s); if (e) e.disabled = false; });
    document.querySelector("#bpName").value = ""; document.querySelector("#bpSlug").value = "";
    document.querySelector("#bpDescription").value = ""; document.querySelector("#bpBase").value = "unitree-go2";
    document.querySelector("#bpIcon").value = "◈"; document.querySelector("#bpColor").value = "#4090e0";
    document.querySelector("#bpExtraArgs").value = ""; document.querySelector("#bpModules").value = ""; document.querySelector("#bpTags").value = "";
  }
  document.querySelector("#blueprintModal").style.display = "";
}

document.querySelector("#addBlueprintBtn")?.addEventListener("click", () => openBlueprintModal(null));
document.querySelector("#bpName")?.addEventListener("input", () => {
  if (editingBlueprintId) return;
  document.querySelector("#bpSlug").value = document.querySelector("#bpName").value.toLowerCase().replace(/\s+/g, "-").replace(/[^a-z0-9_-]/g, "");
});

document.querySelector("#bpModalSave")?.addEventListener("click", async () => {
  const payload = { name: document.querySelector("#bpName").value.trim(), slug: document.querySelector("#bpSlug").value.trim(),
    description: document.querySelector("#bpDescription").value.trim(), base: document.querySelector("#bpBase").value,
    icon: document.querySelector("#bpIcon").value.trim(), color: document.querySelector("#bpColor").value,
    extraArgs: document.querySelector("#bpExtraArgs").value.trim(),
    modules: document.querySelector("#bpModules").value.split(",").map(s => s.trim()).filter(Boolean),
    tags: document.querySelector("#bpTags").value.split(",").map(s => s.trim()).filter(Boolean) };
  let r;
  if (editingBlueprintId) { payload.id = editingBlueprintId; r = await api("/api/blueprints/update", payload); }
  else { if (!payload.name) { append("Name required.", { ok: false }); return; } r = await api("/api/blueprints/create", payload); }
  if (r.ok) { closeModal("blueprintModal"); loadFleet(); }
});

function openDeployModal(robotId) {
  deployingRobotId = robotId;
  const robot = fleetData.find(r => r.id === robotId); if (!robot) return;
  document.querySelector("#deployInfo").textContent = `Deploy to "${robot.name}"`;
  document.querySelector("#deployBlueprintSelect").value = robot.blueprint || "";
  document.querySelector("#deployModal").style.display = "";
}

document.querySelector("#deployConfirm")?.addEventListener("click", async () => {
  if (!deployingRobotId) return;
  const bp = document.querySelector("#deployBlueprintSelect").value;
  const r = await api("/api/fleet/deploy", { robotId: deployingRobotId, blueprint: bp });
  if (r.ok) { closeModal("deployModal"); loadFleet(); }
});

// ── Chat panel toggle ──────────────────────────────────────────────────────

const chatToggle = document.querySelector("#chatToggle");
const appEl = document.querySelector(".app");
if (localStorage.getItem("chatCollapsed") === "1") appEl.classList.add("chat-collapsed");
chatToggle?.addEventListener("click", () => {
  appEl.classList.toggle("chat-collapsed");
  localStorage.setItem("chatCollapsed", appEl.classList.contains("chat-collapsed") ? "1" : "0");
  chatToggle.textContent = appEl.classList.contains("chat-collapsed") ? "▶" : "◀";
});
if (appEl.classList.contains("chat-collapsed")) chatToggle.textContent = "▶";

// ── Robot actions collapse ─────────────────────────────────────────────────

const robotActionsHeader = document.querySelector("#robotActionsHeader");
const robotActionsBody = document.querySelector("#robotActionsBody");
robotActionsHeader?.addEventListener("click", () => {
  robotActionsBody.classList.toggle("collapsed");
  robotActionsHeader.classList.toggle("open");
});

// ── Model failure toast ────────────────────────────────────────────────────

let lastModelSet = null;
function checkModelDrops(currentModels) {
  if (!lastModelSet) { lastModelSet = new Set(currentModels); return; }
  for (const m of lastModelSet) {
    if (!currentModels.includes(m)) {
      showToast(`${m.toUpperCase()} failed to load — not installed?`);
    }
  }
  lastModelSet = new Set(currentModels);
}

function showToast(msg) {
  const el = document.createElement("div");
  el.className = "model-toast";
  el.textContent = msg;
  document.body.appendChild(el);
  setTimeout(() => el.remove(), 4000);
}

// ── Auto-start webcam on first load ────────────────────────────────────────

async function autoStartIfNeeded() {
  try {
    const sim = await api("/api/sim/state");
    if (sim.running) { simActive = true; startStream(); return; }
    const st = await api("/api/webcam/state");
    if (!st.running && st.state !== "running") {
      await startWebcam();
    }
  } catch {}
}

// ── Init ────────────────────────────────────────────────────────────────────

refreshDashboard(true);
loadTasks();
loadEvents();
loadFleet();
loadDatasets();
autoStartIfNeeded();
setInterval(() => refreshDashboard(true), 15000);
setInterval(() => loadTasks(), 20000);
setInterval(() => loadEvents(false), 10000);
setInterval(() => loadFleet(), 30000);
