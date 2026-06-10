/* roborun flight deck — wiring. Everything on screen is real:
   MJPEG camera, SSE event bus, streaming agent, seal/verify API.
   Director keys: M record/seal · V verify · T tamper · R runs · Esc clear · ? help */

const $ = (id) => document.getElementById(id);

const GLYPHS = { mcp_tool: "⚙", detection: "◉", ros: "⬡", agent: "✦", system: "◆", task: "▶", frame: "▣" };
const MAX_FEED = 80;

let eventCount = 0;
let replaying = false;

/* ---------- canonical hash (matches roborun/integrity.py) ---------- */

function canonicalJson(obj) {
  if (obj === null || typeof obj !== "object") return JSON.stringify(obj);
  if (Array.isArray(obj)) return "[" + obj.map(canonicalJson).join(",") + "]";
  return "{" + Object.keys(obj).sort()
    .map((k) => JSON.stringify(k) + ":" + canonicalJson(obj[k])).join(",") + "}";
}

async function sha256hex(text) {
  const buf = await crypto.subtle.digest("SHA-256", new TextEncoder().encode(text));
  return [...new Uint8Array(buf)].map((b) => b.toString(16).padStart(2, "0")).join("");
}

/* ---------- black box feed ---------- */

function fmtTime(ts) {
  const d = new Date(ts * 1000);
  return d.toTimeString().slice(0, 8);
}

async function addEvent(evt, opts = {}) {
  if (!opts.replay) {
    eventCount++;
    $("bbCount").textContent = `${eventCount} events`;
  }

  const row = document.createElement("div");
  row.className = "bb-evt fresh";
  row.dataset.type = evt.type || "system";

  const title = evt.title || "";
  if (evt.source === "integrity") {
    if (/VERIFIED/.test(title)) row.classList.add("integrity-good");
    if (/FAILED|tampered/i.test(title)) row.classList.add("integrity-bad");
  }

  const hash = (await sha256hex(canonicalJson(evt))).slice(0, 8);
  // Live chain head in the header (the same hash chain the journal writes)
  if (!opts.replay) {
    $("rootLabel").textContent = "CHAIN HEAD";
    $("rootHash").textContent = hash + "…";
  }
  row.innerHTML = `
    <span class="evt-time">${fmtTime(evt.ts || Date.now() / 1000)}</span>
    <span class="evt-glyph">${GLYPHS[evt.type] || "◆"}</span>
    <span class="evt-main">
      <span class="evt-title"></span>
      <span class="evt-meta"><span>${evt.source || ""}</span><span class="evt-hash">${hash}</span></span>
    </span>`;
  row.querySelector(".evt-title").textContent = title;

  const feed = $("bbFeed");
  feed.appendChild(row);
  while (feed.children.length > MAX_FEED) feed.removeChild(feed.firstChild);
  feed.scrollTop = feed.scrollHeight;
  setTimeout(() => row.classList.remove("fresh"), 1200);
}

let _es = null;
function connectEvents() {
  _es = new EventSource("/api/events/stream");
  _es.onmessage = (m) => {
    if (replaying) return;
    try { addEvent(JSON.parse(m.data)); } catch {}
  };
  _es.onerror = () => { _es.close(); setTimeout(connectEvents, 2000); };
}

/* ---------- camera HUD ---------- */

let camAlive = false;
$("camStream").addEventListener("load", () => { camAlive = true; $("camOffline").classList.add("hidden"); });
$("camStream").addEventListener("error", () => {
  if (!camAlive) $("camOffline").classList.remove("hidden");
  setTimeout(() => { $("camStream").src = "/api/camera/stream?" + Date.now(); }, 3000);
});

async function pollHud() {
  try {
    const s = await (await fetch("/api/webcam/state")).json();
    const st = s.state || s;
    if (st.running) $("camOffline").classList.add("hidden");
    $("hudFps").textContent = `${st.fps ?? "—"} FPS`;
    const models = (st.models || []).map((m) => m.toUpperCase()).join("+") || "RAW";
    $("hudModels").textContent = models;
    $("hudDet").textContent = `${st.detections ?? 0} OBJECT${st.detections === 1 ? "" : "S"}`;
    if (st.clip_query) {
      $("hudTrack").textContent = `TRACKING: ${st.clip_query.toUpperCase()}` +
        (st.clip_matches ? ` · LOCK` : ` · SEARCHING`);
      $("hudTrack").classList.toggle("lock", !!st.clip_matches);
      $("hudTrack").classList.add("on");
    } else {
      $("hudTrack").classList.remove("on", "lock");
    }
  } catch {}
  setTimeout(pollHud, 1000);
}

/* ---------- live run id ---------- */

async function pollRunId() {
  try {
    const data = await (await fetch("/api/run/list")).json();
    const live = (data.runs || []).find((r) => r.recording);
    if (live) $("runId").textContent = live.run.toUpperCase();
  } catch {}
  setTimeout(pollRunId, 5000);
}
pollRunId();

/* ---------- agent command bar ---------- */

const input = $("cmdInput");
input.focus();

input.addEventListener("keydown", (e) => {
  if (e.key !== "Enter") return;
  const message = input.value.trim();
  if (!message) return;
  input.value = "";

  /* "track the white car" / "find the red truck" / "follow that bus"
     → zero-shot CLIP lock-on, instant. Everything else goes to the agent. */
  const m = message.match(/^(?:track|find|follow)\s+(?:the\s+|that\s+|a\s+)?(.+)$/i);
  if (m) { setTracking(m[1]); return; }
  sendCommand(message);
});

async function setTracking(query) {
  await api("/api/webcam/clip_query", { query });
  $("hudTrack").textContent = `TRACKING: ${query.toUpperCase()}`;
  $("hudTrack").classList.add("on");
}

async function sendCommand(message) {
  const strip = $("agentStrip");
  const textEl = $("agentText");
  const toolsEl = $("agentTools");
  strip.classList.add("active");
  textEl.classList.remove("done");
  textEl.textContent = "";
  toolsEl.innerHTML = "";

  let resp;
  try {
    resp = await fetch("/api/agent/chat", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ message }),
    });
  } catch (err) {
    textEl.textContent = "agent unreachable: " + err.message;
    textEl.classList.add("done");
    return;
  }

  if ((resp.headers.get("content-type") || "").includes("application/json")) {
    const j = await resp.json();
    textEl.textContent = j.error || "agent not available";
    textEl.classList.add("done");
    return;
  }

  const reader = resp.body.getReader();
  const decoder = new TextDecoder();
  let buf = "";
  while (true) {
    const { done, value } = await reader.read();
    if (done) break;
    buf += decoder.decode(value, { stream: true });
    const parts = buf.split("\n\n");
    buf = parts.pop();
    for (const part of parts) {
      const line = part.split("\n").find((l) => l.startsWith("data: "));
      if (!line) continue;
      let chunk;
      try { chunk = JSON.parse(line.slice(6)); } catch { continue; }
      handleChunk(chunk, textEl, toolsEl);
    }
  }
  textEl.classList.add("done");
}

function handleChunk(chunk, textEl, toolsEl) {
  if (chunk.type === "text") {
    textEl.textContent = chunk.accumulated || ((textEl.textContent || "") + (chunk.text || ""));
  } else if (chunk.type === "tool_use") {
    const chip = document.createElement("span");
    chip.className = "tool-chip";
    const args = chunk.tool_input ? JSON.stringify(chunk.tool_input) : "";
    chip.textContent = `⚙ ${chunk.tool_name || chunk.tool_id || "tool"} ${args.length > 48 ? args.slice(0, 48) + "…" : args}`;
    toolsEl.appendChild(chip);
    while (toolsEl.children.length > 4) toolsEl.removeChild(toolsEl.firstChild);
  } else if (chunk.type === "done") {
    if (chunk.text) textEl.textContent = chunk.text;
  } else if (chunk.type === "error") {
    textEl.textContent = "error: " + chunk.error;
  }
}

/* ---------- seal / verify / tamper (director keys) ---------- */

async function api(path, body) {
  const r = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
  return r.json();
}

function showStamp(word, sub, meta, bad) {
  const layer = $("stampLayer");
  const stamp = $("stamp");
  $("stampWord").textContent = word;
  $("stampSub").textContent = sub || "";
  $("stampMeta").textContent = meta || "";
  stamp.classList.toggle("bad", !!bad);
  layer.classList.remove("show", "shake");
  void layer.offsetWidth; /* restart animation */
  layer.classList.add("show");
  if (bad) layer.classList.add("shake");
}

function hideStamp() { $("stampLayer").classList.remove("show", "shake"); }

async function doVerify() {
  const r = await api("/api/run/mcap/verify");
  if (!r.ok) { showStamp("NO RUN", r.error || "record a run first (M)", "", true); return; }
  if (r.state === "broken") {
    showStamp("BROKEN", r.reason || "integrity failure",
      r.byte_range ? `bytes ${r.byte_range[0]}–${r.byte_range[1]}` : "", true);
  } else if (r.state === "verified_anchored") {
    showStamp("VERIFIED", `${r.run} · anchored to an external clock`,
      `${r.reason} · merkle root ${(r.merkle_root || "").slice(0, 24)}…`);
  } else {
    showStamp("CONSISTENT", `${r.run} · chain + seal intact · unanchored`, r.reason || "");
  }
  pollBadgeSoon();
}

async function doTamper() {
  const r = await api("/api/run/mcap/tamper");
  if (!r.ok) { showStamp("NO RUN", r.error || "record a run first (M)", "", true); return; }
  showStamp("TAMPERED", `${r.run} · byte ${r.byte} flipped`,
    "now press V to verify", true);
  setTimeout(hideStamp, 2200);
  pollBadgeSoon();
}

document.addEventListener("keydown", (e) => {
  if (e.target === input && e.key !== "Escape") return;
  if (e.target.tagName === "INPUT") return;
  switch (e.key) {
    case "v": case "V": doVerify(); break;
    case "t": case "T": doTamper(); break;
    case "m": case "M": toggleRecord(); break;
    case "r": case "R": toggleDrawer(); break;
    case "c": case "C": toggleDrawer(); break;
    case "Escape": hideStamp(); closeDrawer(); replaying = false; input.blur(); break;
    case "?": $("director").classList.toggle("hidden"); break;
  }
});

/* ---------- MCAP recording: badge, record toggle, verified clips ---------- */

let mcapRecording = false;

const BADGE_STATES = {
  recording:  { text: "● REC MCAP",   cls: "rec" },
  anchored:   { text: "✓ ANCHORED",   cls: "good" },
  unanchored: { text: "◇ UNANCHORED", cls: "warn" },
  broken:     { text: "✕ BROKEN",     cls: "bad" },
  none:       { text: "—",            cls: "" },
};

async function pollBadge() {
  try {
    const b = await (await fetch("/api/run/badge")).json();
    mcapRecording = b.badge === "recording";
    const s = BADGE_STATES[b.badge] || BADGE_STATES.none;
    const el = $("verifyBadge");
    el.textContent = s.text;
    el.className = "verify-badge " + s.cls;
    el.title = b.reason || (b.badge === "recording"
      ? `recording ${b.run} · ${Object.values(b.messages || {}).reduce((a, n) => a + n, 0)} msgs`
      : "latest recording: verify state");
  } catch {}
  setTimeout(pollBadge, 8000);
}

async function toggleRecord() {
  if (mcapRecording) {
    const r = await api("/api/run/record/stop");
    if (!r.ok) { showStamp("NO RUN", r.error || "", "", true); return; }
    mcapRecording = false;
    const anchor = (r.seal.anchor || {}).status || "unanchored";
    showStamp("SEALED",
      `${r.seal.run} · ${r.seal.segment_count} chunks · merkle root · ${anchor === "anchored" ? "anchored · rfc 3161 timestamp" : anchor}`,
      `${r.seal.merkle_root.slice(0, 32)}… · ${r.indexed && r.indexed.ok ? r.indexed.observations + " observations indexed" : ""}`);
    setTimeout(hideStamp, 3000);
  } else {
    const r = await api("/api/run/record/start");
    if (!r.ok) { showStamp("ERROR", r.error || "", "", true); return; }
    mcapRecording = true;
    showStamp("RECORDING", `${r.run} · camera + detections + events → one sealed mcap`, "press M again to seal + anchor");
    setTimeout(hideStamp, 2200);
  }
  pollBadgeSoon();
  loadMcapRuns();
}

function pollBadgeSoon() { setTimeout(async () => {
  try {
    const b = await (await fetch("/api/run/badge")).json();
    mcapRecording = b.badge === "recording";
    const s = BADGE_STATES[b.badge] || BADGE_STATES.none;
    $("verifyBadge").textContent = s.text;
    $("verifyBadge").className = "verify-badge " + s.cls;
  } catch {}
}, 700); }

async function loadMcapRuns() {
  let data;
  try { data = await (await fetch("/api/run/mcap")).json(); } catch { return; }
  const list = $("mcapList");
  if (!list) return;
  list.innerHTML = "";
  const runs = (data.runs || []).slice(0, 8);
  if (!runs.length && !data.recording) {
    list.innerHTML = `<div class="run-empty">no recordings — press M to start one</div>`;
    return;
  }
  if (data.recording) {
    const row = document.createElement("div");
    row.className = "run-row";
    row.innerHTML = `<span class="run-name">${data.recording.run.replace("run_", "")}</span>
      <span class="run-badge rec">REC</span>
      <span class="run-actions"><button class="src-btn" data-act="stop">seal</button></span>`;
    row.querySelector('[data-act="stop"]').addEventListener("click", toggleRecord);
    list.appendChild(row);
  }
  for (const r of runs) {
    const row = document.createElement("div");
    row.className = "run-row";
    const mb = (r.size / 1048576).toFixed(1);
    const badges = [
      r.anchored ? `<span class="run-badge sealed">TSA</span>` : "",
      r.sealed ? `<span class="run-badge sealed">SEALED</span>` : "",
    ].join("");
    row.innerHTML = `
      <span class="run-name">${r.robot_id}/${r.run.replace("run_", "")}</span>
      <span class="run-n">${mb} MB</span>${badges}
      <span class="run-actions">
        <button class="src-btn" data-act="mverify">verify</button>
        ${r.sealed ? `<button class="src-btn" data-act="mclip" title="export last 30s with proof">clip</button>` : ""}
      </span>`;
    row.querySelector('[data-act="mverify"]').addEventListener("click", async () => {
      const v = await api("/api/run/mcap/verify", { run: r.run, robot_id: r.robot_id });
      closeDrawer();
      if (v.state === "broken") {
        showStamp("BROKEN", v.reason || "integrity failure", "", true);
      } else if (v.state === "verified_anchored") {
        showStamp("VERIFIED", `${r.run} · anchored to an external clock`,
          `${v.reason} · merkle root ${ (v.merkle_root || "").slice(0, 24)}…`);
      } else {
        showStamp("CONSISTENT", `${r.run} · chain + seal intact · unanchored`,
          v.reason || "");
      }
    });
    row.querySelector('[data-act="mclip"]')?.addEventListener("click", async () => {
      // verified clip: last 30 seconds of the run, exported with proof
      const now = Date.now() / 1000;
      const v = await api("/api/run/mcap/clip",
        { run: r.run, robot_id: r.robot_id, start_ts: now - 86400, end_ts: now });
      closeDrawer();
      if (v.ok) {
        showStamp("CLIP CUT", `${v.messages} messages · proof binds it to the sealed run`,
          v.clip.split("/").pop());
        setTimeout(hideStamp, 3000);
      } else {
        showStamp("FAILED", v.error || "clip export failed", "", true);
      }
    });
    list.appendChild(row);
  }
}

/* ---------- drawer: sources + runs ---------- */

function toggleDrawer() {
  const open = $("drawer").classList.toggle("open");
  if (open) { refreshSources(); loadRuns(); loadMcapRuns(); }
}
function closeDrawer() { $("drawer").classList.remove("open"); }
$("btnSources").addEventListener("click", toggleDrawer);
$("btnRuns").addEventListener("click", toggleDrawer);

function srcDot(el, on) { el.classList.toggle("on", !!on); }

async function refreshSources() {
  try {
    const wc = await (await fetch("/api/webcam/state")).json();
    const wcOn = (wc.state || wc).running;
    srcDot($("dotWebcam"), wcOn);
    $("srcWebcam").textContent = wcOn ? "stop" : "start";
  } catch {}
  try {
    const sim = await (await fetch("/api/sim/state")).json();
    const simOn = (sim.state || sim).running;
    srcDot($("dotSim"), simOn);
    $("srcSim").textContent = simOn ? "stop" : "start";
  } catch {}
  try {
    const ros = await (await fetch("/api/ros/status")).json();
    const rosOn = ros.connected || (ros.status || {}).connected;
    srcDot($("dotRobot"), rosOn);
    $("srcRobot").textContent = rosOn ? "disconnect" : "connect";
  } catch {}
}

$("srcWebcam").addEventListener("click", async () => {
  const stopping = $("srcWebcam").textContent === "stop";
  await api(stopping ? "/api/webcam/stop" : "/api/webcam/start",
            stopping ? {} : { camera_index: 0, models: ["yolo"] });
  setTimeout(refreshSources, 600);
});
$("srcSim").addEventListener("click", async () => {
  const stopping = $("srcSim").textContent === "stop";
  await api(stopping ? "/api/sim/stop" : "/api/sim/start");
  setTimeout(refreshSources, 600);
});
$("srcRobot").addEventListener("click", async () => {
  const disconnecting = $("srcRobot").textContent === "disconnect";
  if (disconnecting) { await api("/api/ros/disconnect"); }
  else {
    const ip = $("robotIp").value.trim();
    if (!ip) { $("robotIp").focus(); return; }
    await api("/api/ros/connect", { host: ip, port: 9090 });
  }
  setTimeout(refreshSources, 800);
});

async function loadRuns() {
  let data;
  try { data = await (await fetch("/api/run/list")).json(); } catch { return; }
  const list = $("runList");
  list.innerHTML = "";
  const runs = (data.runs || []).slice(-12).reverse();
  if (!runs.length) {
    list.innerHTML = `<div class="run-empty">no timeline sessions yet</div>`;
    return;
  }
  for (const r of runs) {
    const row = document.createElement("div");
    row.className = "run-row";
    const badges = [
      r.recording ? `<span class="run-badge rec">REC</span>` : "",
    ].join("");
    row.innerHTML = `
      <span class="run-name">${r.run.replace("run_", "")}</span>
      <span class="run-n">${r.events} ev</span>${badges}
      <span class="run-actions">
        ${!r.recording ? `<button class="src-btn" data-act="replay">replay</button>` : ""}
      </span>`;
    row.querySelector('[data-act="replay"]')?.addEventListener("click", () => replayRun(r.run));
    list.appendChild(row);
  }
}

/* ---------- replay: stream a recorded run back through the feed ---------- */

async function replayRun(name) {
  let data;
  try { data = await (await fetch(`/api/run/events?run=${encodeURIComponent(name)}`)).json(); } catch { return; }
  if (!data.ok) return;
  closeDrawer();
  replaying = true;
  const feed = $("bbFeed");
  feed.innerHTML = "";
  $("bbCount").textContent = `REPLAY · ${name}`;
  document.body.classList.add("replaying");
  const events = data.events;
  const step = Math.max(18, Math.min(90, 4000 / events.length));
  for (const evt of events) {
    if (!replaying) break; // Esc aborts
    await addEvent(evt, { replay: true });
    await new Promise((res) => setTimeout(res, step));
  }
  document.body.classList.remove("replaying");
  replaying = false;
  feed.innerHTML = "";
  $("bbCount").textContent = `${eventCount} events`;
  // live SSE feed resumes on the next event
}

/* clicking anywhere outside input refocuses it (clean recording) */
document.addEventListener("click", (e) => {
  if (!e.target.closest(".director")) input.focus();
});

/* ?do=verify — runs the real verify API on load (for screenshots) */
const act = new URLSearchParams(location.search).get("do");
if (act === "verify") setTimeout(doVerify, 1500);

/* ?stamp=verified|failed|sealed — layout preview only, clearly labeled */
const preview = new URLSearchParams(location.search).get("stamp");
if (preview === "verified") showStamp("VERIFIED", "412 chunks · anchored to an external clock", "PREVIEW — not a real verification");
if (preview === "failed") showStamp("BROKEN", "segment 0042 hash mismatch", "PREVIEW — not a real verification", true);
if (preview === "sealed") showStamp("SEALED", "412 chunks · merkle root computed", "PREVIEW — not a real seal");

connectEvents();
pollHud();
pollBadge();
