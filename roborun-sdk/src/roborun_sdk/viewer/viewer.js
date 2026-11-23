// This robot's own viewer: served by the page connector on this computer (http://127.0.0.1:8135/), read-only plus Stop.
// It reads the local robot API through /v/<key>/ (a key this page gets from /view/session, same origin only) and shows only what
// the robot's own cameras saw: live frames, recent tasks and their recordings.
const $ = (sel) => document.querySelector(sel);
const S = { base: '', humanoid: false, cams: [], tasks: [], selected: null, rec: null, frame: 0, playing: null, live: {} };

function el(tag, attrs = {}, ...kids) {
  const node = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (v === undefined || v === null) continue;
    if (k === 'text') node.textContent = v; else node.setAttribute(k, v);
  }
  for (const kid of kids) if (kid !== null && kid !== undefined) node.append(kid);
  return node;
}

async function api(path, options = {}) {
  const reply = await fetch(S.base + path, { cache: 'no-store', ...options });
  let body = null;
  try { body = await reply.json(); } catch { body = null; }
  if (!reply.ok) {
    const err = body && body.error;
    throw new Error((err && (err.message || err)) || `HTTP ${reply.status}`);
  }
  return body;
}

function when(value) {
  if (!value) return '';
  const t = typeof value === 'number' ? new Date(value * 1000) : new Date(value);
  return isNaN(t) ? '' : t.toLocaleString();
}

function setState(text, tone) {
  const chip = $('#state');
  chip.textContent = text;
  chip.dataset.tone = tone || '';
}

// ---------------------------------------------------------------- live
function fact(dl, name, value) {
  if (value === undefined || value === null || value === '') return;
  dl.append(el('dt', { text: name }), el('dd', { text: String(value) }));
}

function odometry(state) {
  const b = (state && state.base) || {};
  const h = (state && state.head) || {};
  const n = (v, d = 2) => (typeof v === 'number' ? v.toFixed(d) : '?');
  const deg = (v) => (typeof v === 'number' ? Math.round(v * 180 / Math.PI) + '°' : '?');
  return { base: `x ${n(b.x)} m, y ${n(b.y)} m, heading ${deg(b.yaw)}`, head: `pitch ${deg(h.pitch)}, yaw ${deg(h.yaw)}` };
}

async function refreshStatus() {
  let st;
  try {
    st = await api('/v1/status');
  } catch (e) {
    setState('Not answering', 'bad');
    return;
  }
  setState(st.ready === false ? 'Starting' : 'Ready', st.ready === false ? '' : 'ok');
  const dl = $('#facts');
  dl.replaceChildren();
  fact(dl, 'Profile', st.label ? `${st.label} (${st.profile})` : st.profile);
  if (st.simulated || st.simulation) fact(dl, 'Simulation', st.scene || st.simulation || 'yes');
  if (st.arms && Object.keys(st.arms).length) {
    fact(dl, 'Arms', Object.entries(st.arms).map(([a, v]) => `${a} ${v && v.ok ? 'ok' : 'not ready'}`).join(', '));
  }
  if (st.overhead_camera && st.overhead_camera.message) fact(dl, 'Overhead camera', st.overhead_camera.message);
  if (st.dataset && st.dataset.message) fact(dl, 'Dataset', st.dataset.message);   // only while episodes were refused
  if (st.update && st.update.available) fact(dl, 'Update', `a newer Roborun is out: roborun upgrade`);
  const task = st.task;
  fact(dl, 'Now', task && task.id ? `${task.status || 'running'}: ${task.prompt || task.id}` : 'nothing running');
  if (S.humanoid) {
    const o = odometry(st);
    fact(dl, 'Base', o.base);
    fact(dl, 'Head', o.head);
  }
  const cams = S.humanoid ? ['head'] : Object.keys(st.cameras || {});
  if (cams.join() !== S.cams.join()) {
    S.cams = cams;
    const box = $('#cams');
    box.replaceChildren(...cams.map((c) => el('figure', { class: 'cam' },
      el('img', { id: `cam-${c}`, alt: `${c} camera, live` }), el('figcaption', { text: `${c} camera` }))));
    if (!cams.length) box.append(el('p', { class: 'muted', text: 'This robot has no cameras to show yet.' }));
  }
}

function refreshCamera(name) {
  if (document.hidden || S.live[name]) return;
  const img = document.getElementById(`cam-${name}`);
  if (!img) return;
  S.live[name] = true;
  const next = new Image();
  const done = () => { S.live[name] = false; };
  next.onload = () => { img.src = next.src; done(); };
  next.onerror = done;
  next.src = `${S.base}/v1/cameras/${encodeURIComponent(name)}.jpg?t=${Date.now()}`;
}

// ---------------------------------------------------------------- tasks
function describeTask(t) {
  if (S.humanoid) {
    const played = t.played || {};
    const bits = [t.stage && t.status === 'running' ? t.stage : null, played.played_s ? `played ${played.played_s.toFixed(1)} s` : null,
      t.error && t.error.message];
    return bits.filter(Boolean).join(' · ');
  }
  const moves = t.moves && !String(t.message || '').includes(`${t.moves} move`) ? `${t.moves} move${t.moves === 1 ? '' : 's'}` : null;
  return [t.message, moves].filter(Boolean).join(' · ');
}

async function refreshTasks() {
  let rows;
  try {
    rows = (await api('/v1/tasks?limit=30')).tasks || [];
  } catch (e) {
    return;
  }
  S.tasks = rows;
  const list = $('#tasks');
  if (!rows.length) {
    list.replaceChildren(el('li', { class: 'muted', text: 'No tasks yet. Tasks you start from the page, the local API or your own code show here.' }));
    return;
  }
  list.replaceChildren(...rows.map((t) => {
    const button = el('button', { type: 'button', 'aria-current': S.selected === t.id ? 'true' : 'false' },
      el('span', { class: 'top' }, el('span', { class: 'prompt', text: t.prompt || t.id }),
        el('span', { class: 'chip', 'data-tone': t.status, text: t.status })),
      el('span', { class: 'muted small', text: [when(t.created || t.created_at), describeTask(t)].filter(Boolean).join(' · ') }));
    button.addEventListener('click', () => select(t.id));
    return el('li', {}, button);
  }));
  const linked = (location.hash.match(/^#task=([A-Za-z0-9_-]+)$/) || [])[1];
  if (linked && !S.selected && rows.some((r) => r.id === linked)) return select(linked);    // a link to one task: #task=<id>
  if (S.selected) {
    const t = rows.find((r) => r.id === S.selected);
    if (t) showDetail(t, false);
  }
}

function outcomeText(outcome) {
  if (!outcome) return null;
  const parts = [];
  for (const [arm, v] of Object.entries(outcome.holding || {})) {
    if (v === null) parts.push(`${arm}: empty`);
    else if (v === 'unknown') parts.push(`${arm}: not checked`);
    else if (typeof v === 'object') parts.push(`${arm}: holding ${v.object || 'an object'}${v.verified ? ' (verified)' : ''}`);
  }
  for (const p of Array.isArray(outcome.placed) ? outcome.placed : []) {
    parts.push(`${p.arm || ''} placed ${p.object || 'it'}${p.destination ? ' at the ' + p.destination : ''}${p.verified ? ' (verified)' : ''}`.trim());
  }
  return parts.join('; ') || null;
}

function showDetail(t, fresh) {
  const box = $('#detail');
  box.hidden = false;
  $('#detail-h').textContent = t.prompt || 'Task';
  const dl = el('dl', { class: 'kv' });
  fact(dl, 'Status', t.status);
  fact(dl, 'Started', when(t.created || t.created_at));
  if (t.elapsed_s) fact(dl, 'Took', `${Math.round(t.elapsed_s)} s`);
  fact(dl, 'Message', t.message);
  fact(dl, 'Outcome', outcomeText(t.outcome));
  if (t.error_info) fact(dl, 'Error', `${t.error_info.code}${t.error_info.next ? ': ' + t.error_info.next : ''}`);
  else if (t.error) fact(dl, 'Error', typeof t.error === 'object' ? t.error.message : t.error);
  if (S.humanoid) {
    const p = t.played || {};
    if (p.odometry_start && p.odometry_end) {
      const a = p.odometry_start, b = p.odometry_end;
      fact(dl, 'Moved', `${Math.hypot((b.x || 0) - (a.x || 0), (b.y || 0) - (a.y || 0)).toFixed(2)} m in ${(p.played_s || 0).toFixed(1)} s`);
    }
    if (Array.isArray(t.deferred) && t.deferred.length) fact(dl, 'Left for a next move', t.deferred.join(', '));
  }
  $('#detail-body').replaceChildren(dl);
  if (fresh && !S.humanoid) loadRecording(t);
  if (S.humanoid) $('#player').hidden = true;
}

function select(id) {
  S.selected = id;
  try { history.replaceState(null, '', `#task=${id}`); } catch {}
  stopPlaying();
  const t = S.tasks.find((r) => r.id === id);
  for (const b of document.querySelectorAll('#tasks button')) b.setAttribute('aria-current', 'false');
  if (t) showDetail(t, true);
  refreshTasks();
}

// ---------------------------------------------------------------- recordings: the robot's own camera frames
async function loadRecording(t) {
  const player = $('#player');
  S.rec = null;
  player.hidden = true;
  if (t.status === 'running') return;
  let rec;
  try {
    rec = await api(`/v1/tasks/${encodeURIComponent(t.id)}/recording`);
  } catch (e) {
    $('#detail-body').append(el('p', { class: 'muted small', text: 'No recording for this task.' }));
    return;
  }
  const r = (rec.recordings || []).at(-1);
  if (!r || !r.cameras || !Object.keys(r.cameras).length) return;
  S.rec = r;
  const pick = $('#rec-cam');
  pick.replaceChildren(...Object.keys(r.cameras).map((c) => el('option', { value: c, text: `${c} (${r.cameras[c]} frames)` })));
  pick.value = Object.keys(r.cameras).includes('top') ? 'top' : Object.keys(r.cameras)[0];
  player.hidden = false;
  seekTo(0);
}

function frameCount() {
  return S.rec ? (S.rec.cameras[$('#rec-cam').value] || 0) : 0;
}

function seekTo(n) {
  if (!S.rec) return;
  const count = frameCount();
  S.frame = Math.max(0, Math.min(count - 1, n));
  const seek = $('#rec-seek');
  seek.max = String(Math.max(0, count - 1));
  seek.value = String(S.frame);
  const path = S.rec.frame_url.replace('{camera}', encodeURIComponent($('#rec-cam').value)).replace('{frame}', String(S.frame));
  $('#rec-img').src = S.base + path;
  $('#rec-pos').textContent = `${(S.frame / (S.rec.hz || 10)).toFixed(1)} s / ${(count / (S.rec.hz || 10)).toFixed(1)} s`;
}

function stopPlaying() {
  if (S.playing) clearInterval(S.playing);
  S.playing = null;
  $('#rec-play').textContent = 'Play';
}

function togglePlay() {
  if (S.playing) return stopPlaying();
  if (S.frame >= frameCount() - 1) seekTo(0);
  $('#rec-play').textContent = 'Pause';
  S.playing = setInterval(() => {
    if (S.frame >= frameCount() - 1) return stopPlaying();
    seekTo(S.frame + 1);
  }, 1000 / Math.min(30, (S.rec && S.rec.hz) || 10));
}

// ---------------------------------------------------------------- Stop
async function stop() {
  const said = $('#stop-said');
  said.hidden = false;
  said.textContent = 'Stopping…';
  try {
    const out = await api('/v1/stop', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' });
    said.textContent = out.message || 'Stop sent: the robot is holding where it is.';
  } catch (e) {
    said.textContent = `The Stop could not be confirmed (${e.message}). If anything still moves, support the arm or cut its power.`;
  }
}

// ---------------------------------------------------------------- start
function wire() {
  $('#stop').addEventListener('click', stop);
  $('#rec-play').addEventListener('click', togglePlay);
  $('#rec-seek').addEventListener('input', (e) => { stopPlaying(); seekTo(Number(e.target.value)); });
  $('#rec-cam').addEventListener('change', () => seekTo(S.frame));
}

async function start() {
  try {
    const session = await (await fetch('/view/session', { cache: 'no-store' })).json();
    S.base = `/v/${session.key}`;
    if (session.robot) $('#robot-name').textContent = session.robot;
  } catch (e) {
    setState('Not answering', 'bad');
    setTimeout(start, 3000);
    return;
  }
  try {
    await api('/v1/body');
    S.humanoid = true;
  } catch (e) {
    S.humanoid = false;
  }
  await refreshStatus();
  await refreshTasks();
  setInterval(refreshStatus, 2000);
  setInterval(refreshTasks, 4000);
  setInterval(() => S.cams.forEach(refreshCamera), 700);
}

wire();
start();
