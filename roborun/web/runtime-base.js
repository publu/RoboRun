/* Runtime discovery: one UI, two power levels.
 *
 * Served by roborun itself, /api/* is same-origin and everything just works.
 * Served from a static host (GitHub Pages, `python -m http.server -d site`),
 * the same page probes for a local roborun runtime on 127.0.0.1:8765 and, if
 * one answers, routes every /api call to it — the demo page becomes the live
 * cockpit. No runtime anywhere → the in-browser arena (Pyodide) carries on.
 *
 * Loaded before arena.js/deck.js; wraps window.fetch so existing relative
 * "/api/…" calls need no changes. Queues /api calls until probing settles.
 */
(() => {
  const LOCAL = "http://127.0.0.1:8765";
  const state = { base: "", live: false, remote: false };
  window.ROBORUN_RUNTIME = state;

  const origFetch = window.fetch.bind(window);

  const probe = async (base) => {
    try {
      const ctl = new AbortController();
      const t = setTimeout(() => ctl.abort(), 1500);
      const r = await origFetch(base + "/api/health", { signal: ctl.signal });
      clearTimeout(t);
      return r.ok;
    } catch {
      return false;
    }
  };

  const resolved = (async () => {
    if (await probe("")) {
      state.live = true;
    } else if (location.port !== "8765" && await probe(LOCAL)) {
      state.base = LOCAL;
      state.live = state.remote = true;
    }
    document.dispatchEvent(new CustomEvent("roborun-runtime", { detail: state }));
    return state;
  })();

  window.fetch = async (input, init) => {
    if (typeof input === "string" && input.startsWith("/api")) {
      await resolved;
      return origFetch(state.base + input, init);
    }
    return origFetch(input, init);
  };

  // ── status badge ─────────────────────────────────────────────────────
  const badge = () => {
    const el = document.createElement("div");
    el.id = "runtime-badge";
    el.style.cssText =
      "position:fixed;right:12px;bottom:12px;z-index:9999;padding:4px 10px;" +
      "border-radius:12px;font:11px/1.6 ui-monospace,Menlo,monospace;" +
      "background:#11161bcc;border:1px solid #2a333d;color:#8a96a3;" +
      "backdrop-filter:blur(4px);cursor:default;user-select:none;";
    document.body.appendChild(el);

    const render = () => {
      if (state.live) {
        el.style.color = "#00d47e";
        el.style.borderColor = "#00d47e55";
        el.textContent = state.remote ? "● live — local runtime :8765" : "● live";
        el.style.cursor = "default";
        el.onclick = null;
      } else {
        el.style.color = "#8a96a3";
        el.textContent = "○ demo — run `roborun` to go live";
      }
    };

    resolved.then(render);

    // demo mode: keep listening so starting `roborun` upgrades the page
    const recheck = async () => {
      if (state.live) return;
      if (await probe(LOCAL)) {
        el.style.color = "#00d47e";
        el.style.borderColor = "#00d47e55";
        el.style.cursor = "pointer";
        el.textContent = "● runtime found — click to go live";
        el.onclick = () => location.reload();
      } else {
        setTimeout(recheck, 5000);
      }
    };
    resolved.then(() => { if (!state.live) setTimeout(recheck, 5000); });
  };

  if (document.body) badge();
  else document.addEventListener("DOMContentLoaded", badge);
})();

/* ── usage analytics (Vercel Web Analytics — events only, never video) ──
 * Lives here, not in arena.js, so it survives arena rewrites and covers
 * every page at once. Counts plays + connect-intent, broken down by
 * visitor and country in the Vercel dashboard. No recording, no storage,
 * no cost. Skipped on localhost so dev runs don't pollute the numbers.
 */
(() => {
  const host = location.hostname;
  if (host === "localhost" || host === "127.0.0.1" || host === "") return;

  window.va = window.va || function () { (window.vaq = window.vaq || []).push(arguments); };
  const s = document.createElement("script");
  s.defer = true;
  s.src = "/_vercel/insights/script.js";
  document.head.appendChild(s);

  const ev = (name, data) => {
    try { window.va("event", data ? { name, data } : { name }); } catch {}
  };

  const wire = () => {
    document.getElementById("btnRun")
      ?.addEventListener("click", () => ev("play"), true);
    document.getElementById("btnConnect")
      ?.addEventListener("click", () => ev("connect_intent"), true);
    // start-screen task buttons are generated at runtime — catch by delegation
    document.addEventListener("click", (e) => {
      const b = e.target.closest?.("#startGrid button");
      if (b) ev("level_start", { task: b.textContent.trim().slice(0, 40) });
    }, true);
  };
  if (document.readyState === "loading")
    document.addEventListener("DOMContentLoaded", wire);
  else wire();
})();
