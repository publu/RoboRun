/* shell.js — the persistent app shell (sidebar + top bar) for every dashboard
 * page. The project/environment switcher in the top bar is the spine: it scopes
 * all data (the server scopes via the active project). Pages just load this
 * script; the shell removes their bespoke header/nav, wraps their <main>, and
 * provides one consistent chrome. The cockpit (/sim) does NOT use the shell.
 */
(function () {
  const PATH = location.pathname.replace(/\/$/, "") || "/";
  const NAV = [
    { items: [{ icon: "⌂", label: "Home", href: "/" }] },
    { group: "Build", items: [
      { icon: "▣", label: "Sims & Robots", href: "/setup" },
      { icon: "▦", label: "Cockpit", href: "/sim" }] },
    { group: "Data", items: [
      { icon: "⊞", label: "Data Browser", href: "/browser" },
      { icon: "✦", label: "Scenarios", href: "/scenarios" },
      { icon: "🕓", label: "Timeline", href: "/timeline" },
      { icon: "⌕", label: "Search", href: "/search" },
      { icon: "📊", label: "Analytics", href: "/analytics" }] },
    { group: "Fleet", items: [
      { icon: "◈", label: "Swarm Lab", href: "/fleet" },          // layer 1: coordination algorithms
      { icon: "🐝", label: "Fleet Sim", href: "/fleet-sim" }] },   // layer 2: real Rapier physics
    { items: [{ icon: "⚙", label: "Projects", href: "/projects" }] },
  ];
  const isActive = (href) => href === "/" ? PATH === "/" :
    (PATH === href || PATH.startsWith(href + "/"));

  function sidebarHTML() {
    return NAV.map(sec =>
      (sec.group ? `<div class="nav-group">${sec.group}</div>` : "") +
      sec.items.map(it =>
        `<a class="nav-item${isActive(it.href) ? " on" : ""}" href="${it.href}">
           <span class="ni-ic">${it.icon}</span><span class="ni-l">${it.label}</span></a>`).join("")
    ).join("");
  }

  function build() {
    // drop any bespoke page header/nav — the shell owns navigation
    document.querySelectorAll("body > header, body > nav").forEach(h => h.remove());
    // capture ALL remaining page content (main + any stray bars/divs), not just
    // <main>, so nothing gets orphaned outside the shell
    const content = [...document.body.children].filter(el =>
      el.tagName !== "SCRIPT" && el.id !== "runtime-badge");

    const title = (document.title || "RoboRun").replace(/^RoboRun\s*[·|-]\s*/i, "");
    const shell = document.createElement("div");
    shell.className = "app-shell";
    shell.innerHTML = `
      <aside class="app-side">
        <a class="app-brand" href="/">◇ <b>RoboRun</b></a>
        <nav class="app-nav">${sidebarHTML()}</nav>
        <div class="app-side-foot" id="shellLive">○ checking…</div>
      </aside>
      <div class="app-col">
        <header class="app-top">
          <div class="app-title">${title}</div>
          <div class="app-top-r">
            <div class="scope-wrap">
              <button class="scope-btn" id="scopeBtn" title="active project / environment — data scopes here">◇ scratch ▾</button>
              <div class="scope-menu" id="scopeMenu" style="display:none"></div>
            </div>
            <a class="btn sm accent" href="/setup">+ New</a>
          </div>
        </header>
        <div class="app-main" id="appMain"></div>
      </div>`;
    document.body.insertBefore(shell, document.body.firstChild);
    const host = shell.querySelector("#appMain");
    content.forEach(el => host.appendChild(el));

    wireScope();
    wireLive();
  }

  // ── project / environment switcher ───────────────────────────────────────
  async function wireScope() {
    const btn = document.getElementById("scopeBtn");
    const menu = document.getElementById("scopeMenu");
    async function refresh() {
      try {
        const a = await (await fetch("/api/projects/active")).json();
        btn.textContent = a.active ? `◆ ${a.active.project} / ${a.active.environment} ▾` : "◇ scratch ▾";
        btn.classList.toggle("scoped", !!a.active);
      } catch { btn.textContent = "○ offline ▾"; }
    }
    async function openMenu() {
      let html = "";
      try {
        const d = await (await fetch("/api/projects")).json();
        const act = d.active;
        if (!d.projects || !d.projects.length) {
          html = `<div class="scope-empty">No projects yet — everything is in <b>scratch</b>.</div>`;
        } else {
          for (const p of d.projects) {
            const envs = await (await fetch("/api/environments?project=" + encodeURIComponent(p.id))).json();
            html += `<div class="scope-proj">${p.name}</div>`;
            html += (envs.environments || []).map(e => {
              const on = act && act.project === p.id && act.environment === e.id;
              return `<a class="scope-env${on ? " on" : ""}" data-p="${p.id}" data-e="${e.id}">
                ${e.name} <span class="se-b">${e.backend}·${e.mode}</span></a>`;
            }).join("");
          }
        }
      } catch { html = `<div class="scope-empty">○ run <code>roborun</code> to go live.</div>`; }
      html += `<div class="scope-acts"><a href="/setup">+ new sim / robot</a>
               <a id="scopeClear">use scratch</a></div>`;
      menu.innerHTML = html;
      menu.style.display = "block";
      menu.querySelectorAll(".scope-env").forEach(a => a.onclick = async () => {
        await fetch("/api/projects/active", { method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ project: a.dataset.p, environment: a.dataset.e }) });
        location.reload();
      });
      const clr = document.getElementById("scopeClear");
      if (clr) clr.onclick = async () => {
        await fetch("/api/projects/active/clear", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
        location.reload();
      };
    }
    btn.onclick = (e) => { e.stopPropagation(); if (menu.style.display === "none") openMenu(); else menu.style.display = "none"; };
    document.addEventListener("click", () => { menu.style.display = "none"; });
    menu.addEventListener("click", e => e.stopPropagation());
    refresh();
  }

  function wireLive() {
    const el = document.getElementById("shellLive");
    const render = (s) => {
      if (s && s.live) { el.innerHTML = '● <span style="color:var(--accent)">live</span>' + (s.remote ? " · :8765" : ""); }
      else { el.innerHTML = '○ <a href="/sim" style="color:var(--accent)">try the sim</a> · run <code>roborun</code>'; }
    };
    if (window.ROBORUN_RUNTIME) render(window.ROBORUN_RUNTIME);
    document.addEventListener("roborun-runtime", e => render(e.detail));
    // fallback probe
    fetch("/api/health").then(r => render({ live: r.ok })).catch(() => render({ live: false }));
  }

  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", build);
  else build();
})();
