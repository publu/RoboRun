/* project-chip.js — a live "where am I recording" indicator on every dashboard.
 *
 * The whole point of projects/environments is data isolation; if you can't SEE
 * which project a page is scoped to, the feature is invisible. This injects a
 * chip into the page nav showing the active project/environment (or "scratch"),
 * linking to /projects to switch. Loaded by every dashboard; the cockpit has its
 * own toolbar chip already.
 */
(function () {
  function render(active) {
    if (document.querySelector(".proj-chip")) return;     // idempotent
    var c = document.createElement("a");
    c.href = "/projects";
    c.className = "proj-chip" + (active ? " scoped" : "");
    c.title = "active project — recordings, scenarios and search scope here. click to switch.";
    c.textContent = active ? ("◆ " + active.project + " / " + active.environment)
                           : "◇ scratch";
    var nav = document.querySelector("header nav, nav.nav, nav");
    if (nav) {
      nav.insertBefore(c, nav.firstChild);
    } else {
      // no nav on this page (e.g. fleet) — pin it so scope is always visible
      c.style.cssText = "position:fixed;top:12px;right:12px;z-index:9998;background:#0b0e0c";
      (document.body || document.documentElement).appendChild(c);
    }
  }
  function go() {
    fetch("/api/projects/active")
      .then(function (r) { return r.json(); })
      .then(function (d) { render(d.active); })
      .catch(function () { render(null); });
  }
  if (document.readyState === "loading")
    document.addEventListener("DOMContentLoaded", go);
  else go();
})();
