/* Shared run-detail component — the scored record rendered the Antioch way:
 * METADATA / PARAMETERS / RESULTS / EVALUATION tree / TAGS.
 *
 * One renderer, two entry points: run.html (with the replay scrubber + charts
 * alongside) and timeline.html (in its right pane). Pure DOM string → innerHTML;
 * styling comes from ui.css (.kv/.chip/.pill/.seclabel). No data invented — a
 * section only renders when the record actually carries it.
 */
(function () {
  const esc = (s) => String(s == null ? "" : s)
    .replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
  const when = (s) => esc((s || "").replace("T", " ").replace("Z", "").split(".")[0]);
  const num = (v) => (typeof v === "number" ? (Number.isInteger(v) ? v : v.toFixed(2)) : esc(v));

  function chips(obj) {
    const e = Object.entries(obj || {}).filter(([k]) => !k.startsWith("_"));
    if (!e.length) return "";
    return `<div class="chips">${e.map(([k, v]) =>
      `<span class="chip"><span class="k">${esc(k)}</span><span class="v">${num(v)}</span></span>`).join("")}</div>`;
  }

  function evalTree(ev) {
    const groups = Object.entries(ev || {});
    if (!groups.length) return "";
    return `<div class="evaltree">${groups.map(([g, vs]) => {
      const subs = vs && typeof vs === "object"
        ? Object.entries(vs).map(([k, v]) =>
            `<div class="evalrow"><span class="k">${esc(k)}</span><span class="v">${num(v)}</span></div>`).join("")
        : `<div class="evalrow"><span class="v">${num(vs)}</span></div>`;
      return `<div class="evalgroup"><div class="evalg-h">${esc(g)}</div>${subs}</div>`;
    }).join("")}</div>`;
  }

  // el: container; rec: scenario record; opts.actions: extra HTML for the action row
  window.renderRunRecord = function (el, rec, opts) {
    opts = opts || {};
    if (!rec) { el.innerHTML = `<div class="empty" style="color:var(--fg-dim);padding:20px">No scored record for this run.</div>`; return; }
    const meta = [
      ["run id", rec.run_id], ["scenario", rec.name], ["suite", rec.suite],
      ["robot", rec.robot], ["seed", rec.seed],
      ["duration", rec.duration_s != null ? rec.duration_s + "s" : null],
      ["started", when(rec.started)], ["ended", when(rec.ended)],
    ].filter(([, v]) => v != null && v !== "");
    const metaHTML = meta.map(([k, v]) =>
      `<div class="k">${esc(k)}</div><div class="v">${esc(v)}</div>`).join("");
    const tags = (rec.tags || []).map((t) => `<span class="chip">${esc(t)}</span>`).join("");
    const params = chips(rec.params), results = chips(rec.metrics), tree = evalTree(rec.evaluation);

    const sub = [rec.suite && ("suite " + rec.suite), rec.robot && ("robot " + rec.robot)]
      .filter(Boolean).map(esc).join("  ·  ");

    el.innerHTML = `
      <div class="rd-head">
        <span class="rd-title">${esc(rec.name || rec.run_id || "run")}</span>
        ${rec.outcome ? `<span class="pill ${esc(rec.outcome)}">${esc(rec.outcome)}</span>` : ""}
        ${sub ? `<span style="font-size:11.5px;color:var(--fg-dim)">${sub}</span>` : ""}
        ${opts.actions ? `<span class="rd-actions">${opts.actions}</span>` : ""}
      </div>
      ${rec.reason ? `<div class="rd-reason">${esc(rec.reason)}</div>` : ""}
      <div class="rd-cols">
        <div class="rd-col">
          <div class="seclabel">Metadata</div><div class="kv">${metaHTML}</div>
          ${params ? `<div class="seclabel" style="margin-top:18px">Parameters</div>${params}` : ""}
        </div>
        <div class="rd-col">
          ${results ? `<div class="seclabel">Results</div>${results}` : `<div class="seclabel">Results</div><div class="rd-none">no metrics recorded</div>`}
          ${tree ? `<div class="seclabel" style="margin-top:18px">Evaluation</div>${tree}` : ""}
          ${tags ? `<div class="seclabel" style="margin-top:18px">Tags</div><div class="chips">${tags}</div>` : ""}
        </div>
      </div>`;
  };
})();
