import { useEffect, useState } from "react";
import { Panel } from "../panels/Panel";

// Native Studio scenarios — run scored, tamper-proof behavior tests. Replaces
// the sparse hosted page; reuses Studio panel chrome. Data from /api/scenarios*.
type Def = { name: string; suite: string; tags: string[]; params: Record<string, unknown>; doc: string };
type Result = {
  name?: string;
  suite?: string;
  outcome?: string; // passed | failed | error
  metrics?: Record<string, number>;
  duration_s?: number;
  ended?: string;
  started?: string;
};

const verdict = (r: Result) => r.outcome || "—";
const color = (v: string) => (v === "passed" ? "var(--ok)" : v === "failed" || v === "error" ? "var(--bad)" : "var(--fg-dim)");
const fmtWhen = (s?: string) => (s ? new Date(s).toLocaleString([], { month: "short", day: "numeric", hour: "numeric", minute: "2-digit" }) : "");

export function Scenarios() {
  const [defs, setDefs] = useState<Def[]>([]);
  const [results, setResults] = useState<Result[]>([]);
  const [busy, setBusy] = useState<string | null>(null);

  const loadResults = () =>
    fetch("/api/scenarios")
      .then((r) => r.json())
      .then((d) => setResults(d.results || []))
      .catch(() => {});

  useEffect(() => {
    fetch("/api/scenarios/defs")
      .then((r) => r.json())
      .then((d) => setDefs(d.defs || []))
      .catch(() => {});
    loadResults();
  }, []);

  const run = async (body: object, key: string) => {
    setBusy(key);
    try {
      await fetch("/api/scenarios/run", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      await loadResults();
    } finally {
      setBusy(null);
    }
  };

  const suites = [...new Set(defs.map((d) => d.suite))];
  const suiteStat = (suite: string) => {
    const rs = results.filter((r) => r.suite === suite);
    const passed = rs.filter((r) => r.outcome === "passed").length;
    return { total: rs.length, passed, pct: rs.length ? Math.round((passed / rs.length) * 100) : null };
  };

  return (
    <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>
      <div>
        <h2 style={{ fontSize: 18, fontWeight: 600, color: "var(--fg)", margin: "0 0 6px" }}>
          Test that your robot behaviors still work
        </h2>
        <p style={{ fontSize: 13.5, color: "var(--fg-2)", margin: 0, lineHeight: 1.6, maxWidth: 760 }}>
          A scenario runs a behavior against a target and scores it <b>pass/fail</b> — like unit tests, but for
          how the robot acts. Run one after you change a behavior to answer “did I break anything?”. Every
          result is recorded and tamper-proof. <b>Press “run” on any test below</b> to try it.
        </p>
      </div>

      <div className="panel-grid" style={{ gridTemplateColumns: `repeat(${Math.min(suites.length || 1, 3)}, minmax(0,1fr))` }}>
        {suites.map((suite) => {
          const items = defs.filter((d) => d.suite === suite);
          const st = suiteStat(suite);
          return (
            <Panel key={suite} title={`${suite} · ${items.length} test${items.length === 1 ? "" : "s"}`}>
              <div style={{ display: "flex", flexDirection: "column", gap: 10 }}>
                <div style={{ display: "flex", alignItems: "center", gap: 10 }}>
                  <button className="btn sm" disabled={!!busy} onClick={() => run({ suite }, "suite:" + suite)}>
                    {busy === "suite:" + suite ? "running…" : "▶ run all"}
                  </button>
                  <span style={{ fontFamily: "var(--mono)", fontSize: 12, color: st.pct === 100 ? "var(--accent)" : st.pct == null ? "var(--fg-dim)" : "var(--warn)" }}>
                    {st.pct == null ? "not run yet" : `${st.passed}/${st.total} passed · ${st.pct}%`}
                  </span>
                </div>
                {items.map((d) => (
                  <div key={d.name} style={{ display: "flex", justifyContent: "space-between", alignItems: "flex-start", gap: 10 }}>
                    <div style={{ minWidth: 0 }}>
                      <div style={{ fontFamily: "var(--mono)", fontSize: 13, color: "var(--fg)" }}>{d.name}</div>
                      <div style={{ fontSize: 11, color: "var(--fg-dim)" }}>{d.doc}</div>
                    </div>
                    <button className="btn sm" disabled={!!busy} onClick={() => run({ scenario: d.name }, "sc:" + d.name)}>
                      {busy === "sc:" + d.name ? "…" : "run"}
                    </button>
                  </div>
                ))}
              </div>
            </Panel>
          );
        })}
        {!suites.length && (
          <Panel title="scenarios">
            <div style={{ color: "var(--fg-dim)", fontSize: 13 }}>No scenarios defined.</div>
          </Panel>
        )}
      </div>

      <Panel title={`Scored runs · ${results.length} — each one recorded & sealed`}>
        {results.length ? (
          <table style={{ width: "100%", borderCollapse: "collapse", fontFamily: "var(--mono)", fontSize: 12 }}>
            <thead>
              <tr style={{ color: "var(--fg-dim)", textAlign: "left" }}>
                <th style={{ padding: "6px 8px" }}>result</th>
                <th style={{ padding: "6px 8px" }}>scenario</th>
                <th style={{ padding: "6px 8px" }}>suite</th>
                <th style={{ padding: "6px 8px" }}>scores</th>
                <th style={{ padding: "6px 8px" }}>when</th>
              </tr>
            </thead>
            <tbody>
              {results.map((r, i) => {
                const v = verdict(r);
                return (
                  <tr key={i} style={{ borderTop: "1px solid var(--line)" }}>
                    <td style={{ padding: "6px 8px", color: color(v) }}>● {v}</td>
                    <td style={{ padding: "6px 8px" }}>{r.name}</td>
                    <td style={{ padding: "6px 8px", color: "var(--fg-dim)" }}>{r.suite}</td>
                    <td style={{ padding: "6px 8px", color: "var(--fg-dim)" }}>{r.metrics ? JSON.stringify(r.metrics) : "—"}</td>
                    <td style={{ padding: "6px 8px", color: "var(--fg-dim)" }}>{fmtWhen(r.ended || r.started)}</td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        ) : (
          <div style={{ color: "var(--fg-dim)", fontSize: 13 }}>
            No runs yet. Run a scenario above (or <code>roborun demo</code>) to see scored, sealed results here.
          </div>
        )}
      </Panel>
    </div>
  );
}
