import { useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";
import { useStudio } from "../store";
import { Live } from "./Live";
import { runTime, fmtSize, summarize } from "./Runs";

// The DS "Welcome back" landing (claude.ai/design f84aeaa8 · HomePage): a tinted
// hero + CTAs, an instrument-grade KPI overview, and recent runs. It replaces the
// old redirect-straight-to-Live so a returning user lands on a real dashboard.
// All numbers are read from the live backend — nothing is fabricated.

type Analytics = {
  observations: { total: number; with_embeddings: number; detections: number };
  runs: { count: number; sealed: number; anchored: number };
  storage: { used_gb: number; cap_gb: number; pct: number };
  fleet: { total: number; online: number };
  robots: { robot_id: string }[];
};
type Run = { run: string; robot_id?: string; size?: number; sealed?: boolean; anchored?: boolean; message_counts?: Record<string, number> };

const fmt = (n: number) => (n >= 1000 ? (n / 1000).toFixed(1).replace(/\.0$/, "") + "k" : String(n));

function Section({ label, count, more, onMore }: { label: string; count?: string; more?: string; onMore?: () => void }) {
  return (
    <div style={{ display: "flex", alignItems: "baseline", gap: 10, margin: "26px 0 12px" }}>
      <span className="seclabel" style={{ fontSize: 11, textTransform: "uppercase", letterSpacing: ".12em", color: "var(--fg-dim)" }}>{label}</span>
      {count && <span style={{ fontSize: 11, color: "var(--fg-dim)", opacity: 0.7 }}>{count}</span>}
      {more && (
        <button onClick={onMore} style={{ marginLeft: "auto", background: "none", border: 0, color: "var(--accent)", fontFamily: "var(--mono)", fontSize: 12, cursor: "pointer" }}>{more}</button>
      )}
    </div>
  );
}

function Kpi({ n, unit, label, sub }: { n: string; unit?: string; label: string; sub?: string }) {
  return (
    <div className="panel" style={{ minHeight: 0, padding: "14px 16px", gap: 0 }}>
      <div style={{ fontSize: 22, fontWeight: 700, lineHeight: 1.1, fontVariantNumeric: "tabular-nums" }}>
        {n}{unit && <span style={{ fontSize: 14, color: "var(--fg-2)", marginLeft: 1 }}>{unit}</span>}
      </div>
      <div style={{ fontSize: 10.5, textTransform: "uppercase", letterSpacing: ".06em", color: "var(--fg-dim)", marginTop: 6 }}>{label}</div>
      {sub && <div style={{ fontSize: 11, color: "var(--fg-dim)", marginTop: 3 }}>{sub}</div>}
    </div>
  );
}

export function Home() {
  const navigate = useNavigate();
  const openRun = useStudio((s) => s.openRun);
  const scopeKey = useStudio((s) => s.scopeKey);
  const [a, setA] = useState<Analytics | null>(null);
  const [runs, setRuns] = useState<Run[]>([]);
  const [loaded, setLoaded] = useState(false);

  useEffect(() => {
    let alive = true;
    Promise.all([
      fetch("/api/analytics").then((r) => r.json()).catch(() => null),
      fetch("/api/run/list").then((r) => r.json()).catch(() => ({ runs: [] })),
      fetch("/api/run/mcap").then((r) => r.json()).catch(() => ({ runs: [] })),
    ]).then(([an, list, mcap]) => {
      if (!alive) return;
      setA(an);
      const mcapMap = new Map<string, Run>((mcap.runs || []).map((m: Run) => [m.run, m]));
      const names = new Set<string>([...(list.runs || []).map((l: Run) => l.run), ...mcapMap.keys()]);
      const merged = [...names].map((run) => ({ run, ...(mcapMap.get(run) || {}) }));
      merged.sort((x, y) => (x.run < y.run ? 1 : -1));
      setRuns(merged);
      setLoaded(true);
    });
    return () => { alive = false; };
  }, [scopeKey]);

  // first-run: nothing recorded and nothing seen → fall through to Live itself,
  // so Home === Live when there's no dashboard to show. If a sim/robot is
  // streaming, the live panels render; if truly idle, Live shows the welcome.
  if (loaded && !runs.length && !(a && a.observations.total)) return <Live />;

  const open = (run: string) => { openRun(run); navigate("/runs"); };

  return (
    <>
      {/* hero — the one tinted surface (radial accent wash + soft shadow) */}
      <div style={{ position: "relative", overflow: "hidden", background: "var(--rr-hero-grad)", border: "1px solid var(--accent-dim)", borderRadius: "var(--r)", padding: "22px 24px", boxShadow: "var(--shadow)" }}>
        <div style={{ display: "flex", alignItems: "center", gap: 10, flexWrap: "wrap" }}>
          <div className="welcome-kicker" style={{ margin: 0, fontSize: 12, letterSpacing: ".16em", textTransform: "uppercase", color: "var(--accent)" }}>◇ RoboRun Studio</div>
          <span style={{ marginLeft: "auto", display: "inline-flex", alignItems: "center", gap: 7, fontSize: 10.5, textTransform: "uppercase", letterSpacing: ".08em", color: "var(--fg-dim)", border: "1px solid var(--line)", borderRadius: 999, padding: "3px 10px", background: "rgba(0,0,0,.2)" }}>
            <span style={{ width: 7, height: 7, borderRadius: "50%", background: "var(--ok)", animation: "rr-pulse 2.2s infinite" }} />runtime online
          </span>
        </div>
        <h1 style={{ fontSize: 21, margin: "11px 0 3px", letterSpacing: ".01em", color: "var(--fg)" }}>Welcome back</h1>
        <div style={{ fontSize: 12.5, color: "var(--fg-2)", maxWidth: "62ch" }}>Your local runtime is recording and indexing. Launch a sim, open the cockpit, or browse what your robots have seen.</div>
        <div style={{ display: "flex", gap: 9, flexWrap: "wrap", marginTop: 16 }}>
          <button className="btn welcome-primary" onClick={() => navigate("/sims")}>▣ New sim / robot</button>
          <a className="btn" href="/sim">▦ Open cockpit</a>
          <button className="btn" onClick={() => navigate("/search")}>⌕ Search history</button>
          <button className="btn" onClick={() => navigate("/runs")}>⊞ Browse runs</button>
        </div>
      </div>

      {a && (
        <>
          <Section label="Data overview" more="explore →" onMore={() => navigate("/analytics")} />
          <div style={{ display: "grid", gridTemplateColumns: "repeat(auto-fill, minmax(140px, 1fr))", gap: 12 }}>
            <Kpi n={fmt(a.runs.count)} label="saved runs" sub={`${a.runs.sealed} sealed · ${a.runs.anchored} anchored`} />
            <Kpi n={fmt(a.observations.total)} label="things seen" sub={`${fmt(a.observations.detections)} detections`} />
            <Kpi n={fmt(a.observations.with_embeddings)} label="AI-searchable" sub={a.observations.with_embeddings ? "embedded for search" : "install vision to embed"} />
            <Kpi n={String(a.storage.used_gb)} unit="G" label="stored" sub={`of ${a.storage.cap_gb}G · ${a.storage.pct}%`} />
            <Kpi n={String(a.fleet.online)} label="robots online" sub={a.fleet.online ? `${a.robots.length} total` : `${a.robots.length} seen, none live`} />
          </div>
        </>
      )}

      <Section label="Recent runs" count={runs.length ? `${runs.length} total` : undefined} more={runs.length ? "browse all →" : undefined} onMore={() => navigate("/runs")} />
      <div style={{ display: "flex", flexDirection: "column", gap: 6 }}>
        {runs.slice(0, 6).map((r) => (
          <button key={r.run} className="run-row col" onClick={() => open(r.run)} style={{ border: "1px solid var(--line)", borderRadius: "var(--r)" }}>
            <div className="rr-top">
              <span className="rr-title">{runTime(r.run)}</span>
              {r.anchored ? <span className="rr-badge ok">ANCHORED</span> : r.sealed ? <span className="rr-badge">SEALED</span> : null}
            </div>
            <div className="rr-meta">
              {r.robot_id || "local"}{summarize(r.message_counts) && " · " + summarize(r.message_counts)}{fmtSize(r.size) && " · " + fmtSize(r.size)}
            </div>
          </button>
        ))}
        {loaded && !runs.length && (
          <div style={{ padding: 14, color: "var(--fg-dim)", fontSize: 12 }}>no runs yet — start a sim and press ● Record, or run <code>roborun demo</code>.</div>
        )}
      </div>
    </>
  );
}
