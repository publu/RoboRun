import { useEffect, useRef, useState } from "react";
import uPlot from "uplot";
import "uplot/dist/uPlot.min.css";
import { Panel } from "../panels/Panel";

// Native Studio analytics — a live read of /api/analytics, laid out in Studio
// panels so it matches the rest of the app (no more hosted-iframe inconsistency).
type Analytics = {
  observations: { total: number; with_embeddings: number; with_position: number; detections: number; runs: number };
  labels: { label: string; count: number }[];
  over_time: { t: number; count: number }[];
  sources: { source: string; count: number }[];
  robots: { robot_id: string; observations: number; last_seen: number; top_label: string }[];
  suites: unknown[];
  runs: { count: number; sealed: number; anchored: number };
  storage: { used_gb: number; cap_gb: number; pct: number };
  fleet: { total: number; online: number };
};

function Stat({ v, label, sub }: { v: string; label: string; sub?: string }) {
  return (
    <div className="panel" style={{ minHeight: 0, padding: "14px 16px" }}>
      {/* DS KPI numerals: 22px bold, .06em label tracking — matches Home tiles */}
      <div style={{ fontSize: 22, fontWeight: 700, lineHeight: 1.1, fontVariantNumeric: "tabular-nums" }}>{v}</div>
      <div style={{ fontSize: 10.5, textTransform: "uppercase", letterSpacing: ".06em", color: "var(--fg-dim)", marginTop: 6 }}>
        {label}
      </div>
      {sub && <div style={{ fontSize: 11, color: "var(--fg-dim)", marginTop: 3 }}>{sub}</div>}
    </div>
  );
}

function Bars({ rows }: { rows: { label: string; count: number }[] }) {
  const max = Math.max(1, ...rows.map((r) => r.count));
  return (
    <div style={{ display: "flex", flexDirection: "column", gap: 9 }}>
      {rows.map((r) => (
        <div key={r.label} style={{ display: "grid", gridTemplateColumns: "120px 1fr 48px", alignItems: "center", gap: 10, fontSize: 12, fontFamily: "var(--mono)" }}>
          <span style={{ color: "var(--fg-2)", overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}>{r.label}</span>
          <span style={{ background: "var(--line)", borderRadius: 4, height: 8 }}>
            <span style={{ display: "block", width: `${(r.count / max) * 100}%`, height: "100%", background: "var(--accent)", borderRadius: 4 }} />
          </span>
          <span style={{ color: "var(--fg-dim)", textAlign: "right" }}>{r.count}</span>
        </div>
      ))}
      {!rows.length && <div style={{ color: "var(--fg-dim)", fontSize: 12 }}>no data yet</div>}
    </div>
  );
}

function TimeChart({ rows }: { rows: { t: number; count: number }[] }) {
  const host = useRef<HTMLDivElement>(null);
  useEffect(() => {
    if (!host.current || !rows.length) return;
    const data: uPlot.AlignedData = [rows.map((r) => r.t), rows.map((r) => r.count)];
    const u = new uPlot(
      {
        width: host.current.clientWidth || 360,
        height: 170,
        series: [{}, { label: "obs", stroke: "#00d47e", width: 1.5, fill: "rgba(0,212,126,.1)" }],
        axes: [
          { stroke: "#6f8a78", grid: { stroke: "#212c22" } },
          { stroke: "#6f8a78", grid: { stroke: "#212c22" } },
        ],
        legend: { show: false },
      },
      data,
      host.current
    );
    return () => u.destroy();
  }, [rows]);
  return <div ref={host} />;
}

export function Analytics() {
  const [a, setA] = useState<Analytics | null>(null);
  useEffect(() => {
    fetch("/api/analytics")
      .then((r) => r.json())
      .then(setA)
      .catch(() => {});
  }, []);

  if (!a) return <div style={{ color: "var(--fg-dim)", padding: 12 }}>loading analytics…</div>;

  return (
    <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>
      <div className="panel-grid" style={{ gridTemplateColumns: "repeat(6, minmax(0,1fr))" }}>
        <Stat v={fmt(a.observations.total)} label="things seen" sub={`${fmt(a.observations.detections)} detections`} />
        <Stat v={fmt(a.observations.with_embeddings)} label="AI-searchable" sub={a.observations.with_embeddings ? undefined : "install vision to embed"} />
        <Stat v={String(a.runs.count)} label="saved runs" sub={`${a.runs.sealed} sealed · ${a.runs.anchored} anchored`} />
        <Stat v={`${a.storage.used_gb}G`} label="stored" sub={`of ${a.storage.cap_gb}G · ${a.storage.pct}%`} />
        <Stat v={String(a.fleet.online)} label="robots online" sub={a.fleet.online ? `${a.robots.length} total` : `${a.robots.length} seen, none live`} />
        <Stat v={String(a.suites.length)} label="test suites" />
      </div>

      <div className="panel-grid">
        <Panel title="Detections by label">
          <Bars rows={a.labels.slice(0, 8)} />
        </Panel>
        <Panel title="Observations · last 24h">
          <TimeChart rows={a.over_time} />
        </Panel>
        <Panel title="Data by source">
          <Bars rows={a.sources.map((s) => ({ label: s.source, count: s.count }))} />
        </Panel>
        <Panel title="Fleet activity">
          <div style={{ display: "flex", flexDirection: "column", gap: 10 }}>
            {a.robots.map((r) => (
              <div key={r.robot_id} style={{ fontFamily: "var(--mono)", fontSize: 12 }}>
                <span style={{ color: "var(--accent)" }}>● {r.robot_id}</span>
                <span style={{ color: "var(--fg-dim)" }}>
                  {" "}
                  · {fmt(r.observations)} obs · mostly {r.top_label}
                </span>
                <div style={{ color: "var(--fg-dim)" }}>last active {new Date(r.last_seen * 1000).toLocaleString()}</div>
              </div>
            ))}
            {!a.robots.length && <div style={{ color: "var(--fg-dim)", fontSize: 12 }}>no robots yet</div>}
          </div>
        </Panel>
      </div>
    </div>
  );
}

const fmt = (n: number) => (n >= 1000 ? (n / 1000).toFixed(1).replace(/\.0$/, "") + "k" : String(n));
