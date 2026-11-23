import { useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";
import { useStudio } from "../store";

type Hit = { obs_id?: string; run_id?: string; ts?: number; robot_id?: string; detections?: { label: string }[] };
type Mode = "label" | "time" | "clip";

const MODES: { id: Mode; label: string }[] = [
  { id: "label", label: "by object" },
  { id: "time", label: "recent" },
  { id: "clip", label: "by meaning (CLIP)" },
];

// Searches everything the robots have recorded in the active project, across all
// runs. A hit jumps straight into replay at that exact moment. Semantic ("by
// meaning") search is only offered when a real CLIP encoder is available — no fakes.
export function Search() {
  const openRun = useStudio((s) => s.openRun);
  const navigate = useNavigate();
  const [q, setQ] = useState("");
  const [mode, setMode] = useState<Mode>("label");
  const [hits, setHits] = useState<Hit[]>([]);
  const [busy, setBusy] = useState(false);
  const [scope, setScope] = useState<string>("scratch");
  const [semantic, setSemantic] = useState(false);

  useEffect(() => {
    fetch("/api/projects/active")
      .then((r) => r.json())
      .then((a) => a.active && setScope(`${a.active.project} / ${a.active.environment}`))
      .catch(() => {});
    fetch("/api/search/caps")
      .then((r) => r.json())
      .then((c) => setSemantic(!!c.semantic))
      .catch(() => {});
    runSearch("time", ""); // show recent so the page is never blank
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const runSearch = async (by: Mode, query: string) => {
    setBusy(true);
    try {
      const r = await fetch("/api/search", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ query, by, k: 48 }),
      });
      const d = await r.json();
      setHits(d.results || []);
    } catch {
      setHits([]);
    } finally {
      setBusy(false);
    }
  };

  // never silently use CLIP when it isn't real — fall back to object/label
  const effective = (m: Mode): Mode => (m === "clip" && !semantic ? "label" : m === "time" && q ? "label" : m);
  const submit = () => runSearch(effective(mode), q);
  const pickMode = (m: Mode) => {
    if (m === "clip" && !semantic) return; // disabled
    setMode(m);
    runSearch(effective(m), q);
  };

  const jump = async (h: Hit) => {
    if (!h.run_id || h.ts == null) return;
    await openRun(h.run_id, h.ts);
    navigate("/runs");
  };

  return (
    <div>
      <div className="search-bar">
        <input
          className="search-input"
          placeholder="find anything the robots have seen — “red mug”, “person”, “open door”…"
          value={q}
          onChange={(e) => setQ(e.target.value)}
          onKeyDown={(e) => e.key === "Enter" && submit()}
          aria-label="search query"
        />
        <button className="btn" onClick={submit} disabled={busy}>
          {busy ? "…" : "Search"}
        </button>
      </div>
      <div className="search-scope">
        Searching every recorded observation in <b>{scope}</b> · across all runs
        {!semantic && (
          <span style={{ color: "var(--fg-dim)" }}>
            {" "}· matching object labels. Semantic/image search needs the vision extra —{" "}
            <code style={{ color: "var(--accent)" }}>pip install 'ros-agent[vision]'</code>
          </span>
        )}
      </div>
      <div className="chips">
        {MODES.map((m) => {
          const disabled = m.id === "clip" && !semantic;
          return (
            <button
              key={m.id}
              className={"chip-btn" + (mode === m.id ? " on" : "")}
              onClick={() => pickMode(m.id)}
              disabled={disabled}
              title={disabled ? "needs the vision extra: pip install 'ros-agent[vision]'" : undefined}
              style={disabled ? { opacity: 0.45, cursor: "default" } : undefined}
            >
              {m.label}
              {disabled && " ·  needs vision"}
            </button>
          );
        })}
      </div>
      {!!hits.length && (
        <div style={{ fontSize: 12, color: "var(--fg-dim)", marginBottom: 10, fontFamily: "var(--mono)" }}>
          {hits.length} result{hits.length === 1 ? "" : "s"} · {hits.filter((h) => h.run_id).length} replayable
        </div>
      )}
      <div className="panel-grid" style={{ gridTemplateColumns: "repeat(4, minmax(0,1fr))" }}>
        {hits.map((h, i) => {
          const replayable = !!h.run_id && h.ts != null;
          return (
            <button
              key={h.obs_id ?? i}
              className="panel"
              style={{ cursor: replayable ? "pointer" : "default", textAlign: "left", minHeight: 0, opacity: replayable ? 1 : 0.6 }}
              onClick={() => jump(h)}
              title={replayable ? "open this moment in replay" : "live/streamed observation — not linked to a recorded run"}
            >
              <div className="panel-body" style={{ fontFamily: "var(--mono)", fontSize: 12 }}>
                {replayable && (
                  <img
                    src={`/api/run/frame?id=${encodeURIComponent(h.run_id!)}&t=${h.ts}&robot=${encodeURIComponent(h.robot_id || "")}`}
                    alt={(h.detections || []).map((d) => d.label).join(", ") || "frame"}
                    loading="lazy"
                    style={{ width: "100%", aspectRatio: "16/10", objectFit: "cover", borderRadius: 6, marginBottom: 8, background: "#000" }}
                    onError={(e) => ((e.currentTarget.style.display = "none"))}
                  />
                )}
                <div style={{ color: "var(--accent)", marginBottom: 4 }}>
                  {(h.detections || []).map((d) => d.label).join(", ") || "frame"}
                </div>
                <div style={{ color: "var(--fg-dim)" }}>{h.robot_id}</div>
                <div style={{ color: "var(--fg-dim)" }}>{h.ts ? new Date(h.ts * 1000).toLocaleString() : ""}</div>
                <div style={{ color: replayable ? "var(--accent)" : "var(--fg-dim)", marginTop: 6 }}>
                  {replayable ? "↳ open in replay" : "· not in a recorded run"}
                </div>
              </div>
            </button>
          );
        })}
      </div>
      {!hits.length && !busy && (
        <div style={{ color: "var(--fg-dim)", fontSize: 14, padding: "32px 4px" }}>
          Nothing recorded in <b style={{ color: "var(--fg-2)" }}>{scope}</b> yet. Run a sim under{" "}
          <b style={{ color: "var(--accent)" }}>Sims</b>, press <b style={{ color: "var(--accent)" }}>Record</b>, and what
          the robots see becomes searchable here.
        </div>
      )}
    </div>
  );
}
