import { useEffect, useState } from "react";
import { useStudio } from "../store";
import { CameraPanel } from "../panels/CameraPanel";
import { EventLogPanel } from "../panels/EventLogPanel";
import { DetectionsPanel } from "../panels/DetectionsPanel";
import { PlotPanel } from "../panels/PlotPanel";
import { DropZone } from "../panels/DropZone";

type ListRow = { run: string; events: number; recording?: boolean };
type McapRow = {
  run: string;
  robot_id?: string;
  size?: number;
  sealed?: boolean;
  anchored?: boolean;
  merkle_root?: string;
  message_counts?: Record<string, number>;
};
type Row = ListRow & Partial<McapRow>;

export function runTime(run: string): string {
  const m = /run_(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})/.exec(run);
  if (!m) return run;
  const [, y, mo, d, h, mi] = m.map(Number) as unknown as number[];
  return new Date(Date.UTC(y, mo - 1, d, h, mi)).toLocaleString([], {
    month: "short", day: "numeric", hour: "numeric", minute: "2-digit",
  });
}
export const fmtSize = (b?: number) => (b == null ? "" : b > 1e6 ? (b / 1e6).toFixed(1) + "MB" : Math.round(b / 1e3) + "KB");

// What the robot saw/did, derived from the recorded channels.
export function summarize(c?: Record<string, number>): string {
  if (!c) return "";
  const frames = c["/detections/arena"] ?? c["/camera/webcam"] ?? 0;
  const det = Object.entries(c).filter(([k]) => k.startsWith("/detections")).reduce((a, [, v]) => a + v, 0);
  const bits = [];
  if (c["/pose"]) bits.push(`${c["/pose"]} poses`);
  if (det) bits.push(`${det} detections`);
  void frames;
  return bits.join(" · ");
}

type Verify = { state?: string; merkle_root?: string; segments?: number; signature_valid?: boolean };
function Provenance({ run, robot, sealed }: { run: string; robot?: string; sealed?: boolean }) {
  const [v, setV] = useState<Verify | null>(null);
  const [busy, setBusy] = useState(false);
  const [tampered, setTampered] = useState(false);
  const body = JSON.stringify({ run, robot_id: robot });

  const verify = async () => {
    setBusy(true);
    try {
      const d = await (await fetch("/api/run/mcap/verify", { method: "POST", headers: { "Content-Type": "application/json" }, body })).json();
      setV(d);
    } finally {
      setBusy(false);
    }
  };
  useEffect(() => {
    setV(null);
    setTampered(false);
    if (sealed) verify();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [run]);

  // reversible XOR byte-flip — proves the chain catches edits, then restores
  const tamper = async () => {
    setBusy(true);
    try {
      await fetch("/api/run/mcap/tamper", { method: "POST", headers: { "Content-Type": "application/json" }, body });
      setTampered((t) => !t);
      await verify();
    } finally {
      setBusy(false);
    }
  };

  if (!sealed) return <span className="prov" title="recording or not yet sealed">● unsealed</span>;

  const state = v?.state;
  const good = state === "verified_anchored" || state === "consistent_unanchored";
  const label = busy
    ? "checking…"
    : state === "verified_anchored" ? "✓ VERIFIED · ANCHORED"
    : state === "consistent_unanchored" ? "✓ SEALED · unanchored"
    : state === "broken" ? "✗ TAMPER DETECTED"
    : "● sealed";
  return (
    <>
      <span className={"prov " + (good ? "ok" : state === "broken" ? "bad" : "warn")}
        title={v?.merkle_root ? `merkle root ${v.merkle_root}` : "tamper-evident seal"}>
        {label}
        {v?.merkle_root && <code className="prov-root">{v.merkle_root.slice(0, 10)}…</code>}
      </span>
      <button className="btn sm" onClick={verify} disabled={busy} title="re-check the seal now">verify</button>
      <button className="btn sm" onClick={tamper} disabled={busy} style={tampered ? { borderColor: "var(--accent)", color: "var(--accent)" } : { color: "var(--warn)" }}
        title="demo: flip one byte to prove detection — click again to restore">
        {tampered ? "↩ restore" : "⚡ tamper demo"}
      </button>
    </>
  );
}

export function Runs() {
  const source = useStudio((s) => s.source);
  const openRun = useStudio((s) => s.openRun);
  const [rows, setRows] = useState<Row[]>([]);

  const load = () =>
    Promise.all([
      fetch("/api/run/list").then((r) => r.json()).catch(() => ({ runs: [] })),
      fetch("/api/run/mcap").then((r) => r.json()).catch(() => ({ runs: [] })),
    ]).then(([list, mcap]) => {
      const listMap = new Map<string, ListRow>((list.runs || []).map((l: ListRow) => [l.run, l]));
      const mcapMap = new Map<string, McapRow>((mcap.runs || []).map((m: McapRow) => [m.run, m]));
      // union: sealed MCAP runs (e.g. demo runs) may have no event journal, and
      // journal runs may not be sealed yet — show both.
      const names = new Set<string>([...listMap.keys(), ...mcapMap.keys()]);
      const recordingRun: string | undefined = mcap.recording?.run;
      const merged: Row[] = [...names].map((run) => ({
        run,
        events: listMap.get(run)?.events ?? 0,
        ...(mcapMap.get(run) || {}),
        recording: run === recordingRun,
      }));
      merged.sort((a, b) => (a.run < b.run ? 1 : -1)); // newest first
      setRows(merged);
    });

  useEffect(() => {
    load();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const selected = rows.find((r) => r.run === source.id);

  return (
    <div style={{ display: "grid", gridTemplateColumns: "260px 1fr", gap: 12 }}>
      <div className="panel" style={{ minHeight: 0 }}>
        <div className="panel-head">Runs · {rows.length}</div>
        <DropZone onLoaded={async (run) => { await load(); openRun(run); }} />
        <div className="panel-body" style={{ padding: 0 }}>
          {rows.map((r) => (
            <button key={r.run} onClick={() => openRun(r.run)} className={"run-row col" + (source.id === r.run ? " on" : "")}>
              <div className="rr-top">
                <span className="rr-title">{runTime(r.run)}</span>
                {r.recording ? <span className="rr-badge rec">● REC</span>
                  : r.anchored ? <span className="rr-badge ok">ANCHORED</span>
                  : r.sealed ? <span className="rr-badge">SEALED</span> : null}
              </div>
              <div className="rr-meta">
                {r.robot_id || "local"}{summarize(r.message_counts) && " · " + summarize(r.message_counts)}
                {fmtSize(r.size) && " · " + fmtSize(r.size)}
              </div>
            </button>
          ))}
          {!rows.length && <div style={{ padding: 14, color: "var(--fg-dim)", fontSize: 12 }}>no runs yet — press ● Record while a sim runs</div>}
        </div>
      </div>
      <div style={{ display: "flex", flexDirection: "column", gap: 12, minWidth: 0 }}>
        {selected && (
          <div className="prov-bar">
            <span className="prov-run">{selected.run}</span>
            <Provenance run={selected.run} robot={selected.robot_id} sealed={selected.sealed} />
          </div>
        )}
        <div className="panel-grid">
          <CameraPanel />
          <EventLogPanel />
          <DetectionsPanel />
          <PlotPanel channel="velocity" />
        </div>
      </div>
    </div>
  );
}
