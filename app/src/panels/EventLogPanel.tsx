import { useEffect, useRef } from "react";
import { useStudio } from "../store";
import type { EventMsg } from "../source/Source";
import { Panel } from "./Panel";

// Newcomers shouldn't drown in system chatter. Drop low-signal noise (the
// "no actuator" warning, behavior-load lines, per-frame camera hashes) and
// collapse consecutive repeats, so the feed shows what actually happened.
function isNoise(e: EventMsg): boolean {
  const t = e.title || "";
  if (/wants to move/i.test(t)) return true; // "no actuator" nag
  if (e.type === "system" && /^loaded /i.test(t)) return true; // behavior autoload
  if (e.type === "frame") return true; // per-frame camera hash heartbeat
  return false;
}

type Row = EventMsg & { _n?: number };
function clean(events: EventMsg[]): Row[] {
  const out: Row[] = [];
  for (const e of events) {
    if (isNoise(e)) continue;
    const prev = out[out.length - 1];
    if (prev && prev.type === e.type && prev.title === e.title) {
      prev._n = (prev._n || 1) + 1; // collapse consecutive duplicates
    } else {
      out.push({ ...e });
    }
  }
  return out.slice(-200);
}

export function EventLogPanel() {
  const source = useStudio((s) => s.source);
  const t = useStudio((s) => s.t);
  const rows = clean(source.snapshotEvents(t));
  const ref = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (ref.current) ref.current.scrollTop = ref.current.scrollHeight;
  }, [rows.length]);

  return (
    <Panel title="Events">
      <div ref={ref} style={{ fontFamily: "var(--mono)", fontSize: 12, lineHeight: 1.6 }}>
        {rows.map((e, i) => (
          <div key={e.id ?? i}>
            <span style={{ color: "var(--fg-dim)" }}>{new Date(e.ts * 1000).toLocaleTimeString()} </span>
            <span style={{ color: "var(--accent)" }}>{e.type}</span> <span>{e.title}</span>
            {e._n && e._n > 1 && <span style={{ color: "var(--fg-dim)" }}> ×{e._n}</span>}
          </div>
        ))}
        {!rows.length && (
          <div style={{ color: "var(--fg-dim)" }}>
            {source.kind === "live"
              ? "No activity yet. Start a sim or connect a robot and what it does shows up here."
              : "No events at this point in the run — scrub forward."}
          </div>
        )}
      </div>
    </Panel>
  );
}
