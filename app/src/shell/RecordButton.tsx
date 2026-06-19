import { useEffect, useRef, useState } from "react";

// Records the *robot's run* (camera, detections, pose) into a sealed black box.
// Truth source is /api/run/mcap `.recording` — NOT the run list, whose flag is
// always-on (it tracks the live event journal) and made the button look like it
// was secretly recording on every fresh load.
function runStartMs(runId: string): number {
  // run_YYYYMMDD_HHMMSS (UTC) → epoch ms
  const m = /run_(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})/.exec(runId);
  if (!m) return Date.now();
  const [, y, mo, d, h, mi, s] = m.map(Number) as unknown as number[];
  return Date.UTC(y, mo - 1, d, h, mi, s);
}
const fmtElapsed = (ms: number) => {
  const s = Math.max(0, Math.floor(ms / 1000));
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, "0")}`;
};

export function RecordButton() {
  const [rec, setRec] = useState<{ run: string; startMs: number } | null>(null);
  const [busy, setBusy] = useState(false);
  const [, tick] = useState(0);
  const startMs = useRef<number>(0);

  const refresh = async () => {
    try {
      const d = await (await fetch("/api/run/mcap")).json();
      if (d.recording && d.recording.run) {
        setRec({ run: d.recording.run, startMs: runStartMs(d.recording.run) });
      } else setRec(null);
    } catch {
      /* offline: treat as not recording */
    }
  };

  useEffect(() => {
    refresh();
  }, []);

  // tick the timer once a second while recording
  useEffect(() => {
    if (!rec) return;
    const id = setInterval(() => tick((n) => n + 1), 1000);
    return () => clearInterval(id);
  }, [rec]);

  const toggle = async () => {
    setBusy(true);
    try {
      if (rec) {
        await fetch("/api/run/record/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
        setRec(null);
      } else {
        startMs.current = Date.now();
        const r = await fetch("/api/run/record/start", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: "{}",
        });
        const d = await r.json();
        if (d.ok !== false && d.run) setRec({ run: d.run, startMs: startMs.current });
      }
    } finally {
      setBusy(false);
    }
  };

  return (
    <button
      className={"btn sm" + (rec ? " rec-on" : "")}
      onClick={toggle}
      disabled={busy}
      title={
        rec
          ? `Recording this run into a sealed black box (${rec.run}). Click to stop & seal.`
          : "Record the robot's run — camera, detections and pose — into a sealed, verifiable black box."
      }
    >
      {rec ? (
        <>
          <span className="dot" /> REC {fmtElapsed(Date.now() - rec.startMs)}
        </>
      ) : (
        "● Record run"
      )}
    </button>
  );
}
