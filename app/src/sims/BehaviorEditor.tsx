import { useEffect, useState } from "react";

// Edit a behavior file in-app. Save writes through /api/behaviors/write, which
// the runtime's file watcher hot-reloads into the running robot — so the
// "change a number, save, watch it change" pitch is actually doable here.
type Beh = { name: string; enabled?: boolean; errors?: number; last_error?: string | null };

export function BehaviorEditor() {
  const [list, setList] = useState<Beh[]>([]);
  const [name, setName] = useState("");
  const [src, setSrc] = useState("");
  const [status, setStatus] = useState<string>("");
  const [busy, setBusy] = useState(false);

  useEffect(() => {
    fetch("/api/behaviors")
      .then((r) => r.json())
      .then((d) => {
        const bs: Beh[] = d.behaviors || [];
        setList(bs);
        const first = bs.find((b) => b.name === "follow_person") || bs[0];
        if (first) load(first.name);
      })
      .catch(() => {});
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const load = async (n: string) => {
    setName(n);
    setStatus("");
    const d = await (await fetch("/api/behaviors/read", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name: n }) })).json();
    if (d.ok) setSrc(d.source);
    else setStatus(d.error || "couldn't read file");
  };

  const save = async () => {
    setBusy(true);
    setStatus("");
    try {
      const d = await (await fetch("/api/behaviors/write", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name, source: src }) })).json();
      setStatus(d.ok ? "✓ saved · hot-reloaded into the running robot" : "✗ " + (d.error || "write failed"));
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="beh-editor">
      <div className="beh-bar">
        <select className="btn sm" value={name} onChange={(e) => load(e.target.value)} aria-label="behavior file">
          {list.map((b) => (
            <option key={b.name} value={b.name}>
              behaviors/{b.name}.py
            </option>
          ))}
        </select>
        <button className="btn sm welcome-primary" onClick={save} disabled={busy || !name}>
          {busy ? "saving…" : "Save & hot-reload"}
        </button>
        {status && <span className="beh-status" style={{ color: status.startsWith("✓") ? "var(--accent)" : "var(--bad)" }}>{status}</span>}
      </div>
      <textarea
        className="beh-code"
        spellCheck={false}
        value={src}
        onChange={(e) => setSrc(e.target.value)}
        onKeyDown={(e) => {
          if ((e.metaKey || e.ctrlKey) && e.key === "s") {
            e.preventDefault();
            save();
          }
        }}
      />
    </div>
  );
}
