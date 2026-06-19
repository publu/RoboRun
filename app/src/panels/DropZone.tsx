import { useRef, useState } from "react";

// BAGEL-style: drop a .mcap and view it. Uploads raw bytes to /api/run/upload,
// which saves it as a replayable run, then opens it.
export function DropZone({ onLoaded }: { onLoaded: (run: string) => void }) {
  const [busy, setBusy] = useState(false);
  const [over, setOver] = useState(false);
  const [err, setErr] = useState("");
  const input = useRef<HTMLInputElement>(null);

  const upload = async (file: File) => {
    setBusy(true);
    setErr("");
    try {
      const r = await fetch(`/api/run/upload?name=${encodeURIComponent(file.name)}`, { method: "POST", body: file });
      const d = await r.json();
      if (d.ok) onLoaded(d.run);
      else setErr(d.error || "upload failed");
    } catch (e) {
      setErr(String(e));
    } finally {
      setBusy(false);
    }
  };

  return (
    <div
      className={"dropzone" + (over ? " over" : "")}
      onDragOver={(e) => {
        e.preventDefault();
        setOver(true);
      }}
      onDragLeave={() => setOver(false)}
      onDrop={(e) => {
        e.preventDefault();
        setOver(false);
        const f = e.dataTransfer.files[0];
        if (f) upload(f);
      }}
      onClick={() => input.current?.click()}
    >
      <input
        ref={input}
        type="file"
        accept=".mcap"
        style={{ display: "none" }}
        onChange={(e) => e.target.files?.[0] && upload(e.target.files[0])}
      />
      {busy ? (
        <span>importing…</span>
      ) : err ? (
        <span style={{ color: "var(--bad)" }}>{err}</span>
      ) : (
        <span>
          <b style={{ color: "var(--accent)" }}>Drop a recording</b> (.mcap) to view it — or click to choose
        </span>
      )}
    </div>
  );
}
