import { useEffect, useRef, useState } from "react";

// Edit a behavior file in-app. Save writes through /api/behaviors/write, which
// the runtime's file watcher hot-reloads into the running robot — so the
// "change a number, save, watch it change" pitch is actually doable here.
type Beh = { name: string; enabled?: boolean; errors?: number; last_error?: string | null };

const esc = (s: string) => s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");

// ponytail: ~15-line regex tokenizer instead of a CodeMirror/Monaco dependency.
// Good enough for short behavior files; if multi-file editing ever lands, swap
// in CodeMirror 6. One pass, escaping the gaps so it's xss-safe.
const TOKEN =
  /(#[^\n]*)|('''[\s\S]*?'''|"""[\s\S]*?"""|'(?:[^'\\]|\\.)*'|"(?:[^"\\]|\\.)*")|(@[A-Za-z_]\w*)|\b(def|class|return|if|elif|else|for|while|import|from|as|with|try|except|finally|raise|in|not|and|or|is|None|True|False|pass|break|continue|lambda|yield|global|nonlocal|assert|del|async|await|self)\b|\b(\d+\.?\d*)\b/g;
function highlight(src: string): string {
  let out = "";
  let last = 0;
  let m: RegExpExecArray | null;
  TOKEN.lastIndex = 0;
  while ((m = TOKEN.exec(src))) {
    out += esc(src.slice(last, m.index));
    const cls = m[1] ? "c" : m[2] ? "s" : m[3] ? "d" : m[4] ? "k" : "n";
    out += `<span class="t-${cls}">${esc(m[0])}</span>`;
    last = m.index + m[0].length;
  }
  // trailing "\n" keeps the highlight layer as tall as the textarea's last line
  return out + esc(src.slice(last)) + "\n";
}

export function BehaviorEditor() {
  const [list, setList] = useState<Beh[]>([]);
  const [name, setName] = useState("");
  const [src, setSrc] = useState("");
  const [status, setStatus] = useState<string>("");
  const [busy, setBusy] = useState(false);
  const taRef = useRef<HTMLTextAreaElement>(null);
  const preRef = useRef<HTMLPreElement>(null);

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

  // keep the highlight layer scrolled in lockstep with the textarea
  const syncScroll = () => {
    const ta = taRef.current, pre = preRef.current;
    if (ta && pre) { pre.scrollTop = ta.scrollTop; pre.scrollLeft = ta.scrollLeft; }
  };

  const onKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if ((e.metaKey || e.ctrlKey) && e.key === "s") { e.preventDefault(); save(); return; }
    // Tab inserts 4 spaces instead of leaving the field — table stakes for a code box
    if (e.key === "Tab") {
      e.preventDefault();
      const ta = e.currentTarget;
      const { selectionStart: a, selectionEnd: b } = ta;
      const next = src.slice(0, a) + "    " + src.slice(b);
      setSrc(next);
      requestAnimationFrame(() => { ta.selectionStart = ta.selectionEnd = a + 4; });
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
      <div className="beh-code-wrap">
        <pre className="beh-hl" aria-hidden="true" ref={preRef}>
          <code dangerouslySetInnerHTML={{ __html: highlight(src) }} />
        </pre>
        <textarea
          ref={taRef}
          className="beh-code"
          spellCheck={false}
          value={src}
          onChange={(e) => setSrc(e.target.value)}
          onScroll={syncScroll}
          onKeyDown={onKeyDown}
        />
      </div>
    </div>
  );
}
