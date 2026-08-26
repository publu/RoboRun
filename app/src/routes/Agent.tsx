import { useEffect, useRef, useState } from "react";
import { Panel } from "../panels/Panel";
import type { EventMsg } from "../source/Source";
import { apiUrl, mcpUrl } from "../runtime";

// Antioch parity: make the agent loop visible. Shows how to point an MCP client
// at this robot, the REAL tools it gets (fetched live from the server, never a
// hand-maintained list that drifts), and a live feed of its tool calls.
// MCP_URL / COMMANDS are computed in the component (after the backend base is
// resolved), so the hosted page shows the user's localhost endpoint, not its own.

// agent loop events worth surfacing (skip plain system noise)
const AGENT_TYPES = new Set(["agent", "mcp_tool", "task", "delegate", "notify"]);

type Tool = { name: string; desc: string };

// Keyword buckets so the capability surface stays organized as the server's
// tool set grows — derived from the name, not a hand-kept membership list.
const GROUPS: { label: string; icon: string; test: RegExp }[] = [
  { label: "Perceive", icon: "◉", test: /^(see|seen|detect|scan_surround|find_object|camera|watch_topic|arena_status)/ },
  { label: "Move & navigate", icon: "▸", test: /(move|navigate|estop|follow_me|patrol|find_object)/ },
  { label: "Behaviors & workflows", icon: "✦", test: /(behavior|workflow|sequence)/ },
  { label: "Record & verify", icon: "▦", test: /(telemetry|record|mcap)/ },
  { label: "ROS & control", icon: "⊞", test: /.*/ },
];
function groupOf(name: string): string {
  return (GROUPS.find((g) => g.test.test(name)) ?? GROUPS[GROUPS.length - 1]).label;
}

function Copy({ cmd }: { cmd: string }) {
  const [done, setDone] = useState(false);
  return (
    <button
      className="btn sm"
      onClick={() => {
        navigator.clipboard?.writeText(cmd).then(() => {
          setDone(true);
          setTimeout(() => setDone(false), 1500);
        });
      }}
    >
      {done ? "copied" : "copy"}
    </button>
  );
}

export function Agent() {
  const [events, setEvents] = useState<EventMsg[]>([]);
  const [connected, setConnected] = useState(false);
  const [tools, setTools] = useState<Tool[] | null>(null);
  const ref = useRef<HTMLDivElement>(null);

  const MCP_URL = mcpUrl();
  const COMMANDS = [
    { label: "Claude Code", cmd: `claude mcp add --transport http roborun ${MCP_URL}` },
    { label: "Codex", cmd: `codex mcp add roborun -- npx -y mcp-remote ${MCP_URL}` },
  ];

  // live agent activity feed
  useEffect(() => {
    const es = new EventSource(apiUrl("/api/events/stream"));
    es.onopen = () => setConnected(true);
    es.onerror = () => setConnected(false);
    es.onmessage = (m) => {
      try {
        const e = JSON.parse(m.data) as EventMsg;
        if (AGENT_TYPES.has(e.type)) setEvents((prev) => [...prev.slice(-199), e]);
      } catch {
        /* ping */
      }
    };
    return () => es.close();
  }, []);

  // the real tool surface, straight from the MCP server (JSON-RPC tools/list)
  useEffect(() => {
    let alive = true;
    fetch("/mcp", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ jsonrpc: "2.0", id: 1, method: "tools/list", params: {} }),
    })
      .then((r) => r.json())
      .then((d) => {
        if (!alive) return;
        const list: Tool[] = (d?.result?.tools || []).map((t: { name: string; description?: string }) => ({
          name: t.name,
          desc: (t.description || "").split("\n")[0],
        }));
        setTools(list);
      })
      .catch(() => alive && setTools([]));
    return () => { alive = false; };
  }, []);

  useEffect(() => {
    if (ref.current) ref.current.scrollTop = ref.current.scrollHeight;
  }, [events]);

  const grouped = GROUPS.map((g) => ({ ...g, items: (tools || []).filter((t) => groupOf(t.name) === g.label) })).filter((g) => g.items.length);

  return (
    <div style={{ display: "flex", flexDirection: "column", gap: 16 }}>
      <div className="panel-grid" style={{ gridTemplateColumns: "minmax(0, 420px) 1fr" }}>
        <Panel title="Connect an agent">
          <p style={{ fontSize: 13, color: "var(--fg-2)", lineHeight: 1.6, marginTop: 0 }}>
            RoboRun is an <b>MCP server</b> — point any agent at it and it can see, move, and drive this robot
            with the same tools you use. Endpoint:
          </p>
          <div style={{ display: "flex", alignItems: "baseline", gap: 8, marginBottom: 14, flexWrap: "wrap" }}>
            <span style={{ fontFamily: "var(--mono)", fontSize: 12, color: "var(--accent)" }}>{MCP_URL}</span>
            {tools && tools.length > 0 && (
              <span style={{ fontSize: 11, color: "var(--fg-dim)" }}>· {tools.length} tools</span>
            )}
          </div>
          {COMMANDS.map((c) => (
            <div key={c.label} style={{ marginBottom: 12 }}>
              <div style={{ fontSize: 11, color: "var(--fg-dim)", textTransform: "uppercase", letterSpacing: ".1em", marginBottom: 4 }}>
                {c.label}
              </div>
              <div style={{ display: "flex", gap: 8, alignItems: "center" }}>
                <code style={{ flex: 1, background: "var(--bg)", border: "1px solid var(--line)", borderRadius: 6, padding: "8px 10px", fontFamily: "var(--mono)", fontSize: 11.5, color: "var(--fg)", overflowX: "auto", whiteSpace: "nowrap" }}>
                  {c.cmd}
                </code>
                <Copy cmd={c.cmd} />
              </div>
            </div>
          ))}
        </Panel>

        <Panel title={`Agent activity ${connected ? "· live" : "· offline"}`}>
          <div ref={ref} style={{ fontFamily: "var(--mono)", fontSize: 12, lineHeight: 1.7 }}>
            {events.map((e, i) => (
              <div key={e.id ?? i}>
                <span style={{ color: "var(--fg-dim)" }}>{new Date(e.ts * 1000).toLocaleTimeString()} </span>
                <span style={{ color: e.type === "mcp_tool" ? "var(--blue)" : "var(--accent)" }}>{e.type}</span>{" "}
                <span>{e.title}</span>
              </div>
            ))}
            {!events.length && (
              <div style={{ color: "var(--fg-dim)" }}>
                No agent activity yet. Connect an agent (left) and ask it to drive the robot — its tool calls
                and decisions stream here.
              </div>
            )}
          </div>
        </Panel>
      </div>

      {/* the real capability surface — what an attached agent actually gets */}
      <Panel title={`What the agent can do${tools ? ` · ${tools.length} tools` : ""}`}>
        {tools === null && <div style={{ color: "var(--fg-dim)", fontSize: 12 }}>reading tools from the MCP server…</div>}
        {tools && !tools.length && (
          <div style={{ color: "var(--fg-dim)", fontSize: 12 }}>
            Couldn't reach the MCP server. Start it with <code style={{ color: "var(--accent)" }}>roborun</code>, then this lists every tool an agent gets.
          </div>
        )}
        {!!grouped.length && (
          <div style={{ display: "grid", gridTemplateColumns: "repeat(auto-fit, minmax(260px, 1fr))", gap: 18 }}>
            {grouped.map((g) => (
              <div key={g.label}>
                <div style={{ fontSize: 10.5, textTransform: "uppercase", letterSpacing: ".12em", color: "var(--fg-dim)", marginBottom: 9 }}>
                  <span style={{ color: "var(--accent)", marginRight: 7 }}>{g.icon}</span>{g.label}
                </div>
                <div style={{ display: "flex", flexWrap: "wrap", gap: 6 }}>
                  {g.items.map((t) => (
                    <span
                      key={t.name}
                      title={t.desc}
                      style={{ fontFamily: "var(--mono)", fontSize: 11.5, color: "var(--fg-2)", background: "var(--panel-2)", border: "1px solid var(--line)", borderRadius: "var(--r-sm)", padding: "3px 9px", cursor: "help" }}
                    >
                      {t.name}
                    </span>
                  ))}
                </div>
              </div>
            ))}
          </div>
        )}
      </Panel>
    </div>
  );
}
