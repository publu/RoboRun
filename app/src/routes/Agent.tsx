import { useEffect, useRef, useState } from "react";
import { Panel } from "../panels/Panel";
import type { EventMsg } from "../source/Source";

// Antioch parity: make the agent loop visible. Shows how to point an MCP client
// at this robot, and a live feed of the agent's tool calls / decisions, so you
// can watch an agent perceive→decide→act through RoboRun.
const MCP_URL = `http://${location.hostname}:${location.port || "8765"}/mcp`;
const COMMANDS = [
  { label: "Claude Code", cmd: `claude mcp add --transport http roborun ${MCP_URL}` },
  { label: "Codex", cmd: `codex mcp add roborun -- npx -y mcp-remote ${MCP_URL}` },
];

// agent loop events worth surfacing (skip plain system noise)
const AGENT_TYPES = new Set(["agent", "mcp_tool", "task", "delegate", "notify"]);

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
  const ref = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const es = new EventSource("/api/events/stream");
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

  useEffect(() => {
    if (ref.current) ref.current.scrollTop = ref.current.scrollHeight;
  }, [events]);

  return (
    <div className="panel-grid" style={{ gridTemplateColumns: "minmax(0, 420px) 1fr" }}>
      <Panel title="Connect an agent">
        <p style={{ fontSize: 13, color: "var(--fg-2)", lineHeight: 1.6, marginTop: 0 }}>
          RoboRun is an <b>MCP server</b> — point any agent at it and it can see, move, and drive this robot
          with the same tools you use. Endpoint:
        </p>
        <div style={{ fontFamily: "var(--mono)", fontSize: 12, color: "var(--accent)", marginBottom: 14 }}>{MCP_URL}</div>
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
  );
}
