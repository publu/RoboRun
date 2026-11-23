import { useEffect, useState } from "react";

// Server-side MuJoCo (Go1 / G1 / drone). Controls hit /api/sim/*; the rendered
// frames already flow into the Live camera panel via /api/camera/frame. So this
// is just the launcher — the data shows up in the shared panels.
type SimState = { running?: boolean; robot?: string; fps?: number };
type RobotDef = { id: string; name?: string; available?: boolean };

export function DataSim() {
  const [robots, setRobots] = useState<RobotDef[]>([]);
  const [state, setState] = useState<SimState>({});
  const [robot, setRobot] = useState("");

  const refresh = () =>
    fetch("/api/sim/state")
      .then((r) => r.json())
      .then((d) => setState(d.ok === false ? {} : d))
      .catch(() => {});

  useEffect(() => {
    fetch("/api/sim/robots")
      .then((r) => r.json())
      .then((d) => {
        const list: RobotDef[] = d.robots || [];
        setRobots(list);
        setRobot(list[0]?.id || "");
      })
      .catch(() => {});
    refresh();
    const iv = setInterval(refresh, 2000);
    return () => clearInterval(iv);
  }, []);

  const post = (path: string, body: object = {}) =>
    fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) }).then(
      refresh
    );

  return (
    <div className="panel">
      <div className="panel-head">Data Sim · MuJoCo</div>
      <div className="panel-body">
        <div style={{ display: "flex", gap: 8, alignItems: "center", marginBottom: 10 }}>
          <select
            className="btn"
            value={robot}
            onChange={(e) => setRobot(e.target.value)}
            aria-label="robot model"
          >
            {robots.map((r) => (
              <option key={r.id} value={r.id} disabled={r.available === false}>
                {r.name || r.id}
              </option>
            ))}
          </select>
          {state.running ? (
            <button className="btn" onClick={() => post("/api/sim/stop")}>
              ■ Stop
            </button>
          ) : (
            <button className="btn" onClick={() => post("/api/sim/start", { robot_id: robot })}>
              ▶ Start
            </button>
          )}
          <button className="btn" onClick={() => post("/api/sim/reset")}>
            ↺ Reset
          </button>
        </div>
        <div style={{ fontFamily: "var(--mono)", fontSize: 12, color: "var(--fg-dim)", display: "flex", gap: 10, alignItems: "center" }}>
          <span>{state.running ? `running ${state.robot} · ${state.fps ?? "?"} fps` : "stopped"}</span>
          {state.running && (
            <a href="/studio/live" style={{ color: "var(--accent)" }}>
              → watch it in Live
            </a>
          )}
        </div>
      </div>
    </div>
  );
}
