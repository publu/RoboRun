import { useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";

// Focused "connect your hardware" flow — NOT the generic multi-backend wizard.
// rosbridge IP → Connect. The same behavior file then drives the real robot.
export function ConnectRobot() {
  const navigate = useNavigate();
  const [host, setHost] = useState("");
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState("");
  const [status, setStatus] = useState<{ connected: boolean; host?: string | null }>({ connected: false });

  const refresh = () =>
    fetch("/api/ros/status").then((r) => r.json()).then(setStatus).catch(() => {});
  useEffect(() => {
    refresh();
    const id = setInterval(refresh, 3000);
    return () => clearInterval(id);
  }, []);

  const connect = async () => {
    setBusy(true);
    setErr("");
    try {
      const r = await fetch("/api/ros/connect", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ host: host.trim() }),
      });
      const d = await r.json();
      if (d.ok) refresh();
      else setErr(d.error || "couldn't reach rosbridge there");
    } catch (e) {
      setErr(String(e));
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="connect-wrap">
      <div className="panel" style={{ maxWidth: 560 }}>
        <div className="panel-head">Connect a robot</div>
        <div className="panel-body" style={{ display: "flex", flexDirection: "column", gap: 14 }}>
          {status.connected ? (
            <div style={{ color: "var(--accent)", fontFamily: "var(--mono)", fontSize: 13 }}>
              ● connected to <b>{status.host}</b> — your behaviors now drive it. Open{" "}
              <a href="/studio/live" style={{ color: "var(--accent)" }}>Live</a> to watch.
              <div style={{ marginTop: 10 }}>
                <button className="btn sm" onClick={() => fetch("/api/ros/disconnect", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" }).then(refresh)}>
                  disconnect
                </button>
              </div>
            </div>
          ) : (
            <>
              <p style={{ fontSize: 14, color: "var(--fg-2)", margin: 0, lineHeight: 1.6 }}>
                Enter your robot's <b>rosbridge</b> address (ROS 1 or 2). The same <code>see / move / ask</code>{" "}
                behavior file drives it — nothing else to install on the robot.
              </p>
              <div style={{ display: "flex", gap: 8 }}>
                <input
                  className="search-input"
                  style={{ flex: 1 }}
                  placeholder="robot IP — e.g. 192.168.1.42"
                  value={host}
                  onChange={(e) => setHost(e.target.value)}
                  onKeyDown={(e) => e.key === "Enter" && host.trim() && connect()}
                />
                <button className="btn welcome-primary" onClick={connect} disabled={busy || !host.trim()}>
                  {busy ? "connecting…" : "Connect"}
                </button>
              </div>
              {err && <div style={{ color: "var(--bad)", fontSize: 12, fontFamily: "var(--mono)" }}>{err}</div>}
              <div style={{ fontSize: 12, color: "var(--fg-dim)", lineHeight: 1.6, borderTop: "1px solid var(--line)", paddingTop: 12 }}>
                No rosbridge yet? On the robot:
                <pre style={{ fontFamily: "var(--mono)", margin: "8px 0 0", color: "var(--fg-2)", whiteSpace: "pre-wrap" }}>
{`ROS 2:  ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ROS 1:  roslaunch rosbridge_server rosbridge_websocket.launch`}
                </pre>
              </div>
              <div style={{ fontSize: 13, color: "var(--fg-dim)" }}>
                No robot handy?{" "}
                <a onClick={() => navigate("/sims")} style={{ color: "var(--accent)", cursor: "pointer" }}>
                  run a sim instead →
                </a>
              </div>
            </>
          )}
        </div>
      </div>
    </div>
  );
}
