import { useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";
import { useStudio } from "../store";
import { CameraPanel } from "../panels/CameraPanel";
import { EventLogPanel } from "../panels/EventLogPanel";
import { DetectionsPanel } from "../panels/DetectionsPanel";
import { PlotPanel } from "../panels/PlotPanel";

// First-run welcome: when nothing is live (no webcam, sim, or robot), a newcomer
// should see a clear "what is this + one obvious action" — not an empty quadrant
// full of system logs. Once something is running, swap to the live panels.
export function LiveWelcome() {
  const navigate = useNavigate();
  const startTour = useStudio((s) => s.startTour);
  const [seeding, setSeeding] = useState(false);
  const loadSample = async () => {
    setSeeding(true);
    await fetch("/api/demo/seed", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" }).catch(() => {});
    setTimeout(() => navigate("/runs"), 4000); // give the seed a moment, then show it
  };
  const tour = async () => {
    setSeeding(true);
    await fetch("/api/demo/seed", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" }).catch(() => {});
    setTimeout(() => startTour(), 4000); // seed, then walk the golden path
  };
  return (
    <div className="welcome">
      <div className="welcome-card">
        <div className="welcome-kicker">◇ RoboRun Studio</div>
        <h1 className="welcome-h1">Run a robot. Watch it. Search everything it saw.</h1>
        <p className="welcome-sub">
          Write one behavior file and run it in a browser sim or on real hardware. Every run is recorded,
          sealed, and searchable — nothing to install to start.
        </p>
        <div className="welcome-actions">
          <button className="btn welcome-primary" onClick={() => navigate("/sims")}>
            ▶ Start a sim — no install
          </button>
          <button className="btn" onClick={tour} disabled={seeding}>
            {seeding ? "loading demo…" : "Take the 60-sec tour"}
          </button>
        </div>
        <div className="welcome-steps">
          <span><b>1</b> Start a sim</span>
          <span className="welcome-arrow">→</span>
          <span><b>2</b> Press ● Record</span>
          <span className="welcome-arrow">→</span>
          <span><b>3</b> Replay & search it</span>
        </div>
        <button className="welcome-sample" onClick={loadSample} disabled={seeding}>
          {seeding ? "seeding sample runs…" : "Just exploring? Load sample data →"}
        </button>
      </div>
    </div>
  );
}

export function Live() {
  const goLive = useStudio((s) => s.goLive);
  const [active, setActive] = useState<boolean | null>(null);

  useEffect(() => {
    if (useStudio.getState().source.kind !== "live") goLive();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // "Live" = anything actually producing data: a server webcam/MuJoCo sim, a
  // connected robot, OR the browser arena (which only shows up as fresh events,
  // not in /api/dashboard). Without the event check, starting the browser sim
  // would leave Live stuck on the welcome.
  useEffect(() => {
    let alive = true;
    const check = async () => {
      let running = false;
      try {
        const d = await (await fetch("/api/dashboard")).json();
        running = !!(d.webcam?.running || d.sim?.running || d.ros?.connected);
      } catch {
        /* offline */
      }
      const evs = useStudio.getState().source.snapshotEvents(Date.now() / 1000);
      const last = evs[evs.length - 1];
      const freshEvent = !!last && Date.now() / 1000 - last.ts < 12;
      if (alive) setActive(running || freshEvent);
    };
    check();
    const id = setInterval(check, 2500);
    return () => {
      alive = false;
      clearInterval(id);
    };
  }, []);

  // show the welcome until something is genuinely live (avoids the empty quadrant)
  if (active !== true) return <LiveWelcome />;

  return (
    <div className="panel-grid">
      <CameraPanel />
      <EventLogPanel />
      <DetectionsPanel />
      <PlotPanel channel="velocity" />
    </div>
  );
}
