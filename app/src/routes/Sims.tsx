import { useEffect, useState } from "react";
import { DataSim } from "../sims/DataSim";
import { ConnectRobot } from "../sims/ConnectRobot";
import { BehaviorEditor } from "../sims/BehaviorEditor";

// A thin launcher, not a catalog. Pick a target, read one line on how to use it,
// and the target's own native UI fills the rest. Code is revealed on demand.
type Target = {
  id: string;
  name: string;
  icon: string;
  sub: string; // compact axis hint (no tag soup)
  real?: boolean;
  howto: string; // the answer to "how do I use this?"
  src?: string; // iframe URL; empty => DataSim component
};

const TARGETS: Target[] = [
  { id: "arena", name: "Arena", icon: "🐕", sub: "3D · single", src: "/sim",
    howto: "Pick a robot and task below, then drive with WASD — or let a behavior file run it autonomously." },
  { id: "fleet", name: "Fleet", icon: "🐝", sub: "3D · warehouse", src: "/fleet-sim",
    howto: "Set robots / floors / speed, press Play, and watch the fleet search the warehouse together." },
  { id: "data", name: "Data Sim", icon: "🦾", sub: "3D · MuJoCo", src: "",
    howto: "Pick a model and press Start, then drive it — its camera frames stream into Live." },
  { id: "real", name: "Real robot", icon: "📡", sub: "your hardware", real: true, src: "",
    howto: "Enter your robot's rosbridge IP and connect — the same behavior file drives the real robot." },
];

export function Sims() {
  const [sel, setSel] = useState("arena");
  const [showCode, setShowCode] = useState(false);
  // mount each sim iframe once on first visit, then just toggle visibility —
  // switching targets no longer reloads the heavy three.js/Rapier page (no flicker)
  const [visited, setVisited] = useState<string[]>(["arena"]);
  useEffect(() => {
    setVisited((v) => (v.includes(sel) ? v : [...v, sel]));
  }, [sel]);
  const active = TARGETS.find((t) => t.id === sel)!;

  return (
    <div style={{ display: "flex", flexDirection: "column", height: "calc(100vh - 92px)", gap: 12 }}>
      {/* pick where the behavior runs */}
      <div className="target-switch" role="tablist" aria-label="run target">
        {TARGETS.map((t) => (
          <button
            key={t.id}
            role="tab"
            aria-selected={t.id === sel}
            className={"target-seg" + (t.id === sel ? " on" : "") + (t.real ? " real" : "")}
            onClick={() => setSel(t.id)}
          >
            <span className="seg-name">
              <span className="seg-ic">{t.icon}</span> {t.name}
            </span>
            <span className="seg-sub">{t.sub}</span>
          </button>
        ))}
      </div>

      {/* the only thing a user needs to read: how to use this one */}
      <div className="howto-strip">
        <span className="howto-text">{active.howto}</span>
        <button className="btn sm" onClick={() => setShowCode((v) => !v)} aria-expanded={showCode}>
          {showCode ? "× close" : "</> how to code it"}
        </button>
      </div>

      {showCode && (
        <div className="code-drawer">
          <BehaviorEditor />
          <div className="code-note">
            Change a number, save — the robot updates <b>while it runs</b>. No build, no restart. Files live in{" "}
            <code>behaviors/</code>; the same file runs on every target above.
          </div>
        </div>
      )}

      {/* one native picker, full-bleed. iframes stay mounted once loaded and just
          show/hide, so switching is instant and doesn't re-flash the heavy page */}
      <div style={{ flex: 1, minHeight: 0, display: "flex" }}>
        {TARGETS.filter((t) => t.src).map((t) =>
          visited.includes(t.id) ? (
            <iframe
              key={t.id}
              title={t.name}
              src={t.src}
              className="scene-stage"
              style={{ flex: 1, display: t.id === sel ? "block" : "none" }}
            />
          ) : null
        )}
        {active.id === "data" && <DataSim />}
        {active.id === "real" && <ConnectRobot />}
      </div>
    </div>
  );
}
