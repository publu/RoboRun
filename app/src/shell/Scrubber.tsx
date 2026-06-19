import { useStudio } from "../store";

// One global timeline. Live: shows NOW with a follow toggle. Run: scrubs within
// the run's bounds. Setting source elsewhere flips this between the two modes
// without any per-panel code.
export function Scrubber() {
  const { source, t, playing, follow, setT, togglePlay, goLive } = useStudio();
  const b = source.bounds();
  const live = source.kind === "live";

  const fmt = (s: number) => {
    if (!b) return new Date(s * 1000).toLocaleTimeString();
    const rel = Math.max(0, s - b.start);
    return `${rel.toFixed(1)}s`;
  };

  return (
    <div className="scrubber">
      <button className="btn sm" onClick={togglePlay} title={live ? "follow live / pause" : "play / pause"}>
        {(live ? follow : playing) ? "⏸" : "▶"}
      </button>
      {live ? (
        <span className={"live-pill" + (follow ? " on" : "")} title="following live">
          <span className="dot" /> {follow ? "LIVE" : "PAUSED"}
        </span>
      ) : (
        <>
          <input
            type="range"
            className="scrub-range"
            min={b?.start ?? 0}
            max={b?.end ?? 1}
            step={0.05}
            value={t}
            onChange={(e) => setT(parseFloat(e.target.value))}
          />
          <span className="scrub-t">{fmt(t)}</span>
          <button className="btn sm" onClick={goLive} title="back to live">
            ⏏ live
          </button>
        </>
      )}
    </div>
  );
}
