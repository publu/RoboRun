import { useState } from "react";
import { useStudio } from "../store";
import { Panel } from "./Panel";

// Source-agnostic: live shows the latest frame (polled ~5Hz via the playhead),
// replay shows the frame nearest the scrubbed t. Same component, same <img>.
export function CameraPanel() {
  const { source, t } = useStudio();
  const [ok, setOk] = useState(true);
  // quantize to 5Hz so live polling and replay scrubbing both stay sane
  const qt = Math.floor(t * 5) / 5;
  const url = source.frameURL(qt);
  return (
    <Panel title="Camera">
      <div style={{ position: "relative", background: "#000", borderRadius: "var(--r-sm)", minHeight: 160 }}>
        <img
          src={url}
          alt="camera feed"
          style={{ opacity: ok ? 1 : 0 }}
          onLoad={() => setOk(true)}
          onError={() => setOk(false)}
        />
        {!ok && (
          <div
            style={{
              position: "absolute",
              inset: 0,
              display: "grid",
              placeItems: "center",
              textAlign: "center",
              gap: 6,
              color: "var(--fg-dim)",
              fontSize: 13,
            }}
          >
            {source.kind === "live" ? (
              <div>
                <div style={{ fontFamily: "var(--mono)" }}>○ no camera</div>
                <div style={{ marginTop: 6 }}>
                  Start a sim in <a href="/studio/sims" style={{ color: "var(--accent)" }}>Sims</a> or connect a robot.
                </div>
              </div>
            ) : (
              <div style={{ fontFamily: "var(--mono)" }}>○ no camera frame at this point in the run</div>
            )}
          </div>
        )}
      </div>
    </Panel>
  );
}
