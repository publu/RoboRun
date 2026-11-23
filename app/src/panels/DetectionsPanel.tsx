import { useStudio } from "../store";
import { Panel } from "./Panel";

// What the robot sees at the current playhead. sceneAt(t) is source-agnostic.
export function DetectionsPanel() {
  const { source, t } = useStudio();
  const { detections, pose } = source.sceneAt(t);
  return (
    <Panel title="Detections">
      {pose && (
        <div style={{ fontFamily: "var(--mono)", fontSize: 12, color: "var(--fg-dim)", marginBottom: 8 }}>
          pose x={pose.x.toFixed(2)} y={pose.y.toFixed(2)} z={pose.z.toFixed(2)}
        </div>
      )}
      {detections.length ? (
        detections.map((d, i) => (
          <div key={i} style={{ fontFamily: "var(--mono)", fontSize: 12, lineHeight: 1.7 }}>
            <span style={{ color: "var(--accent)" }}>{d.label}</span>
            {d.confidence != null && <span style={{ color: "var(--fg-dim)" }}> {(d.confidence * 100).toFixed(0)}%</span>}
          </div>
        ))
      ) : (
        <div style={{ color: "var(--fg-dim)", fontSize: 12 }}>
          {source.kind === "live" ? "No detections yet — start a sim or connect a robot." : "Nothing detected at this point in the run."}
        </div>
      )}
    </Panel>
  );
}
