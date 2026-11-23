import { useEffect } from "react";
import { useNavigate } from "react-router-dom";
import { useStudio } from "../store";

// The 60-second golden path: run → record → seal → search → verify. Each step
// navigates to the relevant page and explains the differentiator there, so a
// newcomer (or investor) sees the whole loop close without hunting.
const STEPS = [
  { to: "/runs", title: "1 · A robot ran — and it was recorded",
    body: "We seeded a few runs (a robot perceiving in sim). Every run RoboRun makes is captured into a sealed black box. Pick one on the left." },
  { to: "/runs", title: "2 · Sealed & tamper-evident",
    body: "Open a run: the bar shows ✓ VERIFIED · ANCHORED with its merkle root. Hit ⚡ tamper demo — the seal flips to TAMPER DETECTED, then restores. No other tool can prove this." },
  { to: "/search", title: "3 · Search everything it ever saw",
    body: "Across all runs and robots, offline. Click any result to jump straight into that exact moment in replay." },
  { to: "/agent", title: "4 · Let an AI agent drive it",
    body: "RoboRun is MCP-native — point Claude or Codex at the endpoint and it can see/move/ask through the same tools. That's the whole loop: run → record → seal → search → drive." },
];

export function TourOverlay() {
  const step = useStudio((s) => s.tourStep);
  const setTour = useStudio((s) => s.setTour);
  const navigate = useNavigate();
  const cur = STEPS[step - 1];

  useEffect(() => {
    if (cur) navigate(cur.to);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [step]);

  if (!cur) return null;
  const last = step >= STEPS.length;
  return (
    <div className="tour">
      <div className="tour-card">
        <div className="tour-step">STEP {step} OF {STEPS.length}</div>
        <div className="tour-title">{cur.title}</div>
        <div className="tour-body">{cur.body}</div>
        <div className="tour-actions">
          <button className="btn sm" onClick={() => setTour(0)}>skip</button>
          {step > 1 && <button className="btn sm" onClick={() => setTour(step - 1)}>back</button>}
          <button className="btn sm welcome-primary" onClick={() => setTour(last ? 0 : step + 1)}>
            {last ? "Done" : "Next →"}
          </button>
        </div>
      </div>
    </div>
  );
}
