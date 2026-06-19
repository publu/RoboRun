import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
// Dashboards use the system monospace stack (RoboRun DS) — no webfont.
import "./theme.css";
import { App } from "./App";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
  </StrictMode>
);
