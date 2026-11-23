import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
// Dashboards use the system monospace stack (RoboRun DS) — no webfont.
import "./theme.css";
import { App } from "./App";
import { resolveBase, installFetchBridge } from "./runtime";

// Find the backend (same-origin local server, or a localhost roborun reached
// from the hosted site), THEN install the /api+/mcp fetch shim and mount. The
// probe is quick and fails fast when nothing's listening.
resolveBase().finally(() => {
  installFetchBridge();
  createRoot(document.getElementById("root")!).render(
    <StrictMode>
      <App />
    </StrictMode>
  );
});
