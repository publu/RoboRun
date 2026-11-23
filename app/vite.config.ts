import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// Studio is served under /studio during migration (flip to / in Phase 5),
// so it never clobbers the existing roborun/web/*.html pages. The python
// server already owns /api, /mcp, SSE, and MJPEG — dev just proxies to it.
// The shared theme (ui.css/deck.css) is served by the python server too, so
// proxy it rather than copy it: one source of truth.
const API = "http://127.0.0.1:8765";

export default defineConfig({
  base: "/studio/",
  plugins: [react()],
  build: {
    outDir: "../roborun/web/studio",
    emptyOutDir: true, // isolated subdir — safe to wipe
  },
  server: {
    proxy: {
      "/api": API,
      "/mcp": API,
      "/ui.css": API,
      "/deck.css": API,
    },
  },
});
