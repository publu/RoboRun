// Resolve where the roborun backend lives, so the Studio SPA works in three modes:
//   1. served BY the local server (same-origin)          -> base = ""           (localhost:8765)
//   2. hosted (Vercel/Pages) + a local roborun running    -> base = "http://localhost:8765"
//   3. nothing answering                                   -> base = ""          (calls fail -> empty/demo states)
// Override with ?server=http://host:port  or  localStorage.roborun_base.
//
// This is the SPA twin of the legacy web/runtime-base.js bridge: it lets a
// page served from Vercel talk to a roborun running on the user's machine.
// Browsers permit https->http/ws to localhost (a "potentially trustworthy"
// origin), and the server CORS-allowlists *.vercel.app / *.github.io.

const DEFAULT_LOCAL = "http://localhost:8765";

let base = ""; // resolved API origin ("" = same-origin)
let resolved = false;

function override(): string | null {
  const u = new URLSearchParams(location.search).get("server");
  if (u) {
    try {
      localStorage.setItem("roborun_base", u);
    } catch {
      /* private mode */
    }
    return u;
  }
  try {
    return localStorage.getItem("roborun_base");
  } catch {
    return null;
  }
}

async function probe(origin: string): Promise<boolean> {
  try {
    const r = await fetch(origin + "/api/health", { signal: AbortSignal.timeout(1500) });
    return r.ok;
  } catch {
    return false;
  }
}

// Resolve once at startup: an override, then same-origin (served by the local
// server), then the default localhost runtime (a hosted page reaching the
// user's machine). NOTE: call this BEFORE installFetchBridge so the probes hit
// real origins, not the rewritten base.
export async function resolveBase(): Promise<string> {
  if (resolved) return base;
  const ov = override();
  if (ov && (await probe(ov))) {
    base = ov.replace(/\/$/, "");
  } else if (await probe("")) {
    base = "";
  } else if (location.port !== "8765" && (await probe(DEFAULT_LOCAL))) {
    base = DEFAULT_LOCAL;
  } else {
    base = "";
  }
  resolved = true;
  return base;
}

export function apiBase(): string {
  return base;
}

/** Are we talking to a roborun on a different origin than the page (hosted mode)? */
export function isRemote(): boolean {
  return base !== "";
}

/** Absolute URL for an /api or /mcp path against the resolved base. */
export function apiUrl(path: string): string {
  if (/^https?:\/\//.test(path)) return path;
  return base + path;
}

/** Telemetry WS URL derived from the resolved base, not location.hostname, so a
 *  hosted page reaches ws://localhost:8766 instead of ws://<vercel-host>:8766. */
export function wsUrl(port = 8766): string {
  const origin = base || location.origin;
  const u = new URL(origin);
  const scheme = u.protocol === "https:" ? "wss" : "ws";
  return `${scheme}://${u.hostname}:${port}`;
}

/** Absolute MCP endpoint URL (for the copy-paste connect commands). */
export function mcpUrl(): string {
  return (base || location.origin).replace(/\/$/, "") + "/mcp";
}

// Install a fetch shim so the SPA's same-origin "/api" + "/mcp" call sites reach
// the resolved backend without editing all ~40 of them. Mirrors runtime-base.js.
export function installFetchBridge(): void {
  const orig = window.fetch.bind(window);
  window.fetch = (input: RequestInfo | URL, init?: RequestInit) => {
    if (typeof input === "string" && (input.startsWith("/api") || input.startsWith("/mcp"))) {
      return orig(base + input, init);
    }
    return orig(input as RequestInfo | URL, init);
  };
}
