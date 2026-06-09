#!/usr/bin/env node
/**
 * npx ros-agent — zero-terminal launch for the ros-agent MCP robot layer.
 *
 * Checks Python ≥ 3.10, installs ros-agent via pip if needed,
 * starts the server, and opens the browser automatically.
 */

import { execSync, spawn } from "child_process";
import { createRequire } from "module";
import { platform } from "os";

const MIN_PYTHON_MINOR = 10;
const PACKAGE = "ros-agent";
const SERVER_MODULE = "roborun.server";
const DEFAULT_PORT = 8765;

function findPython() {
  for (const cmd of ["python3", "python"]) {
    try {
      const ver = execSync(`${cmd} --version 2>&1`, { encoding: "utf8" }).trim();
      const m = ver.match(/Python (\d+)\.(\d+)/);
      if (m && parseInt(m[1]) === 3 && parseInt(m[2]) >= MIN_PYTHON_MINOR) {
        return cmd;
      }
    } catch (_) {}
  }
  return null;
}

function isInstalled(python) {
  try {
    execSync(`${python} -c "import roborun"`, { stdio: "ignore" });
    return true;
  } catch (_) {
    return false;
  }
}

function install(python) {
  console.log("Installing ros-agent...");
  execSync(`${python} -m pip install ros-agent --quiet`, { stdio: "inherit" });
}

function openBrowser(url) {
  const os = platform();
  const cmd =
    os === "darwin" ? "open" : os === "win32" ? "start" : "xdg-open";
  try {
    spawn(cmd, [url], { detached: true, stdio: "ignore" }).unref();
  } catch (_) {}
}

// ── main ─────────────────────────────────────────────────────────────────────

const python = findPython();
if (!python) {
  console.error(
    `\nPython 3.${MIN_PYTHON_MINOR}+ is required. Install it from https://python.org\n`
  );
  process.exit(1);
}

if (!isInstalled(python)) {
  install(python);
}

const port = parseInt(process.env.ROBORUN_PORT || String(DEFAULT_PORT));
const url = `http://127.0.0.1:${port}`;

console.log(`\n  ros-agent — MCP robot control`);
console.log(`  Starting server at ${url}\n`);

const server = spawn(python, ["-m", SERVER_MODULE], {
  stdio: "inherit",
  env: { ...process.env, ROBORUN_PORT: String(port) },
});

// open browser after brief startup delay
setTimeout(() => openBrowser(url), 1500);

server.on("exit", (code) => {
  process.exit(code ?? 0);
});

process.on("SIGINT", () => {
  server.kill("SIGINT");
});
process.on("SIGTERM", () => {
  server.kill("SIGTERM");
});
